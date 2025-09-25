/*
 * ata_hw.c lower half: identify, waits, PIO read/write, flush
 */

#include "ata.h"

static 	int 	ata_is_atapi_by_sig(u8_t, u8_t);
static 	int 	ata_identify_common(int, int, int);
static 	int 	ata_read_signature(int, int, u8_t *, u8_t *);
static 	void 	ata_dump_fdisk(int);
static 	char 	*get_sysid(u8_t);
static 	void 	ata_udelay(int);
static 	int 	ata_probe_unit(int, int);
static 	char 	*atapi_class_name(u8_t, int);
static 	int 	atapi_inquiry(int, int, u8_t *, u8_t *, char *, char *);

int 	atapi_read_capacity(int, int, u32_t *, u32_t *);
int 	atapi_read10(int, int, u32_t, u16_t, void *);
int 	ata_select(int, int, u32_t);

/*********** ATA Routines *************/

int
ata_xfer_uio(int is_write, dev_t dev, struct uio *uiop)
{
	int 	unit = ATA_UNIT(dev);
	ata_unit_t *u = &ata_unit[unit];
	ata_chan_t *ch = &u->chan;
	int 	ctrl = u->ctrl, driv = u->driv;
	u32_t 	base_lba, region_len;
	u32_t 	lba, left_sectors;
	u32_t 	nsec;
	size_t 	nbytes;
	caddr_t bounce;
	ata_req_t *r;
	int 	s, rc;

	if (!ata_check_unit(unit)) return ENODEV;

	ata_region_from_dev(dev, &base_lba, &region_len);
	if ((uiop->uio_resid == 0) || (uiop->uio_resid & 0x1FF))
		return (u32_t)(uiop->uio_resid == 0) ? 0 : EINVAL;

	lba          = base_lba + (u32_t)(uiop->uio_offset >> 9);
	left_sectors = (u32_t)(uiop->uio_resid >> 9);

	if (!ch->xfer_buf || ch->xfer_bufsz == 0) return EIO;
	bounce = (caddr_t)ch->xfer_buf;

	ch->use_intr = ata_should_use_intr(u,u->is_atapi,(u32_t)uiop->uio_resid);

	/* Polled */
	if (!ch->use_intr) {
		rc = 0;
		while (left_sectors > 0) {
			nsec = left_sectors > 256U ? 256U : left_sectors;
			nbytes = (size_t)nsec << 9;
			if (nbytes > ch->xfer_bufsz) return EIO;
			if (nbytes > uiop->uio_resid) {
				nbytes = uiop->uio_resid;
				nsec = (u32_t)(nbytes >> 9);
				if (nsec == 0) break;
			}
			if (is_write) {
				ASSERT_NOT_ISR();
				rc = uiomove(bounce, nbytes, UIO_WRITE, uiop);
				if (rc) break;
				rc = ata_pio_write(ctrl,driv,lba,nsec,bounce);
				if (rc) break;
			} else {
				rc = ata_pio_read(ctrl,driv,lba,nsec,bounce);
				if (rc) break;
				ASSERT_NOT_ISR();
				rc = uiomove(bounce, nbytes, UIO_READ, uiop);
				if (rc) break;
			}
			lba += nsec; 
			left_sectors -= nsec;
		}
		return rc;
	}

	/* Queued/IRQ */
	rc = 0;
	while (left_sectors > 0) {
		int 	need_kick = 0;
		u32_t 	max_buf_secs;

		if (uiop->uio_resid == 0) break;

		nsec = left_sectors > 256U ? 256U : left_sectors;
		max_buf_secs = (u32_t)(ch->xfer_bufsz >> 9);
		if (nsec > max_buf_secs) nsec = max_buf_secs;

		nbytes = (size_t)nsec << 9;
		if (nbytes > uiop->uio_resid) {
			nbytes = uiop->uio_resid;
			nsec   = (u32_t)(nbytes >> 9);
			if (nsec == 0) break;
		}

		if (is_write) {
			ASSERT_NOT_ISR();
			rc = uiomove(bounce, nbytes, UIO_WRITE, uiop);
			if (rc) break;
		}

		s = splbio();
		ch->no_autokick = 1;
		ch->sync_done   = 0;
		splx(s);

		r = (ata_req_t *)alloc_request(lba,nsec,is_write,ch->xfer_buf);
		if (!r) { rc = ENOMEM; break; }

		s = splbio();
		ata_q_put(u, r);
		if (!ch->busy && !ch->cur) need_kick = 1;
		splx(s);
		if (need_kick) ata_kick(u);

		s = splbio();
		while (!ch->sync_done) sleep((caddr_t)&ch->sync_done, PRIBIO);
		splx(s);

		if (!is_write) {
			ASSERT_NOT_ISR();
			rc = uiomove(ch->xfer_buf, nbytes, UIO_READ, uiop);
			if (rc) break;
		}

		s = splbio();
		ch->no_autokick = 0;
		need_kick = (!ch->busy && !ch->cur && ch->q_head != NULL);
		splx(s);
		if (need_kick) ata_kick(u);

		lba += nsec; left_sectors -= nsec;
	}
	return rc;
}

int
ata_strategy(struct buf *bp)
{
	dev_t	dev=bp->b_edev;
	int	unit   = ATA_UNIT(dev);
	ata_unit_t *u  = &ata_unit[unit];
	ata_chan_t *ch = &u->chan;
	ata_req_t *r;
	u32_t   base, len, lba, left;
	char    *addr;
	int     s, need_kick=0, is_write;

	/* Unit present? */
	if (!ata_check_unit(unit)) {
		bp->b_flags |= B_ERROR; 
		bp->b_error = ENODEV; 
		biodone(bp); 
		return 0;
	}

	/* Map region (512-byte sectors) */
	ata_region_from_dev(dev, &base, &len);
	if (len == 0) {
		bp->b_flags |= B_ERROR; 
		bp->b_error = ENXIO; 
		biodone(bp); 
		return 0;
	}

	/* Request geometry from bp */
	lba      = base + (u32_t)bp->b_blkno;        /* 512B sectors */
	left     = (u32_t)(bp->b_bcount >> 9);       /* 512B sectors */
	addr     = (char *)bp->b_un.b_addr;
	is_write = ((bp->b_flags & B_READ) == 0);

	/* Decide policy (IRQ vs polled) */
	ch->use_intr = ata_should_use_intr(u,0,(u32_t)bp->b_bcount);

	ATADEBUG("ata_strategy: build req lba=%lu nsec=%lu bytes=%u is_write=%d buf=%lx\n",
		lba,left,bp->b_bcount,is_write,addr);

	if (ch->use_intr) {
		/* IRQ path: allocate request, enqueue, sleep for completion */

		r = (ata_req_t *)alloc_request(lba,left,is_write,addr);
		if (!r) {
			bp->b_flags |= B_ERROR; 
			bp->b_error = ENOMEM; 
			biodone(bp); 
			return 0;
		}
		r->bp = bp;

		s = splbio();
		ata_q_put(u, r);
		if (!ch->busy && !ch->cur) need_kick=1;
		splx(s);
		if (need_kick) ata_kick(u);

		s=splbio();
		while (!ch->sync_done)
			sleep((caddr_t)&ch->sync_done, PRIBIO);
		splx(s);

		return 0;
	}

	/* Polled PIO fallback: split into <=256-sector commands */
	while (left > 0) {
		u32_t 	chunk = (left > 256U) ? 256U : left;

		int   rc    = is_write
			? ata_pio_write(u->ctrl, u->driv, lba, chunk, addr)
			: ata_pio_read (u->ctrl, u->driv, lba, chunk, addr);
		if (rc) {
			bp->b_flags |= B_ERROR; 
			bp->b_error = EIO; biodone(bp); 
			return 0;
		}
		lba  += chunk;
		left -= chunk;
		addr += (chunk << 9);                /* bytes */
	}

	bp->b_resid = 0;
	biodone(bp);
	return 0;
}

/* Send an ATAPI PACKET and the 12-byte CDB.
 * Returns 0 on success (CDB accepted), or EIO/ETIMEDOUT on failure.
 */
int
ata_send_packet(int ctrl, int driv, const u_char *cdb, int cdb_len)
{
	u_char st, errreg;
	u_short words_cdb[6];
	int i, spins;

	/* Select device + 400ns (4 ALTSTATUS reads) */
	outb(ATA_DRVHD_O(ctrl), (u_char)(0xA0 | (driv ? 0x10 : 0x00)));
	(void)inb(ATA_ALTSTATUS_O(ctrl));
	(void)inb(ATA_ALTSTATUS_O(ctrl));
	(void)inb(ATA_ALTSTATUS_O(ctrl));
	(void)inb(ATA_ALTSTATUS_O(ctrl));

	/* Polled I/O: disable interrupts */
	outb(ATA_DEVCTRL_O(ctrl), ATA_CTL_NIEN);

	/* Clear side registers some chips expect 0 */
	outb(ATA_FEAT_O(ctrl),    0x00);
	outb(ATA_SECTCNT_O(ctrl), 0x00);
	outb(ATA_SECTNUM_O(ctrl), 0x00);

	/* Advertise 64KiB burst size (0x0000 is widely compatible) */
	outb(ATA_CYLLOW_O(ctrl),  0x00);  /* Byte Count Low  */
	outb(ATA_CYLHIGH_O(ctrl), 0x00);  /* Byte Count High */

	/* Issue PACKET */
	outb(ATA_CMD_O(ctrl), 0xA0);

	/* Wait for BSY=0 & DRQ=1 (device ready to accept CDB) */
	spins = 200000; /* ~200ms with 1us polls */
	while (spins--) {
		st = inb(ATA_ALTSTATUS_O(ctrl));
		if (st & ATA_ST_ERR) {
			(void)inb(ATA_STATUS_O(ctrl));
			errreg = inb(ATA_FEAT_O(ctrl));
			(void)errreg; /* quieten unused if not logged */
			return EIO;
		}
		if (!(st & ATA_ST_BSY) && (st & ATA_ST_DRQ))
			break;
		ata_udelay(1);
	}
	if (spins <= 0) {
		(void)inb(ATA_STATUS_O(ctrl));
		return ETIMEDOUT;
	}

	/* Write exactly 12 bytes of CDB (pad to 6 words) */
	for (i = 0; i < 6; ++i) words_cdb[i] = 0;
	for (i = 0; i < cdb_len && i < 12; ++i)
		((u_char *)words_cdb)[i] = cdb[i];
	for (i = 0; i < 6; ++i)
		outw(ATA_DATA_O(ctrl), words_cdb[i]);

	return 0;
}

/* Tiny delay helper (PIO polling); tune if needed */
static void
ata_udelay(int loops)
{
	/* Port 0x80 is POST codes, which consumes some cycles hence delay */
	while (loops--) 
		(void)inb(0x80); /* standard tiny delay port */
}

int
ata_check_unit(int unit)
{
	ata_unit_t *u = &ata_unit[unit];
	return (unit >= 0 && unit < (ATA_MAX_UNITS) && u->present);
}

int
ata_wait(int ctrl, u32_t must_set, u32_t must_clear, long usec)
{
	u8_t st;
	long i;

	for (i = 0; i < usec; ++i) {
		st = inb(ATA_STATUS_O(ctrl));
		if (((st & must_set) == must_set) && ((st & must_clear) == 0))
			return 0;
		ata_udelay(1);
	}
	return EIO;
}

void
ata_wait_ready(ata_unit_t *u)
{
	int	c = u->ctrl;
	u32_t	guard = 100000;

	while(guard--) {
		u8_t as = inb(ATA_ALTSTATUS_O(c));
		if (!(as & ATA_ST_BSY)) {
			if (as & ATA_ST_DRDY) break;
		}
	}
}

void
ata_soft_reset(int ctrl)
{
	outb(ATA_DEVCTRL_O(ctrl), ATA_CTL_SRST | ATA_CTL_NIEN);
	drv_usecwait(5);
	outb(ATA_DEVCTRL_O(ctrl), ATA_CTL_NIEN);
	drv_usecwait(5);
}

int
ata_sel(int ctrl, int driv)
{
	u8_t dh = ATA_DHFIXED | (driv ? ATA_DHDRV : 0);
	outb(ATA_DRVHD_O(ctrl), dh);
	drv_usecwait(5);
	return 0;
}

int
ata_select(int ctrl, int driv, u32_t lba)
{
	outb(ATA_SECTCNT_O(ctrl), 0);   /* count filled later */
	outb(ATA_SECTNUM_O(ctrl), (u8_t)(lba & 0xFF));
	outb(ATA_CYLLOW_O(ctrl),  (u8_t)((lba >> 8) & 0xFF));
	outb(ATA_CYLHIGH_O(ctrl), (u8_t)((lba >> 16) & 0xFF));
	outb(ATA_DRVHD_O(ctrl), ATA_DH(driv, 1, (lba >> 24) & 0x0F));
	return 0;
}

static char *
get_sysid(u8_t systid)
{
	switch(systid) {
	case EMPTY:		return "Empty";
	case UNUSED:		return "Unused";
	case FAT12:		return "FAT12";
	case PCIXOS:		return "PCIXOS";
	case FAT16:		return "FAT16";
	case EXTDOS:		return "EXTDOS";
	case NTFS:		return "NTFS";
	case DOSDATA:		return "DOSDATA";
	case OTHEROS:		return "OTHEROS";
	case SVR4:		return "Unix SVR4";
	case LINUXSWAP:		return "Linux Swap";
	case LINUXNATIVE:	return "Linux";
	case BSD386:		return "BSD386";
	case OPENBSD:		return "OpenBSD";
	case NETBSD:		return "NetBSD";
	case SOLARIS:		return "Solaris";
	}
	return "??";
}

static void
ata_dump_fdisk(int unit)
{
	int	fdisk, s;
	ata_unit_t *u = &ata_unit[unit];

	if (!u->present) return;

	if (u->is_atapi) return;

	for(fdisk=0; fdisk<4; fdisk++) {
		struct ata_fdisk *fp=&u->fd[fdisk];
		u32_t 	base  = fp->base_lba;
		u32_t 	nsec  = fp->nsectors;
		int 	valid = fp->vtoc_valid;
		char	*systid;
		int	any;

		for(s=0; s<ATA_NPART; s++) {
			if (fp->slice[s].p_size != 0) {
				any=1;
				break;
			}
		}
		if (!any && base == 0 && nsec == 0 && !valid) continue;

		systid = get_sysid(u->fd[fdisk].systid);
		printf("  fdisk=%d: Type=%-10s base_lba=%lu size=%lu %s\n",
			fdisk, systid,
			(ulong_t)base, (ulong_t)nsec,
			valid ? "vtoc=VALID" : "");

		for(s=0;s<ATA_NPART; s++) {
			u32_t ps = fp->slice[s].p_start;
			u32_t sz = fp->slice[s].p_size;
			if (sz == 0) continue;

			printf("   %cs%02d: start=%8lu size=%8lu\n",
				(s==ATA_WHOLE_FDISK_SLICE)?'*':' ',
				s,(ulong_t)ps,(ulong_t)sz);
		}
	}
}

int
ata_identify(int ctrl, int driv, int unit)
{
	u16_t id[256];
	int 	i;
	ata_unit_t *u = &ata_unit[unit];

	if (!ata_ctrl[ctrl].present) return ENODEV;

	outb(ATA_DEVCTRL_O(ctrl), 0); /* enable interrupts */
	outb(ATA_DRVHD_O(ctrl), ATA_DH(driv, 0, 0));
	ata_udelay(2);

	outb(ATA_CMD_O(ctrl), ATA_CMD_IDENTIFY);
	if (ata_wait(ctrl, ATA_ST_DRQ, ATA_ST_BSY, 500000) != 0) {
		/* no ATA device mark not present */
		u->present = 0;
		return ENODEV;
	}

	insw(ATA_DATA_O(ctrl),id,256);

	/* Extract model string (word-swapped ASCII) */
	for (i = 0; i < 40; i += 2) {
		u->model[i]   = ((char *)id)[54 + i + 1];
		u->model[i+1] = ((char *)id)[54 + i + 0];
	}
	u->model[40] = 0;

	/* LBA28 capacity in words 60 61 */
	u->nsectors = ((u32_t)id[61] << 16) | (u32_t)id[60];
	u->lba_ok   = (id[49] & (1<<9)) ? 1 : 0;
	u->present  = 1;
	u->read_only = 0;
	return 0;
}

/* --------- PIO WRITE: split into <=256 sector commands --------- */
int
ata_pio_write(int ctrl, int driv, u32_t lba, int nsec, caddr_t buf)
{
	int 	rc, todo, s, i;
	u16_t 	*p;
	u8_t 	cnt8;

	if (nsec <= 0) return EINVAL;
	rc = 0;
	p  = (u16_t *)buf;

	while (nsec > 0) {
		todo = (nsec > 256) ? 256 : nsec;
		/* 0 in SECTCNT means 256 sectors */
		cnt8 = (u8_t)((todo == 256) ? 0 : todo);

		if (ata_wait(ctrl, 0, ATA_ST_BSY, 500000) != 0) { 
			rc = EIO; 
			break; 
		}
		ata_select(ctrl, driv, lba);

		outb(ATA_SECTCNT_O(ctrl), cnt8);
		outb(ATA_CMD_O(ctrl), ATA_CMD_WRITE_SEC);

		for (s = 0; s < todo; ++s) {
			if (ata_wait(ctrl,ATA_ST_DRQ,ATA_ST_BSY,500000) != 0) {
				rc = EIO; 
				goto done; 
			}
			outsw(ATA_DATA_O(ctrl),p,256); p+= 256;
		}

		/* settle and advance for next burst */
		if (ata_wait(ctrl,ATA_ST_DRDY, ATA_ST_BSY|ATA_ST_DRQ,
			     500000) != 0) { 
			rc = EIO; 
			break; 
		}
		lba  += (u32_t)todo;
		nsec -= todo;
	}
done:
	return rc;
}

/* --------- PIO READ: split into <=256 sector commands --------- */
int
ata_pio_read(int ctrl, int driv, u32_t lba, int nsec, caddr_t buf)
{
	int	rc, todo, s, i;
	u16_t	*p;
	u8_t 	cnt8;

	ATADEBUG("ata_pio_read(ctrl=%d driv=%d lba=%d nsec=%d\n",
		ctrl,driv,lba,nsec);

	if (nsec <= 0) return EINVAL;
	rc = 0;
	p  = (u16_t *)buf;

	while (nsec > 0) {
		todo = (nsec > 256) ? 256 : nsec;
		cnt8 = (u8_t)((todo == 256) ? 0 : todo);

		if (ata_wait(ctrl, 0, ATA_ST_BSY, 500000) != 0) { 
			rc = EIO; 
			break; 
		}
		ata_select(ctrl, driv, lba);

		outb(ATA_SECTCNT_O(ctrl), cnt8);
		outb(ATA_CMD_O(ctrl), ATA_CMD_READ_SEC);

		for (s = 0; s < todo; ++s) {
			if (ata_wait(ctrl, ATA_ST_DRQ, 
					   ATA_ST_BSY, 500000) != 0) { 
				rc = EIO; 	
				goto done; 
			}
			insw(ATA_DATA_O(ctrl),p,256); p+= 256;
		}

        	if (ata_wait(ctrl, ATA_ST_DRDY, 
				   ATA_ST_BSY | ATA_ST_DRQ, 500000) != 0) { 
			rc = EIO; 
			break; 
		}
        	lba  += (u32_t)todo;
        	nsec -= todo;
	}
done:
	return rc;
}

int
ata_flush_cache(int ctrl, int driv)
{
	if (ata_wait(ctrl, 0, ATA_ST_BSY, 500000) != 0) return EIO;
	outb(ATA_DRVHD_O(ctrl), ATA_DH(driv, 0, 0));
	outb(ATA_CMD_O(ctrl), ATA_CMD_FLUSH_CACHE);
	if (ata_wait(ctrl, ATA_ST_DRDY, ATA_ST_BSY|ATA_ST_DRQ, 2000000) != 0)
		return EIO;
	return 0;
}

int
ata_read_fdisk(int unit)
{
	int	ctrl = ATA_CTRL_FROM_UNIT(unit),
		driv = ATA_DRIVE_FROM_UNIT(unit);
	ata_unit_t *u = &ata_unit[unit];
	struct ata_fdisk *fp;
	int	i, fdisk, rc;
	struct mboot *mboot;
	struct ipart *ipart;

	if (u->is_atapi) return 0;

	mboot = (struct mboot *)kmem_alloc(DEV_BSIZE,KM_SLEEP);
	if (!mboot) return ENOMEM;

	rc = ata_pio_read(ctrl,driv,0,1,(caddr_t)mboot);;
	if (rc) {
		kmem_free((caddr_t)mboot,DEV_BSIZE);
		return EIO;
	}
	
	u->fdisk_valid = 0;
	for(fdisk=0; fdisk<4; fdisk++) {
		fp=&u->fd[fdisk];

		fp->base_lba   = 0;
		fp->nsectors   = 0;
		fp->systid     = 0;
		fp->vtoc_valid = 0;
	}

	if (mboot->signature != MBB_MAGIC) {
		kmem_free((caddr_t)mboot, DEV_BSIZE);
		return 0;
	}

	u->fdisk_valid = 1;	
	ipart = (struct ipart *)&mboot->parts[0];
	for(fdisk=0; fdisk<4; fdisk++,ipart++) {
		fp=&u->fd[fdisk];

		fp->base_lba = ipart->relsect;
		fp->nsectors = ipart->numsect;
		fp->systid   = (int)ipart->systid;

		fp->slice[ATA_WHOLE_FDISK_SLICE].p_start = 0;
		fp->slice[ATA_WHOLE_FDISK_SLICE].p_size  = u->fd->nsectors;

		if (fp->nsectors > 0 && fp->systid == UNIXOS) 
			fp->vtoc_valid = ata_read_vtoc(unit,fdisk);
	}
	kmem_free((caddr_t)mboot,DEV_BSIZE);
	return 0;
}

/*
 * Read SVR4 pdinfo/vtoc from the UNIX fdisk partition.
 * We always use absolute LBAs here.  No region mapping. 
 */
int 
ata_read_vtoc(int unit,int fdisk)
{
	u32_t 	base, lba, off;
	caddr_t k = 0;
	struct pdinfo *pd;
	struct vtoc *v;
	ata_unit_t *u = &ata_unit[unit];
	struct ata_fdisk *fp = &u->fd[fdisk];
	int 	s;

	if (u->is_atapi) return 0;

	/* Only attempt on UNIX partitions with a size. */
	if (fp->systid != UNIXOS || fp->nsectors == 0) return 0;

	base = fp->base_lba;

	/* pdinfo + vtoc live in the same sector at (unix_base + VTOC_SEC). */
	lba = base + (u32_t)VTOC_SEC;
	if (!(k = kmem_alloc(DEV_BSIZE, KM_SLEEP))) return 0;

	if (ata_pio_read(ATA_CTRL_FROM_UNIT(unit), ATA_DRIVE_FROM_UNIT(unit),
                     lba, 1, k) != 0) {
		kmem_free(k, DEV_BSIZE);
		return 0;
	}

	pd = (struct pdinfo *)k;
	if (pd->sanity != VALID_PD || pd->version != 1) {
		kmem_free(k, DEV_BSIZE);
		return 0;
	}

	/*
	 * vtoc is embedded within this same 512B sector at offset 
	 * dp.vtoc_ptr*dp_secsiz 
	 */
        off = (u32_t)pd->vtoc_ptr % DEV_BSIZE;
        if (off + sizeof(struct vtoc) > DEV_BSIZE) {
            kmem_free(k, DEV_BSIZE);
            return 0;
        }
        v = (struct vtoc *)((char*)k + off);

	if (v->v_sanity != VTOC_SANE || v->v_version != V_VERSION) {
		kmem_free(k, DEV_BSIZE);
		return 0;
	}

	/* Copy slices from vtoc into our driver table. */
	for (s = 0; s < ATA_NPART && s < V_NUMPAR; ++s) {
		fp->slice[s].p_start = v->v_part[s].p_start;
		fp->slice[s].p_size  = v->v_part[s].p_size;
	}

	/* Whole-fdisk pseudo-slice: full partition range. */
	fp->slice[ATA_WHOLE_FDISK_SLICE].p_start = 0;
	fp->slice[ATA_WHOLE_FDISK_SLICE].p_size  = fp->nsectors;

	fp->vtoc_valid = 1;

	kmem_free(k, DEV_BSIZE);
	return 1;
}

void
ata_copy_model(u16_t *id, char *dst)
{
	int 	i;
	char 	*p = dst;
	for (i = 27; i < 27 + 20; i++) {
		u16_t w = id[i];
		*p++ = (char)(w >> 8);
		*p++ = (char)(w & 0xFF);
	}
	*p = 0;
	for (i = (int)strlen(dst)-1; i >= 0; --i) {
		if (dst[i] == ' ') dst[i] = 0; 
		else break;
	}
}

/* --- Low-level: read device signature after a Select+SRST --- */
static int
ata_read_signature(int ctrl, int driv, u8_t *cl, u8_t *ch)
{
	if (ata_sel(ctrl, driv) != 0)
		return EIO;

	/* Issue a device reset on this controller, clear BSY */
	outb(ATA_DEVCTRL_O(ctrl), ATA_CTL_SRST | ATA_CTL_NIEN);
	drv_usecwait(5);
	outb(ATA_DEVCTRL_O(ctrl), ATA_CTL_NIEN);  /* deassert SRST */

	/* Wait for not-BSY */
	if (ata_wait(ctrl, 0, ATA_ST_BSY, 500000L) != 0)
		return EIO;

	/* After SRST you must reselect the target device and wait 400ns */
	if (ata_sel(ctrl, driv) != 0)
		return EIO;

	/* (void)inb(ATA_ALTSTATUS_O(ctrl)); 
	(void)inb(ATA_ALTSTATUS_O(ctrl));
	(void)inb(ATA_ALTSTATUS_O(ctrl));
	(void)inb(ATA_ALTSTATUS_O(ctrl));*/
	drv_usecwait(1);

	*cl = inb(ATA_CYLLOW_O(ctrl));
	*ch = inb(ATA_CYLHIGH_O(ctrl));
	return 0;
}

/* --- Decide if this drive is ATAPI by signature --- */
static int
ata_is_atapi_by_sig(u8_t lc, u8_t hc)
{
	return (lc == 0x14 && hc == 0xEB) ? 1 : 0;
}

/* --- Common identify: EC for ATA, A1 for ATAPI --- */
static int
ata_identify_common(int ctrl, int driv, int is_pkt)
{
	u16_t 	id[256];
	int 	i, 
		p = 0,
		unit = ATA_UNIT_FROM(ctrl, driv);
	ata_unit_t *u = &ata_unit[unit];

	bzero((caddr_t)id, sizeof(id));

	if (ata_sel(ctrl, driv) != 0) return EIO;

	outb(ATA_CMD_O(ctrl), is_pkt ? ATAPI_CMD_IDENTIFY_PKT 
				     : ATA_CMD_IDENTIFY);

	/* Wait: not BSY; for identify we require DRQ=1 and ERR=0 */
	if (ata_wait(ctrl, ATA_ST_DRQ, ATA_ST_ERR, 200000L) != 0)
		return EIO;

	insw(ATA_DATA_O(ctrl),id,256);

	/* Basic bookkeeping */
	u->present  = 1;
	u->is_atapi = is_pkt;

	u->read_only = is_pkt ? 1 : 0;
	/* Model string (words 27..46, bytes swapped) */
	for (i = 27; i <= 46; i++) {
		u16_t w = id[i];
		char a = (char)((w >> 8) & 0xFF);
		char b = (char)(w & 0xFF);

		if (p < (int)sizeof(u->model) - 1) 
			u->model[p++] = a;
		if (p < (int)sizeof(u->model) - 1) 
			u->model[p++] = b;
	}
	while (p > 0 && u->model[p-1] == ' ') p--;
	u->model[p] = 0;

	if (!is_pkt) {
		/* ATA disk capacity */
		u16_t caps = id[49];
		u->lba_ok = !!(caps & (1<<9));  /* LBA bit */

		if (u->lba_ok) {
			/* 28-bit total sectors */
			u->nsectors = ((u32_t)id[61] << 16) | (u32_t)id[60];
		} else {
			u32_t cyl   = id[1];
			u32_t heads = id[3];
			u32_t spt   = id[6];
			u->nsectors = cyl * heads * spt;
		}

        	/* VTOC base is cylinder 0, head 1, sector 1 => LBA 63 */
	} else {
		/* ATAPI device: capacity not meaningful here */
		u->nsectors = 0;
		u->lba_ok   = 0;
	}
	return 0;
}

/* --- Probe both drives on a controller using signature, then identify --- */
void
ata_attach(int ctrl)
{
	int 	driv;
	int	rc=-1;

	if (!ata_ctrl[ctrl].present) return;

	for (driv = 0; driv <= 1; driv++) {
		int unit = ATA_UNIT_FROM(ctrl, driv);
		ata_unit_t *u = &ata_unit[unit];

		/*** Initialise unit to defaults ***/
        	u->atapi_pdt = 0;
        	u->atapi_rmb = 0;
        	u->vendor[0] = 0;
        	u->product[0] = 0;

		u->present   = 0;
		u->is_atapi  = 0;
		u->nsectors  = 0;
		u->lba_ok    = 0;
		u->model[0]  = 0;
		
		/* Clear ATAPI classification flags to avoid stale data */
		u->atapi_pdt = 0;
		u->atapi_rmb = 0;
		u->vendor[0] = 0;
		u->product[0] = 0;

		u->ctrl=ctrl;
		u->driv=driv;
		ata_probe_unit(ctrl,driv);
		if (u->present) rc++;
	}
	if (rc != -1) {
		int irq=ata_ctrl[ctrl].irq;
		/* IRQ_Register(irq, &ataintr, SPL6, INTR_TRIGGER_EDGE);*/
	}
}

static int
ata_probe_unit(int ctrl, int driv)
{
	u8_t 	lc, hc, st;
	int 	unit = ATA_UNIT_FROM(ctrl, driv);
	ata_unit_t *u = &ata_unit[unit];
	ata_chan_t *ch = &u->chan;
	int 	is_pkt;
	char 	*klass;

	if (!ata_ctrl[ctrl].present) return ENXIO;

	if (ata_read_signature(ctrl,driv,&lc,&hc) != 0)
		return ENXIO;

	is_pkt = ata_is_atapi_by_sig(lc, hc);

	if (ata_identify_common(ctrl, driv, is_pkt) != 0)
		return ENXIO;

	u->present = 1;
	ch->sync_done = 1;
	ch->tmo_id    = 0;
	ch->tmo_ticks = drv_usectohz(2000000); /* 2s is sane for PIO */
	if (is_pkt) {
		u32_t blocks=0, blksz=0;

		/* Populate inquiry fields into ata_unit[] */
		(void)atapi_inquiry(ctrl, driv, NULL, NULL, NULL, NULL);
		klass = atapi_class_name(u->atapi_pdt, u->atapi_rmb);
		if ((atapi_read_capacity(ctrl,driv,&blocks,&blksz) == 0) &&
			blocks && blksz) {
			/*printf("blocks=%ld, blksz=%ld\n",blocks,blksz);*/
			;
		}

		printf("ata: c%dd%d: ATAPI %s, model=\"%s\"\n",
			ctrl, driv, klass, u->model);
		u->is_atapi=1;
		u->lbsize = (blksz ? blksz : 2048);
		if (u->lbsize < 512) u->lbsize=512;

	} else { /* ATA disk branch */
		char 	*lba28 = u->lba_ok ? "LBA28" : "";
		unsigned long nsec = u->nsectors;

		ulong_t mib = nsec >> 11; 
		ulong_t gib_i = nsec / 2097152UL;
		ulong_t gib_tenths = ((nsec % 2097152UL) * 10UL) / 2097152UL;

		printf("ata: c%dd%d: ATA disk, model=\"%s\" %s, %lu sectors (%lu.%u GiB)\n",
			ctrl, driv, u->model,
			lba28, nsec, gib_i,gib_tenths);

		u->lbsize=512;
		ata_read_fdisk(unit);
	}
	u->lbshift=(u8_t)((u->lbsize >> 9) 
			? (u->lbsize==512 ?0:(u->lbsize==1024 ? 1 : 2)) : 0);
	/*ata_dump_fdisk(unit);*/
	return 0;
}

/*********** ATAPI Routines *************/

int
atapi_read_capacity(int ctrl, int driv, u32_t *out_blocks, u32_t *out_blksz)
{
	int unit = ATA_UNIT_FROM(ctrl, driv);
	ata_unit_t *u = &ata_unit[unit];
	u16_t xfer_len = 8;   /* READ CAPACITY(10) returns 8 bytes */
	u8_t pkt[12] = {0};
	u8_t buf[16];
	int i;
	u32_t	last_lba, blksz, blocks;

	/* 10-byte CDB inside 12-byte packet: opcode only, rest zero */
	pkt[0] = CDB_READ_CAPACITY; /* READ CAPACITY(10) */

	if (ata_sel(ctrl, driv) != 0) return EIO;

	/* Program expected byte count */
	outb(ATA_FEAT_O(ctrl), 0x00);
	outb(ATA_SECTCNT_O(ctrl), 0x00);
	outb(ATA_CYLLOW_O(ctrl),  (u8_t)(xfer_len & 0xFF));
	outb(ATA_CYLHIGH_O(ctrl), (u8_t)(xfer_len >> 8));

	outb(ATA_CMD_O(ctrl), ATAPI_CMD_PACKET);

	if (ata_wait(ctrl, ATA_ST_DRQ, ATA_ST_BSY, 200000L) != 0) return EIO;

	/* Send 12-byte packet (6 words) */
	for (i = 0; i < 6; i++) {
		u16_t w = ((u16_t)pkt[2*i+1] << 8) | pkt[2*i];
		outw(ATA_DATA_O(ctrl), w);
	}

	if (ata_wait(ctrl, ATA_ST_DRQ, ATA_ST_ERR, 500000L) != 0) return EIO;

	/* Read the 8-byte response (big-endian) */
	for (i = 0; i < 4; i++) {
		u16_t w = inw(ATA_DATA_O(ctrl));
		buf[(i<<1)+0] = (u8_t)(w & 0xFF);
		buf[(i<<1)+1] = (u8_t)(w >> 8);
	}

 	(void)ata_wait(ctrl, 0, ATA_ST_BSY | ATA_ST_DRQ, 500000L);
 	(void)inb(ATA_STATUS_O(ctrl)); /* clear IRQ */

	/* Parse: [last LBA][block length]; both big-endian */
	last_lba = ((u32_t)buf[0] << 24) | ((u32_t)buf[1] << 16) |
		   ((u32_t)buf[2] << 8)  | (u32_t)buf[3];
	blksz    = ((u32_t)buf[4] << 24) | ((u32_t)buf[5] << 16) |
		   ((u32_t)buf[6] << 8)  | (u32_t)buf[7];
	blocks   = last_lba + 1U;

	if (out_blocks) *out_blocks = blocks;
	if (out_blksz)  *out_blksz  = blksz;

	/* Persist for later use (optional fields in ata_unit[]) */
	u->atapi_blocks = blocks;
	u->atapi_blksz  = blksz;
	return 0;
}

/* --- ATAPI: issue 12-byte SCSI INQUIRY and get the device class --- */
static int
atapi_inquiry(int ctrl, int driv, u8_t *out_pdt, u8_t *out_rmb, char *vendor, char *product)
{
	int 	unit = ATA_UNIT_FROM(ctrl, driv),
		i;
	ata_unit_t *u = &ata_unit[unit];
	u16_t 	avail, words, xfer_len = 36; 
	u8_t 	pkt[12] = {0}, buf[64], pdt, rmb;

	pkt[0] = CDB_INQUIRY;  /* INQUIRY */
	pkt[1] = 0x00;  /* EVPD=0 */
	pkt[2] = 0x00;  /* page=0 */
	pkt[3] = 0x00;
	pkt[4] = (u8_t)xfer_len; /* allocation length */
	pkt[5] = 0;

	if (ata_sel(ctrl, driv) != 0) return EIO;

	/* Program expected transfer size into LBA1/LBA2 (ATAPI byte-count) */
	outb(ATA_FEAT_O(ctrl), 0x00);
	outb(ATA_SECTCNT_O(ctrl), 0x00);
	outb(ATA_CYLLOW_O(ctrl),  (u8_t)(xfer_len & 0xFF));
	outb(ATA_CYLHIGH_O(ctrl), (u8_t)(xfer_len >> 8));

	/* Issue PACKET command */
	outb(ATA_CMD_O(ctrl), ATAPI_CMD_PACKET);

	/* Wait for DRQ to send the packet */
	if (ata_wait(ctrl, ATA_ST_DRQ, ATA_ST_BSY, 200000L) != 0) return EIO;

	/* Write the 12-byte packet as 6 words */
	for (i = 0; i < 6; i++) {
		u16_t w = ((u16_t)pkt[2*i+1] << 8) | pkt[2*i];
		outw(ATA_DATA_O(ctrl), w);
	}

	/* Now the device will transfer data; poll for DRQ then read */
	if (ata_wait(ctrl, ATA_ST_DRQ, ATA_ST_ERR, 500000L) != 0) return EIO;

	/* The device sets the actual byte count to read in LBA1/LBA2 */
	avail = ((u16_t)inb(ATA_CYLHIGH_O(ctrl)) << 8) |
		 (u16_t)inb(ATA_CYLLOW_O(ctrl));
	if (avail == 0 || avail > xfer_len) avail = xfer_len;

	words = (int)(avail + 1) >> 1;
	for (i = 0; i < (int)words; i++) {
		u16_t w = inw(ATA_DATA_O(ctrl));
		if ((i<<1) < sizeof(buf)) {
			buf[(i<<1)+0] = (u8_t)(w & 0xFF);
			buf[(i<<1)+1] = (u8_t)(w >> 8);
		}
	}

	/* Final status: wait for DRQ to clear, BSY=0, then read STATUS once */
	(void)ata_wait(ctrl, 0, ATA_ST_BSY | ATA_ST_DRQ, 500000L);
	(void)inb(ATA_STATUS_O(ctrl)); /* clear IRQ, if any */

	/* Parse PDT and strings */
	pdt = buf[0] & 0x1F;
	rmb = (buf[1] >> 7) & 1;

	if (out_pdt) *out_pdt = pdt;
	if (out_rmb) *out_rmb = rmb;

	u->atapi_pdt = pdt;
	u->atapi_rmb = rmb;

	if (vendor) {
        	/* bytes 8..15 vendor, space padded */
        	for (i = 0; i < 8; i++) 
			u->vendor[i] = buf[8+i];
        	u->vendor[8] = 0;

        	/* trim */
        	for (i = 7; i >= 0 && u->vendor[i]==' '; i--) 
			u->vendor[i]=0;
	}

	if (product) {
		/* bytes 16..31 product */
		int j;
		for (i = 0; i < 16; i++) 
			u->product[i] = buf[16+i];
		u->product[16]=0;
		for (j = 15; j >= 0 && u->product[j]==' '; j--) 
			u->product[j]=0;
	}

	/* Map "ZIP" style PDT 0x00 + RMB=1 is removable disk (ZIP/MO) */
	if (pdt == 0x00 && rmb) {
		/* you can special-case print as "ZIP/Removable Disk" */
	}

	return 0;
}

static char *
atapi_class_name(u8_t pdt, int rmb)
{
	switch (pdt) {
	case 0x00: return rmb ? "Removable Disk (ZIP/MO)":"Direct-Access Disk";
	case 0x05: return "CD/DVD";
	case 0x01: return "Tape";
	case 0x04: return "WORM";
	case 0x07: return "Optical Memory";
	case 0x0E: return "Simplified Direct-Access";
	default:   return "Unknown ATAPI";
	}
}

int
atapi_read10(int ctrl,int driv,u32_t lba,u16_t nblks,void *buf)
{
	u8_t	cdb[12], sense[18];
	u32_t	xfer;

	/* Build 12-byte READ(10) command descriptor block */
	bzero((caddr_t)cdb,sizeof(cdb));
	cdb[0] = CDB_READ_10;
	cdb[2] = (u8_t)((lba >> 24) & 0xff);
	cdb[3] = (u8_t)((lba >> 16) & 0xff);
	cdb[4] = (u8_t)((lba >>  8) & 0xff);
	cdb[5] = (u8_t)((lba >>  0) & 0xff);
	cdb[7] = (u8_t)((nblks >> 8) & 0xff);
	cdb[8] = (u8_t)((nblks >> 0) & 0xff);

	/* total transfer size in bytes */
	xfer = (u32_t)nblks * 2048;

	/* send the command */
	return atapi_packet(ctrl,driv,
			    cdb,sizeof(cdb),buf,xfer,1,sense,sizeof(sense));
}

int
atapi_mode_sense10(int ctrl,int driv,u8_t page,u8_t subpage,void *buf,u16_t len)
{
	u8_t	cdb[12], sense[18];

	bzero((caddr_t)cdb,sizeof(cdb));
	cdb[0] = CDB_MODE_SENSE_10;
	cdb[2] = (u8_t)((page & 0x3f) | ((subpage != 0) ? 0x40 : 0x00));
	cdb[3] = subpage;
	cdb[7] = (u8_t)((len >> 8) & 0xff);
	cdb[8] = (u8_t) (len 	   & 0xff);
	return atapi_packet(ctrl,driv,
			    cdb,12,buf,len,1,sense,sizeof(sense));
}

int
atapi_mode_sense6(int ctrl,int driv,u8_t page,u8_t subpage,void *buf,u8_t len)
{
	u8_t	cdb[12], sense[18];

	bzero((caddr_t)cdb,sizeof(cdb));
	cdb[0] = CDB_MODE_SENSE_6;
	cdb[2] = (u8_t)((page & 0x3f) | ((subpage != 0) ? 0x40 : 0x00));
	cdb[3] = subpage;
	cdb[4] = len;
	return atapi_packet(ctrl,driv,
			    cdb,12,buf,len,1,sense,sizeof(sense));
}

/* Issue an ATAPI command and transfer data (polled PIO).
 * dir: 1=data-in, 0=data-out, <0=no data
 * Returns 0 on success; EIO/ETIMEDOUT on failure. If sense!=NULL and an error
 * occurs, a REQUEST SENSE(6) of up to sense_len bytes is attempted.
 */
int
atapi_packet(int ctrl, int driv, u8_t *cdb, int cdb_len, void *buf, u32_t xfer_len, int dir, u8_t *sense, int sense_len)
{
	u_char st, ir, errreg;
	u16_t bc, wcount, w;
	u32_t to_xfer;
	int rc, spins;

	/* Step 1: send PACKET + CDB */
	rc = ata_send_packet(ctrl, driv, cdb, cdb_len);
	if (rc != 0)
		goto sense_or_fail;

	/* Step 2: data/status loop (polled) */
	for (;;) {
		/* Wait BSY clear */
		do { st = inb(ATA_ALTSTATUS_O(ctrl)); } while (st & ATA_ST_BSY);

		/* If DRQ dropped, command is finishing; read final STATUS */
		if ((st & ATA_ST_DRQ) == 0) {
			st = inb(ATA_STATUS_O(ctrl));
			if (st & (ATA_ST_ERR | ATA_ST_DF)) {
				errreg = inb(ATA_FEAT_O(ctrl));
				(void)errreg;
				rc = EIO;
				goto sense_or_fail;
			}
			break; /* success */
		}

		/* DRQ asserted: read IReason + ByteCount */
		ir  = inb(ATA_SECTCNT_O(ctrl));
		bc  = inb(ATA_CYLLOW_O(ctrl));
		bc |= ((u16_t)inb(ATA_CYLHIGH_O(ctrl))) << 8;
		if (bc == 0) bc = 2048;

		if (dir == 1) {
			/* Data-in (device -> host), IO bit must be 1 (ir bit1) */
			if ((ir & 0x02) == 0) { rc = EIO; goto sense_or_fail; }

			to_xfer = (xfer_len < (u32_t)bc) ? xfer_len : (u32_t)bc;
			wcount  = (u16_t)((to_xfer + 1) >> 1);

			insw(ATA_DATA_O(ctrl),buf,wcount);

			/* Drain any surplus the device sent this burst */
			if ((u32_t)bc > to_xfer) {
				u16_t extra = (u16_t)((int)((bc - (u16_t)to_xfer) + 1) >> 1);
				while (extra--) (void)inw(ATA_DATA_O(ctrl));
			}

			buf = (void *)((u_char *)buf + to_xfer);
			if (xfer_len >= to_xfer) xfer_len -= to_xfer;

		} else if (dir == 0) {
			/* Data-out (host -> device), IO bit must be 0 */
			if ((ir & 0x02) != 0) { rc = EIO; goto sense_or_fail; }

			to_xfer = (xfer_len < (u32_t)bc) ? xfer_len : (u32_t)bc;
			wcount  = (u16_t)((to_xfer + 1) >> 1);

			outsw(ATA_DATA_O(ctrl),buf,wcount);

			/* Pad zeros if device asked for more than we have */
			if ((u32_t)bc > to_xfer) {
				u16_t extra2 = (u16_t)((int)((bc - (u16_t)to_xfer) + 1) >> 1);
				while (extra2--) outw(ATA_DATA_O(ctrl), 0);
			}

			buf = (void *)((u_char *)buf + to_xfer);
			if (xfer_len >= to_xfer) xfer_len -= to_xfer;

		} else {
			/* No-data command but device asserts DRQ: drain */
			wcount = (u16_t)((int)(bc + 1) >> 1);
			while (wcount--) (void)inw(ATA_DATA_O(ctrl));
		}
	}

	/* Re-enable interrupts after polled transaction */
	outb(ATA_DEVCTRL_O(ctrl), 0);
	return 0;

sense_or_fail:
	/* Try REQUEST SENSE(6) to report SK/ASC/ASCQ if buffer was provided */
	if (sense != NULL && sense_len >= 18) {
		u_char rs_cdb[12];
		int i;
		for (i = 0; i < 12; ++i) rs_cdb[i] = 0;
		rs_cdb[0] = CDB_REQUEST_SENSE; /* REQUEST SENSE (6) */
		rs_cdb[4] = 18;            /* alloc length */

		/* Clear any pending status then issue sense; ignore its return */
		(void)inb(ATA_STATUS_O(ctrl));
		(void)ata_send_packet(ctrl, driv, rs_cdb, 12);

		/* Wait for DRQ to read sense data (simple, short transfer) */
		spins = 100000;
		while (spins--) {
			st = inb(ATA_ALTSTATUS_O(ctrl));
			if (!(st & ATA_ST_BSY) && (st & ATA_ST_DRQ))
				break;
			ata_udelay(1);
		}
		if (spins > 0) {
			u16_t want = (u16_t)((sense_len < 18) ? sense_len : 18);
			u16_t wds = (u16_t)((int)(want + 1) >> 1);
			for (w = 0; w < wds; ++w)
				*(((u16_t *)sense) + w) = inw(ATA_DATA_O(ctrl));
			(void)inb(ATA_STATUS_O(ctrl));
		}
	}

	outb(ATA_DEVCTRL_O(ctrl), 0); /* re-enable interrupts */
	return (rc != 0) ? rc : EIO;
}

int
atapi_xfer_uio(int is_write, dev_t dev, struct uio *uiop)
{
	int        unit = ATA_UNIT(dev);
	ata_unit_t *u   = &ata_unit[unit];
	ata_chan_t *ch  = &u->chan;
	int        ctrl = u->ctrl;
	int        driv = u->driv;
	u32_t      base_lba, region_len;
	u32_t      atapi_blk, atapi_nblks;
	caddr_t    bounce;
	u32_t      left_bytes;
	int        rc = 0;

	/* CDs are read-only here */
	if (is_write) return EROFS;

	/* Require 2048-byte logical sector */
	if (u->atapi_blksz != 2048) return EINVAL;

	/* Length and offset must be 2KB aligned */
	if ((uiop->uio_resid == 0) ||
	    (uiop->uio_resid & (u->atapi_blksz - 1)))
		return EINVAL;

	/* Map to device region in 512B sectors and convert to 2KB blocks */
	ata_region_from_dev(dev, &base_lba, &region_len);
	atapi_blk   = (base_lba >> 2);     /* 4 * 512 = 2048 */
	atapi_nblks = (region_len >> 2);

	/* Raw/uio path for ATAPI: always polled */
	ch->use_intr = 0;

	bounce     = (caddr_t)ch->xfer_buf; /* must be >= 2048 bytes */
	left_bytes = (u32_t)uiop->uio_resid;

	while (left_bytes > 0) {
		u16_t nblks = (left_bytes / 2048U);
		if (nblks == 0) break;
		if (nblks > 0xFFFFU) nblks = 0xFFFFU;

		/* Issue READ(10) into bounce, then uiomove() out */
		rc = atapi_read10(ctrl, driv, atapi_blk, nblks, bounce);
		if (rc) break;

		rc = uiomove(bounce, (int)(nblks * 2048U), UIO_READ, uiop);
		if (rc) break;

		atapi_blk   += nblks;
		left_bytes  -= (u32_t)nblks * 2048U;
	}
	return rc;
}

int
atapi_strategy(struct buf *bp)
{
	dev_t	   dev=bp->b_edev;
	int        unit = ATA_UNIT(dev);
	ata_unit_t *u   = &ata_unit[unit];
	ata_chan_t *ch  = &u->chan;
	int        ctrl = u->ctrl;
	int        driv = u->driv;
	u32_t      base_lba, region_len;
	u32_t      bsz512, lba10, left, step;
	u16_t      nblks;
	char      *addr;

	if (bp->b_flags & B_READ) {
		/* read-only OK */
	} else {
		bp->b_flags |= B_ERROR;
		bp->b_error  = EROFS;
		biodone(bp);
		return 0;
	}

	if (u->atapi_blksz != 2048) {
		bp->b_flags |= B_ERROR; 
		bp->b_error = EINVAL; 
		biodone(bp); 
		return 0;
	}

	/* Map region (512B sectors) and compute */
	ata_region_from_dev(dev, &base_lba, &region_len);
	bsz512 = 2048U >> 9;     /* = 4 */
	if ((base_lba % bsz512) != 0 || ((bp->b_bcount >> 9) % bsz512) != 0) {
		bp->b_flags |= B_ERROR; 
		bp->b_error = EINVAL; 
		biodone(bp); 
		return 0;
	}

	ch->use_intr = 0; /* polled for ATAPI */

	left = (u32_t)bp->b_bcount >> 9;  /* 512B sectors */
	addr = (char *)bp->b_un.b_addr;

	while (left > 0) {
		lba10 = (base_lba / bsz512);
		nblks = (u16_t)((left / bsz512) > 0xFFFFU ? 0xFFFFU 
							  : (left / bsz512));
		if (nblks == 0) break;

		if (atapi_read10(ctrl, driv, lba10, nblks, addr)) {
			bp->b_flags |= B_ERROR; 
			bp->b_error = EIO; 
			break;
		}

		step     = (u32_t)nblks * bsz512;   /* in 512B sectors */
		base_lba += step;
		left     -= step;
		addr     += (u32_t)nblks * 2048U;
	}

	bp->b_resid = 0;
	biodone(bp);
	return 0;
}
