/*
 * ata_hw.c lower half: identify, waits, PIO read/write, flush
 */

#include "ata.h"

int
ata_sel(ata_ctrl_t *ac, int drive, u32_t lba)
{
	ata_unit_t *u=ac->drive[drive];
	if (!u) return EIO;

	ATADEBUG(9,"ata_sel(%d,%lu): ",drive,lba);
	if (u->lba_ok) {
		u8_t	hi4 = (u8_t)((lba>>24) & 0x0f);
		if (1 || ac->sel_drive != drive || ac->sel_mode != SEL_LBA28 || 
						   ac->sel_hi4 != hi4) {
			ATADEBUG(9,"LBA\n");
			outb(ATA_DRVHD_O(ac), ATA_DH(drive,1,hi4));
			ata_delay400(ac);
			ac->sel_drive = drive;
			ac->sel_mode  = SEL_LBA28;
			ac->sel_hi4   = hi4;
		}
	} else {
		if (1 || ac->sel_drive != drive || ac->sel_mode != SEL_CHS || 
						   ac->sel_hi4 != 0) {
			ATADEBUG(9,"CHS\n");
			outb(ATA_DRVHD_O(ac), ATA_DH(drive,0,0));
			ata_delay400(ac);
			ac->sel_drive = drive;
			ac->sel_mode  = SEL_CHS;
			ac->sel_hi4   = 0;
		}
	}
	return 0;
}

static void
atapi_program_packet(ata_ctrl_t *ac,u8_t drive,u16_t byte_count)
{
	ata_req_t r;

	ATADEBUG(1,"atapi_program_packet(%s)\n",CCstr(ac));
	bzero((caddr_t)&r,sizeof(r));
	r.drive		= drive;
	r.cmd		= ATA_CMD_PACKET;
	r.atapi_bytes	= (byte_count ? byte_count : 2048);
	r.lba_cur	= 0;
	r.sectors_left	= 0;
	r.chunk_left	= 0;
	r.is_write	= 0;
	ata_program_taskfile(ac,&r);
}

static void
atapi_issue_packet(ata_ctrl_t *ac, ata_unit_t *u, u16_t byte_count)
{
	ATADEBUG(1,"atapi_issue_packet(%s)\n",CCstr(ac));

	/* Select device + 400ns settle */
	ata_sel(ac,u->drive,0);

	/* FEAT=0, SECCNT=0, Byte Count in CYCLOW/HIGH */
	outb(ATA_FEAT_O(ac),    0x00);
	outb(ATA_SECTCNT_O(ac), 0x00);
	outb(ATA_CYLLOW_O(ac),  (u8_t)(byte_count & 0xFF));
	outb(ATA_CYLHIGH_O(ac), (u8_t)(byte_count >> 8));

	outb(ATA_CMD_O(ac), ATA_CMD_PACKET);
}

/*********** ATA Routines *************/
void
ata_delay400(ata_ctrl_t *c)
{
	(void)inb(ATA_ALTSTATUS_O(c));
	(void)inb(ATA_ALTSTATUS_O(c));
	(void)inb(ATA_ALTSTATUS_O(c));
	(void)inb(ATA_ALTSTATUS_O(c));
}

int
berror(struct buf *bp, ata_ioque_t *que, int resid, int err)
{
	ATADEBUG(1,"berror()\n");
	bp->b_flags |= B_ERROR; 
	bp->b_error = err; 
	bp->b_resid = resid;
	if (que) que->cur=0;
	biodone(bp); 
	return 0;
}

int
bok(struct buf *bp, ata_ioque_t *que, int resid)
{
	ATADEBUG(1,"bok()\n");
	bp->b_flags &= ~B_ERROR; 
	bp->b_resid = resid;
	if (que) que->cur=0;
	biodone(bp);
	return 0;
}

int
do_ata_strategy(struct buf *bp)
{
	dev_t	dev=bp->b_edev;
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_ioque_t *que = ac->ioque;
	ata_unit_t *u = ac->drive[ATA_DRIVE(dev)];
	ata_req_t *r;
	u32_t   base, len, lba, left;
	u8_t	drive = u->drive;
	char    *addr;
	int     s, is_write;

	ATADEBUG(1,"do_ata_strategy()\n");
	/* Unit present? */
	if (!u->present) return berror(bp,que,0,ENODEV);

	/* Map region (512-byte sectors) */
	ata_region_from_dev(dev, &base, &len);
	if (len == 0) return berror(bp,que,0,ENXIO);

	/* Request geometry from bp */
	lba      = base + (u32_t)bp->b_blkno;        /* 512B sectors */
	left     = (u32_t)(bp->b_bcount >> 9);       /* 512B sectors */
	addr     = (char *)bp->b_un.b_addr;
	is_write = ((bp->b_flags & B_READ) == 0);

	ATADEBUG(5,"do_ata_strategy(%s): build req lba=%lu nsec=%lu bytes=%u is_write=%d buf=%lx\n",
		Cstr(dev),lba,left,bp->b_bcount,is_write,addr);

	/* Polled PIO fallback: split into <=256-sector commands */
	while (left > 0) {
		u32_t 	chunk = (left > 256U) ? 256U : left;

		int   rc    = is_write
			? ata_pio_write(ac, drive, lba, chunk, addr)
			: ata_pio_read (ac, drive, lba, chunk, addr);

		if (rc) return berror(bp,que,0,EIO);

		lba  += chunk;
		left -= chunk;
		addr += (chunk << 9);                /* bytes */
	}
	return bok(bp,que,0);
}

/* Send an ATAPI PACKET and the 12-byte CDB.
 * Returns 0 on success (CDB accepted), or EIO/ETIMEDOUT on failure.
 */
int
ata_send_packet(ata_ctrl_t *ac, u8_t drive, const u8_t *cdb, int cdb_len)
{
	u8_t 	st, errreg;
	u16_t 	words_cdb[6];
	int 	i, spins;

	ATADEBUG(1,"ata_send_packet(%s)\n",CCstr(ac));

	/* Select device + 400ns (4 ALTSTATUS reads) */
	ata_sel(ac,drive,0);

	/* Clear side registers some chips expect 0 */
	outb(ATA_FEAT_O(ac),    0x00);
	outb(ATA_SECTCNT_O(ac), 0x00);
	outb(ATA_SECTNUM_O(ac), 0x00);

	/* Advertise 64KiB burst size (0x0000 is widely compatible) */
	outb(ATA_CYLLOW_O(ac),  0x00);  /* Byte Count Low  */
	outb(ATA_CYLHIGH_O(ac), 0x00);  /* Byte Count High */

	/* Issue PACKET */
	outb(ATA_CMD_O(ac), 0xA0);

	/* Wait for BSY=0 & DRQ=1 (device ready to accept CDB) */
	spins = 200000; /* ~200ms with 1us polls */
	while (spins--) {
		st = inb(ATA_ALTSTATUS_O(ac));
		if (st & ATA_ST_ERR) {
			(void)inb(ATA_ALTSTATUS_O(ac));
			errreg = inb(ATA_FEAT_O(ac));
			return EIO;
		}
		if (!(st & ATA_ST_BSY) && (st & ATA_ST_DRQ)) break;
		drv_usecwait(1); /* ata_udelay(1); */
	}
	if (spins <= 0) {
		(void)inb(ATA_ALTSTATUS_O(ac));
		return ETIMEDOUT;
	}

	/* Write exactly 12 bytes of CDB (pad to 6 words) */
	for (i = 0; i < 6; ++i) words_cdb[i] = 0;
	for (i = 0; i < cdb_len && i < 12; ++i)
		((u8_t *)words_cdb)[i] = cdb[i];
	for (i = 0; i < 6; ++i)
		outw(ATA_DATA_O(ac), words_cdb[i]);

	return 0;
}

int
ata_wait(ata_ctrl_t *ac, u32_t must_set, u32_t must_clear, long usec)
{
	u8_t st;
	long i;

	for (i = 0; i < usec; ++i) {
		st = inb(ATA_ALTSTATUS_O(ac));
		if (((st & must_set) == must_set) && ((st & must_clear) == 0))
			return 0;
		drv_usecwait(1); /* ata_udelay(1);*/
	}
	return EIO;
}

int
ata_wait_ready(ata_ctrl_t *ac)
{
	u32_t	guard = 100000;

	return ata_wait(ac,ATA_ST_DRDY,ATA_ST_BSY|ATA_ST_DRQ,guard);
}

char *
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

void
ata_dump_fdisk(ata_ctrl_t *ac, u8_t drive)
{
	ata_unit_t *u=ac->drive[drive];
	int	fdisk, s;

	ATADEBUG(1,"ata_dump_fdisk(%s)\n",Ustr(u));

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
ata_identify(ata_ctrl_t *ac, int drive)
{
	ata_unit_t *u=ac->drive[drive];
	ata_req_t r;
	u16_t 	id[256];
	int 	i;

	ATADEBUG(1,"ata_identify(%s)\n",CCstr(ac));

	if (!ac->present) return ENODEV;

	ATA_IRQ_OFF(ac,1);

	bzero((caddr_t)&r,sizeof(r));
	r.drive		= drive;
	r.cmd		= (u->devtype & DEV_ATAPI) ? ATA_CMD_IDENTIFY_PKT 
						   : ATA_CMD_IDENTIFY;
	r.lba_cur	= 0;
	r.sectors_left	= 0;
	r.chunk_left	= 0;
	r.is_write	= 0;
	ata_program_taskfile(ac,&r);

	if (ata_wait(ac, ATA_ST_DRQ, ATA_ST_BSY, 500000) != 0) {
		/* no ATA device mark not present */
		u->present = 0;
		return ENODEV;
	}

	insw(ATA_DATA_O(ac),id,256);

	/* Extract model string (word-swapped ASCII) */
	for (i = 0; i < 40; i += 2) {
		u->model[i]   = ((char *)id)[54 + i + 1];
		u->model[i+1] = ((char *)id)[54 + i + 0];
	}
	i=40-1;
	while (i > 0 && u->model[i-1] == ' ') i--;
	u->model[i] = 0;

	/* LBA28 capacity in words 60 61 */
	u->nsectors = ((u32_t)id[61] << 16) | (u32_t)id[60];
	u->lba_ok   = (id[49] & (1<<9)) ? 1 : 0;
	u->present  = 1;
	u->read_only = 0;
	return 0;
}

int
ata_pio_write(ata_ctrl_t *ac, u8_t drive, u32_t lba, int nsec, caddr_t buf)
{
	int 	rc=0, s, i, er;
	u16_t 	*p;
	u8_t 	ast, err;
	ata_req_t r;

	ATADEBUG(1,"ata_pio_write(%s)\n",CCstr(ac));

	if (nsec <= 0) return EINVAL;

	bzero((caddr_t)&r,sizeof(r));
	r.drive		= drive;
	r.lba_cur 	= lba;
	r.nsec 		= nsec;
	r.is_write	= 1;
	r.addr 		= buf;
	r.cmd		= (nsec == 1) ? ATA_CMD_WRITE_SEC 
				      : ATA_CMD_WRITE_SEC_EXT;

	ata_sel(ac,drive,lba);

	while (nsec > 0) {
		r.cmd = ATA_CMD_WRITE_SEC;
		r.nsec = 1;
		ata_program_taskfile(ac,&r);

		if (ata_wait(ac,ATA_ST_DRQ,ATA_ST_BSY,500000) != 0) {
			ATADEBUG(1,"ata_pio_write: no DRQ\n");
			rc = EIO; 
			break; 
		}

		p = (u16_t *)r.addr;
		outsw(ATA_DATA_O(ac),p,256); 

		/* settle and advance for next burst */
		if (ata_wait(ac,ATA_ST_DRDY,ATA_ST_BSY|ATA_ST_DRQ,500000)!=0) { 
			ATADEBUG(1,"ata_pio_write: wait error\n");
			rc = EIO; 
			break; 
		}

		er = ata_err(ac, &ast, &err); 	/*** Check for Error ***/
		if (er) {
			ATADEBUG(9,"pio_write: loop lba=%u AST=%02x Err=%02x\n",
				lba,ast,er);
			rc=EIO;
			break;	
		}

		BUMP(ac,polled_sectors);
		r.addr += ATA_SECTSIZE;
		r.lba_cur++;
		nsec--;
	}
	BUMP(ac,polled_chunks);
	ata_wait(ac,ATA_ST_DRDY,ATA_ST_BSY,500000);
	return rc;
}

int
ata_pio_read(ata_ctrl_t *ac, u8_t drive, u32_t lba, int nsec, caddr_t buf)
{
	int	rc=0, s, i, er;
	u16_t	*p;
	u8_t 	ast, err;
	ata_req_t r;
	
	ATADEBUG(1,"ata_pio_read(ctrl=%s,lba=%d) driv=%d nsec=%d\n",
		CCstr(ac),lba,drive,nsec);

	if (nsec <= 0) return EINVAL;

	bzero((caddr_t)&r,sizeof(r));
	r.drive		= drive;
	r.lba_cur 	= lba;
	r.nsec 		= nsec;
	r.is_write	= 0;
	r.addr 		= buf;
	r.cmd		= (nsec == 1) ? ATA_CMD_WRITE_SEC 
				      : ATA_CMD_WRITE_SEC_EXT;

	while (nsec > 0) {
		r.cmd  = ATA_CMD_READ_SEC;
		r.nsec = 1;
		ata_program_taskfile(ac,&r);

		if (ata_wait(ac, ATA_ST_DRQ, ATA_ST_BSY, 500000) != 0) { 
			ATADEBUG(1,"ata_pio_read: no DRQ\n");
			rc = EIO; 
			break; 
		}

		p = (u16_t *)r.addr;
		insw(ATA_DATA_O(ac),p,256); 

        	if (ata_wait(ac,ATA_ST_DRDY,ATA_ST_BSY|ATA_ST_DRQ,500000)!=0) { 
			ATADEBUG(1,"ata_pio_read: wait error\n");
			rc = EIO; 
			BUMP(ac,wait_bsy_to);
			break; 
		}

		er = ata_err(ac, &ast, &err); 	/*** Check for Error ***/
		if (er) {
			ATADEBUG(9,"pio_read: error AST=%02x Err=%02x\n",
				ast,er);
			rc=EIO;
			break;
		}

        	r.addr  += ATA_SECTSIZE;
		r.lba_cur++;
		nsec--;
	}
	ata_wait(ac,ATA_ST_DRDY,ATA_ST_BSY,500000);
	return rc;
}

int
ata_flush_cache(ata_ctrl_t *ac,u8_t drive)
{
	ata_req_t r;

	ATADEBUG(1,"ata_flush_cache(%s)\n",CCstr(ac));

	bzero((caddr_t)&r,sizeof(r));
	r.drive		= drive;
	r.is_write	= 0;
	r.cmd		= ATA_CMD_FLUSH_CACHE;
	ata_program_taskfile(ac,&r);
	
	if (ata_wait(ac, ATA_ST_DRDY, ATA_ST_BSY|ATA_ST_DRQ, 2000000) != 0)
		return EIO;
	return 0;
}

int
ata_read_fdisk(ata_ctrl_t *ac, u8_t drive)
{
	ata_unit_t *u = ac->drive[drive];
	struct ata_fdisk *fp;
	int	i, fdisk, rc;
	struct mboot *mboot;
	struct ipart *ipart;
	ATADEBUG(1,"ata_read_fdisk(%s)\n",Ustr(u));

	if (u->is_atapi) return 0;

	mboot = (struct mboot *)kmem_alloc(DEV_BSIZE,KM_SLEEP);
	if (!mboot) return ENOMEM;

	rc = ata_pio_read(ac,drive,0,1,(caddr_t)mboot);
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
			fp->vtoc_valid = ata_read_vtoc(ac,drive,fdisk);
	}
	kmem_free((caddr_t)mboot,DEV_BSIZE);
	return 0;
}

int 
ata_read_vtoc(ata_ctrl_t *ac,u8_t drive,int fdisk)
{
	ata_unit_t *u=ac->drive[drive];
	u32_t 	base, lba, off;
	caddr_t k = 0;
	struct pdinfo *pd;
	struct vtoc *v;
	struct ata_fdisk *fp = &u->fd[fdisk];
	int 	s;

	ATADEBUG(1,"ata_read_vtoc(%s)\n",Ustr(u));

	if (u->is_atapi) return 0;

	/* Only attempt on UNIX partitions with a size. */
	if (fp->systid != UNIXOS || fp->nsectors == 0) return 0;

	base = fp->base_lba;

	/* pdinfo + vtoc live in the same sector at (unix_base + VTOC_SEC). */
	lba = base + (u32_t)VTOC_SEC;
	if (!(k = kmem_alloc(DEV_BSIZE, KM_SLEEP))) return 0;

	if (ata_pio_read(ac, drive, lba, 1, k) != 0) {
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

int
ata_read_signature(ata_ctrl_t *ac, u8_t drive,u16_t *type)
{
	u8_t lc=0, hc=0;
	u16_t	dev;

	if (ata_sel(ac,drive,0) != 0) return EIO;

	/* Wait for not-BSY */
	if (ata_wait(ac, 0, ATA_ST_BSY, 500000L) != 0) return EIO;

	lc = inb(ATA_CYLLOW_O(ac));
	hc = inb(ATA_CYLHIGH_O(ac));

	switch ((hc<<8)|lc)
	{
	case 0x0000: 
		dev = DEV_ATA|DEV_PARALLEL;
		break;
	case 0xC33C: 
		dev = DEV_ATA|DEV_SERIAL;
		break;
	case 0xEB14: 
		dev = DEV_ATAPI|DEV_PARALLEL;
		break;
	case 0x9669: 
		dev = DEV_ATAPI|DEV_SERIAL;
		break;
	default:
		dev = DEV_UNKNOWN;
		break;
	}
	*type = dev;
	return 0;
}

void
ata_attach(int ctrl)
{
	ata_ioque_t *que;
	ata_ctrl_t *ac= &ata_ctrl[ctrl];
	int 	i, drive, rc=-1;

	ATADEBUG(1,"ata_attach(%d)\n",ctrl);

	if (!ac->present) return;

	ata_softreset_ctrl(ac);
	for (drive = 0; drive <= 1; drive++) {
		ata_unit_t *u = ac->drive[drive];

		ata_probe_unit(ac,drive);
	}
}

int
ata_probe_unit(ata_ctrl_t *ac, u8_t drive)
{
	ata_ioque_t *que = ac->ioque;
	ata_unit_t *u = ac->drive[drive];
	u8_t 	lc, hc, st;
	int 	is_pkt;
	u32_t	type;
	char 	*klass;

	if (!ac->present) return ENXIO;

	/*** Send an CMD_IDENTIFY to force the signature info on the bus ***/
	outb(ATA_CMD_O(ac), ATA_CMD_IDENTIFY);
	if (ata_read_signature(ac,drive,&u->devtype) != 0) return ENXIO;
	is_pkt = (u->devtype & DEV_ATAPI) ? 1 : 0;

	if (ata_identify(ac, drive) != 0) return ENXIO;

	que->sync_done = 1;
	que->tmo_id    = 0;
	que->tmo_ticks = drv_usectohz(2000000); /* 2s is sane for PIO */

	u->present = 1;
	if (is_pkt) {
		u32_t 	blocks=0, blksz=0;
		char	*med="";
		int	mp = -1;

		/* Populate inquiry fields into ata_unit[] */
		(void)atapi_inquiry(ac, drive, NULL, NULL, NULL, NULL);
		klass = atapi_class_name(u->atapi_pdt, u->atapi_rmb);
		if ((atapi_read_capacity(ac,drive,&blocks,&blksz) == 0) &&
			blocks && blksz) {
			/*printf("blocks=%ld, blksz=%ld\n",blocks,blksz);*/
			;
		}

		u->is_atapi=1;
		u->is_cdrom = (u->atapi_pdt == 0x05);
		u->lbsize = (blksz ? blksz : 2048);
		if (u->lbsize < 512) u->lbsize=512;

		atapi_test_unit_ready(ac,drive,&mp);
		u->media_present=mp;
		if (u->is_cdrom) {
			switch(u->media_present) {
			case 0: med="CD Empty"; break;
			case 1: med="CD Inserted"; break;
			default: med="CD ??"; break;
			}
		}
		printf("%s: ATAPI %s, model=\"%s\" %s\n",
                       Ustr(u),klass,u->model,med);

	} else { /* ATA disk branch */
		char 	*lba28 = u->lba_ok ? "LBA28" : "";
		unsigned long nsec = u->nsectors;

		ulong_t mib = nsec >> 11; 
		ulong_t gib_i = nsec / 2097152UL;
		ulong_t gib_tenths = ((nsec % 2097152UL) * 10UL) / 2097152UL;

		printf("%s: ATA disk, model=\"%s\" %s, %lu sectors (%lu.%u GiB)\n",
			Ustr(u), u->model, lba28, nsec, gib_i,gib_tenths);

		u->lbsize=512;
		ata_read_fdisk(ac,drive);
	}
	u->lbshift=(u8_t)((u->lbsize >> 9) 
			? (u->lbsize==512 ?0:(u->lbsize==1024 ? 1 : 2)) : 0);
	if (atadebug>=9) ata_dump_fdisk(ac,drive);
	return 0;
}

int
atapi_read_capacity(ata_ctrl_t *ac, u8_t drive, u32_t *out_blocks, u32_t *out_blksz)
{
	ata_unit_t *u = ac->drive[drive];
	u16_t xfer_len = 8;   /* READ CAPACITY(10) returns 8 bytes */
	u8_t pkt[12] = {0};
	u8_t buf[16], ast, err;
	int i,er;
	u32_t	last_lba, blksz, blocks;

	/* 10-byte CDB inside 12-byte packet: opcode only, rest zero */
	pkt[0] = CDB_READ_CAPACITY; /* READ CAPACITY(10) */

	if (ata_sel(ac,drive,0) != 0) return EIO;

	/* Program expected byte count */
	atapi_issue_packet(ac, u, (u16_t)xfer_len);

	if (ata_wait(ac, ATA_ST_DRQ, ATA_ST_BSY, 200000L) != 0) return EIO;

	/* Send 12-byte packet (6 words) */
	for (i = 0; i < 6; i++) {
		u16_t w = ((u16_t)pkt[2*i+1] << 8) | pkt[2*i];
		outw(ATA_DATA_O(ac), w);
	}

	if (ata_wait(ac, ATA_ST_DRQ, ATA_ST_ERR, 500000L) != 0) return EIO;

	/* Read the 8-byte response (big-endian) */
	for (i = 0; i < 4; i++) {
		u16_t w = inw(ATA_DATA_O(ac));
		buf[(i<<1)+0] = (u8_t)(w & 0xFF);
		buf[(i<<1)+1] = (u8_t)(w >> 8);
	}

 	(void)ata_wait(ac, 0, ATA_ST_BSY | ATA_ST_DRQ, 500000L);
	er = ata_err(ac, &ast, &err); 	/*** Check for Error ***/

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
int
atapi_inquiry(ata_ctrl_t *ac, u8_t drive, u8_t *out_pdt, u8_t *out_rmb, char *vendor, char *product)
{
	ata_unit_t *u=ac->drive[drive];
	int 	i,er;
	u16_t 	avail, words, xfer_len = 36; 
	u8_t 	pkt[12] = {0}, buf[64], pdt, rmb, ast, err;

	pkt[0] = CDB_INQUIRY;  /* INQUIRY */
	pkt[1] = 0x00;  /* EVPD=0 */
	pkt[2] = 0x00;  /* page=0 */
	pkt[3] = 0x00;
	pkt[4] = (u8_t)xfer_len; /* allocation length */
	pkt[5] = 0;

	if (ata_sel(ac,drive,0) != 0) return EIO;

	/* Program expected transfer size into LBA1/LBA2 (ATAPI byte-count) */
	atapi_issue_packet(ac, u, (u16_t)xfer_len);

	/* Wait for DRQ to send the packet */
	if (ata_wait(ac, ATA_ST_DRQ, ATA_ST_BSY, 200000L) != 0) return EIO;

	/* Write the 12-byte packet as 6 words */
	for (i = 0; i < 6; i++) {
		u16_t w = ((u16_t)pkt[2*i+1] << 8) | pkt[2*i];
		outw(ATA_DATA_O(ac), w);
	}

	/* Now the device will transfer data; poll for DRQ then read */
	if (ata_wait(ac, ATA_ST_DRQ, ATA_ST_ERR, 500000L) != 0) return EIO;

	/* The device sets the actual byte count to read in LBA1/LBA2 */
	avail = ((u16_t)inb(ATA_CYLHIGH_O(ac)) << 8) |
		 (u16_t)inb(ATA_CYLLOW_O(ac));
	if (avail == 0 || avail > xfer_len) avail = xfer_len;

	words = (int)(avail + 1) >> 1;
	for (i = 0; i < (int)words; i++) {
		u16_t w = inw(ATA_DATA_O(ac));
		if ((i<<1) < sizeof(buf)) {
			buf[(i<<1)+0] = (u8_t)(w & 0xFF);
			buf[(i<<1)+1] = (u8_t)(w >> 8);
		}
	}

	/* Final status: wait for DRQ to clear, BSY=0, then read STATUS once */
	(void)ata_wait(ac, 0, ATA_ST_BSY | ATA_ST_DRQ, 500000L); 
	er = ata_err(ac, &ast, &err); 	/*** Check for Error ***/

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

char *
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
atapi_read10(ata_ctrl_t *ac,u8_t drive,u32_t lba,u16_t nblks,void *buf)
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
	return atapi_packet(ac,drive,cdb,sizeof(cdb),buf,xfer,1,sense,sizeof(sense));
}

int
atapi_mode_sense10(ata_ctrl_t *ac,u8_t drive,u8_t page,u8_t subpage,void *buf,u16_t len)
{
	u8_t	cdb[12], sense[18];

	bzero((caddr_t)cdb,sizeof(cdb));
	cdb[0] = CDB_MODE_SENSE_10;
	cdb[2] = (u8_t)((page & 0x3f) | ((subpage != 0) ? 0x40 : 0x00));
	cdb[3] = subpage;
	cdb[7] = (u8_t)((len >> 8) & 0xff);
	cdb[8] = (u8_t) (len 	   & 0xff);
	return atapi_packet(ac,drive,cdb,12,buf,len,1,sense,sizeof(sense));
}

int
atapi_mode_sense6(ata_ctrl_t *ac,u8_t drive,u8_t page,u8_t subpage,void *buf,u8_t len)
{
	u8_t	cdb[12], sense[18];

	bzero((caddr_t)cdb,sizeof(cdb));
	cdb[0] = CDB_MODE_SENSE_6;
	cdb[2] = (u8_t)((page & 0x3f) | ((subpage != 0) ? 0x40 : 0x00));
	cdb[3] = subpage;
	cdb[4] = len;
	return atapi_packet(ac,drive,cdb,12,buf,len,1,sense,sizeof(sense));
}

/* Issue an ATAPI command and transfer data (polled PIO).
 * dir: 1=data-in, 0=data-out, <0=no data
 * Returns 0 on success; EIO/ETIMEDOUT on failure. If sense!=NULL and an error
 * occurs, a REQUEST SENSE(6) of up to sense_len bytes is attempted.
 */
int
atapi_packet(ata_ctrl_t *ac, u8_t drive, u8_t *cdb, int cdb_len, void *buf, u32_t xfer_len, int dir, u8_t *sense, int sense_len)
{
	u8_t st, ir, errreg;
	u16_t bc, wcount, w;
	u32_t to_xfer;
	int rc, spins;

	/* Step 1: send PACKET + CDB */
	rc = ata_send_packet(ac, drive, cdb, cdb_len);
	if (rc != 0)
		goto sense_or_fail;

	/* Step 2: data/status loop (polled) */
	for (;;) {
		/* Wait BSY clear */
		do { st = inb(ATA_ALTSTATUS_O(ac)); } while (st & ATA_ST_BSY);

		/* If DRQ dropped, command is finishing; read final STATUS */
		if ((st & ATA_ST_DRQ) == 0) {
			st = inb(ATA_ALTSTATUS_O(ac));
			if (st & (ATA_ST_ERR | ATA_ST_DF)) {
				errreg = inb(ATA_FEAT_O(ac));
				(void)errreg;
				rc = EIO;
				goto sense_or_fail;
			}
			break; /* success */
		}

		/* DRQ asserted: read IReason + ByteCount */
		ir  = inb(ATA_SECTCNT_O(ac));
		bc  = inb(ATA_CYLLOW_O(ac));
		bc |= ((u16_t)inb(ATA_CYLHIGH_O(ac))) << 8;
		if (bc == 0) bc = 2048;

		if (dir == 1) {
			/* Data-in (device -> host), IO bit must be 1 (ir bit1) */
			if ((ir & 0x02) == 0) { rc = EIO; goto sense_or_fail; }

			to_xfer = (xfer_len < (u32_t)bc) ? xfer_len : (u32_t)bc;
			wcount  = (u16_t)((to_xfer + 1) >> 1);

			insw(ATA_DATA_O(ac),buf,wcount);

			/* Drain any surplus the device sent this burst */
			if ((u32_t)bc > to_xfer) {
				u16_t extra = (u16_t)((int)((bc - (u16_t)to_xfer) + 1) >> 1);
				while (extra--) (void)inw(ATA_DATA_O(ac));
			}

			buf = (void *)((u8_t *)buf + to_xfer);
			if (xfer_len >= to_xfer) xfer_len -= to_xfer;

		} else if (dir == 0) {
			/* Data-out (host -> device), IO bit must be 0 */
			if ((ir & 0x02) != 0) { rc = EIO; goto sense_or_fail; }

			to_xfer = (xfer_len < (u32_t)bc) ? xfer_len : (u32_t)bc;
			wcount  = (u16_t)((to_xfer + 1) >> 1);

			outsw(ATA_DATA_O(ac),buf,wcount);

			/* Pad zeros if device asked for more than we have */
			if ((u32_t)bc > to_xfer) {
				u16_t extra2 = (u16_t)((int)((bc - (u16_t)to_xfer) + 1) >> 1);
				while (extra2--) outw(ATA_DATA_O(ac), 0);
			}

			buf = (void *)((u8_t *)buf + to_xfer);
			if (xfer_len >= to_xfer) xfer_len -= to_xfer;

		} else {
			/* No-data command but device asserts DRQ: drain */
			wcount = (u16_t)((int)(bc + 1) >> 1);
			while (wcount--) (void)inw(ATA_DATA_O(ac));
		}
	}

	return 0;

sense_or_fail:
	/* Try REQUEST SENSE(6) to report SK/ASC/ASCQ if buffer was provided */
	if (sense != NULL && sense_len >= 18) {
		u8_t 	rs_cdb[12];
		int 	i;

		for (i = 0; i < 12; ++i) rs_cdb[i] = 0;
		rs_cdb[0] = CDB_REQUEST_SENSE; /* REQUEST SENSE (6) */
		rs_cdb[4] = 18;            /* alloc length */

		/* Clear pending status then issue sense; ignore its return */
		(void)inb(ATA_ALTSTATUS_O(ac));
		(void)ata_send_packet(ac,drive, rs_cdb, 12);

		/* Wait for DRQ to read sense data (simple, short transfer) */
		spins = 100000;
		while (spins--) {
			st = inb(ATA_ALTSTATUS_O(ac));
			if (!(st & ATA_ST_BSY) && (st & ATA_ST_DRQ))
				break;
			drv_usecwait(1); /* ata_udelay(1);*/
		}
		if (spins > 0) {
			u16_t want = (u16_t)((sense_len < 18) ? sense_len : 18);
			u16_t wds = (u16_t)((int)(want + 1) >> 1);
			for (w = 0; w < wds; ++w)
				*(((u16_t *)sense) + w) = inw(ATA_DATA_O(ac));
			(void)inb(ATA_ALTSTATUS_O(ac));
		}
	}

	return (rc != 0) ? rc : EIO;
}

int
do_atapi_strategy(struct buf *bp)
{
	dev_t	   dev=bp->b_edev;
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_ioque_t *que  = ac->ioque;
	int        drive = ATA_DRIVE(dev);
	ata_unit_t *u   = ac->drive[drive];
	u32_t      base_lba, region_len;
	u32_t      bsz512, lba10, left, step;
	u16_t      nblks;
	char      *addr;

	ATADEBUG(1,"do_atapi_strategy(%s)\n",Ustr(u));

	if (!(bp->b_flags & B_READ)) return berror(bp,que,0,EROFS);

	if (u->atapi_blksz != 2048) return berror(bp,que,0,EINVAL);

	/* Map region (512B sectors) and compute */
	ata_region_from_dev(dev, &base_lba, &region_len);
	bsz512 = 2048U >> 9;     /* = 4 */
	if ((base_lba % bsz512) != 0 || ((bp->b_bcount >> 9) % bsz512) != 0)
		return berror(bp,que,0,EINVAL);

	left = (u32_t)bp->b_bcount >> 9;  /* 512B sectors */
	addr = (char *)bp->b_un.b_addr;

	while (left > 0) {
		lba10 = (base_lba / bsz512);
		nblks = (u16_t)((left / bsz512) > 0xFFFFU ? 0xFFFFU 
							  : (left / bsz512));
		if (nblks == 0) break;

		if (atapi_read10(ac,drive, lba10, nblks, addr)) return berror(bp,que,0,EIO);

		step     = (u32_t)nblks * bsz512;   /* in 512B sectors */
		base_lba += step;
		left     -= step;
		addr     += (u32_t)nblks * 2048U;
	}

	return bok(bp,que,0);
}

void
ata_dump_regs(ata_ctrl_t *ac, char *tag)
{
	u8_t	ast = inb(ATA_ALTSTATUS_O(ac));
	u8_t	er = inb(ATA_ERROR_O(ac));
	ATADEBUG(5,"[%s] %s: AST=%02x ER=%02x\n",
		CCstr(ac),tag,ast,er);
}

void
ata_quiesce_ctrl(ata_ctrl_t *ac)
{
	int	pass;
	u8_t 	ast, st;
	u16_t	bc, words;

	ATADEBUG(9,"ata_quiesce_ctrl(%s)\n",CCstr(ac));
	ata_wait(ac,0,ATA_ST_BSY,500000);

	for(pass=0; pass<2; pass++) {
		ast = inb(ATA_ALTSTATUS_O(ac));
		if (ast & ATA_ST_BSY) break;
		if (!(ast & ATA_ST_DRQ)) break;

		bc = (inb(ATA_CYLHIGH_O(ac)) << 8) |	
		      inb(ATA_CYLLOW_O(ac));
		if (bc == 0) bc=65536;

		words=bc>>1;
		while (words--) inw(ATA_DATA_O(ac));

		inb(ATA_ALTSTATUS_O(ac));
	}
	ATA_IRQ_OFF(ac,1);
	ata_delay400(ac);
}

void	
ata_dump_stats(void)
{
	int	ctrl, driv;

	for(ctrl=0; ctrl<ATA_MAX_CTRL; ctrl++) {
		ata_ctrl_t *c= &ata_ctrl[ctrl];
		ata_unit_t *u0 = c->drive[0];
		ata_unit_t *u1 = c->drive[1];

		if (!c->present) continue;
			
		printf("ATA%d: intrmode=%d present(%d,%d) turnon=%lu turnoff=%lu\n",
			ctrl, c->intr_mode,
			u0 ? u0->present : 0,
			u1 ? u1->present : 0,
			c->counters.irq_turnon,
			c->counters.irq_turnoff);
 		printf("      IRQ: seen=%lu handled=%lu spurious=%lu nocur=%lu bsy=%lu drq_service=%lu\n",
			c->counters.irq_seen, 
			c->counters.irq_handled, 
			c->counters.irq_spurious,
			c->counters.irq_no_cur,
			c->counters.irq_bsy_skipped,
			c->counters.irq_drq_service);
 		printf("      eoc=%lu eoc_polled=%lu lost_irq_rescued=%lu softresets=%lu\n",
			c->counters.irq_eoc, 
			c->counters.eoc_polled,
			c->counters.lost_irq_rescued,
			c->counters.softresets);
 		printf("      WD: arm=%lu cancel=%lu fired=%lu service=%lu rekicked=%lu chunk=%ld\n",
			c->counters.wd_arm,
			c->counters.wd_cancel,
			c->counters.wd_fired,
			c->counters.wd_serviced, 
			c->counters.wd_rekicked,
			c->counters.wd_chunk);
	}
}

void
ata_softreset_ctrl(ata_ctrl_t *ac)
{
	int	was_enabled = ac->irq_enabled;

	ATADEBUG(1,"ata_softreset_ctrl(%d)\n",ac->idx);
	BUMP(ac,softresets);
	ATA_IRQ_OFF(ac,1);
	outb(ATA_DEVCTRL_O(ac), ATA_CTL_SRST | ATA_CTL_NIEN);
	drv_usecwait(5);
	outb(ATA_DEVCTRL_O(ac), ATA_CTL_NIEN); /* deassert SRST */
	drv_usecwait(5);
	(void)inb(ATA_ALTSTATUS_O(ac));

	if (was_enabled) ATA_IRQ_ON(ac);
}

int
atapi_test_unit_ready(ata_ctrl_t *ac, u8_t drive, int *p_media_present)
{
	u8_t	cdb[12] = {0};
	u8_t	sense[18] = {0};
	int	rc;

	cdb[0] = CDB_TEST_UNIT_READY;
	rc = atapi_packet(ac,drive,cdb,sizeof(cdb),NULL,0,1,sense,sizeof(sense));
	if (rc == 0) {
		if (p_media_present) *p_media_present = 1;
		return 0;
	}

	if ((sense[2] & 0x0f) == 0x02 && sense[12] == 0x3a) {
		if (p_media_present) *p_media_present = 0;
		return 0;
	}
	if (p_media_present) *p_media_present = -1;
	return 0;
}
