/*
 * ata_core.c upper half: open/close/read/write/ioctl
 */

#include "ata.h"

/* Exported globals expected by SVR4 */
int 	atadevflag = D_NEW | D_DMA;
int	atadebug   = 0;

ata_ctrl_t ata_ctrl[ATA_MAX_CTRL] = {
	{ 0x1F0, 14, 0 }, /* c0 (Primary)   */
	{ 0x170, 15, 1 }, /* c1 (Secondary) */
	{ 0x1E8, 11, 0 }, /* c2 (Tertiary) */
	{ 0x168, 10, 0 }  /* c3 (Quaternary) */
};

ata_unit_t ata_unit[ATA_MAX_UNITS]; /* up to 2 drives per controller */

/* == IRQ helpers to handle both per-sector and end-of-command interrupts === */
static void
pio_one_sector(ata_unit_t *u, int is_write, caddr_t buf)
{
	int 	c = u->ctrl;

	ATADEBUG("pio_one_sector: %s\n",is_write?"WRITE":"READ");
	if (is_write) outsw(ATA_DATA_O(c), (ushort *)buf, 256);
	else          insw (ATA_DATA_O(c), (ushort *)buf, 256);
}

void
ata_service_irq(ata_unit_t *u, u8_t st)
{
	ata_chan_t *ch = &u->chan;
	ata_req_t  *r  = ch->cur;
	int 	i, c = u->ctrl;

	if (!r) return;

	if (st & (ATA_ST_ERR | ATA_ST_DF)) {
		ata_finish_current(u, EIO);
		return;
	}

	while ((st & ATA_ST_DRQ) && r->chunk_left > 0) {
		pio_one_sector(u, r->is_write, ch->xfer_buf);

		ch->xfer_buf    += 512;
		r->addr         += 512;
		r->xfer_off     += 512;
		r->nleft        -= 512;
		r->sectors_left -= 1;
		r->chunk_left   -= 1;
		r->lba_cur      += 1;

		if (ch->use_intr) ata_arm_watchdog(u, 2*HZ);

		st = inb(ATA_STATUS_O(c));
		ATADEBUG("pio_one_sector: chunk_left=%d sectors_left=%d ST=%02x\n",
			r->chunk_left,r->sectors_left,inb(ATA_ALTSTATUS_O(c)));

		if (st & (ATA_ST_ERR | ATA_ST_DF)) {
			ata_finish_current(u, EIO);
			return;
		}
	}

	if (r->sectors_left == 0) {
		for (i = 0; i < 1000; i++) {
			st = inb(ATA_STATUS_O(c));
			if ((st & (ATA_ST_BSY | ATA_ST_DRQ)) == 0) break;
		}
		ata_finish_current(u, 0);
		return;
	}

	if (r->chunk_left == 0 && r->sectors_left > 0) {
		ata_program_next_chunk(u);
		return;
	}
}

int
ata_should_use_intr(ata_unit_t *u, int is_atapi, u32_t bytes)
{
	int 	unit, ctrl;

	unit = (int)(u - ata_unit);
	ctrl = ATA_CTRL_FROM_UNIT(unit);

 	/* No IRQ wired or unit not present -> polled. */
 	if (ata_ctrl[ctrl].irq == 0 || !u->present) return 0;

	/* ATAPI prefers IRQ handshakes. */
	if (is_atapi) return 1;

	/* ATA PIO: small I/O stays polled; larger I/O uses IRQ. */
	if (bytes < 4096U) return 0;

	return 1;
}

void
ata_region_from_dev(dev_t dev, u32_t *out_base, u32_t *out_len)
{
	int 	ctrl = ATA_CTRL(dev), 
		unit = ATA_UNIT(dev), 
		driv = ATA_DRIVE(dev), 
		fdisk = ATA_FDISK(dev),
		slice = ATA_SLICE(dev);
	ata_unit_t *u = &ata_unit[unit];
	struct ata_fdisk *fp = &u->fd[fdisk];
	u32_t	base, len;

	ATADEBUG("ata_region_from_dev(%s base=%lu, start=%lu\n",
		Cstr(dev), 
		(u32_t)fp->base_lba,
		(u32_t)fp->slice[slice].p_start);

	if (u->is_atapi) {
		u32_t	bsz512=(u->lbsize>>9) ? (u->lbsize>>9) : 1;
		base = 0;
		len = u->atapi_blocks * bsz512;
	} else {
		base = fp->base_lba;
		len  = fp->nsectors;

		if (fp->systid == UNIXOS &&
		    (slice != 0 && slice != ATA_WHOLE_FDISK_SLICE)) {
			base += fp->slice[slice].p_start;
			len  =  fp->slice[slice].p_size;
		}
	}
	*out_base = base;
	*out_len  = len;
	return;
}

char *
Cstr(dev_t dev)
{
static	char	buf[50];

	int 	ctrl = ATA_CTRL(dev), 
		unit = ATA_UNIT(dev), 
		driv = ATA_DRIVE(dev), 
		fdisk = ATA_FDISK(dev),
		slice = ATA_SLICE(dev);
	sprintf(buf,"dev=0x%x (c%dd%dp%ds%d)",dev,ctrl,driv,fdisk,slice);
	return &buf[0];
}
	
void
ataprint(dev_t dev, char *str)
{
	int 	ctrl = ATA_CTRL(dev), 
		unit = ATA_UNIT(dev), 
		driv = ATA_DRIVE(dev), 
		fdisk = ATA_FDISK(dev),
		slice = ATA_SLICE(dev);

	printf("ata: %s (c%dd%dp%d s%d)\n", str ? str : "", 
		ctrl, driv, fdisk, slice);
}

int
ataopen(dev_t *devp, int flags, int otyp, cred_t *crp)
{
	dev_t 	dev = *devp;
	int 	unit = ATA_UNIT(dev),
		fdisk = ATA_FDISK(dev),
		slice = ATA_SLICE(dev);
	ata_unit_t *u = &ata_unit[unit];
	ata_chan_t *ch = &u->chan;
	struct ata_fdisk *fp=&u->fd[fdisk];

	u->ctrl = ATA_CTRL(dev);
	u->driv = ATA_DRIVE(dev);

	ATADEBUG("ataopen(%s)\n",Cstr(dev));

	if (!ata_check_unit(unit)) return ENXIO;
	if ((u->read_only || u->is_atapi) && (flags & FWRITE)) return EROFS;

	if (ch->closing) return EBUSY;

	/* Channel-scoped first-open allocation of bounce buffer */
	if (ch->open_count == 0) {
    		if (ch->xfer_buf == 0) {
        		ch->xfer_buf = (caddr_t)kmem_alloc(ATA_XFER_BUFSZ, KM_SLEEP);
        		if (ch->xfer_buf == 0)
            			return ENOMEM;
        		ch->xfer_bufsz = ATA_XFER_BUFSZ;
    		}
		ch->busy	= 0;
		ch->state	= 0;
		ch->cur		= 0;
		ch->tmo_id	= 0;
	}
	ch->open_count++;
	ch->closing = 0;

	if (u->is_atapi) {
		int	rc;
		u32_t	blocks, blksz;

		rc=atapi_read_capacity(u->ctrl,u->driv,&blocks,&blksz);
		if (rc != 0) {
			if (blksz == 0) blksz = 2048;
			if (blocks == 0) blocks = 0;
		}
		u->atapi_blksz=(blksz ? blksz : 2048);
		u->atapi_blocks=blocks;
		u->nsectors = (u32_t)(blocks*(u->atapi_blksz >> 9));

		if (slice == 0 || slice == ATA_WHOLE_FDISK_SLICE) 
			return 0;
		return ENXIO;
	}

	if (!u->fdisk_valid) return ENXIO;

	if (slice == 0 || slice == ATA_WHOLE_FDISK_SLICE) goto ok;

	if (!fp->vtoc_valid) {
		if (fp->systid == UNIXOS && 
		    fp->nsectors > VTOC_SEC) goto ok;
		return ENXIO;
	}

	if (slice < 0 || slice >= ATA_NPART) return ENXIO;

	if (!fp->vtoc_valid || 
	     fp->slice[slice].p_size == 0) return ENXIO;
ok:
	return 0;
}

int
ataclose(dev_t dev, int flags, int otyp, cred_t *crp)
{
	int	unit = ATA_UNIT(dev),
		ctrl = ATA_CTRL(dev);
	ata_unit_t *u = &ata_unit[unit];
	ata_chan_t *ch = &u->chan;
	int	s;

	ATADEBUG("ataclose(%s)\n",Cstr(dev));
	if (!ata_check_unit(unit)) return ENODEV;

	ch->closing=1;

	s=splbio();
	while (ch->busy || ch->q_head) {
		sleep((caddr_t)&ch->sync_done,PRIBIO);
	}
	splx(s);

	if (ch->open_count > 0) ch->open_count--;

	if (ch->open_count == 0) {
		if (ch->xfer_buf) {
			kmem_free(ch->xfer_buf,ch->xfer_bufsz);
			ch->xfer_buf  = 0;
			ch->xfer_bufsz = 0;
		}
		reset_chan(ch,0);
	}
	ch->closing=0;
	return 0;
}

int
ata_rdwr(int is_write, dev_t dev, struct uio *uiop)
{
	int	unit  = ATA_UNIT(dev);
	ata_unit_t *u = &ata_unit[unit];

	ATADEBUG("ata_rdwr(%s, is_write: %d):\n",Cstr(dev),is_write);
	
	if (!ata_check_unit(unit)) return ENODEV;

	if (is_write && u->read_only) return EROFS;

	if (u->is_atapi)
		return atapi_xfer_uio(is_write,dev,uiop);
	else
		return ata_xfer_uio(is_write,dev,uiop);
}

int
atastrategy(struct buf *bp)
{
	int	dev=bp->b_edev;
	int	unit=ATA_UNIT(dev);
	ata_unit_t *u = &ata_unit[unit];
	ata_chan_t *ch = &u->chan;

        ATADEBUG("atastrategy(%s)\n", Cstr(dev));

        /* Unit present? */
        if (!ata_check_unit(unit)) {
                bp->b_flags |= B_ERROR;
                bp->b_error  = ENODEV;
                biodone(bp);
                return 0;
        }

	if (ch->closing) {
		bp->b_flags |= B_ERROR;
		bp->b_error = EBUSY;
		biodone(bp);
		return 0;
	}

	return (u->is_atapi) ? atapi_strategy(bp)
			     : ata_strategy(bp);
}

int
ataread(dev_t dev, struct uio *uiop, cred_t *crp)
{
	int	err;

	ATADEBUG("ataread(%s)\n",Cstr(dev));
	if ((err=physck(dev,uiop,B_READ)) != 0)
		return err;

	return uiophysio(atastrategy, NULL, dev, B_READ, uiop);
}

int
atawrite(dev_t dev, struct uio *uiop, cred_t *crp)
{
	int	err;

	ATADEBUG("atawrite(%s)\n",Cstr(dev));

	if ((err=physck(dev,uiop,B_WRITE)) != 0)
		return err;
	return uiophysio(atastrategy, NULL, dev, B_WRITE, uiop);
}

/* ---------- ioctls used by vtoc/disksetup tools ----------
 * We honor only the subset you've been exercising:
 *  - V_GETPARMS (disk_parms from vtoc.h)
 *  - V_RDABS / V_WRABS (absolute sector access)
 *  - V_VERIFY   (simple read loop)
 */
int
ataioctl(dev_t dev, int cmd, caddr_t arg, int mode, cred_t *crp, int *rvalp)
{
	int 	ctrl  = ATA_CTRL(dev),
		driv  = ATA_DRIVE(dev),
		fdisk = ATA_FDISK(dev),
		slice = ATA_SLICE(dev),
		unit  = ATA_UNIT_FROM(ctrl, driv);
	ata_unit_t *u = &ata_unit[unit];
	struct ata_fdisk *fp=&u->fd[fdisk];

	ATADEBUG("ataioctl(%s,%x)\n",Cstr(dev),cmd);

	if (!ata_check_unit(unit))
		return ENODEV;

	switch (cmd) {
	case V_CONFIG: {	/* VIOC | 0x01 */
		return 0;
	}
	case V_REMOUNT: {	/* VIOC | 0x02 */
		if (fp->nsectors > 0 && fp->systid == UNIXOS) 
			fp->vtoc_valid = ata_read_vtoc(unit,fdisk);
		return 0;
	}
	case V_GETPARMS: {	/* VIOC | 0x04 */
		struct disk_parms dp;

		/*
		 * We dont have geometry; supply logical CHS compatible 
		 * with 63/16 
		 */
		dp.dp_type   = 1;           /* WINI */
		dp.dp_heads  = 16;
		dp.dp_sectors= 63;
		dp.dp_cyls   = (u16_t)(u->nsectors/(dp.dp_heads*dp.dp_sectors));
		dp.dp_secsiz = DEV_BSIZE;
		dp.dp_ptag   = 0;
		dp.dp_pflag  = 0;
		dp.dp_pstartsec = 0;
		dp.dp_pnumsec  = u->nsectors;

		if (copyout((caddr_t)&dp, arg, sizeof(dp)) < 0)
			return EFAULT;
		return 0;
	}
	case V_FORMAT: {	/* VIOC | 0x05 */
		return 0;
	}
	case V_PDLOC: {		/* VIOC | 0x06 */
		u32_t	vtocloc = ATA_PDLOC;

		if (copyout((caddr_t)&vtocloc, arg,sizeof(vtocloc)) != 0)
			return EFAULT;
		return 0;
	}
	case V_RDABS: {		/* VIOC | 0x0A */
		struct absio ab;
		caddr_t k;
		int rc;

		if (copyin(arg, (caddr_t)&ab, sizeof(ab)) < 0)
			return EFAULT;

		if (ab.abs_buf == 0) return EINVAL;

		k = kmem_alloc(DEV_BSIZE, KM_SLEEP);
		if (!k) return ENOMEM;

		rc = ata_pio_read(ctrl, driv, (u32_t)ab.abs_sec, 1, k);
		if (rc == 0) {
			 if (copyout(k, ab.abs_buf, DEV_BSIZE) < 0) 
				rc = EFAULT;
		}
		else 
			rc = EIO;

		kmem_free(k, DEV_BSIZE);
		return rc;
	}
	case V_WRABS: {		/* VIOC | 0x0B */
		struct absio ab;
		caddr_t k;
		int 	rc;

		if (u->read_only) return EROFS;

		if (copyin(arg, (caddr_t)&ab, sizeof(ab)) < 0)
			return EFAULT;

		k = kmem_alloc(DEV_BSIZE, KM_SLEEP);
		if (!k) return ENOMEM;

		if (copyin((caddr_t)ab.abs_buf, k, DEV_BSIZE) < 0) {
			kmem_free(k, DEV_BSIZE);
			return EFAULT;
		}

		rc = ata_pio_write(ctrl, driv, (u32_t)ab.abs_sec, 1, k);
		kmem_free(k, DEV_BSIZE);
		if (rc==0) u->fd[fdisk].vtoc_valid=0;
		return rc ? EIO : 0;
	}
	case V_VERIFY: {	/* VIOC | 0x0C */
    		union vfy_io vfy;

		vfy.vfy_out.err_code=0;
		if (copyout((caddr_t)&vfy,arg,sizeof(vfy)) < 0)
			return EFAULT;
		return 0;
	}
	default:
		printf("Unknown IOCTL %x\n",cmd);
		return ENOTTY;
	}
}

int
atasize(dev_t dev)
{
	int 	ctrl  = ATA_CTRL(dev), 
		unit  = ATA_UNIT(dev),
		driv  = ATA_DRIVE(dev),
		fdisk = ATA_FDISK(dev),
		slice = ATA_SLICE(dev);
	ata_unit_t *u = &ata_unit[unit];

	if (!ata_check_unit(unit)) return -1;

	if (u->is_atapi || !u->fd[fdisk].vtoc_valid)
		return (int)u->nsectors;

	return (int)u->fd[fdisk].slice[slice].p_size;
}

int
atainit(void)
{
	int 	ctrl; 

	for (ctrl = 0; ctrl < ATA_MAX_CTRL; ctrl++) {
		/*** This skips controllers we dont want to configure ***/
		if (!ata_ctrl[ctrl].present) continue;

		ata_attach(ctrl);
	}
	return 0;
}

/* ---------- interrupt handler (ATA PIO, per-sector handshakes) ---------- */
int
ataintr(int ipl)
{
	int	handled = 0;
	int 	ctrl, d;
	u8_t 	st;

	for (ctrl = 0; ctrl < ATA_MAX_CTRL; ctrl++) {
		if (ata_ctrl[ctrl].irq != ipl) continue;

		for (d = 0; d < 2; d++) {
			ata_unit_t *u = &ata_unit[(ctrl<<1) | d];
			ata_chan_t *ch = &u->chan;

			if (!u->present) continue;
			if (!ch->use_intr) continue;
			if (!ch->cur) continue;

			outb(ATA_DRVHD_O(ctrl), 
			     ATA_DH(u->driv, 1, (ch->cur->lba_cur>>24)&0x0F));

			if (inb(ATA_ALTSTATUS_O(ctrl)) & ATA_ST_BSY) continue;

			st = inb(ATA_STATUS_O(ctrl));
			ATADEBUG(">>> IRQ ENTRY ctrl=%d STATUS=%02x\n", 
				ctrl, st);
			if (st & (ATA_ST_DRQ | ATA_ST_DF | ATA_ST_ERR) || 
			    (st & ATA_ST_BSY) == 0) {
				ata_service_irq(u, st);
				handled = 1;
			}
		}
	}
	return handled ? DDI_INTR_CLAIMED : DDI_INTR_UNCLAIMED;
}
