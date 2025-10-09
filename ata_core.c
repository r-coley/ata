/*
 * ata_core.c upper half: open/close/read/write/ioctl
 */

#include "ata.h"

/* Exported globals expected by SVR4 */
int 	atadevflag = D_NEW | D_DMA;
int	atadebug   = 0;
int 	ata_force_polling = 0;
u32_t	req_seq=0;

ata_ctrl_t ata_ctrl[ATA_MAX_CTRL] = {
	{ 0x1F0, 14, 0 }, /* c0 (Primary)   */
	{ 0x170, 15, 1 }, /* c1 (Secondary) */
	{ 0x1E8, 11, 0 }, /* c2 (Tertiary) */
	{ 0x168, 10, 0 }  /* c3 (Quaternary) */
};

ata_unit_t ata_unit[ATA_MAX_UNITS]; /* up to 2 drives per controller */

void 
ATADEBUG(int x, char *fmt, ... )
{
	if (atadebug >= x) {
		xprintf(fmt, (uint_t)((uint_t *)&fmt + 1));
	}
}

int
ata_err(ata_ctrl_t *ac, u8_t *ast, u8_t *err)
{
	u8_t	x, y;

	x = inb(ATA_ALTSTATUS_O(ac));
	*ast = (x & 0xff);
	if ((x & (ATA_ST_BSY|ATA_ST_DRQ)) == 0) {
		y = inb(ATA_ERROR_O(ac));
		if (err) *err=y;
		return (u8_t)(x & (ATA_ST_ERR | ATA_ST_DF)) ? EIO : 0;
	}
	return 0;
}

void
pio_one_sector(ata_ctrl_t *ac, u8_t drive, int is_write)
{
	ata_ioque_t *que = ac->ioque;
	ata_req_t  *r  = que->cur;
	ata_unit_t *u=ac->drive[drive];
	u16_t	*p = (u16_t *)que->xfer_buf;
	u8_t	ast, err;
	int	er;
	
	er = ata_err(ac, &ast, &err); 	/*** Check for Error ***/

	ATADEBUG(1,"pio_one_sector: is_write=%d AST %02x Er %02x\n", 
		is_write,ast,er);

	if (is_write) 
		outsw(ATA_DATA_O(ac), que->xptr, 256);
	else          
		insw (ATA_DATA_O(ac), que->xptr, 256);
}

void
ata_service_irq(ata_ctrl_t *ac, u8_t drive, u8_t st)
{
	ata_ioque_t *q = ac->ioque;
	ata_req_t   *r = (q ? q->cur : NULL);

	if (!r) {
		BUMP(ac, irq_no_cur);
		return;
	}

	/* ---- (1) Data phase: DRQ asserted → move exactly one sector ---- */
	if (st & ATA_ST_DRQ) {
		u8_t *p = (u8_t *)q->xptr;   /* base of this 512B sector */

		if (r->is_write) {
			/* WRITE: push 512 bytes from queue buffer */
			outsw(ATA_DATA_O(ac), p, 256);
		} else {
			/* READ: pull 512 bytes into queue buffer */
			insw(ATA_DATA_O(ac), p, 256);

			/* Bounce buffer: copy the just-read sector into the real buffer */
			if (r->flags & ATA_RF_NEEDCOPY) {
				bcopy((caddr_t)p,
				      (caddr_t)((u8_t *)r->addr + r->xfer_off),
				      512);
			}
		}

		/* Advance pointers and counters */
		q->xptr     += 512;
		r->xfer_off += 512;
		if (r->chunk_left)   r->chunk_left--;
		if (r->sectors_left) r->sectors_left--;
		r->lba_cur  += 1;

		BUMP(ac, irq_drq_service);
		BUMP(ac, wd_serviced);

		ATADEBUG(5, "%s: IRQ DRQ: is_%s left=%d chunk=%d st=%02x\n",
		         CCstr(ac), r->is_write ? "WR" : "RD",
		         r->sectors_left, r->chunk_left, st);

		/* Do NOT finish/cancel/kick here — wait for completion IRQ (no BSY/DRQ) */
		ata_arm_watchdog(ac, HZ/8);
		return;
	}

	/* ---- (2) Completion phase: not busy and no data requested ---- */
	if ((st & (ATA_ST_BSY | ATA_ST_DRQ)) == 0) {
		if (st & (ATA_ST_ERR | ATA_ST_DF)) {
			u8_t er = inb(ATA_ERROR_O(ac));
			ATADEBUG(2, "%s: ERROR st=%02x er=%02x left=%d chunk=%d\n",
			         CCstr(ac), st, er, r->sectors_left, r->chunk_left);
			ata_cancel_watchdog(ac);
			ata_finish_current(ac, EIO, 8);
			return;
		}

		/* success */
		ata_cancel_watchdog(ac);
		ata_finish_current(ac, 0, 8);
		ata_kick(ac);
		return;
	}

	/* ---- (3) Still busy ---- */
	BUMP(ac, irq_bsy_skipped);
	ata_arm_watchdog(ac, HZ/8);
}


#ifdef ORIG
void
ata_service_irq(ata_ctrl_t *ac, u8_t drive, u8_t st)
{
	ata_ioque_t *que = ac->ioque;
	ata_req_t  *r  = que ? que->cur : NULL;
	u8_t 	st2, er;

	ATADEBUG(1,"ata_service_irq(%s,%02x) reqid=%lu\n",
		CCstr(ac),st,r?r->reqid:0);
	if (!r) {
		printf("No r so return\n");
		BUMP(ac,irq_no_cur);
		return;
	}

	/* --- Drive ERROR or Drive Fault --- */
	if (st & (ATA_ST_ERR | ATA_ST_DF)) {
		BUMP(ac, irq_err);
		er = inb(ATA_ERROR_O(ac));
		ATADEBUG(5,"Err st=%02x er=%02x left=%d chunk=%d\n", 
			st, er, r->sectors_left, r->chunk_left);
		ata_finish_current(ac, EIO,3);
		return;
	}

	if (st & ATA_ST_BSY) {
		BUMP(ac, irq_bsy_skipped);
		printf("ATA_ST_BSY so return\n");
		return;
	}

	/* --- Normal Data Phase (per-sector handshake) --- */
	if (st & ATA_ST_DRQ) {
		ATADEBUG(1,"ATA_ST_DRQ sleft=%lu cleft=%lu\n",
			r->sectors_left,r->chunk_left);

		if (r->chunk_left == 0 && r->sectors_left == 0) {
			prtinf("goto eoc_check\n");
			goto eoc_check;
		}

		pio_one_sector(ac, drive, r->is_write);
		BUMP(ac,irq_drq_service);
		BUMP(ac, wd_serviced);
		
        	/* progress counters */
        	if (r->chunk_left) r->chunk_left--;
        	if (r->sectors_left) r->sectors_left--;
		r->xfer_off += 512;
		r->lba_cur += 1;

		ATADEBUG(5,"%s: IRQ DRQ: is_%s left=%d chunk=%d st=%02x\n",
			CCstr(ac),r->is_write ? "WR" : "RD",
			r->sectors_left, r->chunk_left, st);

		if (r->chunk_left > 0) {
			BUMP(ac,wd_chunk);
			ata_arm_watchdog(ac, HZ/8);
			return;
		}

		if (!r->is_write && que->xfer_buf) 
			r->flags |= ATA_RF_NEEDCOPY;

		if (r->sectors_left == 0) {
			ata_finish_current(ac, 0, 5);
			return;
		}
		/* chunk complete: copy back for READs if using bounce */
		if (!r->is_write && (r->flags & ATA_RF_NEEDCOPY) && 
		    que->xfer_buf && r->chunk_bytes) {
			caddr_t dst = (caddr_t)((char *)r->addr + (r->xfer_off - r->chunk_bytes));
			printf("Copying to bounce buffer\n");
			bcopy(que->xfer_buf, dst, r->chunk_bytes);
			r->flags &= ~ATA_RF_NEEDCOPY;
		}
		ata_program_next_chunk(ac,r,HZ/8);
		return;
	}

eoc_check:
	ATADEBUG(1,"eoc_check sleft=%lu cleft=%lu\n",
			r->sectors_left,r->chunk_left);
	if (r->sectors_left == 0 && r->chunk_left == 0) {
		ata_finish_current(ac, 0, 8);
		return;
	}
	BUMP(ac,irq_spurious);
}
#endif

void
ata_region_from_dev(dev_t dev, u32_t *out_base, u32_t *out_len)
{
	int 	fdisk = ATA_FDISK(dev),
		slice = ATA_SLICE(dev);
	ata_unit_t *u = &ata_unit[ATA_UNIT(dev)];
	struct ata_fdisk *fp = &u->fd[fdisk];
	u32_t	base, len, bsz512;

	ATADEBUG(1,"ata_region_from_dev(%s base=%lu, start=%lu\n",
		Cstr(dev), 
		(u32_t)fp->base_lba,
		(u32_t)fp->slice[slice].p_start);

	if (u->is_atapi) {
		bsz512=(u->lbsize>>9) ? (u->lbsize>>9) : 1;
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
CCstr(ata_ctrl_t *ac) 
{
	int	c, d;
static	char	buf[20];

	c=ac->idx;
	d=ac->sel_drive;
	sprintf(buf,"c%dd%d",c,d);
	return buf;
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
	sprintf(buf,"c%dd%dp%ds%d",ctrl,driv,fdisk,slice);
	return &buf[0];
}
	
void
ataprint(dev_t dev, char *str)
{
	printf("ata: %s %s\n", str ? str : "", Cstr(dev));
}

int
ataopen(dev_t *devp, int flags, int otyp, cred_t *crp)
{
	dev_t 	dev = *devp;
	int 	fdisk = ATA_FDISK(dev),
		slice = ATA_SLICE(dev);
	int	drive=ATA_DRIVE(dev);
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_ioque_t *que = ac->ioque;
	ata_unit_t *u = ac->drive[drive];
	struct ata_fdisk *fp=&u->fd[fdisk];

	ATADEBUG(1,"ataopen(%s)\n",Cstr(dev));

	drive = u->drive = ATA_DRIVE(dev);

	if (!u->present) return ENXIO;
	if ((u->read_only || u->is_atapi) && (flags & FWRITE)) return EROFS;

	if (que->closing) return EBUSY;

	/* Channel-scoped first-open allocation of bounce buffer */
	if (que->open_count == 0) {
    		if (que->xfer_buf == 0) {
        		que->xfer_buf = (caddr_t)kmem_zalloc(ATA_XFER_BUFSZ, KM_SLEEP);
        		if (que->xfer_buf == 0)
            			return ENOMEM;
        		que->xfer_bufsz = ATA_XFER_BUFSZ;
			ATADEBUG(5,"%s: xfer_buf=%lx\n",Ustr(u),que->xfer_buf);
    		}
		que->busy	= 0;
		que->state	= 0;
		que->cur	= 0;
		que->tmo_id	= 0;
	}
	que->open_count++;
	que->closing = 0;

	if (u->is_atapi) {
		int	rc;
		u32_t	blocks, blksz;

		rc=atapi_read_capacity(ac,drive,&blocks,&blksz);
		if (rc != 0) {
			if (blksz == 0) blksz = 2048;
			if (blocks == 0) blocks = 0;
		}
		u->atapi_blksz=(blksz ? blksz : 2048);
		u->atapi_blocks=blocks;
		u->nsectors = (u32_t)(blocks*(u->atapi_blksz >> 9));

		if (slice == 0 || slice == ATA_WHOLE_FDISK_SLICE) return 0;
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
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_ioque_t *que = ac->ioque;
	ata_unit_t *u = ac->drive[ATA_DRIVE(dev)];
	int	s;

	ATADEBUG(1,"ataclose(%s)\n",Cstr(dev));
	if (!u->present) return ENODEV;

	que->closing=1;

	s=splbio();
	while (que->busy || que->q_head) {
		sleep((caddr_t)&que->sync_done,PRIBIO);
	}
	splx(s);

	if (que->open_count > 0) que->open_count--;

	if (que->open_count == 0) {
		if (que->xfer_buf) {
			ATADEBUG(5,"ataclose(%s: free %lx\n",
				Ustr(u),que->xfer_buf);
			kmem_free(que->xfer_buf,que->xfer_bufsz);
			que->xfer_buf  = 0;
			que->xfer_bufsz = 0;
		}
		reset_queue(que,0);
	}
	que->closing=0;
	return 0;

}

int
atastrategy(struct buf *bp)
{
	int	dev=bp->b_edev, is_wr, s, do_kick=0;
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_unit_t *u = ac->drive[ATA_DRIVE(dev)];
	ata_ioque_t *que = ac->ioque;
	ata_req_t *r;
	u32_t	base, len, lba, left;
	char	*addr;

        ATADEBUG(1,"atastrategy(%s)\n", Cstr(dev));

        if (!u->present) return berror(bp,que,0,ENODEV);

	/*** If the controller is IRQ mode we just queue the request ***/
	if (!ac->intr_mode) {
		return (u->is_atapi) ? do_atapi_strategy(bp)
			     	     : do_ata_strategy(bp);
	}

	ata_region_from_dev(dev,&base,&len);
	
	r = (ata_req_t *)kmem_zalloc(sizeof(*r),KM_SLEEP);
	if (!r) return berror(bp,que,0,ENOMEM);

	r->is_write	= (bp->b_flags & B_READ) ? 0 : 1;
	r->reqid 	= ++req_seq;
	r->dev		= dev;
	r->drive	= ATA_DRIVE(dev);
	r->cmd 		= r->is_write ? ATA_CMD_WRITE_SEC : ATA_CMD_READ_SEC;
	r->lba          = base + (u32_t)bp->b_blkno;
	r->lba_cur      = base + (u32_t)bp->b_blkno;
	r->addr         = (char *)bp->b_un.b_addr;
	r->nsec         = (u32_t)(bp->b_bcount >> 9);
	r->sectors_left = (u32_t)(bp->b_bcount >> 9);
	r->bp = bp;

	s = splbio();
	que->sync_done = 0;
	ata_q_put(ac, r);
	do_kick = (!que->busy && !que->cur);
	splx(s);
	if (do_kick) need_kick(ac);

	s=splbio();
	while (!que->sync_done)
		sleep((caddr_t)&que->sync_done, PRIBIO);
	splx(s);
	return 0;
}

int
ataread(dev_t dev, struct uio *uiop, cred_t *crp)
{
	int	err;

	ATADEBUG(1,"ataread(%s)\n",Cstr(dev));
	if ((err=physck(dev,uiop,B_READ)) != 0) return err;
	return uiophysio(atastrategy, NULL, dev, B_READ, uiop);
}

int
atawrite(dev_t dev, struct uio *uiop, cred_t *crp)
{
	int	err;

	ATADEBUG(1,"atawrite(%s)\n",Cstr(dev));

	if ((err=physck(dev,uiop,B_WRITE)) != 0) return err;
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
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_unit_t *u = ac->drive[ATA_DRIVE(dev)];
	ata_ioque_t *que = ac->ioque;
	int 	fdisk = ATA_FDISK(dev);
	int	drive = u->drive;
	struct ata_fdisk *fp=&u->fd[fdisk];

	ATADEBUG(1,"ataioctl(%s,%x)\n",Cstr(dev),cmd);

	if (!u->present) return ENODEV;

	switch (cmd) {
	case V_CONFIG: {	/* VIOC | 0x01 */
		return 0;
	}
	case V_REMOUNT: {	/* VIOC | 0x02 */
		if (fp->nsectors > 0 && fp->systid == UNIXOS) 
			fp->vtoc_valid = ata_read_vtoc(ac,drive,fdisk);
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

		rc = ata_pio_read(ac, drive, (u32_t)ab.abs_sec, 1, k);
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

		rc = ata_pio_write(ac, drive, (u32_t)ab.abs_sec, 1, k);
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
	int 	fdisk = ATA_FDISK(dev),
		slice = ATA_SLICE(dev);
	ata_unit_t *u = &ata_unit[ATA_UNIT(dev)];

	if (!u->present) return -1;

	if (u->is_atapi || !u->fd[fdisk].vtoc_valid)
		return (int)u->nsectors;

	return (int)u->fd[fdisk].slice[slice].p_size;
}

int
atainit(void)
{
	int 	ctrl; 
	ata_ioque_t *que;

	ATADEBUG(1,"atainit()\n");
	bzero((caddr_t)&ata_unit[0],sizeof(ata_unit_t)*ATA_MAX_UNITS);
	for (ctrl = 0; ctrl < ATA_MAX_CTRL; ctrl++) {
		ata_ctrl[ctrl].idx = ctrl;

		que = (ata_ioque_t *)kmem_zalloc(sizeof(ata_ioque_t),KM_SLEEP);
		if (!que) return -1;

		ata_ctrl[ctrl].intr_mode = ata_force_polling ? 0 : 1;
		ata_ctrl[ctrl].irq_enabled = 0;
		ata_ctrl[ctrl].sel_drive = -1;
		ata_ctrl[ctrl].sel_hi4 = 0xff;
		ata_ctrl[ctrl].sel_mode = 0;
		ata_ctrl[ctrl].ioque = que;

		ata_ctrl[ctrl].drive[ 0 ] = &ata_unit[ATA_UNIT_FROM(ctrl,0)];
		ata_ctrl[ctrl].drive[ 1 ] = &ata_unit[ATA_UNIT_FROM(ctrl,1)];

		ata_ctrl[ctrl].drive[0]->drive = 0;
		ata_ctrl[ctrl].drive[1]->drive = 1;

		if (ata_ctrl[ctrl].present) ata_attach(ctrl);
	}
	return 0;
}

char *
Ustr(ata_unit_t *u)
{
	if (!u) return "???";
	return u->drive ? "c1d1" : "c1d0";
}

/* ---------- interrupt handler (ATA PIO, per-sector handshakes) ---------- */
int
ataintr(int ipl)
{
	int 	ctrl;
	ata_ctrl_t *ac;
	ata_ioque_t *q;
	ata_req_t *r;
	int	drive, er;
	u8_t 	st, ast, err;
	u32_t	lba;

	ATADEBUG(1,"ataintr(%d)\n",ipl);
	for (ctrl = 0; ctrl < ATA_MAX_CTRL; ctrl++) {
		ac=&ata_ctrl[ctrl];
		if (ac->irq == ipl && ac->present) break;
	}
	if (ctrl==ATA_MAX_CTRL) {
		return DDI_INTR_UNCLAIMED;
	}

	if (!ac->intr_mode) {
		BUMP(ac,irq_spurious);
		return DDI_INTR_CLAIMED;
	}	

	q = ac->ioque;
	r = q ? q->cur : 0;
	if (!r) { 
		BUMP(ac,irq_no_cur);
		printf("q->cur is NULL\n");
		return DDI_INTR_UNCLAIMED;
	}
	drive = r ? r->drive : ac->sel_drive;

	/*** Check for Error ***/
	er = ata_err(ac, &ast, &err);
	if (er) printf("c%dd%d: Error %02x AST %02x ERR %02x\n",
		ctrl,drive,er,ast,err);

	BUMP(ac,irq_seen);
	if (ast & ATA_ST_BSY) {
		BUMP(ac,irq_bsy_skipped);
		return DDI_INTR_CLAIMED;
	}

	st = inb(ATA_STATUS_O(ac)); /* ataintr() correct */
	BUMP(ac, irq_handled);

	ata_service_irq(ac, drive, st);
	return DDI_INTR_CLAIMED;
}
