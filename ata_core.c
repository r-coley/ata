/*
 * ata_core.c 
 *
 * Drive upper half: open/close/read/write/ioctl
 */

#include "ata.h"

/* Exported globals expected by SVR4 */
int 	atadevflag = D_NEW | D_DMA;
int	atadebug   = 0;
int 	ata_force_polling = 1;

ata_ctrl_t ata_ctrl[ATA_MAX_CTRL] = {
	{ 0x1F0, 14, 0 }, /* c0 (Primary)   */
	{ 0x170, 15, 1 }, /* c1 (Secondary) */
	{ 0x1E8, 11, 1 }, /* c2 (Tertiary) */
	{ 0x168, 10, 0 }  /* c3 (Quaternary) */
};

ata_unit_t ata_unit[ATA_MAX_UNITS]; /* up to 2 drives per controller */

void
ataprint(dev_t dev, char *str)
{
	printf("ata: %s %s\n", str ? str : "", Dstr(dev));
}

int
ataopen(dev_t *devp, int flags, int otyp, cred_t *crp)
{
	dev_t 	dev = *devp;
	int 	fdisk = ATA_PART(dev),
		slice = ATA_SLICE(dev),
		ctrl  = ATA_CTRL(dev),
		drive = ATA_DRIVE(dev);
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_ioque_t *q = ac->ioque;
	ata_unit_t *u = ac->drive[drive];
	ata_part_t *fp=&u->fd[fdisk];
	int	s;

	ATADEBUG(1,"ataopen(%s) present=%d dev=%x part=%d ctrl=%d driv=%d slice=%d\n",
		Dstr(dev),u->present,dev,fdisk,ctrl,drive,slice);

	if (!u->present) return ENXIO;
	if ((u->read_only || u->is_cdrom) && (flags & FWRITE)) return EROFS;

	if (ISABSDEV(dev)) return ENXIO;

	s=splbio();
	if (AC_HAS_FLAG(ac, ACF_CLOSING)) {
		splx(s);
		return EBUSY;
	}

	/* Channel-scoped first-open allocation of bounce buffer */
	if (q->open_count == 0) {

		/*** Read the partition table ***/
		if (ata_pdinfo(ABSDEV(dev)) != 0) {
			printf("Failed to get pdinfo\n");
			splx(s);
			return ENXIO;
		}

    		if (q->xfer_buf == 0) {
        		q->xfer_buf = (caddr_t)kmem_zalloc(ATA_XFER_BUFSZ, KM_SLEEP);
        		if (q->xfer_buf == 0)
            			return ENOMEM;
        		q->xfer_bufsz = ATA_XFER_BUFSZ;
			ATADEBUG(5,"%s: xfer_buf=%lx\n",Cstr(ac),q->xfer_buf);
    		}
		AC_CLR_FLAG(ac,ACF_BUSY); 
		q->state	= AS_IDLE;
		q->cur		= NULL; /* opencount==0 */
		ac->tmo_id	= 0;
	}
	
	AC_CLR_FLAG(ac,ACF_CLOSING); 
	splx(s);

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

		if (slice == 0 || slice == ATA_WHOLE_PART_SLICE) goto ok;
		return ENXIO;
	}

	if (!fp->vtoc_valid) {
		if (fp->systid == UNIXOS) {
			if (fp->nsectors > VTOC_SEC) goto ok;
		}
		if (slice == 0 || slice == ATA_WHOLE_PART_SLICE) goto ok;
		printf("Return here1\n");
		return ENXIO;
	}

	if (slice < 0 || slice > ATA_NPART-1) {
		printf("Return here2\n");
		return ENXIO;
	}

	if (!fp->vtoc_valid || 
	     fp->slice[slice].p_size == 0) {
		printf("Return here3\n");
		return ENXIO;
	}
ok:
	q->open_count++;
	return 0;
}

int
ataclose(dev_t dev, int flags, int otyp, cred_t *crp)
{
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_ioque_t *q = ac->ioque;
	ata_unit_t *u = ac->drive[ATA_DRIVE(dev)];
	ata_part_t *fp=&u->fd[ATA_PART(dev)];
	int	s;

	ATADEBUG(1,"ataclose(%s) present=%d fdisk_valid=%d vtoc_valid=%d\n",
		Dstr(dev),u->present,u->fdisk_valid,fp->vtoc_valid);

	if (!u->present) return ENODEV;

	s=splbio();
	AC_SET_FLAG(ac, ACF_CLOSING);
	while (AC_HAS_FLAG(ac,ACF_BUSY) || q->q_head) {
		printf("busy=%d q_head=%lx\n",AC_HAS_FLAG(ac,ACF_BUSY),q->q_head);
		sleep((caddr_t)ac->ioque,PRIBIO);
	}

	if (q->open_count > 0) q->open_count--;

	if (q->open_count == 0) {
		if (q->xfer_buf) {
			ATADEBUG(5,"ataclose(%s: free %lx\n",
				Cstr(ac),q->xfer_buf);
			kmem_free(q->xfer_buf,q->xfer_bufsz);
			q->xfer_buf  = 0;
			q->xfer_bufsz = 0;
		}
		u->fdisk_valid=0;
		reset_queue(ac,0);
	}
	AC_CLR_FLAG(ac,ACF_CLOSING);
	splx(s);
	return 0;

}

int
atastrategy(struct buf *bp)
{
	int	dev=bp->b_edev, is_wr, s, do_kick;
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_unit_t *u = ac->drive[ATA_DRIVE(dev)];
	ata_ioque_t *q = ac->ioque;
	ata_req_t *r;
	u32_t	base, len, lba, left;
	char	*addr;

        ATADEBUG(1,"atastrategy(%s) %s lba=%lu nsec=%lu flags=%x\n", 
		Dstr(dev), 
		(bp->b_flags&B_READ)?"READ":"WRITE",
		bp->b_blkno, 
		(bp->b_bcount>>DEV_BSHIFT),
		bp->b_flags);

	ata_region_from_dev(dev,&base,&len);
	
	r = (ata_req_t *)kmem_zalloc(sizeof(*r),KM_SLEEP);
	if (!r) return berror(bp,0,ENOMEM);

	r->is_write = (bp->b_flags & B_READ) ? 0 : 1;
	r->reqid    = req_seq++;
	r->drive    = ATA_DRIVE(dev);
	r->addr     = (char *)bp->b_un.b_addr;
	r->bp 	    = bp;

	if (u->is_atapi) {
		u32_t blksz = u->atapi_blksz ? u->atapi_blksz : 2048;
		u32_t bsz512 = blksz >> 9;
		if (bsz512 == 0) bsz512=1;

		r->lba          = base / bsz512 + (u32_t)bp->b_blkno / bsz512;
		r->lba_cur      = r->lba;
		r->nsec         = (u32_t)(bp->b_bcount / blksz);
		r->sectors_left = r->nsec;
		r->cmd		= ATA_CMD_PACKET;
	} else {
		r->lba          = base + (u32_t)bp->b_blkno;
		r->lba_cur      = r->lba;
		r->nsec         = (u32_t)(bp->b_bcount >> 9);
		r->sectors_left = r->nsec;
		r->cmd 		= multicmd(ac,r->is_write,bp->b_blkno,r->nsec);
	}

	ata_pushreq(ac,r);
	return 0;
}

int
ataread(dev_t dev, struct uio *uiop, cred_t *crp)
{
	int	err;

	ATADEBUG(1,"ataread(%s)\n",Dstr(dev));

	if ((err=physck(dev,uiop,B_READ)) != 0) return err;
	return uiophysio(atastrategy, NULL, dev, B_READ, uiop);
}

int
atawrite(dev_t dev, struct uio *uiop, cred_t *crp)
{
	int	err;

	ATADEBUG(1,"atawrite(%s)\n",Dstr(dev));

	if ((err=physck(dev,uiop,B_WRITE)) != 0) return err;
	return uiophysio(atastrategy, NULL, dev, B_WRITE, uiop);
}

int
ataioctl(dev_t dev, int cmd, caddr_t arg, int mode, cred_t *crp, int *rvalp)
{
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_unit_t *u = ac->drive[ATA_DRIVE(dev)];
	int 	fdisk = ATA_PART(dev);
	int	drive = u->drive;
	ata_part_t *fp=&u->fd[fdisk];

	if (cmd != V_VERIFY)	/*** Too noisy ***/
	ATADEBUG(1,"ataioctl(%s,%s)\n",Dstr(dev),Istr(cmd));

	if (!u->present) return ENODEV;

	switch (cmd) {
	case V_CONFIG: 		/* VIOC | 0x01 */
		return 0;

	case V_REMOUNT: /* VIOC | 0x02 */
		if (fp->nsectors > 0 && fp->systid == UNIXOS) 
			fp->vtoc_valid = ata_read_vtoc(dev,fdisk);
		return 0;
	
	case V_GETPARMS: {	/* VIOC | 0x04 */
		struct disk_parms dp;

		/*
		 * We dont have geometry; supply logical CHS compatible 
		 * with 63/16 
		 */
		dp.dp_type   = 1;
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

	case V_FORMAT: 		/* VIOC | 0x05 */
		return 0;

	case V_PDLOC: {		/* VIOC | 0x06 */
		u32_t	vtocloc = VTOC_SEC;

		if (copyout((caddr_t)&vtocloc, arg,sizeof(vtocloc)) != 0)
			return EFAULT;
		return 0;
	    }
	
	case V_RDABS: {		/* VIOC | 0x0A */
        	struct absio ab;
		caddr_t	ptr;

        	if (copyin(arg, (caddr_t)&ab, sizeof(ab)) != 0) return EFAULT;
        	if (ab.abs_buf == 0) return EINVAL;

		if (!(ptr = (caddr_t)kmem_alloc(ATA_SECSIZE,KM_SLEEP))) {
			return ENOMEM;
		}
    		if (ata_getblock(ABSDEV(dev),ab.abs_sec,ptr,ATA_SECSIZE)) {
			kmem_free(ptr,ATA_SECSIZE);
			return EFAULT;
		}
		dumpbuf("RDABS", (u32_t)ab.abs_sec, ptr);

                if (copyout(ptr,ab.abs_buf,ATA_SECSIZE) != 0) {
			kmem_free(ptr,ATA_SECSIZE);
			return EFAULT; 
		}
		kmem_free(ptr,ATA_SECSIZE);
                return 0;
	    }
	
	case V_WRABS: {		/* VIOC | 0x0B */
        	struct absio ab;
		caddr_t	ptr;

        	if (copyin(arg,(caddr_t)&ab,sizeof(ab)) != 0) return EFAULT;
        	if (ab.abs_buf == 0) return EINVAL;

		if (!(ptr = (caddr_t)kmem_alloc(ATA_SECSIZE,KM_SLEEP))) {
			return ENOMEM;
		}
                if (copyin((caddr_t)ab.abs_buf,ptr,ATA_SECSIZE) != 0) { 
			kmem_free(ptr,ATA_SECSIZE);
			return EFAULT; 
		} 
		dumpbuf("WRABS", (u32_t)ab.abs_sec, ptr);

    		if (ata_putblock(ABSDEV(dev),ab.abs_sec,ptr,ATA_SECSIZE) != 0) {
			kmem_free(ptr,ATA_SECSIZE);
			return EIO;
		}
		kmem_free(ptr,ATA_SECSIZE);
                return 0;
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
	int 	fdisk = ATA_PART(dev),
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
	ata_ioque_t *q;
	ata_ctrl_t *ac;
	ata_counters_t *counters;

	ATADEBUG(1,"atainit()\n");

	/*** Hack - we mustn't run before bio subsystem has initialised ***/
	if (bfreelist.av_forw == NULL) binit();

	bzero((caddr_t)&ata_unit[0],sizeof(ata_unit_t)*ATA_MAX_UNITS);
	for (ctrl = 0; ctrl < ATA_MAX_CTRL; ctrl++) {
		ac = &ata_ctrl[ctrl];

		ac->idx = ctrl;
		ac->flags = 0x0;
		ac->multi_set_ok = 0; /* disable MULTIPLE until proven */
		if (!ac->present) continue;

		q = (ata_ioque_t *)kmem_zalloc(sizeof(ata_ioque_t),KM_SLEEP);
		if (!q) return -1;
		counters = (ata_counters_t *)kmem_zalloc(sizeof(ata_counters_t),KM_SLEEP);
		if (!counters) return -1;

		ac->ioque = q;
		ac->counters = counters;

		ac->sel_drive = -1;
		ac->sel_hi4 = 0xff;
		ac->sel_mode = 0;

		ac->drive[ 0 ] = &ata_unit[ATA_UNIT_FROM(ctrl,0)];
		ac->drive[ 1 ] = &ata_unit[ATA_UNIT_FROM(ctrl,1)];

		ac->drive[0]->drive = 0;
		ac->drive[1]->drive = 1;

		ata_attach(ctrl);
		if (AC_HAS_FLAG(ac,ACF_INTR_MODE)) {
			/*
			 * If in Interrupt mode as opposed POLL mode
			 * ACF_INTR_MODE will be set. Force Clear the flag
			 * that is used to cache whether interrupts are on
			 * and then Enable Interrupts
			 */
			AC_CLR_FLAG(ac,ACF_IRQ_ON); 
			ATA_IRQ_ON(ac);
		}
	}
	return 0;
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
	u8_t 	st, ast, err, drvsel;
	u32_t	lba;

	ATADEBUG(1,"ataintr(%d)\n",ipl);

	for (ctrl = 0; ctrl < ATA_MAX_CTRL; ctrl++) {
		ac=&ata_ctrl[ctrl];
		if (ac->irq == ipl && ac->present) break;
	}
	if (ctrl==ATA_MAX_CTRL) {
		return DDI_INTR_UNCLAIMED;
	}

	if (!AC_HAS_FLAG(ac, ACF_INTR_MODE)) {
		BUMP(ac,irq_spurious);
		return DDI_INTR_CLAIMED;
	}	

	q = ac->ioque;
	r = q ? q->cur : NULL;	/* ataintr() */
	if (!r) { 
		BUMP(ac,irq_no_cur);

		ast=inb(ATA_ALTSTATUS_O(ac));
		drvsel=inb(ATA_DRVHD_O(ac));
		if (ast == 0x50) return DDI_INTR_CLAIMED;

		printf("%s: stray IRQ ST=%02x SEL=%02x\n",
			Cstr(ac),st,drvsel);

		printf("q->cur is NULL\n");
		return DDI_INTR_UNCLAIMED;
	}
	drive = r ? r->drive : ac->sel_drive;

	/*** Check for Error ***/
	er = ata_err(ac, &ast, &err);
	if (er) {
		printf("c%dd%d: Error %02x AST %02x ERR %02x\n",
			ctrl,drive,er,ast,err);
		r->err = err;
		r->ast = ast;
	}

	BUMP(ac,irq_seen);
	if (ast & ATA_SR_BSY) {
		BUMP(ac,irq_bsy_skipped);
		return DDI_INTR_CLAIMED;
	}

	st = inb(ATA_STATUS_O(ac)); /* *** This is ataintr() *** */
	BUMP(ac, irq_handled);

	AC_CLR_FLAG(ac,ACF_PENDING_KICK);
	AC_SET_FLAG(ac,ACF_IN_ISR);

	ata_service_irq(ac, drive, st);

	AC_CLR_FLAG(ac,ACF_IN_ISR);
	if (AC_HAS_FLAG(ac,ACF_PENDING_KICK)) {
		AC_CLR_FLAG(ac,ACF_PENDING_KICK);
		ata_kick(ac);
	}
	return DDI_INTR_CLAIMED;
}
