/*
 * ata_xfer.c lower half: identify, waits, PIO read/write, flush
 */

#include "ata.h"

int
ata_pio_read(ata_ctrl_t *ac, ata_req_t *r)
{
	int	rc;

	ATADEBUG(1,"ata_pio_read(ctrl=%s,lba=%d,nsec=%d,drive=%d)\n",
		Cstr(ac),r->lba_cur,r->nsec,r->drive);

	if (ata_wait(ac, ATA_SR_BSY, 0, 500000, 0, 0) != 0) { 
		ATADEBUG(1,"ata_pio_read: controller busy\n");
		return EBUSY;
	}

	r->cmd = multicmd(ac,r->is_write,r->lba_cur,r->nsec);
	ata_program_taskfile(ac,r);

	rc = ata_data_phase_service(ac,r);
	
	ata_finish_current(ac,rc,7);
	return rc;
}

int
ata_pio_write(ata_ctrl_t *ac, ata_req_t *r)
{
	int	rc;

	ATADEBUG(1,"ata_pio_write(ctrl=%s,lba=%d,nsec=%d,drive=%d)\n",
		Cstr(ac),r->lba_cur,r->nsec,r->drive);

	if (ata_wait(ac, ATA_SR_BSY, 0, 500000, 0, 0) != 0) { 
		ATADEBUG(1,"ata_pio_write: controller busy\n");
		return EBUSY;
	}

	r->cmd = multicmd(ac,r->is_write,r->lba_cur,r->nsec);
	ata_program_taskfile(ac,r);

	rc = ata_data_phase_service(ac,r);
	
	ata_finish_current(ac,rc,0);
	return rc;
}

int
pio_one_sector(ata_ctrl_t *ac, ata_req_t *r)
{
	u16_t 	*p16 = (u16_t *)r->xptr;
	int	i;
	
	if (r->is_write) {
		ATADEBUG(2,"pio_one_sector: WRITE lba=%lu addr=%08x\n",
			(u32_t)r->lba_cur, p16);

		for(i=0; i<(ATA_SECSIZE/2); i++)
			outw(ATA_DATA_O(ac), p16[i]);
	} else {
		ATADEBUG(2,"pio_one_sector: READ lba=%lu addr=%08x\n",
			(u32_t)r->lba_cur, p16);

		for(i=0; i<(ATA_SECSIZE/2); i++)
			p16[i] = inw(ATA_DATA_O(ac));
	}
	r->xptr     += ATA_SECSIZE;
	r->xfer_off += ATA_SECSIZE;
	if (r->chunk_left >= 0)   r->chunk_left--;
	if (r->sectors_left >= 0) r->sectors_left--;
	ATADEBUG(3,"pio_one_sector done: xfer_off=%08x chunk_left=%d sectors_left=%d\n", r->xfer_off,r->chunk_left,r->sectors_left);
	return 0;
}

void
ata_service_irq(ata_ctrl_t *ac, u8_t drive, u8_t st)
{
	ata_ioque_t *q = ac ? ac->ioque : 0;
	ata_req_t   *r = q ? q->cur : 0;
	int	i;
	u8_t ast, err;

	if (!r) {
		BUMP(ac, irq_no_cur);
		return;
	}

	ata_err(ac,&ast,&err);
	/*printf("ata_service_irq: ST=%02x ER=%02x DRQ=%d BSY=%d reqid=%d lba=%ld flags=%x\n",
		ast,err,!!(ast&ATA_SR_DRQ),!!(ast&ATA_SR_BSY),
		r->reqid,r->lba_cur,ac->flags);		*/

	/* ---- (1) Error Handling ---- */
	if (st & (ATA_SR_ERR | ATA_SR_DWF)) {
		u8_t er = inb(ATA_ERROR_O(ac));
		r->ast = st;
		r->err = er;
		ata_finish_current(ac,EIO,95); 
		return;
	}

	/* ---- (2) Data phase: DRQ asserted transfer one sector ---- */
	if (st & ATA_SR_DRQ) {
		if (ata_data_phase_service(ac,r) < 0) {
			ata_finish_current(ac, EIO, 88);
			return;
		}
		if (r->chunk_left) return;

		if (!r->is_write && (r->flags & ATA_RF_NEEDCOPY) &&
		    q->xfer_buf && r->chunk_bytes) {
			caddr_t dst = (caddr_t)((u8_t *)r->addr + (r->xfer_off - r->chunk_bytes));
			if (valid_usr_range((addr_t)dst, r->chunk_bytes)) {
				bcopy((caddr_t)q->xfer_buf,dst,(size_t)(r->chunk_bytes));
			} else {
				r->err = EFAULT;
			}
			r->flags &= ~ATA_RF_NEEDCOPY;
		}
		
		/* No more sectors? we're done */
		if (r->sectors_left == 0) {
			ata_finish_current(ac, EOK, 8);
			return;
		}

		/* Otherwise queue next block */
		ata_program_next_chunk(ac,r,HZ/8);
		return;
	}	

	/* ---- (3) Final completion drive idle, no error ---- */
	if (!(st & ATA_SR_BSY) &&
	    !(st & ATA_SR_DRQ)) {
		if (r->sectors_left == 0) {
			printf("Calling finish as !ATA_SR_BSY ATA_SR_DRQ ATA_SR_ERR ST=%02x\n",st);
			ata_finish_current(ac, EOK, 8);
			return;
		}
	}

	ata_arm_watchdog(ac,HZ/8);
}

void
ata_program_taskfile(ata_ctrl_t *ac, ata_req_t *r)
{
	u32_t lba    = r->lba_cur;
	u8_t  drive  = (r->drive & 0x1);
	u8_t  cmd    = r->cmd;
	u16_t todo   = (r->nsec == 0) ? 256 : ((r->nsec>256) ? 256 : r->nsec);
	u8_t  sc     = (todo == 256) ? 0 : todo;
	u8_t 	ast, err, dh;
	int	er;

	ata_wait(ac,0,ATA_SR_BSY,500000,0,0);
	ata_sel(ac, drive, lba);
	er=ata_err(ac,&ast,&err);

	switch (cmd) {
	case ATA_CMD_READ_SEC:
	case ATA_CMD_READ_SEC_EXT:
	case ATA_CMD_READ_MULTI:
		outb(ATA_SECTCNT_O(ac), sc);
		outb(ATA_LBA0_O(ac),    (u8_t)(lba      ));
		outb(ATA_LBA1_O(ac),    (u8_t)(lba >>  8));
		outb(ATA_LBA2_O(ac),    (u8_t)(lba >> 16));
		outb(ATA_CMD_O(ac), cmd);
		break;

	case ATA_CMD_WRITE_SEC:
	case ATA_CMD_WRITE_SEC_EXT:
	case ATA_CMD_WRITE_MULTI:
		outb(ATA_SECTCNT_O(ac), sc);
		outb(ATA_LBA0_O(ac),    (u8_t)(lba      ));
		outb(ATA_LBA1_O(ac),    (u8_t)(lba >>  8));
		outb(ATA_LBA2_O(ac),    (u8_t)(lba >> 16));
		outb(ATA_CMD_O(ac), cmd);
		break;

	case ATA_CMD_IDENTIFY:
	case ATA_CMD_IDENTIFY_PKT:
		outb(ATA_SECTCNT_O(ac), 0);
		outb(ATA_LBA0_O(ac),    0);
		outb(ATA_LBA1_O(ac),    0);
		outb(ATA_LBA2_O(ac),    0);
		outb(ATA_CMD_O(ac), cmd);
		break;

	case ATA_CMD_PACKET:
		outb(ATA_FEAT_O(ac),    0x00);
		outb(ATA_SECTCNT_O(ac), 0x00);
		outb(ATA_LBA0_O(ac),    0x00);
		outb(ATA_LBA1_O(ac),    (u8_t)(r->atapi_bytes      & 0xFF));
		outb(ATA_LBA2_O(ac),    (u8_t)((r->atapi_bytes>>8) & 0xFF));
		outb(ATA_CMD_O(ac), cmd);
		break;

	case ATA_CMD_FLUSH_CACHE:
		outb(ATA_CMD_O(ac), cmd);
		break;

	default:
		outb(ATA_SECTCNT_O(ac), 0);
		outb(ATA_LBA0_O(ac),    0);
		outb(ATA_LBA1_O(ac),    0);
		outb(ATA_LBA2_O(ac),    0);
		outb(ATA_CMD_O(ac), cmd);
		break;
	}
	ata_delay400(ac); 
	if (AC_HAS_FLAG(ac,ACF_INTR_MODE)) {
		drv_usecwait(20);
		if (cmd == ATA_CMD_WRITE_SEC ||
		    cmd == ATA_CMD_WRITE_SEC_EXT ||
		    cmd == ATA_CMD_WRITE_MULTI) {
			if (ata_wait(ac,ATA_SR_DRQ,ATA_SR_BSY,200000,0,0) == 0)
				ata_prime_write(ac,r);
			else
				printf("%s: DRQ not ready for write ST=%02x\n",
					Cstr(ac),inb(ATA_ALTSTATUS_O(ac)));
		}
	}
}

int 
ata_program_next_chunk(ata_ctrl_t *ac, ata_req_t *r,int arm_ticks)
{
	if (r->cmd == ATA_CMD_PACKET)
		return ata_issue_packet(ac,r,arm_ticks);
	else
		return ata_issue_pio_rw(ac,r,arm_ticks);
}

int
ata_issue_packet(ata_ctrl_t *ac,ata_req_t *r,int arm_ticks)
{
	ata_unit_t *u = ac->drive[r->drive];
	u8_t	cdb[12], sense[18];
	u16_t	nblks;
	u32_t	blksz, xfer;
	int	dir, rc;

	if (r->sectors_left == 0) return 0;

	blksz = u->atapi_blksz ? u->atapi_blksz : 2048;

	nblks = (r->sectors_left > 0xFFFFU) ? 0xFFFFU : (u16_t)r->sectors_left;

	bzero((caddr_t)cdb, sizeof(cdb));
	cdb[0] = r->is_write ? CDB_WRITE_10 : CDB_READ_10;
	
	cdb[2] = (u8_t)((r->lba_cur >> 24) & 0xff);
	cdb[3] = (u8_t)((r->lba_cur >> 16) & 0xff);
	cdb[4] = (u8_t)((r->lba_cur >>  8) & 0xff);
	cdb[5] = (u8_t)((r->lba_cur >>  0) & 0xff);

	cdb[7] = (u8_t)((nblks >> 8) & 0xff);
	cdb[8] = (u8_t)((nblks >> 0) & 0xff);

	xfer = (u32_t)nblks * blksz;

	dir = r->is_write ? 0 : 1;

	rc = atapi_packet(ac, r->drive, cdb, sizeof(cdb),(void *)r->addr, 
			  xfer, dir, sense,sizeof(sense));
	if (rc == 0) {
		r->lba_cur      += nblks;
		r->addr         += xfer;
		r->sectors_left -= nblks;

		r->xfer_off    += xfer;
		r->chunk_bytes = xfer;
		r->flags       &= ~ATA_RF_NEEDCOPY;
	} else {
		/* Decode sense here */
	}
	return rc;
}

int 
ata_issue_pio_rw(ata_ctrl_t *ac,ata_req_t *r,int arm_ticks)
{
	u32_t 	n;
	size_t 	bytes;
	ata_ioque_t *q = ac->ioque;
	ata_unit_t  *u = ac->drive[r->drive];
	u8_t	ast;
	int	s, er, multi_ok;
	caddr_t	user_ptr;

	ATADEBUG(2,"ata_program_next_chunk(Reqid=%ld)\n",r ? r->reqid : 0);
	if (!r) return;

	if (AC_HAS_FLAG(ac,ACF_INTR_MODE)) {
		n = (r->sectors_left > 256U) ? 256U : r->sectors_left;
		if (n == 0) return;

		/* Cap sectors to bounce-buffer capacity in interrupt mode */
		if (q->xfer_buf) {
			u32_t maxsecs = (q->xfer_bufsz >> 9);
			if ((u32_t)n > maxsecs) n = (int)maxsecs;
		}
	} else {
		n = r->sectors_left;
		if (n > (u32_t)u->pio_multi) n = (u32_t)u->pio_multi;
		if (n == 0) return;
		n=1;
	}

	bytes = (size_t)n << 9; /* * 512U */

	r->lba_cur 	= r->lba + (r->xfer_off >> 9);
	r->nsec 	= (u16_t)n;
#if 1
	r->chunk_bytes  = (u32_t)bytes;
	r->chunk_left   = (u16_t)n;
#else
	r->chunk_left   = (u16_t)n<<9;
	r->chunk_bytes  = (u32_t)r->chunk_left;
#endif
	r->cmd          = multicmd(ac, r->is_write,r->lba_cur,n);
	r->flags       &= ~ATA_RF_NEEDCOPY;

	if (r->is_write) {
		/* Write: prefer bounce buffer for IRQ path; copy from user if valid */
		if (q->xfer_buf && valid_usr_range((addr_t)r->addr, bytes)) {
			bcopy((caddr_t)r->addr + r->xfer_off, q->xfer_buf, bytes);
			r->xptr = q->xfer_buf;
		} else {
			/* kernel buffers only */
			r->xptr = (caddr_t)r->addr + r->xfer_off;
		}
	} else {
		/* Read: receive into bounce buffer when available; copy back later if user VA */
		if (q->xfer_buf) {
			r->xptr = q->xfer_buf;
			if (valid_usr_range((addr_t)r->addr, bytes))
				r->flags |= ATA_RF_NEEDCOPY;
		} else {
			r->xptr = (caddr_t)r->addr + r->xfer_off;
		}
	}

	ATADEBUG(5,"%s: ata_program_next_chunk(%s) blk=%lu count=%lu\n",
		Cstr(ac),r->is_write?"Write":"Read",r->lba_cur,n);

	s=splbio();
	q->state = AS_XFER;
	AC_SET_FLAG(ac,ACF_BUSY);
	q->cur  = r;
	splx(s);

	ata_program_taskfile(ac, r);

	/* reset DRQ wait budget for this chunk */
	r->await_drq_ticks = HZ * 2;

	if (arm_ticks) ata_arm_watchdog(ac,arm_ticks);

	if (!AC_HAS_FLAG(ac,ACF_INTR_MODE)) need_kick(ac);
}

void 
ata_finish_current(ata_ctrl_t *ac, int err,int place)
{
	ata_ioque_t *que;
	ata_req_t  *r;
	buf_t      *bp = NULL;
	int 	s;
	size_t bytes_done;
	u32_t 	resid;

	ATADEBUG(2,"ata_finish_current(err=%d place=%d)\n",err,place);
	if (!ac) {
		printf("ata_finish_current() ac NULL\n");
		return;
	}
	ata_cancel_watchdog(ac);

	que = ac->ioque;
	if (!que) {
		printf("ata_finish_current() que NULL\n");
	}

	s = splbio();
	r  = que ? que->cur : NULL;
	if (!r) {
		ATADEBUG(9,"ata_finish(reqid: None)\n");
		if (que) { 
			que->last_err = err;
			que->state = AS_IDLE;
			AC_CLR_FLAG(ac, ACF_BUSY);
			AC_SET_FLAG(ac, ACF_SYNC_DONE); 
			wakeup((caddr_t)ac->ioque); 
		}
		splx(s);
		return;
	}

	ATADEBUG(9,"ata_finish(reqid: %lu)\n",r->reqid);
	r->err = err;
	if (r->flags & ATA_RF_DONE) { splx(s); return; }
	r->flags |= ATA_RF_DONE;
	splx(s);

	bp = r->bp;
	bytes_done = r->xfer_off;

	if (bp) {
		if (bytes_done > bp->b_bcount) bytes_done = bp->b_bcount;
		resid = bp->b_bcount - bytes_done;
	} else {
		bytes_done=0;
		resid=0;
	}

	/*** Xfer done - now copy the buffer if needed ***/
	if (!r->err && 
	    !r->is_write && 
	    (r->flags & ATA_RF_NEEDCOPY) && 
	    que->xfer_buf && r->chunk_bytes) { 
		caddr_t dst = (caddr_t)((char *)r->addr + (r->xfer_off - r->chunk_bytes));
		if (valid_usr_range((addr_t)dst, r->chunk_bytes))
			bcopy(que->xfer_buf, dst, r->chunk_bytes);
		else
			r->err = EFAULT;

		r->flags &= ~ATA_RF_NEEDCOPY;
	}

	s=splbio();
	AC_CLR_FLAG(ac,ACF_BUSY);
	AC_SET_FLAG(ac, ACF_SYNC_DONE);
	que->cur = NULL;
	que->state = AS_IDLE;
	que->last_err = r->err;
	splx(s);

	if (bp) {
		if (err) berror(bp,resid,EIO);
		else     bok(bp,resid);
	}

	wakeup((caddr_t)ac->ioque);

	if (r) kmem_free(r,sizeof(*r));
	ata_kick(ac);
}

void
ata_poll_engine(ata_ctrl_t *ac)
{
	ata_ioque_t *q = ac ? ac->ioque : 0;
	ata_req_t   *r = q ? q->cur : 0;
	u8_t	ast;

	if (!r) return;

	if (!AC_HAS_FLAG(ac, ACF_BUSY)) return;

	AC_SET_FLAG(ac, ACF_POLL_RUNNING);

	/* drive the state machine until we either need to yield or finish */
	do {
		/* tiny yeild to avoid a hot loop if the device is stll busy */
		drv_usecwait(2);

		ata_err(ac,&ast,0);
		ATADEBUG(5,"poll: ST=%02x\n",ast);
		if (ast & ATA_SR_BSY) continue;

		if (ast & ATA_SR_ERR) {
			ata_finish_current(ac,EIO,97);
			break;
		}

		if (ast & ATA_SR_DRQ) {
			if (ata_data_phase_service(ac,r) < 0) {
				ata_finish_current(ac, EIO, 97);
				break;
			}

			if (r->chunk_left == 0) {
				if (r->sectors_left > 0) {
					ata_program_next_chunk(ac,r,HZ/8);
				} else {
					ata_finish_current(ac,0,0);
					break;
				}
			}
			continue;
		}

		/* No BSY, no DRQ, no ERR: command may have completed with
		 * no data */
		if (r->sectors_left == 0 && r->chunk_left == 0) {
			ata_finish_current(ac,0,0);
			break;
		}
	} while (AC_HAS_FLAG(ac, ACF_PENDING_KICK));

	if (AC_HAS_FLAG(ac, ACF_PENDING_KICK) &&
	    !AC_HAS_FLAG(ac, ACF_INTR_MODE)) {
		AC_CLR_FLAG(ac, ACF_PENDING_KICK);
		ata_kick(ac);
	}

	AC_CLR_FLAG(ac,ACF_POLL_RUNNING);
}

int
ata_data_phase_service(ata_ctrl_t *ac, ata_req_t *r)
{
	u8_t 	ast;
	int 	rc=0, count, i;

	r->xptr = r->addr + r->xfer_off;

	/* Wait briefly for BSY to clear and DRQ to assert */
	if (ata_wait(ac,ATA_SR_DRQ|ATA_SR_DRDY, ATA_SR_BSY, 10000, &ast, 0)) {
		ATADEBUG(2,"ata_data_phase: DRQ wait timeout %02x\n",ast);
		r->err = ast;
		return -1;
	}

	drv_usecwait(10);

	if (AC_HAS_FLAG(ac,ACF_INTR_MODE))
		count = 1;
	else
		count = (r->chunk_left != 0) ? r->chunk_left : 1;

	for (i = 0; i < count; i++) {
		if (pio_one_sector(ac, r) != 0) {
			ATADEBUG(1,"ata_data_phase: pio_one_sector failed\n");
			rc = -1;
			break;
		}

		/* derive from bytes already transferred to avoid drift */
		r->lba_cur = r->lba + (r->xfer_off >> 9);

		if (!AC_HAS_FLAG(ac,ACF_INTR_MODE)) {
			if (ata_wait(ac, ATA_SR_DRDY, 
				ATA_SR_BSY|ATA_SR_DRQ, 10000, &ast, 0)) {
				ATADEBUG(2,"ata_data_phase: DRQ clear timeout %02x\n", ast);
				rc = -1;
				break;
			}
		}
	}
	return rc;
}

void
ata_prime_write(ata_ctrl_t *ac, ata_req_t *r)
{
	ata_ioque_t *q = ac->ioque;
	u8_t	ast, err;
	u8_t	drive = r->drive & 1;
	int	er;

	if (ata_wait(ac,ATA_SR_DRQ,ATA_SR_BSY,1000000,&ast,&err) != 0) {
		if (!(ast & ATA_SR_DRQ)) {
			printf("ata_prime_write(): wait DRQ fail ST=%02x ER=%02x\n",ast,err);
			return;
		}
	}

	if (!q) printf("que is null\n");
	if (!r->xptr) printf("r->xptr is null\n");
	
	if (pio_one_sector(ac,r) != 0) {
		/* Error */
	}
	return;
}

int
ata_pushreq(ata_ctrl_t *ac,ata_req_t *r)
{
	ata_ioque_t *que = ac->ioque;
	int	s;

	s = splbio();
	AC_CLR_FLAG(ac, ACF_SYNC_DONE);
	ata_q_put(ac, r);

	if (AC_HAS_FLAG(ac,ACF_INTR_MODE) || 
	    !AC_HAS_FLAG(ac,ACF_POLL_RUNNING))
		need_kick(ac);

	if (AC_HAS_FLAG(ac, ACF_INTR_MODE)) {
		while (!AC_HAS_FLAG(ac, ACF_SYNC_DONE)) {
			sleep((caddr_t)ac->ioque, PRIBIO);
			/* printf("Woken2 ac-ioque flags=%08x\n",ac->flags);*/
		}
		AC_CLR_FLAG(ac,ACF_SYNC_DONE);
		splx(s);
		return que->last_err;
	}
	splx(s);

	/* Polled mode: drive engine until done, never sleep. */
	for (;;) {
		if (AC_HAS_FLAG(ac, ACF_SYNC_DONE)) break;
		if (!AC_HAS_FLAG(ac, ACF_POLL_RUNNING)) ata_kick(ac);
		drv_usecwait(10);
	}
	AC_CLR_FLAG(ac,ACF_SYNC_DONE);
	return que->last_err;
}

int
multicmd(ata_ctrl_t *ac, int is_write, u32_t lba, u32_t nsec)
{
	int	use_ext = 0; /* (lba > 0xffffffff) && ac->lba48_ok;*/
	int	multi_ok = (nsec>1) && (ac->pio_multi>1) && ac->multi_set_ok;

	if (is_write) {
		if (use_ext) return multi_ok ? ATA_CMD_WRITE_MULTI_EXT
					     : ATA_CMD_WRITE_SEC_EXT;
		return multi_ok ? ATA_CMD_WRITE_MULTI
				: ATA_CMD_WRITE_SEC;
	} else {
		if (use_ext) return multi_ok ? ATA_CMD_READ_MULTI_EXT
					     : ATA_CMD_READ_SEC_EXT;
		return multi_ok ? ATA_CMD_READ_MULTI
				: ATA_CMD_READ_SEC;
	}
}

void
dumpbuf(char *msg, u32_t lba, char *buf)
{
	u8_t *p = (u8_t *)buf;

	ATADEBUG(1,"**%10s: lba=%6u [%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]\n",
		msg,lba,
		p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7],p[8],
		p[9],p[10],p[11],p[12],p[13],p[510],p[511]);
}
