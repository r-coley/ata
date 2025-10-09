/*
 * ata_queue.c 
 */

#include "ata.h"

void
ata_program_taskfile(ata_ctrl_t *ac, ata_req_t *r)
{
	ata_ioque_t *que = ac->ioque;
	u32_t lba    = r->lba_cur;
	u8_t  drive  = r->drive & 1;
	u8_t  cmd    = r->cmd;
	u16_t todo   = (r->nsec == 0) ? 256 : ((r->nsec>256) ? 256 : r->nsec);
	u8_t  sc     = (todo == 256) ? 0 : todo;
	u8_t 	ast, err;
	int	er;

	switch (cmd) {
	case ATA_CMD_READ_SEC:
		ata_sel(ac, drive, lba);
		er=ata_err(ac,&ast,&err);
		ATADEBUG(9,"ata_program_taskfile(): DH=%02x SC=%02x L=%02x %02x %02x ST=%02x\n",
		drive,sc,(u8_t)(lba),(u8_t)(lba>>8),(u8_t)(lba>>16),ast);
		outb(ATA_SECTCNT_O(ac), sc);
		outb(ATA_LBA0_O(ac),    (u8_t)(lba      ));
		outb(ATA_LBA1_O(ac),    (u8_t)(lba >>  8));
		outb(ATA_LBA2_O(ac),    (u8_t)(lba >> 16));
		outb(ATA_CMD_O(ac), cmd);
		ata_delay400(ac);
		return;

	case ATA_CMD_WRITE_SEC:
		ata_sel(ac, drive, lba);
		er=ata_err(ac,&ast,&err);
		ATADEBUG(9,"ata_program_taskfile(): DH=%02x SC=%02x L=%02x %02x %02x ST=%02x\n",
		drive,sc,(u8_t)(lba),(u8_t)(lba>>8),(u8_t)(lba>>16),ast);
		outb(ATA_SECTCNT_O(ac), sc);
		outb(ATA_LBA0_O(ac),    (u8_t)(lba      ));
		outb(ATA_LBA1_O(ac),    (u8_t)(lba >>  8));
		outb(ATA_LBA2_O(ac),    (u8_t)(lba >> 16));
		outb(ATA_CMD_O(ac), cmd);

		{ /*** PRIME WRITES ***/
		if (ata_wait(ac,ATA_ST_DRQ,ATA_ST_BSY,1000000) != 0) {
			er=ata_err(ac,&ast,&err);
			ATADEBUG(9,"ST=%02x ERR=%02x\n",ast,err);
			if (!(ast & ATA_ST_DRQ)) {
				printf("WRITE: DRQ never asserted\n");
				return;
			}
		}

		pio_one_sector(ac,drive,1);
		
		que->xptr += 512;
		r->xfer_off += 512;
		if (r->chunk_left)   r->chunk_left--;
		if (r->sectors_left) r->sectors_left--;
		}
		ata_delay400(ac);
		return;

	case ATA_CMD_IDENTIFY:
	case ATA_CMD_IDENTIFY_PKT:
		ata_sel(ac, drive, 0);
		outb(ATA_SECTCNT_O(ac), 0);
		outb(ATA_LBA0_O(ac),    0);
		outb(ATA_LBA1_O(ac),    0);
		outb(ATA_LBA2_O(ac),    0);
		outb(ATA_CMD_O(ac), cmd);
		ata_delay400(ac);
		return;

	case ATA_CMD_PACKET:
		ata_sel(ac, drive, 0);
		outb(ATA_FEAT_O(ac),    0x00);
		outb(ATA_SECTCNT_O(ac), 0x00);
		outb(ATA_LBA0_O(ac),    0x00);
		outb(ATA_LBA1_O(ac),    (u8_t)(r->atapi_bytes      & 0xFF));
		outb(ATA_LBA2_O(ac),    (u8_t)((r->atapi_bytes>>8) & 0xFF));
		outb(ATA_CMD_O(ac), cmd);
		ata_delay400(ac);
		return;

	default:
		ata_sel(ac, drive, 0);
		outb(ATA_SECTCNT_O(ac), 0);
		outb(ATA_LBA0_O(ac),    0);
		outb(ATA_LBA1_O(ac),    0);
		outb(ATA_LBA2_O(ac),    0);
		outb(ATA_CMD_O(ac), cmd);
		ata_delay400(ac);
		return;
	}
}

void 
ata_program_next_chunk(ata_ctrl_t *ac, ata_req_t *r,int arm_ticks)
{
	ata_ioque_t *que = ac->ioque;
	u32_t	n;
	u8_t	ast;
	int	er;
	caddr_t	user_ptr;
	size_t 	bytes;

	ATADEBUG(1,"ata_program_next_chunk()\n");
	if (!r) return;

	n = (r->sectors_left > 256U) ? 256U : r->sectors_left;
	if (n == 0) return;

	bytes = (size_t)n << 9; /* * 512U */
	user_ptr = (caddr_t)((u8_t *)r->addr + r->xfer_off);

	if (r->is_write) {
		if (que->xfer_buf) {
			bcopy(user_ptr,que->xfer_buf,bytes);
			que->xptr = que->xfer_buf;
		} else {
			que->xptr = user_ptr;
		}
	} else {
		if (que && que->xfer_buf) {
			que->xptr = que->xfer_buf;
			r->flags |= ATA_RF_NEEDCOPY;
		} else {
			que->xptr = user_ptr;
			r->flags &= ~ATA_RF_NEEDCOPY;
		}
	}

	que->state = AS_PRIMED;

	r->chunk_left   = (u16_t)n;
	r->chunk_bytes  = (u32_t)bytes;
	r->lba_cur 	= r->lba + (r->xfer_off >> 9);
	r->nsec	      	= (u16_t)n;
	r->cmd	      	= r->is_write ? ATA_CMD_WRITE_SEC : ATA_CMD_READ_SEC;

	ATADEBUG(5,"%s: ata_program_next_chunk(%s) blk=%lu count=%lu\n",
		CCstr(ac),r->is_write?"Write":"Read",r->lba_cur,n);

	ata_program_taskfile(ac, r);
	if (arm_ticks) ata_arm_watchdog(ac,arm_ticks);
}

void 
ata_arm_watchdog(ata_ctrl_t *ac, int ticks)
{
	ata_ioque_t *que = ac->ioque;

	if (ticks <= 0)
		ticks = (que->tmo_ticks ? que->tmo_ticks : 2*HZ);
	if (que->tmo_id) {
		untimeout(que->tmo_id);
		que->tmo_id = 0;
	}
	BUMP(ac,wd_arm);
	que->tmo_id = timeout(ata_watchdog, (caddr_t)ac, ticks);
	ATADEBUG(5,"%s: ata_arm_watchdog(%d)\n", CCstr(ac), ticks);
}

void 
ata_cancel_watchdog(ata_ctrl_t *ac)
{
	ata_ioque_t *que = ac->ioque;

	if (que->tmo_id) {
		BUMP(ac,wd_cancel);
		untimeout(que->tmo_id);
		que->tmo_id = 0;
		ATADEBUG(5,"%s: ata_cancel_watchdog()\n", CCstr(ac));
	}
}

void
ata_watchdog(caddr_t arg)
{
	ata_ctrl_t *ac = (ata_ctrl_t *)arg;
	ata_ioque_t *que = ac->ioque;
	ata_req_t  *r  = que ? que->cur : NULL;
	int	s, REKICK_LIMIT=8, er;
	u8_t 	ast, st, err;

	if (!r) { que->tmo_id=0; return; }

	BUMP(ac,wd_fired);

	er = ata_err(ac,&ast,&err); 	/*** Check for Error ***/

	if (r->chunk_left > 0 && r->sectors_left > 0) {
		BUMP(ac,wd_serviced);
		ata_arm_watchdog(ac,HZ/10);
		return;	
	}
	if (r->chunk_left == 0 && r->sectors_left > 0) {
		BUMP(ac,wd_rekicked);
		ata_program_next_chunk(ac, r, HZ/8);
		return;
	}
	if (r->chunk_left == 0 && r->sectors_left == 0) {
		BUMP(ac,eoc_polled);
		ata_finish_current(ac, 0, 12);
        	return;
	}
}

void
ata_q_put(ata_ctrl_t *ac, ata_req_t *r)
{
	ata_ioque_t *que=ac->ioque;
	int	s;

	ATADEBUG(1,"ata_q_put()\n");
	s=splbio();
        r->next = (ata_req_t *)0;
        if (que->q_tail)
                que->q_tail->next = r;
        else
                que->q_head = r;
        que->q_tail = r;
	splx(s);	
}

ata_req_t *
ata_q_get(ata_ctrl_t *ac)
{
	ata_ioque_t *que=ac->ioque;
        ata_req_t *r;
	int	s;

	ATADEBUG(1,"ata_q_get()\n");
	s=splbio();
        r=que ? que->q_head : NULL;
        if (r) {
                que->q_head = r->next;
                if (!que->q_head) que->q_tail = (ata_req_t *)0;
                r->next = (ata_req_t *)0;
        }
	splx(s);
        return r;
}

void
ata_kick(ata_ctrl_t *ac)
{
	ata_ioque_t *que=ac->ioque;
	int 	s = splbio();

	ATADEBUG(1,"ata_kick()\n");
	if (que->busy || que->cur) {
		splx(s);
		return;
	}

	splx(s);
        if (que->q_head)
                ata_start(ac);
}

void
reset_queue(ata_ioque_t *que,int hard)
{
	if (que->tmo_id) {
    		untimeout(que->tmo_id);
    		que->tmo_id = 0;
	}
	/* Soft reset of the channel engine; do NOT free xfer_buf here. */
	que->cur       = 0;
	que->busy      = 0;
	que->state     = AS_IDLE;
}

void 
ata_finish_current(ata_ctrl_t *ac, int err,int place)
{
	ata_ioque_t *que = ac->ioque;
	ata_req_t  *r;
	buf_t      *bp = NULL;
	int 	s;
	size_t bytes_done;
	u32_t 	resid;

	ata_cancel_watchdog(ac);

	s = splbio();
	r  = que ? que->cur : NULL;
	if (!r) {
		ATADEBUG(9,"ata_finish(reqid: None)\n");
		if (que) { 
			que->busy  = 0;
			que->sync_done = 1; 
			wakeup((caddr_t)&que->sync_done); 
		}
		splx(s);
		return;
	}

	ATADEBUG(9,"ata_finish(reqid: %lu)\n",r->reqid);
	if (r->flags & ATA_RF_DONE) { splx(s); return; }
	r->flags |= ATA_RF_DONE;

	bp = r->bp;

	bytes_done = r->xfer_off;
	if (bp && bytes_done > bp->b_bcount) bytes_done = bp->b_bcount;
	resid = bp->b_bcount - bytes_done;

	if (!err && !r->is_write && 
	    (r->flags & ATA_RF_NEEDCOPY) && que->xfer_buf && r->chunk_bytes) {
		caddr_t dst = (caddr_t)((char *)r->addr + (r->xfer_off - r->chunk_bytes));
		bcopy(que->xfer_buf, dst, r->chunk_bytes);
		r->flags &= ~ATA_RF_NEEDCOPY;
	}

	if (bp) {
		if (err) berror(bp,que,resid,EIO);
		else     bok(bp,que,resid);
	}

	que->busy  = 0;
	que->cur = NULL;

	if (r) kmem_free(r,sizeof(*r));

	que->sync_done = 1;
	wakeup((caddr_t)&que->sync_done);
	splx(s);
	ata_kick(ac);
}


void
ata_start(ata_ctrl_t *ac)
{
	ata_ioque_t *que = ac->ioque;
        ata_req_t  *r;
        int         s;
	u8_t	drive;

	ATADEBUG(1,"ata_start()\n");
	if (!ac->intr_mode) return;

	s=splbio();
	if (que->busy || que->cur || que->closing) { splx(s); return; }

        /* pop from queue */
	r=ata_q_get(ac);
        if (!r) { 
		splx(s);
		return;
	}
	ATADEBUG(9,"atastart reqid %lu\n",r->reqid);

        que->cur   = r;
	que->state = AS_PRIMING;

	ATA_IRQ_ON(ac);
	r->lba_cur 	= r->lba;
	r->sectors_left	= r->nsec;
	r->chunk_left	= 0;
	r->xfer_off	= 0;
	drive		= r->drive & 1;
	
	ATADEBUG(5,"lba=%lu lba_cur=%lu addr=%lx nsec=%lu secleft=%lu chunkleft=%lu xoff=%lu xlen=%lu nsec=%lu\n",
			r->lba,
			r->lba_cur,
			r->addr,
			r->nsec,
			r->sectors_left,
			r->chunk_left,
			r->xfer_off,
			r->chunk_bytes);
	ata_program_next_chunk(ac,r,HZ/8);
	que->busy  = 1;
	que->state = AS_PRIMED;
	splx(s);
	return;
}

void
need_kick(ata_ctrl_t *c)
{
	ata_ioque_t *que = c->ioque;
	
	if (que->in_isr) {
		que->pending_kick=1;
		return;
	}
	ata_kick(c);
}
