/*
 * ata_queue.c 
 */

#include "ata.h"

void 	ata_q_put(ata_unit_t *, ata_req_t *);
ata_req_t *ata_q_get(ata_unit_t *);
void 	ata_finish_current(ata_unit_t *, int );
void 	ata_start(ata_unit_t *);
void 	ata_watchdog(caddr_t arg);
void 	ata_cancel_watchdog(ata_unit_t *u);
void 	ata_arm_watchdog(ata_unit_t *u, int ticks);

ata_req_t *
alloc_request(u32_t lba,u32_t nsec,int is_write,caddr_t buf)
{
	ata_req_t *r;

	r = (ata_req_t *)kmem_alloc(sizeof(*r),KM_SLEEP);
	if (!r) return 0;

	r->next         = (ata_req_t *)0;
	r->bp           = (struct buf *)0;  /* raw/uio path */
	r->is_write     = is_write;
	r->lba          = lba;
	r->nsec         = nsec;
	r->addr         = buf;
	r->nleft        = nsec * 512;
	r->kaddr        = 0;
	r->done         = 0;

	r->lba_cur      = lba;
	r->sectors_left = nsec;
	r->chunk_left   = 0;
	r->xfer_off     = 0;
	return r;
}

void 
ata_program_next_chunk(ata_unit_t *u)
{
	ata_chan_t *ch = &u->chan;
	ata_req_t  *r;
	u16_t 	cnt;
	u8_t 	sectcnt;
	int 	s, i;

	s = splbio();
	r = ch->cur;
	if (!r) { 
		splx(s); 
		return; 
	}
	ATADEBUG("ata_program_next_chunk(%s) blk=%lu count=%lu\n",
		r->is_write?"Write":"Read",r->lba_cur,r->chunk_left);

	cnt     = (r->sectors_left > 256) ? 256 : (u16_t)r->sectors_left;
	sectcnt = (cnt == 256) ? 0 : (u8_t)cnt;

	r->chunk_left = cnt;
	ch->xfer_buf  = r->addr;

	ata_wait_ready(u);
	outb(ATA_DRVHD_O(u->ctrl), ATA_DH(u->driv,1,(r->lba_cur>>24) & 0x0F));
	(void)inb(ATA_ALTSTATUS_O(u->ctrl));
	outb(ATA_SECTCNT_O(u->ctrl), sectcnt);
	outb(ATA_SECTNUM_O(u->ctrl),  (u8_t)(r->lba_cur & 0xFF));
	outb(ATA_CYLLOW_O(u->ctrl),   (u8_t)((r->lba_cur >> 8)  & 0xFF));
	outb(ATA_CYLHIGH_O(u->ctrl),  (u8_t)((r->lba_cur >> 16) & 0xFF));

	if (ch->use_intr) {
		outb(ATA_DEVCTRL_O(u->ctrl), 0x00);
		ATADEBUG("DEVCTRL <- 00 (use_intr=1)\n");
	} else {
		outb(ATA_DEVCTRL_O(u->ctrl), ATA_CTL_NIEN);
		ATADEBUG("DEVCTRL <- 02 (use_intr=0)\n");
	}

	outb(ATA_CMD_O(u->ctrl), r->is_write ? ATA_CMD_WRITE_SEC 
					     : ATA_CMD_READ_SEC);

	if (ch->use_intr) ata_arm_watchdog(u, 2*HZ);
	splx(s);

	if (r->is_write) {
		u8_t	st;

		for(i=0; i<100000; i++) {
			st = inb(ATA_ALTSTATUS_O(u->ctrl));
			if ((st & (ATA_ST_BSY | ATA_ST_DRQ)) == ATA_ST_DRQ)
				break;
		}
		st=inb(ATA_STATUS_O(u->ctrl));
		if (st & (ATA_ST_ERR|ATA_ST_DF)) {
			ata_finish_current(u,EIO);
			return;
		}
		if (st & ATA_ST_DRQ) {
			ata_service_irq(u,st);
		}
	}

}

void 
ata_arm_watchdog(ata_unit_t *u, int ticks)
{
	ata_chan_t *ch = &u->chan;

	if (ticks <= 0)
		ticks = (ch->tmo_ticks ? ch->tmo_ticks : 2*HZ);
	if (ch->tmo_id) {
		untimeout(ch->tmo_id);
		ch->tmo_id = 0;
	}
	ch->tmo_id = timeout(ata_watchdog, (caddr_t)u, ticks);
	ATADEBUG("ata_arm_watchdog(ctrl=%d|%d)\n", u->ctrl, ticks);
}

void ata_cancel_watchdog(ata_unit_t *u)
{
	ata_chan_t *ch = &u->chan;

	if (ch->tmo_id) {
		untimeout(ch->tmo_id);
		ch->tmo_id = 0;
		ATADEBUG("ata_cancel_watchdog(ctrl=%d)\n", u->ctrl);
	}
}

void
ata_watchdog(caddr_t arg)
{
        ata_unit_t *u = (ata_unit_t *)arg;
	ata_chan_t *ch = &u->chan;

        /* consume this timeout instance */
        ch->tmo_id = 0;

        if (!ch->busy || ch->cur == 0)
                return;

        ata_finish_current(u, EIO);   /* cancels (no-op), wakes, kicks next */
}

void
ata_q_put(ata_unit_t *u, ata_req_t *r)
{
	ata_chan_t *ch=&u->chan;
	int	s;

	s=splbio();
        r->next = (ata_req_t *)0;
        if (ch->q_tail)
                ch->q_tail->next = r;
        else
                ch->q_head = r;
        ch->q_tail = r;
	splx(s);	
}

ata_req_t *
ata_q_get(ata_unit_t *u)
{
	ata_chan_t *ch=&u->chan;
        ata_req_t *r;
	int	s;

	s=splbio();
        r=ch->q_head;
        if (r) {
                ch->q_head = r->next;
                if (!ch->q_head) ch->q_tail = (ata_req_t *)0;
                r->next = (ata_req_t *)0;
        }
	splx(s);
        return r;
}

void
ata_kick(ata_unit_t *u)
{
	ata_chan_t *ch=&u->chan;
	int 	s = splbio();

	if (ch->busy || ch->cur) {
		splx(s);
		return;
	}

	splx(s);
        if (ch->q_head)
                ata_start(u);

}

void
reset_chan(ata_chan_t *ch,int hard)
{
	if (ch->tmo_id) {
    		untimeout(ch->tmo_id);
    		ch->tmo_id = 0;
	}
	/* Soft reset of the channel engine; do NOT free xfer_buf here. */
	ch->cur       = 0;
	ch->busy      = 0;
	ch->state     = AS_IDLE;
}

void 
ata_finish_current(ata_unit_t *u, int err)
{
	ata_chan_t *ch = &u->chan;
	ata_req_t  *r;
	buf_t      *bp = NULL;
	int 	s;

	s = splbio();

	ata_cancel_watchdog(u);

	r  = ch->cur;
	if (r) bp = r->bp;

	ch->sync_done = 1;
	wakeup((caddr_t)&ch->sync_done);

	if (r) {
		r->done = 1;
		wakeup((caddr_t)&r->done);
	}

	if (bp) {
		if (err) {
			bp->b_flags |= B_ERROR;
			bp->b_error  = (err > 0) ? err : EIO;
		}
		bp->b_resid = r ? r->nleft : bp->b_bcount;
		biodone(bp);
	}

	ch->cur  = NULL;
	ch->busy = 0;

	if (r) kmem_free((caddr_t)r, sizeof(*r));

	if (ch->q_head != NULL) ata_start(u);

	splx(s);
}

void
ata_start(ata_unit_t *u)
{
        int         ctrl=u->ctrl, driv=u->driv;
	ata_chan_t *ch = &u->chan;
        ata_req_t  *r;
        int         s;
        u8_t        cnt8;


	s=splbio();
	if (ch->busy || ch->cur || ch->closing) {
		splx(s);
		return;
	}

	r=ata_q_get(u);
        if (!r) {
		splx(s);
		return;
	}
        ATADEBUG("ata_start(%s)\n",r->is_write?"Write":"Read");
	if (!r->addr) {
		ATADEBUG("BUG r->addr=NULL\n");
		ata_finish_current(u,EFAULT);
		splx(s);
		return;
	}

        /* pop from queue and mark active */
        ch->cur       = r;
        ch->busy      = 1;
	splx(s);

	r->lba_cur	= r->lba;
	r->sectors_left	= r->nsec;
	r->chunk_left  	= 0;
	r->nleft  	= r->nsec * 512;
	ch->xfer_buf  	= r->addr;

	ata_program_next_chunk(u);

	ATADEBUG("ata_start: lba=%lu nsec=%lu use_intr=%d buf=%x is_write=%d\n",
		r->lba,r->nsec,ch->use_intr,ch->xfer_buf,r->is_write);
}
