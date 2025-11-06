/*
 * ata_hw.c lower half: identify, waits, PIO read/write, flush
 */

#include "ata.h"
#include <sys/varargs.h>
#define ATA_LOG_SIZE 1024

u32_t	req_seq=0;
extern	int ata_major;

int
berror(struct buf *bp, int resid, int err)
{
	char *str = ISABSDEV(bp->b_edev) ? "<ABSDEV>" : "";

	ATADEBUG(1,"berror(%s)\n",str);
	bp->b_flags |= B_ERROR; 
	bp->b_error = err; 
	bp->b_resid = resid;
	biodone(bp); 
	return 0;
}

int
bok(struct buf *bp, int resid)
{
	char *str = ISABSDEV(bp->b_edev) ? "<ABSDEV>" : "";
	ATADEBUG(1,"bok(%s)\n",str);
	bp->b_flags &= ~B_ERROR; 
	bp->b_resid = resid;
	if (ISABSDEV(bp->b_edev) && !(bp->b_flags & B_READ)) {
		ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(bp->b_edev)];
		u8_t drive = ATA_DRIVE(bp->b_edev);
		bflush(bp->b_edev);
		ata_flush_cache(ac,drive);
	}
	biodone(bp);
	return 0;
}

int
ata_sel(ata_ctrl_t *ac, int drive, u32_t lba)
{
	ata_unit_t *u=ac->drive[drive];
	u8_t ast, erc; 

	if (!u) return EIO;
	ata_err(ac, &ast, &erc);

	ATADEBUG(9,"ata_sel(%d,%lu) ST=%02x ER=%02x: ",drive,lba,ast,erc);
	if (u->lba_ok) {
		u8_t	hi4 = (u8_t)((lba>>24) & 0x0f);
		if (1 || ac->sel_drive != drive || ac->sel_mode != SEL_LBA28 || 
						   ac->sel_hi4 != hi4) {
			ATADEBUG(1,"LBA %02x\n",ATA_DH(drive,1,hi4));
			outb(ATA_DRVHD_O(ac), ATA_DH(drive,1,hi4));
			ata_delay400(ac);
			ac->sel_drive = drive;
			ac->sel_mode  = SEL_LBA28;
			ac->sel_hi4   = hi4;
		}
	} else {
		if (1 || ac->sel_drive != drive || ac->sel_mode != SEL_CHS || 
						   ac->sel_hi4 != 0) {
			ATADEBUG(1,"CHS %02x\n",ATA_DH(drive,0,0));
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
	ata_req_t rb, *r=&rb;

	ATADEBUG(1,"atapi_program_packet(%s)\n",Cstr(ac));
	bzero((caddr_t)r,sizeof(r));
	r->drive	= drive;
	r->cmd		= ATA_CMD_PACKET;
	r->atapi_bytes	= (byte_count ? byte_count : 2048);
	r->lba_cur	= 0;
	r->sectors_left	= 0;
	r->chunk_left	= 0;
	r->is_write	= 0;
	ata_program_taskfile(ac,r);
}

static void
atapi_issue_packet(ata_ctrl_t *ac, int drive, u16_t byte_count)
{
	ATADEBUG(1,"atapi_issue_packet(%s)\n",Cstr(ac));

	/* Select device + 400ns settle */
	ata_sel(ac,drive,0);

	/* FEAT=0, SECCNT=0, Byte Count in CYCLOW/HIGH */
	outb(ATA_FEAT_O(ac),    0x00);
	outb(ATA_SECTCNT_O(ac), 0x00);
	outb(ATA_CYLLOW_O(ac),  (u8_t)(byte_count & 0xFF));
	outb(ATA_CYLHIGH_O(ac), (u8_t)(byte_count >> 8));

	outb(ATA_CMD_O(ac), ATA_CMD_PACKET);
}

void
ata_delay400(ata_ctrl_t *c)
{
	(void)inb(ATA_ALTSTATUS_O(c));	/*Force delay*/
	(void)inb(ATA_ALTSTATUS_O(c));	/*Force delay*/
	(void)inb(ATA_ALTSTATUS_O(c));	/*Force delay*/
	(void)inb(ATA_ALTSTATUS_O(c));	/*Force delay*/
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

	ATADEBUG(1,"ata_send_packet(%s)\n",Cstr(ac));

	/* Select device */
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
	if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_BSY, 200000, 0, 0) != 0) {
		return ETIMEDOUT;
	}
	/* Check for ERR/DWF via altstatus */
	{ u8_t ast, erc; if (ata_err(ac, &ast, &erc)) return EIO; }

	/* Write exactly 12 bytes of CDB (pad to 6 words) */
	for (i = 0; i < 6; ++i) words_cdb[i] = 0;
	for (i = 0; i < cdb_len && i < 12; ++i)
		((u8_t *)words_cdb)[i] = cdb[i];

	outsw(ATA_DATA_O(ac),(u16_t *)words_cdb,6);
	return 0;
}

int
ata_wait(ata_ctrl_t *ac, u8_t must_set, u8_t must_clear, long usec, u8_t *st, u8_t *err)
{
	u8_t s;
	long i;

	for (i = 0; i <= usec; ++i) {
		s = inb(ATA_ALTSTATUS_O(ac));
		if (((s & must_set) == must_set) && ((s & must_clear) == 0)) {
			if (st) *st = s;
			if (err) *err = inb(ATA_ERROR_O(ac));
			return 0;
		}
		drv_usecwait(1);
	}
	if (st) *st = s;
	if (err) *err = inb(ATA_ERROR_O(ac));
	return EIO;
}

int
ata_fdisk(int mode)
{
	ata_ctrl_t *ac = &ata_ctrl[1];
	struct ipart *ipart;
	struct mboot *mboot;
	int	i;
	dev_t	dev=ATA_DEV(1,0);

	if (mode == 0) {
		ata_dump_fdisk(ac,1);
		return;
	}

	mboot = (struct mboot *)kmem_alloc(DEV_BSIZE,KM_SLEEP);
	if (!mboot) return ENOMEM;

        if (ata_getblock(ABSDEV(dev),0,(caddr_t)mboot,DEV_BSIZE) != 0) {
		kmem_free((caddr_t)mboot,DEV_BSIZE);
		return EIO;
	}

	for(i=0; i<4; i++,ipart++) {
		ipart = (struct ipart *)&mboot->parts[i];
		ipart->bootid  = 0;
		ipart->systid  = 0;
		ipart->relsect = 0L;
		ipart->numsect = 0L;

		ipart->beghead = 0;
		ipart->begsect = 0;
		ipart->begcyl  = 0;
		ipart->endhead = 0;
		ipart->endsect = 0;
		ipart->endcyl  = 0;
	}
        if (ata_putblock(ABSDEV(dev),0,(caddr_t)mboot,DEV_BSIZE) != 0) {
		kmem_free((caddr_t)mboot,DEV_BSIZE);
		return EIO;
	}
	kmem_free((caddr_t)mboot,DEV_BSIZE);
}

void
ata_dump_fdisk(ata_ctrl_t *ac, u8_t drive)
{
	ata_unit_t *u=ac->drive[drive];
	int	fdisk, s;

	ATADEBUG(1,"ata_dump_fdisk(%s)\n",Cstr(ac));

	if (!u->present) return;
	if (u->is_atapi) return;

	for(fdisk=0; fdisk<4; fdisk++) {
		ata_part_t *fp=&u->fd[fdisk];

		u32_t 	base   = fp->base_lba;
		u32_t 	nsec   = fp->nsectors;
		int 	valid  = fp->vtoc_valid;
		u8_t	active = fp->active;
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
		printf(" %cPart=%d: Type=%-10s base_lba=%lu size=%lu %s\n",
			active ? '+' : ' ',
			fdisk, systid,
			(ulong_t)base, (ulong_t)nsec,
			valid ? "vtoc=VALID" : "");

		for(s=0;s<ATA_NPART; s++) {
			u32_t ps = fp->slice[s].p_start;
			u32_t sz = fp->slice[s].p_size;
			if (sz == 0) continue;

			printf("   %cs%02d: start=%8lu size=%8lu\n",
				(s==ATA_WHOLE_PART_SLICE)?'*':' ',
				s,(ulong_t)ps,(ulong_t)sz);
		}
	}
}

int
ata_identify(ata_ctrl_t *ac, int drive)
{
	ata_unit_t *u=ac->drive[drive];
	ata_req_t rb, *r=&rb;
	u16_t 	id[256];
	int 	i;

	ATADEBUG(1,"ata_identify(%s)\n",Cstr(ac));

	if (!ac->present) return ENODEV;

	ATA_IRQ_OFF(ac,1);

	bzero((caddr_t)r,sizeof(r));
	r->drive	= drive;
	r->cmd		= (u->devtype & DEV_ATAPI) ? ATA_CMD_IDENTIFY_PKT 
						   : ATA_CMD_IDENTIFY;
	r->lba_cur	= 0;
	r->sectors_left	= 0;
	r->chunk_left	= 0;
	r->is_write	= 0;
	ata_program_taskfile(ac,r);

	if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_BSY, 500000, 0, 0) != 0) {
		/* no ATA device mark not present */
		u->present = 0;
		return ENODEV;
	}

	insw(ATA_DATA_O(ac),id,256);

	/* Extract model string (word-swapped ASCII) */
	strcpy(u->model,getstr(&((char *)id)[54],40,TRUE,TRUE));

	/* LBA28 capacity in words 60 61 */
	u->nsectors = ((u32_t)id[61] << 16) | (u32_t)id[60];
	u->lba_ok   = (id[49] & (1<<9)) ? 1 : 0;
	u->present  = 1;
	u->read_only = 0;
	return 0;
}

int
ata_flush_cache(ata_ctrl_t *ac,u8_t drive)
{
	ata_req_t rb, *r=&rb;

	ATADEBUG(1,"ata_flush_cache(%s)\n",Cstr(ac));

	bzero((caddr_t)r,sizeof(r));
	r->drive	= drive;
	r->is_write	= 0;
	r->cmd		= ATA_CMD_FLUSH_CACHE;
	ata_program_taskfile(ac,r);
	
	if (ata_wait(ac, ATA_SR_DRDY, 
			 ATA_SR_BSY|ATA_SR_DRQ, 2000000, 0, 0) != 0)
		return EIO;
	return 0;
}

void
CopyTbl(ata_part_t *fp,struct ipart *ipart)
{
	fp->active   = ipart->bootid;
	fp->base_lba = (u32_t)ipart->relsect;
	fp->nsectors = (u32_t)ipart->numsect;
	fp->systid   = (int)ipart->systid;
}

int
ata_pdinfo(dev_t dev)
{
	int 	part  = ATA_PART(dev),
		slice = ATA_SLICE(dev),
		ctrl  = ATA_CTRL(dev),
		drive = ATA_DRIVE(dev);
	ata_ctrl_t *ac = &ata_ctrl[ctrl];
	ata_unit_t *u = ac->drive[drive];
	struct mboot *mboot;
	struct ipart *ip;
	struct buf *bp;
	ata_part_t *fp;
	int	i, s, rc;

	ATADEBUG(1,"ata_pdinfo(%s, dev=%x) drive=%d part=%d\n",	
		Dstr(dev),dev,drive,part);

	if (u->is_atapi) return 0;

	mboot = (struct mboot *)kmem_alloc(DEV_BSIZE,KM_SLEEP);
	if (!mboot) return ENOMEM;

        if (ata_getblock(ABSDEV(dev),0,(caddr_t)mboot,DEV_BSIZE) != 0) {
		kmem_free((caddr_t)mboot,DEV_BSIZE);
		return EIO;
	}

	if (mboot->signature != MBB_MAGIC) {
		kmem_free((caddr_t)mboot, DEV_BSIZE);
		fp = &u->fd[ part ];
		ATADEBUG(1,"Part %d: start=0 p_size=%lu\n",
			part,fp->nsectors);
		fp->slice[ATA_WHOLE_PART_SLICE].p_start = 0;
		fp->slice[ATA_WHOLE_PART_SLICE].p_size  = fp->nsectors;
		return 0;
	}

	/*** Now copy the others in sequence ***/
	ip = (struct ipart *)&mboot->parts;
	for(i=0; i<FD_NUMPART; ip++) {
		fp = &u->fd[ i ];
		if (ip->systid == EMPTY) continue;
		CopyTbl(fp,ip);
		if (ip->systid == UNIXOS) {
			fp->slice[ATA_WHOLE_PART_SLICE].p_start = 0;
			fp->slice[ATA_WHOLE_PART_SLICE].p_size  = fp->nsectors;
			if (fp->nsectors > 0) {
				fp->vtoc_valid = ata_read_vtoc(dev, i);
			}
		}
		i++;
	}
	kmem_free((caddr_t)mboot,DEV_BSIZE);
	/*ata_dump_fdisk(ac,drive); */
	return 0;
}

int 
ata_read_vtoc(dev_t dev,int part)
{
	int 	slice = ATA_SLICE(dev),
		ctrl  = ATA_CTRL(dev),
		drive = ATA_DRIVE(dev);
	ata_ctrl_t *ac = &ata_ctrl[ctrl];
	ata_unit_t *u=ac->drive[drive];
	u32_t 	base, lba, off;
	caddr_t k = 0;
	struct pdinfo *pd;
	struct vtoc *v;
	ata_part_t *fp = &u->fd[part];
	int 	s;

	ATADEBUG(1,"ata_read_vtoc(%s dev=%x)\n",Dstr(dev),dev);

	if (u->is_atapi) return 0;

	/* Only attempt on UNIX partitions with a size. */
	if (fp->systid != UNIXOS || fp->nsectors == 0) return 0;

	base = fp->base_lba;

	/* pdinfo + vtoc live in the same sector at (unix_base + VTOC_SEC). */
	lba = base + (u32_t)VTOC_SEC;
	if (!(k = kmem_alloc(DEV_BSIZE, KM_SLEEP))) return 0;

        if (ata_getblock(ABSDEV(dev),lba,(caddr_t)k,DEV_BSIZE) != 0) {
		kmem_free(k, DEV_BSIZE);
		return EIO;
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
	for (s = 0; s < V_NUMPAR; ++s) {
		fp->slice[s].p_start = v->v_part[s].p_start;
		fp->slice[s].p_size  = v->v_part[s].p_size;
	}

	/* Whole-fdisk pseudo-slice: full partition range. */
	fp->slice[ATA_WHOLE_PART_SLICE].p_start = 0;
	fp->slice[ATA_WHOLE_PART_SLICE].p_size  = fp->nsectors;

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
	if (ata_wait(ac, 0, ATA_SR_BSY, 500000L, 0, 0) != 0) return EIO;

	lc = inb(ATA_CYLLOW_O(ac));
	hc = inb(ATA_CYLHIGH_O(ac));

	switch ((hc<<8)|lc)
	{
	case 0x0000: dev = DEV_ATA|DEV_PARALLEL; 	break;
	case 0xC33C: dev = DEV_ATA|DEV_SERIAL; 		break;
	case 0xEB14: dev = DEV_ATAPI|DEV_PARALLEL; 	break;
	case 0x9669: dev = DEV_ATAPI|DEV_SERIAL; 	break;
	default:     dev = DEV_UNKNOWN;			break;
	}
	*type = dev;
	return 0;
}

void
ata_attach(int ctrl)
{
	ata_ctrl_t *ac= &ata_ctrl[ctrl];
	int 	i, drive, rc=-1;

	ATADEBUG(1,"ata_attach(%d)\n",ctrl);

	if (!ac->present) return;

	if (ata_force_polling == 0) {
		AC_SET_FLAG(ac,ACF_INTR_MODE);
	}
	ata_softreset_ctrl(ac);
	for (drive = 0; drive <= 1; drive++) {
		ata_unit_t *u = ac->drive[drive];

		u->pio_multi=1;		/* 1 sector xfers */
		ata_probe_unit(ac,drive);
	}
}

int
ata_probe_unit(ata_ctrl_t *ac, u8_t drive)
{
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

	AC_SET_FLAG(ac, ACF_SYNC_DONE);
	ac->tmo_id    = 0;
	ac->tmo_ticks = drv_usectohz(2000000); /* 2s is sane for PIO */

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
			case 0:  med="CD Empty"; break;
			case 1:  med="CD Inserted"; break;
			default: med="CD ??"; break;
			}
		}
		printf("%s: ATAPI %s, model=\"%s\" %s\n",
                       Cstr(ac),klass,u->model,med);

	} else { /* ATA disk branch */
		char 	*lba28 = u->lba_ok ? "LBA28" : "";
		unsigned long nsec = u->nsectors;

		ulong_t mib = nsec >> 11; 
		ulong_t gib_i = nsec / 2097152UL;
		ulong_t gib_tenths = ((nsec % 2097152UL) * 10UL) / 2097152UL;

		printf("%s: ATA disk, model=\"%s\" %s, %lu sectors (%lu.%u GiB)\n",
			Cstr(ac), u->model, lba28, nsec, gib_i,gib_tenths);

		u->lbsize=512;
	}
	u->lbshift=(u8_t)((u->lbsize >> 9) 
			? (u->lbsize==512 ?0:(u->lbsize==1024 ? 1 : 2)) : 0);

	ata_negotiate_pio_multiple(ac,drive);

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
	atapi_issue_packet(ac, drive, (u16_t)xfer_len);

	if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_BSY, 200000L, 0, 0) != 0) 
		return EIO;

	/* Send 12-byte packet (6 words) */
	for (i = 0; i < 6; i++) {
		u16_t w = ((u16_t)pkt[2*i+1] << 8) | pkt[2*i];
		outw(ATA_DATA_O(ac), w);
	}

	if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_ERR, 500000L, 0, 0) != 0) 	
		return EIO;

	{
		u16_t tmp[4];
		insw(ATA_DATA_O(ac),tmp,4);
		for(i=0;i<4;i++) {
			buf[(i<<1)+0] = (u8_t)(tmp[i] & 0xFF);
			buf[(i<<1)+1] = (u8_t)(tmp[i] >> 8);
		}
	}
 	(void)ata_wait(ac, 0, ATA_SR_BSY | ATA_SR_DRQ, 500000L, &ast, 0);

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
	atapi_issue_packet(ac, drive, (u16_t)xfer_len);

	/* Wait for DRQ to send the packet */
	if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_BSY, 200000L, 0, 0) != 0) 
		return EIO;

	/* Write the 12-byte packet as 6 words */
	outsw(ATA_DATA_O(ac), (u16_t *)pkt, 6);

	/* Now the device will transfer data; poll for DRQ then read */
	if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_ERR, 500000L, 0, 0) != 0) 
		return EIO;

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
	(void)ata_wait(ac, 0, ATA_SR_BSY | ATA_SR_DRQ, 500000L, &ast, 0); 

	/* Parse PDT and strings */
	pdt = buf[0] & 0x1F;
	rmb = (buf[1] >> 7) & 1;

	if (out_pdt) *out_pdt = pdt;
	if (out_rmb) *out_rmb = rmb;

	u->atapi_pdt = pdt;
	u->atapi_rmb = rmb;

	if (vendor)
		strcpy(u->vendor,getstr((char *)&buf[8],8,FALSE,TRUE));

	if (product)
		strcpy(u->product,getstr((char *)&buf[16],16,FALSE,TRUE));

	/* Map "ZIP" style PDT 0x00 + RMB=1 is removable disk (ZIP/MO) */
	if (pdt == 0x00 && rmb) {
		/* you can special-case print as "ZIP/Removable Disk" */
	}
	return 0;
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
		/* Wait for BSY to clear */
		ata_wait(ac, 0, ATA_SR_BSY, 1000000L, &st, 0);

		/* If DRQ dropped, command is finishing; read final STATUS */
		if ((st & ATA_SR_DRQ) == 0) {
			if (st & (ATA_SR_ERR | ATA_SR_DWF)) {
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

		if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_BSY, 100000, &st, 0) == 0) {
			u16_t want = (u16_t)((sense_len < 18) ? sense_len : 18);
			u16_t wds  = (u16_t)((int)(want+1) >> 1);
			for(w=0; w < wds; w++)
				*(((u16_t *)sense) + w) = inw(ATA_DATA_O(ac));
			(void)inb(ATA_ALTSTATUS_O(ac));
		
		}
	}

	return (rc != 0) ? rc : EIO;
}

void
ata_quiesce_ctrl(ata_ctrl_t *ac)
{
	int	pass;
	u8_t 	ast, st;
	u16_t	words, bc, chunk, scratch[256];

	ATADEBUG(9,"ata_quiesce_ctrl(%s)\n",Cstr(ac));
	ata_wait(ac, 0, ATA_SR_BSY, 500000, 0, 0);

	for(pass=0; pass<2; pass++) {
		ast = inb(ATA_ALTSTATUS_O(ac));
		if (ast & ATA_SR_BSY) break;
		if (!(ast & ATA_SR_DRQ)) break;

		bc = (inb(ATA_CYLHIGH_O(ac)) << 8) |	
		      inb(ATA_CYLLOW_O(ac));
		if (bc == 0) bc=65536;

		words = bc>>1;
		while (words) {
			u16_t chunk = (words > 256) ? 256 : words;
			insw(ATA_DATA_O(ac),scratch,chunk);
			words -= chunk;
		}

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
		ata_ctrl_t *ac= &ata_ctrl[ctrl];
		ata_unit_t *u0 = ac->drive[0];
		ata_unit_t *u1 = ac->drive[1];

		if (!ac->present) continue;
			
		printf("ATA%d: intrmode=%d flag=%08x present(%d,%d) turnon=%lu turnoff=%lu\n",
			ctrl, AC_HAS_FLAG(ac,ACF_INTR_MODE),ac->flags,
			u0 ? u0->present : 0,
			u1 ? u1->present : 0,
			ac->counters->irq_turnon,
			ac->counters->irq_turnoff);
 		printf("      IRQ: seen=%lu handled=%lu spurious=%lu nocur=%lu bsy=%lu drq_service=%lu\n",
			ac->counters->irq_seen, 
			ac->counters->irq_handled, 
			ac->counters->irq_spurious,
			ac->counters->irq_no_cur,
			ac->counters->irq_bsy_skipped,
			ac->counters->irq_drq_service);
 		printf("      eoc=%lu eoc_polled=%lu lost_irq_rescued=%lu softresets=%lu\n",
			ac->counters->irq_eoc, 
			ac->counters->eoc_polled,
			ac->counters->lost_irq_rescued,
			ac->counters->softresets);
 		printf("      WD: arm=%lu cancel=%lu fired=%lu service=%lu rekicked=%lu chunk=%ld\n",
			ac->counters->wd_arm,
			ac->counters->wd_cancel,
			ac->counters->wd_fired,
			ac->counters->wd_serviced, 
			ac->counters->wd_rekicked,
			ac->counters->wd_chunk);
	}
}

void
ata_softreset_ctrl(ata_ctrl_t *ac)
{
	int	was_enabled = AC_HAS_FLAG(ac,ACF_IRQ_ON);

	ATADEBUG(1,"ata_softreset_ctrl(%d)\n",ac->idx);
	BUMP(ac,softresets);
	/*ATA_IRQ_OFF(ac,1);*/
	AC_CLR_FLAG(ac,ACF_IRQ_ON); /* Clear the flag */
	outb(ATA_DEVCTRL_O(ac), ATA_CTL_SRST | ATA_CTL_NIEN);
	drv_usecwait(5);
	outb(ATA_DEVCTRL_O(ac), ATA_CTL_NIEN); /* deassert SRST */
	drv_usecwait(5);
	(void)inb(ATA_ALTSTATUS_O(ac));

	if (was_enabled) {
		printf("Was enabled so re-enabling\n");
		ATA_IRQ_ON(ac);
	}
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

int
ata_enable_pio_multiple(ata_ctrl_t *ac, u8_t drive, u8_t multi)
{
	u8_t	ast, err;

	if (!multi || multi <= 1) return 0;

	ata_sel(ac,drive,0);

	outb(ATA_SECTCNT_O(ac),multi);
	outb(ATA_CMD_O(ac),ATA_CMD_SET_MULTI);
	ata_delay400(ac);

	if (ata_wait(ac, 0, ATA_SR_BSY, 1000000, &ast, &err) != 0) return -1;
	if (ast & (ATA_SR_ERR|ATA_SR_DWF))  return -1;

	return 0;
}

/* Negotiate a valid multi-sector count: try 16/8/4/2 (or 8/4/2) based on policy. */
void
ata_negotiate_pio_multiple(ata_ctrl_t *ac, u8_t drive)
{
    u8_t target = (ATA_USE_MAX_MULTIPLE ? 16 : 8);
    u8_t n;
    if (target > 16) target = 16;
    if (target < 2)  target = 2;

    for (n = target; n >= 2; n >>= 1) {
	if (ata_enable_pio_multiple(ac,drive,n) == 0) {
            ac->pio_multi = n;
            printf("%s: PIO multiple negotiated = %u\n", 
			Cstr(ac), ac->pio_multi);
            return;
        }
    }
    ac->pio_multi = 1;
    ATADEBUG(1, "%s: PIO multiple not supported, using single-sector\n",
		Cstr(ac));
}

int
ata_err(ata_ctrl_t *ac, u8_t *ast, u8_t *err)
{
	u8_t	x, y;

	x = inb(ATA_ALTSTATUS_O(ac));
	*ast = (x & 0xff);
	if ((x & (ATA_SR_BSY|ATA_SR_DRQ)) == 0) {
		y = inb(ATA_ERROR_O(ac));
		if (err) *err=y;
		return (u8_t)(x & (ATA_SR_ERR | ATA_SR_DWF)) ? EIO : 0;
	}
	return 0;
}

void
ata_region_from_dev(dev_t dev, u32_t *out_base, u32_t *out_len)
{
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_unit_t *u = &ata_unit[ATA_UNIT(dev)];
	int	part  = ATA_PART(dev);
	int	slice = ATA_SLICE(dev);
	u32_t	base=0, len=0, bsz512;
	ata_part_t *fp = &u->fd[part];

	ATADEBUG(1,"ata_region_from_dev(%s base=%lu, start=%lu)\n",
		Dstr(dev), (u32_t)fp->base_lba, 
		(u32_t)fp->slice[slice].p_start);

	if (u->is_atapi) {
        	bsz512 = (u->lbsize >> 9) ? (u->lbsize >> 9) : 1;
        	base = 0;
        	len  = u->atapi_blocks * bsz512;
	} else {
		if (ISABSDEV(dev)) {
			base = 0;
			len  = u->nsectors;
		} else {
			base = fp->base_lba;
			len  = fp->nsectors;

			if (fp->systid == UNIXOS) {
				if (slice != 0 && 
				    slice != ATA_WHOLE_PART_SLICE) {
					base += fp->slice[slice].p_start-1;
					len    = fp->slice[slice].p_size;
				}
			}
		}
	}
	ATADEBUG(5,"region_from_dev: %s part_base=%lu slice_start=%lu final_base=%lu\n",
		Dstr(dev),fp->base_lba,fp->slice[slice].p_start,base);

	*out_base = base;
	*out_len  = len;
	return;
}

void
reset_queue(ata_ctrl_t *ac,int hard)
{
	ata_ioque_t *q=ac->ioque;

	ATADEBUG(2,"reset_queue()\n");
	if (ac->tmo_id) {
    		untimeout(ac->tmo_id);
    		ac->tmo_id = 0;
	}
	/* Soft reset of the channel engine; do NOT free xfer_buf here. */
	q->cur       = 0;
	q->state     = AS_IDLE;
	AC_CLR_FLAG(ac, ACF_BUSY);
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

char *
Istr(int cmd)
{
	switch (cmd) {
	case V_CONFIG:	 return "V_CONFIG";
	case V_REMOUNT:  return "V_REMOUNT";
	case V_GETPARMS: return "V_GETPARMS";
	case V_FORMAT: 	 return "V_FORMAT";
	case V_PDLOC: 	 return "V_PDLOC";
	case V_RDABS: 	 return "V_RDABS";
	case V_WRABS:	 return "V_WRABS";
	case V_VERIFY:	 return "V_VERIFY";
	default:	 return "V_default";
	}
}

char *
Cstr(ata_ctrl_t *ac) 
{
static	char	buf[20];
	int	d=ac->sel_drive;
	int	c=ac->idx;
	ata_unit_t *u = ac->drive[d];
	dev_t	dev=ATA_DEV(c,d);
	char 	*suffix=ISABSDEV(dev) ? " <ABSDEV>" : "";

	sprintf(buf,"c%dd%d%s",c,d,suffix);
	return buf;
}

char *
Dstr(dev_t dev)
{
static	char	buf[50];
	char 	*suffix=ISABSDEV(dev) ? " <ABSDEV>" : "";
	int 	ctrl = ATA_CTRL(dev), 
		unit = ATA_UNIT(dev), 
		driv = ATA_DRIVE(dev), 
		fdisk = ATA_PART(dev),
		slice = ATA_SLICE(dev);
	sprintf(buf,"c%dd%dp%ds%d%s",ctrl,driv,fdisk,slice,suffix);
	return buf;
}

char *
getstr(char *ptr, int len, int swap, int blanks)
{
static	char	buf[50];
	int	i;
	
	for(i=0;i<len;i++) {
		buf[i]=ptr[i];
		if (swap && i%2) {
			buf[i]   = ptr[i-1];
			buf[i-1] = ptr[i];
		}
	}
	if (blanks) {
		i=len;
		while (i > 0 && buf[i-1] == ' ') i--;
		buf[i] = 0;
	}
	return buf;
} 

void
ata_rescue()
{
	ata_ctrl_t *ac= &ata_ctrl[1];

	ata_rescue(ac);
}

void
ata_rescueit(ata_ctrl_t *ac)
{
	int	s;
	ata_ioque_t *q;

	if (!ac) return;

	s = splbio();
	q = ac->ioque;
	if (!q || !q->cur) { splx(s); return; }

	if (ac->tmo_id) {
		untimeout(ac->tmo_id);
		ac->tmo_id = 0;
	}
	splx(s);

	ata_finish_current(ac,EIO,97);
	need_kick(ac);
}

void
ATADEBUG(int x, char *fmt, ...) 
{
	if (atadebug >= x) 
		xprintf(fmt,(u32_t)((u32_t *)&fmt+1));
}

int 	
ata_getblock(dev_t dev, daddr_t blkno, caddr_t buf, u32_t count)
{
	int	rc;
	struct buf *bp; 

	ATADEBUG(1,"ata_getblock(%x,%lu,%x,%lu)\n",dev,blkno,buf,count);

	if (!(bp = geteblk())) return EIO;

	bp->b_dev    = dev;
	bp->b_edev   = dev;
	bp->b_error  = 0;
	bp->b_resid  = 0;
	bp->b_flags  = B_READ | B_BUSY;
	bp->b_blkno  = blkno;
	bp->b_bcount = count;
	/* bp->b_reqcnt = */ 		/* Req Count */

	atastrategy(bp);
	iowait(bp);

	rc = (bp->b_flags & B_ERROR) ? EIO : 0;
	bcopy(bp->b_un.b_addr,buf,count);
	dumpbuf("GETBLOCK", bp->b_blkno, buf);
	brelse(bp);

	return rc;
}

int 	
ata_putblock(dev_t dev, daddr_t blkno, caddr_t buf, u32_t count)
{
	int rc;
	struct buf *bp; 

	ATADEBUG(1,"ata_putblock(%x,%lu,%x,%lu)\n",dev,blkno,buf,count);

	if (!(bp = geteblk())) return EIO;

	bp->b_dev    = dev;
	bp->b_edev   = dev;
	bp->b_error  = 0;
	bp->b_resid  = 0;
	bp->b_flags  = B_WRITE | B_BUSY;
	bp->b_blkno  = blkno;
	bp->b_bcount = count;

	bcopy(buf, bp->b_un.b_addr, count);
	dumpbuf("PUTBLOCK", bp->b_blkno, bp->b_un.b_addr);

	atastrategy(bp);
	iowait(bp);
	rc = (bp->b_flags & B_ERROR) ? EIO : 0;
	brelse(bp);
	return rc;
}
