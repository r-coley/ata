/*
 * ATA header for SVR4 
 */

#ifndef _ATA_H
#define _ATA_H

#include <sys/types.h>
#include <sys/param.h>
#include <sys/buf.h>
#include <sys/kmem.h>
#include <sys/uio.h>
#include <sys/file.h>
#include <sys/cred.h>
#include <sys/conf.h>
#include <sys/ddi.h>
#include <sys/ipl.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/vtoc.h>
#include <sys/inline.h>
#include <sys/param.h>
#include <sys/fdisk.h>
#include <sys/varargs.h>

#define insw(port,addr,count)	linw(port,addr,count)
#define outsw(port,addr,count)	loutw(port,addr,count)

#define ATA_MAX_XFER_SECTORS 256 /* 128KiB per command */

/* ---------- Controller limits ---------- */
#define ATA_MAX_CTRL   	4		
#define ATA_MAX_DRIVES	2
#define ATA_MAX_UNITS	(ATA_MAX_CTRL*ATA_MAX_DRIVES)

#define ATA_MAX_RETRIES 3
#define ATA_NPART 16

struct ata_ctrl;
struct ata_ioque;
struct ata_unit;
struct ata_req;
struct ata_fdisk;
typedef struct ata_ctrl ata_ctrl_t;
typedef struct ata_ioque ata_ioque_t;
typedef struct ata_unit ata_unit_t;
typedef struct ata_req ata_req_t;
typedef struct ata_fdisk ata_fdisk_t;

typedef struct 
{
	u32_t	polled_reads;
	u32_t	polled_write;
	u32_t	polled_chunks;
	u32_t	polled_sectors;
	u32_t	reads;
	u32_t	writes;
	u32_t	loops;
	u32_t	sectors;
	u32_t	wait_drq_ok;
	u32_t	wait_drq_to;
	u32_t	wait_bsy_to;
	u32_t	err;
	u32_t	irq_turnon;
	u32_t	irq_turnoff;
	u32_t	irq_seen;
	u32_t	irq_handled;
	u32_t	irq_spurious;
	u32_t	irq_no_cur;
	u32_t	irq_drq_service;
	u32_t	irq_eoc;
	u32_t	irq_err;
	u32_t	irq_bsy_skipped;
	u32_t	lost_irq_rescued;
	u32_t	wd_fired;
	u32_t	wd_serviced;
	u32_t	wd_rekicked;
	u32_t	wd_arm;
	u32_t	wd_cancel;
	u32_t	wd_resets;
	u32_t	wd_chunk;
	u32_t	eoc_polled;
	u32_t	softresets;
} ata_counters_t;

/* Simple states for interrupt engine */
typedef enum {
	AS_IDLE = 0,
	AS_PRIMING,
	AS_PRIMED,
	AS_XFER,
	AS_WAIT,
	AS_DONE,
	AS_RESET,
	AS_ERROR
} ata_state_t;

struct ata_fdisk {
	u32_t	base_lba;
	u32_t	nsectors;
	u8_t	systid;
	int	vtoc_valid;
	struct partition slice[ATA_NPART];
} ;

/* ---------- I/O port base + helpers ---------- */
struct ata_ctrl {
	u16_t	io_base;   	/* base for data..status */
	u8_t	irq;       	/* wired IRQ (14/15 typical) */
	u8_t	present;   	/* probed */

	int	idx;
	ata_ioque_t *ioque;
	ata_unit_t *drive[ATA_MAX_DRIVES];

	int	sel_drive;
	u8_t	sel_hi4;
	u8_t	sel_mode;
	
	u8_t	intr_mode;
	u8_t	irq_enabled;

	/*** Counters ***/
	ata_counters_t counters;
} ;

struct ata_ioque {
	/*** Submission Queue ***/
	ata_req_t *q_head;
	ata_req_t *q_tail;
	/*** Inflight request ***/
	ata_req_t *cur;
	int     busy;
	ata_state_t state;
	/*** IRQ Policy/close gating ***/
	int     closing;
	/*** sync/wakeup for raw/uio and close drain ***/
	int     sync_done;
	/*** watchdog/timeout ***/
	int	tmo_id;
	int	tmo_ticks;
	/*** staging buffer (lifetime: first open / last close ***/
	caddr_t xfer_buf;
	u32_t	xfer_bufsz;
	caddr_t	xptr;
	/*** open/close ownership ***/
	int	open_count;
	/*** Stuff ***/
	int	in_isr;
	int	pending_kick;

} ;

/* Request representing a hardware chunk derived from a struct buf */
struct ata_req {
	struct ata_req *next;
	struct buf   *bp;        /* original request */

	dev_t	dev;		/* Original dev, udriv could be deduced */
	int	drive;		/* Drive: 0 master 1 slave */
	u32_t	reqid;

	int	flags;
	int 	is_write;  /* 1=write, 0=read */
	u32_t	lba;       /* device LBA(512B for ATA, 2048B for ATAPI) */
	u32_t	nsec;
	caddr_t	addr;     /* current kernel addr within bp */
	int	done;
	u8_t	dh_cmd;

	/*** Chunking/progress ***/
	u32_t	lba_cur;
	u32_t	sectors_left;
	u16_t	chunk_left;
	u32_t	xfer_off;
	u32_t 	chunk_bytes;

	u8_t	cmd;
	u16_t	atapi_bytes;
	int	use_bounce;
} ;

struct ata_unit {
	int 	present;             	/* device probed */
	int 	is_atapi;            	/* 1 if packet device */
	int	is_cdrom:1;
	int	media_present:1;
	int 	lba_ok;              	/* 1 if LBA28 supported */
	u32_t 	nsectors;  		/* total 512B sectors (ATA only) */

	ata_fdisk_t fd[4];
	int	fdisk_valid;

	char 	model[41];
	int  	ioctl_warned;
	int  	read_only;          	/* 1 if media/device RW locked */
	uchar_t atapi_pdt;
	uchar_t atapi_rmb;
	char	vendor[9];
	char 	product[17];
	u16_t	devtype;

	u32_t	atapi_blocks;
	u32_t	atapi_blksz;
	u16_t	lbsize;
	u8_t	lbshift;

	u8_t	cdb[12];
	caddr_t	atapi_bounce;

	int	drive;
};

#define ATA_RF_NEEDCOPY	0x0001
#define ATA_RF_DONE	0x80000000u

#define SEL_CHS		0
#define SEL_LBA28	1

#define ATA_SECTSIZE	512

/* Register offsets from io_base */
#define ATA_DATA         0x00
#define ATA_FEAT         0x01
#define ATA_ERROR        0x01
#define ATA_SECTCNT      0x02
#define ATA_SECTNUM      0x03
#define ATA_LBA0	 0x03
#define ATA_CYLLOW       0x04
#define ATA_LBA1	 0x04
#define ATA_CYLHIGH      0x05
#define ATA_LBA2	 0x05
#define ATA_DRIVEHD      0x06
#define ATA_COMMAND      0x07
#define ATA_STATUS       0x07

/* Control block (io_ctl) */
#define ATA_ALTSTATUS    0x00
#define ATA_DEVCTRL      0x00   /* write-only */

#define ATA_CMD_O(c)         (c->io_base + ATA_COMMAND)
#define ATA_FEAT_O(c)        (c->io_base + ATA_FEAT)
#define ATA_ERROR_O(c)       (c->io_base + ATA_ERROR)
#define ATA_STATUS_O(c)      (c->io_base + ATA_STATUS)
#define ATA_DATA_O(c)        (c->io_base + ATA_DATA)
#define ATA_SECTCNT_O(c)     (c->io_base + ATA_SECTCNT)
#define ATA_SECTNUM_O(c)     (c->io_base + ATA_SECTNUM)
#define ATA_CYLLOW_O(c)      (c->io_base + ATA_CYLLOW)
#define ATA_CYLHIGH_O(c)     (c->io_base + ATA_CYLHIGH)
#define ATA_DRVHD_O(c)       (c->io_base + ATA_DRIVEHD)
#define ATA_LBA0_O(c)        (c->io_base + ATA_LBA0)
#define ATA_LBA1_O(c)        (c->io_base + ATA_LBA1)
#define ATA_LBA2_O(c)        (c->io_base + ATA_LBA2)

#define ATA_ALTSTATUS_O(c)   (c->io_base+0x206 + ATA_ALTSTATUS)
#define ATA_DEVCTRL_O(c)     (c->io_base+0x206 + ATA_DEVCTRL)


#define U2CTRLNO(X)	((X)->ctrl-&ata_ctrl[0])

/* Status bits */
#define ATA_ST_BSY   		0x80	/* Drive is busy */
#define ATA_ST_DRDY  		0x40	/* Drive is ready for command */
#define ATA_ST_DF    		0x20	/* Drive Fault */
#define ATA_ST_DSC   		0x10	/* Drive seek complete */
#define ATA_ST_DRQ   		0x08	/* Data request RD/WR xfer sector */
#define ATA_ST_ERR   		0x01	/* Error */

/* Devctl */
#define ATA_CTL_SRST 		0x04	/* Software Reset */
#define ATA_CTL_NIEN 		0x02	/* Disable INTRQ */
#define ATA_CTL_IRQEN 		0x00	/* Enable INTRQ */

/* Simple hd.c-style IRQ helpers */
#define ATA_IRQ_ON(c)   do { \
		if (!c->irq_enabled) { \
			outb(ATA_DEVCTRL_O(c), ATA_CTL_IRQEN); \
			c->irq_enabled=1; \
		} \
		BUMP(c,irq_turnon); \
	} while (0)
#define ATA_IRQ_OFF(c,force)  do { \
		if (c->irq_enabled || force) { \
			outb(ATA_DEVCTRL_O(c), ATA_CTL_NIEN); \
			c->irq_enabled=0; \
		} \
		BUMP(c,irq_turnoff); \
	} while (0)

/* DRVHD bits */
#define ATA_DHFIXED 		0xA0  /* 1,0,1,x, head3..0 */
#define ATA_DHDRV   		0x10
#define ATA_DHLBA     		0x40

#define ATA_DH(d,lba,h4)  (ATA_DHFIXED|(((d)&1)<<4)|((lba)?0x40:0)|((h4)&0x0F))

/* ATA/ATAPI commands */
#define ATA_CMD_IDENTIFY        0xEC
#define ATA_CMD_READ_SEC        0x20
#define ATA_CMD_WRITE_SEC       0x30
#define ATA_CMD_READ_SEC_EXT    0x24
#define ATA_CMD_WRITE_SEC_EXT   0x34
#define ATA_CMD_READ_SEC_RETRY  0x21
#define ATA_CMD_FLUSH_CACHE     0xE7
#define ATA_CMD_FLUSH_CACHE_EXT 0xEA
#define ATA_CMD_PACKET          0xA0
#define ATA_CMD_IDENTIFY_PKT    0xA1

/* ATAPI CDB opcodes */
#define CDB_TEST_UNIT_READY     0x00
#define CDB_REQUEST_SENSE       0x03
#define CDB_INQUIRY             0x12
#define CDB_MODE_SENSE_6     	0x1A
#define CDB_MODE_SENSE_10       0x5A
#define CDB_START_STOP_UNIT     0x1B
#define CDB_PREVENT_ALLOW       0x1E
#define CDB_READ_10             0x28
#define CDB_READ_CAPACITY       0x25

/* ---------- Minor layout (USL-style, extended) ----------
 * bits 7..5 = controller (0..3)
 * bit    4  = drive      (0..1)
 * bits 3..0 = partition  (0..15) (we use 0..15 typically)
 */
#define ATA_WHOLE_FDISK_SLICE   0x0F
#define ATA_CTRL(m)       (((getminor(m)) >> 5) & 0x03)
#define ATA_DRIVE(m)      (((getminor(m)) >> 4) & 0x01)
#define ATA_SLICE(m)	  ((getminor(m)) & ATA_WHOLE_FDISK_SLICE)
#define ATA_FDISK(m)      (((getminor(m)) >> 8) & 0x03)

/* Unit helpers: unit = ctrl*2 + drive */
#define ATA_UNIT_FROM(c,d)   	(((c) << 1) | ((d) & 1))
#define ATA_CTRL_FROM_UNIT(u) 	((u) >> 1)
#define ATA_DRIVE_FROM_UNIT(u) 	((u) & 1)
#define ATA_UNIT(m)          	ATA_UNIT_FROM(ATA_CTRL(m), ATA_DRIVE(m))

/* ---------- Per-unit state ---------- */

#define BUMP(C,field)	(C)->counters.field++

/* Well known FDISK partition types */
#define EMPTY		0x00
#define FAT12		0x01	
#define FAT16		0x04	
#define EXTDOS0		0x05
#define NTFS		0x07	
#define DOSDATA0	0x56
#define OTHEROS0	0x62
#define SVR4		0x63	/* UNIX V.x partition */
#define UNUSED2		0x64
#define LINUXSWAP	0x82
#define LINUXNATIVE	0x83
#define BSD386		0xA5
#define OPENBSD		0xA6
#define NETBSD		0xA9
#define SOLARIS		0xBF


#define ATA_PDLOC	29	/* Sector location of PDINFO */

#define ATA_XFER_BUFSZ	(64*1024)	/* 64k */

#define DDI_INTR_UNCLAIMED	0
#define DDI_INTR_CLAIMED	1
#define	splbio	spl5

extern 	ata_ctrl_t ata_ctrl[ATA_MAX_CTRL];
extern	int 	atadebug;
extern	int 	ata_force_polling;
extern 	ata_unit_t ata_unit[];

void 	pio_one_sector(ata_ctrl_t *, u8_t, int);
void 	ata_service_irq(ata_ctrl_t *, u8_t, u8_t);
void 	ata_region_from_dev(dev_t, u32_t *, u32_t *);
char 	*CCstr(ata_ctrl_t *);
char 	*Cstr(dev_t);
void 	ataprint(dev_t, char *);
int 	ataopen(dev_t *, int, int, cred_t *);
int 	ataclose(dev_t, int, int, cred_t *);
int 	atastrategy(struct buf *);
int 	ataread(dev_t, struct uio *, cred_t *);
int 	atawrite(dev_t, struct uio *, cred_t *);
int 	ataioctl(dev_t, int, caddr_t, int, cred_t *, int *);
int 	atasize(dev_t dev);
int 	atainit(void);
int 	ataintr(int);
char 	*Ustr(ata_unit_t *);
void 	ata_region_from_dev(dev_t, u32_t *, u32_t *);
char 	*Cstr(dev_t);
void 	ataprint(dev_t, char *);
int 	ataopen(dev_t *, int, int, cred_t *);
int 	ataclose(dev_t, int, int, cred_t *);
int 	atastrategy(struct buf *);
int 	ataread(dev_t, struct uio *, cred_t *);
int 	atawrite(dev_t, struct uio *, cred_t *);
int 	ataioctl(dev_t, int, caddr_t, int, cred_t *, int *);
int 	atasize(dev_t dev);
int 	atainit(void);
int 	ataintr(int);
char 	*Ustr(ata_unit_t *);

int 	do_ata_strategy(struct buf *);
int 	ata_send_packet(ata_ctrl_t *, u8_t, const u8_t *, int);
void 	ata_udelay(int);
int 	ata_wait(ata_ctrl_t *, u32_t, u32_t, long);
int 	ata_wait_ready(ata_ctrl_t *);
int 	ata_sel(ata_ctrl_t *,int, u32_t);
int 	ata_sel_chs(ata_ctrl_t *,int);
int 	ata_sel_lba28(ata_ctrl_t *, u8_t, u8_t);
char 	*get_sysid(u8_t);
void 	ata_dump_fdisk(ata_ctrl_t *,u8_t);
int 	ata_identify(ata_ctrl_t *, int);
int 	ata_pio_write(ata_ctrl_t *,u8_t, u32_t, int, caddr_t);
int 	ata_pio_read(ata_ctrl_t *,u8_t, u32_t, int, caddr_t);
int 	ata_flush_cache(ata_ctrl_t *,u8_t);
int 	ata_read_fdisk(ata_ctrl_t *,u8_t);
int 	ata_read_vtoc(ata_ctrl_t *, u8_t, int);
void 	ata_copy_model(u16_t *, char *);
int 	ata_read_signature(ata_ctrl_t *, u8_t, u16_t *);
int 	ata_is_atapi_by_sig(u8_t, u8_t);
int 	ata_identify_common(ata_ctrl_t *,u8_t, int);
void 	ata_attach(int);
int 	ata_probe_unit(ata_ctrl_t *,u8_t);
int 	atapi_read_capacity(ata_ctrl_t *ac, u8_t, u32_t *, u32_t *);
int 	atapi_inquiry(ata_ctrl_t *, u8_t,u8_t *, u8_t *, char *, char *);
char 	*atapi_class_name(u8_t, int);
int 	atapi_read10(ata_ctrl_t *, u8_t, u32_t, u16_t,void *);
int 	atapi_mode_sense10(ata_ctrl_t *, u8_t, u8_t, u8_t, void *, u16_t);
int 	atapi_mode_sense6(ata_ctrl_t *, u8_t, u8_t, u8_t, void *, u8_t);
int 	atapi_packet(ata_ctrl_t *, u8_t, u8_t *, int, void *, u32_t, int, u8_t *, int );
int 	do_atapi_strategy(struct buf *);
void 	ata_dump_regs(ata_ctrl_t *, char *);
void 	ata_quiesce_ctrl(ata_ctrl_t *);
void	ata_dump_stats(void);
int 	berror(struct buf *, ata_ioque_t *, int, int);
int 	bok(struct buf *, ata_ioque_t *, int);
void 	ata_softreset_ctrl(ata_ctrl_t *);
void 	ata_softreset_ctrl2(ata_ctrl_t *);

void 	ata_softreset_ctrl(ata_ctrl_t *);
ata_req_t *alloc_request(dev_t, u32_t, u32_t, int, caddr_t);
ata_req_t *ata_q_get(ata_ctrl_t *);
void 	ata_program_next_chunk(ata_ctrl_t *,ata_req_t *,int);
void 	ata_arm_watchdog(ata_ctrl_t *, int);
void 	ata_cancel_watchdog(ata_ctrl_t *);
void 	ata_watchdog(caddr_t);
void 	ata_q_put(ata_ctrl_t *, ata_req_t *);
void 	ata_kick(ata_ctrl_t *);
void 	reset_queue(ata_ioque_t *,int);
void 	ata_finish_current(ata_ctrl_t *, int,int);
void 	ata_start(ata_ctrl_t *);
void 	free_request(ata_req_t *);
void 	ata_delay400(ata_ctrl_t *);
void 	ATADEBUG(int, char *, ... );
int 	ata_err(ata_ctrl_t *, u8_t *, u8_t *);
void 	ata_program_taskfile(ata_ctrl_t *, ata_req_t *);
int	atapi_test_unit_ready(ata_ctrl_t *, u8_t, int *);

#define DEV_UNKNOWN	0x0000
#define DEV_ATA		0x0010
#define DEV_ATAPI	0x0020
#define DEV_PARALLEL	0x0000
#define DEV_SERIAL	0x0001

#endif /* _ATA_H */
