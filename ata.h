/*
 * Minimal ATA header for SVR4 port upper/lower split
 * - No sunddi.h usage
 * - Callers use inb/outb/inw/outw directly (no *_p helpers)
 * - Minor layout matches your last working version
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

extern int ipl, iplmask;
#define ASSERT_NOT_ISR() do { if (ipl > 0) panic("ata userspace op at ipl>0"); } while (0)

#define insw(port,addr,count)	linw(port,addr,count)
#define outsw(port,addr,count)	loutw(port,addr,count)

/* ---------- Controller limits ---------- */
#define ATA_MAX_CTRL   4		
#define ATA_MAX_UNITS	(ATA_MAX_CTRL*2)

#define ATA_MAX_RETRIES 3

struct ata_ctrl;
struct ata_chan;
struct ata_unit;
struct ata_req;
typedef struct ata_ctrl ata_ctrl_t;
typedef struct ata_chan ata_chan_t;
typedef struct ata_unit ata_unit_t;
typedef struct ata_req ata_req_t;

/* ---------- I/O port base + helpers ---------- */
struct ata_ctrl {
	u16_t	io_base;   	/* base for data..status */
	u8_t	irq;       	/* wired IRQ (14/15 typical) */
	u8_t	present;   	/* probed */
} ;

extern ata_ctrl_t ata_ctrl[ATA_MAX_CTRL];

/* Register offsets from io_base */
#define ATA_DATA         0x00
#define ATA_FEAT         0x01
#define ATA_ERROR        0x01
#define ATA_SECTCNT      0x02
#define ATA_SECTNUM      0x03
#define ATA_CYLLOW       0x04
#define ATA_CYLHIGH      0x05
#define ATA_DRIVEHD      0x06
#define ATA_COMMAND      0x07
#define ATA_STATUS       0x07

/* Control block (io_ctl) */
#define ATA_ALTSTATUS    0x00
#define ATA_DEVCTRL      0x00   /* write-only */

#define ATA_CMD_O(c)         (ata_ctrl[(c)].io_base + ATA_COMMAND)
#define ATA_FEAT_O(c)        (ata_ctrl[(c)].io_base + ATA_FEAT)
#define ATA_ERROR_O(c)       (ata_ctrl[(c)].io_base + ATA_ERROR)
#define ATA_STATUS_O(c)      (ata_ctrl[(c)].io_base + ATA_STATUS)
#define ATA_DATA_O(c)        (ata_ctrl[(c)].io_base + ATA_DATA)
#define ATA_SECTCNT_O(c)     (ata_ctrl[(c)].io_base + ATA_SECTCNT)
#define ATA_SECTNUM_O(c)     (ata_ctrl[(c)].io_base + ATA_SECTNUM)
#define ATA_CYLLOW_O(c)      (ata_ctrl[(c)].io_base + ATA_CYLLOW)
#define ATA_CYLHIGH_O(c)     (ata_ctrl[(c)].io_base + ATA_CYLHIGH)
#define ATA_DRVHD_O(c)       (ata_ctrl[(c)].io_base + ATA_DRIVEHD)

#define ATA_ALTSTATUS_O(c)   (ata_ctrl[(c)].io_base+0x206 + ATA_ALTSTATUS)
#define ATA_DEVCTRL_O(c)     (ata_ctrl[(c)].io_base+0x206 + ATA_DEVCTRL)

/* Drive/head bits */
#define ATA_DH(d, lba, h4)   (0xA0|(((d)&1)<<4)|((lba)?0x40:0)|((h4)&0x0F))

/* Status bits */
#define ATA_ST_BSY   		0x80
#define ATA_ST_DRDY  		0x40
#define ATA_ST_DF    		0x20
#define ATA_ST_DSC   		0x10
#define ATA_ST_DRQ   		0x08
#define ATA_ST_ERR   		0x01

/* Devctl */
#define ATA_CTL_SRST 		0x04
#define ATA_CTL_NIEN 		0x02

/* DRVHD bits */
#define ATA_DHFIXED 		0xA0  /* 1,0,1,x, head3..0 */
#define ATA_DHDRV   		0x10
#define ATA_LBA     		0x40

/* ATA/ATAPI commands */
#define ATA_CMD_IDENTIFY        0xEC
#define ATA_CMD_READ_SEC        0x20
#define ATA_CMD_WRITE_SEC       0x30
#define ATA_CMD_READ_SEC_RETRY  0x21
#define ATA_CMD_FLUSH_CACHE     0xE7
#define ATA_CMD_FLUSH_CACHE_EXT 0xEA
#define ATAPI_CMD_PACKET        0xA0
#define ATAPI_CMD_IDENTIFY_PKT  0xA1

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
#define ATA_NPART 16


struct ata_fdisk {
	u32_t	base_lba;
	u32_t	nsectors;
	u8_t	systid;
	int	vtoc_valid;
	struct partition slice[ATA_NPART];
};

/* Request representing a hardware chunk derived from a struct buf */
struct ata_req {
	struct ata_req *next;
	struct buf   *bp;        /* original request */
	int 	is_write;  /* 1=write, 0=read */
	u32_t	lba;       /* device LBA(512B for ATA, 2048B for ATAPI) */
	u32_t	nsec;
	caddr_t	addr;     /* current kernel addr within bp */
	u32_t	nleft;
	caddr_t	kaddr;
	int	done;
	/*** Chunking/progress ***/
	u32_t	lba_cur;
	u32_t	sectors_left;
	u16_t	chunk_left;
	u16_t	xfer_off;
} ;

/* Simple states for interrupt engine */
typedef enum {
	AS_IDLE = 0,
	AS_ATA_DATA,
	AS_ATAPI_SENDPKT,
	AS_ATAPI_DATA,
	AS_ATAPI_SENSE,
	AS_DONE,
	AS_ERROR
} ata_state_t;

struct ata_chan {
	/*** Submission Queue ***/
	ata_req_t *q_head;
	ata_req_t *q_tail;
	/*** Inflight request ***/
	ata_req_t *cur;
	int     busy;
	ata_state_t state;
	/*** IRQ Policy/close gating ***/
	int     use_intr;     /* enable INT path */
	int     closing;
	/*** sync/wakeup for raw/uio and close drain ***/
	int     no_autokick;
	int     sync_done;
	/*** watchdog/timeout ***/
	int	tmo_id;
	int	tmo_ticks;
	/*** staging buffer (lifetime: first open / last close ***/
	caddr_t xfer_buf;
	u32_t	xfer_bufsz;
	/*** open/close ownership ***/
	int	open_count;
} ;

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

struct ata_unit {
	ata_chan_t chan;
	int 	present;             	/* device probed */
	int 	is_atapi;            	/* 1 if packet device */
	int 	lba_ok;              	/* 1 if LBA28 supported */
	u32_t 	nsectors;  		/* total 512B sectors (ATA only) */

	struct	ata_fdisk fd[4];
	int	fdisk_valid;

	char 	model[41];
	int  	ioctl_warned;
	int  	read_only;          	/* 1 if media/device RW locked */
	uchar_t atapi_pdt;
	uchar_t atapi_rmb;
	char	vendor[9];
	char 	product[17];

	u32_t	atapi_blocks;
	u32_t	atapi_blksz;
	u16_t	lbsize;
	u8_t	lbshift;

	u8_t	cdb[12];
	int	ctrl;
	int	driv;

	caddr_t	atapi_bounce;
};

extern ata_unit_t ata_unit[];

#define ATA_PDLOC	29	/* Sector location of PDINFO */

#define ATA_XFER_BUFSZ	(64*1024)	/* 64k */

extern	int atadebug;
#define	ATADEBUG	if (atadebug) printf

#define DDI_INTR_UNCLAIMED	0
#define DDI_INTR_CLAIMED	1
#define	splbio	spl5

/* ---------- DDI entry points (upper half / core) ---------- */
int 	ataopen(dev_t *, int, int, cred_t *);
int 	ataclose(dev_t, int, int, cred_t *);
int 	ataread(dev_t, struct uio *, cred_t *);
int 	atawrite(dev_t, struct uio *, cred_t *);
int 	ataioctl(dev_t, int, caddr_t, int, cred_t *, int *);
int 	atasize(dev_t);
int 	ataintr(int);
void 	ataprint(dev_t, char *);

/* ---------- Low-level (hardware) ---------- */
int 	ata_wait(int, u32_t, u32_t, long);
int 	ata_pio_read(int, int, u32_t, int, caddr_t);
int 	ata_pio_write(int, int, u32_t, int, caddr_t);
int 	ata_identify(int, int, int);
int 	ata_flush_cache(int, int);
int	ata_read_fdisk(int);
int 	ata_should_use_intr(ata_unit_t *, int, u32_t);
void 	ata_region_from_dev(dev_t, u32_t *, u32_t *);
int 	ata_rdwr(int, dev_t, struct uio *);
char	*Cstr(dev_t);

#endif /* _ATA_H */

