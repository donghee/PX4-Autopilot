// This file is meant to tackle the dependencies found in PX4
// that have not been implemented in the VxWorks SDK yet.

#pragma once

// for VxWorks, DKMs are not supported, so we use the
#ifdef NONE
#undef NONE
#endif

#ifdef OK
//#undef OK
#endif

#ifdef ERROR
#undef ERROR
#endif

// for VxWorks DKMs are not supported, so we use the
#ifdef _WRS_KERNEL
#ifndef STDIN_FILENO
#  define STDIN_FILENO 0
#endif
#ifndef STDOUT_FILENO
#  define STDOUT_FILENO 1
#endif
#ifndef STDERR_FILENO
#  define STDERR_FILENO 2
#endif
#endif

/* IOCTL types: */
#define VX_IOC_VOID  0x20000000
#define VX_IOC_OUT   0x40000000
#define VX_IOC_IN    0x80000000
#define VX_IOC_INOUT (VX_IOC_IN | VX_IOC_OUT)
#define VX_IOC_READ(a) (((0x40 << 24) & (a)) == VX_IOC_OUT)
#define VX_IOC_WRITE(a) (((0x80 << 24) & (a)) == VX_IOC_IN)
#define VX_IOC_RW(a) (((0xc0 << 24) & (a)) == VX_IOC_INOUT)

/* IOCTL size: */
#define VX_IOCPARM_MASK    0xfff      /* Parameter length 12 bits */
#define VX_IOCPARM_LEN(r)  (VX_IOCPARM_MASK & ((r) >> 16))

/* IOCTL flags. */
#define VX_IOCF_R    0x00000000   /* read. */
#define VX_IOCF_W    0x10000000   /* write bit. */

/* IOCTL groups: */
#define VX_IOC_GROUP(c)  (((c) & 0x0000ff00) >> 8)
#define VX_IOCG_BASE        'b'
#define VX_IOCG_VXBUS       'v'
#define VX_IOCG_VXBUS_RTP   'V'
#define VX_IOCG_VXBUS_SIO   'S'
#define VX_IOCG_SOCK        's'
#define VX_IOCG_NETIF       'n'
#define VX_IOCG_ARP         'a'
#define VX_IOCG_INET        '4'
#define VX_IOCG_INET6       '6'
#define VX_IOCG_ETH         '['
#define VX_IOCG_PPP         'P'
#define VX_IOCG_WLAN        'W'
#define VX_IOCG_MCAST       'M'
#define VX_IOCG_MCAST_IN6   'C'
#define VX_IOCG_RTAB        'R'
#define VX_IOCG_DS          'D'
#define VX_IOCG_POLICY_RT   'r'
#define VX_IOCG_FBDEV       'F'
#define VX_IOCG_GPUDEV      'G'
#define VX_IOCG_XRT         'g'
#define VX_IOCG_OPTEE       'o'
#define VX_IOCG_FW          'f'
#define VX_IOCG_AUDIO       'A'
#define VX_IOCG_CAMERA      'c'
#define VX_IOCG_EVDEV       'E'
#define VX_IOCG_UVC         'U'
#define VX_IOCG_WATCHDOG    'w'
#define VX_IOCG_DPAA2       'd'
#define VX_IOCG_OVL         'O'
#define VX_IOCG_GPIO        'I'
#define VX_IOCG_SPI         'k'
#define VX_IOCG_GENERIC     '\0' /* Any ioctl that is defined as a simple
                                  * numerical value will fall into this group */

#define VX_IOX(grp,idx,flg,t,dir)                                       \
    ((int) ( (dir)                                                      \
             | ((sizeof(t) & VX_IOCPARM_MASK) << 16)                    \
             | (flg)                                                    \
             | ((grp) << 8)                                             \
             | (idx & 0xff)))

#define VX_IO(g,i,f,t)   \
    VX_IOX(VX_IOCG_ ## g,i,VX_IOCF_ ## f,t,IOC_VOID)	/* no parameter */

#define VX_IOR(g,i,f,t)                                                 \
    VX_IOX(VX_IOCG_ ## g,i,VX_IOCF_ ## f,t,IOC_OUT)  	/* out-parameter */

#define VX_IOW(g,i,f,t)                                                 \
    VX_IOX(VX_IOCG_ ## g,i,VX_IOCF_ ## f,t,IOC_IN)	/* in-parameter */

#define VX_IOWR(g,i,f,t)                                                \
    VX_IOX(VX_IOCG_ ## g,i,VX_IOCF_ ## f,t,IOC_INOUT)	/* in- and out-parameter */

#define	IOCPARM_MASK	VX_IOCPARM_MASK
#define IOCPARM_MAX	0xfff		/* absolute maximum param size */
#define	IOC_VOID	VX_IOC_VOID	/* no parameters */
#define	IOC_OUT		VX_IOC_OUT	/* copy out parameters */
#define	IOC_IN		VX_IOC_IN	/* copy in parameters */
#define	IOC_INOUT	VX_IOC_INOUT
/* the 0x20000000 is so we can distinguish new ioctl's from old */

#define IOCPARM_LEN(x)  VX_IOCPARM_LEN(x)
#define	IOCGROUP(x)	VX_IOC_GROUP(x)

#define	_IOC(inout,group,num,len)                     \
	((int)(inout | ((len & IOCPARM_MASK) << 16) | \
			 ((group) << 8) | (num)))
#define	_IO(g,n)	VX_IOX(g, n, 0, 0, IOC_VOID)
#define	_IOR(g,n,t)	VX_IOX(g, n, 0, t, IOC_OUT)
#define	_IOW(g,n,t)	VX_IOX(g, n, 0, t, IOC_IN)
/* this should be _IORW, but stdio got there first */
#define	_IOWR(g,n,t)	VX_IOX(g, n, 0, t, IOC_INOUT)

/*
 * These definitions are primarily to support SIOCMUXPASSTHRU.
 * It's assumed that tunnellable IOCTLs will have IOC_USER set.
 * Note that IOC_USER overlaps the high bit of the type (x) byte.
 * When this byte is an ASCII character, its high bit will be 0;
 * but if the high bit of the type byte x is set, _IOR(x,y,t)
 * and _IORU(x,y,t) (for instance) will be indistinguishable.
 */

#define IOC_USER        0x8000 /* tunnellable from RTP space */
#define	_IOU(g,n)	(IOC_USER|_IO(g,n))
#define	_IORU(g,n,t)	(IOC_USER|_IOR(g,n,t))
#define	_IOWU(g,n,t)	(IOC_USER|_IOW(g,n,t))
#define	_IOWRU(g,n,t)	(IOC_USER|_IOWR(g,n,t))

#if false
struct spi_ioc_transfer {
	uint64_t	tx_buf;
	uint64_t	rx_buf;

	uint32_t	len;
	uint32_t	speed_hz;

	uint16_t	delay_usecs;
	uint8_t		bits_per_word;
	uint8_t		cs_change;
	uint8_t		tx_nbits;
	uint8_t		rx_nbits;
	uint8_t		word_delay_usecs;
	uint8_t		pad;

	/* If the contents of 'struct spi_ioc_transfer' ever change
	 * incompatibly, then the ioctl number (currently 0) must change;
	 * ioctls with constant size fields get a bit more in the way of
	 * error checking than ones (like this) where that field varies.
	 *
	 * NOTE: struct layout is the same in 64bit and 32bit userspace.
	 */
};

#define SPI_MSGSIZE(N) \
    ((((N)*(sizeof (struct spi_ioc_transfer))) < IOCPARM_MAX) \
    ? ((N)*(sizeof (struct spi_ioc_transfer))) : 0)

#else

typedef struct    /* describes a single SPI ioctl transfer message type */
    {
    uint32_t txBuf;
    uint32_t rxBuf;

    size_t    txLen;
    size_t    rxLen;

    uint16_t  delayUsecs;
    uint8_t   dummyCycles;
    uint8_t   txBuswidth;
    uint8_t   rxBuswidth;
    uint8_t   csHold;
    uint16_t  res2;
    } SPI_IOC_MSG;

/* N * msgsize must less than the ioctl */

#define SPI_MSGSIZE(N) \
    ((((N)*(sizeof (SPI_IOC_MSG))) < IOCPARM_MAX) \
    ? ((N)*(sizeof (SPI_IOC_MSG))) : 0)
#endif
/* composite operation */

#define SPI_IOC_MESSAGE(N)       _IOW(VX_IOCG_SPI, 0, char[SPI_MSGSIZE(N)])

/*
 * Read / Write of SPI device mode.
 * They are unsupport at present version, If it is needed in future, you can do
 * the mode conversion in vxbSpiIosDrvIoctl().
 */

#define SPI_IOC_RD_MODE          _IOR(VX_IOCG_SPI, 1, uint32_t)
#define SPI_IOC_WR_MODE          _IOW(VX_IOCG_SPI, 2, uint32_t)

/* read / write SPI device default max speed hz */

#define SPI_IOC_RD_MAX_SPEED_HZ  _IOR(VX_IOCG_SPI, 3, uint32_t)
#define SPI_IOC_WR_MAX_SPEED_HZ  _IOW(VX_IOCG_SPI, 4, uint32_t)

/* read / write SPI device chip select */

#define SPI_IOC_RD_CHIP_SELECT   _IOR(VX_IOCG_SPI, 5, uint8_t)
#define SPI_IOC_WR_CHIP_SELECT   _IOW(VX_IOCG_SPI, 6, uint8_t)


/* I2C message flags */
#define VX_IOCG_ZYNQ_DEV         'Z'

#define I2C_M_WR            0x0000      /* write data, from master to slave */
#define I2C_M_RD            0x0001      /* read data, from slave to master */

// I2C
struct i2c_msg {
    uint16_t addr;  /* Slave address, either seven or ten bits. When this is */
                    /* a ten bit address, I2C_M_TEN must be set in flags */
                    /* field and the driver must support ten bits */
    uint32_t len;   /* number of data bytes in buf being read from or written */
    uint8_t *buf;   /* the buffer into which data is read, or from which */
                    /* it's written */
    uint32_t flags; /* flags for the message */
    uint32_t wrTime;  /* Write Cycle Time if necessary */
    uint32_t scl;     /* SCL Clock Frequency, 0 means use controller's default */
};

#if false
struct i2c_master_s
{
  const struct i2c_ops_s ops;
};

struct i2c_ops_s
{
  int (transfer)(struct i2c_master_s dev,
                  struct i2c_msg_smsgs, int count);
#ifdef CONFIG_I2C_RESET
  int (reset)(struct i2c_master_sdev);
#endif
};
#endif

typedef	struct i2c_data{
   	uint8_t		cmd;	/* iic command or register offset */
   	char *	    buf;	/* data buffer */
   	size_t		cnt;	/* data length */
} I2C_DATA;

struct i2c_rdwr_ioctl_data {
     struct i2c_msg *msgs;    /* pointers to i2c_msgs */
     uint32_t nmsgs;           /* number of i2c_msgs */
 };

 //#define I2C_MSGSIZE(N) \
 //   ((((N)*(sizeof (struct i2c_rdwr_ioctl_data))) < IOCPARM_MAX) \
 //   ? ((N)*(sizeof (struct i2c_rdwr_ioctl_data))) : 0)

    /* I2C_0 (baro) */
#define	IIC_BARO_REG_SET		_IOW(VX_IOCG_ZYNQ_DEV, 1, sizeof(I2C_DATA))
#define	IIC_BARO_REG_GET		_IOR(VX_IOCG_ZYNQ_DEV, 2, sizeof(I2C_DATA))

    /* I2C_1 (mag) */
#define	IIC_MAG_REG_SET			_IOW(VX_IOCG_ZYNQ_DEV, 3, sizeof(I2C_DATA))
#define	IIC_MAG_REG_GET			_IOR(VX_IOCG_ZYNQ_DEV, 4, sizeof(I2C_DATA))

#define	IIC_TRANSFER	_IOR(VX_IOCG_ZYNQ_DEV, 5, sizeof(struct i2c_rdwr_ioctl_data))

/* tty IO ioctl */
/* #define FIONREAD        VX_IOR(GENERIC, 1, R, int) moved to filio.h */
#define FIOFLUSH        2   /* flush any chars in buffers */
#define FIOOPTIONS      3   /* set options (FIOSETOPTIONS) */
#define FIOBAUDRATE     4   /* set serial baud rate */
#define FIODISKFORMAT   5   /* format disk */
#define FIODISKINIT     6   /* initialize disk directory */
#define FIOSEEK         7   /* set current file char position */
#define FIOWHERE        8   /* get current file char position */
#define FIODIRENTRY     9   /* return a directory entry (obsolete)*/
#define FIORENAME       10  /* rename a directory entry */
#define FIOREADYCHANGE  11  /* return TRUE if there has been a
                               media change on the device */
#define FIONWRITE       12  /* get num chars still to be written */
#define FIODISKCHANGE   13  /* set a media change on the device */
#define FIOCANCEL       14  /* cancel read or write on the device */
#define FIOSQUEEZE      15  /* OBSOLETED since RT11 is obsoleted */
/* #define FIONBIO         VX_IOW(GENERIC, 16, W, int) moved to filio.h */
#define FIONMSGS        17  /* return num msgs in pipe */
#define FIOGETNAME      18  /* return file name in arg */
#define FIOGETOPTIONS   19  /* get options */
#define FIOSETOPTIONS   FIOOPTIONS  /* set options */
#define FIOISATTY       20  /* is a tty */
#define FIOSYNC         21  /* sync to disk */
#define FIOPROTOHOOK    22  /* specify protocol hook routine */
#define FIOPROTOARG     23  /* specify protocol argument */
#define FIORBUFSET      24  /* alter the size of read buffer  */
#define FIOWBUFSET      25  /* alter the size of write buffer */
#define FIORFLUSH       26  /* flush any chars in read buffers */
#define FIOWFLUSH       27  /* flush any chars in write buffers */
#define FIOSELECT       28  /* wake up process in select on I/O */
#define FIOUNSELECT     29  /* wake up process in select on I/O */
#define FIONFREE        30  /* get free byte count on device */
#define FIOMKDIR        31  /* create a directory */
#define FIORMDIR        32  /* remove a directory */
#define FIOLABELGET     33  /* get volume label */
#define FIOLABELSET     34  /* set volume label */
#define FIOATTRIBSET    35  /* set file attribute */
#define FIOCONTIG       36  /* allocate contiguous space */
#define FIOREADDIR      37  /* read a directory entry (POSIX) */
#define FIOUNMOUNT      39  /* unmount disk volume */
#define FIOSCSICOMMAND  40  /* issue a SCSI command */
#define FIONCONTIG      41  /* get size of max contig area on dev */
#define FIOTRUNC        42  /* truncate file to specified length */
#define FIOGETFL        43  /* get file mode, like fcntl(F_GETFL) */
#define FIOTIMESET      44  /* change times on a file for utime() */
#define FIOINODETONAME  45  /* given inode number, return filename*/
#define FIOFSTATFSGET   46  /* get file system status info */
#define FIOMOVE         47  /* move file, ala mv, (mv not rename) */

#define OPT_ECHO        0x01    /* echo input */
#define OPT_CRMOD       0x02    /* lf -> crlf */
#define OPT_TANDEM      0x04    /* ^S/^Q flow control protocol */
#define OPT_7_BIT       0x08    /* strip parity bit from 8 bit input */
#define OPT_MON_TRAP    0x10    /* enable trap to monitor */
#define OPT_ABORT       0x20    /* enable shell restart */
#define OPT_LINE        0x40    /* enable basic line protocol */
#define OPT_XANY        0x80    /* enable XANY for OPT_TANDEM */

#define OPT_LINE_PART 0x8000    /* In line mode, deliver a partial   */
                                /* line that reaches maximum length. */
                                /* This was an old behavior previous */
                                /* to VxWorks 6.2.                   */

#define OPT_RAW         0       /* raw mode */

#define OPT_TERMINAL    (OPT_ECHO | OPT_CRMOD | OPT_TANDEM | \
                         OPT_MON_TRAP | OPT_7_BIT | OPT_ABORT | OPT_LINE)

#define CONTIG_MAX      0xFFFFFFFFUL      /* "count" for FIOCONTIG if requesting maximum contiguous space on dev */

/* AXI_TIMER (PWM) */
#define	PWM_PULSE_FREQ_SET		_IOW(VX_IOCG_ZYNQ_DEV, 6, long)
#define	PWM_PULSE_DUTY_SET		_IOW(VX_IOCG_ZYNQ_DEV, 7, long)
#define	PWM_PULSE_DUTYTIME_SET	_IOW(VX_IOCG_ZYNQ_DEV, 8, long)
#define	PWM_PULSE_START			_IOW(VX_IOCG_ZYNQ_DEV, 9, int)
#define	PWM_PULSE_STOP			_IOW(VX_IOCG_ZYNQ_DEV, 10, int)