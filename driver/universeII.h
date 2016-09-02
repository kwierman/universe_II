#ifndef universeII_H_
#define universeII_H_ 1

struct universe_ioport_ioctl {
	char value;
	uint16_t address;
};

enum universeII_ioctl
{
	IOCTL_EX_LSI_IMAGE=0,
	IOCTL_EX_VSI_IMAGE,
	IOCTL_EX_DMA_DIRECT,
	IOCTL_EX_DMA_LIST,
	IOCTL_EX_INTERRUPT,
	IOCTL_EX_REGUSRINT,
	IOCTL_EX_INTCOUNT,
	IOCTL_EX_REMUSRINT,
	IOCTL_EX_INSCOUNT_CTL,
	IOCTL_EX_GENERATE_IT,
	IOCTL_EX_GET_GEO,
	IOCTL_EX_SETCAPMODE,
	IOCTL_EX_ENHVECTREAD,
	IOCTL_EX_INTERRUPT_BRIDGELESS,
	IOCTL_EX_SET_SYSFAIL,
	IOCTL_EX_GET_SYSFAIL,
	IOCTL_EX_CLEAR_SYSFAIL,
};

#define VME_SUCCESS		0
#define	VME_EPERM		1	/* Operation not permitted */
#define	VME_EINTR		4	/* Interrupted system call */
#define	VME_EIO		 	5	/* I/O error */
#define	VME_ENOMEM		12	/* Out of memory */
#define	VME_EFAULT		14	/* Bad address */
#define	VME_EBUSY		16	/* Device or resource busy */
#define	VME_ENODEV		19	/* No such device */
#define	VME_EINVAL		22	/* Invalid argument */
#define	VME_ETIME		62	/* Timer expired */

// Defining IOCTL calls
// We are adding ioctl numbers to the kernel
// so we obey linux convention.
#define UNIVERSE_MAGIC_NUMBER  0xF1

#define UNIVERSE_IOCSET_CTL          _IO(UNIVERSE_MAGIC_NUMBER,   1)
#define UNIVERSE_IOCSET_BS           _IO(UNIVERSE_MAGIC_NUMBER,   2)
#define UNIVERSE_IOCSET_BD           _IO(UNIVERSE_MAGIC_NUMBER,   3)
#define UNIVERSE_IOCSET_VME          _IO(UNIVERSE_MAGIC_NUMBER,   4)
#define UNIVERSE_IOCPCI_SIZE         _IO(UNIVERSE_MAGIC_NUMBER,   5)
#define UNIVERSE_IOCSET_IOREMAP      _IO(UNIVERSE_MAGIC_NUMBER,   6)
#define UNIVERSE_IOCSET_HW_BYTESWAP  _IO(UNIVERSE_MAGIC_NUMBER,   7)
#define UNIVERSE_IOCGET_MEM_SIZE     _IOR(UNIVERSE_MAGIC_NUMBER,  8, unsigned long)
#define UNIVERSE_IOCGET_BOARD_TYPE   _IOR(UNIVERSE_MAGIC_NUMBER,  9, unsigned int)
#define UNIVERSE_IOCIO_PORT_READ     _IOWR(UNIVERSE_MAGIC_NUMBER, 10, struct universe_ioport_ioctl)
#define UNIVERSE_IOCIO_PORT_WRITE    _IOW(UNIVERSE_MAGIC_NUMBER,  11, struct universe_ioport_ioctl)
#define UNIVERSE_IOCCHECK_BUS_ERROR  _IO(UNIVERSE_MAGIC_NUMBER,   12)
#define UNIVERSE_IOCSET_VME_COMM     _IO(UNIVERSE_MAGIC_NUMBER,   13)
#define UNIVERSE_IOCREAD_VME_COMM    _IOR(UNIVERSE_MAGIC_NUMBER,  14, unsigned int)
#define UNIVERSE_IOC_MAXNR      14

enum universe_motherboard_type {
	UNIVERSE_BOARD_TYPE_UNKNOWN = 0,
	UNIVERSE_BOARD_TYPE_CCT,
	UNIVERSE_BOARD_TYPE_VMIC
};

// Byte-swapping arguments for SET_HW_BYTESWAP
#define UNIVERSE_IOCMASTER_BYTESWAP  0x8
#define UNIVERSE_IOCSLAVE_BYTESWAP   0x10
#define UNIVERSE_IOCFAST_BYTESWAP    0x20

#define UNIVERSE_VMIC_ENABLE_VME_BUS 		0x800
#define UNIVERSE_VMIC_ENABLE_MASTER_BIG_ENDIAN	0x1
#define UNIVERSE_VMIC_ENABLE_SLAVE_BIG_ENDIAN	0x2
#define UNIVERSE_VMIC_ENABLE_ENDIAN_CONV_BYPASS	0x400

#define U2SPEC                 0x04FC


#endif
