#ifndef universeII_H_
#define universeII_H_ 1

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

enum universe_motherboard_type {
	UNIVERSE_BOARD_TYPE_UNKNOWN = 0,
	UNIVERSE_BOARD_TYPE_CCT,
	UNIVERSE_BOARD_TYPE_VMIC
};


#endif
