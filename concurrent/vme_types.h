/******************************************************************************
*
* Filename: 	vme_types.h
* 
* Description:	Header file for VME types
*
* $Revision: 1.4 $
*
* $Date: 2012-10-30 13:48:07 $
*
* $Source: /home/cvs/cvsroot/Linuxvme4/linuxvmeen/vme_types.h,v $
*
* Copyright 2000-2005 Concurrent Technologies.
*
******************************************************************************/

#ifndef __INCvme_types
#define __INCvme_types

typedef struct
{
	UINT16 vendor;
	UINT16 device;
	UINT32 class;
	UINT32 baseAddress;
	UINT32 irq;
	void *pciDev;

} UNIPCIINFO;

typedef struct
{
	unsigned long totalram;
	unsigned long totalhigh;
	unsigned long freeram;
	unsigned long freehigh;

} UMEMINFO;

typedef struct
{
	long tv_sec;
	long tv_usec;

} UNITIME;

#define BIOS_ROM_START  0xf0000         /* BIOS ROM space */
#define BIOS_ROM_END_    0xfffff

#define BIOS_ROM_START_ 0xe0000			/* BIOS ROM space - VP717*/

typedef struct
{
        char   signature [4];   /* "$VME" */
        UINT8  version;                 /* version = 0x01 */
        UINT8  length;                  /* length = 0x1a (excludes header) */

        UINT16 reserved;                /* reserved = 0x0000 */
        UINT32 pciBaseAddress;  /* base address of free PCI space */
        UINT32 pciSize;                 /* size of remapped area */
        UINT32 pciImage0Base;   /* base address of PCI image 0 set in the BIOS */
        UINT32 pciImage1Base;   /* base address of PCI image 1 set in the BIOS */
        UINT32 pciImage2Base;   /* base address of PCI image 2 set in the BIOS */
        UINT32 pciImage3Base;   /* base address of PCI image 3 set in the BIOS */
        UINT32 pciImage4Base;   /* base address of PCI image 4 set in the BIOS */
        UINT32 pciImage5Base;   /* base address of PCI image 5 set in the BIOS */
        UINT32 pciImage6Base;   /* base address of PCI image 6 set in the BIOS */
        UINT32 pciImage7Base;   /* base address of PCI image 7 set in the BIOS */

} PCIADDRTBL;

typedef struct
{
        char   signature [4];   /* "$VME" */
        UINT8  version;                 /* version = 0x01 */
        UINT8  length;                  /* length = 0x1a (excludes header) */

        UINT16 reserved;                /* reserved = 0x0000 */
        UINT32 pciBaseAddress;  /* base address of free PCI space */
        UINT32 pciSize;                 /* size of remapped area */
        UINT32 pciImage0Base;   /* base address of PCI image 0 set in the BIOS */
        UINT32 pciImage1Base;   /* base address of PCI image 1 set in the BIOS */
        UINT32 pciImage2Base;   /* base address of PCI image 2 set in the BIOS */
        UINT32 pciImage3Base;   /* base address of PCI image 3 set in the BIOS */
        UINT32 pciImage4Base;   /* base address of PCI image 4 set in the BIOS */
        UINT32 pciImage5Base;   /* base address of PCI image 5 set in the BIOS */
        UINT32 pciImage6Base;   /* base address of PCI image 6 set in the BIOS */
        UINT32 pciImage7Base;   /* base address of PCI image 7 set in the BIOS */
        UINT32 vmeBaseAddress;  /* base address of free PCI space */
        UINT32 vmeSize;                 /* size of remapped area */
        UINT32 vmeImage0Base;   /* base address of PCI image 0 set in the BIOS */
        UINT32 vmeImage1Base;   /* base address of PCI image 1 set in the BIOS */
        UINT32 vmeImage2Base;   /* base address of PCI image 2 set in the BIOS */
        UINT32 vmeImage3Base;   /* base address of PCI image 3 set in the BIOS */
        UINT32 vmeImage4Base;   /* base address of PCI image 4 set in the BIOS */
        UINT32 vmeImage5Base;   /* base address of PCI image 5 set in the BIOS */
        UINT32 vmeImage6Base;   /* base address of PCI image 6 set in the BIOS */
        UINT32 vmeImage7Base;   /* base address of PCI image 7 set in the BIOS */

} PCIADDRTBL_VER2;

enum vmeBridge {
	UNIVERSEII=0,
	TSI148
};

#define VME_CONTROL_MINOR 	0	/* minor device numbers */
#define VME_LSI0_MINOR		1
#define VME_LSI1_MINOR		2
#define VME_LSI2_MINOR		3
#define VME_LSI3_MINOR		4
#define VME_LSI4_MINOR		5
#define VME_LSI5_MINOR		6
#define VME_LSI6_MINOR		7
#define VME_LSI7_MINOR		8
#define VME_VSI0_MINOR		9
#define VME_VSI1_MINOR		10
#define VME_VSI2_MINOR		11
#define VME_VSI3_MINOR		12
#define VME_VSI4_MINOR		13
#define VME_VSI5_MINOR		14
#define VME_VSI6_MINOR		15
#define VME_VSI7_MINOR		16
#define VME_DMA_MINOR		17
#define VME_DMA1_MINOR		18
#define VME_MAX_MINORS		19	/* number of device files */

/* operating modes */
#define MODE_UNDEFINED	0
#define MODE_IOREMAPPED	1

#define SEEK_SET 0	/* set current offset to given value */
#define SEEK_CUR 1	/* add given value to current offset */

#ifndef NULL
#define NULL (void *) 0
#endif

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

#if defined( DEBUG ) || defined( ERROR_MSG )
extern void os_dbgMsg( char *fmt, ... );
#endif
#ifdef DEBUG
#define DBMSG(msg, args...) os_dbgMsg( "dbgVME %03d: " msg, __LINE__, ##args )
#else
#define DBMSG(msg, args...)
#endif

#ifdef ERROR_MSG
#define ERRMSG(msg, args...) os_dbgMsg( "errVME %03d: " msg, __LINE__, ##args )
#else
#define ERRMSG(msg, args...)
#endif

#define MSG(msg, args...) os_logMsg( msg, ##args )

#endif	/* __INCvme_types */
