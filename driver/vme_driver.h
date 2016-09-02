/******************************************************************************
*
* Filename: 	vme_driver.h
* 
* Description:	Header file for VME device driver
*
* $Revision: 1.4 $
*
* $Date: 2014-05-30 14:27:16 $
*
* $Source: /home/cvs/cvsroot/Linuxvme4/linuxvmeen/vme_driver.h,v $
*
* Copyright 2000-2005 Concurrent Technologies.
*
******************************************************************************/

#ifndef __INCvme_driver
#define __INCvme_driver

enum addrSpace {
	VME_USER_SPACE=0,
	VME_KERNEL_SPACE
};

typedef struct 
{
	/*INT32 fileOpen;*/ 	/* Removed the device locking mechanism to allow multiple access*/
	UINT32 opendev;			/* Simultaneous multiple device access */
	UINT32 userlock;		/* User locked to device */
	UINT32 addrSpace;		/* user/kernel space API indicator*/
	UINT32 devrefcount;		/* Device access reference count */
	uid_t owner;			/* current owner */
	int mmapCount;			/* memory mapping counter */

} VMECTRL;

int vmedrv_getInfo( char *buffer );

#endif	/* __INCvme_driver */
