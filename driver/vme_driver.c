/******************************************************************************
*
* Filename: 	vme_driver.c
* 
* Description:	VME device driver for Linux.
*
* $Revision: 1.20 $
*
* $Date: 2015-04-30 12:03:33 $
*
* $Source: /home/cvs/cvsroot/Linuxvme4/linuxvmeen/vme_driver.c,v $
*
* Copyright 2000-2005 Concurrent Technologies.
*
******************************************************************************/
#ifndef __KERNEL__
#define __KERNEL__
#endif


#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33)
  #include <generated/autoconf.h>
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
  #include <linux/autoconf.h>
#else
  #include <linux/config.h>
#endif

#include <linux/module.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
  #include <generated/utsrelease.h>
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
  #include <linux/utsrelease.h>
#endif

#ifndef MODULE
#include <linux/init.h>
#define INIT_FUNC 		static int init_vmedriver
#define CLEANUP_FUNC 	static void cleanup_vmedriver
#else
#define INIT_FUNC 		int init_module
#define CLEANUP_FUNC 	void cleanup_module
#endif

#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/fs.h>
/*#include <linux/smp_lock.h>*/
#include <asm/uaccess.h>


#include "vme_api_en.h"
#include "vme_types.h"
#include "vme_driver.h"
#include "vme_os.h"
#include "vme_compat.h"
#include "vme_proc.h"

#define VME_DRIVER
#include "vme_version.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,10))
MODULE_LICENSE("Proprietary");
#endif

extern int umem_init( int mem_size );
extern void umem_cleanup( void );
extern int getVmeDriverLibInfo( char *buffer );

extern void unv_funcInit(void);
extern void tsi148_funcInit(void);
extern int unv_devQuery(void);
extern int tsi148_devQuery(void);
extern int bridgeless_init(void);
extern int bridgeless_cleanup(void);

int (*bridge_init)(int vecBufSize, ULONG pciAddr, ULONG pciSize, int bsUnaligned );
void (*bridge_cleanup)(void);
int (*bridge_readRegs)( long currPos, UINT8 *buf, unsigned int count );
int (*bridge_writeRegs)( long currPos, const UINT8 *buf, unsigned int count );
long (*bridge_readImage)( UINT32 minorNum, long currPos, UINT8 *buf, unsigned int length );
long (*bridge_writeImage)( UINT32 minorNum, long currPos, const UINT8 *buf, unsigned int length );
long (*bridge_readDma)( UINT32 minorNum, long currPos, UINT8 *buf, unsigned int length );
long (*bridge_writeDma)( UINT32 minorNum, long currPos, const UINT8 *buf, unsigned int length );
long (*bridge_seek)( UINT32 minorNum, long currPos, unsigned int offset, int whence );
int (*bridge_ioctlPciImage)( UINT32 minorNum, UINT32 cmd, ULONG arg , UINT8 addrSpace );
int (*bridge_ioctlVmeImage)( UINT32 minorNum, UINT32 cmd, ULONG arg,UINT8 addrSpace );
int (*bridge_ioctlDma)( UINT32 minorNum, UINT32 cmd, ULONG arg,UINT8 addrSpace);
int (*bridge_compatioctlDma)( UINT32 minorNum, UINT32 cmd, ULONG arg,UINT8 addrSpace);
int (*bridge_ioctl)( UINT32 cmd, ULONG arg,UINT8 addrSpace);
int (*bridge_mmapImage)( UINT32 minorNum, void *pVma, UINT32 size, UINT32 offset );
int (*bridge_mmapDma)( UINT32 minorNum,void *pVma, UINT32 size, UINT32 offset );
void (*bridge_closePciImage)( UINT32 minorNum );
void (*bridge_closeVmeImage)( UINT32 minorNum );
void (*bridge_closeDma)( UINT32 minorNum );
void (*bridge_closeCtl)( void );

#ifdef USE_CONSOLE
extern void os_dbgOpenConsole( void );
void os_dbgCloseConsole( void );
#endif

static int vmeMajor = 0; /* VME_DRIVER_MAJOR; 0 = use dynamic allocation */
static int initdev = 0;  /* Device lock data */

static loff_t vmedrv_llseek( struct file *file, loff_t offset, int whence );
static ssize_t vmedrv_read( struct file *file, char *buf, size_t count, 
							loff_t *ppos );
static ssize_t vmedrv_write( struct file *file, const char *buf, 
							size_t count, loff_t *ppos );

#if ((HAVE_UNLOCKED_IOCTL) && (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)))
static long vmedrv_unlockedioctl( struct file *file,
                                                unsigned int cmd, unsigned long arg );
#else
static int vmedrv_ioctl( struct inode *inode, struct file *file,
                                                unsigned int cmd, unsigned long arg );
#endif

#ifdef CONFIG_COMPAT
static long vmedrv_compatunlockedioctl( struct file *file,
                                                unsigned int cmd, unsigned long arg );
#endif

static int vmedrv_mmap( struct file *file, struct vm_area_struct *vma );
static int vmedrv_open( struct inode *inode, struct file *file );
static int vmedrv_close( struct inode *inode, struct file *file );

static void vmedrv_vmaClose( struct vm_area_struct *vma );

/* device methods declared using tagged format */
/* note: GNU compiler specific */ 
static struct file_operations vme_fops = 
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	llseek:		vmedrv_llseek,
	read:		vmedrv_read,
	write:		vmedrv_write,
	ioctl:		vmedrv_ioctl,
	mmap:		vmedrv_mmap,   
	open:		vmedrv_open,
	release:	vmedrv_close
#else
	.llseek     =    vmedrv_llseek,
        .read       =    vmedrv_read,
        .write      =    vmedrv_write,
#if ((HAVE_UNLOCKED_IOCTL) && (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)))
        .unlocked_ioctl = vmedrv_unlockedioctl,
#else
        .ioctl      =    vmedrv_ioctl,
#endif
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
#ifdef CONFIG_COMPAT
        .compat_ioctl = vmedrv_compatunlockedioctl,
#endif
#endif
        .mmap       =    vmedrv_mmap,
        .open       =    vmedrv_open,
        .release    =    vmedrv_close
#endif
};

static struct file_operations vme_bridgeless_fops =
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
        ioctl:          vmedrv_ioctl,
        open:           vmedrv_open,
        release:        vmedrv_close
#else
#if ((HAVE_UNLOCKED_IOCTL) && (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)))
        .unlocked_ioctl = vmedrv_unlockedioctl,
#else
        .ioctl      =    vmedrv_ioctl,
#endif
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
#ifdef CONFIG_COMPAT
        .compat_ioctl = vmedrv_compatunlockedioctl,
#endif
#endif
        .open       =    vmedrv_open,
        .release    =    vmedrv_close
#endif
};

struct vm_operations_struct vme_vm_ops = 
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	close: vmedrv_vmaClose
#else
	.close = vmedrv_vmaClose,
#endif
};

/* device status record */
static VMECTRL vmeCtrl[VME_MAX_MINORS];

char *boardName = NULL;
static ULONG resMemSize = 0;
static ULONG vecBufSize = 32;
static ULONG pciAddr = 0;
static ULONG pciSize = 0;
static ULONG bsUnaligned = 0;
static ULONG bridgeless_mode = 0;

/*Declare the mutex which would be used to control accesses
during acquiring device control*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
  DEFINE_MUTEX(vmeDevCtrl);
#else
  DECLARE_MUTEX(vmeDevCtrl);
#endif

#ifdef CONFIG_XEN
ULONG resMemStart = 0;
#endif

#ifdef MODULE
/* Module parameters, passed to module at load time */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5))
MODULE_PARM( boardName, "s" );  /* Can be used to override board type auto detection */

MODULE_PARM( resMemSize, "i" ); /* Can be used to specify the amount of reserved memory in MB's. 
                                   resMemSize values:  = 0  probe for user reserved memory
                                                       > 0  number of MB's of user reserved memory
                                                       < 0  disabled
								*/

MODULE_PARM( vecBufSize, "i" ); /* Can be used to increase the number of VME vectors
								   captured before buffer wraps, range from 32-128.
								   When number is > 32 extended vector capture mode
                                   is used.	
								*/

MODULE_PARM ( pciAddr, "i" );   /* Can be used to specify free PCI space for VME PCI
                                   window allocation.
                                */

MODULE_PARM ( pciSize, "i" );   /* Can be used to specify the total amount of PCI space
                                   to use for VME PCI window allocation.
                                */

MODULE_PARM ( bsUnaligned, "i" );   /* Can be used to byteswap unaligned transfers
                                       progressively
                                    */
#ifdef CONFIG_XEN
MODULE_PARM (resMemStart, "i" ); /*Can be used to specify the start of reserved memory when
					using a kernel configured for XEN*/
#endif
#else
module_param( boardName, charp, 0 );
MODULE_PARM_DESC(boardName, "board name");
module_param( resMemSize, long, 0 );
MODULE_PARM_DESC(resMemSize, "specify the amount of reserved memory");
module_param( vecBufSize, long, 0 );
MODULE_PARM_DESC(vecBufSize, "increase the number of VME vectors");
module_param( pciAddr, long, 0 );
MODULE_PARM_DESC(pciAddr, "specify free PCI space start address");
module_param( pciSize, long, 0 );
MODULE_PARM_DESC(pciSize, "specify size of free PCI space");
module_param(bsUnaligned, long, 0);
MODULE_PARM_DESC(bsUnaligned, "Enable byteswap for unaligned transfers");
#ifdef CONFIG_XEN
module_param( resMemStart, long, 0 );
MODULE_PARM_DESC(resMemStart, "specify the starting address of the reserved memory");
#endif
#endif
#endif


/******************************************************************************
*
* 		DEVICE LOCKING INTERFACE
*
******************************************************************************/


/******************************************************************************
*
* vmedrv_acquireDevice
*
* Acquire the device to operate on. This is used to arbitrate between the
* the request from the userland and the kernel APIs.
*
* RETURN: VME_SUCCESS if device is acquired successfully else error code
******************************************************************************/
int vmedrv_acquireDevice( UINT32 minorNum,UINT8 addrSpace )
{
	VMECTRL *cPtr;
	int result, i;

	/* Check for bridgeless mode */
	if( bridgeless_mode == 1)
	{
	    if(minorNum != 0)
	      return -VME_ENODEV;
	}
	cPtr = &vmeCtrl[minorNum];

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
	mutex_lock(&vmeDevCtrl);
#else
	down(&vmeDevCtrl);
#endif
	/* Kernel or User who comes first acquires the device */
	if(!initdev)
	{
		for(i = 0; i < VME_MAX_MINORS; i++)
		{
			vmeCtrl[i].opendev = 0;
			vmeCtrl[i].userlock = 0;
			vmeCtrl[i].devrefcount = 0;
		}
		initdev = 1;
	}

	if(!cPtr->opendev)
	{
		cPtr->addrSpace = addrSpace;
		cPtr->opendev = 1;
	}

	/*Check if the device is available*/	
	/*if(!cPtr->fileOpen)
	{
		cPtr->fileOpen++;
		cPtr->addrSpace=addrSpace;
		result=VME_SUCCESS;
	}
	else
	{
		result=(-VME_EBUSY);
	}*/

	if(cPtr->addrSpace == addrSpace)
	{
		cPtr->devrefcount++;
		result = VME_SUCCESS;
	}
	else
		result = -VME_EBUSY;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
	mutex_unlock(&vmeDevCtrl);
#else
	up(&vmeDevCtrl);
#endif
	return result;		
}

/******************************************************************************
*
* vmedrv_releaseDevice
*
* release the device acquired. 
*
*
* RETURN: None
******************************************************************************/
int vmedrv_releaseDevice( UINT32 minorNum,  UINT8 addrSpace)
{
        VMECTRL *cPtr;
        int result;

        /* Check for bridgeless mode */
        if( bridgeless_mode == 1)
        {
            if(minorNum != 0)
              return -VME_ENODEV;
        }

        cPtr = &vmeCtrl[minorNum];

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
		mutex_lock(&vmeDevCtrl);
#else
        down(&vmeDevCtrl);
#endif
        /*Release the device*/    
        /*if(cPtr->fileOpen)
        {
                cPtr->fileOpen--;
		cPtr->addrSpace=0;
        }*/
        if(cPtr->addrSpace == addrSpace)
		{
            if(addrSpace == 1)
            {
            	cPtr->devrefcount--;
    			if(cPtr->devrefcount == 0)
    			{
    		                cPtr->opendev = 0;
    		                cPtr->userlock = 0;
    				if ( (minorNum > VME_CONTROL) && (minorNum <= VME_LSI7) )
    				{
    					bridge_closePciImage( minorNum );
    				}
    				else if ( (minorNum > VME_LSI7) && (minorNum <= VME_VSI7) )
    				{
    						bridge_closeVmeImage( minorNum );
    				}
    				else if ( minorNum == VME_DMA0 || minorNum == VME_DMA1)
    				{
    						bridge_closeDma( minorNum );
    				}
    				else
    				{
    						bridge_closeCtl();
    				}
    			}
            }
        	result = VME_SUCCESS;
		}
        else
        {
        	result = -VME_EBUSY;
        }
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
		mutex_unlock(&vmeDevCtrl);
#else
        up(&vmeDevCtrl);
#endif
        return result;
}

/******************************************************************************
*
* vmedrv_getDeviceStat
*
* Get the status of the device
*
*
* RETURN: 0 = Free else Acquired
******************************************************************************/
int vmedrv_getDeviceStat( UINT32 minorNum,UINT8* addrSpace )
{
        VMECTRL *cPtr;
	INT32 result = 0;

        cPtr = &vmeCtrl[minorNum];

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
		mutex_lock(&vmeDevCtrl);
#else
        down(&vmeDevCtrl);
#endif
        /* result=cPtr->fileOpen;*/
        if(initdev == 0)
        {
        	result = 0; /* Device not acquired */
        }
        else
        {
        	if( addrSpace != NULL )
        	{
        		*addrSpace = cPtr->addrSpace;
        	}
        	result = 1;
        }
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
		mutex_unlock(&vmeDevCtrl);
#else
        up(&vmeDevCtrl);
#endif
	return result;
}


/******************************************************************************
*
*
* 		VME CORE DRIVER INTERFACE 
*
*******************************************************************************/

/******************************************************************************
*
* vmedrv_vmaClose
*
* Just keep track of how many times the device is mapped, to avoid releasing it.
* mmap() calls MOD_INC_USE_COUNT, vmedrv_vmaClose() excutes when munmap() called.
* Need to do it like this because driver is unaware of munmap().
*  
*
* RETURNS: None.
*
******************************************************************************/
static void vmedrv_vmaClose( struct vm_area_struct *vma )
{
	VMECTRL *cPtr;
	UINT32 minorNum;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
	minorNum = MINOR(vma->vm_file->f_path.dentry->d_inode->i_rdev);
#else
	minorNum = MINOR(vma->vm_file->f_dentry->d_inode->i_rdev);
#endif
	cPtr = &vmeCtrl[minorNum];
	cPtr->mmapCount--;

	DBMSG( "vmedrv_vmaClose minorNum: %u mmapCount %d", minorNum, cPtr->mmapCount );

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	MOD_DEC_USE_COUNT;
#else
	module_put( THIS_MODULE );
#endif
}


/******************************************************************************
*
* vmedrv_read
*
* Read from VME device driver file.
*
*
*  
*
* RETURNS: Number of bytes read if successful else an error code.
*
******************************************************************************/
static ssize_t vmedrv_read( struct file *file, char *buf, size_t count, loff_t *ppos )
{
	UINT32 minorNum;
	ssize_t result;
	char *nPtr;  
	 
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
	minorNum = MINOR(file->f_path.dentry->d_inode->i_rdev);
	nPtr = (char *) file->f_path.dentry->d_iname;
#else
	minorNum = MINOR(file->f_dentry->d_inode->i_rdev);
	nPtr = (char *) file->f_dentry->d_iname;
#endif

	/* check for invalid device file */
	if (minorNum >= VME_MAX_MINORS) 
	{
		ERRMSG( "attempt to read invalid device %s", nPtr );
		result = (-VME_ENODEV);
	}
	else
	{
		DBMSG( "%s read %u f_pos 0x%X %d bytes", nPtr, minorNum, 
				(UINT32) file->f_pos, (int) count );

		/* check user data buffer address is valid */
		if ( access_ok( VERIFY_READ, (ULONG) buf, (UINT32) count ) == 1 )
		{
			if (minorNum == VME_CONTROL_MINOR) 
			{
				result = bridge_readRegs( file->f_pos, buf, (count / 4) );
			}
			else if ( (minorNum > VME_CONTROL_MINOR) && (minorNum <= VME_VSI7_MINOR) ) 
			{
				result = bridge_readImage( minorNum, file->f_pos, buf, count );
			}
			else if ( minorNum == VME_DMA_MINOR || minorNum == VME_DMA1_MINOR )
			{
				result = bridge_readDma( minorNum ,file->f_pos, buf, count );
			}
			else
			{
				ERRMSG( "%s", "read function not supported" );
				result = (-VME_EPERM);
			}
		}
		else
		{
			ERRMSG( "%s read failed, user buffer invalid", nPtr);
			result = (-VME_EFAULT);
		}
	}

	if ( result > 0 )
	{
		*ppos += result;
	}

	return result;
}


/******************************************************************************
*
* vmedrv_write
*
* Write to VME device driver file.
*
*
*  
*
* RETURNS: Number of bytes written if successful else an error code.
*
******************************************************************************/
static ssize_t vmedrv_write( struct file *file, const char *buf, size_t count, loff_t *ppos )
{
	UINT32 minorNum;
	ssize_t result;  
	char *nPtr;
	 
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
	minorNum = MINOR(file->f_path.dentry->d_inode->i_rdev);
	nPtr = (char *) file->f_path.dentry->d_iname;
#else
	minorNum = MINOR(file->f_dentry->d_inode->i_rdev);
	nPtr = (char *) file->f_dentry->d_iname;
#endif

	/* check for invalid device file */
	if (minorNum >= VME_MAX_MINORS) 
	{
		ERRMSG( "attempt to write invalid device", nPtr );
		result = (-VME_ENODEV);
	}
	else
	{
		DBMSG( "%s write %u f_pos 0x%X %d bytes", nPtr, minorNum, 
				(UINT32) file->f_pos, (int) count );

		/* check user data buffer address is valid */
		if ( access_ok( VERIFY_WRITE, (ULONG) buf, (UINT32) count ) == 1 )
		{
			if (minorNum == VME_CONTROL_MINOR) 
			{
				result = bridge_writeRegs( file->f_pos, buf, (count / 4)  );
			}
			else if ( (minorNum > VME_CONTROL_MINOR) && (minorNum <= VME_VSI7_MINOR) ) 
			{
				result = bridge_writeImage( minorNum, file->f_pos, buf, count );
			}
			else if ( minorNum == VME_DMA_MINOR || minorNum == VME_DMA1_MINOR )
			{
				result = bridge_writeDma( minorNum ,file->f_pos, buf, count );
			}
			else
			{
				ERRMSG( "%s", "write function not supported" );
				result = (-VME_EPERM);
			}
		}
		else
		{
			ERRMSG( "%s write failed, user buffer invalid", nPtr);
			result = (-VME_EFAULT);
		}
	}

	if ( result > 0 )
	{
		*ppos += result;
	}

	return result;
}

/******************************************************************************
*
* vmedrv_getInstanceCount
*
* Obtain the Count of instances the device has been opened
*
*
*
*
* RETURNS: VME_SUCCESS or error code
*
******************************************************************************/
int vmedrv_getInstanceCount( UINT32 minorNum, ULONG arg )
{
    VMECTRL *cPtr;
    int count;

    cPtr = &vmeCtrl[minorNum];
    if(cPtr->opendev)
    {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
        mutex_lock(&vmeDevCtrl);
#else
    	down(&vmeDevCtrl);
#endif
    	count = cPtr->devrefcount;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
        mutex_unlock(&vmeDevCtrl);
#else
    	up(&vmeDevCtrl);
#endif
    	os_put_user32_nc( count, (UINT32*)arg );
    	return VME_SUCCESS;
    }
    else
    	return (-VME_EPERM);
}

/******************************************************************************
*
* vmedrv_ioctl
*
* Perform ioctl operations on a VME device driver file.
*
*
*  
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmedrv_ioctl( struct inode *inode, struct file *file,
						unsigned int cmd, unsigned long arg )
{
	UINT32 minorNum;
	int result=0;
	char *nPtr;


	minorNum = MINOR(inode->i_rdev);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
	nPtr = (char *) file->f_path.dentry->d_iname;
#else
	nPtr = (char *) file->f_dentry->d_iname;
#endif
	DBMSG( "%s ioctl %u 0x%X", nPtr, cmd, arg );

	/* check for invalid device file */
	if (minorNum >= VME_MAX_MINORS) 
	{
		DBMSG( "attempt to ioctl invalid device %s", nPtr );
		result = (-VME_ENODEV);
	}
	else
	{
		DBMSG( "%s ioctl %u 0x%X", nPtr, cmd, arg );

		if ( cmd == IOCTL_GET_POS || cmd == IOCTL_EN_GET_POS )
		{
			/* use checking version, calls access_ok() internally */ 
			if ( put_user( file->f_pos, (int *)arg ) )
			{
				ERRMSG( "%s", "ioctl failed to copy data to user space" );
				result = (-VME_EFAULT);
			}
			else
			{
				result = VME_SUCCESS;
			}
		}
		else if( cmd == IOCTL_GET_INST_COUNT )
		{
			result = vmedrv_getInstanceCount( minorNum, (ULONG) arg );
		}
		else
		{
			if ( (minorNum > VME_CONTROL_MINOR) && (minorNum <= VME_LSI7_MINOR) ) 
			{
				result = bridge_ioctlPciImage( minorNum, cmd, (ULONG) arg,0 );
			}
			else if ( (minorNum > VME_LSI7_MINOR) && (minorNum <= VME_VSI7_MINOR) )
			{
				result = bridge_ioctlVmeImage( minorNum, cmd, (ULONG) arg,0 );
			}
			else if ( minorNum == VME_DMA_MINOR || minorNum == VME_DMA1_MINOR )
			{
				result = bridge_ioctlDma( minorNum ,cmd, (ULONG) arg,0 );
			}
			else
			{
				result = bridge_ioctl( cmd, (ULONG) arg ,0);
			}
		}
	}
	return result;
}


/******************************************************************************
*
* vmedrv_compatioctl
*
* Perform ioctl operations on a VME device driver file.
*
*
*
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
#ifdef CONFIG_COMPAT
static int vmedrv_compatioctl( struct inode *inode, struct file *file,
                                                unsigned int cmd, unsigned long arg )
{
        UINT32 minorNum;
        int result=0;
        char *nPtr;


        minorNum = MINOR(inode->i_rdev);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
        nPtr = (char *) file->f_path.dentry->d_iname;
#else
        nPtr = (char *) file->f_dentry->d_iname;
#endif
        DBMSG( "%s ioctl %u 0x%X", nPtr, cmd, arg );

        /* check for invalid device file */
        if (minorNum >= VME_MAX_MINORS)
        {
                DBMSG( "attempt to ioctl invalid device %s", nPtr );
                result = (-VME_ENODEV);
        }
        else
        {
                DBMSG( "%s ioctl %u 0x%X", nPtr, cmd, arg );

                if ( cmd == IOCTL_GET_POS || cmd == IOCTL_EN_GET_POS )
                {
                        /* use checking version, calls access_ok() internally */
                        if ( put_user( file->f_pos, (int *)arg ) )
                        {
                                ERRMSG( "%s", "ioctl failed to copy data to user space" );
                                result = (-VME_EFAULT);
                        }
                        else
                        {
                                result = VME_SUCCESS;
                        }
                }
                else if( cmd == IOCTL_GET_INST_COUNT )
                {
                        result = vmedrv_getInstanceCount( minorNum, (ULONG) arg );
                }
                else
                {
                        if ( (minorNum > VME_CONTROL_MINOR) && (minorNum <= VME_LSI7_MINOR) )
                        {
                                result = bridge_ioctlPciImage( minorNum, cmd, (ULONG) arg,0 );
                        }
                        else if ( (minorNum > VME_LSI7_MINOR) && (minorNum <= VME_VSI7_MINOR) )
                        {
                                result = bridge_ioctlVmeImage( minorNum, cmd, (ULONG) arg,0 );
                        }
                        else if ( minorNum == VME_DMA_MINOR || minorNum == VME_DMA1_MINOR )
                        {
                                result = bridge_compatioctlDma( minorNum ,cmd, (ULONG) arg,0 );
                        }
                        else
                        {
                                result = bridge_ioctl( cmd, (ULONG) arg ,0);
                        }
                }
        }
        return result;
}
#endif

/******************************************************************************
*
* vmedrv_unlockedioctl
*
* Perform ioctl operations on a VME device driver file.
*
*
*
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
#if ((HAVE_UNLOCKED_IOCTL) && (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)))
static long vmedrv_unlockedioctl( struct file *file,
                                                unsigned int cmd, unsigned long arg )
{
    long result;
    os_ioctl_lock();
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
    result = (long)vmedrv_ioctl( (struct inode *)file->f_path.dentry->d_inode, file, cmd, arg );
#else
    result = (long)vmedrv_ioctl( (struct inode *)file->f_dentry->d_inode, file, cmd, arg );
#endif
    os_ioctl_unlock();
    return result;
}
#endif

/******************************************************************************
*
* vmedrv_compatunlockedioctl
*
* Perform compat ioctl operations on a VME device driver file.
*
*
*
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
#ifdef CONFIG_COMPAT
static long vmedrv_compatunlockedioctl( struct file *file,
                                                unsigned int cmd, unsigned long arg )
{
    long result;
    os_ioctl_lock();
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
    result = (long)vmedrv_compatioctl( (struct inode *)file->f_path.dentry->d_inode, file, cmd, arg );
#else
    result = (long)vmedrv_compatioctl( (struct inode *)file->f_dentry->d_inode, file, cmd, arg );
#endif
    os_ioctl_unlock();
    return result;
}
#endif
#endif
/******************************************************************************
*
* vmedrv_mmap
*
* Maps VME device driver memory to user memory. 
*
*
*  
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmedrv_mmap( struct file *file, struct vm_area_struct *vma )
{
	int result;
	char *nPtr;
	VMECTRL *cPtr;
	UINT32 size;
	UINT32 minorNum;
	UINT32 offset;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
	minorNum = MINOR(file->f_path.dentry->d_inode->i_rdev);
	nPtr = (char *) file->f_path.dentry->d_iname;
#else
	minorNum = MINOR(file->f_dentry->d_inode->i_rdev);
	nPtr = (char *) file->f_dentry->d_iname;
#endif
	/* check for invalid device file */
	if (minorNum >= VME_MAX_MINORS) 
	{
		ERRMSG( "attempt to mmap invalid device", nPtr );
		result = (-VME_ENODEV);
	}
	else
	{
		DBMSG( "%s mmap", nPtr );
		size = vma->vm_end - vma->vm_start;
		cPtr = &vmeCtrl[minorNum];

		offset = vma->vm_pgoff << PAGE_SHIFT;

		/* offset must be page aligned */ 
		if ( ((offset & (PAGE_SIZE-1)) == 0) &&
				((size % PAGE_SIZE) == 0) )  
		{
			/* dont't allow mmap of control device */
			if ( (minorNum > VME_CONTROL_MINOR) && (minorNum <= VME_VSI7_MINOR) ) 
			{
				result = bridge_mmapImage( minorNum, (void *) vma, size, offset );
			}
			else if ( minorNum == VME_DMA_MINOR || minorNum == VME_DMA1_MINOR )
			{
				result = bridge_mmapDma( minorNum ,(void *) vma, size, offset );
			}
			else
			{
				ERRMSG("Unsupported minor number");
				result = (-VME_EPERM);
			}
		
			/* if successful */
			if ( result == VME_SUCCESS )
			{

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
				vma->vm_file = file;
				vma->vm_ops = &vme_vm_ops;

				cPtr->mmapCount++;

				MOD_INC_USE_COUNT;	 /* done here because open(vma) is not called */

				DBMSG( "%s mmap OK", nPtr );
#else
				if ( try_module_get( THIS_MODULE ) )
				{
					vma->vm_file = file;
					vma->vm_ops = &vme_vm_ops;

					cPtr->mmapCount++;

					DBMSG( "%s mmap OK", nPtr );
				}
				else
				{
					DBMSG( "%s mmap failed", nPtr );
					result = (-VME_EINVAL);
				}
#endif	
			}
			else
			{
				DBMSG( "%s mmap failed", nPtr );
			}
		}
		else
		{
			result = (-VME_EINVAL);
		}
	}
	
	return result;
}


/******************************************************************************
*
* vmedrv_llseek
*
* Seek given postion in VME device driver file.
*
*
*  
*
* RETURNS: position if successful else an error code.
*
******************************************************************************/
static loff_t vmedrv_llseek( struct file *file, loff_t offset, int whence )
{
	UINT32 minorNum;
	loff_t result; 
	char *nPtr; 

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
	minorNum = MINOR(file->f_path.dentry->d_inode->i_rdev);
	nPtr = (char *) file->f_path.dentry->d_iname;
#else
	minorNum = MINOR(file->f_dentry->d_inode->i_rdev);
	nPtr = (char *) file->f_dentry->d_iname;
#endif

	/* check for invalid device file */
	if (minorNum >= VME_MAX_MINORS) 
	{
		ERRMSG( "attempt to seek invalid device %s", nPtr );
		result = (-VME_ENODEV);
	}
	else
	{
		DBMSG( "%s seek %d offset: 0x%X", 
				nPtr, whence, (UINT32) offset );

		result = bridge_seek( minorNum, file->f_pos, offset, whence );
		
		if (  result >= 0  )
		{
			file->f_pos = offset;
		}
		
		DBMSG( "f_pos: 0x%X", (UINT32) file->f_pos );
	}

	return result;
}


/******************************************************************************
*
* vmedrv_open
*
* Opens a VME device driver file.
*
*
*  
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmedrv_open( struct inode *inode, struct file *file )
{
	UINT32 minorNum;
	VMECTRL *cPtr;
	int result;
	char *nPtr;


	minorNum = MINOR(inode->i_rdev);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
	nPtr = (char *) file->f_path.dentry->d_iname;
#else
	nPtr = (char *) file->f_dentry->d_iname;
#endif
	/* check for invalid device file */
	if (minorNum >= VME_MAX_MINORS) 
	{
		ERRMSG( "attempt to open invalid device %s", nPtr );
		result = (-VME_ENODEV);
	}
	else
	{
		cPtr = &vmeCtrl[minorNum];

#ifndef MULTI_PROCESS
		/* if device already open return busy */
		if ( vmedrv_getDeviceStat(minorNum,NULL) != 0 )
		{
					ERRMSG( "%s", "device busy" );
					result = (-VME_EBUSY);
		}
		else
		{
#endif

			result =  vmedrv_acquireDevice(minorNum,0);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
			mutex_lock(&vmeDevCtrl);
#else
			down(&vmeDevCtrl);
#endif
			if( result == VME_SUCCESS )
			{
/* define for debug use only */  
#ifdef MULTI_PROCESS
		/* if device already open or not permitted user return busy */
		if ( cPtr->userlock == 1)
		{
			if(
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
                        (cPtr->owner != current->uid) &&        /* allow user */
                        (cPtr->owner != current->euid) &&       /* allow whoever did su */
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0))
                        (cPtr->owner != current->cred->uid) &&          /* allow user */
                        (cPtr->owner != current->cred->euid) &&         /* allow whoever did su */
#elif ((LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(3,13,9)))

  #ifdef CONFIG_UIDGID_STRICT_TYPE_CHECKS
                        (cPtr->owner != current->cred->uid.val) &&          /* allow user */
                        (cPtr->owner != current->cred->euid.val) &&         /* allow whoever did su */

  #else
			(cPtr->owner != current->cred->uid) &&  	/* allow user */
			(cPtr->owner != current->cred->euid) && 	/* allow whoever did su */
  #endif
#else
                        (cPtr->owner != current->cred->uid.val) &&          /* allow user */
                        (cPtr->owner != current->cred->euid.val) &&         /* allow whoever did su */
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
			!suser() ) 							/* still allow root suser()*/
#else
			!capable(CAP_SYS_ADMIN))
#endif
			{
				ERRMSG( "%s", "device busy" );
				result = (-VME_EBUSY);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
				mutex_unlock(&vmeDevCtrl);
#else
				up(&vmeDevCtrl);
#endif
				return result;
			}
		}
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
				cPtr->owner = current->uid;
				DBMSG( "%s device open successful", nPtr );

				MOD_INC_USE_COUNT;

				result = VME_SUCCESS;
#else
				if ( try_module_get( THIS_MODULE ) )
				{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
                                        cPtr->owner = current->uid;
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0))
                                        cPtr->owner = current->cred->uid;
#elif ((LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(3,13,9)))
  #ifdef CONFIG_UIDGID_STRICT_TYPE_CHECKS
				    cPtr->owner = current->cred->uid.val;

  #else
					cPtr->owner = current->cred->uid;
  #endif
#else
					cPtr->owner = current->cred->uid.val;
#endif
					cPtr->userlock = 1;
					DBMSG( "%s device open successful", nPtr );
					result = VME_SUCCESS;
				}
				else
				{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
					mutex_unlock(&vmeDevCtrl);
#else
					up(&vmeDevCtrl);
#endif
					vmedrv_releaseDevice(minorNum, 0);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
                    mutex_lock(&vmeDevCtrl);
#else
					down(&vmeDevCtrl);
#endif
					result = (-VME_EINVAL);
				}
#endif
			}
			else
			{
				ERRMSG( "%s", "device busy" );
				result = (-VME_EBUSY);
			}
#ifndef MULTI_PROCESS
		}
#endif
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
	mutex_unlock(&vmeDevCtrl);
#else
	up(&vmeDevCtrl);
#endif
	return result;
}


/******************************************************************************
*
* vmedrv_close
*
* Closes a VME device driver file.
*
*
*  
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmedrv_close( struct inode *inode, struct file *file )
{
  	UINT32 minorNum;
	VMECTRL *cPtr;
	int result;
	char *nPtr;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
	mutex_lock(&vmeDevCtrl);
#else
	down(&vmeDevCtrl);
#endif
	minorNum = MINOR(inode->i_rdev);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
	nPtr = (char *) file->f_path.dentry->d_iname;
#else
	nPtr = (char *) file->f_dentry->d_iname;
#endif

	/* check for invalid device file */
	if (minorNum >= VME_MAX_MINORS) 
	{
		ERRMSG( "attempt to close invalid device %s", nPtr );
		result = (-VME_ENODEV);
	}
	else
	{
		cPtr = &vmeCtrl[minorNum];

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
                if ( (cPtr->owner != current->uid) &&   /* allow user */
                (cPtr->owner != current->euid) &&       /* allow whoever did su */
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0))
                if ( (cPtr->owner != current->cred->uid) &&     /* allow user */
                   (cPtr->owner != current->cred->euid) &&         /* allow whoever did su */
#elif ((LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(3,13,9)))
  #ifdef CONFIG_UIDGID_STRICT_TYPE_CHECKS
                if ( (cPtr->owner != current->cred->uid.val) &&         /* allow user */
                (cPtr->owner != current->cred->euid.val) &&     /* allow whoever did su */

  #else
                if ( (cPtr->owner != current->cred->uid) &&     /* allow user */
                (cPtr->owner != current->cred->euid) &&         /* allow whoever did su */
  #endif
#else
                if ( (cPtr->owner != current->cred->uid.val) &&         /* allow user */
                    (cPtr->owner != current->cred->euid.val) &&     /* allow whoever did su */
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
			!suser() ) 							/* still allow root suser()*/
#else
        	!capable(CAP_SYS_ADMIN))
#endif
		{
  			ERRMSG( "%s", "close attempt by invalid user" );
			result = (-VME_EPERM);
  		} 
		else
		{ 
			cPtr->devrefcount--;
			if(cPtr->devrefcount == 0)
			{
                                cPtr->opendev = 0;
                                cPtr->userlock = 0;
				if ( (minorNum > VME_CONTROL_MINOR) && (minorNum <= VME_LSI7_MINOR) )
				{
					bridge_closePciImage( minorNum );
				}
				else if ( (minorNum > VME_LSI7_MINOR) && (minorNum <= VME_VSI7_MINOR) )
				{
					bridge_closeVmeImage( minorNum );
				}
				else if ( minorNum == VME_DMA_MINOR || minorNum == VME_DMA1_MINOR)
				{
					bridge_closeDma( minorNum );
				}
				else
				{
					bridge_closeCtl();
				}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
				mutex_unlock(&vmeDevCtrl);
#else
				up(&vmeDevCtrl);
#endif
				vmedrv_releaseDevice(minorNum, 0);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
				mutex_lock(&vmeDevCtrl);
#else
				down(&vmeDevCtrl);
#endif
			}
				DBMSG( "%s device closed", nPtr );

	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
				MOD_DEC_USE_COUNT;
	#else
				module_put( THIS_MODULE );
	#endif
			result = VME_SUCCESS;
		}
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
	mutex_unlock(&vmeDevCtrl);
#else
	up(&vmeDevCtrl);
#endif
  	return result;
}


/******************************************************************************
*
* vmedrv_getInfo
*
* Copy VME device driver information into given buffer.
*
*
*  
*
* RETURNS: number of bytes copied.
*
******************************************************************************/
int vmedrv_getInfo( char *buffer )
{
	int len = 0;


	if( buffer != NULL )
	{
		len += sprintf( buffer+len, "%s\n", vmeName );
		len += sprintf( buffer+len, "%s %s\n", vmeDriverVersion, creationDate);
		len += sprintf( buffer+len, "%s\n", copyright);
		len += sprintf( buffer+len, "Compiled with Kernel Version %s\n",  kernel_version);
		len += getVmeDriverLibInfo( buffer+len );
	}

	return len;
}


/******************************************************************************
*
* INIT_FUNC
*
* Initialization function.
* Called when module is loaded.
*
*
*  
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
INIT_FUNC( void )
{
	int result;
	char *sPtr;
	unsigned int vendor,device;
	void *pciDev=NULL;

	memset( vmeCtrl, 0, sizeof( vmeCtrl ) );

#ifdef USE_CONSOLE
	os_dbgOpenConsole();	/* for debug use */
#endif

	sPtr = (char *) kmalloc( 200, GFP_KERNEL );
	vmedrv_getInfo( sPtr );
	MSG( "%s", sPtr );
	kfree( sPtr );

	/* initialise the hookup functions depending upon the bridge detected*/
	if( os_devQuery( &vendor , &device , pciDev ) == VME_SUCCESS )
	{
		if( (vendor == TUNDRA_PCI_VENDOR_ID) && 
			( device == TSI148_PCI_DEVICE_ID ) )
		{
			MSG("TSI148 bridge found");
			tsi148_funcInit();
		}
		else
		{
			MSG("Universe bridge found");
			unv_funcInit();
		}
                /* Show module parameters */
                MSG("resMemSize      : %d", resMemSize);
                MSG("vecBufSize      : %d", vecBufSize);
                MSG("pciAddr         : 0x%x", pciAddr);
                MSG("pciSize         : 0x%x", pciSize);
                MSG("bsUnaligned     : %d", bsUnaligned);
        #ifdef CONFIG_XEN
                MSG("resMemStart     : 0x%x", resMemStart);
                if(resMemStart == 0 && resMemSize != 0)
                {
                        MSG("%s", "Please use the resMemStart command line parameter");
                        return -ENODEV;
                }
        #endif
	}
	else
	{
	    /*Register character driver Bridgless VME mode */
	    result = register_chrdev( vmeMajor, "vmedriver", &vme_bridgeless_fops );
	    if ( result >= 0 )
            {
                    /* if major number allocated dynamically */
                    if ( vmeMajor == 0 )
                    {
                            vmeMajor = result;
                    }
                    MSG( "VME device driver (MAJOR %d) initialised", vmeMajor );
                    MSG("Bridgeless Mode selected");
                    bridgeless_mode = 1;
                    result = 0;     /* must return zero if successful */
            }
	    else
            {
                    MSG( "Failed to register VME device driver with kernel" );
                    //bridgeless_cleanup();
            }
            /* Handle Bridgeless VME boards */
            result = bridgeless_init();
            if(result)
            {
                /* unregister device */
                unregister_chrdev( vmeMajor, "vmedriver" );
                return -ENODEV;
            }
            /* register with proc file system */
            vme_procInit(BRIDGELESS_DEVICE_ID);
	    return result;
	}

	/* initialise Universe device */
	result = bridge_init( vecBufSize, pciAddr, pciSize, bsUnaligned );
	

	if ( result == VME_SUCCESS )
	{
		/* register device with kernel */
		result = register_chrdev( vmeMajor, "vmedriver", &vme_fops );
		if ( result >= 0 ) 
		{
			/* if major number allocated dynamically */
			if ( vmeMajor == 0 )
			{
				vmeMajor = result;
			}

			/* register with proc file system */
			vme_procInit(device);
			/* see if there is any user reserved memory */
			umem_init( resMemSize );

			MSG( "VME device driver (MAJOR %d) initialised", vmeMajor );

			result = 0;	/* must return zero if successful */
		}
		else
		{
			MSG( "Failed to register VME device driver with kernel" );
			bridge_cleanup();
		}
	}
	else
	{
		MSG( "VME device driver failed to initialise" );
	} 

	return result;
}


/******************************************************************************
*
* CLEANUP_FUNC
*
* Cleanup function, free's resources.
* Called when module is unloaded.
*
*
*  
*
* RETURNS: None.
*
******************************************************************************/
CLEANUP_FUNC( void )
{
  if(!bridgeless_mode)
  {
	/* tidy up bridge device */
	bridge_cleanup();

	/* free user reserved memory */
	umem_cleanup();

	/* unregister device */
	unregister_chrdev( vmeMajor, "vmedriver" );

	/* remove proc file system entries */
	vme_procCleanup();

	MSG( "VME device driver removed" );

#ifdef USE_CONSOLE
	os_dbgCloseConsole();	/* for debug use */
#endif
  }
  else
  {
      /* tidy up bridgeless device */
      bridgeless_cleanup();
      /* unregister device */
      unregister_chrdev( vmeMajor, "vmedriver" );
      /* remove proc file system entries */
      vme_procCleanup();
  }
}

#ifndef MODULE
module_init( init_vmedriver );
module_exit( cleanup_vmedriver );
#endif

