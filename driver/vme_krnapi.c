/******************************************************************************
*
* Filename: 	vme_krnapi.c
* 
* Description:	Kernel space VME API.
*
* Copyright 2000-2008 Concurrent Technologies.
*
******************************************************************************/
#ifndef __KERNEL__
#define __KERNEL__
#endif

#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
  #if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
    #include <generated/utsrelease.h>
    #include <generated/autoconf.h>
  #else
    #include <linux/utsrelease.h>
    #include <linux/autoconf.h>
  #endif
#else
  #include <linux/config.h>
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>

#include "vme_api_en.h"
#include "vme_types.h"
#include "vme_os.h"
#include "vme_driver.h"

#if defined(CONFIG_MODVERSIONS) & (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
#define EXPORT_SYMBOL_VME	EXPORT_SYMBOL_NOVERS
#else
#define EXPORT_SYMBOL_VME	EXPORT_SYMBOL
#endif

extern int (*bridge_init)(int vecBufSize, int pciAddr, int pciSize );
extern void (*bridge_cleanup)(void);
extern int (*bridge_readRegs)( long currPos, UINT8 *buf, unsigned int count );
extern int (*bridge_writeRegs)( long currPos, const UINT8 *buf, unsigned int count );
extern int (*bridge_readImage)( UINT32 minorNum, long currPos, UINT8 *buf, unsigned int length );
extern int (*bridge_writeImage)( UINT32 minorNum, long currPos, const UINT8 *buf, unsigned int length );
extern int (*bridge_readDma)( UINT32 minorNum, long currPos, UINT8 *buf, unsigned int length );
extern int (*bridge_writeDma)( UINT32 minorNum, long currPos, const UINT8 *buf, unsigned int length );
extern long (*bridge_seek)( UINT32 minorNum, long currPos, unsigned int offset, int whence );
extern int (*bridge_ioctlPciImage)( UINT32 minorNum, UINT32 cmd, ULONG arg , UINT8 addrSpace );
extern int (*bridge_ioctlVmeImage)( UINT32 minorNum, UINT32 cmd, ULONG arg,UINT8 addrSpace );
extern int (*bridge_ioctlDma)( UINT32 minorNum, UINT32 cmd, ULONG arg,UINT8 addrSpace);
extern int (*bridge_ioctl)( UINT32 cmd, ULONG arg,UINT8 addrSpace);
extern int (*bridge_mmapImage)( UINT32 minorNum, void *pVma, UINT32 size, UINT32 offset );
extern int (*bridge_mmapDma)( UINT32 minorNum,void *pVma, UINT32 size, UINT32 offset );
extern void (*bridge_closePciImage)( UINT32 minorNum );
extern void (*bridge_closeVmeImage)( UINT32 minorNum );
extern void (*bridge_closeDma)( UINT32 minorNum );
extern void (*bridge_closeCtl)( void );

extern int vmedrv_acquireDevice( UINT32 minorNum,UINT8 addrSpace );
extern void vmedrv_releaseDevice( UINT32 minorNum,UINT8 addrSpace );
extern int vmedrv_getDeviceStat( UINT32 minorNum,UINT8* addrSpace );
extern int vmedrv_getInstanceCount( UINT32 minorNum, ULONG arg );

/******************************************************************************
*
*
* 		VME KERNEL API FUNCTION DEFINITIONS 
*
*******************************************************************************/

/***************************** Common Device Functions*************************/

/******************************************************************************
*
* vmekrn_acquireDevice
*
* Acquires the device for the kernel API operations
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_acquireDevice( UINT32 imageNumber )
{
	int result;

	if( imageNumber >= VME_CONTROL && imageNumber < VME_MAXS )
	{
		result = vmedrv_acquireDevice( imageNumber, VME_KERNEL_SPACE );
	}
	else
	{
		DBMSG("Invalid device number");
		result = (-VME_EINVAL);
	}

	return result; 

}

/******************************************************************************
*
* vmekrn_releaseDevice
*
* Releases the device 
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
void vmekrn_releaseDevice( UINT32 imageNumber )
{

		if( imageNumber >= VME_CONTROL && imageNumber < VME_MAXS )
        {
                vmedrv_releaseDevice( imageNumber, 1 );
        }
}


/***************************** Control functions *****************************/

/******************************************************************************
*
* vmekrn_setInterruptMode
*
* Sets the TSI148/UNiverseII interrupt mode to ROAK (default) or RORA.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_setInterruptMode( UINT8 mode )
{
	int result;
        UINT8 addressSpace;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {

                        result = bridge_ioctl(IOCTL_EN_SET_INT_MODE,
                                        (ULONG)mode,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;

}

/******************************************************************************
*
* vmekrn_enableInterrupt
*
* Enables the given TSI148/Universe II interrupt.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_enableInterrupt( UINT8 intNumber )
{
	int result;
        UINT8 addressSpace=0;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_ENABLE_INT,
                                        (ULONG)intNumber,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}


/******************************************************************************
*
* vmekrn_disableInterrupt
*
* Disables the given TSI148/Universe II interrupt.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_disableInterrupt( UINT8 intNumber )
{
	int result;
        UINT8 addressSpace=0;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_DISABLE_INT,
                                        (ULONG)intNumber,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_generateInterrupt
*
* Generates the given VME interrupt.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_generateInterrupt( UINT8 intNumber )
{
	int result;
        UINT8 addressSpace=0;

	DBMSG("%s",__FUNCTION__);

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_GENERATE_INT,
                                        (ULONG)intNumber,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}


/******************************************************************************
*
* vmekrn_registerInterrupt
*
* Registers the given VME interrupt.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_registerInterrupt( EN_VME_INT_DATA *iPtr )
{
	int result;
        UINT8 addressSpace=0;

	DBMSG("%s",__FUNCTION__);

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_REGISTER_INT,
                        		(ULONG)iPtr,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_removeInterrupt
*
* Removes the given handler for VME interrupt.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_removeInterrupt( EN_VME_INT_DATA *iPtr )
{
	int result;
        UINT8 addressSpace=0;

	DBMSG("%s",__FUNCTION__);

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_REMOVE_INT,
                        		(ULONG)iPtr,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_readInterruptInfo
*
* Read the interrupt information for given VME interrupt.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_readInterruptInfo( EN_VME_INT_INFO *iPtr )
{
 	int result;
        UINT8 addressSpace=0;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_GET_VME_INT_INFO,
                                        (ULONG)iPtr,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_readExtInterruptInfo
*
* Read the extended interrupt information for given VME interrupt.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_readExtInterruptInfo(  EN_VME_EXTINT_INFO *iPtr )
{
	int result;
        UINT8 addressSpace=0;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_GET_VME_EXTINT_INFO,
                                        (ULONG)iPtr,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_readInterruptEnhInfo
*
* Read the enhanced interrupt information for given VME interrupt.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_readInterruptEnhInfo(  EN_VME_ENHINT_INFO *iPtr )
{
        int result;
        UINT8 addressSpace=0;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL, &addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_GET_VME_ENHINT_INFO,
                                        (ULONG)iPtr,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_setVectorCaptureMode
*
* Set ACK mode and number of Vectors.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_setVectorCaptureMode( UINT8 iackMode, UINT32 numVectors )
{
        int result;
        UINT8 addressSpace=0;
        EN_VME_CAPTURE_MODE vectCapt;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL, &addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                    vectCapt.ackMode = iackMode;
                    vectCapt.numberVect = numVectors;
                    result = bridge_ioctl(IOCTL_EN_SET_CAPTURE_MODE,
                                    (ULONG)&vectCapt, VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}


/******************************************************************************
*
* vmekrn_setStatusId
*
* Writes the given status ID the the TSI148 STATUSID field in VICR.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_setStatusId(  UINT8 statusId )
{
	int result;
        UINT8 addressSpace=0;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_SET_STATUSID,
                                        (ULONG)statusId,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_waitInterrupt
*
* Waits for one of the given interrupts or timeout to occur.
* If timeout is zero wait forever.
* Timeout is specified in jiffies.
*
* Selected interrupts are passed as bit settings in the selectedInts parameter
* where bit 0 = Reserve
*               bit 1 = VIRQ1
*               bit 2 = VIRQ2
*               etc.
*
* The interrupt received is returned in the intNum parameter in the same
* format as above.
*
* If a wait call is already pending on a selected interrupt the function will
* return a error code and the conflicting interrupt bit will be returned in the
* intNum parameter
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_waitInterrupt( UINT32 selectedInts, UINT32 timeout, UINT32 *intNum )
{
	EN_VME_WAIT_LINT idata;
        int result;
	UINT8 addressSpace;

        *intNum = 0;

        /* only 26 bits should be used */
        if ( (selectedInts & 0xf0000000) == 0 )
        {
		/*Check if the device is acquired for kernel API usage*/
        	result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        	if( result != 0 )
        	{
                	if ( addressSpace == VME_KERNEL_SPACE )
                	{
				idata.intNum = selectedInts;
				idata.timeout = timeout;
                        	result = bridge_ioctl(IOCTL_EN_WAIT_LINT,
                                        (ULONG)&idata,VME_KERNEL_SPACE);

				if ( (idata.intNum & 0x80000000) != 0 )
                		{
                        		*intNum = idata.intNum;
                		}
                	}
                	else
                	{
                        	DBMSG("Device is acquired by userspace API");
                        	result = (-VME_EBUSY);
                	}
        	}
        	else
        	{
                	DBMSG("Device not acquired");
                	result = (-VME_EPERM);
        	}
        }
        else
        {
                result = (-EINVAL);
        }

        return result;
}

/******************************************************************************
*
* vmekrn_setByteSwap
*
* Enable/disable hardware byte swapping on supported boards
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_setByteSwap(  UINT8 enable )
{
	int result;
        UINT8 addressSpace=0;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_SET_BYTE_SWAP,
                                        (ULONG)enable,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_setUserAmCodes
*
* Sets the User address modifier codes, in the USER_AM register, to the given
* values.
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_setUserAmCodes( EN_VME_USER_AM *amPtr )
{
	int result;
        UINT8 addressSpace=0;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_SET_USER_AM_CODES,
                                        (ULONG)amPtr,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_readVerrInfo
*
* Read the VERR interrupt information 
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_readVerrInfo(  ULONG *Address,UINT8 *Direction, UINT8 *AmCode)
{
	UINT8 i;
        UINT8 shift;
        UINT8 capBuf[16];
	UINT8 addressSpace;
        int result;

        *Address = 0;
        *Direction = 0;
        *AmCode = 0;

	/*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_GET_VERR_INFO,
                                        (ULONG)capBuf,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        if ( result >= 0 )
        {
                shift = 28;
                for (i = 0; i < 7; i++ )
                {
                        *Address |= (UINT32) (capBuf[i] & 0xf) << shift;
                        shift -= 4;
                }

                *Address |= (UINT32) capBuf[7] & 0xe;
                *Address |= (UINT32) (capBuf[8] & 0x8) >> 3; // use DS1 for A0

                /* Address modifier codes */
                /* See Table 2 -3 in the VME64 Specification */
                *AmCode |= (UINT8) (capBuf[8] & 0x3) << 4;
                *AmCode |= (UINT8) (capBuf[9] & 0xf);

                /* Data transfer direction 0 = Master to Slave */
                /*                         1 = Slave to Master */
                *Direction |= (UINT8) (capBuf[10] & 0xf) >> 3;
        }

        return result;

}

/******************************************************************************
*
* vmekrn_readVerrInfoUserHandler
*
* Read the VERR interrupt information
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_readVerrInfoUserHandler(  ULONG *Address,UINT8 *Direction, UINT8 *AmCode)
{
        UINT8 i;
        UINT8 shift;
        UINT8 capBuf[16];
        int result;

        *Address = 0;
        *Direction = 0;
        *AmCode = 0;

        result = bridge_ioctl(IOCTL_EN_GET_VERR_INFO,
                        (ULONG)capBuf,VME_KERNEL_SPACE);

        if ( result >= 0 )
        {
                shift = 28;
                for (i = 0; i < 7; i++ )
                {
                        *Address |= (UINT32) (capBuf[i] & 0xf) << shift;
                        shift -= 4;
                }

                *Address |= (UINT32) capBuf[7] & 0xe;
                *Address |= (UINT32) (capBuf[8] & 0x8) >> 3; // use DS1 for A0

                /* Address modifier codes */
                /* See Table 2 -3 in the VME64 Specification */
                *AmCode |= (UINT8) (capBuf[8] & 0x3) << 4;
                *AmCode |= (UINT8) (capBuf[9] & 0xf);

                /* Data transfer direction 0 = Master to Slave */
                /*                         1 = Slave to Master */
                *Direction |= (UINT8) (capBuf[10] & 0xf) >> 3;
        }

        return result;

}

/******************************************************************************
*
* vmekrn_clearStats
*
* Clears the drivers statistic counters.
*
*
*
*
* RETURNS:  0 if successful else an error code.
*
******************************************************************************/
int vmekrn_clearStats( void )
{
	int result;
        UINT8 addressSpace;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_CLEAR_STATS,
                                        0,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_getStats
*
* Get the driver statistics
*
*
*
*
* RETURNS:  0 if successful else an error code.
*
******************************************************************************/
int vmekrn_getStats( UINT32 type, void *iPtr )
{
        EN_VME_STATS sPtr;
        int result;
	UINT8 addressSpace;

        if( type >= VME_STATUS_CTRL && type <= VME_STATUS_INTS )
        {
		/*Check if the device is acquired for kernel API usage*/
        	result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        	if( result != 0 )
        	{
                	if ( addressSpace == VME_KERNEL_SPACE )
                	{
                		sPtr.type = type;
                        	result = bridge_ioctl(IOCTL_EN_GET_STATS,
                                        (ULONG)&sPtr,VME_KERNEL_SPACE);

				if( result == VME_SUCCESS )
				{
					os_memcpy(iPtr,&sPtr.driverStat,
						sizeof(EN_VME_DRIVER_STAT));
                	}	}
                	else
                	{
                        	DBMSG("Device is acquired by userspace API");
                        	result = (-VME_EBUSY);
                	}
        	}
        	else
        	{
                	DBMSG("Device not acquired");
                	result = (-VME_EPERM);
        	}
        }
        else
        {
		DBMSG("Unsupported status code");
                result = (-VME_EINVAL);
        }

        return result;
}

/******************************************************************************
*
* vmekrn_getBoardCap
*
* Get the board capabalities
*
*
*
*
* RETURNS: number of bytes written if successful else error code.
*
******************************************************************************/
int vmekrn_getBoardCap( UINT32 *boardFlags )
{
	int result;
        UINT8 addressSpace;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_GET_CAPABILITY,
                                        (ULONG)boardFlags,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;

}

/******************************************************************************
*
* vmekrn_enableRegAccessImage
*
* Enables TSI148/Universe II register access image window.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_enableRegAccessImage( EN_VME_IMAGE_ACCESS *iPtr )
{
	int result;
	UINT8 addressSpace;

        /*Check if the device is acquired for kernel API usage*/
	result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
	{
		if ( addressSpace == VME_KERNEL_SPACE )
        	{	
			result = bridge_ioctl(IOCTL_EN_ENABLE_REG_IMAGE,
					(ULONG)iPtr,VME_KERNEL_SPACE);
		}
		else
		{
			DBMSG("Device is acquired by userspace API");
			result = (-VME_EBUSY);
		}
	}
	else 
	{
		DBMSG("Device not acquired");
		result = (-VME_EPERM);
	}
	
	return  result;
}

/******************************************************************************
*
* vmekrn_disableRegAccessImage
*
* Disables Universe II/TSI148 register access image window.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_disableRegAccessImage( void )
{
	int result;
        UINT8 addressSpace;
        DBMSG("%s Entered",__FUNCTION__);

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_DISABLE_REG_IMAGE,
				0,VME_KERNEL_SPACE);
                }
                else
                {
			DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
		DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_enableLocationMon
*
* Enables TSI148/Universe II location monitor image window.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_enableLocationMon( EN_VME_IMAGE_ACCESS *iPtr )
{
        int result;
        UINT8 addressSpace;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_ENABLE_LM_IMAGE,
                                        (ULONG)iPtr,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_disableLocationMon
*
* Enables TSI148/Universe II location monitor image window.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_disableLocationMon( void )
{
        int result;
        UINT8 addressSpace;

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_DISABLE_LM_IMAGE,
                                        0,VME_KERNEL_SPACE);
                }
                else
                {
                        DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
                DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}


/******************************************************************************
*
* vmekrn_enableCsrImage
*
* Enables the given CR/CSR image.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_enableCsrImage( UINT8 imageNumber )
{
        int result;
        UINT8 addressSpace;
        DBMSG("%s Entered",__FUNCTION__);

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_ENABLE_CSR_IMAGE,
				(ULONG)imageNumber,VME_KERNEL_SPACE);
                }
                else
                {
			DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
		DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_disableCsrImage
*
* Disables the given CR/CSR image.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_disableCsrImage( UINT8 imageNumber )
{
	int result;
        UINT8 addressSpace;
        DBMSG("%s Entered",__FUNCTION__);

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_ioctl(IOCTL_EN_DISABLE_CSR_IMAGE,
						(ULONG)imageNumber,VME_KERNEL_SPACE);
                }
                else
                {
			DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
        }
        else
        {
		DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}


/******************************************************************************
*
* vmekrn_readRegister
*
* Reads a TSI148/UniverseII device register at the given offset.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_readRegister( UINT32 offset, UINT32 *reg )
{
	int result;
        UINT8 addressSpace;
        DBMSG("%s Entered",__FUNCTION__);

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                	result = bridge_readRegs( offset ,(UINT8*)reg , 1 );
                }
                else
                {
			DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
                if ( result >= 0 )
                {
                        result = 0;
                }
        }
        else
        {
		DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;
}

/******************************************************************************
*
* vmekrn_writeRegister
*
* Writes to a TSI148/UNiverseII device register at the given offset.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_writeRegister( UINT32 offset, UINT32 reg )
{
	int result;
        UINT8 addressSpace;
        DBMSG("%s Entered",__FUNCTION__);

        /*Check if the device is acquired for kernel API usage*/
        result = vmedrv_getDeviceStat(VME_CONTROL,&addressSpace);

        if( result != 0 )
        {
                if ( addressSpace == VME_KERNEL_SPACE )
                {
                        result = bridge_writeRegs( offset , (UINT8*)&reg , 1);
                }
                else
                {
			DBMSG("Device is acquired by userspace API");
                        result = (-VME_EBUSY);
                }
                if ( result >= 0 )
                {
                        result = 0;
                }
        }
        else
        {
		DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return  result;

}

/***************************** PCI Image functions *****************************/

/******************************************************************************
*
* vmekrn_enablePciImage
*
* Enables a TSI148/UniverseII PCI image window.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_enablePciImage( UINT32 imageNumber, EN_PCI_IMAGE_DATA *iPtr )
{
	int result;
	UINT8 addressSpace=0;

	if ( (imageNumber > VME_CONTROL) && (imageNumber <= VME_LSI7) )
	{
        	/*Check if the device is acquired for kernel API usage*/
		result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

		if( result != 0 )
		{
			if( result == VME_KERNEL_SPACE )
			{
				result = bridge_ioctlPciImage( imageNumber, 
				IOCTL_EN_ENABLE_PCI_IMAGE, (ULONG) iPtr ,VME_KERNEL_SPACE);
			}
			else
			{
				DBMSG("Device is acquired by userspace API");
				result= (-VME_EBUSY);
			}
		}
		else
		{
			DBMSG("Device not acquired");
			result = (-VME_EPERM);
		}
	}
	else
	{
		DBMSG("Invalid device number");
		result = (-VME_EINVAL);
	}

	return result;
}

/******************************************************************************
*
* vmekrn_PciImageAddr
*
* Obtains TSI148/UniverseII PCI image's Address.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_PciImageAddr( UINT32 imageNumber, ULONG *iPtr )
{
	int result;
	UINT8 addressSpace=0;

	if ( (imageNumber > VME_CONTROL) && (imageNumber <= VME_LSI7) )
	{
        	/*Check if the device is acquired for kernel API usage*/
		result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

		if( result != 0 )
		{
			if( result == VME_KERNEL_SPACE )
			{
				result = bridge_ioctlPciImage( imageNumber, 
				IOCTL_EN_PCI_IMAGE_ADDR, (ULONG) iPtr ,VME_KERNEL_SPACE);
			}
			else
			{
				DBMSG("Device is acquired by userspace API");
				result= (-VME_EBUSY);
			}
		}
		else
		{
			DBMSG("Device not acquired");
			result = (-VME_EPERM);
		}
	}
	else
	{
		DBMSG("Invalid device number");
		result = (-VME_EINVAL);
	}

	return result;
}

/******************************************************************************
*
* vmekrn_disablePciImage
*
* Disables a TSI148/Universe II PCI image window.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_disablePciImage( UINT32 imageNumber )
{
	int result;
	UINT8 addressSpace=0;

        if ( (imageNumber > VME_CONTROL) && (imageNumber <= VME_LSI7) )
        {
        	/*Check if the device is acquired for kernel API usage*/
                result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( result == VME_KERNEL_SPACE )
                        {
				result = bridge_ioctlPciImage( imageNumber,
                                        IOCTL_EN_DISABLE_PCI_IMAGE, 0,VME_KERNEL_SPACE);
                        }
                        else
                        {
				DBMSG("Device is acquired by userspace API");
                                result= (-VME_EBUSY);
                        }
                }
                else
                {
			DBMSG("Device not acquired");
                        result = (-VME_EPERM);
                }
        }
        else
        {
		DBMSG("Invalid device number");
                result = (-VME_EINVAL);
        }

        return result;
}

/***************************** VME Image functions *****************************/

/******************************************************************************
*
* vmekrn_enableVmeImage
*
* Enables a TSI148/UniverseII VME image window.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_enableVmeImage( UINT32 imageNumber, EN_VME_IMAGE_DATA *iPtr )
{
	int result;
	UINT8 addressSpace=0;

	if ( (imageNumber > VME_LSI7) && (imageNumber <= VME_VSI7) )	
	{
        	/*Check if the device is acquired for kernel API usage*/
		result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( result == VME_KERNEL_SPACE )
                        {
				result = bridge_ioctlVmeImage( imageNumber, 
				IOCTL_EN_ENABLE_VME_IMAGE, (ULONG) iPtr ,VME_KERNEL_SPACE);
			}
			else
			{
				DBMSG("Device is acquired by userspace API");
				result= (-VME_EBUSY);
			}
		}
		else
		{
			DBMSG("Device not acquired");
			result= (-VME_EPERM);
		}
	}
	else
	{
		DBMSG("Invalid device number");
		result = (-VME_EINVAL);
	}

	return result;
}

/******************************************************************************
*
* vmekrn_VmeImageAddr
*
* Obtains TSI148/UniverseII VME image's Address.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_VmeImageAddr( UINT32 imageNumber, ULONG *iPtr )
{
	int result;
	UINT8 addressSpace=0;

	if ( (imageNumber > VME_LSI7) && (imageNumber <= VME_VSI7) )	
	{
        	/*Check if the device is acquired for kernel API usage*/
		result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( result == VME_KERNEL_SPACE )
                        {
				result = bridge_ioctlVmeImage( imageNumber, 
				IOCTL_EN_VME_IMAGE_ADDR, (ULONG) iPtr ,VME_KERNEL_SPACE);
			}
			else
			{
				DBMSG("Device is acquired by userspace API");
				result= (-VME_EBUSY);
			}
		}
		else
		{
			DBMSG("Device not acquired");
			result= (-VME_EPERM);
		}
	}
	else
	{
		DBMSG("Invalid device number");
		result = (-VME_EINVAL);
	}

	return result;
}

/******************************************************************************
*
* vmekrn_disableVmeImage
*
* Disables a TSI148/Universe II VME image window.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_disableVmeImage( UINT32 imageNumber )
{
	int result;
        UINT8 addressSpace=0;

        if ( (imageNumber > VME_LSI7) && (imageNumber <= VME_VSI7) )
        {
        	/*Check if the device is acquired for kernel API usage*/
                result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( result == VME_KERNEL_SPACE )
                        {
                                result = bridge_ioctlVmeImage( imageNumber,
                                        IOCTL_EN_DISABLE_VME_IMAGE, 0,VME_KERNEL_SPACE);
                        }
                        else
                        {
				DBMSG("Device is acquired by userspace API");
                                result= (-VME_EBUSY);
                        }
                }
                else
                {
			DBMSG("Device not acquired");
                        result= (-VME_EPERM);
                }
        }
        else
        {
		DBMSG("Invalid device number");
                result = (-VME_EINVAL);
        }

        return result;

}

/******************************************************************************
*
* vmekrn_readImage
*
* Reads from a PCI or VME Image 
*
*
*
*
* RETURNS: number of bytes read if successful else error code.
*
******************************************************************************/
int vmekrn_readImage( UINT32 imageNumber, ULONG offset, UINT8 *buf, ULONG count )
{
	int result;
	UINT8 addressSpace=0;

	DBMSG("%s Entered",__FUNCTION__);

        /*Check if the device is acquired for kernel API usage*/
	result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

        if( result != 0 )
        {
                 if( result == VME_KERNEL_SPACE )
                 {
		 	if((imageNumber > VME_CONTROL) && (imageNumber <= VME_VSI7))
			{
				result = bridge_readImage( imageNumber,offset, buf, count );
			}
			else
			{
				DBMSG("Invalid device number");
				result = (-VME_EINVAL);
			}
		}
		else
		{
			DBMSG("Device is acquired by userspace API");
			result = (-VME_EBUSY);
		}
	}
	else
	{
		DBMSG("Device not acquired");
		result = (-VME_EPERM);
	}
	
	return result;
}

/******************************************************************************
*
* vmekrn_writeImage
*
* Write into a PCI or VME Image
*
*
*
*
* RETURNS: number of bytes  written if successful else error code.
*
******************************************************************************/
int vmekrn_writeImage( UINT32 imageNumber, ULONG offset, const UINT8 *buf, ULONG count )
{
	int result;
	UINT8 addressSpace=0;

	DBMSG("%s Entered",__FUNCTION__);
        /*Check if the device is acquired for kernel API usage*/
	result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

        if( result != 0 )
        {
                 if( result == VME_KERNEL_SPACE )
                 {
		 	if((imageNumber > VME_CONTROL) && (imageNumber <= VME_VSI7))
        		{
                		result = bridge_writeImage( imageNumber,offset, buf, count );
        		}
        		else
        		{
				DBMSG("Invalid device number");
                		result = (-VME_EINVAL);
        		}
		}
		else
		{
			DBMSG("Device is acquired by userspace API");
			result = (-VME_EBUSY);
		}
	}
	else
	{
		DBMSG("Device not acquired");
                result = (-VME_EPERM);
        }

        return result;

}

/****************************DMA Functions************************************/
/******************************************************************************
*
* vmekrn_allocDmaBuffer
*
* Allocates a DMA buffer.
*
*
*
*
* RETURNS:  0 if successful else an error code.
*
******************************************************************************/
int vmekrn_allocDmaBuffer( UINT32 imageNumber, ULONG *size )
{
	int result;
	UINT8 addressSpace=0;

	if ( imageNumber == VME_DMA0 || imageNumber == VME_DMA1 )
	{
        	/*Check if the device is acquired for kernel API usage*/
		result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

		if( result != 0 )
		{
			if( addressSpace == VME_KERNEL_SPACE )
			{
				result = bridge_ioctlDma( imageNumber,
				IOCTL_EN_ALLOC_DMA_BUFFER,(ULONG)size,VME_KERNEL_SPACE);
			}
			else
			{
				DBMSG("Device is acquired by userspace API");
				result = (-VME_EBUSY);
			}
		}
		else
		{
			DBMSG("Device not acquired");
			result = (-VME_EPERM);
		}
	}
	else
	{
		DBMSG("Invalid device number");
		result = (-VME_EINVAL);
	}
	
	return result;
}

/******************************************************************************
*
* vmekrn_getDmaBufferAddr
*
* Get the physical address of the allocated DMA buffer.
*
*
*
*
* RETURNS:  0 if successful else an error code.
*
******************************************************************************/
int vmekrn_getDmaBufferAddr( UINT32 imageNumber, ULONG *bufferAddress )
{
        int result;
        UINT8 addressSpace=0;

        if ( imageNumber == VME_DMA0 || imageNumber == VME_DMA1 )
        {
                /*Check if the device is acquired for kernel API usage*/
                result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( addressSpace == VME_KERNEL_SPACE )
                        {
                                result = bridge_ioctlDma( imageNumber,
                                IOCTL_EN_GET_DMA_BUFFER_ADDR,(ULONG)bufferAddress,VME_KERNEL_SPACE);
                        }
                        else
                        {
                                DBMSG("Device is acquired by userspace API");
                                result = (-VME_EBUSY);
                        }
                }
                else
                {
                        DBMSG("Device not acquired");
                        result = (-VME_EPERM);
                }
        }
        else
        {
                DBMSG("Invalid device number");
                result = (-VME_EINVAL);
        }

        return result;
}

/******************************************************************************
*
* vmekrn_freeDmaBuffer
*
* Free's a previously allocated DMA buffer.
*
*
*
*
* RETURNS:  0 if successful else an error code.
*
******************************************************************************/
int vmekrn_freeDmaBuffer( UINT32 imageNumber )
{
	int result;
	UINT8 addressSpace=0;

        if ( imageNumber == VME_DMA0 || imageNumber == VME_DMA1 )
        {
        	/*Check if the device is acquired for kernel API usage*/
                result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( addressSpace == VME_KERNEL_SPACE )
                        {
				result =  bridge_ioctlDma( imageNumber,
					IOCTL_EN_FREE_DMA_BUFFER,0,VME_KERNEL_SPACE );				
                        }
                        else
                        {
				DBMSG("Device is acquired by userspace API");
                                result = (-VME_EBUSY);
                        }
                }
                else
                {
			DBMSG("Device not acquired");
                        result = (-VME_EPERM);
                }
        }
        else
        {
		DBMSG("Invalid device number");
                result = (-VME_EINVAL);
        }

	return result;
}

/******************************************************************************
*
* vmekrn_dmaDirectTransfer
*
* Initiates a direct mode DMA transfer using the given parameters.
*
*
*
*
* RETURNS:  0 if successful else an error code.
*
******************************************************************************/
int vmekrn_dmaDirectTransfer( UINT32 imageNumber, EN_VME_DIRECT_TXFER *dPtr )
{
	int result;
	UINT8 addressSpace=0;

        if ( imageNumber == VME_DMA0 || imageNumber == VME_DMA1 )
        {
        	/*Check if the device is acquired for kernel API usage*/
                result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( addressSpace == VME_KERNEL_SPACE )
                        {
				result = bridge_ioctlDma( imageNumber ,
				IOCTL_EN_DMA_DIRECT_TXFER,(ULONG)dPtr,VME_KERNEL_SPACE);

                        }
                        else
                        {
				DBMSG("Device is acquired by userspace API");
                                result = (-VME_EBUSY);
                        }
                }
                else
                {
			DBMSG("Device not acquired");
                        result = (-VME_EPERM);
                }
        }
        else
        {
		DBMSG("Invalid device number");
                result = (-VME_EINVAL);
        }

	return result;
}

/******************************************************************************
*
* vmekrn_addDmaCmdPkt
*
* Adds a DMA command packet to the linked list with the given parameters.
*
*
*
*
* RETURNS:  0 if successful else an error code.
*
******************************************************************************/
int vmekrn_addDmaCmdPkt( UINT32 imageNumber, EN_VME_CMD_DATA *cmdPtr )
{
	int result;
	UINT8 addressSpace=0;

        if ( imageNumber == VME_DMA0 || imageNumber == VME_DMA1 )
        {
        	/*Check if the device is acquired for kernel API usage*/
                result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( addressSpace == VME_KERNEL_SPACE )
                        {
				result = bridge_ioctlDma( imageNumber ,
				IOCTL_EN_DMA_ADD_CMD_PKT,(ULONG)cmdPtr,VME_KERNEL_SPACE);
                        }
                        else
                        {
				DBMSG("Device is acquired by userspace API");
                                result = (-VME_EBUSY);
                        }
                }
                else
                {
			DBMSG("Device not acquired");
                        result = (-VME_EPERM);
                }
        }
        else
        {
		DBMSG("Invalid device number");
                result = (-VME_EINVAL);
        }

        return result;
}

/******************************************************************************
*
* vmekrn_clearDmaCmdPkts
*
* Clears the DMA command packet linked list.
*
*
*
*
* RETURNS:  0 if successful else an error code.
*
******************************************************************************/
int vmekrn_clearDmaCmdPkts( UINT32 imageNumber )
{
 	int result;
        UINT8 addressSpace=0;

        if ( imageNumber == VME_DMA0 || imageNumber == VME_DMA1 )
        {
        	/*Check if the device is acquired for kernel API usage*/
                result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( addressSpace == VME_KERNEL_SPACE )
                        {
				result = bridge_ioctlDma( imageNumber ,
					IOCTL_EN_DMA_CLEAR_CMD_PKTS,0,VME_KERNEL_SPACE);
                        }
                        else
                        {
				DBMSG("Device is acquired by userspace API");
                                result = (-VME_EBUSY);
                        }
                }
                else
                {
			DBMSG("Device not acquired");
                        result = (-VME_EPERM);
                }
        }
        else
        {
		DBMSG("Invalid device number");
                result = (-VME_EINVAL);
        }

        return result;
}

/******************************************************************************
*
* vmekrn_dmaListTransfer
*
* Initiates a link list mode DMA transfer using the given parameters.
*
*
*
*
* RETURNS:  0 if successful else an error code.
*
******************************************************************************/
int vmekrn_dmaListTransfer( UINT32 imageNumber, EN_VME_TXFER_PARAMS *tPtr )
{
	int result;
        UINT8 addressSpace=0;

        if ( imageNumber == VME_DMA0 || imageNumber == VME_DMA1 )
        {
        	/*Check if the device is acquired for kernel API usage*/
                result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( addressSpace == VME_KERNEL_SPACE )
                        {
				result = bridge_ioctlDma( imageNumber ,
				IOCTL_EN_DMA_LIST_TXFER,(ULONG)tPtr,VME_KERNEL_SPACE);

                        }
                        else
                        {
				DBMSG("Device is acquired by userspace API");
                                result = (-VME_EBUSY);
                        }
                }
                else
                {
			DBMSG("Device not acquired");
                        result = (-VME_EPERM);
                }
        }
        else
        {
		DBMSG("Invalid device number");
                result = (-VME_EINVAL);
        }

        return result;
}


/******************************************************************************
*
* vmekrn_readDma
*
* Read using DMA
*
*
*
*
* RETURNS: number of bytes read if successful else error code.
*
******************************************************************************/
int vmekrn_readDma( UINT32 imageNumber, ULONG offset, UINT8 *buf, ULONG count )
{
	int result;
	UINT8 addressSpace=0;

	DBMSG("%s Entered",__FUNCTION__);

	if ( imageNumber == VME_DMA0 || imageNumber == VME_DMA1 )
        {
        	/*Check if the device is acquired for kernel API usage*/
                result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( addressSpace == VME_KERNEL_SPACE )
                        {
				result = bridge_readDma( imageNumber ,offset, buf,count );
			}
			else
			{
				DBMSG("Device is acquired by userspace API");
				result = (-VME_EBUSY);
			}
		}
		else
		{
			DBMSG("Device not acquired");
			result = (-VME_EPERM);
		}
 	}
        else
        {
		DBMSG("Invalid device number");
                result = (-VME_EINVAL);
        }

	return result;		
}

/******************************************************************************
*
* vmekrn_writeDma
*
* Write using DMA
*
*
*
*
* RETURNS: number of bytes written if successful else error code.
*
******************************************************************************/
int vmekrn_writeDma( UINT32 imageNumber,ULONG offset, const UINT8 *buf, ULONG count )
{
	int result;
        UINT8 addressSpace=0;

        DBMSG("%s Entered",__FUNCTION__);

        if ( imageNumber == VME_DMA0 || imageNumber == VME_DMA1 )
        {
        	/*Check if the device is acquired for kernel API usage*/
                result = vmedrv_getDeviceStat(imageNumber,&addressSpace);

                if( result != 0 )
                {
                        if( addressSpace == VME_KERNEL_SPACE )
                        {
                                result = bridge_writeDma( imageNumber ,offset, buf,count );
                        }
                        else
                        {
				DBMSG("Device is acquired by userspace API");
                                result = (-VME_EBUSY);
                        }
                }
                else
                {
			DBMSG("Device not acquired");
                        result = (-VME_EPERM);
                }
        }
        else
        {
		DBMSG("Invalid device number");
                result = (-VME_EINVAL);
        }

        return result;
}

int vmekrn_getInstanceCount( INT32 minorNum, UINT32* arg )
{
	return vmedrv_getInstanceCount( minorNum, (ULONG)arg );
}

/******************************************************************************
*
* vmekrn_getGeographicAddr
*
* Obtains the slot number of board with TSI148 bridge.
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_getGeographicAddr( INT32 minorNum, UINT8 *slotNumber )
{
        int result;
        UINT8 addressSpace=0;

        if(minorNum == VME_CONTROL)
        {
          /*Check if the device is acquired for kernel API usage*/
          result = vmedrv_getDeviceStat(minorNum,&addressSpace);

          if( result != 0 )
          {
                  if ( addressSpace == VME_KERNEL_SPACE )
                  {
                          result = bridge_ioctl(IOCTL_GET_GEO_ADDR,
                                          (ULONG)slotNumber,VME_KERNEL_SPACE);
                  }
                  else
                  {
                          DBMSG("Device is acquired by userspace API");
                          result = (-VME_EBUSY);
                  }
          }
          else
          {
                  DBMSG("Device not acquired");
                  result = (-VME_EPERM);
          }
        }
        else
        {
          DBMSG("Invalid device number");
          result = (-VME_EINVAL);
        }
        return  result;
}

/******************************************************************************
*
* vmekrn_clearSysFail
*
* Clear SYSFAIL_I signal
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_clearSysFail( INT32 minorNum )
{
        int result;
        UINT8 addressSpace=0;

        if(minorNum == VME_CONTROL)
        {
          /*Check if the device is acquired for kernel API usage*/
          result = vmedrv_getDeviceStat(minorNum, &addressSpace);

          if( result != 0 )
          {
                  if ( addressSpace == VME_KERNEL_SPACE )
                  {
                          result = bridge_ioctl(IOCTL_EN_CLEAR_SYSFAIL,
                                          0,VME_KERNEL_SPACE);
                  }
                  else
                  {
                          DBMSG("Device is acquired by userspace API");
                          result = (-VME_EBUSY);
                  }
          }
          else
          {
                  DBMSG("Device not acquired");
                  result = (-VME_EPERM);
          }
        }
        else
        {
          DBMSG("Invalid device number");
          result = (-VME_EINVAL);
        }
        return  result;
}

/******************************************************************************
*
* vmekrn_setSysFail
*
* Sets SYSFAIL_O signal
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_setSysFail( INT32 minorNum )
{
        int result;
        UINT8 addressSpace=0;

        if(minorNum == VME_CONTROL)
        {
          /*Check if the device is acquired for kernel API usage*/
          result = vmedrv_getDeviceStat(minorNum, &addressSpace);

          if( result != 0 )
          {
                  if ( addressSpace == VME_KERNEL_SPACE )
                  {
                          result = bridge_ioctl(IOCTL_EN_SET_SYSFAIL,
                                          0,VME_KERNEL_SPACE);
                  }
                  else
                  {
                          DBMSG("Device is acquired by userspace API");
                          result = (-VME_EBUSY);
                  }
          }
          else
          {
                  DBMSG("Device not acquired");
                  result = (-VME_EPERM);
          }
        }
        else
        {
          DBMSG("Invalid device number");
          result = (-VME_EINVAL);
        }
        return  result;
}

/******************************************************************************
*
* vmekrn_getSysFail
*
* Gets SYSFAIL_I signal
*
*
*
*
* RETURNS: 0 if successful else an error code.
*
******************************************************************************/
int vmekrn_getSysFail( INT32 minorNum, UINT8* status )
{
        int result;
        UINT8 addressSpace=0;

        if(minorNum == VME_CONTROL)
        {
          /*Check if the device is acquired for kernel API usage*/
          result = vmedrv_getDeviceStat(minorNum, &addressSpace);

          if( result != 0 )
          {
                  if ( addressSpace == VME_KERNEL_SPACE )
                  {
                          result = bridge_ioctl(IOCTL_EN_GET_SYSFAIL,
                                          (ULONG)status,VME_KERNEL_SPACE);
                  }
                  else
                  {
                          DBMSG("Device is acquired by userspace API");
                          result = (-VME_EBUSY);
                  }
          }
          else
          {
                  DBMSG("Device not acquired");
                  result = (-VME_EPERM);
          }
        }
        else
        {
          DBMSG("Invalid device number");
          result = (-VME_EINVAL);
        }
        return  result;
}

EXPORT_SYMBOL_VME(vmekrn_acquireDevice);
EXPORT_SYMBOL_VME(vmekrn_releaseDevice);
EXPORT_SYMBOL_VME(vmekrn_setInterruptMode);
EXPORT_SYMBOL_VME(vmekrn_enableInterrupt);
EXPORT_SYMBOL_VME(vmekrn_disableInterrupt);
EXPORT_SYMBOL_VME(vmekrn_generateInterrupt);
EXPORT_SYMBOL_VME(vmekrn_readInterruptInfo);
EXPORT_SYMBOL_VME(vmekrn_readExtInterruptInfo);
EXPORT_SYMBOL_VME(vmekrn_setStatusId);
EXPORT_SYMBOL_VME(vmekrn_waitInterrupt);
EXPORT_SYMBOL_VME(vmekrn_setByteSwap);
EXPORT_SYMBOL_VME(vmekrn_setUserAmCodes);
EXPORT_SYMBOL_VME(vmekrn_readVerrInfo);
EXPORT_SYMBOL_VME(vmekrn_readVerrInfoUserHandler);
EXPORT_SYMBOL_VME(vmekrn_enableLocationMon);
EXPORT_SYMBOL_VME(vmekrn_disableLocationMon);
EXPORT_SYMBOL_VME(vmekrn_writeDma);
EXPORT_SYMBOL_VME(vmekrn_readDma);
EXPORT_SYMBOL_VME(vmekrn_writeImage);
EXPORT_SYMBOL_VME(vmekrn_readImage);
EXPORT_SYMBOL_VME(vmekrn_writeRegister);
EXPORT_SYMBOL_VME(vmekrn_readRegister);
EXPORT_SYMBOL_VME(vmekrn_enableCsrImage);
EXPORT_SYMBOL_VME(vmekrn_disableCsrImage);
EXPORT_SYMBOL_VME(vmekrn_enableRegAccessImage);
EXPORT_SYMBOL_VME(vmekrn_disableRegAccessImage);
EXPORT_SYMBOL_VME(vmekrn_enablePciImage);
EXPORT_SYMBOL_VME(vmekrn_disablePciImage);
EXPORT_SYMBOL_VME(vmekrn_enableVmeImage);
EXPORT_SYMBOL_VME(vmekrn_disableVmeImage);
EXPORT_SYMBOL_VME(vmekrn_allocDmaBuffer);
EXPORT_SYMBOL_VME(vmekrn_freeDmaBuffer);
EXPORT_SYMBOL_VME(vmekrn_dmaDirectTransfer);
EXPORT_SYMBOL_VME(vmekrn_addDmaCmdPkt);
EXPORT_SYMBOL_VME(vmekrn_clearDmaCmdPkts);
EXPORT_SYMBOL_VME(vmekrn_dmaListTransfer);
EXPORT_SYMBOL_VME(vmekrn_getDmaBufferAddr);
EXPORT_SYMBOL_VME(vmekrn_clearStats);
EXPORT_SYMBOL_VME(vmekrn_getStats);
EXPORT_SYMBOL_VME(vmekrn_getBoardCap);
EXPORT_SYMBOL_VME(vmekrn_PciImageAddr);
EXPORT_SYMBOL_VME(vmekrn_VmeImageAddr);
EXPORT_SYMBOL_VME(vmekrn_registerInterrupt);
EXPORT_SYMBOL_VME(vmekrn_removeInterrupt);
EXPORT_SYMBOL_VME(vmekrn_getInstanceCount);
EXPORT_SYMBOL_VME(vmekrn_getGeographicAddr);
EXPORT_SYMBOL_VME(vmekrn_readInterruptEnhInfo);
EXPORT_SYMBOL_VME(vmekrn_setVectorCaptureMode);
EXPORT_SYMBOL_VME(vmekrn_getSysFail);
EXPORT_SYMBOL_VME(vmekrn_setSysFail);
EXPORT_SYMBOL_VME(vmekrn_clearSysFail);
