/******************************************************************************
*
* Filename: 	example_drv.c
* 
* Description:	Example driver demonstrating the usage of VME kernel APIs
*
* Copyright 2000-2009 Concurrent Technologies.
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


#ifndef MODULE
#include <linux/init.h>
#define INIT_FUNC 		static int init_vmedriver
#define CLEANUP_FUNC 	static void cleanup_vmedriver
#else
#define INIT_FUNC 		int init_module
#define CLEANUP_FUNC 	void cleanup_module
#endif

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
/*#include <linux/smp_lock.h>*/
#include <linux/fs.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,10))
MODULE_LICENSE("Proprietary");
#endif

#include "example_drv.h" 
#include "vme_api_en.h"

static int vmeMajor = 0; /* VME_DRIVER_MAJOR; 0 = use dynamic allocation */
#if ((HAVE_UNLOCKED_IOCTL) && (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)))
static long vmeexdrv_unlockedioctl( struct file *file,
						unsigned int cmd, unsigned long arg );
#else
static int vmeexdrv_ioctl( struct inode *inode, struct file *file,
                                                unsigned int cmd, unsigned long arg );
#endif

static int vmeexdrv_open( struct inode *inode, struct file *file );
static int vmeexdrv_close( struct inode *inode, struct file *file );


EN_VME_INT_DATA iDataPtr[26];
int *tstPtr, a = 10;
int *tstPtr1, b = 20;
static int intCount[26] = {0};

/* device methods declared using tagged format */
/* note: GNU compiler specific */ 
static struct file_operations vme_fops = 
{
#if ((HAVE_UNLOCKED_IOCTL) && (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)))
	.unlocked_ioctl =		vmeexdrv_unlockedioctl,
#else
	.ioctl=		vmeexdrv_ioctl,
#endif
	.open=		vmeexdrv_open,
	.release=	vmeexdrv_close
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
  DEFINE_MUTEX(vmeDevCtrl);
#else
  DECLARE_MUTEX(vmeDevCtrl);
#endif

/******************************************************************************
*
* vmeexdrv_lsi
*
* The following routine shows how to enable a PCI image and access the same
* 
*
*
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmeexdrv_lsi( void )
{
        EN_PCI_IMAGE_DATA idata;
        int i=0,result;
	UINT8 buffer[8];

	memset((char *)&idata,0,sizeof(EN_PCI_IMAGE_DATA));

	/*Acquire the ownership to the PCI Image 0*/
        result = vmekrn_acquireDevice(VME_LSI0);

        if( result < 0 )
        {
                printk(KERN_ERR "Unable to acquire PCI image 0\n");
                return result;
        }
        else
        {
                idata.pciAddress=0;
                idata.pciAddressUpper=0;
                idata.vmeAddress=0x2000000;
                idata.vmeAddressUpper=0;
                idata.size = 0x10000;
                idata.sizeUpper = 0;
                idata.readPrefetch=1;		/* Enable read prefetch */
                idata.prefetchSize= 3;  	/* Set the memory prefetch
					  	   cache line size to 16 */
                idata.sstMode = 0;
                idata.dataWidth = EN_VME_D32;
                idata.addrSpace = EN_VME_A32;
                idata.type = EN_LSI_DATA; 	/* data AM code */
                idata.mode = EN_LSI_USER; 	/* non-previledged*/
                idata.vmeCycle = 0;		/* SCT vme cycle type*/
                idata.sstbSel =0;
		idata.ioremap =1;		/* This parameter needs
						  to be always set to 1*/

                result = vmekrn_enablePciImage( VME_LSI0, &idata );

                if ( result < 0 )
                {
                        printk(KERN_DEBUG "Error - failed to enable PCI image 0 \n");
                }
                else
                {
                        printk(KERN_DEBUG "PCI image 0 enabled, reading data...\n");

			result = vmekrn_readImage( VME_LSI0, 0, buffer, 8 );

                        if ( result < 0 )
                        {
                                printk(KERN_DEBUG "Error - failed to read data\n");
                        }
                        else
                        {
                                for ( i = 0; i < 8; i++ )
                                {
                                        printk(KERN_DEBUG "0x%02X\n", buffer[i] );
                                }
                        }

                        result = vmekrn_disablePciImage( VME_LSI0 );

                        if ( result < 0 )
                        {
                                printk(KERN_ERR "Error - failed to disable PCI image 0 \n");
                        }
                }

                vmekrn_releaseDevice( VME_LSI0 );
        }
 
	return result;
}

/******************************************************************************
*
* vmeexdrv_vsi
*
* The following routine shows how to enable a VME image and access the same
*
*
*
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmeexdrv_vsi( void )
{
	int result;
	int i;
	UINT8 buffer[8];
	EN_VME_IMAGE_DATA idata;

    	memset((char *)&idata,0,sizeof(EN_VME_IMAGE_DATA));

 	/*Acquire the ownership to the VME Image 0*/
        result = vmekrn_acquireDevice(VME_VSI0);


    	if ( result < 0 )
    	{   
        	printk("Error - failed to acquire VME Image 0\n");    
    	}
    	else
    	{
        	idata.vmeAddress = 0x2000000;
		idata.vmeAddressUpper =0;       
        	idata.size = 0x10000;
		idata.sizeUpper =0;
		idata.threshold = 1;
		idata.virtualFifoSize = 0;
		idata.sstMode = 0;
		idata.vmeCycle = 0;
		idata.addrSpace = EN_VME_A32;
        	idata.type = EN_VSI_BOTH;  /* both program and data */
        	idata.mode = EN_VSI_BOTH;  /* both supervisor and non-privileged */
        	idata.ioremap = 1;         /* This parameter needs
                                              to be always set to 1*/

		/*Enable the VME image*/
        	result = vmekrn_enableVmeImage( VME_VSI0, &idata );

		if ( result < 0 )
                {
                        printk(KERN_DEBUG "Error - failed to enable VME image 0\n");
                }
                else
                {
                        printk(KERN_DEBUG "VME image 0 enabled, reading data...\n");

                        result = vmekrn_readImage( VME_VSI0, 0, buffer, 8 );

                        if ( result < 0 )
                        {
                                printk(KERN_DEBUG "Error - failed to read data\n");
                        }
                        else
                        {
                                for ( i = 0; i < 8; i++ )
                                {
                                        printk(KERN_DEBUG "0x%02X\n", buffer[i] );
                                }
                        }

                        result = vmekrn_disableVmeImage( VME_VSI0 );

                        if ( result < 0 )
                        {
                                printk(KERN_ERR "Error - failed to disable VME image 0 \n");
                        }
                }

                vmekrn_releaseDevice( VME_VSI0 );

	}

	return result;
}

/******************************************************************************
*
* vmeexdrv_dmaDirect
*
* The following routine shows how to perform a direct mode DMA data transfer
*
*
*
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmeexdrv_dmaDirect( void )
{
	UINT8 buffer[8];
	EN_VME_DIRECT_TXFER tdata;
	ULONG size;
	int result;
	int i;

    	memset((char *)&tdata,0,sizeof(EN_VME_DIRECT_TXFER));


	 /*Acquire the ownership to DMA0 */
        result = vmekrn_acquireDevice(VME_DMA0);

        if ( result < 0 )
        {
                printk("Error - failed to acquire DMA 0\n");
        }
        else
        {
        	size = 4096;

        	result = vmekrn_allocDmaBuffer( VME_DMA0, &size );

        	if ( result < 0 )
        	{
            		printk("Error - failed to allocate DMA buffer \n");
		}
        	else
        	{
            		printk("DMA buffer allocted, %lu bytes available\n",size);

            		tdata.direction = EN_DMA_READ;
		        tdata.offset = 0;       /* start of DMA buffer */
            		tdata.size = 4096;  /* read 4KB */
            		tdata.vmeAddress = 0x2000000;

            		tdata.txfer.timeout = 200;  /* 2 second timeout */
	    		tdata.txfer.vmeBlkSize = 0;
	   	 	tdata.txfer.vmeBackOffTimer = 0;
	    		tdata.txfer.pciBlkSize = 0;
	    		tdata.txfer.pciBackOffTimer = 0;		

	    		tdata.access.sstMode = 0;
	    		tdata.access.vmeCycle = 0;
	    		tdata.access.sstbSel = 0;
            		tdata.access.dataWidth = EN_VME_D32;
            		tdata.access.addrSpace = EN_VME_A32;
            		tdata.access.type = EN_LSI_DATA;   /* data AM code */
            		tdata.access.mode = EN_LSI_USER;   /* non-privileged */

            		result = vmekrn_dmaDirectTransfer( VME_DMA0, &tdata );

            		if ( result < 0 )
            		{
                		printk("Error - DMA transfer failed \n");
			}
            		else
            		{
                		printk("DMA transfer successful\n");

				result = vmekrn_readDma( VME_DMA0, 0, buffer, 8 );

                        	if ( result < 0 )
                        	{
                                	printk("Error - failed to read data\n");
                        	}
                        	else
                        	{
                                	for ( i = 0; i < 8; i++ )
                                	{
                                        	printk("0x%02X\n", buffer[i] );
                                	}
				}
                        }

            		result = vmekrn_freeDmaBuffer( VME_DMA0 );

            		if ( result < 0 )
            		{
                		printk("Error - failed to free DMA buffer \n");
			}
		}
	
		vmekrn_releaseDevice( VME_DMA0 );
    	}

	return result;
}

/******************************************************************************
*
* vmeexdrv_dmaList
*
* The following routine shows how to perform a link-list mode DMA data transfer
*
*
*
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmeexdrv_dmaList( void )
{
    	UINT8 buffer[8];
    	EN_VME_CMD_DATA cmddata;
    	EN_VME_TXFER_PARAMS tdata;
    	ULONG size;
    	int result;
    	int i;

    	memset((char *)&tdata,0,sizeof(EN_VME_TXFER_PARAMS));
    	memset((char *)&cmddata,0,sizeof(EN_VME_CMD_DATA));

	 /*Acquire the ownership to the DMA0*/
        result = vmekrn_acquireDevice(VME_DMA0);

        if ( result < 0 )
        {
                printk("Error - failed to acquire DMA 0\n");
        }
        else
        {
        	size = 4096;
        	result = vmekrn_allocDmaBuffer( VME_DMA0, &size );

        	if ( result < 0 )
        	{
            		printk("Error - failed to allocate DMA buffer\n");
		}
        	else
        	{
            		printk("DMA buffer allocted, %lu bytes available\n",size);

            		/* clear any existing command packets */
            		result = vmekrn_clearDmaCmdPkts( VME_DMA0 );

            		if ( result < 0 )
            		{
                		printk("Error - failed clear list \n");
            		}
            		else
            		{
                		cmddata.direction = EN_DMA_READ;
                		cmddata.offset = 0; /* start of DMA buffer */
                		cmddata.size = 4096;    /* read 4KB */
                		cmddata.vmeAddress = 0x2000000;
				cmddata.vmeAddressUpper = 0;

				cmddata.access.sstMode = 0;
            			cmddata.access.vmeCycle = 0;
            			cmddata.access.sstbSel = 0;
            			cmddata.access.dataWidth = EN_VME_D32;
            			cmddata.access.addrSpace = EN_VME_A32;
            			cmddata.access.type = EN_LSI_DATA;   /* data AM code */
            			cmddata.access.mode = EN_LSI_USER;   /* non-privileged */

                		result = vmekrn_addDmaCmdPkt( VME_DMA0, &cmddata );

                		if ( result < 0 )
                		{
                     			printk("Error - failed to add command packet\n");
				}
                		else
                		{
		    			tdata.timeout = 200;  /* 2 second timeout */
            	    			tdata.vmeBlkSize = 0;
            	    			tdata.vmeBackOffTimer = 0;
            	    			tdata.pciBlkSize = 0;
            	    			tdata.pciBackOffTimer = 0;

                    			result = vmekrn_dmaListTransfer( VME_DMA0, &tdata );

                    			if ( result < 0 )
                    			{
                        			printk("Error - DMA transfer failed\n");
					}
                    			else
                    			{
                        			printk("DMA transfer successful\n");

                        			/* read and display the first 8 bytes */
                        			result = vmekrn_readDma( VME_DMA0, 0, buffer, 8 );

                        			if ( result > 0 )
                        			{
                            				for ( i = 0; i < 8; i++ )
                            				{
                                				printk("0x%02X\n", buffer[i] );
                            				}
                        			}		
                    			}
                		}
			}

            		vmekrn_clearDmaCmdPkts( VME_DMA0 );

            		result = vmekrn_freeDmaBuffer( VME_DMA0 );

            		if ( result < 0 )
            		{
                		printk("Error - failed to free DMA buffer \n");
            		}
		}

        	vmekrn_releaseDevice( VME_DMA0 );
    	}

	return result;
}

/******************************************************************************
*
* vmeuser_interrupt1
*
* The following routine is user specific, has to be interrupt safe.
* User has to remove the registered interrupt handler
* at exit of application.
*
*
* RETURNS: None
*
******************************************************************************/
static void vmeuser_interrupt1( void* sPtr )
{
	EN_VME_INT_USR_DATA *lPtr;

	lPtr = (EN_VME_INT_USR_DATA*)sPtr;
	intCount[lPtr->intNum]++;
	/*printk("CALLBACK INT %d %d\n", lPtr->intNum, *(int*)(lPtr->usrPtr));*/
	/* lPtr->intVec, lPtr->usrPtr provides the interrupt */
	/* vector and user sent Pointer */
}

/******************************************************************************
*
* vmeuser_interrupt2
*
* The following routine is user specific, has to be interrupt safe.
* User has to remove the registered interrupt handler
* at exit of application.
*
*
* RETURNS: None
*
******************************************************************************/
static void vmeuser_interrupt2( void* sPtr )
{
	EN_VME_INT_USR_DATA *lPtr;

	lPtr = (EN_VME_INT_USR_DATA*)sPtr;
	intCount[lPtr->intNum]++;
	/*printk("CALLBACK INT %d %d\n", lPtr->intNum, *(int*)(lPtr->usrPtr));*/
	/* lPtr->intVec, lPtr->usrPtr provides the interrupt */
	/* vector and user sent Pointer */
}

/******************************************************************************
*
* vmeuser_interrupt3
*
* The following routine is user specific, has to be interrupt safe.
* User has to remove the registered interrupt handler
* at exit of application.
*
*
* RETURNS: None
*
******************************************************************************/
static void vmeuser_interrupt3( void* sPtr )
{

	EN_VME_INT_USR_DATA *lPtr;

	lPtr = (EN_VME_INT_USR_DATA*)sPtr;
	intCount[lPtr->intNum]++;
	/* lPtr->intVec, lPtr->usrPtr provides the interrupt */
	/* vector and user sent Pointer */
}

/******************************************************************************
*
* vmeuser_interrupt4
*
* The following routine is user specific, has to be interrupt safe.
* User has to remove the registered interrupt handler
* at exit of application.
*
*
* RETURNS: None
*
******************************************************************************/
static void vmeuser_interrupt4( void* sPtr )
{

	EN_VME_INT_USR_DATA *lPtr;

	lPtr = (EN_VME_INT_USR_DATA*)sPtr;
	intCount[lPtr->intNum]++;
	/* lPtr->intVec, lPtr->usrPtr provides the interrupt */
	/* vector and user sent Pointer */
}

/******************************************************************************
*
* vmeuser_interrupt5
*
* The following routine is user specific, has to be interrupt safe.
* User has to remove the registered interrupt handler
* at exit of application.
*
*
* RETURNS: None
*
******************************************************************************/
static void vmeuser_interrupt5( void* sPtr )
{
	EN_VME_INT_USR_DATA *lPtr;

	lPtr = (EN_VME_INT_USR_DATA*)sPtr;
	intCount[lPtr->intNum]++;
	/* lPtr->intVec, lPtr->usrPtr provides the interrupt */
	/* vector and user sent Pointer */
}

/******************************************************************************
*
* vmeuser_interrupt6
*
* The following routine is user specific, has to be interrupt safe.
* User has to remove the registered interrupt handler
* at exit of application.
*
*
* RETURNS: None
*
******************************************************************************/
static void vmeuser_interrupt6( void* sPtr )
{

	EN_VME_INT_USR_DATA *lPtr;

	lPtr = (EN_VME_INT_USR_DATA*)sPtr;
	intCount[lPtr->intNum]++;
	/* lPtr->intVec, lPtr->usrPtr provides the interrupt */
	/* vector and user sent Pointer */
}

/******************************************************************************
*
* vmeuser_interrupt7
*
* The following routine is user specific, has to be interrupt safe.
* User has to remove the registered interrupt handler
* at exit of application.
*
*
* RETURNS: None
*
******************************************************************************/
static void vmeuser_interrupt7( void* sPtr )
{

	EN_VME_INT_USR_DATA *lPtr;

	lPtr = (EN_VME_INT_USR_DATA*)sPtr;
	intCount[lPtr->intNum]++;
	/* lPtr->intVec, lPtr->usrPtr provides the interrupt */
	/* vector and user sent Pointer */
}

/******************************************************************************
*
* vmeuser_interrupt12
*
* The following routine is user specific, has to be interrupt safe.
* User has to remove the registered interrupt handler
* at exit of application.
*
*
* RETURNS: None
*
******************************************************************************/
#if 0
static void vmeuser_interrupt12( void* sPtr )
{

        EN_VME_INT_USR_DATA *lPtr;
        unsigned long address;
        unsigned char direction, amCode;

        lPtr = (EN_VME_INT_USR_DATA*)sPtr;
        intCount[lPtr->intNum]++;
        /* vmekrn_readVerrInfoUserHandler( &address, &direction, &amCode); Only for TSI148 */
        /* CALLBACK BERR */
        /* lPtr->intVec, lPtr->usrPtr provides the interrupt */
        /* vector and user sent Pointer */
}
#endif

/******************************************************************************
*
* vmeuser_interrupt23
*
* The following routine is user specific, has to be interrupt safe.
* User has to remove the registered interrupt handler
* at exit of application.
*
*
* RETURNS: None
*
******************************************************************************/
static void vmeuser_interrupt23( void* sPtr )
{

	EN_VME_INT_USR_DATA *lPtr;

	lPtr = (EN_VME_INT_USR_DATA*)sPtr;
	intCount[lPtr->intNum]++;
	/* lPtr->intVec, lPtr->usrPtr provides the interrupt */
	/* vector and user sent Pointer */
}

/******************************************************************************
*
* vmeexdrv_regusrInt
*
* The following routine shows how a kernel driver can register an interrupt
* handler with the VME driver and remove the same
*
*
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmeexdrv_regusrInt( void )
{
	int result, i;
	EN_VME_INT_DATA *iPtr;
	tstPtr = &a;
	tstPtr1 = &b;

	 /*Acquire the ownership to the control device*/
        result = vmekrn_acquireDevice(VME_CONTROL);

        if ( result < 0 )
        {
                printk("Error - failed to acquire control device\n");
        }
        else
        {

              if(result == 0)
              {
                      iPtr = &iDataPtr[0];
                      iPtr->intNum = 1;
                      iPtr->userInt = &vmeuser_interrupt1;
                      iPtr->usrData.usrPtr = tstPtr;
                      result = vmekrn_registerInterrupt( iPtr );
              }
              if(result == 0)
              {
                      iPtr = &iDataPtr[1];
                      iPtr->intNum = 2;
                      iPtr->userInt = &vmeuser_interrupt2;
                      iPtr->usrData.usrPtr = tstPtr1;
                      result = vmekrn_registerInterrupt( iPtr );
              }
              if(result == 0)
              {
                      iPtr = &iDataPtr[2];
                      iPtr->intNum = 3;
                      iPtr->userInt = &vmeuser_interrupt3;
                      result = vmekrn_registerInterrupt( iPtr );
              }
              if(result == 0)
              {
                      iPtr = &iDataPtr[3];
                      iPtr->intNum = 4;
                      iPtr->userInt = &vmeuser_interrupt4;
                      result = vmekrn_registerInterrupt( iPtr );
              }
              if(result == 0)
              {
                      iPtr = &iDataPtr[4];
                      iPtr->intNum = 5;
                      iPtr->userInt = &vmeuser_interrupt5;
                      result = vmekrn_registerInterrupt( iPtr );
              }
              if(result == 0)
              {
                      iPtr = &iDataPtr[5];
                      iPtr->intNum = 6;
                      iPtr->userInt = &vmeuser_interrupt6;
                      result = vmekrn_registerInterrupt( iPtr );
              }
              if(result == 0)
              {
                      iPtr = &iDataPtr[6];
                      iPtr->intNum = 7;
                      iPtr->userInt = &vmeuser_interrupt7;
                      result = vmekrn_registerInterrupt( iPtr );
              }
              /* if(result == 0)
              {
                      iPtr = &iDataPtr[11];
                      iPtr->intNum = 12;
                      iPtr->userInt = &vmeuser_interrupt12;
                      result = vmekrn_registerInterrupt( iPtr );
              } */
              if(result == 0)
              {
                      iPtr = &iDataPtr[23];
                      iPtr->intNum = 23;
                      iPtr->userInt = &vmeuser_interrupt23;
                      result = vmekrn_registerInterrupt( iPtr );
              }

        }
		if( result < 0)
		{
			printk("Error - failed to register User interrupt\n");
		}
		else
		{
			printk("User Interrupt registered\n");
			for(i = 1; i < 24; i++ )
			{
				if((i== 14) || (i==15)) continue;
				result = vmekrn_enableInterrupt(i);

				if( result < 0)
				{
					printk("Error - failed to enable VME interrupt %d\n", i);
				}
			}
		}
		vmekrn_releaseDevice( VME_CONTROL );

	return result;
}


/******************************************************************************
*
* vmeexdrv_remusrInt
*
* The following routine shows how a kernel driver can remove an interrupt
* handler registered by the user kernel driver
*
*
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmeexdrv_remusrInt( void )
{
	int result, i;
	EN_VME_INT_DATA *iPtr;

	 /*Acquire the ownership to the control device*/
        result = vmekrn_acquireDevice(VME_CONTROL);

        if ( result < 0 )
        {
                printk("Error - failed to acquire control device\n");
        }
        else
        {
                for(i = 1; i < 24; i++ )
                {
                        intCount[i] = 0;
                        if((i== 14) || (i==15)) continue;
                        result = vmekrn_disableInterrupt(i);

                        if( result < 0)
                        {
                                printk("Error - failed to disable VME interrupt %d\n", i);
                        }
                }

                if(result == 0)
                {
                        iPtr = &iDataPtr[0];
                        iPtr->intNum = 1;
                        iPtr->userInt = &vmeuser_interrupt1;
                        iPtr->usrData.usrPtr = tstPtr;
                        result = vmekrn_removeInterrupt( iPtr );
                }
                if(result == 0)
                {
                        iPtr = &iDataPtr[1];
                        iPtr->intNum = 2;
                        iPtr->userInt = &vmeuser_interrupt2;
                        iPtr->usrData.usrPtr = tstPtr1;
                        result = vmekrn_removeInterrupt( iPtr );
                }
                if(result == 0)
                {
                        iPtr = &iDataPtr[2];
                        iPtr->intNum = 3;
                        iPtr->userInt = &vmeuser_interrupt3;
                        result = vmekrn_removeInterrupt( iPtr );
                }
                if(result == 0)
                {
                        iPtr = &iDataPtr[3];
                        iPtr->intNum = 4;
                        iPtr->userInt = &vmeuser_interrupt4;
                        result = vmekrn_removeInterrupt( iPtr );
                }
                if(result == 0)
                {
                        iPtr = &iDataPtr[4];
                        iPtr->intNum = 5;
                        iPtr->userInt = &vmeuser_interrupt5;
                        result = vmekrn_removeInterrupt( iPtr );
                }
                if(result == 0)
                {
                        iPtr = &iDataPtr[5];
                        iPtr->intNum = 6;
                        iPtr->userInt = &vmeuser_interrupt6;
                        result = vmekrn_removeInterrupt( iPtr );
                }
                if(result == 0)
                {
                        iPtr = &iDataPtr[6];
                        iPtr->intNum = 7;
                        iPtr->userInt = &vmeuser_interrupt7;
                        result = vmekrn_removeInterrupt( iPtr );
                }
                /* if(result == 0)
                {
                        iPtr = &iDataPtr[11];
                        iPtr->intNum = 12;
                        iPtr->userInt = &vmeuser_interrupt12;
                        result = vmekrn_removeInterrupt( iPtr );
                }*/
                if(result == 0)
                {
                        iPtr = &iDataPtr[23];
                        iPtr->intNum = 23;
                        iPtr->userInt = &vmeuser_interrupt23;
                        result = vmekrn_removeInterrupt( iPtr );
                }

        }
		if( result < 0)
		{
			printk("Error - failed to unregister User interrupt\n");
		}

		vmekrn_releaseDevice( VME_CONTROL );

	return result;
}

/******************************************************************************
*
* vmeexdrv_interrupt
*
* The following routine shows how a mailbox could be used
*
*
*
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmeexdrv_interrupt( void )
{
	int result;
	UINT32 reg;
	UINT32 intNum;
	UINT32 selectedInts;
	EN_VME_IMAGE_ACCESS idata;

	 /*Acquire the ownership to the control device*/
        result = vmekrn_acquireDevice(VME_CONTROL);

        if ( result < 0 )
        {
                printk("Error - failed to acquire control device\n");
        }
        else
        {
        	idata.vmeAddress = 0x2000000;
        	idata.addrSpace = EN_VME_A32;
        	idata.type = EN_VSI_BOTH;  /* both Program and Data */
        	idata.mode = EN_VSI_BOTH;  /* both Supervisor and non-privileged */

        	result = vmekrn_enableRegAccessImage( &idata );

        	if ( result < 0 )
        	{
            		printk("Error - failed to enable Register Access\n");
		}
        	else
        	{
            		printk("Register Access enabled\n");
            		printk("Waiting for someone to write to Mailbox...\n");

            		/* wait for mailbox interrupt to occur
               		  note: vmekrn_waitInterrupt makes sure the interrupt is enabled
               		 so there no need to call vmekrn_enableInterrupt first 
           		*/
            		selectedInts = 1L <<EN_MBOX0;
            		intNum = 0;

            		result = vmekrn_waitInterrupt( selectedInts, 0, &intNum );

            		if ( result < 0 )
            		{
                		printk("Error - failed to receive interrupt\n");
			}
            		else
            		{
                		printk("EN_MBOX0 RECEIVED (0x%08X)\n", intNum);
                		printk("Interrupt %u received\n", EN_MBOX0);

                		result = vmekrn_readRegister( 0x348, &reg  );

                		if ( result < 0 )
                		{
                    			printk("Error - failed to read register \n");
				}
				else
				{
                    			printk("MBOX0: 0x%08X\n", reg);
                		}
            		}

            		result = vmekrn_disableRegAccessImage();

            		if ( result < 0 )
            		{
                		printk("Error - failed to disable Register Access \n");
            		}
        	}

        	vmekrn_releaseDevice( VME_CONTROL );
    	}

	return result;
}


/******************************************************************************
*
* vmeexdrv_intcount
*
* The following routine is to obtain the interrupt count
*
*
*
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmeexdrv_intcount( int* arg )
{
	int i;

	for(i = 0; i < 26; i++)
	{
		*arg++ = intCount[i];
	}
	return 0;
}

static int vmeexdrv_generate_interrupt(int arg)
{
	int result;
	UINT8 itNum = (UINT8)arg;
	
	result = vmekrn_acquireDevice(VME_CONTROL);

	if ( result < 0 )
	{
			printk("Error - failed to acquire control device\n");
	}
	else
	{
		result= vmekrn_generateInterrupt(itNum);
		if(result == 0)
		{
				printk("interrupt %d generated\n", itNum);
		}
		else
		{
			printk("interrupt %d error\n",itNum);
		}
	
		vmekrn_releaseDevice( VME_CONTROL );
	}
	
	return result;
}
/******************************************************************************
*
* vmeexdrv_ioctl
*
* Demo driver ioctl routine
*
*
*  
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmeexdrv_ioctl( struct inode *inode, struct file *file,
						unsigned int cmd, unsigned long arg )
{
	int result=0, count;
	EN_VME_ENHINT_INFO intrInfo;
	int selectedInts;
	int intNum = 0;

	printk(KERN_DEBUG "ioctl %u 0x%lX\n",cmd, arg );

	switch( cmd )
	{
		case IOCTL_EX_LSI_IMAGE:
			result = vmeexdrv_lsi();
			break;
		case IOCTL_EX_VSI_IMAGE:
			result = vmeexdrv_vsi();
                        break;
		case IOCTL_EX_DMA_DIRECT:
			result = vmeexdrv_dmaDirect();
			break;
		case IOCTL_EX_DMA_LIST:
			result = vmeexdrv_dmaList();
			break;
		case IOCTL_EX_REGUSRINT:
			result = vmeexdrv_regusrInt();
			break;
		case IOCTL_EX_INTERRUPT:
			result = vmeexdrv_interrupt();
			break;
		case IOCTL_EX_INTCOUNT:
			result = vmeexdrv_intcount((int*)arg);
			break;
		case IOCTL_EX_REMUSRINT:
			result = vmeexdrv_remusrInt();
			break;
		case IOCTL_EX_INSCOUNT_CTL:
			result = vmekrn_acquireDevice(VME_CONTROL);
			if( result < 0 )
			{
					printk(KERN_ERR "Unable to acquire CTL Device 0\n");
			}
			else
			{
				 vmekrn_getInstanceCount( VME_CONTROL, (UINT32*)&count );
				 printk( "Ctl inst count: %d\n", count);
			}
			vmekrn_releaseDevice(VME_CONTROL);
			break;
		case IOCTL_EX_GENERATE_IT:
				result = vmeexdrv_generate_interrupt(arg);
			break;
		case IOCTL_EX_GET_GEO:
                        result = vmekrn_acquireDevice(VME_CONTROL);
                        if( result < 0 )
                        {
                                printk(KERN_ERR "Unable to acquire CTL Device 0\n");
                        }
                        else
                        {
                                 result = vmekrn_getGeographicAddr( VME_CONTROL, (UINT8*)arg );
                        }
                        vmekrn_releaseDevice(VME_CONTROL);
                        break;
		case IOCTL_EX_SETCAPMODE:
		        result = vmekrn_acquireDevice(VME_CONTROL);
                        if( result < 0 )
                        {
                                printk(KERN_ERR "Unable to acquire CTL Device 0\n");
                        }
                        else
                        {
                            vmekrn_setVectorCaptureMode( 8, 32 );
                        }
		        vmekrn_releaseDevice(VME_CONTROL);
		        break;
		case IOCTL_EX_ENHVECTREAD:
                        result = vmekrn_acquireDevice(VME_CONTROL);
                        if( result < 0 )
                        {
                                printk(KERN_ERR "Unable to acquire CTL Device 0\n");
                        }
                        else
                        {
                            intrInfo.intNum = 1;
                            intrInfo.vecSize = 8;
                            intrInfo.bufLen = 32;
                            intrInfo.buffer = kzalloc(intrInfo.bufLen, GFP_KERNEL);
                            printk("Waiting for Interrupt\n");
                            mdelay(1000);
                            result = vmekrn_readInterruptEnhInfo(&intrInfo);

                            printk("Obtained first vector: %d %d\n", *(char*)intrInfo.buffer, result);

                            kfree(intrInfo.buffer);
                        }
                        vmekrn_releaseDevice(VME_CONTROL);
		        break;
		case IOCTL_EX_INTERRUPT_BRIDGELESS:
		        selectedInts = (1L << EN_ACFAIL);

		        result = vmekrn_acquireDevice(VME_CONTROL);
                        if( result < 0 )
                        {
                                printk(KERN_ERR "Unable to acquire CTL Device 0\n");
                        }
                        else
                        {
                            result = vmekrn_waitInterrupt( selectedInts, 0, &intNum );
                            if( result < 0 )
                            {
                                printk("Error - failed to receive interrupt\n");
                            }
                            else
                            {
                                printk("Interrupt Received\n");
                            }
                            vmekrn_releaseDevice(VME_CONTROL);
		        }
		        break;
		case IOCTL_EX_SET_SYSFAIL:
                        result = vmekrn_acquireDevice(VME_CONTROL);
                        if( result < 0 )
                        {
                                printk(KERN_ERR "Unable to acquire CTL Device 0\n");
                        }
                        else
                        {
                            result = vmekrn_setSysFail( VME_CONTROL );
                            if( result < 0 )
                            {
                                printk("Error - failed to set SYSFAIL\n");
                            }
                            else
                            {
                                printk("SYSFAIL_O SET\n");
                                vmekrn_releaseDevice( VME_CONTROL );
                            }
                        }
		        break;
		case IOCTL_EX_CLEAR_SYSFAIL:
                        result = vmekrn_acquireDevice(VME_CONTROL);
                        if( result < 0 )
                        {
                                printk(KERN_ERR "Unable to acquire CTL Device 0\n");
                        }
                        else
                        {
                            result = vmekrn_clearSysFail( VME_CONTROL );
                            if( result < 0 )
                            {
                                printk("Error - failed to clear SYSFAIL\n");
                            }
                            else
                            {
                                printk("SYSFAIL_I cleared\n");
                                vmekrn_releaseDevice( VME_CONTROL );
                            }
                        }
                        break;
                case IOCTL_EX_GET_SYSFAIL:
                        result = vmekrn_acquireDevice(VME_CONTROL);
                        if( result < 0 )
                        {
                                printk(KERN_ERR "Unable to acquire CTL Device 0\n");
                        }
                        else
                        {
                            result = vmekrn_getSysFail( VME_CONTROL, (UINT8*)arg );
                            if( result < 0 )
                            {
                                printk("Error - Obtaining SYSFAIL_I status\n");
                            }
                            else
                            {
                                printk("SYSFAIL_I obtained\n");
                                vmekrn_releaseDevice( VME_CONTROL );
                            }
                        }
                        break;
		default:
			printk( KERN_ERR "Unsupported ioctl\n");
			result = -EINVAL;
			break;
	}	

	return result;
}

/******************************************************************************
*
* vmeexdrv_unlockedioctl
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
static long vmeexdrv_unlockedioctl( struct file *file,
                                                unsigned int cmd, unsigned long arg )
{
    long result;
    /*lock_kernel();*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
        mutex_lock(&vmeDevCtrl);
#else
        down(&vmeDevCtrl);
#endif
    result = (long)vmeexdrv_ioctl( (struct inode *)file->f_path.dentry->d_inode, file, cmd, arg );
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
        mutex_unlock(&vmeDevCtrl);
#else
        up(&vmeDevCtrl);
#endif
    /*unlock_kernel();*/
    return result;
}
#endif

/******************************************************************************
*
* vmeexdrv_open
*
* Demo driver open function
*
*
*  
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmeexdrv_open( struct inode *inode, struct file *file )
{
	int result=0;
  
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	printk(KERN_DEBUG "device open successful");
	MOD_INC_USE_COUNT;
#else
	if ( try_module_get( THIS_MODULE ) )
	{
		printk(KERN_DEBUG "open successful" );
	}
	else
	{
		printk(KERN_DEBUG "Unable to get the module");
		result = -EINVAL;
	}
#endif
	return result;
}


/******************************************************************************
*
* vmeexdrv_close
*
* Demo driver close routine
*
*
*  
*
* RETURNS: VME_SUCCESS if successful else an error code.
*
******************************************************************************/
static int vmeexdrv_close( struct inode *inode, struct file *file )
{
	printk(KERN_DEBUG "device closed" );

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	MOD_DEC_USE_COUNT;
#else
	module_put( THIS_MODULE );
#endif
  	return 0;
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
* RETURNS: V0 if successful else an error code.
*
******************************************************************************/
INIT_FUNC( void )
{
	int result;

	printk(KERN_DEBUG "\nVME Kernel API Demo Driver\n");

	/* register device with kernel */
	result = register_chrdev( vmeMajor, "example_drv", &vme_fops );
	if ( result >= 0 ) 
	{
		/* if major number allocated dynamically */
		if ( vmeMajor == 0 )
		{
			vmeMajor = result;
		}

		result = 0;	/* must return zero if successful */
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
	/* unregister device */
	unregister_chrdev( vmeMajor, "example_drv" );
}

#ifndef MODULE
module_init( init_vmedriver );
module_exit( cleanup_vmedriver );
#endif
