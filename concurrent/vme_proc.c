/******************************************************************************
*
* Filename: 	vme_proc.c
* 
* Description:	VME device driver proc file system interface.
*
* $Revision: 1.5 $
*
* $Date: 2015-04-30 12:03:33 $
*
* $Source: /home/cvs/cvsroot/Linuxvme4/linuxvmeen/vme_proc.c,v $
*
* Copyright 2000-2005 Concurrent Technologies.
*
******************************************************************************/

#ifndef __KERNEL__
#define __KERNEL__
#endif

#define __NO_VERSION__

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
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

#include "vme_api_en.h"
#include "vme_types.h"
#include "vme_driver.h"
#include "vme_os.h"
#include "vme_proc.h"
#include "vme_compat.h"

extern int unv_readProcCtl( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int unv_readProcDma( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int unv_readProcImage( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int unv_readProcInts( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int unv_readProcLsiStats( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int unv_readProcVsiStats( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int unv_readProcDmaStats( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int unv_readProcPciAlloc( char *buffer, char **start, long offset, int length, int *eof, void *data );

extern int tsi148_readProcCtl( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int tsi148_readProcDma( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int tsi148_readProcImage( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int tsi148_readProcInts( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int tsi148_readProcLsiStats( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int tsi148_readProcVsiStats( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int tsi148_readProcDmaStats( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int tsi148_readProcPciAlloc( char *buffer, char **start, long offset, int length, int *eof, void *data );

extern int bridgeless_readProcCtl( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int bridgeless_readProcInts( char *buffer, char **start, long offset, int length, int *eof, void *data );

#ifdef DEBUG
extern int unv_readProcDmaList( char *buffer, char **start, long offset, int length, int *eof, void *data );
extern int tsi148_readProcDmaList( char *buffer, char **start, long offset, int length, int *eof, void *data );
#endif

static int vme_procOpen( struct inode *inode, struct file *file );
static int vme_procClose( struct inode *inode, struct file *file );
static ssize_t vme_procRead( struct file *file, char *buf, size_t count, loff_t *ppos );

static struct file_operations proc_fops = 
{
	read: 		vme_procRead,
	open: 		vme_procOpen,
	release:	vme_procClose
};

static VME_PROC_ENTRY unv_proc_entries[] = 
{
	{"ctl", unv_readProcCtl, NULL},
	{"dma0", unv_readProcDma, NULL},
	{"dma1", unv_readProcDma, NULL},
	{"lsi0", unv_readProcImage, (void *) 1},
	{"lsi1", unv_readProcImage, (void *) 2},
	{"lsi2", unv_readProcImage, (void *) 3},
	{"lsi3", unv_readProcImage, (void *) 4},
	{"lsi4", unv_readProcImage, (void *) 5},
	{"lsi5", unv_readProcImage, (void *) 6},
	{"lsi6", unv_readProcImage, (void *) 7},
	{"lsi7", unv_readProcImage, (void *) 8},
	{"vsi0", unv_readProcImage, (void *) 9},
	{"vsi1", unv_readProcImage, (void *) 10},
	{"vsi2", unv_readProcImage, (void *) 11},
	{"vsi3", unv_readProcImage, (void *) 12},
	{"vsi4", unv_readProcImage, (void *) 13},
	{"vsi5", unv_readProcImage, (void *) 14},
	{"vsi6", unv_readProcImage, (void *) 15},
	{"vsi7", unv_readProcImage, (void *) 16},
	{"ints", unv_readProcInts, NULL},
	{"lsiStats", unv_readProcLsiStats, NULL},
	{"vsiStats", unv_readProcVsiStats, NULL},
	{"dma0Stats", unv_readProcDmaStats, NULL},
	{"pciAlloc", unv_readProcPciAlloc, NULL},
#ifdef DEBUG
	{"dma0List", unv_readProcDmaList, NULL},
#endif
	{NULL, NULL},
};

static VME_PROC_ENTRY bridgeless_proc_entries[] =
{
        {"ctl", bridgeless_readProcCtl, NULL},
        {"ints", bridgeless_readProcInts, NULL},
        {NULL, NULL},

};

static VME_PROC_ENTRY tsi148_proc_entries[] =
{
        {"ctl", tsi148_readProcCtl, NULL},
        {"dma0", tsi148_readProcDma, (void *)17},
        {"dma1", tsi148_readProcDma, (void *)18},
        {"lsi0", tsi148_readProcImage, (void *) 1},
        {"lsi1", tsi148_readProcImage, (void *) 2},
        {"lsi2", tsi148_readProcImage, (void *) 3},
        {"lsi3", tsi148_readProcImage, (void *) 4},
        {"lsi4", tsi148_readProcImage, (void *) 5},
        {"lsi5", tsi148_readProcImage, (void *) 6},
        {"lsi6", tsi148_readProcImage, (void *) 7},
        {"lsi7", tsi148_readProcImage, (void *) 8},
        {"vsi0", tsi148_readProcImage, (void *) 9},
        {"vsi1", tsi148_readProcImage, (void *) 10},
        {"vsi2", tsi148_readProcImage, (void *) 11},
        {"vsi3", tsi148_readProcImage, (void *) 12},
        {"vsi4", tsi148_readProcImage, (void *) 13},
        {"vsi5", tsi148_readProcImage, (void *) 14},
        {"vsi6", tsi148_readProcImage, (void *) 15},
        {"vsi7", tsi148_readProcImage, (void *) 16},
        {"ints", tsi148_readProcInts, NULL},
        {"lsiStats", tsi148_readProcLsiStats, NULL},
        {"vsiStats", tsi148_readProcVsiStats, NULL},
        {"dma0Stats", tsi148_readProcDmaStats, (void*)17},
        {"dma1Stats", tsi148_readProcDmaStats, (void*)18},
        {"pciAlloc", tsi148_readProcPciAlloc, NULL},
#ifdef DEBUG
        {"dma0List", tsi148_readProcDmaList, (void*)17},
        {"dma1List", tsi148_readProcDmaList, (void*)18},
#endif
        {NULL, NULL},
};


static VME_PROC_ENTRY	*proc_entries;

static struct proc_dir_entry *proc_vmeRoot = NULL;


/******************************************************************************
*
* vme_procOpen
*
* Called when proc file is opened. Used to increment usage count, 
* preventing driver from being unloaded if proc file is still open. 
* 
*
*  
*
* RETURNS: VME_SUCCESS.
*
******************************************************************************/
static int vme_procOpen( struct inode *inode, struct file *file )
{
	int result;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	DBMSG("%s", "Proc open");
	MOD_INC_USE_COUNT;
	result = VME_SUCCESS;
#else
	if ( try_module_get( THIS_MODULE ) )
	{
		DBMSG("%s", "Proc open");
		result = VME_SUCCESS;
	}
	else
	{
		DBMSG("%s", "Proc open failed");
		result = (-VME_EINVAL);
	}
#endif

	return result;
}


/******************************************************************************
*
* vme_procClose
*
* Called when proc file is closed. Used to decrement module usage count. 
*  
* 
*
*  
*
* RETURNS: VME_SUCCESS.
*
******************************************************************************/
static int vme_procClose( struct inode *inode, struct file *file )
{

	DBMSG("%s", "Proc close");

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	MOD_DEC_USE_COUNT;
#else
	module_put( THIS_MODULE );
#endif

	return VME_SUCCESS;
}


/******************************************************************************
*
* vme_procRead
*
* Called when proc file is read. Searches our table of entries and if found 
* calls the appropriate read function. 
* 
*
*  
*
* RETURNS: length of data read if successful else error code.
*
******************************************************************************/
static ssize_t vme_procRead( struct file *file, char *buf, size_t count, 
								loff_t *ppos )
{
	ssize_t result;
	char *nPtr;
	int i;
	int found;
	int eof;
	char *startPtr;
	 
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
	nPtr = (char *) file->f_path.dentry->d_iname;
#else
	nPtr = (char *) file->f_dentry->d_iname;
#endif

	DBMSG( "%s read %d bytes", nPtr, (UINT32) file->f_pos, (int) count );

	i = 0;
	found = 0;

	/* get data when request is made from start position only */ 
	if (*ppos == 0)
	{
		result = (-VME_ENODEV);

		while ( (proc_entries[i].name != NULL) && (found == 0) )
		{
			if ( strcmp( proc_entries[i].name, nPtr ) == 0 )
			{
				nPtr = (char *) kmalloc( count, GFP_KERNEL );

				if ( nPtr != NULL )
				{
					result = proc_entries[i].read_proc( nPtr, &startPtr, 
													0, (int) count, &eof, 
													proc_entries[i].data);

					if ( result > 0 )
					{
						if ( copy_to_user( (void *) buf, nPtr, result) != 0 )
						{
							result = (-VME_EFAULT);
						}
						else
						{
							/* update position so we can return no more data */
							/* if we get called again */
							*ppos = result;
						}
					}
 
					kfree( nPtr );

					DBMSG( "Proc entry %d, result %d", i, result);

					found = 1;
				}
				else
				{
					result = (-VME_ENOMEM);
				}
			}

			i++;
		}
	}
	else
	{
		/* we have already returned data */
		/* so just signal no more available */
		result = 0;
	}

	return result;
}


/******************************************************************************
*
* vme_procInit
*
* Creates and registers our entries with the proc file system.
*
*
*  
*
* RETURNS: None.
*
******************************************************************************/
void vme_procInit( unsigned int device )
{
	int i = 0;
	struct proc_dir_entry *res;

	/* create our root directory, /proc/vme */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	proc_vmeRoot = proc_mkdir("vme", NULL);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
	proc_vmeRoot = create_proc_entry("vme", S_IFDIR|S_IRUGO|S_IXUGO,
								NULL);
#else
	proc_vmeRoot = create_proc_entry("vme", S_IFDIR|S_IRUGO|S_IXUGO,
								&proc_root);
#endif
	
	if (proc_vmeRoot != NULL)
	{
		if( device == TSI148_PCI_DEVICE_ID )
		{
			proc_entries = &tsi148_proc_entries[0];
		}
		else if( device == BRIDGELESS_DEVICE_ID )
		{
		        proc_entries = &bridgeless_proc_entries[0];
		}
		else
		{
			proc_entries = &unv_proc_entries[0];
                }

		while ( proc_entries[i].name != NULL)
		{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
		    res = proc_create(proc_entries[i].name, S_IFREG|S_IRUGO|S_IWUSR, proc_vmeRoot, &proc_fops);
#else
			res = create_proc_entry( proc_entries[i].name, 
									S_IFREG|S_IRUGO|S_IWUSR, proc_vmeRoot );

			if (res == NULL)
			{
				MSG("Failed to create proc entry: %s", proc_entries[i].name);
				break;
			}
			
			res->proc_fops = &proc_fops;
#endif
			i++;
		}

		DBMSG("%d Proc entries created", i);
	}
	else
	{
		MSG("%s", "Failed to create proc entries");
	}
}


/******************************************************************************
*
* vme_procCleanup
*
* Removes and unregisters our proc file system entries.
*
*
*  
*
* RETURNS: None.
*
******************************************************************************/
void vme_procCleanup( void )
{
	int i = 0;


	while ( proc_entries[i].name != NULL)
	{
		remove_proc_entry( proc_entries[i].name, proc_vmeRoot);

		i++;
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
	remove_proc_entry( "vme", NULL );
#else
	remove_proc_entry( "vme", &proc_root );
#endif		
}
