/******************************************************************************
*
* Filename: 	vme_proc.h
* 
* Description:	Header file VME device driver proc file system interface.
*
* $Revision: 1.2 $
*
* $Date: 2014-05-30 14:27:16 $
*
* $Source: /home/cvs/cvsroot/Linuxvme4/linuxvmeen/vme_proc.h,v $
*
* Copyright 2000-2005 Concurrent Technologies.
*
******************************************************************************/

#ifndef __INCvme_proc
#define __INCvme_proc

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
        typedef int (readfn_t)(char *page, char **start, off_t off, int count, int *eof, void *data);
#endif

typedef struct 
{
	char *name;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	readfn_t *read_proc;
#else
	read_proc_t *read_proc;
#endif

	void *data;

} VME_PROC_ENTRY;


void vme_procInit( unsigned int device );
void vme_procCleanup( void );

#endif	/* __INCvme_proc */
