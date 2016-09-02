/******************************************************************************
*
* Filename:	 vme_os.c
*
* Description:	OS wrapper functions.
*
* $Revision: 1.16 $
*
* $Date: 2015-06-01 08:22:45 $
*
* $Source: /home/cvs/cvsroot/Linuxvme4/linuxvmeen/vme_os.c,v $
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
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/tty.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <asm/page.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/efi.h>

#include "vme_api_en.h"
#include "vme_types.h"
#include "vme_os.h"
#include "vme_compat.h"


#ifdef CONFIG_XEN
extern int resMemStart;
#endif


typedef struct
{
	struct semaphore wait;

} OSSEMA;

typedef struct
{
	wait_queue_head_t wait;
	int wait_active;

} OSWQUEUE;

typedef struct
{
	struct timer_list timer;

} OSTLIST;


typedef void (*userInt)( void* ); /* Kernel mode user routine called upon interrupt */

/* User ISR Link list */
typedef struct
{
  UINT32 intNum;
  userInt UserIntFn;                         /* User registered kernel interrupt handlers */
  EN_VME_INT_USR_DATA usrData;                /* Argument for user registered kernel interrupt handler */
  struct list_head usrIsrList;
} TSI148USRISR;


/* List of kernel mode user interrupt handlers */
//LIST_HEAD( usrTsi148IsrList );
struct list_head usrTsi148IsrList[26];

/* Device Identifier */
static int deviceIdent = -1;

/* Interrupt - Kernel Thread  spinlock */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
spinlock_t iLock = SPIN_LOCK_UNLOCKED;
#else
DEFINE_SPINLOCK(iLock);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
  DEFINE_MUTEX(vmeDevIOCtrl);
#else
  DECLARE_MUTEX(vmeDevIOCtrl);
#endif


/* Default free PCI memory space */
/* Can be overridden by using module parameters pciAddr and pciSize */
#define PCI_SPACE_START	0xb0000000
#define PCI_SPACE_SIZE	0x10000000


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
extern void unv_isr_wr( int irq, void *dev_id, struct pt_regs *regs );
extern void tsi148_isr_wr( int irq, void *dev_id, struct pt_regs *regs );
extern void bridgeless_isr_wr( int irq, void *dev_id, struct pt_regs *regs );
#define unv_interrupt unv_isr_wr
#define tsi148_interrupt tsi148_isr_wr
#define bridgeless_interrupt bridgeless_isr_wr
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20))
extern irqreturn_t unv_isr_re( int irq, void *dev_id );
extern irqreturn_t tsi148_isr_re( int irq, void *dev_id );
extern irqreturn_t bridgeless_isr_re( int irq, void *dev_id );
#define unv_interrupt unv_isr_re
#define tsi148_interrupt tsi148_isr_re
#define bridgeless_interrupt bridgeless_isr_re
#else
extern irqreturn_t unv_isr( int irq, void *dev_id, struct pt_regs *regs );
extern irqreturn_t tsi148_isr( int irq, void *dev_id, struct pt_regs *regs );
extern irqreturn_t bridgeless_isr( int irq, void *dev_id, struct pt_regs *regs );
#define unv_interrupt unv_isr
#define tsi148_interrupt tsi148_isr
#define bridgeless_interrupt bridgeless_isr
#endif
#endif

/******************************************************************************
*
* os_mdelay
*
* Millisec delay.
*
*
*
*
* RETURNS: IO port value.
*
******************************************************************************/
void os_mdelay( unsigned long msec )
{
        mdelay( msec );
}

/******************************************************************************
*
* os_inb
*
* IO read byte.
*
*
*
*
* RETURNS: IO port value.
*
******************************************************************************/
unsigned char os_inb( unsigned int address )
{
	return inb( address );
}


/******************************************************************************
*
* os_outb
*
* IO write byte.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_outb( unsigned char value, unsigned int address )
{
	outb_p( value, address );
}


/******************************************************************************
*
* os_inb_p
*
* IO read byte with pause.
*
*
*
*
* RETURNS: IO port value.
*
******************************************************************************/
unsigned char os_inb_p( unsigned int address )
{
	return inb_p( address );
}


/******************************************************************************
*
* os_outb_p
*
* IO write byte with pause.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_outb_p( unsigned char value, unsigned int address )
{
	outb_p( value, address );
}


#if defined(__powerpc__)
/******************************************************************************
*
* os_readPvr
*
* Get processor version.
*
*
*
*
* RETURNS: Processor version register value.
*
******************************************************************************/
unsigned int os_readPvr( void )
{
	return mfspr( PVR );
}
#endif


/******************************************************************************
*
* os_readb
*
* Read byte.
*
*
*
*
* RETURNS: Byte value.
*
******************************************************************************/
unsigned char os_readb( void *addr )
{
	return readb( addr );
}


/******************************************************************************
*
* os_writeb
*
* Write byte value.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_writeb( unsigned char val, void *addr )
{
	writeb( val, addr );
}


/******************************************************************************
*
* os_readw
*
* Read word.
*
*
*
*
* RETURNS: Word value.
*
******************************************************************************/
unsigned short os_readw( void *addr )
{
	return readw( addr );
}


/******************************************************************************
*
* os_writew
*
* Write word value.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_writew( unsigned short val, void *addr )
{
	writew( val, addr );
}


/******************************************************************************
*
* os_readl
*
* Read long.
*
*
*
*
* RETURNS: Long value.
*
******************************************************************************/
unsigned int os_readl( void *addr )
{
	return readl( addr );
}


/******************************************************************************
*
* os_writel
*
* Write long value.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_writel( unsigned int val, void *addr )
{
	writel( val, addr );
}


/******************************************************************************
*
* os_disable_interrupts
*
* Disable interrupts.
*
*
*
*
* RETURNS: flags.
*
******************************************************************************/
unsigned long os_disable_interrupts( void )
{
	unsigned long flags;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	save_flags( flags );
	cli();
#else
	local_save_flags( flags );
	local_irq_disable();
#endif

	return flags;
}


/******************************************************************************
*
* os_enable_interrupts
*
* Enable interrupts.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_enable_interrupts( unsigned long flags )
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	restore_flags( flags ); 
#else
	local_irq_restore( flags ); 
#endif
}

/******************************************************************************
*
* os_dma_cmd_pkt_alloc
*
* Allocate kernel memory for DMA packets.This function has been provided
* for portability across the OS
*
*
*
* RETURNS: Pointer to the allocated memory or NULL.
*
******************************************************************************/
void *os_dma_cmd_pkt_alloc( unsigned long size )
{
        return kmalloc( size, GFP_KERNEL | GFP_DMA );
}

/******************************************************************************
*
* os_dma_cmd_pkt_free
*
* Free kernel memory obtained for DMA packets
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_dma_cmd_pkt_free ( const void *objp )
{
	if ( objp != NULL )
        {
                kfree( objp );
        }
}	

/******************************************************************************
*
* os_kmalloc
*
* Allocate kernel memory.
*
*
*
*
* RETURNS: Pointer to the allocated memory or NULL.
*
******************************************************************************/
void *os_kmalloc( unsigned int size )
{
	return kmalloc( size, GFP_KERNEL );
}


/******************************************************************************
*
* os_kfree
*
* Free kernel memory.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_kfree( const void *objp )
{
	if ( objp != NULL )
	{
		kfree( objp );
	}
}


/******************************************************************************
*
* os_alloc_wqueue
*
* Allocate and initialize a wait queue.
*
*
*
*
* RETURNS: Pointer to the allocated wait queue or NULL.
*
******************************************************************************/
void *os_alloc_wqueue( void )
{
    OSWQUEUE *pWqueue;


	pWqueue = (OSWQUEUE *) kmalloc( sizeof( OSWQUEUE ), GFP_KERNEL );
    
    if (pWqueue != NULL)
    {
		init_waitqueue_head( &pWqueue->wait );
		pWqueue->wait_active = 0;
    }

    return pWqueue;
}


/******************************************************************************
*
* os_schedule
*
* Relinquish the processor temporarily.
*
*
*
*
* RETURNS: 1.
*
******************************************************************************/
int os_schedule( void )
{
	schedule();

	return 1;
}


/******************************************************************************
*
* os_signal_pending
*
* Check if signal pending.
*
*
*
*
* RETURNS: True if signal pending else false.
*
******************************************************************************/
int os_signal_pending( void )
{
	return signal_pending( current );
}


/******************************************************************************
*
* os_interruptible_condition
*
* Updates the condition for the corresponding wait queue
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_interruptible_condition( void *pWqueue, int status )
{
        OSWQUEUE *os_wqueue = (OSWQUEUE *)pWqueue;

        os_wqueue->wait_active = status;

}


/******************************************************************************
*
* interruptible_sleep_on
*
* Put process to sleep and make interruptible.
* Allow safe signals to occur and not terminate the DMA transfer or wait4Interrupt
*
*
*
* RETURNS: None.
*
******************************************************************************/
int os_interruptible_sleep_on( void *pWqueue )
{
	OSWQUEUE *os_wqueue = (OSWQUEUE *)pWqueue;
	int ret;
	unsigned long mask = (    (1 << (SIGUSR1-1))
	                        | (1 << (SIGUSR2-1))
	                        | (1 << (SIGCHLD-1))
	                        | (1 << (SIGCONT-1))
	                        | (1 << (SIGURG-1))
	                        | (1 << (SIGWINCH-1)) );

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	interruptible_sleep_on( &os_wqueue->wait );
#else
	do
	{
	    ret = wait_event_interruptible( os_wqueue->wait, (os_wqueue->wait_active==1) );
	}while( (ret < 0) && ((current->signal->shared_pending.signal.sig[0]) & mask) );

	return ret;
#endif
}


/******************************************************************************
*
* os_wake_up_interruptible
*
* Wake up sleeping interruptible process.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_wake_up_interruptible( void *pWqueue )
{
	OSWQUEUE *os_wqueue = (OSWQUEUE *)pWqueue;


	os_wqueue->wait_active = 1;
	wake_up_interruptible( &os_wqueue->wait );
}


/******************************************************************************
*
* os_waitqueue_active
*
* Check if wait queue is available.
*
*
*
*
* RETURNS: True if active else false.
*
******************************************************************************/
int os_waitqueue_active( void *pWqueue )
{
	OSWQUEUE *os_wqueue = (OSWQUEUE *)pWqueue;

	return waitqueue_active( &os_wqueue->wait );
	
}


/******************************************************************************
*
* os_alloc_timer
*
* Allocate and initialize a timer.
*
*
*
*
* RETURNS: Pointer to the allocated timer or NULL.
*
******************************************************************************/
void *os_alloc_timer( void )
{
    OSTLIST *pTimer;


	pTimer = (OSTLIST *) kmalloc( sizeof( OSTLIST ), GFP_KERNEL );
    
    if (pTimer != NULL)
    {
		init_timer( &pTimer->timer );
    }

    return pTimer;
}


/******************************************************************************
*
* os_start_timer
*
* Setup and start timer.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_start_timer( void *pTimer, void (*function)( unsigned long ), 
							unsigned long arg, unsigned long timeout )
{
    OSTLIST *os_timer = (OSTLIST *)pTimer;
    
    init_timer( &os_timer->timer );
	os_timer->timer.function = function;
	os_timer->timer.data = arg;
	os_timer->timer.expires = jiffies + timeout; 
	add_timer( &os_timer->timer );
}


/******************************************************************************
*
* os_del_timer
*
* Delete timer.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_del_timer( void *pTimer )
{
    OSTLIST *os_timer = (OSTLIST *)pTimer;
    
	del_timer( &os_timer->timer );
}

/******************************************************************************
*
* os_spinlock_init
*
* Initialize the spinlock
*
*
*
*
* RETURNS: Pointer to the allocated semaphore or NULL.
*
******************************************************************************/
void os_spinlock_init( void )
{
        spin_lock_init( &iLock );
}

/******************************************************************************
*
* os_spinlock_lock
*
* Lock the spinlock
*
*
*
*
* RETURNS: Saved interrrupt flags
*
******************************************************************************/
unsigned long os_spinlock_lock( void  )
{
        unsigned long flags;

        spin_lock_irqsave( &iLock, flags );

        return flags;
}

/******************************************************************************
*
* os_spinlock_unlock
*
* Unlock the spinlock
*
*
*
*
* RETURNS:
*
******************************************************************************/
void os_spinlock_unlock( unsigned long flags )
{
        spin_unlock_irqrestore( &iLock, flags );
}


/******************************************************************************
*
* os_alloc_sema
*
* Allocate and initialize a semaphore.
*
*
*
*
* RETURNS: Pointer to the allocated semaphore or NULL.
*
******************************************************************************/
void *os_alloc_sema( void )
{
    OSSEMA *pSema;


	pSema = (OSSEMA *) kmalloc( sizeof( OSSEMA ), GFP_KERNEL );
    
    if (pSema != NULL)
    {
    	sema_init( &pSema->wait, 1 );
    }

    return pSema;
}


/******************************************************************************
*
* os_acquire_sema
*
* Acquire a semaphore.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_acquire_sema( void *pSema )
{
    OSSEMA *os_sema = (OSSEMA *)pSema;
    
    down( &os_sema->wait );
}


/******************************************************************************
*
* os_release_sema
*
* Release a semaphore.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_release_sema( void  *pSema )
{
    OSSEMA *os_sema = (OSSEMA *)pSema;
    
    up( &os_sema->wait );
}


/******************************************************************************
*
* os_ioremap
*
* ioremap memory.
*
*
*
*
* RETURNS: Pointer to the remapped memory or NULL.
*
******************************************************************************/
void *os_ioremap( unsigned long offset, unsigned long size )
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25))
		return ioremap( offset, size);
#else
		return ioremap_cache( offset, size);
#endif
}


/******************************************************************************
*
* os_iounmap
*
* iounmap memory.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_iounmap( void *addr )
{
	iounmap( addr );
}


/******************************************************************************
*
* os_remap_page_range
*
* Remap page range for given physical address.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
int os_remap_page_range( void *pVma, unsigned long phys_add, unsigned long size )
{
	struct vm_area_struct *vma = (struct vm_area_struct *) pVma;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
#ifdef CONFIG_XEN
	return io_remap_pfn_range( vma, vma->vm_start, (phys_add >> PAGE_SHIFT),
			     					size, vma->vm_page_prot );
#else
	return remap_pfn_range( vma, vma->vm_start, (phys_add >> PAGE_SHIFT),
			     					size, vma->vm_page_prot );
#endif
#else
#ifdef REMAP_PFN_RANGE
	return remap_pfn_range( vma, vma->vm_start, (phys_add >> PAGE_SHIFT),
			     					size, vma->vm_page_prot );
#else

#ifdef REMAP_PAGE_RANGE_4
	return remap_page_range( vma->vm_start, phys_add,
			     					size, vma->vm_page_prot );
#else
	return remap_page_range( vma->vm_start, phys_add,
			     					size, vma->vm_page_prot );
#endif
#endif
#endif

}


/******************************************************************************
*
* os_memcpy
*
* Memory copy.
*
*
*
*
* RETURNS: Pointer to the destination.
*
******************************************************************************/
void *os_memcpy( void *to, const void *from, unsigned int n )
{
	return memcpy( to, from, n );
}


/******************************************************************************
*
* os_memset
*
* Memory set.
*
*
*
*
* RETURNS: Pointer to the memory being set.
*
******************************************************************************/
void *os_memset( void *s, char c, unsigned int count)
{
	return memset( s, c, count);
}


/******************************************************************************
*
* os_strcpy
*
* String copy.
*
*
*
*
* RETURNS: Pointer to the destination.
*
******************************************************************************/
char *os_strcpy( char *dest, const char *src )
{
	return strcpy( dest, src );
}


/******************************************************************************
*
* os_strcmp
*
* String compare.
*
*
*
*
* RETURNS: An integer greater than, equal to, or less than 0.
*
******************************************************************************/
int os_strcmp( const char *cs, const char *ct )
{

	return strcmp( cs, ct );
}

/******************************************************************************
*
* os_strncmp
*
* String compare.
*
*
*
*
* RETURNS: An integer greater than, equal to, or less than 0.
*
******************************************************************************/
int os_strncmp( const char *cs, const char *ct, unsigned long count )
{

        return strncmp( cs, ct, count );
}

/******************************************************************************
*
* os_page_align
*
* Get a page aligned size.
*
*
*
*
* RETURNS: Page aligned size.
*
******************************************************************************/
unsigned int os_page_align( unsigned int size )
{
	return PAGE_ALIGN( size );
}


/******************************************************************************
*
* os_getMemInfo
*
* Get system memory information.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_getMemInfo( UMEMINFO *pmi )
{
	struct sysinfo si;


	si_meminfo( &si );	/* get memory information */

	pmi->totalram = si.totalram;
	pmi->totalhigh = si.totalhigh;
	pmi->freeram = si.freeram;
	pmi->freehigh = si.freehigh;
}


/******************************************************************************
*
* os_high_memory
*
* Get the virtual high memory address.
*
*
*
*
* RETURNS: Pointer to the virtual high memory address.
*
******************************************************************************/
void *os_high_memory( void )
{
	return high_memory;
}


/******************************************************************************
*
* os_virt_to_phys
*
* Convert a virtual address to a physical address.
*
*
*
*
* RETURNS: A physical address value.
*
******************************************************************************/
unsigned long os_virt_to_phys( volatile void *address )
{
	return virt_to_phys( address );
}


/******************************************************************************
*
* os_page_shift
*
* Get the page shift value.
*
*
*
*
* RETURNS: The page shift value.
*
******************************************************************************/
int os_page_shift( void )
{
	return PAGE_SHIFT;
}


/******************************************************************************
*
* os_page_size
*
* Get the page size value.
*
*
*
*
* RETURNS: The page size value.
*
******************************************************************************/
unsigned long os_page_size( void )
{
	return PAGE_SIZE;
}


/******************************************************************************
*
* os_sprintf
*
* Write a formatted string to a buffer.
*
*
*
*
* RETURNS: The number of characters copied to buffer.
*
******************************************************************************/
int os_sprintf( char *buf, const char *fmt, ... )
{
    va_list arglist;
    int result;


    va_start( arglist, fmt );
    result = vsprintf( buf, fmt, arglist );
    va_end( arglist );

    return result;
}


/******************************************************************************
*
* os_logMsg
*
* Formats and sends the message to the kernel log.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_logMsg( char *fmt, ... )
{
	va_list args;
	char *msg;


	msg = (char *) kmalloc( PAGE_SIZE, GFP_KERNEL );

	if ( msg != NULL )
	{
		/* format message */
		va_start( args, fmt );
		vsprintf( msg, (const char *)fmt, args );
		va_end( args );

		printk( KERN_DEBUG "%s\n", msg );

		kfree( msg );
	}
}


/******************************************************************************
*
* os_copy_from_user
*
* Copy data from user space.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
unsigned long os_copy_from_user( void *dest, void *src, unsigned long size )
{
	return copy_from_user( dest, src, size );
}


/******************************************************************************
*
* os_copy_from_user_nc
*
* Copy data from user space with no check.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
unsigned long os_copy_from_user_nc( void *dest, const void *src, unsigned long size )
{
	return __copy_from_user( dest, src, size );
}


/******************************************************************************
*
* os_copy_to_user
*
* Copy data to user space.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
unsigned long os_copy_to_user( void *dest, void *src, unsigned long size )
{
	return copy_to_user( dest, src, size );
}


/******************************************************************************
*
* os_copy_to_user_nc
*
* Copy data to user space with no check.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
unsigned long os_copy_to_user_nc( void *dest, const void *src, unsigned long size )
{
	return __copy_to_user( dest, src, size );
}


/******************************************************************************
*
* os_put_user8_nc
*
* Copy a single 8 bit value to user space with no check.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_put_user8_nc( UINT8 value, UINT8 *src )
{
	__put_user( value, src );
}


/******************************************************************************
*
* os_put_user16_nc
*
* Copy a single 16 bit value to user space with no check.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_put_user16_nc( UINT16 value, UINT16 *src )
{
	__put_user( value, src );
}


/******************************************************************************
*
* os_put_user32_nc
*
* Copy a single 32 bit value to user space with no check.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_put_user32_nc( UINT32 value, UINT32 *src )
{
	__put_user( value, src );
}

/******************************************************************************
*
* os_put_user64_nc
*
* Copy a single 64 bit value to user space with no check.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_put_user64_nc( ULONG value, ULONG *src )
{
	__put_user( value, src );
}

/******************************************************************************
*
* os_get_user8_nc
*
* Copy a single 8 bit value from user space with no check.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
unsigned long os_get_user8_nc( UINT8 *value, const UINT8 *src )
{
	UINT8 tmp;
	unsigned long result;

	result = __get_user( tmp, src );
	*value = tmp;

	return result;
}


/******************************************************************************
*
* os_get_user16_nc
*
* Copy a single 16 bit value from user space with no check.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
unsigned long os_get_user16_nc( UINT16 *value, const UINT16 *src )
{
	UINT16 tmp;
	unsigned long result;

	result = __get_user( tmp, src );
	*value = tmp;

	return result;
}


/******************************************************************************
*
* os_get_user32_nc
*
* Copy a single 32 bit value from user space with no check.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
unsigned long os_get_user32_nc( UINT32 *value, const UINT32 *src )
{
	UINT32 tmp;
	unsigned long result;


	result = __get_user( tmp, src );
	*value = tmp;

	return result;
}


/******************************************************************************
*
* os_get_user64_nc
*
* Copy a single 64 bit value from user space with no check.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
unsigned long os_get_user64_nc( ULONG *value, const ULONG *src )
{
	ULONG tmp;
	unsigned long result;


	result = __get_user( tmp, src );
	*value = tmp;

	return result;
}

/******************************************************************************
*
* os_get_user64
*
* Copy a single 64 bit value from user space.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
unsigned long os_get_user64( ULONG *value, const ULONG *src )
{
	ULONG tmp;
	unsigned long result;


	result = get_user( tmp, src );
	*value = tmp;

	return result;
}

/******************************************************************************
*
* os_get_user32
*
* Copy a single 32 bit value from user space.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
unsigned long os_get_user32( UINT32 *value, const UINT32 *src )
{
	UINT32 tmp;
	unsigned long result;


	result = get_user( tmp, src );
	*value = tmp;

	return result;
}


/******************************************************************************
*
* os_memcpy_fromio
*
* Copy data from memory mapped IO space.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_memcpy_fromio( void *dest, void *src, unsigned long count )
{
	memcpy_fromio( dest, src, count );
}


/******************************************************************************
*
* os_memcpy_toio
*
* Copy data tp memory mapped IO space.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_memcpy_toio( void *dest, const void *src, unsigned long count )
{
	memcpy_toio( dest, src, count );
}



/******************************************************************************
*
* os_cpu_to_le32
*
* Convert CPU to little-endian byte order.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
UINT32 os_cpu_to_le32( UINT32 value )
{
	return __cpu_to_le32( value );
}


/******************************************************************************
*
* os_cpu_to_le32
*
* Convert little-endian to CPU byte order.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
UINT32 os_le32_to_cpu( UINT32 value )
{
	return __le32_to_cpu( value );
}


/******************************************************************************
*
* os_virt_to_bus
*
* Convert virtual address to bus (physical) address.
*
*
*
*
* RETURNS: The bus (physical) address.
*
******************************************************************************/
unsigned long os_virt_to_bus( void *address )
{
	return virt_to_bus( address );
}


/******************************************************************************
*
* os_request_irq
*
* Attach an Universe interrupt service routine.
*
*
*
*
* RETURNS: 0 or error code.
*
******************************************************************************/
int os_request_irq( unsigned int irq, const char *devname )
{
	unsigned int vendor,device;
	void *pciDev=NULL;
	 
	os_devQuery( &vendor , &device , pciDev);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20))
	 if( device == TSI148_PCI_DEVICE_ID )
        {
                return request_irq( irq, tsi148_interrupt, IRQF_SHARED,
                                                devname, tsi148_interrupt );
        }
        else
        {
                return request_irq( irq, unv_interrupt, IRQF_SHARED,
                                                devname, unv_interrupt );
        }
#else
	if( device == TSI148_PCI_DEVICE_ID )
	{
		return request_irq( irq, tsi148_interrupt, (SA_INTERRUPT | SA_SHIRQ),
						devname, tsi148_interrupt );	
	}
	else
	{
		return request_irq( irq, unv_interrupt, (SA_INTERRUPT | SA_SHIRQ),
						devname, unv_interrupt );	
	}
#endif
}

/******************************************************************************
*
* os_request_irq_bridgeless
*
* Attach Bridgeless interrupt service routine.

* RETURNS: 0 or error code.
*
******************************************************************************/
int os_request_irq_bridgeless( unsigned int irq, const char *devname )
{

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20))
                return request_irq( irq, bridgeless_interrupt, IRQF_SHARED,
                                                devname, bridgeless_interrupt );
#else
                return request_irq( irq, bridgeless_interrupt, (SA_INTERRUPT | SA_SHIRQ),
                                                devname, bridgeless_interrupt );
#endif
}

/******************************************************************************
*
* os_free_irq
*
* Release an interrupt service routine.
*
*
*
*
* RETURNS: None.
*
******************************************************************************/
void os_free_irq( unsigned int irq )
{
	unsigned int vendor,device;
        void *pciDev=NULL;

        os_devQuery( &vendor , &device , pciDev);

        if( device == TSI148_PCI_DEVICE_ID )
        {
                free_irq( irq, tsi148_interrupt );
        }
        else
        {
                free_irq( irq, unv_interrupt );
        }
}

/******************************************************************************
*
* os_free_irq_bridgeless
*
* Release an interrupt service routine.
*
* RETURNS: None.
*
******************************************************************************/
void os_free_irq_bridgeless( unsigned int irq )
{
      free_irq( irq, bridgeless_interrupt );
}

/******************************************************************************
*
* os_find_bridge
*
* Finds the bridge device.
*
*
*
*
* RETURNS: VME_SUCCESS or error code.
*
******************************************************************************/
int os_find_bridge( UNIPCIINFO *pUni )
{
	int result;
	UINT32 temp;
	struct pci_dev *pciDev = NULL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))	
	pciDev = pci_get_device(pUni->vendor,pUni->device,pciDev);
#else
	pciDev = pci_find_device(pUni->vendor,pUni->device,pciDev);
#endif

	if ( pciDev != NULL )
	{
		/* For ACPI systems , before you do anything with the device we've found,
		 * we need to enable it by calling pci_enable_device() which enables
		 * I/O and memory regions of the device, allocates an IRQ if necessary,
		 * assigns missing resources if needed and wakes up the device if it 
		 * was in suspended state. 
		 */
		result = pci_enable_device( pciDev );
		result = VME_SUCCESS;		/* Until BIOS is up and ready for VP426*/
		if (  result == VME_SUCCESS )
		{
			/* Get device information */
			pUni->vendor = (UINT16) pciDev->vendor;
			pUni->device = (UINT16) pciDev->device;
			pUni->class = (UINT32) pciDev->class;

			/* Get base address and IRQ */
			pci_read_config_dword(pciDev, PCI_BASE_ADDRESS_0, &temp);
			pUni->baseAddress = temp;

			/* CGD following code only works on non APIC/multiprocessor
			 * enabled kernels. Use irq from supplied device
			 * structure instead
			 *
			 * pci_read_config_dword( pciDev, PCI_INTERRUPT_LINE, &temp );
			 * pUni->irq = (temp & 0x000000FF);
			 */
			pUni->irq = pciDev->irq;

			pUni->pciDev = (void *) pciDev;

			/* Turn latency off */
			pci_write_config_byte(pciDev, PCI_LATENCY_TIMER, 0);
		}
		else
		{
			pUni->pciDev = NULL;
		}
	}
	else
	{
		result = (-VME_ENODEV);
	}

	return result;
}


/******************************************************************************
*
* os_disable_bridge
*
* Disable the bridge device.
*
*
*
*
* RETURNS: VME_SUCCESS
*
******************************************************************************/
int os_disable_bridge( UNIPCIINFO *pUni )
{

	if ( pUni->pciDev != NULL )
	{
		pci_disable_device( (struct pci_dev *) pUni->pciDev );
	}
	
	return VME_SUCCESS;
}


/******************************************************************************
*
* os_getPciSpace
*
* Gets PCI memory base address and size.
*
* Users may change the default values or add their own code to determine the
* values appropriate for their system.
*
* See also module parameters pciAddr and pciSize. These can be used to override
* the defaults at module load time.
*
*
* RETURNS: VME_SUCCESS or an error code may be returned to the driver.
*
******************************************************************************/
int os_getPciSpace( unsigned long *baseAddress, unsigned long *size )
{

	*baseAddress = PCI_SPACE_START;
	*size = PCI_SPACE_SIZE;

	return VME_SUCCESS;
}


/******************************************************************************
*
* os_getXenResMemStart
*
* Gets the start address of the memory reserved when kernel is configured for
* Xen
*
* RETURNS: Reserve memory start address if kernel is configured for Xen else
* NULL.
*
******************************************************************************/
unsigned int os_getXenResMemStart(void)
{

#ifdef CONFIG_XEN
	return resMemStart;
#else
	return (unsigned int)NULL;
#endif
}

/******************************************************************************
*
* os_devQuery
*
* Gets the vendor and device id of the VME bridge detected
* 
*
* RETURNS: VME_SUCCESS or error code on failure
* 
*
******************************************************************************/	
unsigned int os_devQuery( unsigned int *vendor , unsigned int *device ,void *pciDev)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
	pciDev = pci_get_device( TUNDRA_PCI_VENDOR_ID , TSI148_PCI_DEVICE_ID ,NULL);
#else
	pciDev = pci_find_device( TUNDRA_PCI_VENDOR_ID , TSI148_PCI_DEVICE_ID ,NULL);
#endif
	if( pciDev != NULL )
	{
		*vendor = TUNDRA_PCI_VENDOR_ID;
		*device	= TSI148_PCI_DEVICE_ID;
		deviceIdent = TSI148;
		return VME_SUCCESS;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))	
	pciDev = pci_get_device ( TUNDRA_PCI_VENDOR_ID , UNIVERSE_PCI_DEVICE_ID ,NULL);
#else
	pciDev = pci_find_device ( TUNDRA_PCI_VENDOR_ID , UNIVERSE_PCI_DEVICE_ID ,NULL);
#endif
	
	if( pciDev != NULL )
	{
		*vendor = TUNDRA_PCI_VENDOR_ID;
		*device = UNIVERSE_PCI_DEVICE_ID;
		deviceIdent = UNIVERSEII;
		return VME_SUCCESS;
	}

	return -VME_ENODEV;
}

/******************************************************************************
*
* os_devIdent
*
* Returns the VME bridge detected
*
*
* RETURNS: 0  =UNIVERSEII
*	   1  =TSI148
*	  -1  =NONE 
******************************************************************************/
int os_devIdent( void )
{
	return deviceIdent;
}

/******************************************************************************
*
* os_refresh_mem_hook
*
* Solaris Specific:Refreshes the Solaris memory table 
*
*
* RETURNS: VME_SUCCESS or error code on failure
*
*
******************************************************************************/
void os_refresh_mem_hook( ULONG baseAddress , ULONG size )
{
}

/******************************************************************************
*
* os_platform_detect
*
* Detects if the platform is a 32bit or 64 bit system
*
*
* RETURNS: 0 - 32 Bit System
*	   1 - 64 Bit System	
*
******************************************************************************/
int os_platform_detect( void )
{
	return( (sizeof(long) == 8 )? 1: 0 );
}

/******************************************************************************
*
* os_ioctl_lock
*
* Detects if the platform is a 32bit or 64 bit system
*
*
* RETURNS: None
*
******************************************************************************/
void os_ioctl_lock( void )
{
  /*lock_kernel();*/
  #if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
        mutex_lock(&vmeDevIOCtrl);
  #else
        down(&vmeDevIOCtrl);
  #endif
}

/******************************************************************************
*
* os_ioctl_unlock
*
* Detects if the platform is a 32bit or 64 bit system
*
*
* RETURNS: none
*
******************************************************************************/
void os_ioctl_unlock( void )
{
  /*unlock_kernel();*/
  #if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33))
        mutex_unlock(&vmeDevIOCtrl);
  #else
        up(&vmeDevIOCtrl);
  #endif
}

/******************************************************************************
*
* os_kernel_usrHandler
*
* Perform calls to userHandler in kernel mode
*
*
* RETURNS: none
*
******************************************************************************/
void os_kernel_usrHandler( UINT32 intNum,  UINT32 vectorInfo)
{
  TSI148USRISR *usrIsr;

  list_for_each_entry(usrIsr, &usrTsi148IsrList[intNum], usrIsrList)
  {
    if(usrIsr->UserIntFn)
    {
        usrIsr->usrData.intNum = intNum;
        if((intNum > 0) && (intNum < 8))
          usrIsr->usrData.intVec = vectorInfo;
        else
          usrIsr->usrData.intVec = 0;

        (usrIsr->UserIntFn)(&usrIsr->usrData);
    }
  }
}

/******************************************************************************
*
* os_init_usrIsr
*
* Initialise KERNEL USER interrupt handler List.
*
*
* RETURNS: none
*
******************************************************************************/
void os_init_usrIsr()
{
  int i;

  for(i = 0; i < 26; i++)
  {
      INIT_LIST_HEAD(&usrTsi148IsrList[i]);
  }
}

/******************************************************************************
*
* os_kernel_usrHandler
*
* Register the given KERNEL USER interrupt handler for an IRQ.
*
*
* RETURNS: none
*
******************************************************************************/
int os_register_usrIsr(EN_VME_INT_DATA *iptr)
{
  TSI148USRISR *usrIsr;

  usrIsr = (TSI148USRISR *)kmalloc(sizeof(TSI148USRISR), GFP_KERNEL);
  usrIsr->intNum = iptr->intNum;
  usrIsr->UserIntFn = iptr->userInt;
  os_memcpy((void*)&(usrIsr->usrData), (void*)&(iptr->usrData), sizeof(EN_VME_INT_USR_DATA));
  list_add(&(usrIsr->usrIsrList), &usrTsi148IsrList[iptr->intNum]);
  return VME_SUCCESS;
}

/******************************************************************************
*
* os_remove_usrIsr
*
* Remove the given KERNEL USER interrupt handler for an IRQ.
*
*
* RETURNS: none
*
******************************************************************************/
int os_remove_usrIsr(EN_VME_INT_DATA *iptr)
{
  struct list_head *pos, *q;
  int result = -VME_EFAULT;

  list_for_each_safe( pos, q, &usrTsi148IsrList[iptr->intNum] )
  {
    TSI148USRISR *tmp;
    tmp = list_entry( pos, TSI148USRISR, usrIsrList);
    if(tmp->UserIntFn == iptr->userInt)
    {
        if(tmp->usrData.usrPtr)
        {
            if(tmp->usrData.usrPtr == iptr->usrData.usrPtr)
            {
                list_del(pos);
                kfree(tmp);
                result = VME_SUCCESS;
            }
        }
        else
        {
            list_del(pos);
            kfree(tmp);
            result = VME_SUCCESS;
        }
    }
  }
  return result;
}

/******************************************************************************
*
* os_get_efi_variable
*
* Obtain EFI variable data
*
*
*
*
* RETURNS: 0 on success -1 on failure
*
******************************************************************************/
int os_get_efi_variable(unsigned short *efiname, int name_length, char *efidata, unsigned long data_length)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32))
  unsigned long name_size = 1024;
  efi_char16_t *var_name;
  efi_guid_t vendor_guid;
  unsigned int attr;
  int ret, i;
  unsigned long long status=EFI_NOT_FOUND;

  var_name = kzalloc(name_size, GFP_KERNEL);
  do
  {
    status = efi.get_next_variable(&name_size, var_name, &vendor_guid);
    for(i = 0; i < name_length; i++)
    {
        if(efiname[i] == var_name[i])
        {
          ret = 0;
        }
        else
        {
          ret = -1;
          break;
        }
    }
    if(ret == 0)
    {
      break;
    }
  }
  while(status != EFI_NOT_FOUND);
  if(ret == 0)
  {
    status = efi.get_variable(var_name, &vendor_guid, &attr, &data_length, efidata);
  }
  kfree(var_name);
  return ret;
#else
  return -1;
#endif
}

/******************************************************************************
*
* os_efi
*
* Check EFI availability
*
*
*
*
* RETURNS: 1 on success 0 on failure
*
******************************************************************************/
int os_efi(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32))
  return (efi.get_next_variable?1:0);
#else
  return 0;
#endif
}


/******************************************************************************
*
* os_request_region
*
* Request for an I/O region
*
*
*
*
* RETURNS: Non-Null on success Null on failure
*
******************************************************************************/
struct resource* os_request_region(unsigned long first, unsigned long n, const char* name)
{
  return request_region(first, n, name);
}

/******************************************************************************
*
* os_release_region
*
* Release I/O region
*
*
*
*
* RETURNS: None
*
******************************************************************************/
void os_release_region(unsigned long start, unsigned long n)
{
  release_region(start, n);
}
