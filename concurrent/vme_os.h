/******************************************************************************
*
* Filename: 	vme_os.h
* 
* Description:	Header file for OS wrapper functions
*
* $Revision: 1.9 $
*
* $Date: 2014-12-17 09:14:51 $
*
* $Source: /home/cvs/cvsroot/Linuxvme4/linuxvmeen/vme_os.h,v $
*
* Copyright 2000-2005 Concurrent Technologies.
*
******************************************************************************/

#ifndef __INCvme_os
#define __INCvme_os

void os_mdelay( unsigned long msec );
unsigned char os_inb( unsigned int address );
void os_outb( unsigned char value, unsigned int address );
unsigned char os_inb_p( unsigned int address );
void os_outb_p( unsigned char value, unsigned int address );

#if defined(__powerpc__)
unsigned int os_readPvr( void );
#endif

unsigned char os_readb( void *addr );
void os_writeb( unsigned char val, void *addr );
unsigned short os_readw( void *addr );
void os_writew( unsigned short val, void *addr );
unsigned int os_readl( void *addr );
void os_writel( unsigned int val, void *addr );

unsigned long os_disable_interrupts( void );
void os_enable_interrupts( unsigned long flags );

void *os_kmalloc( unsigned int size );
void os_kfree( const void *objp );
void *os_alloc_wqueue( void );

void *os_dma_cmd_pkt_alloc( unsigned long size );
void os_dma_cmd_pkt_free( const void * objp);

int os_schedule( void );
int os_signal_pending( void );
int os_interruptible_sleep_on( void *pWqueue );
void os_wake_up_interruptible( void *pWqueue );
int os_waitqueue_active( void *pWqueue );

void *os_alloc_timer( void );
void os_start_timer( void *pTimer, void (*function)( unsigned long ), 
							unsigned long arg, unsigned long timeout );
void os_del_timer( void *pTimer );
void *os_alloc_sema( void );
void os_acquire_sema( void *pSema );
void os_release_sema( void *pSema );

void *os_ioremap( unsigned long offset, unsigned long size );
void os_iounmap( void *addr );
int os_remap_page_range( void *pVma, unsigned long phys_add, unsigned long size );

void *os_memcpy( void* to, const void *from, unsigned int n );
void *os_memset( void* s, char c, unsigned int count);
char *os_strcpy( char* dest, const char *src );
int os_strcmp( const char *cs, const char *ct );

unsigned int os_page_align( unsigned int size );
void os_getMemInfo( UMEMINFO *pmi );
void *os_high_memory( void );
unsigned long os_virt_to_phys( volatile void *address );
int os_page_shift( void );
unsigned long os_page_size( void );

int os_sprintf( char *buf, const char *fmt, ... );
void os_logMsg( char *fmt, ... );

unsigned long os_copy_from_user( void *dest, void *src, unsigned long size );
unsigned long os_copy_from_user_nc( void *dest, const void *src, unsigned long size );
unsigned long os_copy_to_user( void *dest, void *src, unsigned long size );
unsigned long os_copy_to_user_nc( void *dest, const void *src, unsigned long size );
void os_put_user8_nc( UINT8 value, UINT8 *src );
void os_put_user16_nc( UINT16 value, UINT16 *src );
void os_put_user32_nc( UINT32 value, UINT32 *src );
void os_put_user64_nc( ULONG value, ULONG *src );
unsigned long os_get_user8_nc( UINT8 *value, const UINT8 *src );
unsigned long os_get_user16_nc( UINT16 *value, const UINT16 *src );
unsigned long os_get_user32_nc( UINT32 *value, const UINT32 *src );
unsigned long os_get_user64_nc( ULONG *value, const ULONG *src );
unsigned long os_get_user32( UINT32 *value, const UINT32 *src );
unsigned long os_get_user64( ULONG *value, const ULONG *src);
void os_memcpy_fromio( void *dest, void *src, unsigned long count );
void os_memcpy_toio( void *dest, const void *src, unsigned long count );

UINT32 os_cpu_to_le32( UINT32 value );
UINT32 os_le32_to_cpu( UINT32 value );
unsigned long os_virt_to_bus( void *address );

int os_request_irq( unsigned int irq, const char *devname );
int os_request_irq_bridgeless( unsigned int irq, const char *devname );
void os_free_irq( unsigned int irq );
void os_free_irq_bridgeless( unsigned int irq );

int os_find_bridge( UNIPCIINFO *pUni );
int os_disable_bridge( UNIPCIINFO *pUni );

int os_getPciSpace( unsigned long *baseAddress, unsigned long *size );
unsigned int os_getXenResMemStart( void );
unsigned int os_devQuery( unsigned int *vendor , unsigned int *device ,void *pciDev);
int os_strncmp( const char *cs, const char *ct, unsigned long count );
void os_refresh_mem_hook( ULONG baseAddress , ULONG size );
void os_interruptible_condition( void *pWqueue, int status );
void os_spinlock_init( void );
unsigned long os_spinlock_lock( void  );
void os_spinlock_unlock( unsigned long flags );
int os_platform_detect( void );
int os_devIdent( void );
void os_ioctl_lock( void );
void os_ioctl_unlock( void );
void os_kernel_usrHandler( UINT32 intNum,  UINT32 vectorInfo);
int os_register_usrIsr(EN_VME_INT_DATA *iptr);
int os_remove_usrIsr(EN_VME_INT_DATA *iptr);
void os_init_usrIsr(void);
int os_get_efi_variable(unsigned short *efiname, int name_length, char *efidata, unsigned long data_length);
int os_efi(void);
struct resource* os_request_region(unsigned long first, unsigned long n, const char* name);
void os_release_region(unsigned long start, unsigned long n);
#endif	/* __INCvme_os */
