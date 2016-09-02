/******************************************************************************
*
* Filename: 	vme_version.h
* 
* Description:	Header file for version information
*
* $Revision: 1.28 $
*
* $Date: 2015-04-30 12:03:33 $
*
* $Source: /home/cvs/cvsroot/Linuxvme4/linuxvmeen/vme_version.h,v $
*
* Copyright 2000-2005 Concurrent Technologies.
*
******************************************************************************/

#ifndef __INCvme_version
#define __INCvme_version

static const char *creationDate  = __DATE__ ", " __TIME__;
static const char *copyright = "Copyright 2001-2015 Concurrent Technologies Plc";


#ifdef VME_DRIVER
static const char *vmeName      = "Linux Enhanced VME device driver";
char kernel_version[] = UTS_RELEASE;
static const char *vmeDriverVersion	= "Version 1.10.05";
#endif

#ifdef VME_LIB
static const char *vmeLibVersion = "Version 1.10.05";
#endif

#ifdef VME_API_EN
static const char *vmeEnApiVersion = "Version 1.10.05";
#endif
#endif	/* __INCvme_version */
