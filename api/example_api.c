/******************************************************************************
*
* Filename:     example_api.c
*
* Description:  Application to invoke example IOCTLs
*
* Copyright 2000 - 2009 Concurrent Technologies.
*
******************************************************************************/

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "example_drv.h"

extern int errno;

int main( void )
{
	int fd,result;

	fd = open("/dev/vmeex/ctl",O_RDWR);

	if( fd >= 0 )
	{
		result = ioctl(fd, IOCTL_EX_INSCOUNT_CTL);

		if( result < 0 )
		{
			printf("Error executing IOCTL:(%s)\n",strerror(errno));
		}

		result = ioctl(fd,IOCTL_EX_LSI_IMAGE);

		if( result < 0 )
		{
			printf("Error executing IOCTL:(%s)\n",strerror(errno));
		}

		result = ioctl(fd,IOCTL_EX_VSI_IMAGE);

		if( result < 0 )
		{
			printf("Error executing IOCTL:(%s)\n",strerror(errno));
		}

		result = ioctl(fd,IOCTL_EX_DMA_DIRECT);

		if( result < 0 )
		{
			printf("Error executing IOCTL:(%s)\n",strerror(errno));
		}

		result = ioctl(fd,IOCTL_EX_DMA_LIST);

		if( result < 0 )
		{
			printf("Error executing IOCTL:(%s)\n",strerror(errno));
		}

		result = ioctl(fd,IOCTL_EX_INTERRUPT);

		if( result < 0 )
		{
			printf("Error executing IOCTL:(%s)\n",strerror(errno));
		}

		close(fd);
	}
	else
	{
		printf("Error opening the device(%s)\n",strerror(errno));
		result=-1;
	}
	
	return result;
}
