/******************************************************************************
*
* Filename:     example_appc.c
*
* Description:  Application to invoke example Interrupt Count IOCTLs
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

#define ENH_VECT_READ   0
#define BRIDGELESS_MODE 0

int main( void )
{
	int fd, result, i;
	unsigned char value;

	fd = open("/dev/vmeex/ctl",O_RDWR);

	if( fd >= 0 )
	{
#if (!BRIDGELESS_MODE)
#if (!ENH_VECT_READ)
	        int intCount[26] = {0};
		result = ioctl(fd,IOCTL_EX_REGUSRINT);

		if( result < 0 )
		{
			printf("Error executing IOCTL:(%s)\n",strerror(errno));
		}
		

		result = ioctl(fd, IOCTL_EX_GENERATE_IT,  1);
		sleep(1);
		result = ioctl(fd, IOCTL_EX_GENERATE_IT,  2);
		sleep(1);
		result = ioctl(fd, IOCTL_EX_GENERATE_IT,  1);
		result = ioctl(fd, IOCTL_EX_LSI_IMAGE);
		result = ioctl(fd, IOCTL_EX_LSI_IMAGE);
						
		result = ioctl(fd, IOCTL_EX_INTCOUNT, (unsigned long)&intCount);

		if( result < 0 )
		{
			printf("Error executing IOCTL:(%s)\n",strerror(errno));
			close(fd);
			return (-1);
		}

		for(i = 0; i < 26; i++)
		{
			printf("Int number: %d count: %d\n", i, intCount[i]);
		}

		result = ioctl(fd, IOCTL_EX_REMUSRINT);

		if( result < 0 )
		{
			printf("Error executing IOCTL:(%s)\n",strerror(errno));
			close(fd);
			return (-1);
		}
		result = ioctl(fd, IOCTL_EX_GET_GEO, &value);
		if( result == 0)
		{
		  printf("Boards Slot number: %d\n", value);
		}
		else
		{
                  printf("Error executing IOCTL:(%s)\n",strerror(errno));
                  close(fd);
                  return (-1);
		}
#else
		/* Enable Interrupt VIRQ 1 then run this test */
		result = ioctl(fd, IOCTL_EX_SETCAPMODE);
                if( result != 0)
                {
                  printf("Error executing IOCTL:(%s)\n",strerror(errno));
                  close(fd);
                  return (-1);
                }
                result = ioctl(fd, IOCTL_EX_ENHVECTREAD);
                if( result != 0)
                {
                  printf("Error executing IOCTL:(%s)\n",strerror(errno));
                  close(fd);
                  return (-1);
                }
#endif
#else
                result = ioctl(fd, IOCTL_EX_INTERRUPT_BRIDGELESS);
                if( result != 0)
                {
                  printf("Error executing IOCTL:(%s)\n",strerror(errno));
                  close(fd);
                  return (-1);
                }
                result = ioctl(fd, IOCTL_EX_GET_SYSFAIL, &value);
                if( result != 0)
                {
                  printf("Error executing IOCTL:(%s)\n",strerror(errno));
                  close(fd);
                  return (-1);
                }
                printf("Boards SYSFAIL Status: %d\n", value);
                result = ioctl(fd, IOCTL_EX_SET_SYSFAIL);
                if( result != 0)
                {
                  printf("Error executing IOCTL:(%s)\n",strerror(errno));
                  close(fd);
                  return (-1);
                }
                result = ioctl(fd, IOCTL_EX_CLEAR_SYSFAIL);
                if( result != 0)
                {
                  printf("Error executing IOCTL:(%s)\n",strerror(errno));
                  close(fd);
                  return (-1);
                }
                result = ioctl(fd, IOCTL_EX_GET_SYSFAIL, &value);
                if( result != 0)
                {
                  printf("Error executing IOCTL:(%s)\n",strerror(errno));
                  close(fd);
                  return (-1);
                }
                printf("Boards SYSFAIL Status: %d\n", value);
#endif
                i = 0;
		close(fd);
	}
	return 0;
}
