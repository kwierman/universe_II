#!/bin/bash

SCRIPTDIR=`dirname $0`
cd $SCRIPTDIR
PATH="${PATH}:/bin:/sbin"

CC="$1"
ISYSTEM=`$CC -print-file-name=include`
SOURCES=$2
HEADERS=$SOURCES/include
OUTPUT=$3


CFLAGS="-D__KERNEL__ "-DKBUILD_BASENAME=""" \
-nostdinc -isystem $ISYSTEM \
-Werror -Wimplicit-function-declaration \
-I$HEADERS -I$HEADERS/asm/mach-default"

case "$4" in
    remap_page_range)

        # Determine the number of arguments expected by remap_page_range function.

        echo "#include <linux/version.h>
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
	#include <linux/autoconf.h>
	#endif
	#include <linux/mm.h>
        int os_remap_page_range(void) {
           pgprot_t pgprot = __pgprot(0);
           remap_page_range(NULL, 0L, 0L, 0L, pgprot);
        }" > systest$$.c

        $CC $CFLAGS -c systest$$.c > /dev/null 2>&1
        rm -f systest$$.c

        if [ -f systest$$.o ]; then
          echo "5"
          rm -f systest$$.o
          exit 0
        fi

        echo "#include <linux/version.h>
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
        #include <linux/autoconf.h>
        #endif
	#include <linux/mm.h>
        int os_remap_page_range(void) {
           pgprot_t pgprot = __pgprot(0);
           remap_page_range(0L, 0L, 0L, pgprot);
        }" > systest$$.c

        $CC $CFLAGS -c systest$$.c > /dev/null 2>&1
        rm -f systest$$.c

        if [ -f systest$$.o ]; then
          echo "4"
          rm -f systest$$.o
          exit 0
        else
          # We couldn't determine the number of arguments.
          exit 1
        fi
    ;;

    remap_pfn_range)

        # Determine if remap_pfn_range() is present.

	echo "#include <linux/version.h>
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
        #include <linux/autoconf.h>
        #endif
	#include <linux/mm.h>
        int os_remap_pfn_range(void) {
            pgprot_t pgprot = __pgprot(0);
            remap_pfn_range(NULL, 0L, 0L, 0L, pgprot);
        }" > systest$$.c

        $CC $CFLAGS -c systest$$.c > /dev/null 2>&1

        rm -f systest$$.c

        if [ -f systest$$.o ]; then
            rm -f systest$$.o
            echo 1
        else
            echo 0
        fi
    ;;
esac
