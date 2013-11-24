
# Copyright (C) 2006, 2007 Junjiro Okajima
# Copyright (C) 2006, 2007 Tomas Matejicek, slax.org
#
# LICENSE follows the described ones in lzma and squashfs.

# $Id: Makefile,v 1.1 2010-10-29 09:43:45 steven Exp $

# paths
Sqlzma = ${CURDIR}
LzmaVer = lzma457
Lzma = ${Sqlzma}/${LzmaVer}
SqVer = squashfs3.4
Squashfs = ${Sqlzma}/${SqVer}

LzmaC = ${Lzma}/C/Compress/Lzma/
LzmaAlone = ${Lzma}/CPP/7zip/Compress/LZMA_Alone/

SqTools = ${Squashfs}/squashfs-tools

# enable it if you want to add -g option when compiling
UseDebugFlags =
# disable it if you don't want to compile squashfs kernel module here
BuildSquashfs = 1

export

all:
	${MAKE} -C ${LzmaC} -f sqlzma.mk $@
	${MAKE} -C ${LzmaAlone} -f sqlzma.mk $@
	${MAKE} -C ${SqTools} $@

clean:
	${MAKE} -C ${LzmaC} -f sqlzma.mk $@
	${MAKE} -C ${LzmaAlone} -f sqlzma.mk $@
	${MAKE} -C ${SqTools} $@
	${RM} *~

install: all
	-cp squashfs3.4/squashfs-tools/mksquashfs /opt/buildroot-gcc342/bin/mksquashfs_lzma-3.4
	-cp ${LzmaAlone}/lzma /opt/buildroot-gcc342/bin/lzma_alone

########################################



