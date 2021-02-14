#
# (C) Copyright 2007 - 2019 North Atlantic Industries
#
# Michal SIMEK <monstr@monstr.eu>
#
# SPDX-License-Identifier:	GPL-2.0+
#

NAI_UBOOT		:= UBOOT
NAI_UBOOT_MAJOR_VER	:= 00001
NAI_UBOOT_MINOR_VER	:= 00000
NAI_UBOOT_VER		:= $(NAI_UBOOT_MAJOR_VER).$(NAI_UBOOT_MINOR_VER)
NAI_UBOOT_DATE		:= $(shell date +'%b %d %Y')
NAI_UBOOT_TIME		:= $(shell date +'%T')
NAI_UBOOT_BIN_DATE	:= $(shell date -d"$(NAI_UBOOT_DATE)" +'%Y%m%d')
NAI_UBOOT_BIN_TIME	:= $(shell date -d"$(NAI_UBOOT_TIME)" +'%H%M%S')
