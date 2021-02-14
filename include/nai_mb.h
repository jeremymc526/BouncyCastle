/*
 * (C) Copyright 2014 North Atlantic Industries, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _NAI_MB_H
#define _NAI_MB_H

#include <nai_common.h>

typedef struct Nai_mb
{
	struct Nai_version masterVer;
#ifdef CONFIG_NAI_ZYNQ_SLAVE
	struct Nai_version slaveVer;
#endif	
} Nai_mb;

//Get MB Version and Other Info
void nai_update_mb_version(void);
void nai_get_mb_info(Nai_mb* mb);


#endif /*_NAI_MB_H*/
