/*
 * (C) Copyright 2014 North Atlantic Industries, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _NAI_COMMON_H
#define _NAI_COMMON_H

typedef struct Nai_fpga_build_time {
   u8 day;
   u8 month;
   u16 year;
   u8 hour;
   u8 minute;
   u8 second;
} Nai_fpga_build_time;

#define BUILD_TIME_STRING_LEN 24

typedef struct Nai_version {
	u16    fsblRevHiLo;
	u8     fsblRevPatchLevel;
	u8     fsblRevSubLevel;
	u16    fsblMajor;
        u16    fsblMinor;	
	char   fsblBuildTimeStr[BUILD_TIME_STRING_LEN];
	u16    ubootRevHiLo;
	u8     ubootRevPatchLevel;
	u8     ubootRevSubLevel;
	u16    ubootMajor;
	u16    ubootMinor;
	char   ubootBuildTimeStr[BUILD_TIME_STRING_LEN];
	u32    fpgaRev;
	u16    fpgaMajor;
	u16    fpgaMinor;
	u32    fpgaBuildTime;
	Nai_fpga_build_time fpgaDecodedBuildTime;
	u8     fwRevMajor;
	u8     fwRevMinor;
	char   fwBuildTimeStr[BUILD_TIME_STRING_LEN];
	u8     kernelRevMajor;
	u8     kernelRevMinor;
	u8     kernelRevBuild;
	char   kernelBuildTimeStr[BUILD_TIME_STRING_LEN];
	u8     rootfsRevMajor;
	u8     rootfsRevMinor;
	u8     rootfsRevBuild;
	char   rootfsBuildTimeStr[BUILD_TIME_STRING_LEN];
} Nai_version;

void nai_decode_fpga_compile_count(Nai_fpga_build_time *priv, u32 buildTime);

#endif /*_NAI_COMMON_H*/
