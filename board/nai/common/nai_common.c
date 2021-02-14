/*
 * North Atlantic Industries
 * tyang@naii.com
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <nai_common.h>

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DEBUGF(x...) printf(x)
#else
#define DEBUGF(x...)
#endif /* DEBUG */

void nai_decode_fpga_compile_count(Nai_fpga_build_time *parm, u32 buildTime)
{
	Nai_fpga_build_time *priv = parm;
		
	//5 bits ddddd 31days
	priv->day = ((buildTime & 0xf8000000) >> 27); 
	//4 bits mmmm 12 months
	priv->month = ((buildTime & 0x07800000) >> 23); 
	//6 bits yyyyyy 0 t0 63 years 2000 ~ 2063
	priv->year = ((buildTime & 0x007e0000) >> 17);
	priv->year += 2000;
	//5 bits hhhhhh 00 ~ 23 hours
	priv->hour = ((buildTime & 0x0001f000) >> 12); 
	//6 bits mmmmmm 59 minutes
	priv->minute = ((buildTime & 0x00000fc0) >> 6);
	//6 bits ssssss 59 secnods
	priv->second = ((buildTime & 0x0000003f) >> 0);
	
}
