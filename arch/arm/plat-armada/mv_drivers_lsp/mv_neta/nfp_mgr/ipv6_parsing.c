/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.


********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/capability.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include "nfp_sysfs.h"

void insertAddr(char *str, int *addr, int count, int len)
{
	char aligned[5] = "0000", top[3], bottom[3];
	int itr = 0, i = 0;
	while (len++ < 4)
		itr++;
	while (itr < 4)
		aligned[itr++] = str[i++];
	top[0] = aligned[0];
	top[1] = aligned[1];
	bottom[0] = aligned[2];
	bottom[1] = aligned[3];
	top[2] = bottom[2] = '\0';
	sscanf(top, "%02x", addr+count);
	sscanf(bottom, "%02x", addr+count+1);
}

/* get a string of ipv6 address, parse it and insert right values to addr */
void ipv6Parser(char *str, unsigned char *ipv6Addr)
{
	char currentStr[5], *ptr = str;
	int i, j, count, len = 0;
	int addr[16];
	for (i = 0; i < 16; i++)
		addr[i] = -1;
	i = j = count = 0;
	memset(currentStr, 0, sizeof(currentStr));
	while (*ptr) {
		if (*ptr == ':') {
			insertAddr(currentStr, addr, count, len);
			memset(currentStr, 0, sizeof(currentStr));
			len = 0;
			count += 2;
			ptr++;
			continue;
		}
		currentStr[len++] = *ptr;
		ptr++;
	}
	insertAddr(currentStr, addr, count, len);
	j = 0;
	for (i = 15; i > 0; i -= 2) { /* find number of unfilled elements */
		if (addr[i] == -1)
			j += 2;
		else
			break;
	}
	if (!j) /* completely filled */
		goto ipv6_insert;
	for (i = 15 - j; i > 0; i -= 2) { /* shift numbers to the right */
		addr[i+j] = addr[i];
		addr[i+j-1] = addr[i-1];
		if (addr[i] == 0 && addr[i-1] == 0)
			break;
	}
	/* fill zero gap */
	for (i = 15; i > 0; i -= 2) {
		if (addr[i] == 0 && addr[i-1] == 0) {
			i -= 2;
			while (!(addr[i] == 0 && addr[i-1] == 0)) {
				addr[i] = addr[i-1] = 0;
				i -= 2;
			}
			break;
		}
	}
ipv6_insert:
	for (i = 0; i < 16; i++)
		ipv6Addr[i] = addr[i];
}



