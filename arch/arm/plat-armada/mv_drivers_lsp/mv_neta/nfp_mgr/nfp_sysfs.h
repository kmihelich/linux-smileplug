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
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

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
********************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File under the following licensing terms.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    *   Redistributions of source code must retain the above copyright notice,
	    this list of conditions and the following disclaimer.

    *   Redistributions in binary form must reproduce the above copyright
	notice, this list of conditions and the following disclaimer in the
	documentation and/or other materials provided with the distribution.

    *   Neither the name of Marvell nor the names of its contributors may be
	used to endorse or promote products derived from this software without
	specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef NFP_SYS_FS
#define NFP_SYS_FS

#include "nfp/mvNfpDefs.h"

/* Constants */
#define NAME_SIZE 17
#define MAC_SIZE 24
#define IPV6_MAX_SIZE 50

#define PRINT(s) (off += mvOsSPrintf(buf+off, s))

/* #define NFP_SYSFS_DBG_Y */
#ifdef NFP_SYSFS_DBG_Y
#define NFP_SYSFS_DBG(x) x
#else
#define NFP_SYSFS_DBG(x)
#endif /* NFP_SYSFS_DBG_Y */

void ipv6Parser(char *str, unsigned char *ipv6Addr);

#ifdef NFP_BRIDGE
int __init nfp_bridge_sysfs_init(struct kobject *nfp_kobj);
#endif /* NFP_BRIDGE */

#ifdef NFP_VLAN
int __init nfp_vlan_sysfs_init(struct kobject *nfp_kobj);
#endif /* NFP_VLAN */

#ifdef NFP_FIB
int __init nfp_fib_sysfs_init(struct kobject *nfp_kobj);
int __init nfp_fib6_sysfs_init(struct kobject *nfp_kobj);
int do_fib_add_del_age(const char *name, int family, unsigned char *sip,
				unsigned char *dip, char *oif, unsigned char *gateway);
int do_arp_add_del_age(const char *name, int family, unsigned char *gtw, char *macStr);
#endif /* NFP_FIB */

#ifdef NFP_CT
int __init nfp_ct_sysfs_init(struct kobject *nfp_kobj);
int __init nfp_ct6_sysfs_init(struct kobject *nfp_kobj);
int do_ct_age_del_forward(const char *name, int family, unsigned char *sip,
		unsigned char *dip, int sport, int dport, char *protoStr, int mode);
int get_protocol_num(char *str);
#ifdef NFP_LIMIT
int do_ct_tbf(const char *name, int family, unsigned char *sip,
		unsigned char *dip, int sport, int dport, char *protoStr, int tbfIndex);
#endif /* NFP_LIMIT */
#ifdef NFP_CLASSIFY
int do_ct_classify(const char *name, int family, unsigned char *sip,
		unsigned char *dip, int sport, int dport, char *protoStr, int val, int nval);
#endif /* NFP_CLASSIFY */
#endif /* NFP_CT */

#ifdef NFP_CLASSIFY
int __init nfp_cls_sysfs_init(struct kobject *nfp_kobj);
#endif /* NFP_CLASSIFY */

#ifdef NFP_PPP
	int __init nfp_ppp_sysfs_init(struct kobject *nfp_kobj);
#endif /* NFP_PPP */


#endif /* __INCmvCommonh */
