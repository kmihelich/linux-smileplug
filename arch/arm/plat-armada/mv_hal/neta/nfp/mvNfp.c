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

/*******************************************************************************
* mvNfp.c - Marvell Network Fast Processing
*
* DESCRIPTION:
*
*       Supported Features:
*       - OS independent.
*
*******************************************************************************/

/* includes */
#include "mvOs.h"
#include "mv802_3.h"
#include "mvDebug.h"
#include "gbe/mvEthRegs.h"
#include "gbe/mvNetaRegs.h"
#include "gbe/mvNeta.h"
#include "mvNfp.h"

/* Set Debug Level */
MV_U32 nfpDebugLevel;

MV_U32 nfp_ports[NFP_MAX_PORTS];

NFP_IF_MAP *nfp_if_map[NFP_DEV_HASH_SZ];
NFP_IF_MAP *nfp_if_real_map[NFP_MAX_PORTS][NFP_MAX_SWITCH_GROUPS];

unsigned int nfp_jhash_iv;

#if (CONFIG_MV_ETH_NFP_DEF == 0)
static int nfpMode = MV_NFP_DISABLED;
#else
static int nfpMode = CONFIG_MV_ETH_NFP_MODE_DEF;
#endif
static int nfpFreeExtPort = MV_ETH_MAX_PORTS;
static NFP_STATS nfp_stats[NFP_MAX_PORTS];

#ifdef NFP_CLASSIFY
MV_NFP_CLASSIFY_POLICY dscpPolicy = MV_NFP_CLASSIFY_HIGHEST;
MV_NFP_CLASSIFY_POLICY vlanPrioPolicy = MV_NFP_CLASSIFY_HIGHEST;
MV_NFP_CLASSIFY_POLICY txqPolicy = MV_NFP_CLASSIFY_HIGHEST;
MV_NFP_CLASSIFY_POLICY txpPolicy = MV_NFP_CLASSIFY_HIGHEST;
MV_NFP_CLASSIFY_POLICY mhPolicy = MV_NFP_CLASSIFY_HIGHEST;
#endif /* NFP_CLASSIFY */

#ifdef NFP_STAT
#define NFP_INC(p, s) nfp_stats[p].s++;
#else
#define NFP_INC(p, s)
#endif

/*
 * Init
 */
MV_VOID _INIT mvNfpInit(MV_VOID)
{
	mvOsMemset(nfp_ports, 0, sizeof(MV_U32) * NFP_MAX_PORTS);
	mvOsMemset(nfp_stats, 0, sizeof(NFP_STATS) * NFP_MAX_PORTS);
	mvOsMemset(nfp_if_real_map, 0, sizeof(nfp_if_real_map));
	mvOsMemset(nfp_if_map, 0, sizeof(nfp_if_map));
	nfpDebugLevel = NFP_WARN_PRINT; /* Note: can also be (NFP_DBG_PRINT | NFP_WARN_PRINT) */
}

MV_VOID   mvNfpModeSet(int mode)
{
	nfpMode = mode;
}

MV_U32 mvNfpPortCapGet(MV_U32 port)
{
	return nfp_ports[port];
}

MV_VOID mvNfpDebugLevelSet(int dbgLevelFlags)
{
	nfpDebugLevel = dbgLevelFlags;
}

static INLINE int needFragment(MV_IP_HEADER_INFO *pIpInfo, NFP_IF_MAP *pOutIf)
{
	return (pIpInfo->ipLen > pOutIf->mtu);
}

/* Update packet's MAC header, including Marvell Header, DA and SA */
static INLINE void mvNfpFibMacUpdate(MV_U8 *pData, NFP_RULE_FIB *pFib)
{
	*(MV_U32 *) (pData + 0) = *(MV_U32 *) (&pFib->mh);
	*(MV_U32 *) (pData + 4) = *(MV_U32 *) (&pFib->da[2]);
	*(MV_U32 *) (pData + 8) = *(MV_U32 *) (&pFib->sa[0]);
	*(MV_U16 *) (pData + 12) = *(MV_U16 *) (&pFib->sa[4]);
}

/* Update packet's IPv4 Header (decrement TTL field) */
static INLINE void mvNfpFibIpUpdate(MV_IP_HEADER_INFO *pIpInfo)
{
	if (pIpInfo->family == MV_INET)
		pIpInfo->ip_hdr.ip4->ttl--;
	else
		pIpInfo->ip_hdr.ip6->hoplimit--;
}

#ifdef NFP_CLASSIFY
static INLINE MV_VOID mvNfpClassifyInit(NFP_CLASSIFY_INFO *info)
{
	info->flags = 0;
	info->pkt_vlan_prio = NFP_INVALID_VPRIO;
	info->bridge_vlan_prio = NFP_INVALID_VPRIO;
	info->ct_vlan_prio = NFP_INVALID_VPRIO;
	info->ct_dscp = NFP_INVALID_DSCP;
}

static INLINE MV_VOID mvNfpVpriPktClassifySave(MV_U16 vlanId, NFP_CLASSIFY_INFO *info)
{
	info->pkt_vlan_prio = (vlanId >> 13); /* save 3 MSBits in VLAN ID */
	info->flags |= NFP_F_SET_VLAN_PRIO;
}

static INLINE MV_VOID mvNfpVpriBridgeClassifySave(MV_U8 *pData, NFP_RULE_BRIDGE	*bridgeRule, NFP_CLASSIFY_INFO *info)
{
	int i;
	MV_U16 eth_type;

	if (info->pkt_vlan_prio != NFP_INVALID_VPRIO)
		eth_type = MV_16BIT_BE(*(MV_U16 *)(pData + MV_ETH_MH_SIZE + MV_MAC_ADDR_SIZE +
							   MV_MAC_ADDR_SIZE + MV_VLAN_HLEN));
	else
		eth_type = MV_16BIT_BE(*(MV_U16 *)(pData + MV_ETH_MH_SIZE + MV_MAC_ADDR_SIZE + MV_MAC_ADDR_SIZE));

	for (i = 0; i < NFP_VPRI_MAP_GLOBAL; i++) {
		if ((bridgeRule->vpri_map[i].eth_type == eth_type) && bridgeRule->vpri_map[i].valid) {
			info->bridge_vlan_prio = bridgeRule->vpri_map[i].new_prio;
			info->flags |= NFP_F_SET_VLAN_PRIO;
			return;
		}
	}
	if (bridgeRule->vpri_map[NFP_VPRI_MAP_GLOBAL].valid) {
		info->bridge_vlan_prio = bridgeRule->vpri_map[NFP_VPRI_MAP_GLOBAL].new_prio;
		info->flags |= NFP_F_SET_VLAN_PRIO;
	}
}

#ifdef NFP_CT
static INLINE MV_VOID mvNfpDscpClassifySave(MV_IP_HEADER_INFO *pIpHdr, NFP_RULE_CT *pCt, NFP_CLASSIFY_INFO *info)
{
		MV_IP6_HEADER *pIp6Hdr = pIpHdr->ip_hdr.ip6;
		MV_IP_HEADER *pIph = pIpHdr->ip_hdr.ip4;
		MV_U8 *pIp6 = (char *)pIp6Hdr;

		/* First get the packet's original DSCP */
		if (pIpHdr->family == MV_INET) {
			/* 6 MSBits of the TOS field are DSCP, 2 LSBits are ECN */
			info->pkt_dscp = ((pIph->tos) >> 2);
			info->pkt_ecn = (pIph->tos & 0x3);
		} else {
			/* TC field is divided on the first 2 bytes of IPv6 header */
			info->pkt_dscp = (((*pIp6 & 0xF) << 4) | (*(pIp6 + 1) >> 6));
			info->pkt_ecn = ((*(pIp6 + 1) >> 4) & 0x3);
		}

		/* Now get the DSCP value from the 5 tuple rule */
		if (pCt->dscp_map[info->pkt_dscp].valid)
			info->ct_dscp = pCt->dscp_map[info->pkt_dscp].new_dscp;
		else if (pCt->dscp_map[NFP_DSCP_MAP_GLOBAL].valid)
			info->ct_dscp = pCt->dscp_map[NFP_DSCP_MAP_GLOBAL].new_dscp;
}

static INLINE MV_VOID mvNfpVpriCtClassifySave(NFP_RULE_CT *pCt, NFP_CLASSIFY_INFO *info)
{
	if (info->pkt_vlan_prio != NFP_INVALID_VPRIO) {
		if (pCt->vpri_map[info->pkt_vlan_prio].valid) {
			info->ct_vlan_prio = pCt->vpri_map[info->pkt_vlan_prio].new_prio;
			info->flags |= NFP_F_SET_VLAN_PRIO;
			return;
		}
	}
	if (pCt->vpri_map[NFP_VPRI_MAP_GLOBAL].valid) {
		info->ct_vlan_prio = pCt->vpri_map[NFP_VPRI_MAP_GLOBAL].new_prio;
		info->flags |= NFP_F_SET_VLAN_PRIO;
	}
}
#endif /* NFP_CT */

static INLINE MV_VOID mvNfpTxqClassifySave(MV_U8 txq, NFP_CLASSIFY_INFO *info)
{
	if (info->flags & NFP_F_SET_TXQ) {
		/* update info->txq according to policy */
		switch (txqPolicy) {
		case MV_NFP_CLASSIFY_FIRST:
			/* do nothing - this is not the first time this function was called */
			break;
		case MV_NFP_CLASSIFY_LAST:
			info->txq = txq;
			break;
		case MV_NFP_CLASSIFY_HIGHEST:
			if (txq > info->txq)
				info->txq = txq;
			break;
		case MV_NFP_CLASSIFY_LOWEST:
			if (txq < info->txq)
				info->txq = txq;
			break;
		default:
			NFP_WARN("%s: unknown txq policy %d\n", __func__, txqPolicy);
			return;
		}
	} else {
		info->flags |= NFP_F_SET_TXQ;
		info->txq = txq;
	}
}

static INLINE MV_VOID mvNfpTxpClassifySave(MV_U8 txp, NFP_CLASSIFY_INFO *info)
{
	if (info->flags & NFP_F_SET_TXP) {
		/* update info->txp according to policy */
		switch (txpPolicy) {
		case MV_NFP_CLASSIFY_FIRST:
			/* do nothing - this is not the first time this function was called */
			break;
		case MV_NFP_CLASSIFY_LAST:
			info->txp = txp;
			break;
		case MV_NFP_CLASSIFY_HIGHEST:
			if (txp > info->txp)
				info->txp = txp;
			break;
		case MV_NFP_CLASSIFY_LOWEST:
			if (txp < info->txp)
				info->txp = txp;
			break;
		default:
			NFP_WARN("%s: unknown txp policy %d\n", __func__, txpPolicy);
			return;
		}
	} else {
		info->flags |= NFP_F_SET_TXP;
		info->txp = txp;
	}
}

static INLINE MV_VOID mvNfpMhClassifySave(MV_U16 mh, NFP_CLASSIFY_INFO *info)
{
	if (info->flags & NFP_F_SET_MH) {
		/* update info->mh according to policy */
		switch (mhPolicy) {
		case MV_NFP_CLASSIFY_FIRST:
			/* do nothing - this is not the first time this function was called */
			break;
		case MV_NFP_CLASSIFY_LAST:
			info->mh = mh;
			break;
		case MV_NFP_CLASSIFY_HIGHEST:
			if (mh > info->mh)
				info->mh = mh;
			break;
		case MV_NFP_CLASSIFY_LOWEST:
			if (mh < info->mh)
				info->mh = mh;
			break;
		default:
			NFP_WARN("%s: unknown MH policy %d\n", __func__, mhPolicy);
			return;
		}
	} else {
		info->flags |= NFP_F_SET_MH;
		info->mh = mh;
	}
}

#ifdef NFP_CT
static INLINE MV_VOID mvNfpDscpClassifyUpdate(MV_IP_HEADER_INFO *pIpHdr, NFP_CLASSIFY_INFO *info)
{
	MV_IP6_HEADER *pIp6Hdr = pIpHdr->ip_hdr.ip6;
	MV_IP_HEADER *pIph = pIpHdr->ip_hdr.ip4;
	MV_U8 *pIp6 = (char *)pIp6Hdr;
	MV_U8 new_dscp;

	switch (dscpPolicy) {
	case MV_NFP_CLASSIFY_FIRST:
		new_dscp = info->pkt_dscp; /* should always be valid */
		break;
	case MV_NFP_CLASSIFY_LAST:
		if (info->ct_dscp != NFP_INVALID_DSCP)
			new_dscp = info->ct_dscp;
		else
			new_dscp = info->pkt_dscp;
		break;
	case MV_NFP_CLASSIFY_HIGHEST:
		new_dscp = (info->ct_dscp > info->pkt_dscp) ? info->ct_dscp : info->pkt_dscp;
		break;
	case MV_NFP_CLASSIFY_LOWEST:
		if (info->ct_dscp != NFP_INVALID_DSCP)
			new_dscp = (info->ct_dscp < info->pkt_dscp) ? info->ct_dscp : info->pkt_dscp;
		else
			new_dscp = info->pkt_dscp;
		break;
	default:
		NFP_WARN("%s: unknown DSCP policy %d\n", __func__, dscpPolicy);
		return;
	}

	/* now update the packet */
	if (new_dscp != info->pkt_dscp) {
		if (pIpHdr->family == MV_INET) {
			pIph->tos = ((new_dscp << 2) | info->pkt_ecn);
		} else {
			*pIp6 &= ~0xF;			/* Clear 4 LSBits of 1st byte of IPv6 header */
			*pIp6 |= (new_dscp >> 4);	/* Set 4 MSBits of new Traffic Class value */
			pIp6++;
			*pIp6 &= ~0xF0;			/* Clear 4 MSBits of 2nd byte of IPv6 header */
			*pIp6 |= (((new_dscp & 0x3) << 6) | (info->pkt_ecn << 4));	/* Set 4 LSBits of new Traffic Class value */
		}
	}
}
#endif /* NFP_CT */

static INLINE MV_VOID mvNfpVpriClassifyUpdate(MV_U16 *vid, NFP_CLASSIFY_INFO *info)
{
	int vprio = 0;

	switch (vlanPrioPolicy) {
	case MV_NFP_CLASSIFY_FIRST:
		if (info->pkt_vlan_prio != NFP_INVALID_VPRIO)
			vprio = info->pkt_vlan_prio;
		else if (info->bridge_vlan_prio != NFP_INVALID_VPRIO)
			vprio = info->bridge_vlan_prio;
		else if (info->ct_vlan_prio != NFP_INVALID_VPRIO)
			vprio = info->ct_vlan_prio;
		break;
	case MV_NFP_CLASSIFY_LAST:
		if (info->ct_vlan_prio != NFP_INVALID_VPRIO)
			vprio = info->ct_vlan_prio;
		else if (info->bridge_vlan_prio != NFP_INVALID_VPRIO)
			vprio = info->bridge_vlan_prio;
		else if (info->pkt_vlan_prio != NFP_INVALID_VPRIO)
			vprio = info->pkt_vlan_prio;
		break;
	case MV_NFP_CLASSIFY_HIGHEST:
		vprio = info->pkt_vlan_prio;
		if (info->bridge_vlan_prio > vprio)
			vprio = info->bridge_vlan_prio;
		if (info->ct_vlan_prio > vprio)
			vprio = info->ct_vlan_prio;
		if (vprio == NFP_INVALID_VPRIO)
			vprio = 0;
		break;
	case MV_NFP_CLASSIFY_LOWEST:
		vprio = info->pkt_vlan_prio;
		if ((info->bridge_vlan_prio != NFP_INVALID_VPRIO) && (info->bridge_vlan_prio < vprio))
			vprio = info->bridge_vlan_prio;
		if ((info->ct_vlan_prio != NFP_INVALID_VPRIO) && (info->ct_vlan_prio < vprio))
			vprio = info->ct_vlan_prio;
		if (vprio == NFP_INVALID_VPRIO)
			vprio = 0;
		break;
	default:
		NFP_WARN("%s: unknown VLAN Priority policy %d\n", __func__, vlanPrioPolicy);
		return;
	}

	(*vid) &= ~0xE000;
	(*vid) |= (vprio << 13); /* Set 3 MSBits */
}

static INLINE MV_VOID mvNfpMhClassifyUpdate(MV_U8 *pData, NFP_CLASSIFY_INFO *info)
{
	*(MV_U16 *)(pData) = MV_16BIT_BE(info->mh);
}
#endif /* NFP_CLASSIFY */

#ifdef NFP_VLAN
MV_STATUS mvNfpVlanPvidSet(int if_index, MV_U16 pvid)
{
	NFP_IF_MAP *vlanIf = mvNfpIfMapGet(if_index);

	if (vlanIf == NULL) {
		mvOsPrintf("%s: interface %d is not valid\n", __func__, if_index);
		return MV_NOT_FOUND;
	}

	if (pvid == NFP_INVALID_VLAN)
		vlanIf->flags &= ~NFP_F_MAP_VLAN_PVID;
	else
		vlanIf->flags |= NFP_F_MAP_VLAN_PVID;

	vlanIf->pvid = pvid;

	return MV_OK;
}

MV_STATUS mvNfpVlanVidSet(int if_index, MV_U16 vid)
{
	NFP_IF_MAP *vlanIf = mvNfpIfMapGet(if_index);

	if (vlanIf == NULL) {
		mvOsPrintf("%s: interface %d is not valid\n", __func__, if_index);
		return MV_NOT_FOUND;
	}
	vlanIf->vlanId = vid;

	return MV_OK;
}

MV_STATUS mvNfpVlanVidGet(int if_index, MV_U16 *vid)
{
	NFP_IF_MAP *vlanIf = mvNfpIfMapGet(if_index);

	if (vlanIf == NULL) {
		mvOsPrintf("%s: interface %d is not valid\n", __func__, if_index);
		return MV_NOT_FOUND;
	}
	*vid = vlanIf->vlanId;

	return MV_OK;
}

/* Find virtual interface match vlanId */
static INLINE NFP_IF_MAP *mvNfpVlanIfFind(NFP_IF_MAP *ifMap, MV_U16 vlanId)
{
	NFP_IF_MAP *vlanIf;

	if (ifMap->vlanId == vlanId)
		return ifMap;

	vlanIf = ifMap->virtIf;
	while (vlanIf != NULL) {
		if ((vlanIf->vlanId == vlanId))
			return vlanIf;

		vlanIf = vlanIf->virtNext;
	}
	return NULL;
}

static INLINE MV_STATUS mvNfpVlanRx(int port, NETA_RX_DESC *pRxDesc, MV_U8 *pData,
				    MV_ETH_PKT *pPkt, NFP_IF_MAP **ppIfMap,
				    MV_NFP_RESULT *pRes)
{
	MV_U16     vlanId;
	NFP_IF_MAP *vlanIfMap = NULL, *ifMap = *ppIfMap;

	/* PVID processing */
	if (NETA_RX_IS_VLAN(pRxDesc)) {
		/* tagged packets */
		if (ifMap->flags & NFP_F_MAP_VLAN_RX_DROP_TAGGED) {
			/* Drop tagged packets */
			NFP_INC(port, vlan_rx_tag_drop);
			return MV_DROPPED;
		}
		vlanId = *((MV_U16 *)(pData + MV_ETH_MH_SIZE + sizeof(MV_802_3_HEADER)));
		vlanId = MV_16BIT_BE(vlanId);
#ifdef NFP_CLASSIFY
		mvNfpVpriPktClassifySave(vlanId, &(pRes->classifyInfo));
#endif /* NFP_CLASSIFY */
		vlanId &= 0xFFF;
	} else {
		/* Untagged packet */
		if (ifMap->flags & NFP_F_MAP_VLAN_RX_DROP_UNTAGGED) {
			/* Drop untagged packets */
			NFP_INC(port, vlan_rx_untag_drop);
			return MV_DROPPED;
		}
		if (ifMap->flags & NFP_F_MAP_VLAN_PVID)
			vlanId = ifMap->pvid;
		else
			vlanId = NFP_INVALID_VLAN;
	}
	pPkt->vlanId = vlanId;

	if (vlanId != NFP_INVALID_VLAN) {
		/* Tagged packet */
		/* look for vlanId through virtual interfaces mapped to this ifMap */
		vlanIfMap = mvNfpVlanIfFind(ifMap, vlanId);
		if (vlanIfMap) {
			/* found */
			NFP_INC(port, vlan_rx_found);
			*ppIfMap = vlanIfMap;
			return MV_CONTINUE;
		}
		/* not found */
		if (ifMap->flags & NFP_F_MAP_VLAN_RX_DROP_UNKNOWN) {
			/* Drop packets with unknown VIDs */
			NFP_INC(port, vlan_rx_unknown_drop);
			return MV_DROPPED;
		}
	}
	/* Default - Transparent mode */
	NFP_INC(port, vlan_rx_trans);
	return MV_CONTINUE;
}

static INLINE int mvNfpVlanAdd(int port, MV_U8 *pData, MV_U16 vid, MV_BOOL moveMac, MV_NFP_RESULT *pRes)
{
	MV_U8  *pNew;
	MV_U16 *pVlan;

	NFP_INC(port, vlan_tx_add);

	pNew = pData - MV_VLAN_HLEN;
	if (moveMac) {
		/* move MAC header 4 bytes left. Copy 12 bytes (DA + SA) */
		*(MV_U32 *)(pNew + 2) = *(MV_U32 *)(pData + 2);
		*(MV_U32 *)(pNew + 2 + 4) = *(MV_U32 *)(pData + 2 + 4);
		*(MV_U32 *)(pNew + 2 + 4 + 4) = *(MV_U32 *)(pData + 2 + 4 + 4);
	}
	pVlan = (MV_U16 *)(pNew + MV_ETH_MH_SIZE + MV_MAC_ADDR_SIZE + MV_MAC_ADDR_SIZE);
	/* Set VLAN Type - 0x8100 */
	*pVlan = MV_16BIT_BE(MV_VLAN_TYPE);
	pVlan++;
	/* Set VID + priority */
#ifdef NFP_CLASSIFY
	if (pRes->classifyInfo.flags & NFP_F_SET_VLAN_PRIO)
		mvNfpVpriClassifyUpdate(&vid, &(pRes->classifyInfo));
#endif /* NFP_CLASSIFY */
	*pVlan = MV_16BIT_BE(vid);

	return -MV_VLAN_HLEN;
}

static INLINE int mvNfpVlanRemove(int port, MV_U8 *pData, MV_BOOL moveMac)
{
	MV_U8  *pNew;

	NFP_INC(port, vlan_tx_remove);

	if (moveMac) {
		/* move MAC header 4 bytes right. Copy 12 bytes (DA + SA) */
		pNew = pData + MV_VLAN_HLEN;
		*(MV_U32 *)(pNew + 2 + 4 + 4) = *(MV_U32 *)(pData + 2 + 4 + 4);
		*(MV_U32 *)(pNew + 2 + 4) = *(MV_U32 *)(pData + 2 + 4);
		*(MV_U32 *)(pNew + 2) = *(MV_U32 *)(pData + 2);
	}
	return MV_VLAN_HLEN;
}

static INLINE int mvNfpVlanReplace(int port, MV_U8 *pData, MV_U16 vid, MV_NFP_RESULT *pRes)
{
	MV_U16 *pVlan;

	NFP_INC(port, vlan_tx_replace);

	pVlan = (MV_U16 *)(pData + MV_ETH_MH_SIZE + MV_MAC_ADDR_SIZE + MV_MAC_ADDR_SIZE);
	/* Set VLAN Type - 0x8100 */
	*pVlan = MV_16BIT_BE(MV_VLAN_TYPE);
	pVlan++;
	/* Set VID + priority */
#ifdef NFP_CLASSIFY
	if (pRes->classifyInfo.flags & NFP_F_SET_VLAN_PRIO)
		mvNfpVpriClassifyUpdate(&vid, &(pRes->classifyInfo));
#endif /* NFP_CLASSIFY */
	*pVlan = MV_16BIT_BE(vid);

	return 0;
}

static INLINE int mvNfpVlanTxUpdate(int port, NETA_RX_DESC *pRxDesc, NFP_IF_MAP *pOutIf,
					MV_U8 *pData, MV_ETH_PKT *pPkt, MV_BOOL saveMac,
					MV_NFP_RESULT *pRes)
{
	int vlanShift = 0;

	if (NETA_RX_IS_VLAN(pRxDesc)) {
		/* Original packet was tagged */
		if (pOutIf->flags & NFP_F_MAP_VLAN_TX_UNTAGGED)
			vlanShift = mvNfpVlanRemove(port, pData, saveMac);
		else if (pOutIf->flags & NFP_F_MAP_VLAN_TX_TAGGED)
			vlanShift = mvNfpVlanReplace(port, pData, pOutIf->vlanId, pRes);
	} else {
		/* Original packet was untagged */
		if (pOutIf->flags & NFP_F_MAP_VLAN_TX_TAGGED)
			vlanShift = mvNfpVlanAdd(port, pData, pOutIf->vlanId, saveMac, pRes);
		else if (pPkt->vlanId != NFP_INVALID_VLAN) /* PVID case */
			vlanShift = mvNfpVlanAdd(port, pData, pPkt->vlanId, saveMac, pRes);
	}
	pPkt->bytes -= vlanShift;
	return vlanShift;
}
#endif /* NFP_VLAN */

#ifdef NFP_PPP
/* Find PPPoE interface with the "sid" */
static INLINE NFP_IF_MAP *mvNfpPppIfFind(NFP_IF_MAP *ifMap, MV_U16 sid)
{
	NFP_IF_MAP *virtIf;

	if ((ifMap->flags & NFP_F_MAP_PPPOE) && (ifMap->sid == sid))
		return ifMap;

	virtIf = ifMap->virtIf;
	while (virtIf != NULL) {
		if ((virtIf->flags & NFP_F_MAP_PPPOE) && (virtIf->sid == sid))
			return virtIf;

		virtIf = virtIf->virtNext;
	}
	return NULL;
}

MV_STATUS mvNfpPppAdd(int ifIndex, MV_U16 sid, MV_U8 *remoteMac)
{
	NFP_IF_MAP *pppIf = mvNfpIfMapGet(ifIndex);

	if (pppIf == NULL) {
		mvOsPrintf("%s: interface %d is not valid\n", __func__, ifIndex);
		return MV_NOT_FOUND;
	}
	/* Copy SA MAC address from parent interface */
	memcpy(pppIf->mac, pppIf->parentIf->mac, MV_MAC_ADDR_SIZE);

	memcpy(pppIf->remoteMac, remoteMac, MV_MAC_ADDR_SIZE);
	pppIf->sid = MV_16BIT_BE(sid);
	pppIf->flags |= NFP_F_MAP_PPPOE;

	return MV_OK;
}

MV_STATUS mvNfpPppDel(int ifIndex)
{
	NFP_IF_MAP *pppIf = mvNfpIfMapGet(ifIndex);

	if (pppIf == NULL) {
		mvOsPrintf("%s: interface %d is not valid\n", __func__, ifIndex);
		return MV_NOT_FOUND;
	}
	pppIf->flags &= ~NFP_F_MAP_PPPOE;
	pppIf->sid = 0;
	mvOsMemset(pppIf->remoteMac, 0, MV_MAC_ADDR_SIZE);

	return MV_OK;
}

MV_STATUS mvNfpPppRx(int port, NETA_RX_DESC *pRxDesc, MV_U8 *pData, MV_ETH_PKT *pPkt ,
					 NFP_IF_MAP **ppIfMap, MV_IP_HEADER_INFO *pIpHdrInfo)
{
	unsigned char *pIpHdr;
	PPPoE_HEADER  *pPppHdr;
	NFP_IF_MAP    *ifMap = *ppIfMap;
	NFP_IF_MAP    *ifPppMap = NULL;

	if (NETA_RX_IS_PPPOE(pRxDesc)) {
		pIpHdr = (unsigned char *)pIpHdrInfo->ip_hdr.ip4;
		pPppHdr = (PPPoE_HEADER *)(pIpHdr - MV_PPPOE_HDR_SIZE);

		ifPppMap = mvNfpPppIfFind(ifMap, pPppHdr->session);
		if (ifPppMap == NULL) {
			NFP_WARN("%s: PPPoE sid=%d not found; ifMap->name=%s\n",
				__func__, MV_16BIT_BE(pPppHdr->session), ifMap->name);
			NFP_INC(port, pppoe_rx_not_found);
			return MV_TERMINATE;
		}
		NFP_INC(port, pppoe_rx_found);
	}
	return MV_CONTINUE;
}
#endif /* NFP_PPP */


#ifdef NFP_BRIDGE
/* Do bridging: MV_OK - bridging, MV_TERMINATE - slow path, MV_CONTINUE - Routing */
static INLINE MV_STATUS  mvNfpBridgeRx(int port, NETA_RX_DESC *pRxDesc, NFP_IF_MAP *inIfMap, MV_ETH_PKT *pPkt,
				       MV_NFP_RESULT *pRes)
{
	int             shift = 0;
	MV_U8           *pData;
	MV_U8           *da, *sa;
	NFP_IF_MAP      *outIfMap;
#ifdef NFP_FDB_MODE
	NFP_RULE_FDB	*fdbRule;
#else
	NFP_RULE_BRIDGE	*bridgeRule;
#endif

	pData = pPkt->pBuf + pPkt->offset;
	da = pData + MV_ETH_MH_SIZE;
	sa = da + MV_MAC_ADDR_SIZE;

#ifdef NFP_FDB_MODE
	/* BridgeIf + SA lookup */
	fdbRule = mvNfpFdbLookup(inIfMap->bridgeIf, pData + MV_ETH_MH_SIZE + MV_MAC_ADDR_SIZE);
	if (fdbRule == NULL) {
		NFP_INC(port, fdb_sa_miss);
		return MV_TERMINATE;
	}
	/* Check if FDB entry move to other port of the bridge */
	if (fdbRule->if_index != inIfMap->ifIdx) {
		NFP_INC(port, fdb_port_miss);
		return MV_TERMINATE;
	}

	/* BridgeIf + DA lookup */
	fdbRule = mvNfpFdbLookup(inIfMap->bridgeIf, pData + MV_ETH_MH_SIZE);
	if (fdbRule == NULL) {
		NFP_INC(port, fdb_da_miss);
		return MV_TERMINATE;
	}
	fdbRule->age++;

	if (fdbRule->status == NFP_BRIDGE_LOCAL) {
		NFP_INC(port, fdb_local);
		return MV_CONTINUE;
	}
	outIfMap = mvNfpIfMapGet(fdbRule->if_index);
	if (outIfMap == NULL) {
		mvOsPrintf("%s: bridge rule interface %d is not valid\n", __func__, fdbRule->if_index);
		NFP_INC(port, oif_err);
		return MV_TERMINATE;
	}
	NFP_INC(port, fdb_hit);
#else
	bridgeRule = mvNfpBridgeLookup(da, sa, inIfMap->ifIdx);
	if (bridgeRule == NULL) {
		NFP_INC(port, bridge_miss);
		return MV_TERMINATE;
	}
	bridgeRule->age++;

#ifdef NFP_CLASSIFY
	if (bridgeRule->flags & NFP_F_BR_SET_VLAN_PRIO)
		mvNfpVpriBridgeClassifySave(pData, bridgeRule, &(pRes->classifyInfo));
	if (bridgeRule->flags & NFP_F_BR_SET_TXQ)
		mvNfpTxqClassifySave(bridgeRule->txq, &(pRes->classifyInfo));
	if (bridgeRule->flags & NFP_F_BR_SET_TXP)
		mvNfpTxpClassifySave(bridgeRule->txp, &(pRes->classifyInfo));
	if (bridgeRule->flags & NFP_F_BR_SET_MH)
		mvNfpMhClassifySave(bridgeRule->mh, &(pRes->classifyInfo));
#endif /* NFP_CLASSIFY */

	/* Check if this is a local bridge rule (DA is "to me") */
	if (inIfMap->bridgeIf == bridgeRule->oif) {
		NFP_INC(port, bridge_local);
		return MV_CONTINUE;
	}
	outIfMap = mvNfpIfMapGet(bridgeRule->oif);
	if (outIfMap == NULL) {
		mvOsPrintf("%s: bridge rule out interface %d is not valid\n", __func__, bridgeRule->oif);
		NFP_INC(port, oif_err);
		return MV_TERMINATE;
	}
	NFP_INC(port, bridge_hit);
#endif /* NFP_FDB_MODE */

#ifdef NFP_VLAN
	/* Process VLAN tag */
	shift += mvNfpVlanTxUpdate(port, pRxDesc, outIfMap, pData + shift, pPkt, MV_TRUE, pRes);
#endif /* NFP_VLAN */

	while (outIfMap->parentIf)
		outIfMap = outIfMap->parentIf;

	/* Update pPkt for bridging TX */
	pRes->dev = outIfMap->dev;
	if (outIfMap->flags & NFP_F_MAP_EXT)
		pRes->flags |= MV_NFP_RES_NETDEV_EXT;

	/* Process 2B of MH */
	if (outIfMap->flags & NFP_F_MAP_TX_MH) {
		/* Transmit with MH */
		*(MV_U16 *)(pData + shift) = outIfMap->txMh;
#ifdef NFP_CLASSIFY
		/* allow setting MH only for PON ports */
		if (MV_PON_PORT(outIfMap->port) && (pRes->classifyInfo.flags & NFP_F_SET_MH))
			mvNfpMhClassifyUpdate(pData + shift, &(pRes->classifyInfo));
#endif /* NFP_CLASSIFY */
	} else {
		/* Transmit without MH */
		shift += MV_ETH_MH_SIZE;
		pPkt->bytes -= MV_ETH_MH_SIZE;
	}

	pRes->shift = shift;

#ifdef NFP_CLASSIFY
	if (pRes->classifyInfo.flags & NFP_F_SET_TXQ) {
		pRes->flags |= MV_NFP_RES_TXQ_VALID;
		pRes->txq = pRes->classifyInfo.txq;
	}
	/* allow setting txp only for PON ports */
	if (MV_PON_PORT(outIfMap->port) && (pRes->classifyInfo.flags & NFP_F_SET_TXP)) {
		pRes->flags |= MV_NFP_RES_TXP_VALID;
		pRes->txp = pRes->classifyInfo.txp;
	}
#endif /* NFP_CLASSIFY */

	pRes->tx_cmd = NETA_TX_L4_CSUM_NOT;

	if ((pRes->flags & MV_NFP_RES_NETDEV_EXT) == 0)
		mvOsCacheLineFlush(NULL, pData);

	return MV_OK;
}
#endif /* NFP_BRIDGE */

#ifdef NFP_NAT
static INLINE void mvNfpNatUpdate(MV_IP_HEADER_INFO *pIpInfo, NFP_RULE_CT *pCt, MV_NFP_RESULT *pRes, NFP_IF_MAP *pOutIf)
{
	MV_IP_HEADER *pIpHdr = pIpInfo->ip_hdr.ip4;
	char          *l4Hdr = ((char *)pIpHdr + pIpInfo->ipHdrLen);
	MV_U16        *pPort;
	MV_U32        old_val = 0, new_val = 0;
	int           csum;

	if ((pIpHdr->protocol == MV_IP_PROTO_UDP) && (((MV_UDP_HEADER *)l4Hdr)->check == 0)) {
		/* skip L4 csum */
		csum = 0;
	} else if ((pOutIf->flags & NFP_F_MAP_EXT) ||
			needFragment(pIpInfo, pOutIf) ||
			(pIpInfo->ipLen > MV_ETH_TX_CSUM_MAX_SIZE)) {
		/* L4 checksum must be calculated by SW */
		csum = 1;
	} else {
		/* L4 checksum can be calculated by HW */
		csum = 2;
	}

	if (pCt->flags & NFP_F_CT_SNAT) {
		pPort = (MV_U16 *)l4Hdr;

		if (csum == 1) {
			old_val = pIpHdr->srcIP + *pPort;
			new_val = pCt->new_sip + pCt->new_sport;
		}
		pIpHdr->srcIP = pCt->new_sip;
		*pPort = pCt->new_sport;
	}

	if (pCt->flags & NFP_F_CT_DNAT) {
		pPort = (MV_U16 *)(l4Hdr + 2);

		if (csum == 1) {
			old_val += pIpHdr->dstIP + *pPort;
			new_val += pCt->new_dip + pCt->new_dport;
		}
		pIpHdr->dstIP = pCt->new_dip;
		*pPort = pCt->new_dport;
	}

	switch (csum) {
	case 0:
		pRes->pWrite = l4Hdr + sizeof(MV_UDP_HEADER);
		break;

	case 1:
		pRes->diffL4[0] = ~old_val;
		pRes->diffL4[1] = new_val;

		pRes->flags |= MV_NFP_RES_L4_CSUM_NEEDED;
		break;

	case 2:
		pRes->tx_cmd &= ~NETA_TX_L4_CSUM_MASK;
		if (pIpHdr->protocol == MV_IP_PROTO_TCP) {
			pRes->pWrite = l4Hdr + 4;
			pRes->tx_cmd |= NETA_TX_L4_TCP | NETA_TX_L4_CSUM_FULL;
		} else if (pIpHdr->protocol == MV_IP_PROTO_UDP) {
			pRes->pWrite = l4Hdr + sizeof(MV_UDP_HEADER);
			pRes->tx_cmd |= NETA_TX_L4_UDP | NETA_TX_L4_CSUM_FULL;
		}
		break;
	}
}
#endif /* NFP_NAT */

#ifdef NFP_PPP
static INLINE int removePppoeHeader(MV_IP_HEADER_INFO *pIpHdrInfo)
{
	unsigned char *pIpHdr;

	/* writing IP ethertype to the new location of ether header */
	if (pIpHdrInfo->family == MV_INET) {
		pIpHdr = (unsigned char *)(pIpHdrInfo->ip_hdr.ip4);
		*(pIpHdr - 1) = 0x00;
		*(pIpHdr - 2) = 0x08;
	} else {
		pIpHdr = (unsigned char *)(pIpHdrInfo->ip_hdr.ip6);
	  *(pIpHdr - 1) = 0xDD;
	  *(pIpHdr - 2) = 0x86;
	}
	return MV_PPPOE_HDR_SIZE;
}

static INLINE int addPppoeHeader(NFP_IF_MAP *pOutIf, MV_IP_HEADER_INFO *pIpHdrInfo)
{
	unsigned char *pIpHdr;
	PPPoE_HEADER *pPPPNew;

	if (pIpHdrInfo->family == MV_INET)
		pIpHdr = (unsigned char *)pIpHdrInfo->ip_hdr.ip4;
	else
		pIpHdr = (unsigned char *)pIpHdrInfo->ip_hdr.ip6;

	/* Keep VLAN fields*/
	*(pIpHdr - MV_PPPOE_HDR_SIZE - 3) = *(pIpHdr - 3);
	*(pIpHdr - MV_PPPOE_HDR_SIZE - 4) = *(pIpHdr - 4);
	*(pIpHdr - MV_PPPOE_HDR_SIZE - 5) = *(pIpHdr - 5);
	*(pIpHdr - MV_PPPOE_HDR_SIZE - 6) = *(pIpHdr - 6);

	*(pIpHdr - MV_PPPOE_HDR_SIZE - 1) = 0x64;
	*(pIpHdr - MV_PPPOE_HDR_SIZE - 2) = 0x88;

	pPPPNew = (PPPoE_HEADER *)(pIpHdr - MV_PPPOE_HDR_SIZE);
	pPPPNew->version = 0x11;
	pPPPNew->code = 0x0;
	pPPPNew->session = pOutIf->sid;

	/* calculate PPPoE payload len considering padding for short packets */
	if (pIpHdrInfo->family == MV_INET) {
		pPPPNew->proto =  MV_16BIT_BE(MV_IP_PPP);
		pPPPNew->len = MV_16BIT_BE(pIpHdrInfo->ipLen + MV_PPP_HDR_SIZE);
	} else {
		pPPPNew->proto = MV_16BIT_BE(MV_IP6_PPP);
		pPPPNew->len = MV_16BIT_BE(pIpHdrInfo->ipLen + MV_PPP_HDR_SIZE + pIpHdrInfo->ipHdrLen);
	}
	return -MV_PPPOE_HDR_SIZE;
}


static INLINE void replacePppoeHeader(NFP_IF_MAP *pOutIf, MV_IP_HEADER_INFO *pIpHdrInfo)
{
	PPPoE_HEADER *pPPPHdr;
	unsigned char *pIpHdr;

	pIpHdr = (unsigned char *)pIpHdrInfo->ip_hdr.ip4;
	pPPPHdr = (PPPoE_HEADER *)(pIpHdr - MV_PPPOE_HDR_SIZE);
	if (pPPPHdr)
		pPPPHdr->session = pOutIf->sid;
}


static INLINE int mvNfpPppTxUpdate(MV_U32 port, NETA_RX_DESC *pRxDesc,
					NFP_IF_MAP *pOutIf, MV_ETH_PKT *pPkt,
					MV_IP_HEADER_INFO *pIpHdrInfo)
{
	int pppShift = 0;

	if (NETA_RX_IS_PPPOE(pRxDesc)) {
		/* Ingress packet has PPPoE header */
		if (pOutIf->flags & NFP_F_MAP_PPPOE) {
			replacePppoeHeader(pOutIf, pIpHdrInfo);
			NFP_INC(port, pppoe_tx_replace);
		} else {
			/* remove pppoe header */
			pppShift = removePppoeHeader(pIpHdrInfo);
			pPkt->bytes -= pppShift;
			NFP_INC(port, pppoe_tx_remove);
		}
	} else {
		/* Ingress packet doesn't have PPPoE header */
		if (pOutIf->flags & NFP_F_MAP_PPPOE) {
			/* add pppoe header */
			pppShift = addPppoeHeader(pOutIf, pIpHdrInfo);
			pPkt->bytes -= pppShift;
			NFP_INC(port, pppoe_tx_add);
		}
	}
	return pppShift;
}
#endif /* NFP_PPP */


static INLINE MV_STATUS mvNfpStatusCheck(MV_U32 port, const NETA_RX_DESC *pRxDesc)
{
#ifdef CONFIG_MV_ETH_PNC
	if ((pRxDesc->status & NETA_RX_L3_MASK) == NETA_RX_L3_IP4_ERR) {
		NFP_INC(port, ipv4_csum_err);
		return MV_TERMINATE;
	} else if ((pRxDesc->status & NETA_RX_L3_MASK) == NETA_RX_L3_UN) {
		NFP_INC(port, non_ip);
		return MV_TERMINATE;
	}
	if ((((pRxDesc->status & NETA_RX_L4_MASK) == NETA_RX_L4_TCP) ||
	     ((pRxDesc->status & NETA_RX_L4_MASK) == NETA_RX_L4_UDP)) &&
	    ((pRxDesc->status & NETA_RX_L4_CSUM_OK_MASK) == 0)) {
		NFP_INC(port, l4_csum_err);
		return MV_TERMINATE;
	}
#else
	if (((pRxDesc->status & ETH_RX_NOT_LLC_SNAP_FORMAT_MASK) == 0) ||
	    ((pRxDesc->status & ETH_RX_IP_HEADER_OK_MASK) == 0)) {
		NFP_INC(port, non_ip);
		return MV_TERMINATE;
	}

	if ((((pRxDesc->status & ETH_RX_L4_TYPE_MASK) == ETH_RX_L4_TCP_TYPE) ||
	     ((pRxDesc->status & ETH_RX_L4_TYPE_MASK) == ETH_RX_L4_UDP_TYPE)) &&
	    ((pRxDesc->status & NETA_RX_L4_CSUM_OK_MASK) == 0)) {
		NFP_INC(port, l4_csum_err);
		return MV_TERMINATE;
	}
#endif /* CONFIG_MV_ETH_PNC */

	if (nfpMode == MV_NFP_5_TUPLE) {
		/* Only UDP or TCP packets with correct checksum are processed in 5 tuple mode */
		if (!NETA_RX_L4_CSUM_IS_OK(pRxDesc->status)) {
			NFP_INC(port, l4_unknown);
			return MV_TERMINATE;
		}
	}
	return MV_CONTINUE;
}

static INLINE MV_STATUS mvNfpParseIpHeader(MV_U32 port, const NETA_RX_DESC *pRxDesc, MV_U8 *pData,
							MV_IP_HEADER_INFO *pIpHdr)
{
	MV_U8 *pEth;

	pEth = pData + MV_ETH_MH_SIZE;
	if (pEth[0] & 0x01) {	/* Check multicast and broadcast */
		NFP_INC(port, mac_mcast);
		return MV_TERMINATE;
	}

	pIpHdr->ipOffset = NETA_RX_GET_IPHDR_OFFSET(pRxDesc);
	pIpHdr->ipHdrLen =  NETA_RX_GET_IPHDR_HDRLEN(pRxDesc) << 2;

	if (NETA_RX_L3_IS_IP6(pRxDesc->status)) {
		pIpHdr->family = MV_INET6;
		pIpHdr->ip_hdr.ip6 = (MV_IP6_HEADER *) (pData + pIpHdr->ipOffset);
		if ((pIpHdr->ip_hdr.ip6)->hoplimit <= 1) {
			NFP_INC(port, ttl_exp);
			return MV_TERMINATE;
		}
		pIpHdr->ipLen = MV_16BIT_BE(pIpHdr->ip_hdr.ip6->payloadLength);
		pIpHdr->ipProto = pIpHdr->ip_hdr.ip6->protocol;

		NFP_INC(port, ipv6);
	} else {
		pIpHdr->family = MV_INET;
		pIpHdr->ip_hdr.ip4 = (MV_IP_HEADER *) (pData + pIpHdr->ipOffset);
		pIpHdr->ipLen = MV_16BIT_BE(pIpHdr->ip_hdr.ip4->totalLength);
		pIpHdr->ipProto = pIpHdr->ip_hdr.ip4->protocol;

#ifdef NFP_CT
		if (nfpMode == MV_NFP_5_TUPLE) {
			if (NETA_RX_IP_IS_FRAG(pRxDesc->status)) {
				NFP_INC(port, ipv4_rx_frag);
				return MV_TERMINATE;
			}
		}
#endif /* NFP_CT */

		if ((pIpHdr->ip_hdr.ip4)->ttl <= 1) {
			NFP_INC(port, ttl_exp);
			return MV_TERMINATE;
		}
		NFP_INC(port, ipv4);
	}
	return MV_CONTINUE;
}

static INLINE MV_STATUS mvNfpFragmentCheck(int port, MV_IP_HEADER_INFO *pIpInfo, NFP_IF_MAP *pOutIf)
{
	if (needFragment(pIpInfo, pOutIf)) {
		if ((pIpInfo->family == MV_INET) &&
			(pIpInfo->ip_hdr.ip4->fragmentCtrl & MV_16BIT_BE(MV_IP4_DF_FLAG_MASK))) {
			NFP_INC(port, ip_tx_frag_err);
			return MV_TERMINATE;
		}
		NFP_INC(port, ip_tx_frag);
	}
	return MV_CONTINUE;
}

#ifdef NFP_CLASSIFY
MV_VOID mvNfpDscpPolicySet(MV_NFP_CLASSIFY_POLICY policy)
{
	if ((policy < MV_NFP_CLASSIFY_HIGHEST) || (policy > MV_NFP_CLASSIFY_LAST))
		mvOsPrintf("%s: Illegal policy value %d\n", __func__, policy);

	dscpPolicy = policy;
}

MV_VOID mvNfpVlanPrioPolicySet(MV_NFP_CLASSIFY_POLICY policy)
{
	if ((policy < MV_NFP_CLASSIFY_HIGHEST) || (policy > MV_NFP_CLASSIFY_LAST))
		mvOsPrintf("%s: Illegal policy value %d\n", __func__, policy);

	vlanPrioPolicy = policy;
}

MV_VOID mvNfpTxqPolicySet(MV_NFP_CLASSIFY_POLICY policy)
{
	if ((policy < MV_NFP_CLASSIFY_HIGHEST) || (policy > MV_NFP_CLASSIFY_LAST))
		mvOsPrintf("%s: Illegal policy value %d\n", __func__, policy);

	txqPolicy = policy;
}

MV_VOID mvNfpTxpPolicySet(MV_NFP_CLASSIFY_POLICY policy)
{
	if ((policy < MV_NFP_CLASSIFY_HIGHEST) || (policy > MV_NFP_CLASSIFY_LAST))
		mvOsPrintf("%s: Illegal policy value %d\n", __func__, policy);

	txpPolicy = policy;
}

MV_VOID mvNfpMhPolicySet(MV_NFP_CLASSIFY_POLICY policy)
{
	if ((policy < MV_NFP_CLASSIFY_HIGHEST) || (policy > MV_NFP_CLASSIFY_LAST))
		mvOsPrintf("%s: Illegal policy value %d\n", __func__, policy);

	mhPolicy = policy;
}

MV_NFP_CLASSIFY_POLICY mvNfpDscpPolicyGet(MV_VOID)
{
	return dscpPolicy;
}

MV_NFP_CLASSIFY_POLICY mvNfpVlanPrioPolicyGet(MV_VOID)
{
	return vlanPrioPolicy;
}

MV_NFP_CLASSIFY_POLICY mvNfpTxqPolicyGet(MV_VOID)
{
	return txqPolicy;
}

MV_NFP_CLASSIFY_POLICY mvNfpTxpPolicyGet(MV_VOID)
{
	return txpPolicy;
}

MV_NFP_CLASSIFY_POLICY mvNfpMhPolicyGet(MV_VOID)
{
	return mhPolicy;
}
#endif /* NFP_CLASSIFY */


static INLINE MV_STATUS mvNfpTwoTupleProcess(MV_U32 port, MV_IP_HEADER_INFO *pIpHdr,
					     MV_ETH_PKT *pPkt, NFP_RULE_FIB **pFib)
{
	MV_U8		*srcL3, *dstL3;

	if (pIpHdr->family == MV_INET) {
		/* Remove padding */
		pPkt->bytes = (pIpHdr->ipLen + pIpHdr->ipOffset);

		dstL3 = (MV_U8 *)&(pIpHdr->ip_hdr.ip4->dstIP);
		srcL3 = (MV_U8 *)&(pIpHdr->ip_hdr.ip4->srcIP);
	} else {
		dstL3 = pIpHdr->ip_hdr.ip6->dstAddr;
		srcL3 = pIpHdr->ip_hdr.ip6->srcAddr;
	}

	*pFib = mvNfpFibLookup(pIpHdr->family, srcL3, dstL3);

	if (!(*pFib)) {
		NFP_WARN("%s failed: ", __func__);
		mvNfp2TupleInfoPrint(NFP_WARN_PRINT, pIpHdr->family, srcL3, dstL3);

		NFP_INC(port, fib_miss);
		return MV_TERMINATE;
	}
	(*pFib)->age++;

	NFP_INC(port, fib_hit);

	return MV_CONTINUE;
}

#ifdef NFP_CT

#ifdef NFP_CT_LEARN
/* Check for FIN/RST flags in TCP header */
static INLINE MV_U8 mvNfpTcpClose(MV_IP_HEADER_INFO *pIpHdr)
{
	MV_TCP_HEADER	*pTcpHdr;

	if (pIpHdr->ipProto != MV_IP_PROTO_TCP)
		return 0;

	pTcpHdr = (MV_TCP_HEADER *)((unsigned)(pIpHdr->ip_hdr.l3) + pIpHdr->ipHdrLen);

	return (MV_16BIT_BE(pTcpHdr->flags) & (MV_TCP_FLAG_RST | MV_TCP_FLAG_FIN));
}
#endif /* NFP_CT_LEARN */

static INLINE MV_STATUS mvNfpFiveTupleProcess(MV_U32 port, MV_IP_HEADER_INFO *pIpHdr,
					      MV_ETH_PKT *pPkt, NFP_RULE_CT **ppCt,
					      MV_NFP_RESULT *pRes)
{
	NFP_RULE_CT	*pCt = NULL;
	MV_U8		*srcL3, *dstL3;
	MV_U16 		proto = 0;
	MV_U32		ports = 0;

	if (pIpHdr->family == MV_INET) {
		MV_IP_HEADER *pIph = pIpHdr->ip_hdr.ip4;

		/* Remove padding */
		pPkt->bytes = (pIpHdr->ipLen + pIpHdr->ipOffset);

		dstL3 = (MV_U8 *)&(pIph->dstIP);
		srcL3 = (MV_U8 *)&(pIph->srcIP);
		proto = pIph->protocol;
		ports = *(MV_U32 *)((char *)pIph + pIpHdr->ipHdrLen);
	} else {
		MV_IP6_HEADER *pIp6Hdr = pIpHdr->ip_hdr.ip6;

		srcL3 = pIp6Hdr->srcAddr;
		dstL3 = pIp6Hdr->dstAddr;
		proto = pIp6Hdr->protocol;
		ports = *(MV_U32 *)((char *)pIp6Hdr + pIpHdr->ipHdrLen);
	}
	pCt = mvNfpCtLookupByTuple(pIpHdr->family, srcL3, dstL3, ports, proto);

	if (!pCt) {
		NFP_WARN("%s failed: ", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, pIpHdr->family, srcL3, dstL3,
					MV_16BIT_BE(ports & 0xFFFF), MV_16BIT_BE((ports >> 16) & 0xFFFF), proto);

		NFP_INC(port, ct_miss);
		return MV_TERMINATE;
	}
	pCt->age++;
	pCt->hit_cntr++;
	NFP_INC(port, ct_hit);

#ifdef NFP_CT_LEARN
	if (mvNfpTcpClose(pIpHdr)) {
		NFP_INC(port, ct_tcp_fin_rst);
		return MV_TERMINATE;
	}
#endif /* NFP_CT_LEARN */

	if (pCt->flags & NFP_F_CT_DROP)
		return MV_DROPPED;

	*ppCt = pCt;

#ifdef NFP_CLASSIFY
	if (pCt->flags & NFP_F_CT_SET_DSCP)
		mvNfpDscpClassifySave(pIpHdr, pCt, &(pRes->classifyInfo));
	if (pCt->flags & NFP_F_CT_SET_VLAN_PRIO)
		mvNfpVpriCtClassifySave(pCt, &(pRes->classifyInfo));
	if (pCt->flags & NFP_F_CT_SET_TXQ)
		mvNfpTxqClassifySave(pCt->txq, &(pRes->classifyInfo));
	if (pCt->flags & NFP_F_CT_SET_TXP)
		mvNfpTxpClassifySave(pCt->txp, &(pRes->classifyInfo));
	if (pCt->flags & NFP_F_CT_SET_MH)
		mvNfpMhClassifySave(pCt->mh, &(pRes->classifyInfo));
#endif /* NFP_CLASSIFY */
#ifdef NFP_LIMIT
	if (pCt->tbfInfo)
		return mvNfpTbfProcess(pCt->tbfInfo, pPkt->bytes);
#endif /* NFP_LIMIT */
	return MV_CONTINUE;
}
#endif /* NFP_CT */

 /* NFP Process */
MV_STATUS mvNfpRx(MV_U32 port, NETA_RX_DESC *pRxDesc, MV_ETH_PKT *pPkt, MV_NFP_RESULT *pRes)
{
	NFP_RULE_FIB      *pFib;
	MV_U8             *pData;
	MV_IP_HEADER_INFO *ipHdrInfo = &pRes->ipInfo;
	int               shift = 0;
	MV_STATUS         status;
	NFP_IF_MAP        *pInIf, *pOutIf;
#ifdef NFP_CT
	NFP_RULE_CT       *pCt = NULL;
#endif /* NFP_CT */

	NFP_INC(port, rx);
	pData = pPkt->pBuf + pPkt->offset;
	pRes->flags = 0;
	pRes->shift = 0;
	pRes->tx_cmd = NETA_TX_L4_CSUM_NOT;

#ifdef NFP_CLASSIFY
	mvNfpClassifyInit(&(pRes->classifyInfo));
#endif /* NFP_CLASSIFY */

	/* Lookup incoming interface (port + switchGroup) */
	pInIf = mvNfpIfMapRealGet(port, pData);
	if (pInIf == NULL) {
		NFP_INC(port, iif_err);
		return MV_TERMINATE;
	}

#ifdef NFP_VLAN
	status = mvNfpVlanRx(port, pRxDesc, pData, pPkt, &pInIf, pRes);
	if (status != MV_CONTINUE)
		return status;
#endif /* NFP_VLAN */

#ifdef NFP_BRIDGE
	if (pInIf->flags & NFP_F_MAP_BRIDGE_PORT) {
		/* Do bridging: OK - bridging, TERMINATE - slow path, CONTINUE - Routing */
		status = mvNfpBridgeRx(port, pRxDesc, pInIf, pPkt, pRes);
		if (status != MV_CONTINUE)
			return status;
	}
#endif /* NFP_BRIDGE */

	status = mvNfpStatusCheck(port, pRxDesc);
	if (status != MV_CONTINUE)
		return status;

	status = mvNfpParseIpHeader(port, pRxDesc, pData, ipHdrInfo);
	if (status != MV_CONTINUE)
		return status;

	pRes->flags |= MV_NFP_RES_IP_INFO_VALID;

#ifdef NFP_PPP
		status = mvNfpPppRx(port, pRxDesc, pData, pPkt, &pInIf,  ipHdrInfo);
		if (status != MV_CONTINUE)
			return status;
#endif /* NFP_PPP */

	if (nfpMode == MV_NFP_2_TUPLE) {
		status = mvNfpTwoTupleProcess(port, ipHdrInfo, pPkt, &pFib);
		if (status != MV_CONTINUE)
			return status;
	} else {
#ifdef NFP_CT
		status = mvNfpFiveTupleProcess(port, ipHdrInfo, pPkt, &pCt, pRes);
		if (status != MV_CONTINUE)
			return status;

		pFib = pCt->fib;
		pFib->age++;
#else
		return MV_TERMINATE;
#endif /* NFP_CT */
	}

	/* At this point pFib is valid */
	pOutIf = mvNfpIfMapGet(pFib->oif);
	if (pOutIf == NULL) {
		mvOsPrintf("%s: fib out interface %d is not valid\n", __func__, pFib->oif);
		NFP_INC(port, oif_err);
		return MV_TERMINATE;
	}

	/* Check if fragmentation needed but don't fragment bit is set */
	status = mvNfpFragmentCheck(port, ipHdrInfo, pOutIf);
	if (status != MV_CONTINUE)
		return status;

	pRes->mtu = pOutIf->mtu;
	pRes->pWrite = ((MV_U8 *)ipHdrInfo->ip_hdr.l3) + ipHdrInfo->ipHdrLen;

#ifdef NFP_NAT
	if (pCt && (pCt->flags & (NFP_F_CT_SNAT | NFP_F_CT_DNAT)))
		mvNfpNatUpdate(ipHdrInfo, pCt, pRes, pOutIf);
#endif /* NFP_NAT */

	mvNfpFibIpUpdate(ipHdrInfo);

#ifdef NFP_PPP
	shift = mvNfpPppTxUpdate(port, pRxDesc, pOutIf, pPkt, ipHdrInfo);
	if (pOutIf->flags & NFP_F_MAP_PPPOE)
		pOutIf = pOutIf->parentIf;
#endif /* NFP_PPP */

#ifdef NFP_VLAN
	shift += mvNfpVlanTxUpdate(port, pRxDesc, pOutIf, pData + shift, pPkt, MV_FALSE, pRes);
#endif /* NFP_VLAN */

	mvNfpFibMacUpdate(pData + shift, pFib);

	while (pOutIf->parentIf)
		pOutIf = pOutIf->parentIf;

	pRes->dev = pOutIf->dev;
	if (pOutIf->flags & NFP_F_MAP_EXT)
		pRes->flags |= MV_NFP_RES_NETDEV_EXT;

#ifdef NFP_CLASSIFY
	/* Update packet according to classification results */

#ifdef NFP_CT
	if (pCt) {
		if (pCt->flags & NFP_F_CT_SET_DSCP)
			mvNfpDscpClassifyUpdate(ipHdrInfo, &(pRes->classifyInfo));
	}
#endif /* NFP_CT */

	/* allow setting MH only for PON ports */
	if (MV_PON_PORT(pOutIf->port) && (pRes->classifyInfo.flags & NFP_F_SET_MH))
		mvNfpMhClassifyUpdate(pData + shift, &(pRes->classifyInfo));

	if (pRes->classifyInfo.flags & NFP_F_SET_TXQ) {
		pRes->flags |= MV_NFP_RES_TXQ_VALID;
		pRes->txq = pRes->classifyInfo.txq;
	}
	/* allow setting txp only for PON ports */
	if (MV_PON_PORT(pOutIf->port) && (pRes->classifyInfo.flags & NFP_F_SET_TXP)) {
		pRes->flags |= MV_NFP_RES_TXP_VALID;
		pRes->txp = pRes->classifyInfo.txp;
	}

#endif /* NFP_CLASSIFY */

	/* Process 2B of MH */
	if (!(pOutIf->flags & NFP_F_MAP_TX_MH)) {
		shift += MV_ETH_MH_SIZE;
		pPkt->bytes -= MV_ETH_MH_SIZE;
	}
	pRes->shift = shift;

	if ((pRes->flags & (MV_NFP_RES_L4_CSUM_NEEDED | MV_NFP_RES_NETDEV_EXT)) == 0) {

		/* Flush maximum accessed data before TX */
		if (shift < 0)
			pData += shift;

		mvOsCacheMultiLineFlushInv(NULL, pData, (pRes->pWrite - pData));
	}
	return MV_OK;
}

MV_STATUS mvNfpIfMapCreate(NFP_IF_MAP *ifMap2)
{
	MV_U32      flags;
	NFP_IF_MAP  *newMap, *ifMap;

	newMap = mvOsMalloc(sizeof(NFP_IF_MAP));
	if (newMap == NULL) {
		mvOsPrintf("%s: can't allocate NFP_IF_MAP\n", __func__);
		return MV_NO_RESOURCE;
	}
	memcpy(newMap, ifMap2, sizeof(NFP_IF_MAP));
	flags = newMap->flags;

	if (!(flags & NFP_F_MAP_INT))
		newMap->port = NFP_INVALID_PORT;

	if (flags & NFP_F_MAP_EXT) {
		if (nfpFreeExtPort >= NFP_MAX_PORTS) {
			mvOsPrintf("%s: No free place for external interface. nfpFreeExtPort=%d\n",
				__func__, nfpFreeExtPort);
			mvOsFree(newMap);
			return MV_BUSY;
		}
		newMap->port = nfpFreeExtPort;
		nfpFreeExtPort++;
	}

	if (flags & NFP_F_MAP_SWITCH_PORT) {
		/* Set MH flag for port */
		nfp_ports[newMap->port] |= NFP_F_PORT_MH;
	} else
		newMap->switchGroup = NFP_INVALID_SWITCH_GROUP;

	if (!(flags & NFP_F_MAP_TX_MH))
		newMap->txMh = 0;

	newMap->vlanId = NFP_INVALID_VLAN;

	/* Check if such entry already exist */
	ifMap = nfp_if_map[newMap->ifIdx & NFP_DEV_HASH_MASK];
	while (ifMap) {
		if (ifMap->ifIdx == newMap->ifIdx) {
			mvOsPrintf("%s: ifMap for ifIdx=%d already exist\n", __func__, newMap->ifIdx);
			mvOsFree(newMap);
			return MV_BUSY;
		}
		if (mvNfpIfMapCmp(newMap->port, newMap->switchGroup, ifMap)) {
			mvOsPrintf("%s: ifMap with port=%d, switchGroup=%d already exist\n",
					__func__, newMap->port, newMap->switchGroup);
			mvOsFree(newMap);
			return MV_BUSY;
		}
		ifMap = ifMap->nextMap;
	}
	/* Add to nfp_if_map */
	newMap->nextMap = nfp_if_map[newMap->ifIdx & NFP_DEV_HASH_MASK];
	nfp_if_map[newMap->ifIdx & NFP_DEV_HASH_MASK] = newMap;

	/* Add to nfp_if_real_map */
	if (newMap->port != NFP_INVALID_PORT) {
		if (newMap->switchGroup == NFP_INVALID_SWITCH_GROUP)
			nfp_if_real_map[newMap->port][0] = newMap;
		else
			nfp_if_real_map[newMap->port][newMap->switchGroup] = newMap;
	}
	return MV_OK;
}

MV_STATUS mvNfpIfVirtMap(int ifIdx, int virtIf)
{
	NFP_IF_MAP *ifMap = mvNfpIfMapGet(ifIdx);
	NFP_IF_MAP *ifVirt = mvNfpIfMapGet(virtIf);
	NFP_IF_MAP *ifTemp;

	/* parentIf and virtIf must be created */
	if ((ifMap == NULL) || (ifVirt == NULL)) {
		mvOsPrintf("%s: interface not valid - parent=%d (%p), virt=%d (%p)\n",
				__func__, ifIdx, ifMap, virtIf, ifVirt);
		return MV_NOT_FOUND;
	}
	/* Check validity - TBD */

	/* Set MH as in parent interface */
	ifVirt->txMh = ifMap->txMh;
	/* Set external flag if parent is external */
	if (ifMap->flags & NFP_F_MAP_EXT)
		ifVirt->flags |= NFP_F_MAP_EXT;

	/* Bind */
	ifTemp = ifMap->virtIf;
	ifMap->virtIf = ifVirt;
	ifVirt->virtNext = ifTemp;

	/* Remember parent interface */
	ifVirt->parentIf = ifMap;

	return MV_OK;
}

MV_STATUS mvNfpIfVirtUnmap(int virtIf)
{
	NFP_IF_MAP *ifVirt = mvNfpIfMapGet(virtIf);
	NFP_IF_MAP *ifUp, *ifTemp;

	if (ifVirt == NULL) {
		mvOsPrintf("%s: virtual interface is not valid - virt=%d\n",
				__func__, virtIf);
		return MV_NOT_FOUND;
	}

	if (ifVirt->virtIf != NULL) {
		mvOsPrintf("%s: Can't unmap (%s), virtual interface %s was created\n",
			__func__, ifVirt->name, ifVirt->virtIf->name);
		return MV_NOT_FOUND;
	}

	ifUp = ifVirt->parentIf;

	if (ifUp == NULL) {
		mvOsPrintf("%s: Can't unmap (%s), parentIf is NULL\n", __func__, ifVirt->name);
		return MV_NOT_FOUND;
	}

	ifTemp = ifUp->virtIf;
	if (ifTemp == ifVirt) {
		ifUp->virtIf = ifVirt->virtNext;
		/* FIXME: PPPoE over VLAN */
		return MV_OK;
	}

	while (ifTemp->virtNext != NULL) {
		if (ifTemp->virtNext == ifVirt) {
			ifTemp->virtNext = ifVirt->virtNext;
			/* FIXME: PPPoE over VLAN */
			return MV_OK;
		}
		ifTemp = ifTemp->virtNext;
	}

	mvOsPrintf("%s: virtual interface %d is not mapped to interface %d\n",
				__func__, virtIf, ifUp->ifIdx);

	return MV_NOT_FOUND;
}

MV_STATUS mvNfpIfFlagsSet(int ifIdx, MV_U32 flags)
{
	NFP_IF_MAP *ifMap = mvNfpIfMapGet(ifIdx);

	if (ifMap == NULL) {
		mvOsPrintf("%s: interface %d is not valid\n", __func__, ifIdx);
		return MV_NOT_FOUND;
	}
	ifMap->flags |= flags;
	return MV_OK;
}

MV_STATUS mvNfpIfFlagsClear(int ifIdx, MV_U32 flags)
{
	NFP_IF_MAP *ifMap = mvNfpIfMapGet(ifIdx);

	if (ifMap == NULL) {
		mvOsPrintf("%s: interface %d is not valid\n", __func__, ifIdx);
		return MV_NOT_FOUND;
	}
	ifMap->flags &= ~flags;
	return MV_OK;
}

MV_STATUS mvNfpIfMapMacUpdate(int ifIdx, const MV_U8 *mac)
{
	NFP_IF_MAP *map = mvNfpIfMapGet(ifIdx);

	if (map != NULL) {
		mvOsMemcpy(map->mac, mac, MV_MAC_ADDR_SIZE);
		return MV_OK;
	}
	return MV_NOT_FOUND;
}

MV_STATUS mvNfpIfMapMtuUpdate(int ifIdx, int mtu)
{
	NFP_IF_MAP *map = mvNfpIfMapGet(ifIdx);

	if (map != NULL) {
		map->mtu = mtu;
		return MV_OK;
	}
	return MV_NOT_FOUND;
}

MV_STATUS mvNfpIfMapPortGet(int ifIdx, int *port)
{
	NFP_IF_MAP *ifMap = mvNfpIfMapGet(ifIdx);

	if (ifMap && (ifMap->port != NFP_INVALID_PORT)) {
		*port = ifMap->port;
		return MV_OK;
	}
	return MV_NOT_FOUND;
}

MV_STATUS mvNfpIfMapDelete(int ifIdx)
{
	MV_U32 hash = 0;
	NFP_IF_MAP *currMap = NULL, *prevMap = NULL;

	if ((ifIdx <= 0) || (mvNfpIfMapGet(ifIdx) == NULL)) {
		mvOsPrintf("%s: interface %d is not valid\n", __func__, ifIdx);
		return MV_BAD_PARAM;
	}
	hash = (ifIdx & NFP_DEV_HASH_MASK);

	/* remove from nfp_if_map */
	for (currMap = nfp_if_map[hash]; currMap != NULL; prevMap = currMap, currMap = currMap->nextMap) {
		if (currMap->ifIdx == ifIdx) {
			if (prevMap == NULL)
				nfp_if_map[hash] = currMap->nextMap;
			else
				prevMap->nextMap = currMap->nextMap;

			/* do not delete element yet, need to remove it from nfp_if_real_map */
			break;
		}
	}

	if (currMap) {
		if (currMap->port != NFP_INVALID_PORT) {
			/* remove from nfp_if_real_map */
			if (currMap->switchGroup == NFP_INVALID_SWITCH_GROUP)
				nfp_if_real_map[currMap->port][0] = NULL;
			else
				nfp_if_real_map[currMap->port][currMap->switchGroup] = NULL;
		}
		/* delete element */
		mvOsFree(currMap);
		return MV_OK;
	}
	return MV_NOT_FOUND;
}

#ifdef NFP_BRIDGE
/* There are functions common for mvNfpBridge.c and for mvNfpFdb.c */
MV_STATUS mvNfpIfToBridgeAdd(int bridgeIf, int portIf)
{
	NFP_IF_MAP  *portIfMap = mvNfpIfMapGet(portIf);
	NFP_IF_MAP  *bridgeIfMap = mvNfpIfMapGet(bridgeIf);

	if (bridgeIfMap == NULL) {
		mvOsPrintf("%s: trying to add interface (%d) to non-registered bridge (%d)\n",
			__func__, portIf, bridgeIf);
		return MV_NOT_FOUND;
	}
	if (!(bridgeIfMap->flags & NFP_F_MAP_BRIDGE) || (bridgeIfMap->ifIdx != bridgeIf)) {
		mvOsPrintf("%s: Wrong bridgeIf=%d\n", __func__, bridgeIf);
		return MV_FAIL;
	}

	if (portIfMap != NULL) {
		/* Check that port_if isn't already connected to an other bridge */
		if ((portIfMap->flags & NFP_F_MAP_BRIDGE_PORT) && (portIfMap->bridgeIf != bridgeIf))
			return MV_BUSY;

		portIfMap->flags |= NFP_F_MAP_BRIDGE_PORT;
		portIfMap->bridgeIf = bridgeIf;
		return MV_OK;
	}
	return MV_NOT_FOUND;
}

MV_STATUS mvNfpIfToBridgeDel(int bridge_if, int port_if)
{
	NFP_IF_MAP  *portIfMap = mvNfpIfMapGet(port_if);

	if (portIfMap != NULL) {
		/* Check that port_if is connected to this bridge */
		if (!(portIfMap->flags & NFP_F_MAP_BRIDGE_PORT) || (portIfMap->bridgeIf != bridge_if))
			return MV_BAD_PARAM;

		portIfMap->flags &= ~NFP_F_MAP_BRIDGE_PORT;
		portIfMap->bridgeIf = 0;
		return MV_OK;
	}
	return MV_NOT_FOUND;
}

MV_BOOL mvNfpIfOnSameBridge(NFP_IF_MAP *ifMap1, NFP_IF_MAP *ifMap2)
{
	if ((ifMap1 != NULL) && (ifMap2 != NULL)) {
		/* first interface is bridge and second is port on the same bridge */
		if ((ifMap1->flags & NFP_F_MAP_BRIDGE) &&
			(ifMap2->flags & NFP_F_MAP_BRIDGE_PORT) &&
			(ifMap1->ifIdx == ifMap2->bridgeIf))
			return MV_TRUE;

		/* second interface is bridge and first is port on the same bridge */
		if ((ifMap1->flags & NFP_F_MAP_BRIDGE_PORT) &&
			(ifMap2->flags & NFP_F_MAP_BRIDGE) &&
			(ifMap1->bridgeIf == ifMap2->ifIdx))
			return MV_TRUE;

		/* two interfaces are ports of the same bridge */
		if ((ifMap1->flags & NFP_F_MAP_BRIDGE_PORT) &&
		    (ifMap2->flags & NFP_F_MAP_BRIDGE_PORT) &&
		    (ifMap1->bridgeIf == ifMap2->bridgeIf))
			return MV_TRUE;
	}
	return MV_FALSE;
}
#endif /* NFP_BRIDGE */

MV_VOID mvNfpIpInfoPrint(u32 dbgLevel, int family, u8 *ipAddr)
{
	if (nfpDebugLevel & dbgLevel) {
		if (family == MV_INET)
			mvOsPrintf("IPv4: " MV_IPQUAD_FMT "\n",	MV_IPQUAD(ipAddr));
		else
			mvOsPrintf("IPv6: " MV_IP6_FMT "\n", MV_IP6_ARG(ipAddr));
	}
}

MV_VOID mvNfp2TupleInfoPrint(u32 dbgLevel, int family, u8 *srcL3, u8 *dstL3)
{
	if (nfpDebugLevel & dbgLevel) {
		if (family == MV_INET)
			mvOsPrintf("IPv4: " MV_IPQUAD_FMT "->" MV_IPQUAD_FMT"\n",
				MV_IPQUAD(srcL3), MV_IPQUAD(dstL3));
		else
			mvOsPrintf("IPv6: " MV_IP6_FMT "->" MV_IP6_FMT"\n",
				MV_IP6_ARG(srcL3), MV_IP6_ARG(dstL3));
	}
}


MV_VOID mvNfp5TupleInfoPrint(u32 dbgLevel, int family, u8 *srcL3, u8 *dstL3, u16 sport, u16 dport, u8 proto)
{
	if (nfpDebugLevel & dbgLevel) {
		if (family == MV_INET)
			mvOsPrintf("IPv4: " MV_IPQUAD_FMT ":%d->" MV_IPQUAD_FMT":%d",
				MV_IPQUAD(srcL3), sport, MV_IPQUAD(dstL3), dport);
		else
			mvOsPrintf("IPv6: " MV_IP6_FMT ":%d->" MV_IP6_FMT":%d",
				MV_IP6_ARG(srcL3), sport, MV_IP6_ARG(dstL3), dport);

		if (proto == MV_IP_PROTO_TCP)
			mvOsPrintf(", proto = TCP\n");
		else if (proto == MV_IP_PROTO_UDP)
			mvOsPrintf(", proto = UDP\n");
		else
			mvOsPrintf(", proto = Unknown (%d)\n", proto);
	}
}

static void mvNfpIfMapPrint(NFP_IF_MAP *ifMap)
{
	mvOsPrintf(" %8s  %3d  %3d   %3d   %4d  0x%04x  "MV_MACQUAD_FMT"  %4d  %3d  %3d    0x%04x\n",
				ifMap->name, ifMap->ifIdx, ifMap->port, ifMap->switchGroup, ifMap->vlanId,
				ifMap->txMh, MV_MACQUAD(ifMap->mac), ifMap->mtu,
				ifMap->bridgeIf, ifMap->parentIf ? ifMap->parentIf->ifIdx : ifMap->ifIdx, ifMap->flags);
	if (ifMap->virtIf) {
		NFP_IF_MAP *ifVirt = ifMap->virtIf;

		mvOsPrintf("\t Virtual list: ");
		while (ifVirt) {
			mvOsPrintf(" %d,", ifVirt->ifIdx);
			ifVirt = ifVirt->virtNext;
		}
		mvOsPrintf("\n");
	}
}

MV_VOID mvNfpIfMapDump(void)
{
	int		i;
	NFP_IF_MAP	*ifMap;

	mvOsPrintf("\n(ifMap - direct)\n");
	mvOsPrintf("[No]:      name   idx  port  swGr  vid   txMh           mac         mtu  brIf  upIf  flags\n");
	for (i = 0; i < NFP_DEV_HASH_SZ; i++) {
		ifMap = nfp_if_map[i];

		while (ifMap != NULL) {
			mvOsPrintf("[%2d]: ", i);
			mvNfpIfMapPrint(ifMap);
			ifMap = ifMap->nextMap;
		}
	}
	mvOsPrintf("\n");
	mvOsPrintf("(ifMap - Real)\n");
	mvOsPrintf("[Port][Group]:      name  idx\n");
	for (i = 0; i < NFP_MAX_PORTS; i++) {
		int j;

		for (j = 0; j < NFP_MAX_SWITCH_GROUPS; j++) {
			ifMap = nfp_if_real_map[i][j];
			if (ifMap != NULL)
				mvOsPrintf("[%4d][%5d]:  %8s  %3d\n", i, j, ifMap->name, ifMap->ifIdx);
		}
	}
}

MV_VOID mvNfpStats(MV_U32 port)
{
	if ((port < 0) || (port >= NFP_MAX_PORTS)) {
		mvOsPrintf("Invalid port number %d\n", port);
		return;
	}

	mvOsPrintf("\n====================================================\n");
	mvOsPrintf(" NFP statistics");
	mvOsPrintf("\n----------------------------------------------------\n");

#ifdef NFP_STAT
	mvOsPrintf("nfp_rx........................%10u\n", nfp_stats[port].rx);
	mvOsPrintf("nfp_iif_err...................%10u\n", nfp_stats[port].iif_err);
	mvOsPrintf("nfp_oif_err...................%10u\n", nfp_stats[port].oif_err);

#ifdef NFP_VLAN
	mvOsPrintf("nfp_vlan_rx_tag_drop..........%10u\n", nfp_stats[port].vlan_rx_tag_drop);
	mvOsPrintf("nfp_vlan_rx_untag_drop........%10u\n", nfp_stats[port].vlan_rx_untag_drop);
	mvOsPrintf("nfp_vlan_rx_unknown_drop......%10u\n", nfp_stats[port].vlan_rx_unknown_drop);
	mvOsPrintf("nfp_vlan_rx_found.............%10u\n", nfp_stats[port].vlan_rx_found);
	mvOsPrintf("nfp_vlan_rx_trans.............%10u\n", nfp_stats[port].vlan_rx_trans);
	mvOsPrintf("nfp_vlan_tx_add...............%10u\n", nfp_stats[port].vlan_tx_add);
	mvOsPrintf("nfp_vlan_tx_remove............%10u\n", nfp_stats[port].vlan_tx_remove);
	mvOsPrintf("nfp_vlan_tx_replace...........%10u\n", nfp_stats[port].vlan_tx_replace);
#endif /* NFP_VLAN */

#ifdef NFP_BRIDGE
#ifdef NFP_FDB_MODE
	mvOsPrintf("nfp_fdb_local.................%10u\n", nfp_stats[port].fdb_local);
	mvOsPrintf("nfp_fdb_sa_miss...............%10u\n", nfp_stats[port].fdb_sa_miss);
	mvOsPrintf("nfp_fdb_da_miss...............%10u\n", nfp_stats[port].fdb_da_miss);
	mvOsPrintf("nfp_fdb_port_miss.............%10u\n", nfp_stats[port].fdb_port_miss);
	mvOsPrintf("nfp_fdb_hit...................%10u\n", nfp_stats[port].fdb_hit);
#else
	mvOsPrintf("nfp_bridge_miss...............%10u\n", nfp_stats[port].bridge_miss);
	mvOsPrintf("nfp_bridge_hit................%10u\n", nfp_stats[port].bridge_hit);
	mvOsPrintf("nfp_bridge_local..............%10u\n", nfp_stats[port].bridge_local);
#endif /* NFP_FDB_MODE */
#endif /* NFP_BRIDGE */

	mvOsPrintf("nfp_non_ip....................%10u\n", nfp_stats[port].non_ip);
	mvOsPrintf("nfp_ipv4_csum_err.............%10u\n", nfp_stats[port].ipv4_csum_err);
	mvOsPrintf("nfp_mac_mcast.................%10u\n", nfp_stats[port].mac_mcast);
	mvOsPrintf("nfp_ttl_exp...................%10u\n", nfp_stats[port].ttl_exp);
	mvOsPrintf("nfp_l4_unknown................%10u\n", nfp_stats[port].l4_unknown);
	mvOsPrintf("nfp_l4_csum_err...............%10u\n", nfp_stats[port].l4_csum_err);
	mvOsPrintf("nfp_ipv4......................%10u\n", nfp_stats[port].ipv4);
	mvOsPrintf("nfp_ipv6......................%10u\n", nfp_stats[port].ipv6);
	mvOsPrintf("nfp_ip_rx_frag................%10u\n", nfp_stats[port].ipv4_rx_frag);
	mvOsPrintf("nfp_ip_tx_frag................%10u\n", nfp_stats[port].ip_tx_frag);
	mvOsPrintf("nfp_ip_tx_frag_err............%10u\n", nfp_stats[port].ip_tx_frag_err);

#ifdef NFP_PPP
	mvOsPrintf("pppoe_rx_not_found............%10u\n", nfp_stats[port].pppoe_rx_not_found);
	mvOsPrintf("pppoe_rx_found................%10u\n", nfp_stats[port].pppoe_rx_found);
	mvOsPrintf("pppoe_tx_add..................%10u\n", nfp_stats[port].pppoe_tx_add);
	mvOsPrintf("pppoe_tx_remove...............%10u\n", nfp_stats[port].pppoe_tx_remove);
	mvOsPrintf("pppoe_tx_replace..............%10u\n", nfp_stats[port].pppoe_tx_replace);
#endif /* NFP_PPP */

#ifdef NFP_FIB
	mvOsPrintf("nfp_fib_hit...................%10u\n", nfp_stats[port].fib_hit);
	mvOsPrintf("nfp_fib_miss..................%10u\n", nfp_stats[port].fib_miss);
#endif /* NFP_FIB */

#ifdef NFP_CT
	mvOsPrintf("nfp_ct_hit....................%10u\n", nfp_stats[port].ct_hit);
	mvOsPrintf("nfp_ct_miss...................%10u\n", nfp_stats[port].ct_miss);
	mvOsPrintf("nfp_ct_tcp_fin_rst............%10u\n", nfp_stats[port].ct_tcp_fin_rst);
#ifdef NFP_NAT
	mvOsPrintf("nfp_dnat_hit..................%10u\n", nfp_stats[port].dnat_hit);
	mvOsPrintf("nfp_dnat_miss.................%10u\n", nfp_stats[port].dnat_miss);
	mvOsPrintf("nfp_dnat_inv..................%10u\n", nfp_stats[port].dnat_inv);

	mvOsPrintf("nfp_snat_hit..................%10u\n", nfp_stats[port].snat_hit);
	mvOsPrintf("nfp_snat_miss.................%10u\n", nfp_stats[port].snat_miss);
	mvOsPrintf("nfp_snat_inv..................%10u\n", nfp_stats[port].snat_inv);
#endif /* NFP_NAT */
#endif /* NFP_CT */

	mvOsMemset(&nfp_stats[port], 0, sizeof(NFP_STATS));
#endif /* NFP_STAT */
}
