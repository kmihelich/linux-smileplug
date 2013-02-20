/*******************************************************************************
Copyright (C) Marvell Interfdbional Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
Interfdbional Ltd. and/or its affiliates ("Marvell") under the following
alterfdbive licensing terms.  Once you have made an election to distribute the
File under one of the following license alterfdbives, please (i) delete this
introductory statement regarding license alterfdbives, (ii) delete the two
license alterfdbives that you have not elected to use and (iii) preserve the
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
* mvNfpBridge.c - Marvell Fast Network Processing
*
* DESCRIPTION:
*
*       Supported Features:
*       - OS independent.
*
*******************************************************************************/

#include "mvOs.h"
#include "mvDebug.h"
#include "mvList.h"
#include "gbe/mvNeta.h"
#include "mvNfpDefs.h"
#include "mvNfp.h"

NFP_RULE_BRIDGE **nfp_bridge_hash = NULL;

MV_STATUS _INIT mvNfpBridgeInit(void)
{
	MV_U32 bytes = sizeof(NFP_RULE_BRIDGE *) * NFP_BRIDGE_HASH_SIZE;

	nfp_bridge_hash = (NFP_RULE_BRIDGE **)mvOsMalloc(bytes);
	if (nfp_bridge_hash == NULL) {
		mvOsPrintf("NFP (bridge hash): not enough memory\n");
		return MV_NO_RESOURCE;
	}
	mvOsMemset(nfp_bridge_hash, 0, bytes);

	mvOsPrintf("NFP (bridge) init %d entries, %d bytes\n", NFP_BRIDGE_HASH_SIZE, bytes);

	return MV_OK;
}

/* Clear Bridge Rule Database */
MV_STATUS mvNfpBridgeClear(void)
{
	int	i;
	NFP_RULE_BRIDGE	*rule, *tmp;

	if (nfp_bridge_hash == NULL)
		return MV_NOT_INITIALIZED;

	for (i = 0; i < NFP_BRIDGE_HASH_SIZE; i++) {

		rule = nfp_bridge_hash[i];
		while (rule) {
			tmp = rule;
			rule = rule->next;
			mvOsFree(tmp);
		}
		nfp_bridge_hash[i] = NULL;
	}
	return MV_OK;
}

void mvNfpBridgeDestroy(void)
{
	if (nfp_bridge_hash != NULL)
		mvOsFree(nfp_bridge_hash);
}

void mvNfpBridgeFibUpdate(NFP_RULE_BRIDGE *rule, int prev_oif)
{
	MV_LIST_ELEMENT	*curr, *tmp;
	int i;
	NFP_RULE_FIB	*fib;
	NFP_IF_MAP      *outIfMap;

	outIfMap = mvNfpIfMapGet(rule->oif);
	/* Update relevant FIB rules */
	for (i = 0; i < NFP_FIB_HASH_SIZE; i++) {
		fib = fib_hash[i];
		while (fib) {
			if ((prev_oif == fib->oif) &&
			    (!memcmp(rule->da, fib->da, MV_MAC_ADDR_SIZE)) &&
			    (!memcmp(rule->sa, fib->sa, MV_MAC_ADDR_SIZE))) {
				fib->oif = outIfMap->ifIdx;
				fib->mh = outIfMap->txMh;
			}
			fib = fib->next;
		}
	}

	curr = fib_inv_list->next;
	while (curr) {
		fib = (NFP_RULE_FIB *)curr->data;
		tmp = curr->next;
		if ((prev_oif == fib->oif) &&
		    (!memcmp(rule->da, fib->da, MV_MAC_ADDR_SIZE)) &&
		    (!memcmp(rule->sa, fib->sa, MV_MAC_ADDR_SIZE))) {
			fib->oif = outIfMap->ifIdx;
			fib->mh = outIfMap->txMh;
			fib->flags &= ~NFP_F_FIB_BRIDGE_INV;

			mvNfpFibRuleValid(fib, curr);
		}
		curr = tmp;
	}
}

MV_STATUS mvNfpBridgeRuleAdd(NFP_RULE_BRIDGE *rule2)
{
	MV_U32          hash;
	NFP_RULE_BRIDGE *rule;
	NFP_RULE_FIB    *fib;
	MV_LIST_ELEMENT	*curr, *tmp;
	NFP_IF_MAP      *inIfMap, *outIfMap;

	hash = mvNfpBridgeRuleHash(rule2->da, rule2->sa, rule2->iif);
	rule = nfp_bridge_hash[hash];
	while (rule) {
		if (mvNfpBridgeRuleCmp(rule2->da, rule2->sa, rule2->iif, rule)) {
			MV_U32 age = rule->age;

			/* Update rule, but save age */
			mvNfpBridgeFibUpdate(rule2, mvNfpIfMapGet(rule->oif)->ifIdx);
			mvOsMemcpy(rule, rule2, sizeof(NFP_RULE_BRIDGE));
			rule->age = age;
			goto out;
		}
		rule = rule->next;
	}

	/* New rule - check that iif and oif connected to the same bridgeIf */
	inIfMap = mvNfpIfMapGet(rule2->iif);
	outIfMap = mvNfpIfMapGet(rule2->oif);

	if (mvNfpIfOnSameBridge(inIfMap, outIfMap) == MV_FALSE) {
		mvOsPrintf("%s: iif=%d and oif=%d are not connected to the same bridge\n",
					__func__, rule2->iif, rule2->oif);
		return MV_FAIL;
	}

	rule = (NFP_RULE_BRIDGE *)mvOsMalloc(sizeof(NFP_RULE_BRIDGE));
	if (rule == NULL) {
		mvOsPrintf("%s: NFP (bridge rule) - OOM\n", __func__);
		return MV_FAIL;
	}

	mvOsMemcpy(rule, rule2, sizeof(NFP_RULE_BRIDGE));

	rule->next = nfp_bridge_hash[hash];
	nfp_bridge_hash[hash] = rule;

	/* Update incomplete FIB entries */
	curr = fib_inv_list->next;
	while (curr) {
		fib = (NFP_RULE_FIB *)curr->data;
		tmp = curr->next;
		if ((rule->iif == fib->oif) &&
		    (!memcmp(rule->da, fib->da, MV_MAC_ADDR_SIZE)) &&
		    (!memcmp(rule->sa, fib->sa, MV_MAC_ADDR_SIZE))) {

			/* Found incomplete FIB entry */
			fib->oif = outIfMap->ifIdx;
			fib->mh = outIfMap->txMh;
			fib->flags &= ~NFP_F_FIB_BRIDGE_INV;

			mvNfpFibRuleValid(fib, curr);
		}
		curr = tmp;
	}

out:
	NFP_DBG("NFP (bridge) add %p\n", rule);
	return MV_OK;
}

MV_STATUS mvNfpBridgeRuleDel(NFP_RULE_BRIDGE *rule2)
{
	MV_U32 hash;
	NFP_RULE_BRIDGE *rule, *prev;
	NFP_RULE_FIB	*fib, *fib_prev;
	NFP_IF_MAP      *ifMap;
	int i;

	hash = mvNfpBridgeRuleHash(rule2->da, rule2->sa, rule2->iif);

	rule = nfp_bridge_hash[hash];
	prev = NULL;

	while (rule) {
		if (mvNfpBridgeRuleCmp(rule2->da, rule2->sa, rule2->iif, rule)) {
			/* Found: delete rule */
			if (prev)
				prev->next = rule->next;
			else
				nfp_bridge_hash[hash] = rule->next;

			NFP_DBG("NFP (bridge) del %p\n", rule);
			rule2->oif = rule->oif;
			mvOsFree(rule);
			break;
		}
		prev = rule;
		rule = rule->next;
	}
	if (rule == NULL)
		return MV_NOT_FOUND;

	/* Invalidate relevant FIB rules */
	ifMap = mvNfpIfMapGet(rule2->oif);

	for (i = 0; i < NFP_FIB_HASH_SIZE; i++) {
		fib = fib_hash[i];
		fib_prev = NULL;
		while (fib) {
			if ((rule2->oif == fib->oif) &&
				(!memcmp(rule2->da, fib->da, MV_MAC_ADDR_SIZE))) {
				/* Invalidate FIB route. FIXME - update HWF rule */
				fib->flags |= NFP_F_FIB_BRIDGE_INV;
				fib->oif = ifMap->bridgeIf;

				/* Remove FIB rule from hash table */
				if (fib_prev)
					fib_prev->next = fib->next;
				else
					fib_hash[i] = fib->next;

				/* Add FIB rule to incomplete list */
				mvListAddHead(fib_inv_list, (MV_ULONG)fib);
#ifdef NFP_CT
				/* Invalidate relevant CT rules */
				mvNfpCtRuleFibInvalidate(fib);
#endif /* NFP_CT */
			}
			fib_prev = fib;
			fib = fib->next;
		}
	}
	return MV_OK;
}

MV_STATUS mvNfpBridgeRuleAge(NFP_RULE_BRIDGE *rule2)
{
	NFP_RULE_BRIDGE *rule;

	rule = mvNfpBridgeLookup(rule2->da, rule2->sa, rule2->iif);
	if (rule) {
		rule2->age = rule->age;
		rule->age = 0;
		return MV_OK;
	}
	rule2->age = 0;
	return MV_NOT_FOUND;
}

#ifdef NFP_CLASSIFY
MV_STATUS mvNfpBridgeTxqRuleAdd(NFP_RULE_BRIDGE *rule2)
{
	NFP_RULE_BRIDGE *rule;

	/* sanity: chack txq parameter */
	if ((rule2->txq < 0) || (rule2->txq >= CONFIG_MV_ETH_TXQ)) {
		mvOsPrintf("%s Error: txq (%d) is out of range\n", __func__, rule2->txq);
		return MV_BAD_PARAM;
	}

	rule = mvNfpBridgeLookup(rule2->da, rule2->sa, rule2->iif);
	if (rule) {
		/* Update rule Txq table */
		if (rule2->flags & NFP_F_BR_SET_TXQ) {
			rule->txq = rule2->txq;
			rule->flags |= NFP_F_BR_SET_TXQ;
		} else {
			mvOsPrintf("%s: NFP_F_BR_SET_TXQ flag is not set\n", __func__);
			return MV_BAD_PARAM;
		}
	} else {
		mvOsPrintf("%s Error: Could not find existing Bridging rule\n", __func__);
		return MV_NOT_FOUND;
	}

	NFP_DBG("NFP (bridge txq) set %p\n", rule);

	return MV_OK;
}

MV_STATUS mvNfpBridgeTxqRuleDel(NFP_RULE_BRIDGE *rule2)
{
	NFP_RULE_BRIDGE *rule;

	rule = mvNfpBridgeLookup(rule2->da, rule2->sa, rule2->iif);
	if (rule) {
		rule->txq = 0;
		rule->flags &= ~NFP_F_BR_SET_TXQ;
		return MV_OK;
	}

	return MV_NOT_FOUND;
}

MV_STATUS mvNfpBridgeTxpRuleAdd(NFP_RULE_BRIDGE *rule2)
{
	NFP_RULE_BRIDGE *rule;

	/* sanity: chack txp parameter */
	if (rule2->txp < 0) {
		mvOsPrintf("%s Error: txp (%d) is out of range\n", __func__, rule2->txp);
		return MV_BAD_PARAM;
	}

	rule = mvNfpBridgeLookup(rule2->da, rule2->sa, rule2->iif);
	if (rule) {
		/* Update rule Txp table */
		if (rule2->flags & NFP_F_BR_SET_TXP) {
			rule->txp = rule2->txp;
			rule->flags |= NFP_F_BR_SET_TXP;
		} else {
			mvOsPrintf("%s: NFP_F_BR_SET_TXP flag is not set\n", __func__);
			return MV_BAD_PARAM;
		}
	} else {
		mvOsPrintf("%s Error: Could not find existing Bridging rule\n", __func__);
		return MV_NOT_FOUND;
	}

	NFP_DBG("NFP (bridge txp) set %p\n", rule);

	return MV_OK;
}

MV_STATUS mvNfpBridgeTxpRuleDel(NFP_RULE_BRIDGE *rule2)
{
	NFP_RULE_BRIDGE *rule;

	rule = mvNfpBridgeLookup(rule2->da, rule2->sa, rule2->iif);
	if (rule) {
		rule->txp = 0;
		rule->flags &= ~NFP_F_BR_SET_TXP;
		return MV_OK;
	}

	return MV_NOT_FOUND;
}

MV_STATUS mvNfpBridgeMhRuleAdd(NFP_RULE_BRIDGE *rule2)
{
	NFP_RULE_BRIDGE *rule;

	/* sanity: chack MH parameter */
	if ((rule2->mh < 0) || (rule2->mh >= 0xFFFF)) {
		mvOsPrintf("%s Error: mh (%d) is out of range\n", __func__, rule2->mh);
		return MV_BAD_PARAM;
	}

	rule = mvNfpBridgeLookup(rule2->da, rule2->sa, rule2->iif);
	if (rule) {
		/* Update rule Txq table */
		if (rule2->flags & NFP_F_BR_SET_MH) {
			rule->mh = rule2->mh;
			rule->flags |= NFP_F_BR_SET_MH;
		} else {
			mvOsPrintf("%s: NFP_F_BR_SET_MH flag is not set\n", __func__);
			return MV_BAD_PARAM;
		}
	} else {
		mvOsPrintf("%s Error: Could not find existing Bridging rule\n", __func__);
		return MV_NOT_FOUND;
	}

	NFP_DBG("NFP (bridge mh) set %p\n", rule);

	return MV_OK;
}

MV_STATUS mvNfpBridgeMhRuleDel(NFP_RULE_BRIDGE *rule2)
{
	NFP_RULE_BRIDGE *rule;

	rule = mvNfpBridgeLookup(rule2->da, rule2->sa, rule2->iif);
	if (rule) {
		rule->mh = 0;
		rule->flags &= ~NFP_F_BR_SET_MH;
		return MV_OK;
	}

	return MV_NOT_FOUND;
}

MV_STATUS mvNfpBridgeVlanPrioRuleAdd(NFP_RULE_BRIDGE *rule2, int eth_type, int new_prio)
{
	NFP_RULE_BRIDGE *rule;
	int i, first;

	/* sanity: chack new_prio parameter */
	if ((new_prio < NFP_VPRI_MIN) || (new_prio > NFP_VPRI_MAX)) {
		mvOsPrintf("%s Error: new_prio value (%d) is out of range\n", __func__, new_prio);
		return MV_BAD_PARAM;
	}

	rule = mvNfpBridgeLookup(rule2->da, rule2->sa, rule2->iif);

	if (rule) {
		if (eth_type == MV_ETH_NFP_GLOBAL_MAP) {
			rule->vpri_map[NFP_VPRI_MAP_GLOBAL].eth_type = eth_type;
			rule->vpri_map[NFP_VPRI_MAP_GLOBAL].new_prio = new_prio;
			rule->vpri_map[NFP_VPRI_MAP_GLOBAL].valid = MV_TRUE;
		} else {
			for (first = -1, i = 0; i < NFP_VPRI_MAP_GLOBAL; i++) {
				if (rule->vpri_map[i].eth_type == eth_type) {
					rule->vpri_map[i].new_prio = new_prio;	/* just update */
					return MV_OK;
				}
				if ((first == -1) && (rule->vpri_map[i].valid == MV_FALSE))
					first = i;
			}
			if (first != -1) {
				rule->vpri_map[first].eth_type = eth_type;
				rule->vpri_map[first].new_prio = new_prio;
				rule->vpri_map[first].valid = MV_TRUE;
			} else {
				mvOsPrintf("%s Error: No available space for additional VPRIO mapping\n", __func__);
			}
		}
		rule->flags |= NFP_F_BR_SET_VLAN_PRIO;
	} else {
		mvOsPrintf("%s Error: Could not find existing bridging rule\n", __func__);
		return MV_NOT_FOUND;
	}

	return MV_OK;
}

static INLINE MV_STATUS mvNfpIsVpriSet(NFP_RULE_BRIDGE *rule)
{
	int i;

	for (i = 0; i <= NFP_VPRI_MAP_GLOBAL; i++)
		if (rule->vpri_map[i].valid)
			return MV_TRUE;

	return MV_FALSE;
}

MV_STATUS mvNfpBridgeVlanPrioRuleDel(NFP_RULE_BRIDGE *rule2, int eth_type)
{
	NFP_RULE_BRIDGE *rule;
	int i;

	/* sanity: check eth_type parameter */
	if ((eth_type < MV_ETH_NFP_GLOBAL_MAP) || (eth_type > 0xFFFF)) {
		mvOsPrintf("%s Error: eth_type value (%d) is out of range\n", __func__, eth_type);
		return MV_BAD_PARAM;
	}

	rule = mvNfpBridgeLookup(rule2->da, rule2->sa, rule2->iif);
	if (rule) {
		if (eth_type == MV_ETH_NFP_GLOBAL_MAP) {
			rule->vpri_map[NFP_VPRI_MAP_GLOBAL].eth_type = 0;
			rule->vpri_map[NFP_VPRI_MAP_GLOBAL].new_prio = 0;
			rule->vpri_map[NFP_VPRI_MAP_GLOBAL].valid = MV_FALSE;
		} else {
			for (i = 0; i <= NFP_VPRI_MAP_GLOBAL; i++) {
				if (rule->vpri_map[i].eth_type == eth_type) {
					rule->vpri_map[i].eth_type = 0;
					rule->vpri_map[i].new_prio = 0;
					rule->vpri_map[i].valid = MV_FALSE;
				}
			}
		}
		if (!mvNfpIsVpriSet(rule))
			rule->flags &= ~NFP_F_BR_SET_VLAN_PRIO;

		return MV_OK;
	}

	return MV_NOT_FOUND;
}
#endif /* NFP_CLASSIFY */


static void mvNfpBridgeRulePrint(NFP_RULE_BRIDGE *rule)
{
	int i;

	mvOsPrintf("Bridge: ");

	mvOsPrintf(" DA=" MV_MACQUAD_FMT ", SA=" MV_MACQUAD_FMT ", iif=%u, oif=%u, flags=0x%04x, age=%u\n",
		MV_MACQUAD(rule->da), MV_MACQUAD(rule->sa), rule->iif, rule->oif, rule->flags, rule->age);

#ifdef NFP_CLASSIFY
	if (rule->flags & NFP_F_BR_SET_VLAN_PRIO) {
		mvOsPrintf(" VLAN Prio Map: ");
		mvOsPrintf("eth_type\t\tVPrio\n");
		for (i = 0; i <= NFP_VPRI_MAP_GLOBAL; i++) {
			if (rule->vpri_map[i].valid) {
				if (i == NFP_VPRI_MAP_GLOBAL)
					mvOsPrintf("Global:\t\t%d\n", rule->vpri_map[i].new_prio);
				else
					mvOsPrintf("0x%X\t\t%d\n", rule->vpri_map[i].eth_type, rule->vpri_map[i].new_prio);
			}
		}
	}
#endif /* NFP_CLASSIFY */
}

void mvNfpBridgeDump(void)
{
	int             i;
	NFP_RULE_BRIDGE *rule;

	mvOsPrintf("\n(bridge)\n");
	for (i = 0; i < NFP_BRIDGE_HASH_SIZE; i++) {
		rule = nfp_bridge_hash[i];

		while (rule) {
			mvOsPrintf(" [%5d] ", i);
			mvNfpBridgeRulePrint(rule);
			rule = rule->next;
		}
	}
}

void mvNfpFlushBridge(int ifindex)
{
	int i;

	NFP_RULE_BRIDGE *rule;

	for (i = 0; i < NFP_BRIDGE_HASH_SIZE; i++) {
		rule = nfp_bridge_hash[i];

		while (rule) {
			if ((rule->iif == ifindex) || (rule->oif == ifindex) || (ifindex == -1))
				mvNfpBridgeRuleDel(rule);
			rule = rule->next;
		}
	}
}

