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

/*******************************************************************************
* mv_nfp_mgr.c - Marvell Network Fast Processing Manager
*
* DESCRIPTION:
*
*       Supported Features:
*
*******************************************************************************/

#include "mvCommon.h"
#include "mvTypes.h"
#include <linux/kernel.h>
#include <linux/ip.h>
#include <net/arp.h>
#include <net/route.h>

#include "mvDebug.h"
#include "mvOs.h"
#include "mvSysHwConfig.h"
#include "ctrlEnv/mvCtrlEnvLib.h"
#include "gbe/mvNeta.h"
#include "nfp/mvNfp.h"
#include "net_dev/mv_netdev.h"
#include "mv_nfp_mgr.h"

#ifdef NFP_LEARN

#if defined(NFP_CT_LEARN)
#include <net/netfilter/nf_nat.h>
#include <linux/netfilter/ipt_NFP.h>
#endif /* NFP_CT_LEARN */

#if defined(NFP_CLASSIFY) && defined(NFP_CT_LEARN)
static inline void nfp_ct_classify_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto,
					struct ipt_nfp_info *info)
{
	int i;

	if (info->flags & IPT_NFP_F_SET_TXQ) {
		if (info->txq >= 0)
			nfp_ct_txq_set(family, src_l3, dst_l3, sport, dport, proto, info->txq);
		else
			nfp_ct_txq_del(family, src_l3, dst_l3, sport, dport, proto);
	}
	if (info->flags & IPT_NFP_F_SET_TXP) {
		if (info->txp >= 0)
			nfp_ct_txp_set(family, src_l3, dst_l3, sport, dport, proto, info->txp);
		else
			nfp_ct_txp_del(family, src_l3, dst_l3, sport, dport, proto);
	}
	if (info->flags & IPT_NFP_F_SET_MH) {
		if (info->mh >= 0)
			nfp_ct_mh_set(family, src_l3, dst_l3, sport, dport, proto, info->mh);
		else
			nfp_ct_mh_del(family, src_l3, dst_l3, sport, dport, proto);
	}

	if (info->flags & IPT_NFP_F_SET_DSCP) {
		for (i = NFP_DSCP_MIN; i <= NFP_DSCP_MAX; i++) {
			if (info->dscp_map[i].valid)
				nfp_ct_dscp_set(family, src_l3, dst_l3, sport, dport, proto, i, info->dscp_map[i].new_dscp);
		}
	}

	if (info->flags & IPT_NFP_F_SET_VLAN_PRIO) {
		for (i = NFP_VPRI_MIN; i <= NFP_VPRI_MAX; i++) {
			if (info->vpri_map[i].valid)
				nfp_ct_vlan_prio_set(family, src_l3, dst_l3, sport, dport, proto, i, info->vpri_map[i].new_prio);
		}
	}

	if (info->flags & IPT_NFP_F_SET_DSCP_DEF) {
		if (info->dscp_map[IPT_NFP_DSCP_MAP_GLOBAL].valid) {
			nfp_ct_dscp_set(family, src_l3, dst_l3, sport, dport, proto,
					MV_ETH_NFP_GLOBAL_MAP, info->dscp_map[IPT_NFP_DSCP_MAP_GLOBAL].new_dscp);
		}
	}

	if (info->flags & IPT_NFP_F_SET_VLAN_PRIO_DEF) {
		if (info->vpri_map[IPT_NFP_VPRI_MAP_GLOBAL].valid) {
			nfp_ct_vlan_prio_set(family, src_l3, dst_l3, sport, dport, proto,
						MV_ETH_NFP_GLOBAL_MAP, info->vpri_map[IPT_NFP_VPRI_MAP_GLOBAL].new_prio);

		}
	}
}
#endif /* defined(NFP_CLASSIFY) && defined(NFP_CT_LEARN) */

extern rwlock_t nfp_lock;

#ifdef NFP_FDB_LEARN
int nfp_bridge_learn_en = CONFIG_MV_ETH_NFP_FDB_LEARN_DEF;
int nfp_bridge_age_en = CONFIG_MV_ETH_NFP_FDB_LEARN_DEF;

void fdb_sync(void);

int nfp_bridge_is_learning_enabled(void)
{
	return nfp_bridge_learn_en;
}

int nfp_bridge_learn_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_bridge_learn_en = en;
	return MV_OK;
}

int nfp_bridge_is_aging_enabled(void)
{
	return nfp_bridge_age_en;
}

int nfp_bridge_age_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_bridge_age_en = en;
	return MV_OK;
}
#endif /* NFP_FDB_LEARN */

#ifdef NFP_VLAN_LEARN
int nfp_vlan_learn_en = CONFIG_MV_ETH_NFP_VLAN_LEARN_DEF;

void vlan_sync(void);

int nfp_vlan_is_learning_enabled(void)
{
	return nfp_vlan_learn_en;
}

int nfp_vlan_learn_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_vlan_learn_en = en;
	return MV_OK;
}
#endif /* NFP_VLAN_LEARN */

#ifdef NFP_FIB_LEARN
void neigh_sync(int family);

int nfp_fib_learn_en = CONFIG_MV_ETH_NFP_FIB_LEARN_DEF;
int nfp_fib_age_en = CONFIG_MV_ETH_NFP_FIB_LEARN_DEF;


int nfp_fib_is_learning_enabled(void)
{
	return nfp_fib_learn_en;
}

int nfp_fib_learn_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_fib_learn_en = en;
	return MV_OK;
}

int nfp_fib_is_aging_enabled(void)
{
	return nfp_fib_age_en;
}

int nfp_fib_age_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_fib_age_en = en;
	return MV_OK;
}

#ifdef CONFIG_IPV6
int nfp_fib6_learn_en = CONFIG_MV_ETH_NFP_FIB_LEARN_DEF;
int nfp_fib6_age_en = CONFIG_MV_ETH_NFP_FIB_LEARN_DEF;

void nfp_fib6_sync(void);

int nfp_fib6_is_learning_enabled(void)
{
	return nfp_fib6_learn_en;
}

int nfp_fib6_learn_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_fib6_learn_en = en;
	return MV_OK;
}

int nfp_fib6_is_aging_enabled(void)
{
	return nfp_fib6_age_en;
}

int nfp_fib6_age_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_fib6_age_en = en;
	return MV_OK;
}
#endif /* CONFIG_IPV6 */

#endif /* NFP_FIB_LEARN */

#ifdef NFP_PPP_LEARN
int nfp_ppp_learn_en = CONFIG_MV_ETH_NFP_PPP_LEARN_DEF;

int nfp_ppp_is_learning_enabled(void)
{
	return nfp_ppp_learn_en;
}

int nfp_ppp_learn_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_ppp_learn_en = en;
	return MV_OK;
}
#endif /* NFP_PPP_LEARN */

#ifdef NFP_CT_LEARN
int nfp_ct_learn_en = CONFIG_MV_ETH_NFP_CT_LEARN_DEF;
int nfp_ct_age_en = CONFIG_MV_ETH_NFP_CT_LEARN_DEF;

void nfp_ct_sync(int family);

int nfp_ct_is_learning_enabled(void)
{
	return nfp_ct_learn_en;
}

int nfp_ct_learn_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_ct_learn_en = en;
	return MV_OK;
}

int nfp_ct_is_aging_enabled(void)
{
	return nfp_ct_age_en;
}

int nfp_ct_age_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_ct_age_en = en;
	return MV_OK;
}

#ifdef CONFIG_IPV6
int nfp_ct6_learn_en = CONFIG_MV_ETH_NFP_CT_LEARN_DEF;
int nfp_ct6_age_en = CONFIG_MV_ETH_NFP_CT_LEARN_DEF;

int nfp_ct6_is_learning_enabled(void)
{
	return nfp_ct6_learn_en;
}

int nfp_ct6_learn_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_ct6_learn_en = en;
	return MV_OK;
}

int nfp_ct6_is_aging_enabled(void)
{
	return nfp_ct6_age_en;
}

int nfp_ct6_age_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return MV_BAD_PARAM;
	}

	nfp_ct6_age_en = en;
	return MV_OK;
}
#endif /* CONFIG_IPV6 */

#endif /* NFP_CT_LEARN */


struct nfp_hook_stats {
	MV_U32 add;
	MV_U32 add_fail;
	MV_U32 del;
	MV_U32 del_fail;
	MV_U32 age;
	MV_U32 age_fail;
};

/* global learning - control all features learning
   when 0: no learning at all
   when 1: learning is enabled for features with local learn enable(ex. fib_learn_enable) */
int nfp_learn_en = 0;
int nfp_age_en = 0;

int nfp_is_learning_enabled(void)
{
	return nfp_learn_en;
}

int nfp_is_age_enabled(void)
{
	return nfp_age_en;
}

void nfp_hook_sync(void)
{
#ifdef NFP_CT_LEARN
       nfp_ct_sync(MV_INET);
#ifdef CONFIG_IPV6
       nfp_ct_sync(MV_INET6);
#endif /* CONFIG_IPV6 */
#endif /* NFP_CT_LEARN */

#ifdef NFP_VLAN_LEARN
	vlan_sync();
#endif

#ifdef NFP_FDB_LEARN
	fdb_sync();
#endif

#ifdef NFP_FIB_LEARN
	neigh_sync(MV_INET);
	nfp_fib_sync();
#ifdef CONFIG_IPV6
	neigh_sync(MV_INET6);
	nfp_fib6_sync();
#endif /* CONFIG_IPV6 */
#endif /* NFP_FIB_LEARN */
}

void nfp_learn_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return;
	}
	nfp_learn_en = en;
}

void nfp_age_enable(int en)
{
	if (en < 0 && en > 1) {
		printk(KERN_ERR	"%s Error: invalid param.\n", __func__);
		return;
	}
	nfp_age_en = en;
}

#ifdef NFP_PPP_LEARN
static MV_NFP_PPP_RULE_MAP **nfp_half_ppp_rules;
static int halfCurrentIndex = 0;
int nfp_hook_ppp_half_set(u16 sid, u32 chan, struct net_device *eth_dev, char *remoteMac)
{
	MV_RULE_PPP rule;
	int                 h_index = 0;
	MV_NFP_PPP_RULE_MAP *new_rule = NULL, *curr_rule = NULL;

	if (!nfp_ppp_learn_en || !nfp_learn_en)
		return 1;

	memset(&rule, 0, sizeof(MV_RULE_PPP));

	rule.sid  = MV_16BIT_BE(sid);
	rule.dev  = eth_dev;
	rule.chan = chan;
	if (remoteMac)
		memcpy(rule.da, remoteMac, MV_MAC_ADDR_SIZE);

	new_rule = kmalloc(sizeof(MV_NFP_PPP_RULE_MAP), GFP_ATOMIC);
	if (new_rule == NULL)
		return -ENOMEM;

	new_rule->rule_ppp = rule;
	h_index = (halfCurrentIndex & NFP_DEV_HASH_MASK);
	if (nfp_half_ppp_rules[h_index] == NULL) {
		nfp_half_ppp_rules[h_index] = new_rule;
	} else {
		/* insert at the end of the list */
		for (curr_rule = nfp_half_ppp_rules[h_index]; curr_rule->next != NULL; curr_rule = curr_rule->next)
			;
		curr_rule->next = new_rule;
	}
	halfCurrentIndex++;
	return 0;
}


int nfp_hook_ppp_complete(u32 chan, struct net_device *ppp_dev)
{
	int i = NFP_DEV_HASH_SZ;
	int res;
	int iif = ppp_dev->ifindex;
	MV_NFP_PPP_RULE_MAP *curr_ppp_rule = NULL;
	unsigned long flags;

	if (!nfp_ppp_learn_en || !nfp_learn_en)
		return 1;

	while (i--) {
		curr_ppp_rule = nfp_half_ppp_rules[i];
		if (curr_ppp_rule == NULL)
			continue;

		if ((curr_ppp_rule != NULL) && (curr_ppp_rule->rule_ppp.chan == chan)) {
			struct net_device *ethNetdev = (struct net_device *)curr_ppp_rule->rule_ppp.dev;
			res = nfp_mgr_if_virt_add(ethNetdev->ifindex, iif);
			if (res < 0) {
				NFP_DBG("nfp_mgr_if_virt_add() failed in %s\n", __func__);
				return -1;
			}
			write_lock_irqsave(&nfp_lock, flags);
			res = mvNfpPppAdd(iif, curr_ppp_rule->rule_ppp.sid, curr_ppp_rule->rule_ppp.da);
			write_unlock_irqrestore(&nfp_lock, flags);
			if (res != 0) {
				NFP_DBG("mvNfpPppAdd failed in %s\n", __func__);
				return -1;
			}
			/* keep original iif of ppp device */
			curr_ppp_rule->rule_ppp.iif = iif;

			/* iif can grow beyond NFP_DEV_HASH_SZ */
			iif = iif & NFP_DEV_HASH_MASK;
			break;
		} else {
			/* go to last entry in the list */
			for (curr_ppp_rule = nfp_half_ppp_rules[i]; curr_ppp_rule->next != NULL;
								curr_ppp_rule = curr_ppp_rule->next)
				;

			if ((curr_ppp_rule != NULL) && (curr_ppp_rule->rule_ppp.chan == chan)) {
				struct net_device *ethNetdev = (struct net_device *)curr_ppp_rule->rule_ppp.dev;
				res = nfp_mgr_if_virt_add(ethNetdev->ifindex, iif);
				if (res < 0) {
					NFP_DBG("nfp_mgr_if_virt_add() failed in %s\n", __func__);
					return -1;
				}
				write_lock_irqsave(&nfp_lock, flags);
				res = mvNfpPppAdd(iif, curr_ppp_rule->rule_ppp.sid, curr_ppp_rule->rule_ppp.da);
				write_unlock_irqrestore(&nfp_lock, flags);
				if (res != 0) {
					NFP_DBG("mvNfpPppAdd failed in %s\n", __func__);
					return -1;
				}
				/* keep original iif of ppp device */
				curr_ppp_rule->rule_ppp.iif = iif;

				/* iif can grow beyond NFP_DEV_HASH_SZ */
				iif = iif & NFP_DEV_HASH_MASK;

				break;
			}
		}	/* of else */
	} /* of while */
	return MV_OK;
}


int nfp_hook_ppp_info_del(u32 chan)
{
	int i = NFP_DEV_HASH_SZ;
	MV_NFP_PPP_RULE_MAP *curr_ppp_rule = NULL;
	unsigned long flags;
	MV_STATUS status;

        if (!nfp_ppp_learn_en || !nfp_learn_en)
                return 1;

	while (i--) {
		curr_ppp_rule = nfp_half_ppp_rules[i];
		if (curr_ppp_rule == NULL)
			continue;

		if ((curr_ppp_rule != NULL) && (curr_ppp_rule->rule_ppp.chan == chan)) {
			write_lock_irqsave(&nfp_lock, flags);
			status = mvNfpIfVirtUnmap(curr_ppp_rule->rule_ppp.iif);
			write_unlock_irqrestore(&nfp_lock, flags);
			if (status != MV_OK) {
				printk(KERN_ERR "%s Error: Can't unmap if_virt (%d) interface\n",
						__func__, curr_ppp_rule->rule_ppp.iif);
				return -1;
			}
			write_lock_irqsave(&nfp_lock, flags);
			status = mvNfpIfMapDelete(curr_ppp_rule->rule_ppp.iif);
			write_unlock_irqrestore(&nfp_lock, flags);
			if (status != MV_OK) {
				printk(KERN_ERR "%s Error: can't delete (%d) interface\n",
				__func__, curr_ppp_rule->rule_ppp.iif);
				return -1;
			}

			break;
		} else {
			/* go to last entry in the list */
			for (curr_ppp_rule = nfp_half_ppp_rules[i]; curr_ppp_rule->next != NULL;
								curr_ppp_rule = curr_ppp_rule->next)
				;
				if ((curr_ppp_rule != NULL) && (curr_ppp_rule->rule_ppp.chan == chan)) {
					write_lock_irqsave(&nfp_lock, flags);
					status = mvNfpIfVirtUnmap(curr_ppp_rule->rule_ppp.iif);
					write_unlock_irqrestore(&nfp_lock, flags);
					if (status != MV_OK) {
						printk(KERN_ERR "%s Error: Can't unmap if_virt (%d) interface\n",
								__func__, curr_ppp_rule->rule_ppp.iif);
						return -1;
					}
					write_lock_irqsave(&nfp_lock, flags);
					status = mvNfpIfMapDelete(curr_ppp_rule->rule_ppp.iif);
					write_unlock_irqrestore(&nfp_lock, flags);
					if (status != MV_OK) {
						printk(KERN_ERR "%s Error: can't delete (%d) interface\n",
						__func__, curr_ppp_rule->rule_ppp.iif);
						return -1;
					}

					break;
				}
		}	/* of else */
	} /* of while */
	return MV_OK;
}

#endif /* NFP_PPP_LEARN */

#if defined(NFP_FDB_LEARN)

/* Add FDB rule */
int nfp_hook_fdb_rule_add(int br_index, int if_index, u8 *mac, int is_local)
{
	int			err;

	if (!nfp_bridge_learn_en || !nfp_learn_en)
		return 1;

	NFP_DBG("%s: br_index=%d, if_index=%d, mac="MV_MACQUAD_FMT", is_local=%d\n",
			__func__, br_index, if_index, MV_MACQUAD(mac), is_local);

	if (is_local) {
		err = nfp_if_to_bridge_add(br_index, if_index);
		if (err)
			return 1;
	}
	err = nfp_fdb_rule_add(br_index, if_index, mac, is_local);

	return err;
}

/* Check age of FDB entry */
int nfp_hook_fdb_rule_age(int br_index, int if_index, u8 *mac)
{
	if (!nfp_bridge_age_en || !nfp_age_en)
		return 1;

	NFP_DBG("%s: br_index=%d, if_index=%d, mac="MV_MACQUAD_FMT"\n",
		__func__, br_index, if_index, MV_MACQUAD(mac));

	return nfp_fdb_rule_age(br_index, mac);
}

/* Delete FDB rule */
int nfp_hook_fdb_rule_del(int br_index, int if_index, u8 *mac)
{
	if (!nfp_bridge_learn_en || !nfp_learn_en)
		return 1;

	NFP_DBG("%s: br_index=%d, if_index=%d, mac="MV_MACQUAD_FMT"\n",
			__func__, br_index, if_index, MV_MACQUAD(mac));

	return nfp_fdb_rule_del(br_index, mac);
}

int nfp_hook_del_port_from_br(int bridge_if, int port_if)
{
	return nfp_if_to_bridge_del(bridge_if, port_if);
}

int nfp_hook_del_br(int ifindex)
{
	unsigned long flags;

	write_lock_irqsave(&nfp_lock, flags);
	mvNfpFdbFlushBridge(ifindex);
	write_unlock_irqrestore(&nfp_lock, flags);

	return nfp_mgr_if_unregister(ifindex);
}
#endif /* NFP_FDB_LEARN */

#if defined(NFP_VLAN_LEARN)
int nfp_hook_vlan_add(int vlan_index, struct net_device *dev, int real_if_index, int vlan_id)
{
	int err;

	if (!nfp_vlan_learn_en || !nfp_learn_en)
		return -EIO;

	NFP_DBG("%s: if_index=%d, name=%s, vlan_id=0x%03x, real_if_index=%d\n",
			__func__, vlan_index, dev->name, vlan_id, real_if_index);

	/* real_if_index must be bind to NFP */
	err = nfp_mgr_if_virt_add(real_if_index, vlan_index);
	if (err == MV_ALREADY_EXIST)
		return 0;
	if (err) {
		printk(KERN_ERR "%s error: Can't add virtual interface - vlan_index=%d, name=%s\n",
					__func__, vlan_index, dev->name);
		return -EIO;
	}

	err = nfp_vlan_vid_set(vlan_index, vlan_id);
	if (err) {
		printk(KERN_ERR "%s error: Can't set vlanId = %d for %s (%d) interface\n",
					__func__, vlan_id, dev->name, vlan_index);
		return -EIO;
	}
	err = nfp_vlan_tx_mode_set(vlan_index, MV_NFP_VLAN_TX_TAGGED);
	if (err) {
		printk(KERN_ERR "%s error: Can't set tx_mode = %d for %s (%d) interface\n",
					__func__, MV_NFP_VLAN_TX_TAGGED, dev->name, vlan_index);
		return -EIO;
	}
	return 0;
}

int nfp_hook_vlan_del(int if_index)
{
	int err;

	if (!nfp_vlan_learn_en || !nfp_learn_en)
		return -EIO;

	NFP_DBG(KERN_DEBUG "%s: if_index=%d\n", __func__, if_index);
	err = nfp_mgr_if_virt_del(if_index);
	if (err < 0) {
		printk(KERN_ERR "%s error: Can't delete interface %d\n", __func__, if_index);
		return -EIO;
	}
	return err;
}
#endif /* NFP_VLAN_LEARN */


#if defined(NFP_FIB_LEARN)
int nfp_hook_fib_rule_add(int family, u8 *src_l3, u8 *dst_l3, u8 *gw, int iif, int oif)
{
	int                 err;
	MV_NFP_NETDEV_MAP   *pdevOut = NULL;
	MV_NFP_NETDEV_MAP   *pdevIn  = NULL;

	if ((family == AF_INET && !nfp_fib_learn_en) || !nfp_learn_en)
		return 1;
#ifdef CONFIG_IPV6
	if ((family == AF_INET6 && !nfp_fib6_learn_en) || !nfp_learn_en)
		return 1;
#endif /* CONFIG_IPV6 */

	pdevIn = nfp_eth_dev_find(iif);
	if (pdevIn == NULL) {
		NFP_DBG("%s: iif=%d is invalid index\n", __func__, iif);
		return 1;
	}

	pdevOut = nfp_eth_dev_find(oif);
	if (pdevOut == NULL) {
		NFP_DBG("%s: oif=%d is invalid index\n", __func__, oif);
		return 1;
	}

	NFP_DBG("%s: %s - iif=%d, oif=%d\n",
			__func__, (family == MV_INET) ? "IPv4" : "IPv6", iif, oif);

	err = nfp_fib_rule_add(family, src_l3, dst_l3, gw, oif);

	return err;
}

int nfp_hook_fib_rule_del(int family, u8 *src_l3, u8 *dst_l3, int iif, int oif)
{
	int err;

	if ((family == AF_INET && !nfp_fib_learn_en) || !nfp_learn_en)
		return 1;
#ifdef CONFIG_IPV6
	if ((family == AF_INET6 && !nfp_fib6_learn_en) || !nfp_learn_en)
		return 1;
#endif /* CONFIG_IPV6 */

	NFP_DBG("%s: iif=%d, oif=%d\n", __func__, iif, oif);
	mvNfp2TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3);

	err = nfp_fib_rule_del(family, src_l3, dst_l3);
	return err;
}

int nfp_hook_fib_rule_age(int family, u8 *src_l3, u8 *dst_l3, int iif, int oif)
{

	if ((family == AF_INET && !nfp_fib_age_en) || !nfp_age_en)
		return 1;
#ifdef CONFIG_IPV6
	if ((family == AF_INET6 && !nfp_fib6_age_en) || !nfp_age_en)
		return 1;
#endif /* CONFIG_IPV6 */

	NFP_DBG("%s: iif=%d, oif=%d\n", __func__, iif, oif);
	mvNfp2TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3);

	return nfp_fib_rule_age(family, src_l3, dst_l3);
}

int nfp_hook_arp_add(int family, u8 *ip, u8 *mac, int if_index)
{
	MV_NFP_NETDEV_MAP *pdevOut = NULL;

	if ((family == AF_INET && !nfp_fib_learn_en) || !nfp_learn_en)
		return 1;
#ifdef CONFIG_IPV6
	if ((family == AF_INET6 && !nfp_fib6_learn_en) || !nfp_learn_en)
		return 1;
#endif /* CONFIG_IPV6 */

	pdevOut = nfp_eth_dev_find(if_index);
	if (pdevOut == NULL)
		return 1;

	NFP_DBG("%s: %s, if_index=%d, if_name=%s, "MV_MACQUAD_FMT"\n",
		__func__, (family == MV_INET) ? "IPv4" : "IPv6", if_index, pdevOut->dev->name, MV_MACQUAD(mac));

	return nfp_arp_add(family, ip, mac);
}

int nfp_hook_arp_is_confirmed(int family, u8 *ip)
{
	if ((family == AF_INET && !nfp_fib_age_en) || !nfp_age_en)
		return 1;
#ifdef CONFIG_IPV6
	if ((family == AF_INET6 && !nfp_fib6_age_en) || !nfp_age_en)
		return 1;
#endif /* CONFIG_IPV6 */

	return nfp_arp_age(family, ip);
}

/* Delete neighbour for IP */
int nfp_hook_arp_delete(int family, u8 *ip)
{
	if ((family == AF_INET && !nfp_fib_learn_en) || !nfp_learn_en)
		return 1;
#ifdef CONFIG_IPV6
	if ((family == AF_INET6 && !nfp_fib6_learn_en) || !nfp_learn_en)
		return 1;
#endif /* CONFIG_IPV6 */

	return nfp_arp_del(family, ip);
}
#endif /* NFP_FIB_LEARN */


#ifdef NFP_CT_LEARN
int nfp_hook_ct_filter_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int if_index, int mode)
{
	if ((family == AF_INET && !nfp_ct_learn_en) || !nfp_learn_en)
		return 1;
#ifdef CONFIG_IPV6
	if ((family == AF_INET6 && !nfp_ct6_learn_en) || !nfp_learn_en)
		return 1;
#endif /* CONFIG_IPV6 */

	if (!nfp_mgr_is_valid_index(if_index))
		return 1;

	return nfp_ct_filter_set(family, src_l3, dst_l3, sport, dport, proto, mode);
}

/* Delete a 5-tuple rule */
void nfp_hook_ct_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto)
{
	if ((family == AF_INET && !nfp_ct_learn_en) || !nfp_learn_en)
		return;
#ifdef CONFIG_IPV6
	if ((family == AF_INET6 && !nfp_ct6_learn_en) || !nfp_learn_en)
		return;
#endif /* CONFIG_IPV6 */

	nfp_ct_del(family, src_l3, dst_l3, sport, dport, proto);
}

/* rule aging */
int nfp_hook_ct_age(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto)
{
	if ((family == AF_INET && !nfp_ct_age_en) || !nfp_age_en)
		return 1;
#ifdef CONFIG_IPV6
	if ((family == AF_INET6 && !nfp_ct6_age_en) || !nfp_age_en)
		return 1;
#endif /* CONFIG_IPV6 */

	return nfp_ct_age(family, src_l3, dst_l3, sport, dport, proto);
}

#ifdef NFP_NAT

/* Add a 5-tuple NAT rule */
int nfp_hook_ct_nat_add(u32 sip, u32 dip, u16 sport, u16 dport, u8 proto,
			u32 new_sip, u32 new_dip, u16 new_sport, u16 new_dport,
			int if_index, int maniptype)
{
	int manip_flags = 0;

	if (!nfp_ct_learn_en || !nfp_learn_en)
		return 1;

	if (!nfp_mgr_is_valid_index(if_index))
		return 1;

	if (maniptype == IP_NAT_MANIP_SRC)
		manip_flags = MV_ETH_NFP_NAT_MANIP_SRC;
	else if (maniptype == IP_NAT_MANIP_DST)
		manip_flags = MV_ETH_NFP_NAT_MANIP_DST;

	return nfp_ct_nat_add(sip, dip, sport, dport, proto,
				new_sip, new_dip, new_sport, new_dport,
				manip_flags);
}

int move_nat_to_nfp(const struct nf_conntrack_tuple *tuple,
			const struct nf_conntrack_tuple *target,
			enum nf_nat_manip_type maniptype)
{
	int status = 0;

	status = nfp_hook_ct_nat_add(tuple->src.u3.ip, tuple->dst.u3.ip,
			ntohs(tuple->src.u.all), ntohs(tuple->dst.u.all),
			tuple->dst.protonum, target->src.u3.ip,
			target->dst.u3.ip, ntohs(target->src.u.all),
			ntohs(target->dst.u.all), tuple->ifindex, maniptype);

	if (status != MV_OK) {
		printk(KERN_ERR	"%s Error: nfp_hook_ct_nat_add failed.\n", __func__);
		return status;
	}

	if (tuple->dst.protonum == IPPROTO_UDP)
		nfp_ct_udp_csum_set(AF_INET, (u8 *)&(tuple->src.u3.ip), (u8 *)&(tuple->dst.u3.ip),
						ntohs(tuple->src.u.all), ntohs(tuple->dst.u.all), tuple->dst.protonum, tuple->udpCsum);

#ifdef NFP_CLASSIFY
	{
		u8 *src_l3 = (u8 *)&(tuple->src.u3.ip);
		u8 *dst_l3 = (u8 *)&(tuple->dst.u3.ip);
		u16 sport = ntohs(tuple->src.u.all);
		u16 dport = ntohs(tuple->dst.u.all);
		u8 proto = tuple->dst.protonum;
		int family = AF_INET; /* support NAT only for IPv4 */

		nfp_ct_classify_set(family, src_l3, dst_l3, sport, dport, proto, tuple->info);
	}
#endif /* NFP_CLASSIFY */

	return status;
}
#endif /* NFP_NAT */

int move_fwd_to_nfp(const struct nf_conntrack_tuple *tuple, int mode)
{
	int status = 0;
	int family = tuple->src.l3num;

	status = nfp_hook_ct_filter_set(family, (u8 *)&(tuple->src.u3.ip), (u8 *)&(tuple->dst.u3.ip),
					ntohs(tuple->src.u.all), ntohs(tuple->dst.u.all),
					tuple->dst.protonum, tuple->ifindex, mode);

	if (status != MV_OK) {
		printk(KERN_ERR	"%s Error: nfp_hook_ct_filter_set failed.\n", __func__);
		return status;
	}

#ifdef NFP_CLASSIFY
	{
		u8 *src_l3;
		u8 *dst_l3;
		u16 sport = ntohs(tuple->src.u.all);
		u16 dport = ntohs(tuple->dst.u.all);
		u8 proto = tuple->dst.protonum;

		if (family == AF_INET) {
			src_l3 = (u8 *)&(tuple->src.u3.ip);
			dst_l3 = (u8 *)&(tuple->dst.u3.ip);
		} else {
			src_l3 = (u8 *)&(tuple->src.u3.ip6);
			dst_l3 = (u8 *)&(tuple->dst.u3.ip6);
		}

		nfp_ct_classify_set(family, src_l3, dst_l3, sport, dport, proto, tuple->info);
	}
#endif /* NFP_CLASSIFY */

	return status;
}

#endif /* NFP_CT_LEARN */


#ifdef NFP_PPP_LEARN
/* Initialize Rule Database */
int nfp_ppp_db_init(void)
{
	NFP_DBG("FP_MGR: Initializing PPPoE Rule Database\n");

	nfp_half_ppp_rules = kmalloc(sizeof(MV_NFP_PPP_RULE_MAP *) * NFP_DEV_HASH_SZ, GFP_ATOMIC);
	if (nfp_half_ppp_rules == NULL)	{
		NFP_WARN("nfp_mgr_init: Error allocating memory for NFP Manager Half PPP rules Database\n");
		return -ENOMEM;
	}
	memset(nfp_half_ppp_rules, 0, sizeof(MV_NFP_PPP_RULE_MAP *) * NFP_DEV_HASH_SZ);

	return MV_OK;
}


/* Clear Fast Path PPP Rule Database */
int nfp_ppp_db_clear(void)
{
	unsigned long flags;

	write_lock_irqsave(&nfp_lock, flags);
	memset(nfp_half_ppp_rules, 0, sizeof(MV_NFP_PPP_RULE_MAP) * NFP_DEV_HASH_SZ);
	write_unlock_irqrestore(&nfp_lock, flags);

	return MV_OK;
}
#endif  /* NFP_PPP_LEARN */


#endif /* NFP_LEARN */


