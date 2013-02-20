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

int mv_ctrl_nfp_state = CONFIG_MV_ETH_NFP_DEF;
int mv_ctrl_nfp_mode = CONFIG_MV_ETH_NFP_MODE_DEF;
int mv_ctrl_nfp_data_path = CONFIG_MV_ETH_NFP_DEF;

struct mgr_stats {
	MV_U32 add;
	MV_U32 del;
	MV_U32 age;
};

#ifdef NFP_BRIDGE
struct mgr_br_stats {
	MV_U32 br_if_add;
	MV_U32 br_if_del;
	MV_U32 br_rule_add;
	MV_U32 br_rule_del;
	MV_U32 br_rule_age;
	MV_U32 br_vpri_set;
	MV_U32 br_vpri_del;
	MV_U32 br_txq_set;
	MV_U32 br_txq_del;
	MV_U32 br_txp_set;
	MV_U32 br_txp_del;
	MV_U32 br_mh_set;
	MV_U32 br_mh_del;
};

static struct mgr_br_stats br_stats;
#endif /* NFP_BRIDGE */

#ifdef NFP_CT
struct mgr_ct_stats {
	MV_U32 ct_nat_add;
	MV_U32 ct_drop_set;
	MV_U32 ct_forward_set;
	MV_U32 ct_rate_limit_set;
	MV_U32 ct_rate_limit_del;
	MV_U32 ct_dscp_set;
	MV_U32 ct_dscp_del;
	MV_U32 ct_vpri_set;
	MV_U32 ct_vpri_del;
	MV_U32 ct_txq_set;
	MV_U32 ct_txq_del;
	MV_U32 ct_txp_set;
	MV_U32 ct_txp_del;
	MV_U32 ct_mh_set;
	MV_U32 ct_mh_del;
	MV_U32 ct_del;
	MV_U32 ct_age;
};

static struct mgr_ct_stats ct_stats;
#endif /* NFP_CT */

#ifdef NFP_HWF
typedef struct nfp_hwf_map {
	int txp;
	int txq;

} NFP_HWF_MAP;

static NFP_HWF_MAP	nfp_hwf_txq_map[MV_ETH_MAX_PORTS][MV_ETH_MAX_PORTS];
static void     	nfp_fib_rule_hwf_set(int in_port, int out_port,	NFP_RULE_FIB *rule);
#endif /* NFP_HWF */


static MV_NFP_NETDEV_MAP **nfp_net_devs;
rwlock_t nfp_lock;


#ifdef NFP_FIB
static struct mgr_stats arp_stats;
static struct mgr_stats fib_stats;
#endif /* NFP_FIB */

static int nfp_mgr_notifier_event(struct notifier_block *this,
			      unsigned long event, void *ptr);

static struct notifier_block nfp_mgr_notifier = {
	.notifier_call = nfp_mgr_notifier_event,
};

static char *nfp_mgr_if_type_str(MV_NFP_IF_TYPE if_type)
{
	char *name;

	switch (if_type) {
	case MV_NFP_IF_INT:
		name = "Marvell";
		break;
	case MV_NFP_IF_EXT:
		name = "External";
		break;
	case MV_NFP_IF_BRG:
		name = "Bridge";
		break;
	case MV_NFP_IF_VIRT:
		name = "Virtual";
	default:
		name = "Unknown";
	}
	return name;
}

int __init nfp_mgr_init(void)
{
	nfp_net_devs = kmalloc(sizeof(MV_NFP_NETDEV_MAP *) * NFP_DEV_HASH_SZ, GFP_ATOMIC);
	if (nfp_net_devs == NULL) {
		printk(KERN_ERR "nfp_mgr_init: Error allocating memory for NFP Manager Interface Database\n");
		return -ENOMEM;
	}
	memset(nfp_net_devs, 0, sizeof(MV_NFP_NETDEV_MAP *) * NFP_DEV_HASH_SZ);

#ifdef NFP_BRIDGE
	memset(&br_stats,  0, sizeof(struct mgr_br_stats));
#endif /* NFP_BRIDGE */

#ifdef NFP_FIB
	memset(&arp_stats, 0, sizeof(struct mgr_stats));
	memset(&fib_stats, 0, sizeof(struct mgr_stats));
#endif /* NFP_FIB */

#ifdef NFP_CT
	memset(&ct_stats,  0, sizeof(struct mgr_ct_stats));
#endif /* NFP_CT */

#ifdef NFP_HWF
	memset(nfp_hwf_txq_map, 0xFF, sizeof(nfp_hwf_txq_map));
#endif /* NFP_HWF */

	rwlock_init(&nfp_lock);

	mvNfpInit();

#ifdef NFP_BRIDGE
#ifdef NFP_FDB_MODE
	mvNfpFdbInit();
#else
	mvNfpBridgeInit();
#endif /* NFP_FDB_MODE */
#endif /* NFP_BRIDGE */

#ifdef NFP_FIB
	mvNfpArpInit();
	mvNfpFibInit();
#endif /* NFP_FIB */

#ifdef NFP_CT
	mvNfpCtInit();
#endif /* NFP_CT */

#ifdef NFP_PNC
	mvNfpPncInit();
#endif

#ifdef NFP_PMT
	mvNfpPmtInit();
#endif /* NFP_PMT */

	if (register_netdevice_notifier(&nfp_mgr_notifier) < 0)
		printk(KERN_ERR "%s: register_netdevice_notifier() failed\n", __func__);

    return 0;
}

module_init(nfp_mgr_init);


void nfp_eth_dev_db_clear(void)
{
	int                 i = 0;
	MV_NFP_NETDEV_MAP   *curr_dev = NULL, *tmp_dev = NULL;

	for (i = 0; i < NFP_DEV_HASH_SZ; i++) {
		curr_dev = nfp_net_devs[i];
		while (curr_dev != NULL) {
			tmp_dev = curr_dev;
			curr_dev = curr_dev->next;
			kfree(tmp_dev);
		}
		nfp_net_devs[i] = NULL;
	}
}


static void __exit nfp_mgr_exit(void)
{
	nfp_eth_dev_db_clear();
	kfree(nfp_net_devs);

#ifdef NFP_FIB
	mvNfpFibClean();
	mvNfpFibDestroy();
#endif

#ifdef NFP_CT
	mvNfpCtClean(MV_INET);
#ifdef CONFIG_IPV6
	mvNfpCtClean(MV_INET6);
#endif /* CONFIG_IPV6 */
	mvNfpCtDestroy();
#endif

#ifdef NFP_PNC
	mvNfpPncDestroy();
#endif
}

module_exit(nfp_mgr_exit);

MV_NFP_NETDEV_MAP *nfp_eth_dev_find(int if_index)
{
	int                 h_index = 0;
	MV_NFP_NETDEV_MAP   *curr_dev = NULL;

	/* sanity checks */
	if ((nfp_net_devs == NULL) || (if_index <= 0))
		return NULL;

	h_index = (if_index & NFP_DEV_HASH_MASK);

	for (curr_dev = nfp_net_devs[h_index]; curr_dev != NULL; curr_dev = curr_dev->next) {
		if (curr_dev->if_index == if_index)
			return curr_dev;
	}
	return NULL;
}

int nfp_mgr_is_valid_index(int if_index)
{
	if (nfp_eth_dev_find(if_index) != NULL)
		return 1;

	return 0;
}


void nfp_mgr_clean(void)
{
#ifdef NFP_BRIDGE
	nfp_bridge_clean();
#endif

#ifdef NFP_FIB
	nfp_clean_fib(MV_INET);
	nfp_clean_arp(MV_INET);
	nfp_clean_fib(MV_INET6);
	nfp_clean_arp(MV_INET6);
#endif

#ifdef NFP_CT
	nfp_ct_clean(MV_INET);
	nfp_ct_clean(MV_INET6);
#endif
}

/* Enable/Disable NFP processing for each interface */
void nfp_mgr_enable_interfaces(int en)
{
	int i;
	MV_NFP_NETDEV_MAP *curr_dev = NULL;

	for (i = 0; i < NFP_DEV_HASH_SZ; i++) {
		curr_dev = nfp_net_devs[i];
		while (curr_dev != NULL) {
			if (curr_dev->if_type == MV_NFP_IF_INT)
				mv_eth_ctrl_nfp(curr_dev->dev, en);

			curr_dev = curr_dev->next;
		}
	}
}

/* Set NFP mode 	 	*/
/* mode - NFP operation mode:	*/
/* 	0 - 2 tuple mode 	*/
/* 	1 - 5 tuple mode 	*/
int nfp_mgr_mode_set(int mode)
{
	if (mode == 0)
		mv_ctrl_nfp_mode = MV_NFP_2_TUPLE;
	else if (mode == 1)
		mv_ctrl_nfp_mode = MV_NFP_5_TUPLE;
	else {
		printk(KERN_ERR "%s Error: invalid NFP mode (%d)\n", __func__, mode);
		return 1;
	}

	if (mv_ctrl_nfp_state == MV_NFP_ON)
		mvNfpModeSet(mv_ctrl_nfp_mode);

	return 0;
}

/* Get NFP mode					*/
/* return: 					*/
/*	MV_NFP_2_TUPLE - for 2 tuple mode 	*/
/*	MV_NFP_5_TUPLE - for 5 tuple mode 	*/
int nfp_mgr_mode_get(void)
{
	return mv_ctrl_nfp_mode;
}

/* Get NFP state			*/
/* return: 				*/
/*	MV_NFP_ON  - nfp is off 	*/
/*	MV_NFP_OFF - nfp is on	 	*/
int nfp_mgr_state_get(void)
{
	return mv_ctrl_nfp_state;
}

/* Enable/Disable NFP packet processing							*/
/* en - operation(enable/disable)							*/
/*	0 - disable NFP packet processing						*/
/*	1 - enable NFP packet processing (only if NFP is ON, via nfp_mgr_enable())	*/
int nfp_mgr_data_path_enable(int en)
{
	if (en < 0 || en > 1) {
		printk(KERN_ERR "%s Error: invalid NFP mode (%d)\n", __func__, en);
		return 1;
	}
	/* can't enable data path when nfp is off */
	if (en && mv_ctrl_nfp_state == MV_NFP_OFF) {
		printk(KERN_ERR "%s Error: NFP state is off\n", __func__);
		return 1;
	}

	mv_ctrl_nfp_data_path = en;
	nfp_mgr_enable_interfaces(en);
	return 0;
}

/* Turn on/off NFP						*/
/* en - operation(on/off):					*/
/*	0 - NFP is off						*/
/*		disable learn, disable data path, clean all 	*/
/*	1 - NFP is on						*/
/*		enable learn, enable data path, sync all 	*/
int nfp_mgr_enable(int en)
{
	int mode;

	if (en < 0 || en > 1) {
		printk(KERN_ERR "%s Error: invalid NFP mode (%d)\n", __func__, en);
		return 1;
	}

	if (en) {
		printk(KERN_ERR "NFP state: ON\n");
		printk(KERN_ERR "Enabling NFP in %d tuple mode\n", (mv_ctrl_nfp_mode == MV_NFP_2_TUPLE) ? 2 : 5);
		mv_ctrl_nfp_state = MV_NFP_ON;
		mode = mv_ctrl_nfp_mode;
	} else {
		printk(KERN_ERR "NFP state: OFF\n");
		mv_ctrl_nfp_state = MV_NFP_OFF;
		mode = MV_NFP_DISABLED;
	}

	mvNfpModeSet(mode);

	nfp_learn_enable(en);
	nfp_age_enable(en);
	nfp_mgr_data_path_enable(en);
	if (en)
		nfp_hook_sync();
	else
		nfp_mgr_clean();

	return 0;
}

void nfp_mgr_status(void)
{
	if (mv_ctrl_nfp_state > 0)
		printk(KERN_ERR "NFP state: ON\n");
	else
		printk(KERN_ERR "NFP state: OFF\n");

	printk(KERN_ERR "NFP mode: %d tuple mode\n", (mv_ctrl_nfp_mode == MV_NFP_2_TUPLE) ? 2 : 5);

	printk(KERN_ERR "NFP data path: %s\n", (mv_ctrl_nfp_data_path == 0) ? "disabled" : "enabled");

	printk(KERN_ERR "NFP learning   - %s\n", nfp_is_learning_enabled() ? "enabled" : "disabled");
	printk(KERN_ERR "NFP aging      - %s\n\n", nfp_is_age_enabled() ? "enabled" : "disabled");

	printk(KERN_ERR "Per feature learning / aging:\n");
#ifdef NFP_FIB_LEARN
	printk(KERN_ERR "NFP IPv4 routing learning   - %s\n", nfp_fib_is_learning_enabled() ? "enabled" : "disabled");
	printk(KERN_ERR "NFP IPv4 routing aging      - %s\n", nfp_fib_is_aging_enabled() ? "enabled" : "disabled");
#ifdef CONFIG_IPV6
	printk(KERN_ERR "NFP IPv6 routing learning   - %s\n", nfp_fib6_is_learning_enabled() ? "enabled" : "disabled");
	printk(KERN_ERR "NFP IPv6 routing aging      - %s\n", nfp_fib6_is_aging_enabled() ? "enabled" : "disabled");
#endif /* CONFIG_IPV6 */
#endif /* NFP_FIB_LEARN */

#ifdef NFP_CT_LEARN
	printk(KERN_ERR "NFP IPv4 5 tuple learning   - %s\n", nfp_ct_is_learning_enabled() ? "enabled" : "disabled");
	printk(KERN_ERR "NFP IPv4 5 tuple aging      - %s\n", nfp_ct_is_aging_enabled() ? "enabled" : "disabled");
#ifdef CONFIG_IPV6
	printk(KERN_ERR "NFP IPv6 5 tuple learning   - %s\n", nfp_ct6_is_learning_enabled() ? "enabled" : "disabled");
	printk(KERN_ERR "NFP IPv6 5 tuple aging      - %s\n", nfp_ct6_is_aging_enabled() ? "enabled" : "disabled");
#endif /* CONFIG_IPV6 */
#endif /* NFP_CT_LEARN */

#ifdef NFP_FDB_LEARN
	printk(KERN_ERR "NFP bridging learning   - %s\n", nfp_bridge_is_learning_enabled() ? "enabled" : "disabled");
	printk(KERN_ERR "NFP bridging aging      - %s\n", nfp_bridge_is_aging_enabled() ? "enabled" : "disabled");
#endif /* NFP_FDB_LEARN */

#ifdef NFP_VLAN_LEARN
	printk(KERN_ERR "NFP vlan learning   - %s\n", nfp_vlan_is_learning_enabled() ? "enabled" : "disabled");
#endif /* NFP_VLAN_LEARN */

#ifdef NFP_PPP_LEARN
	printk(KERN_ERR "NFP pppoe learning   - %s\n", nfp_ppp_is_learning_enabled() ? "enabled" : "disabled");
#endif /* NFP_PPP_LEARN */
}

/* Register a network interface that works with NFP */
static int nfp_mgr_if_register(int if_index, MV_NFP_IF_TYPE if_type)
{
	int			h_index = 0;
	struct net_device	*dev;
	MV_STATUS		status;
	MV_NFP_NETDEV_MAP	*new_dev = NULL;
	NFP_IF_MAP              ifMap;
	unsigned long		flags;

	/* sanity checks */
	if ((nfp_net_devs == NULL)	||
	    (if_index <= 0)		||
	    (nfp_eth_dev_find(if_index) != NULL)) {
		printk(KERN_ERR "%s Error: invalid if_index (%d) or interface is already registered\n", __func__, if_index);
		return -1;
	}
	memset(&ifMap, 0, sizeof(ifMap));

	ifMap.ifIdx = if_index;
	dev = dev_get_by_index(&init_net, if_index);
	if (dev == NULL) {
		printk(KERN_ERR "%s Error: Can't get netdevice for if_index = %d\n", __func__, if_index);
		return 1;
	}
	/* Virtual interface can't be registered as INT or EXT */
	if ((if_type == MV_NFP_IF_INT) || (if_type == MV_NFP_IF_EXT)) {
		if (dev->tx_queue_len == 0) {
			printk(KERN_ERR "%s Error: virtual interface %s can't be bind to NFP\n",
				__func__, dev->name);
			return 1;
		}
		/* Default VLAN modes for real interfaces: RX - drop unknown, TX - send untagged */
		ifMap.flags |= (NFP_F_MAP_VLAN_RX_DROP_UNKNOWN | NFP_F_MAP_VLAN_TX_UNTAGGED);
	}

	ifMap.dev = dev;

	h_index = (if_index & NFP_DEV_HASH_MASK);

	new_dev = kmalloc(sizeof(MV_NFP_NETDEV_MAP), GFP_ATOMIC);
	if (new_dev == NULL) {
		dev_put(dev);
		return -ENOMEM;
	}

	new_dev->if_type  = if_type;
	new_dev->if_index = if_index;
	new_dev->dev      = dev;

	if (if_type == MV_NFP_IF_INT) {
		struct eth_port   *pp = MV_ETH_PRIV(new_dev->dev);
		struct eth_netdev *dev_priv;

		new_dev->port = pp->port;
		ifMap.port = pp->port;
		ifMap.flags |= NFP_F_MAP_INT;

		if (pp->flags & MV_ETH_F_SWITCH) {
			/* GbE port connected to switch */
			dev_priv = MV_DEV_PRIV(new_dev->dev);
			ifMap.switchGroup = (MV_U8)dev_priv->group;
			ifMap.flags |= (NFP_F_MAP_SWITCH_PORT | NFP_F_MAP_TX_MH);
			ifMap.txMh = dev_priv->tx_vlan_mh;
		} else if (pp->flags & MV_ETH_F_MH) {
			ifMap.txMh = pp->tx_mh;
			ifMap.flags |= NFP_F_MAP_TX_MH;
		}
	} else {
		new_dev->port = NFP_INVALID_PORT;
		if (if_type == MV_NFP_IF_BRG)
			ifMap.flags |= NFP_F_MAP_BRIDGE;
		else if (if_type == MV_NFP_IF_EXT)
			ifMap.flags |= NFP_F_MAP_EXT;
	}
	memcpy(ifMap.mac, dev->dev_addr, MV_MAC_ADDR_SIZE);
	ifMap.mtu = dev->mtu;
	strncpy(ifMap.name, dev->name, sizeof(ifMap.name));

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpIfMapCreate(&ifMap);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status != MV_OK)
		kfree(new_dev);

	dev_put(dev);
	new_dev->next = nfp_net_devs[h_index];
	nfp_net_devs[h_index] = new_dev;

	return status;
}

int nfp_mgr_if_unregister(int if_index)
{
	int                 h_index = 0;
	MV_NFP_NETDEV_MAP   *curr_dev = NULL, *prev_dev = NULL;
	NFP_IF_MAP *ifMap   = NULL;
	unsigned long flags;
	MV_STATUS status;

	if ((nfp_net_devs == NULL) || (if_index <= 0) || (nfp_eth_dev_find(if_index) == NULL)) {
		printk(KERN_ERR "%s Error: invalid if_index (%d) or interface is not registered\n",
				__func__, if_index);
		return -1;
	}

	read_lock_irqsave(&nfp_lock, flags);
	ifMap = mvNfpIfMapGet(if_index);
	read_unlock_irqrestore(&nfp_lock, flags);

	if (ifMap->virtIf) {
		printk(KERN_ERR "%s Error: cannot unregister: virtIf (%s) was created on this interface\n",
			__func__, ifMap->virtIf->name);
		return -1;
	}
	if (ifMap->flags & NFP_F_MAP_BRIDGE_PORT) {
		printk(KERN_ERR "%s Error: cannot unregister: the interface was added to a bridge\n", __func__);
		return -1;
	}

	h_index = (if_index & NFP_DEV_HASH_MASK);

	for (curr_dev = nfp_net_devs[h_index]; curr_dev != NULL; prev_dev = curr_dev, curr_dev = curr_dev->next) {
		if (curr_dev->if_index == if_index) {
			if (prev_dev == NULL)
				nfp_net_devs[h_index] = curr_dev->next;
			else
				prev_dev->next = curr_dev->next;

			kfree(curr_dev);
			break;
		}
	}
	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpIfMapDelete(if_index);
	write_unlock_irqrestore(&nfp_lock, flags);

	if (status != MV_OK) {
		printk(KERN_ERR "%s Error: can't delete (%d) interface\n",
				__func__, if_index);
		return -1;
	}

	return 0;
}


void nfp_mgr_flush(int ifindex)
{
	unsigned long flags;

	write_lock_irqsave(&nfp_lock, flags);

#ifdef NFP_FIB
	mvNfpFlushFib(ifindex);
#endif

#ifdef NFP_BRIDGE
#ifdef NFP_FDB_MODE
	mvNfpFdbFlushBridge(ifindex);
#else
	mvNfpFlushBridge(ifindex);
#endif /* NFP_FDB_MODE */
#endif /* NFP_BRIDGE */

	write_unlock_irqrestore(&nfp_lock, flags);
}

/* Bind real interface to NFP processing */
int nfp_mgr_if_bind(int if_index)
{
	MV_STATUS status = MV_FAIL;

	/* sanity checks */
	if ((nfp_net_devs == NULL)	||
	    (if_index <= 0)		||
	    (nfp_eth_dev_find(if_index) != NULL)) {
		printk(KERN_ERR "%s Error: invalid if_index (%d) or interface is already registered\n",
				__func__, if_index);
		return -1;
	}
	/* Check that this is Marvell network interface */
	if (mv_eth_netdev_find(if_index)) {
		status = nfp_mgr_if_register(if_index, MV_NFP_IF_INT);
		if (status == MV_OK) {
			MV_NFP_NETDEV_MAP	*pdevMap;

			pdevMap = nfp_eth_dev_find(if_index);

			mv_eth_ctrl_nfp(pdevMap->dev, mv_ctrl_nfp_data_path);
		}
	} else {
#ifdef CONFIG_MV_ETH_NFP_EXT
		int port;
		unsigned long flags;

		status = nfp_mgr_if_register(if_index, MV_NFP_IF_EXT);
		if (status != MV_OK) {
			printk(KERN_ERR "%s: Can't bind external interface %d\n", __func__, if_index);
		} else {
			read_lock_irqsave(&nfp_lock, flags);
			status = mvNfpIfMapPortGet(if_index, &port);
			read_unlock_irqrestore(&nfp_lock, flags);
			printk(KERN_INFO "%s: external interface %d is bound to NFP as port %d\n", __func__, if_index, port);
		}
#else
		printk(KERN_ERR "%s: NFP doesn't support external interfaces (if_index=%d)\n", __func__, if_index);
#endif /* CONFIG_MV_ETH_NFP_EXT*/
	}
	return status;
}

int nfp_mgr_if_unbind(int if_index)
{
	int                 h_index = 0;
	MV_NFP_NETDEV_MAP   *curr_dev = NULL, *prev_dev = NULL;
	NFP_IF_MAP *ifMap = NULL;
	unsigned long flags;
	MV_STATUS     status;

	if ((nfp_net_devs == NULL) || (if_index <= 0) || (nfp_eth_dev_find(if_index) == NULL)) {
		printk(KERN_ERR "%s Error: invalid if_index (%d) or interface is not registered\n",
				__func__, if_index);
		return -1;
	}

	nfp_mgr_flush(if_index);

	read_lock_irqsave(&nfp_lock, flags);
	ifMap = mvNfpIfMapGet(if_index);
	read_unlock_irqrestore(&nfp_lock, flags);

	if (ifMap->virtIf) {
		printk(KERN_ERR "%s Error: cannot unbind: virtIf (%s) was created on this real interface\n",
			__func__, ifMap->virtIf->name);
		return -1;
	}
	if (ifMap->flags & NFP_F_MAP_BRIDGE_PORT) {
		printk(KERN_ERR "%s Error: cannot unbind: the real interface was added to a bridge\n",
			__func__);
		return -1;
	}

	h_index = (if_index & NFP_DEV_HASH_MASK);

	for (curr_dev = nfp_net_devs[h_index]; curr_dev != NULL; prev_dev = curr_dev, curr_dev = curr_dev->next) {
		if (curr_dev->if_index == if_index) {
			if (prev_dev == NULL)
				nfp_net_devs[h_index] = curr_dev->next;
			else
				prev_dev->next = curr_dev->next;

			kfree(curr_dev);
			break;
		}
	}
	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpIfMapDelete(if_index);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status != MV_OK) {
		printk(KERN_ERR "%s Error: can't delete (%d) interface\n",
				__func__, if_index);
		return -1;
	}

	return 0;
}

int nfp_mgr_if_virt_add(int if_index, int if_virt)
{
	int err;
	unsigned long flags;
	MV_STATUS     status;

	if (nfp_eth_dev_find(if_index) == NULL) {
		printk(KERN_ERR "%s Error: if_index (%d) interface is not bound to NFP\n",
				__func__, if_index);
		return -1;
	}
	if (nfp_eth_dev_find(if_virt) != NULL) {
		NFP_WARN("%s Warning: if_virt (%d) interface is already bound\n",
				__func__, if_virt);
		return MV_ALREADY_EXIST;
	}
	/* Register virtual interface to NFP */
	err = nfp_mgr_if_register(if_virt, MV_NFP_IF_VIRT);
	if (err) {
		printk(KERN_ERR "%s error: Can't register virtual interface if_virt=%d\n",
				__func__, if_virt);
		return 1;
	}
	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpIfVirtMap(if_index, if_virt);
	write_unlock_irqrestore(&nfp_lock, flags);
	return status;
}

int nfp_mgr_if_virt_del(int if_virt)
{
	unsigned long flags;
	MV_STATUS     status;

	if (nfp_eth_dev_find(if_virt) == NULL) {
		printk(KERN_ERR "%s Error: if_virt (%d) interface is not bound to NFP\n",
				__func__, if_virt);
		return -1;
	}
	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpIfVirtUnmap(if_virt);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status != MV_OK) {
		printk(KERN_ERR "%s Error: Can't unmap if_virt (%d) interface\n",
				__func__, if_virt);
		return -1;
	}
	nfp_mgr_flush(if_virt);
	return nfp_mgr_if_unregister(if_virt);
}

int nfp_mgr_if_mtu_set(int if_index, int mtu)
{
	unsigned long flags;
	MV_STATUS     status;

	if (nfp_eth_dev_find(if_index) == NULL) {
		NFP_DBG("%s Error: if_index (%d) interface is not bound to NFP\n",
				__func__, if_index);
		return -1;
	}
	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpIfMapMtuUpdate(if_index, mtu);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status != MV_OK)
		return -1;

	return 0;
}

static int nfp_mgr_notifier_event(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	struct net_device *dev = (struct net_device *)ptr;
	MV_NFP_NETDEV_MAP *currdev;

	if (dev == NULL)
		return NOTIFY_DONE;

	currdev = nfp_eth_dev_find(dev->ifindex);

	if (!currdev)
		return NOTIFY_DONE;

	switch (event) {
	case NETDEV_UP:
		if (nfp_mgr_if_mtu_set(dev->ifindex, dev->mtu) < 0)
			printk(KERN_ERR "%s: nfp_mgr_if_mtu_set() failed\n", __func__);
		break;
	case NETDEV_CHANGEMTU:
		if (nfp_mgr_if_mtu_set(dev->ifindex, dev->mtu) < 0)
			printk(KERN_ERR "%s: nfp_mgr_if_mtu_set() failed\n", __func__);
		break;
	case NETDEV_CHANGEADDR:
		if (nfp_mgr_if_mac_set(dev->ifindex, dev->dev_addr) < 0)
			printk(KERN_ERR "%s: nfp_mgr_if_mac_set() failed\n", __func__);
		break;
	}
	return NOTIFY_DONE;
}

int nfp_mgr_if_mac_set(int if_index, u8 *mac)
{
	unsigned long flags;
	MV_STATUS     status;

	if (nfp_eth_dev_find(if_index) == NULL) {
		NFP_DBG("%s Error: if_index (%d) interface is not bound to NFP\n",
				__func__, if_index);
		return -1;
	}
	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpIfMapMacUpdate(if_index, mac);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status != MV_OK)
		return -1;

	return 0;
}

#ifdef NFP_HWF
int	nfp_hwf_txq_set(int rxp, int port, int txp, int txq)
{
	if (mvNetaPortCheck(rxp))
		return 1;

	if (mvNetaTxpCheck(port, txp))
		return 1;

	if (mvNetaMaxCheck(txq, CONFIG_MV_ETH_TXQ))
		return 1;

	nfp_hwf_txq_map[rxp][port].txp = txp;
	nfp_hwf_txq_map[rxp][port].txq = txq;

	return 0;
}

int 	nfp_hwf_txq_get(int rxp, int port, int *txp, int *txq)
{
	if (mvNetaPortCheck(rxp))
		return 1;

	if (mvNetaPortCheck(port))
		return 1;

	if (txp)
		*txp = nfp_hwf_txq_map[rxp][port].txp;

	if (txq)
		*txq = nfp_hwf_txq_map[rxp][port].txq;

	return 0;
}

static void nfp_fib_rule_hwf_set(int in_port, int out_port, NFP_RULE_FIB *rule)
{
	int hwf_txp, hwf_txq;

	if (nfp_hwf_txq_get(in_port, out_port, &hwf_txp, &hwf_txq))
		return;

	if ((hwf_txq != -1) && (hwf_txp != -1)) {
		rule->flags |= NFP_F_FIB_HWF;
		rule->hwf_txp = (MV_U8)hwf_txp;
		rule->hwf_txq = (MV_U8)hwf_txq;
	}
}
#endif /* NFP_HWF */


#ifdef NFP_FIB
int nfp_arp_add(int family, u8 *ip, u8 *mac)
{
	NFP_RULE_ARP  rule;
	unsigned long flags;
	MV_STATUS     status;

	memset(&rule, 0, sizeof(NFP_RULE_ARP));
	memcpy(rule.da, mac, MV_MAC_ADDR_SIZE);
	rule.family = family;
	l3_addr_copy(family, rule.nextHopL3, ip);

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpArpRuleAdd(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		arp_stats.add++;
	else {
		NFP_WARN("%s failed\n", __func__);
		mvNfpIpInfoPrint(NFP_WARN_PRINT, family, ip);
	}

	return status;
}

/* Delete neighbour for IP */
int nfp_arp_del(int family, u8 *ip)
{
	NFP_RULE_ARP	rule;
	unsigned long	flags;
	MV_STATUS	status;

	l3_addr_copy(family, rule.nextHopL3, ip);
	rule.family = family;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpArpRuleDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		arp_stats.del++;
	else {
		NFP_WARN("%s failed\n", __func__);
		mvNfpIpInfoPrint(NFP_WARN_PRINT, family, ip);
	}

	return status;
}

int nfp_arp_age(int family, u8 *ip)
{
	NFP_RULE_ARP	rule;
	unsigned long	flags;

	l3_addr_copy(family, rule.nextHopL3, ip);
	rule.family = family;
	read_lock_irqsave(&nfp_lock, flags);
	mvNfpArpRuleAge(&rule);
	read_unlock_irqrestore(&nfp_lock, flags);
	arp_stats.age++;

	return rule.age;
}

int nfp_fib_rule_add(int family, u8 *src_l3, u8 *dst_l3, u8 *gw, int oif)
{
	MV_STATUS		status;
	NFP_RULE_FIB		rule;
	struct net_device	*out_dev;
	unsigned long		flags;
	MV_NFP_NETDEV_MAP	*pdevOut;

	pdevOut = nfp_eth_dev_find(oif);
	if (pdevOut == NULL) {
		NFP_WARN("%s: oif=%d is invalid index\n", __func__, oif);
		return 1;
	}
	out_dev = pdevOut->dev;

	memset(&rule, 0, sizeof(NFP_RULE_FIB));
	rule.flags = NFP_F_FIB_ARP_INV;
	rule.family = family;
	l3_addr_copy(family, rule.srcL3, src_l3);
	l3_addr_copy(family, rule.dstL3, dst_l3);
	l3_addr_copy(family, rule.defGtwL3, gw);
	rule.oif = out_dev->ifindex;
	rule.flags |= NFP_F_FIB_ARP_INV;

#ifdef NFP_BRIDGE
	if (pdevOut->if_type == MV_NFP_IF_BRG)
		rule.flags |= NFP_F_FIB_BRIDGE_INV;
#endif /* NFP_BRIDGE */

#ifdef NFP_HWF
	/* enable PMT / HWF capabilities for IPv4 */
	if ((family == MV_INET)	&& (pdevOut->if_type == MV_NFP_IF_INT) && (pdevIn->if_type == MV_NFP_IF_INT))
		nfp_fib_rule_hwf_set(pdevIn->port, pdevOut->port, &rule);
#endif /* NFP_HWF */

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpFibRuleAdd(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		fib_stats.add++;
	} else {
		NFP_WARN("%s failed:\n", __func__);
		mvNfp2TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3);
	}

	return status;
}


int nfp_fib_rule_del(int family, u8 *src_l3, u8 *dst_l3)
{
	MV_STATUS	status;
	NFP_RULE_FIB	rule;
	unsigned long	flags;

	memset(&rule, 0, sizeof(NFP_RULE_FIB));
	rule.family = family;
	l3_addr_copy(family, rule.srcL3, src_l3);
	l3_addr_copy(family, rule.dstL3, dst_l3);

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpFibRuleDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		fib_stats.del++;
	} else {
		/* FIXME: hook should not call us in such case */
		NFP_WARN("%s failed:\n", __func__);
		mvNfp2TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3);
	}
	return status;
}

int nfp_fib_rule_age(int family, u8 *src_l3, u8 *dst_l3)
{
	NFP_RULE_FIB rule;
	unsigned long flags;
	MV_STATUS status;

	l3_addr_copy(family, rule.srcL3, src_l3);
	l3_addr_copy(family, rule.dstL3, dst_l3);
	rule.family = family;
	rule.age = 0;

	read_lock_irqsave(&nfp_lock, flags);
	status = mvNfpFibRuleAge(&rule);
	read_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		fib_stats.age++;
	} else {
		/* FIXME: hook should not call us in such case*/
		NFP_WARN("%s failed:\n", __func__);
		mvNfp2TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3);
	}
	return rule.age;
}
#endif /* NFP_FIB */

#ifdef NFP_VLAN
int nfp_vlan_vid_set(int if_index, u16 vid)
{
	MV_STATUS status;
	unsigned long flags;

	/* if_index must be bound to NFP */
	if (nfp_eth_dev_find(if_index) == NULL) {
		printk(KERN_ERR "%s Error: if_index (%d) interface is not bound to NFP\n",
				__func__, if_index);
		return -1;
	}
	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpVlanVidSet(if_index, vid);
	write_unlock_irqrestore(&nfp_lock, flags);

	return status;
}

int nfp_vlan_pvid_set(int if_index, u16 pvid)
{
	MV_STATUS status;
	unsigned long flags;

	/* if_index must be bound to NFP */
	if (nfp_eth_dev_find(if_index) == NULL) {
		printk(KERN_ERR "%s Error: if_index (%d) interface is not bound to NFP\n",
				__func__, if_index);
		return -1;
	}
	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpVlanPvidSet(if_index, pvid);
	write_unlock_irqrestore(&nfp_lock, flags);

	return status;
}

int nfp_vlan_rx_mode_set(int if_index, int mode)
{
	MV_U32 flags;
	unsigned long lock_flags;

	/* if_index must be bound to NFP */
	if (nfp_eth_dev_find(if_index) == NULL) {
		printk(KERN_ERR "%s Error: if_index (%d) interface is not bound to NFP\n",
				__func__, if_index);
		return -1;
	}
	switch (mode) {
	case MV_NFP_VLAN_RX_TRANSPARENT:
		flags = 0;
		break;
	case MV_NFP_VLAN_RX_DROP_UNTAGGED:
		flags = NFP_F_MAP_VLAN_RX_DROP_UNTAGGED;
		break;
	case MV_NFP_VLAN_RX_DROP_TAGGED:
		flags = NFP_F_MAP_VLAN_RX_DROP_TAGGED;
		break;
	case MV_NFP_VLAN_RX_DROP_UNKNOWN:
		flags = NFP_F_MAP_VLAN_RX_DROP_UNKNOWN;
		break;
	case MV_NFP_VLAN_RX_DROP_UNTAGGED_AND_UNKNOWN:
		flags = NFP_F_MAP_VLAN_RX_DROP_UNKNOWN | NFP_F_MAP_VLAN_RX_DROP_UNTAGGED;
		break;
	default:
		printk(KERN_ERR "%s Error: Unexpected VLAN RX mode = %d\n", __func__, mode);
		return -1;
	}
	write_lock_irqsave(&nfp_lock, lock_flags);
	mvNfpIfFlagsClear(if_index, NFP_F_MAP_VLAN_RX_FLAGS);
	mvNfpIfFlagsSet(if_index, flags);
	write_unlock_irqrestore(&nfp_lock, lock_flags);

	return 0;
}

int nfp_vlan_tx_mode_set(int if_index, int mode)
{
	MV_U32 flags;
	MV_STATUS status;
	MV_U16 vid;
	unsigned long lock_flags;

	/* if_index must be bound to NFP */
	if (nfp_eth_dev_find(if_index) == NULL) {
		printk(KERN_ERR "%s Error: if_index (%d) interface is not bound to NFP\n",
				__func__, if_index);
		return -1;
	}

	switch (mode) {
	case MV_NFP_VLAN_TX_TRANSPARENT:
		flags = 0;
		break;
	case MV_NFP_VLAN_TX_UNTAGGED:
		flags = NFP_F_MAP_VLAN_TX_UNTAGGED;
		break;
	case MV_NFP_VLAN_TX_TAGGED:
		read_lock_irqsave(&nfp_lock, lock_flags);
		status = mvNfpVlanVidGet(if_index, &vid);
		read_unlock_irqrestore(&nfp_lock, lock_flags);
		if ((status != MV_OK) || (vid == NFP_INVALID_VLAN)) {
			printk(KERN_ERR "%s Error: Cannot set mode to TX_TAGGED on an interface without a VID\n", __func__);
			return -1;
		} else {
			flags = NFP_F_MAP_VLAN_TX_TAGGED;
			break;
		}
	default:
		printk(KERN_ERR "%s Error: Unexpected VLAN TX mode = %d\n", __func__, mode);
		return -1;
	}
	write_lock_irqsave(&nfp_lock, lock_flags);
	mvNfpIfFlagsClear(if_index, NFP_F_MAP_VLAN_TX_FLAGS);
	mvNfpIfFlagsSet(if_index, flags);
	write_unlock_irqrestore(&nfp_lock, lock_flags);

	return 0;
}
#endif /* NFP_VLAN */

#ifdef NFP_NAT
/* Add / update a 5-tuple NAT rule */
int nfp_ct_nat_add(u32 sip, u32 dip, u16 sport, u16 dport, u8 proto,
		       u32 new_sip, u32 new_dip, u16 new_sport, u16 new_dport,
		       int manip_flags)
{
	NFP_RULE_CT rule;
	MV_STATUS status;
	unsigned long flags;

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = MV_INET;
	memcpy(rule.srcL3, (const MV_U8 *)&sip, MV_MAX_IPV4_ADDR_SIZE);
	memcpy(rule.dstL3, (const MV_U8 *)&dip, MV_MAX_IPV4_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;
	rule.new_sport = MV_16BIT_BE(new_sport);			/* store as BE 	*/
	rule.new_dport = MV_16BIT_BE(new_dport);			/* store as BE 	*/
	rule.new_sip = new_sip;
	rule.new_dip = new_dip;
	if (manip_flags & MV_ETH_NFP_NAT_MANIP_DST)
		rule.flags |= NFP_F_CT_DNAT;
	if (manip_flags & MV_ETH_NFP_NAT_MANIP_SRC)
		rule.flags |= NFP_F_CT_SNAT;

	NFP_DBG("%s: "MV_IPQUAD_FMT":%d->"MV_IPQUAD_FMT":%d, proto=%d, manip_flags=0x%X\n",
		__func__, MV_IPQUAD(((MV_U8 *)&sip)), sport, MV_IPQUAD(((MV_U8 *)&dip)), dport, proto, manip_flags);
	NFP_DBG("new: "MV_IPQUAD_FMT":%d-> "MV_IPQUAD_FMT":%d\n",
			MV_IPQUAD(((MV_U8 *)&new_sip)), new_sport, MV_IPQUAD(((MV_U8 *)&new_dip)), new_dport);

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtNatRuleAdd(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		ct_stats.ct_nat_add++;
	else
		NFP_WARN("%s failed: "MV_IPQUAD_FMT" 0x%04x->"MV_IPQUAD_FMT" 0x%04x, proto=%d, manip_flags=0x%X\n",
			__func__, MV_IPQUAD(((MV_U8 *)&sip)), sport, MV_IPQUAD(((MV_U8 *)&dip)), dport, proto, manip_flags);

	return status;
}
#endif /* NAT */

#ifdef NFP_CT
/* Delete an existing 5-tuple connection tracking rule.			*/
/* The rule will be deleted with all the configured information, e.g.	*/
/* NAPT, Drop/Forward, Classification, Rate Limit etc.			*/
void nfp_ct_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto)
{
	NFP_RULE_CT rule;
	unsigned long flags;
	MV_STATUS status;

	NFP_DBG("%s: \n", __func__);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtRuleDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
	    ct_stats.ct_del++;
	} else {
		NFP_WARN("%s failed, ", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}
}

/* Return a 5 Tuple rule age */
int nfp_ct_age(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto)
{
	NFP_RULE_CT	rule;
	unsigned long	flags;
	MV_STATUS status;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;

	read_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtRuleAge(&rule);
	read_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_age++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return rule.age;
}

/* Set a 5 Tuple filtering rule: drop or forward	*/
/* mode: 0 - drop, 1 - forward.				*/
int nfp_ct_filter_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int mode)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;
	if (mode == MV_ETH_NFP_CT_DROP) {
		rule.flags |= NFP_F_CT_DROP;
	} else if (mode == MV_ETH_NFP_CT_FORWARD) {
		rule.flags &= ~NFP_F_CT_DROP;
	} else {
		NFP_WARN(" bad mode parameter\n");
		return MV_BAD_PARAM;
	}

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtFilterModeSet(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		if (mode == MV_ETH_NFP_CT_DROP)
			ct_stats.ct_drop_set++;
		else
			ct_stats.ct_forward_set++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

int nfp_ct_udp_csum_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int mode)
{
	NFP_RULE_CT rule;
	MV_STATUS status;
	unsigned long flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtRuleUdpCsumSet(&rule, mode);
	write_unlock_irqrestore(&nfp_lock, flags);

	if (status != MV_OK) {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

#ifdef NFP_CLASSIFY

/* Set a 5 Tuple DSCP Manipulation Rule */
/* Update the DSCP value in packets matching the 5 Tuple and dscp information to new_dscp */
int nfp_ct_dscp_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int dscp, int new_dscp)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;
	rule.flags |= NFP_F_CT_SET_DSCP;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtDscpRuleAdd(&rule, dscp, new_dscp);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_dscp_set++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

/* Delete a 5 Tuple DSCP Manipulation Rule */
int nfp_ct_dscp_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int dscp)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtDscpRuleDel(&rule, dscp);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_dscp_del++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}


/* Set a 5 Tuple VLAN Priority Manipulation Rule */
/* Update the VLAN Priority value in 802.1Q VLAN-tagged packets matching the 5 Tuple and dscp information to new_dscp */
int nfp_ct_vlan_prio_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int prio, int new_prio)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;
	rule.flags |= NFP_F_CT_SET_VLAN_PRIO;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtVlanPrioRuleAdd(&rule, prio, new_prio);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_vpri_set++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

/* Delete a 5 Tuple VLAN Priority Manipulation Rule */
int nfp_ct_vlan_prio_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int prio)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtVlanPrioRuleDel(&rule, prio);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_vpri_del++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

/* Set a Tx queue value for a 5 Tuple rule */
int nfp_ct_txq_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int txq)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;
	rule.flags |= NFP_F_CT_SET_TXQ;
	rule.txq = txq;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtTxqRuleAdd(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_txq_set++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

/* Delete a Tx queue value from a 5 Tuple rule */
int nfp_ct_txq_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtTxqRuleDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_txq_del++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

/* Set a Tx port value for a 5 Tuple rule */
int nfp_ct_txp_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int txp)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;
	rule.flags |= NFP_F_CT_SET_TXP;
	rule.txp = txp;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtTxpRuleAdd(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_txp_set++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

/* Delete a Tx port value from a 5 Tuple rule */
int nfp_ct_txp_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtTxpRuleDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_txp_del++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

/* Set a Marvell Header value for a 5 Tuple rule */
int nfp_ct_mh_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, u16 mh)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;
	rule.flags |= NFP_F_CT_SET_MH;
	rule.mh = mh;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtMhRuleAdd(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_mh_set++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

/* Delete a Marvell Header value from a 5 Tuple rule */
int nfp_ct_mh_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtMhRuleDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_mh_del++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

#endif /* NFP_CLASSIFY */

/* Get hit counter for ct rule */
MV_STATUS nfp_ct_hit_cntr_get(MV_NFP_CT_KEY *ct_key, MV_U32 *hit_cntr)
{
	MV_STATUS status;
	unsigned long flags;

	if (!ct_key || !hit_cntr)
		return MV_BAD_PARAM;
	read_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtRuleHitCntrGet(ct_key, hit_cntr);
	read_unlock_irqrestore(&nfp_lock, flags);
	return status;
}

/* Clear hit counter fot ct rule */
MV_STATUS nfp_ct_hit_cntr_set(MV_NFP_CT_KEY *ct_key, MV_U32 val)
{
	MV_STATUS status;
	unsigned long flags;

	if (!ct_key)
		return MV_BAD_PARAM;
	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtRuleHitCntrSet(ct_key, val);
	write_unlock_irqrestore(&nfp_lock, flags);
	return status;
}

/* Get all NFP information about specific ct rule */
MV_STATUS nfp_ct_info_get(MV_NFP_CT_KEY *ct_key, MV_NFP_CT_INFO *ct_info)
{
	MV_STATUS status;
	unsigned long flags;

	if (!ct_key || !ct_info)
		return MV_BAD_PARAM;
	read_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtRuleInfoGet(ct_key, ct_info);
	read_unlock_irqrestore(&nfp_lock, flags);
	return status;
}

/* Get mostly used ct rule and its hit counter */
MV_STATUS nfp_ct_mostly_used_get(MV_NFP_CT_KEY *ct_key, MV_U32 *hit_cntr)
{
	NFP_RULE_CT *rule;
	MV_STATUS status;
	unsigned long flags;

	if (!ct_key || !hit_cntr)
		return MV_BAD_PARAM;
	read_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtRuleMaxHitCntrGet(&rule);
	read_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_key->family = rule->family;
		memcpy(ct_key->src_l3, rule->srcL3, MV_MAX_L3_ADDR_SIZE);
		memcpy(ct_key->dst_l3, rule->dstL3, MV_MAX_L3_ADDR_SIZE);
		ct_key->sport = MV_BYTE_SWAP_16BIT((MV_U16)rule->ports);
		ct_key->dport = MV_BYTE_SWAP_16BIT((MV_U16)(rule->ports >> 16));
		ct_key->proto = rule->proto;
		*hit_cntr = rule->hit_cntr;
	}
	return status;
}

/* Get first valid ct rule in the NFP ct data-base and its hit counter */
MV_STATUS nfp_ct_first_get(MV_NFP_CT_KEY *ct_key, MV_U32 *hit_cntr)
{
	NFP_RULE_CT *rule;
	MV_STATUS status;
	unsigned long flags;

	if (!ct_key || !hit_cntr)
		return MV_BAD_PARAM;
	read_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtFirstRuleGet(&rule, 0);
	read_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_key->family = rule->family;
		memcpy(ct_key->src_l3, rule->srcL3, MV_MAX_L3_ADDR_SIZE);
		memcpy(ct_key->dst_l3, rule->dstL3, MV_MAX_L3_ADDR_SIZE);
		ct_key->sport = MV_BYTE_SWAP_16BIT((MV_U16)rule->ports);
		ct_key->dport = MV_BYTE_SWAP_16BIT((MV_U16)(rule->ports >> 16));
		ct_key->proto = rule->proto;
		*hit_cntr = rule->hit_cntr;
	}
	return status;
}

/* Get next valid ct rule in the NFP ct data-base and its hit counter */
MV_STATUS nfp_ct_next_get(MV_NFP_CT_KEY *ct_key, MV_U32 *hit_cntr)
{
	NFP_RULE_CT *rule;
	MV_STATUS status;
	unsigned long flags;

	if (!ct_key || !hit_cntr)
		return MV_BAD_PARAM;
	read_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtNextRuleGet(&rule, 0);
	read_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_key->family = rule->family;
		memcpy(ct_key->src_l3, rule->srcL3, MV_MAX_L3_ADDR_SIZE);
		memcpy(ct_key->dst_l3, rule->dstL3, MV_MAX_L3_ADDR_SIZE);
		ct_key->sport = MV_BYTE_SWAP_16BIT((MV_U16)rule->ports);
		ct_key->dport = MV_BYTE_SWAP_16BIT((MV_U16)(rule->ports >> 16));
		ct_key->proto = rule->proto;
		*hit_cntr = rule->hit_cntr;
	}
	return status;
}

/* Inform NFP that specific ct rule added to HWF processing */
MV_STATUS nfp_ct_hwf_set(MV_NFP_CT_KEY *ct_key)
{
	MV_STATUS status;
	unsigned long flags;

	if (!ct_key)
		return MV_BAD_PARAM;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtRuleHwfSet(ct_key, 1);
	write_unlock_irqrestore(&nfp_lock, flags);
	return status;
}

/* Inform NFP that specific ct rule removed from HWF processing */
MV_STATUS nfp_ct_hwf_clear(MV_NFP_CT_KEY *ct_key)
{
	MV_STATUS status;
	unsigned long flags;

	if (!ct_key)
		return MV_BAD_PARAM;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtRuleHwfSet(ct_key, 0);
	write_unlock_irqrestore(&nfp_lock, flags);
	return status;
}

#endif /* NFP_CT */


#ifdef NFP_LIMIT
void nfp_mgr_tbfs_dump(void)
{
	mvNfpCtTbfsDump();
}

/* Create a Token Bucket Filter */
/* Return an index representing the Token Bucket Filter, or -1 on failure */
int nfp_tbf_create(int limit, int burst_limit)
{
	unsigned long	flags;
	int tbf = -1;

	write_lock_irqsave(&nfp_lock, flags);
	tbf = mvNfpTbfCreate(limit, burst_limit);
	write_unlock_irqrestore(&nfp_lock, flags);

	return tbf;
}

/* Delete a Token Bucket Filter */
int nfp_tbf_del(int tbf_index)
{
	unsigned long	flags;
	MV_STATUS	status;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpTbfDel(tbf_index);
	write_unlock_irqrestore(&nfp_lock, flags);

	return status;
}

/* Create an Ingress Rate Limit rule - attach a 5 Tuple to an existing Token Bucket Filter */
int nfp_ct_rate_limit_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int tbf_index)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;
	rule.flags |= NFP_F_CT_LIMIT;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtRateLimitSet(&rule, tbf_index);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_rate_limit_set++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}

/* Delete an Ingress Rate Limit Rule - detach a 5 Tuple from its Token Bucket Filter */
int nfp_ct_rate_limit_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto)
{
	NFP_RULE_CT	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);
	mvNfp5TupleInfoPrint(NFP_DBG_PRINT, family, src_l3, dst_l3, sport, dport, proto);

	memset(&rule, 0, sizeof(NFP_RULE_CT));
	rule.family = family;
	memcpy(rule.srcL3, src_l3, MV_MAX_L3_ADDR_SIZE);
	memcpy(rule.dstL3, dst_l3, MV_MAX_L3_ADDR_SIZE);
	rule.ports = MV_32BIT_BE((sport << 16) | dport); /* store as BE 	*/
	rule.proto = proto;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpCtRateLimitDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK) {
		ct_stats.ct_rate_limit_del++;
	} else {
		NFP_WARN("%s failed: \n", __func__);
		mvNfp5TupleInfoPrint(NFP_WARN_PRINT, family, src_l3, dst_l3, sport, dport, proto);
	}

	return status;
}
#endif /* NFP_LIMIT */


void nfp_mgr_dump(void)
{
	unsigned long flags;

	read_lock_irqsave(&nfp_lock, flags);

	mvNfpIfMapDump();

#ifdef NFP_BRIDGE
#ifdef NFP_FDB_MODE
	mvNfpFdbDump();
#else
	mvNfpBridgeDump();
#endif /* NFP_FDB_MODE */
#endif /* NFP_BRIDGE */

#ifdef NFP_FIB
	mvNfpArpDump();
	mvNfpFibDump();
#endif
#ifdef NFP_CT
	mvNfpCtDump();
#endif
#ifdef NFP_PNC
	mvNfpPncDump();
#endif

	read_unlock_irqrestore(&nfp_lock, flags);
}


void nfp_mgr_stats(void)
{
	int			        i;
	MV_NFP_NETDEV_MAP	*curr_dev = NULL;

	printk(KERN_ERR "(interfaces)\n");
	for (i = 0; i < NFP_DEV_HASH_SZ; i++) {
		curr_dev = nfp_net_devs[i];
		while (curr_dev != NULL) {
			printk(KERN_CONT " [%2d]: %10s, if=%2d - %s",
					i, nfp_mgr_if_type_str(curr_dev->if_type),
					 curr_dev->if_index, curr_dev->dev->name);

			if (curr_dev->if_type == MV_NFP_IF_INT)
				printk(KERN_CONT ", gbe=%d", curr_dev->port);

			printk(KERN_CONT "\n");
			curr_dev = curr_dev->next;
		}
	}

#ifdef NFP_HWF
	printk(KERN_ERR "HWF map: \n");
	printk(KERN_ERR "inp -> outp : txp  txq\n");
	for (i = 0; i < MV_ETH_MAX_PORTS; i++) {
		int j;

		for (j = 0; j < MV_ETH_MAX_PORTS; j++) {
			if (nfp_hwf_txq_map[i][j].txq != -1) {
				printk(KERN_ERR "  %d ->  %d   :  %d    %d\n", i, j,
						nfp_hwf_txq_map[i][j].txp, nfp_hwf_txq_map[i][j].txq);
			}
		}
	}
#endif /* NFP_HWF */

	printk(KERN_ERR "(mgr)\n");
	printk(KERN_ERR "         add   del   age\n");

#ifdef NFP_BRIDGE
	printk(KERN_ERR "br_mac : %3d   %3d   %3d\n",
		br_stats.br_rule_add, br_stats.br_rule_del, br_stats.br_rule_age);
	printk(KERN_ERR "br_if  : %3d   %3d   %3s\n",
		br_stats.br_if_add, br_stats.br_if_del, "N/A");

#if defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE)
	printk(KERN_ERR "vpri   : %3d   %3d   %3s\n",
		br_stats.br_vpri_set, br_stats.br_vpri_del, "N/A");
	printk(KERN_ERR "txq    : %3d   %3d   %3s\n",
		br_stats.br_txq_set, br_stats.br_txq_del, "N/A");
	printk(KERN_ERR "txp    : %3d   %3d   %3s\n",
		br_stats.br_txp_set, br_stats.br_txp_del, "N/A");
	printk(KERN_ERR "mh     : %3d   %3d   %3s\n",
		br_stats.br_mh_set, br_stats.br_mh_del, "N/A");
#endif /* defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE) */

#endif /* NFP_BRIDGE */

#ifdef NFP_FIB
	printk(KERN_ERR "arp    : %3d   %3d   %3d\n",
		arp_stats.add, arp_stats.del, arp_stats.age);
	printk(KERN_ERR "fib    : %3d   %3d   %3d\n",
		fib_stats.add, fib_stats.del, fib_stats.age);
#endif

#ifdef NFP_CT
	printk(KERN_ERR "ct_fwd : %3d   %3s   %3s\n",
					ct_stats.ct_forward_set, "N/A", "N/A");
	printk(KERN_ERR "ct_drop: %3d   %3s   %3s\n",
					ct_stats.ct_drop_set, "N/A", "N/A");
	printk(KERN_ERR "ct_nat : %3d   %3d   %3d\n",
					ct_stats.ct_nat_add, ct_stats.ct_del, ct_stats.ct_age);
	printk(KERN_ERR "ct_lim : %3d   %3d   %3s\n",
					ct_stats.ct_rate_limit_set, ct_stats.ct_rate_limit_del, "N/A");
#ifdef NFP_CLASSIFY
	printk(KERN_ERR "ct_dscp: %3d   %3d   %3s\n",
					ct_stats.ct_dscp_set, ct_stats.ct_dscp_del, "N/A");
	printk(KERN_ERR "ct_vpri: %3d   %3d   %3s\n",
					ct_stats.ct_vpri_set, ct_stats.ct_vpri_del, "N/A");
	printk(KERN_ERR "ct_txq : %3d   %3d   %3s\n",
					ct_stats.ct_txq_set, ct_stats.ct_txq_del, "N/A");
	printk(KERN_ERR "ct_txp : %3d   %3d   %3s\n",
					ct_stats.ct_txp_set, ct_stats.ct_txp_del, "N/A");
	printk(KERN_ERR "ct_mh  : %3d   %3d   %3s\n",
					ct_stats.ct_mh_set, ct_stats.ct_mh_del, "N/A");
#endif /* NFP_CLASSIFY */

#endif /* NFP_CT */
}



#ifdef NFP_PPP
int nfp_ppp_add(int ppp_if, u16 sid, u8 *remoteMac)
{
	unsigned long flags;
	MV_STATUS status;

	NFP_DBG("%s: sid=%d remoteMac= " MV_MACQUAD_FMT " ppp_if=%d\n",
		 __func__, sid, MV_MACQUAD(remoteMac), ppp_if);

	if (nfp_eth_dev_find(ppp_if) == NULL) {
		printk(KERN_ERR "%s Error: ppp_if (%d) interface is not bound to NFP\n",
				__func__, ppp_if);
		return -1;
	}
	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpPppAdd(ppp_if, sid, remoteMac);
	write_unlock_irqrestore(&nfp_lock, flags);

	return status;
}

int nfp_ppp_del(int ppp_if)
{
	unsigned long flags;
	MV_STATUS status;

	NFP_DBG("%s: ppp_if = %d\n", __func__, ppp_if);

	if (nfp_eth_dev_find(ppp_if) == NULL) {
		printk(KERN_ERR "%s Error: ppp_if (%d) interface is not bound to NFP\n",
				__func__, ppp_if);
		return -1;
	}
	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpPppDel(ppp_if);
	write_unlock_irqrestore(&nfp_lock, flags);

	return status;
}
#endif /* NFP_PPP */

#ifdef NFP_BRIDGE
/* Add port_if interface to bridge */
int nfp_if_to_bridge_add(int bridge_if, int port_if)
{
	int               err;
	MV_NFP_NETDEV_MAP *pdevBridge, *pdevPort;
	MV_STATUS         status;
	unsigned long	  flags;

	pdevPort = nfp_eth_dev_find(port_if);
	if (pdevPort == NULL) {
		/* port_if must be part of NFP */
		NFP_WARN("%s: port_if #%d is invalid\n", __func__, port_if);
		return 1;
	}

	pdevBridge = nfp_eth_dev_find(bridge_if);
	if (pdevBridge == NULL) {
		/* Add bridge_if to NFP if not added yet */
		err = nfp_mgr_if_register(bridge_if, MV_NFP_IF_BRG);
		if (err) {
			printk(KERN_ERR "%s error: Can't register bridge interface bridge_if=%d\n",
					__func__, bridge_if);
			return 1;
		}
	}

	write_lock_irqsave(&nfp_lock, flags);
	/* Add Bridge indication to port_if interface */
	status = mvNfpIfToBridgeAdd(bridge_if, port_if);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_if_add++;
	else
		printk(KERN_ERR "%s error: Can't add port_if=%d to bridge_if=%d\n",
					__func__, port_if, bridge_if);

	return status;
}

int nfp_if_to_bridge_del(int bridge_if, int port_if)
{
	MV_NFP_NETDEV_MAP	*pdevBridge, *pdevPort;
	MV_STATUS		status;
	unsigned long		flags;

	pdevPort = nfp_eth_dev_find(port_if);
	if (pdevPort == NULL) {
		/* port_if must be part of NFP */
		NFP_WARN("%s: port_if #%d is invalid\n", __func__, port_if);
		return 1;
	}

	pdevBridge = nfp_eth_dev_find(bridge_if);
	if (pdevBridge == NULL) {
		NFP_WARN("%s error: bridge (%d) is not registered in NFP\n", __func__, bridge_if);
		return 1;
	}

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpIfToBridgeDel(bridge_if, port_if);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_if_del++;
	else
		printk(KERN_ERR "%s error: Can't delete port_if=%d from bridge_if=%d\n",
					__func__, port_if, bridge_if);

	nfp_mgr_flush(port_if);

	return status;
}

void nfp_bridge_clean(void)
{
	unsigned long flags;

	write_lock_irqsave(&nfp_lock, flags);
#ifdef NFP_FDB_MODE
	mvNfpFdbFlushBridge(-1);
#else
	mvNfpFlushBridge(-1);
#endif
	write_unlock_irqrestore(&nfp_lock, flags);
}

#ifdef NFP_FDB_MODE
/* Add FDB rule */
int nfp_fdb_rule_add(int br_index, int if_index, u8 *mac, bool is_local)
{
	MV_NFP_NETDEV_MAP	*pdevPort, *pdevBridge;
	NFP_RULE_FDB		rule;
	unsigned long		flags;
	MV_STATUS		status;

	pdevPort = nfp_eth_dev_find(if_index);
	if (pdevPort == NULL) {
		/* if_index must be part of NFP */
		NFP_WARN("%s: if_index #%d is invalid\n", __func__, if_index);
		return 1;
	}
	pdevBridge = nfp_eth_dev_find(br_index);
	if (pdevBridge == NULL) {
		/* br_index must be part of NFP */
		NFP_WARN("%s: br_index #%d is invalid\n", __func__, br_index);
		return 1;
	}

	/* Prepare FDB rule */
	memset(&rule, 0, sizeof(NFP_RULE_FDB));
	memcpy(rule.mac, mac, MV_MAC_ADDR_SIZE);

	rule.if_index = if_index;
	rule.bridgeIf = br_index;
	if (is_local)
		rule.status = NFP_BRIDGE_LOCAL;
	else
		rule.status = NFP_BRIDGE_NON_LOCAL;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpFdbRuleAdd(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_rule_add++;
	else
		NFP_WARN("%s FAILED: br_index=%d, if_index=%d, mac="MV_MACQUAD_FMT", is_local=%d\n",
			__func__, br_index, if_index, MV_MACQUAD(mac), is_local);

	return status;
}

/* Check FDB rule age */
int nfp_fdb_rule_age(int br_index, u8 *mac)
{
	MV_NFP_NETDEV_MAP       *pdevBridge;
	NFP_RULE_FDB		rule;
	unsigned long		flags;
	MV_STATUS status;

	pdevBridge = nfp_eth_dev_find(br_index);
	if (pdevBridge == NULL) {
		/* if_index must be part of NFP */
		NFP_WARN("%s: br_index #%d is invalid\n", __func__, br_index);
		return 0;
	}
	/* Prepare FDB rule */
	memset(&rule, 0, sizeof(NFP_RULE_FDB));
	memcpy(rule.mac, mac, MV_MAC_ADDR_SIZE);
	rule.bridgeIf = br_index;
	rule.age = 0;

	read_lock_irqsave(&nfp_lock, flags);
	status = mvNfpFdbRuleAge(&rule);
	read_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_rule_age++;
	else {
		/* FIXME: hook should not call us in such case*/
		NFP_WARN("%s: Not found - br_index=%d, mac="MV_MACQUAD_FMT"\n",
						__func__, br_index, MV_MACQUAD(mac));
	}

	return rule.age;
}

/* Delete FDB rule */
int nfp_fdb_rule_del(int br_index, u8 *mac)
{
	MV_NFP_NETDEV_MAP	*pdevBridge;
	NFP_RULE_FDB		rule;
	unsigned long		flags;
	MV_STATUS		status;

	pdevBridge = nfp_eth_dev_find(br_index);
	if (pdevBridge == NULL) {
		/* if_index must be part of NFP */
		NFP_WARN("%s: br_index #%d is invalid\n", __func__, br_index);
		return 0;
	}

	/* Prepare FDB rule */
	memset(&rule, 0, sizeof(NFP_RULE_FDB));
	memcpy(rule.mac, mac, MV_MAC_ADDR_SIZE);
	rule.bridgeIf = br_index;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpFdbRuleDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_rule_del++;
	else
		NFP_WARN("%s FAILED: br_index=%d, mac="MV_MACQUAD_FMT"\n",
					__func__, br_index, MV_MACQUAD(mac));

	return status;
}

#else /* Static Learn mode */

/* Add bridging rule */
int nfp_bridge_rule_add(u8 *sa, u8 *da, int iif, int oif)
{
	MV_STATUS		status;
	MV_NFP_NETDEV_MAP	*pdevIn, *pdevOut;
	NFP_RULE_BRIDGE		rule;
	unsigned long       	flags;

	pdevIn = nfp_eth_dev_find(iif);
	if (pdevIn == NULL) {
		/* iif must be part of NFP */
		NFP_WARN("%s: iif #%d is invalid\n", __func__, iif);
		return 1;
	}

	pdevOut = nfp_eth_dev_find(oif);
	if (pdevOut == NULL) {
		/* oif must be part of NFP */
		NFP_WARN("%s: oif #%d is invalid\n", __func__, oif);
		return 1;
	}

	/* Prepare bridge rule */
	memset(&rule, 0, sizeof(NFP_RULE_BRIDGE));
	memcpy(rule.da, da, MV_MAC_ADDR_SIZE);
	memcpy(rule.sa, sa, MV_MAC_ADDR_SIZE);
	rule.iif = iif;
	rule.oif = oif;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpBridgeRuleAdd(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_rule_add++;
	else
		NFP_WARN("%s failed:\n", __func__);

	return status;
}

int nfp_bridge_rule_del(u8 *sa, u8 *da, int iif)
{
	MV_STATUS		status;
	MV_NFP_NETDEV_MAP	*pdevIn;
	NFP_RULE_BRIDGE		rule;
	unsigned long		flags;

	pdevIn = nfp_eth_dev_find(iif);
	if (pdevIn == NULL) {
		/* iif must be part of NFP */
		NFP_WARN("%s: iif #%d is invalid\n", __func__, iif);
		return 1;
	}

	/* Prepare bridge rule */
	memset(&rule, 0, sizeof(NFP_RULE_BRIDGE));
	memcpy(rule.da, da, MV_MAC_ADDR_SIZE);
	memcpy(rule.sa, sa, MV_MAC_ADDR_SIZE);
	rule.iif = iif;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpBridgeRuleDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_rule_del++;
	else
		NFP_WARN("%s failed:\n", __func__);

	return 0;
}

int nfp_bridge_rule_age(u8 *sa, u8 *da, int iif)
{
	MV_STATUS		status;
	MV_NFP_NETDEV_MAP	*pdevIn;
	NFP_RULE_BRIDGE		rule;
	unsigned long		flags;

	pdevIn = nfp_eth_dev_find(iif);
	if (pdevIn == NULL) {
		/* iif must be part of NFP */
		NFP_WARN("%s: iif #%d is invalid\n", __func__, iif);
		return 1;
	}

	/* Prepare bridge rule */
	memset(&rule, 0, sizeof(NFP_RULE_BRIDGE));
	memcpy(rule.da, da, MV_MAC_ADDR_SIZE);
	memcpy(rule.sa, sa, MV_MAC_ADDR_SIZE);
	rule.iif = iif;

	read_lock_irqsave(&nfp_lock, flags);
	status = mvNfpBridgeRuleAge(&rule);
	read_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_rule_age++;
	else
		NFP_WARN("%s failed:\n", __func__);

	return rule.age;
}
#endif /* NFP_FDB_MODE */

#if defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE)
int nfp_bridge_vlan_prio_set(u8 *sa, u8 *da, int iif, int eth_type, int vlan_prio)
{
	NFP_RULE_BRIDGE	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);

	memset(&rule, 0, sizeof(NFP_RULE_BRIDGE));
	memcpy(rule.da, da, MV_MAC_ADDR_SIZE);
	memcpy(rule.sa, sa, MV_MAC_ADDR_SIZE);
	rule.iif = iif;
	rule.flags |= NFP_F_BR_SET_VLAN_PRIO;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpBridgeVlanPrioRuleAdd(&rule, eth_type, vlan_prio);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_vpri_set++;
	else
		NFP_WARN("%s failed: \n", __func__);

	return status;
}

int nfp_bridge_vlan_prio_del(u8 *sa, u8 *da, int iif, int eth_type)
{
	NFP_RULE_BRIDGE	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);

	memset(&rule, 0, sizeof(NFP_RULE_BRIDGE));
	memcpy(rule.da, da, MV_MAC_ADDR_SIZE);
	memcpy(rule.sa, sa, MV_MAC_ADDR_SIZE);
	rule.iif = iif;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpBridgeVlanPrioRuleDel(&rule, eth_type);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_vpri_del++;
	else
		NFP_WARN("%s failed: \n", __func__);

	return status;
}

int nfp_bridge_txq_set(u8 *sa, u8 *da, int iif, int txq)
{
	NFP_RULE_BRIDGE	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);

	memset(&rule, 0, sizeof(NFP_RULE_BRIDGE));
	memcpy(rule.da, da, MV_MAC_ADDR_SIZE);
	memcpy(rule.sa, sa, MV_MAC_ADDR_SIZE);
	rule.iif = iif;
	rule.flags |= NFP_F_BR_SET_TXQ;
	rule.txq = txq;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpBridgeTxqRuleAdd(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_txq_set++;
	else
		NFP_WARN("%s failed: \n", __func__);

	return status;
}

int nfp_bridge_txq_del(u8 *sa, u8 *da, int iif)
{
	NFP_RULE_BRIDGE	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);

	memset(&rule, 0, sizeof(NFP_RULE_BRIDGE));
	memcpy(rule.da, da, MV_MAC_ADDR_SIZE);
	memcpy(rule.sa, sa, MV_MAC_ADDR_SIZE);
	rule.iif = iif;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpBridgeTxqRuleDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_txq_del++;
	else
		NFP_WARN("%s failed: \n", __func__);

	return status;
}

int nfp_bridge_txp_set(u8 *sa, u8 *da, int iif, int txp)
{
	NFP_RULE_BRIDGE	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);

	memset(&rule, 0, sizeof(NFP_RULE_BRIDGE));
	memcpy(rule.da, da, MV_MAC_ADDR_SIZE);
	memcpy(rule.sa, sa, MV_MAC_ADDR_SIZE);
	rule.iif = iif;
	rule.flags |= NFP_F_BR_SET_TXP;
	rule.txp = txp;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpBridgeTxpRuleAdd(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_txp_set++;
	else
		NFP_WARN("%s failed: \n", __func__);

	return status;
}

int nfp_bridge_txp_del(u8 *sa, u8 *da, int iif)
{
	NFP_RULE_BRIDGE	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);

	memset(&rule, 0, sizeof(NFP_RULE_BRIDGE));
	memcpy(rule.da, da, MV_MAC_ADDR_SIZE);
	memcpy(rule.sa, sa, MV_MAC_ADDR_SIZE);
	rule.iif = iif;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpBridgeTxpRuleDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_txp_del++;
	else
		NFP_WARN("%s failed: \n", __func__);

	return status;
}

int nfp_bridge_mh_set(u8 *sa, u8 *da, int iif, u16 mh)
{
	NFP_RULE_BRIDGE	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);

	memset(&rule, 0, sizeof(NFP_RULE_BRIDGE));
	memcpy(rule.da, da, MV_MAC_ADDR_SIZE);
	memcpy(rule.sa, sa, MV_MAC_ADDR_SIZE);
	rule.iif = iif;
	rule.flags |= NFP_F_BR_SET_MH;
	rule.mh = mh;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpBridgeMhRuleAdd(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_mh_set++;
	else
		NFP_WARN("%s failed: \n", __func__);

	return status;
}

int nfp_bridge_mh_del(u8 *sa, u8 *da, int iif)
{
	NFP_RULE_BRIDGE	rule;
	MV_STATUS	status;
	unsigned long	flags;

	NFP_DBG("%s:\n", __func__);

	memset(&rule, 0, sizeof(NFP_RULE_BRIDGE));
	memcpy(rule.da, da, MV_MAC_ADDR_SIZE);
	memcpy(rule.sa, sa, MV_MAC_ADDR_SIZE);
	rule.iif = iif;

	write_lock_irqsave(&nfp_lock, flags);
	status = mvNfpBridgeMhRuleDel(&rule);
	write_unlock_irqrestore(&nfp_lock, flags);
	if (status == MV_OK)
		br_stats.br_mh_del++;
	else
		NFP_WARN("%s failed: \n", __func__);

	return status;
}
#endif /* defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE) */
#endif /* NFP_BRIDGE */


#ifdef NFP_CLASSIFY
int nfp_dscp_policy_set(int mode)
{
	mvNfpDscpPolicySet(mode);
	return 0;
}

int nfp_vlan_prio_policy_set(int mode)
{
	mvNfpVlanPrioPolicySet(mode);
	return 0;
}

int nfp_txq_policy_set(int mode)
{
	mvNfpTxqPolicySet(mode);
	return 0;
}

int nfp_txp_policy_set(int mode)
{
	mvNfpTxpPolicySet(mode);
	return 0;
}

int nfp_mh_policy_set(int mode)
{
	mvNfpMhPolicySet(mode);
	return 0;
}

int nfp_dscp_policy_get(void)
{
	return mvNfpDscpPolicyGet();
}

int nfp_vlan_prio_policy_get(void)
{
	return mvNfpVlanPrioPolicyGet();
}

int nfp_txq_policy_get(void)
{
	return mvNfpTxqPolicyGet();
}

int nfp_txp_policy_get(void)
{
	return mvNfpTxpPolicyGet();
}

int nfp_mh_policy_get(void)
{
	return mvNfpMhPolicyGet();
}
#endif /* NFP_CLASSIFY */

#ifdef NFP_FIB
void nfp_clean_fib(int family)
{
	NFP_RULE_FIB *fib;
	MV_LIST_ELEMENT	*curr;
	int i;
	unsigned long		flags;
	for (i = 0; i < NFP_FIB_HASH_SIZE; i++) {
		fib = fib_hash[i];

		while (fib) {
			if (fib->family == family) {
					write_lock_irqsave(&nfp_lock, flags);
					mvNfpFibRuleDel(fib);
					write_unlock_irqrestore(&nfp_lock, flags);
					}
			fib = fib->next;
		}
	}
	if (fib_inv_list) {
		curr = fib_inv_list->next;
		while (curr) {
			fib = (NFP_RULE_FIB *)curr->data;
			if (fib->family == family) {
				write_lock_irqsave(&nfp_lock, flags);
				mvNfpFibRuleDel(fib);
				write_unlock_irqrestore(&nfp_lock, flags);
				}

			curr = curr->next;
		}
	}
}

void nfp_clean_arp(int family)
{
	MV_U32 i;
	NFP_RULE_ARP *arp;
	unsigned long		flags;

	for (i = 0; i < NFP_ARP_HASH_SIZE; i++) {
		arp = nfp_arp_hash[i];

		while (arp) {
			if (arp->family == family) {
				write_lock_irqsave(&nfp_lock, flags);
				mvNfpArpRuleDel(arp);
				write_unlock_irqrestore(&nfp_lock, flags);
				}

			arp = arp->next;
		}
	}
}

#endif /* NFP_FIB */

#ifdef NFP_CT

void nfp_ct_clean(int family)
{
	unsigned long flags;
	write_lock_irqsave(&nfp_lock, flags);
	mvNfpCtClean(family);
	write_unlock_irqrestore(&nfp_lock, flags);
}

#endif /* NFP_CT */

