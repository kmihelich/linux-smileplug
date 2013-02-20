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
* mv_nfp_mgr.h - Header File for Marvell NFP Manager
*
* DESCRIPTION:
*       This header file contains macros, typedefs and function declarations
* 	specific to the Marvell Network Fast Processing Manager.
*
* DEPENDENCIES:
*       None.
*
*******************************************************************************/

#ifndef __mv_nfp_mgr_h__
#define __mv_nfp_mgr_h__

#include <linux/netdevice.h>

#include "nfp/mvNfp.h"
#include "net_dev/mv_netdev.h"


/* NFP interface type, used for registration */
typedef enum {
	MV_NFP_IF_INT  = 0,   /* Marvell GbE interface */
	MV_NFP_IF_EXT  = 1,   /* External interface (e.g WLAN) */
	MV_NFP_IF_BRG  = 2,   /* Bridge interface */
	MV_NFP_IF_VIRT = 3,   /* Virtual interface */
} MV_NFP_IF_TYPE;

typedef struct nfp_netdev_map {
	struct nfp_netdev_map	*next;
	MV_NFP_IF_TYPE		if_type;
	int			if_index;
	MV_U8			port;
	struct net_device	*dev;
} MV_NFP_NETDEV_MAP;

typedef struct {
	MV_U16  sid;
	MV_U32  iif;
	MV_U32  chan;
	void    *dev;
	MV_U8	da[MV_MAC_ADDR_SIZE];
} MV_RULE_PPP;

typedef struct nfp_ppp_rule_map {
	struct nfp_ppp_rule_map *next;
	MV_RULE_PPP rule_ppp;
} MV_NFP_PPP_RULE_MAP;


int nfp_hwf_txq_set(int rxp, int port, int txp, int txq);
int nfp_hwf_txq_get(int rxp, int port, int *txp, int *txq);




/* Turn on/off NFP						*/
/* en - operation(on/off):					*/
/*	0 - NFP is off						*/
/*		disable learn, disable data path, clean all 	*/
/*	1 - NFP is on						*/
/*		enable learn, enable data path, sync all 	*/
int nfp_mgr_enable(int en);

/* Get NFP state			*/
/* return: 				*/
/*	MV_NFP_ON  - nfp is off 	*/
/*	MV_NFP_OFF - nfp is on	 	*/
int nfp_mgr_state_get(void);

/* Set NFP mode 	 	*/
/* mode - NFP operation mode:	*/
/* 	0 - 2 tuple mode 	*/
/* 	1 - 5 tuple mode 	*/
int nfp_mgr_mode_set(int mode);

/* Get NFP mode					*/
/* return: 					*/
/*	MV_NFP_2_TUPLE - for 2 tuple mode 	*/
/*	MV_NFP_5_TUPLE - for 5 tuple mode 	*/
int nfp_mgr_mode_get(void);

/* Enable/Disable NFP packet processing							*/
/* en - operation(enable/disable)							*/
/*	0 - disable NFP packet processing						*/
/*	1 - enable NFP packet processing (only if NFP is ON, via nfp_mgr_enable())	*/
int nfp_mgr_data_path_enable(int en);

void nfp_mgr_clean(void);

void nfp_mgr_status(void);

#ifdef NFP_LEARN
/* global learning - control all features learning
   when 0: no learning at all
   when 1: learning is enabled for features with local learn enable(ex. fib_learn_enable) */
void nfp_learn_enable(int en);
void nfp_age_enable(int en);
void nfp_hook_sync(void);
int nfp_is_learning_enabled(void);
int nfp_is_age_enabled(void);
#endif /* NFP_LEARN */

#ifdef CONFIG_MV_ETH_NFP_FIB_LEARN
int  nfp_fib_is_learning_enabled(void);
int nfp_fib_learn_enable(int en);
int  nfp_fib_is_aging_enabled(void);
int nfp_fib_age_enable(int en);
int  nfp_fib6_is_learning_enabled(void);
int nfp_fib6_learn_enable(int en);
int  nfp_fib6_is_aging_enabled(void);
int nfp_fib6_age_enable(int en);
void nfp_fib_sync(void);
#ifdef CONFIG_IPV6
void nfp_fib6_sync(void);
#endif
void neigh_sync(int family);
#endif

#ifdef CONFIG_MV_ETH_NFP_CT_LEARN
int  nfp_ct_is_learning_enabled(void);
int nfp_ct_learn_enable(int en);
int  nfp_ct_is_aging_enabled(void);
int nfp_ct_age_enable(int en);
int  nfp_ct6_is_learning_enabled(void);
int nfp_ct6_learn_enable(int en);
int  nfp_ct6_is_aging_enabled(void);
int nfp_ct6_age_enable(int en);
void nfp_ct_sync(int family);
#endif

#ifdef CONFIG_MV_ETH_NFP_FDB_LEARN
int  nfp_bridge_is_learning_enabled(void);
int nfp_bridge_learn_enable(int en);
int  nfp_bridge_is_aging_enabled(void);
int nfp_bridge_age_enable(int en);
void fdb_sync(void);
#endif

#ifdef CONFIG_MV_ETH_NFP_VLAN_LEARN
int  nfp_vlan_is_learning_enabled(void);
int nfp_vlan_learn_enable(int en);
void vlan_sync(void);
#endif

#ifdef CONFIG_MV_ETH_NFP_PPP_LEARN
int  nfp_ppp_is_learning_enabled(void);
int nfp_ppp_learn_enable(int en);
#endif

void nfp_mgr_flush(int ifindex);

void nfp_mgr_dump(void);
void nfp_mgr_stats(void);
int  nfp_mgr_if_bind(int if_index);
int  nfp_mgr_if_unbind(int if_index);
int  nfp_mgr_if_virt_add(int if_index, int if_virt);
int  nfp_mgr_if_virt_del(int if_virt);
int  nfp_mgr_if_mac_set(int if_index, u8 *mac);
int  nfp_mgr_if_mtu_set(int if_index, int mtu);
int  nfp_mgr_is_valid_index(int if_index);
void nfp_eth_dev_db_clear(void);

MV_NFP_NETDEV_MAP *nfp_eth_dev_find(int if_index);

#ifdef NFP_BRIDGE
int nfp_if_to_bridge_add(int bridge_if, int port_if);
int nfp_if_to_bridge_del(int bridge_if, int port_if);
void nfp_bridge_clean(void);

#ifdef NFP_FDB_MODE
/* Add/Del FDB rule */
int nfp_fdb_rule_add(int br_index, int if_index, u8 *mac, bool is_local);
int nfp_fdb_rule_age(int br_index, u8 *mac);
int nfp_fdb_rule_del(int br_index, u8 *mac);
#else
/* Add/Del/Age bridging rule */
int nfp_bridge_rule_add(u8 *sa, u8 *da, int iif, int oif);
int nfp_bridge_rule_del(u8 *sa, u8 *da, int iif);
int nfp_bridge_rule_age(u8 *sa, u8 *da, int iif);
#endif /* NFP_FDB_MODE */

#if defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE)
int nfp_bridge_vlan_prio_set(u8 *sa, u8 *da, int iif, int eth_type, int vlan_prio);
int nfp_bridge_vlan_prio_del(u8 *sa, u8 *da, int iif, int eth_type);
int nfp_bridge_txq_set(u8 *sa, u8 *da, int iif, int txq);
int nfp_bridge_txq_del(u8 *sa, u8 *da, int iif);
int nfp_bridge_txp_set(u8 *sa, u8 *da, int iif, int txp);
int nfp_bridge_txp_del(u8 *sa, u8 *da, int iif);
int nfp_bridge_mh_set(u8 *sa, u8 *da, int iif, u16 mh);
int nfp_bridge_mh_del(u8 *sa, u8 *da, int iif);
#endif /* defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE) */

#endif /* NFP_BRIDGE */

#ifdef NFP_FIB
int nfp_fib_rule_add(int family, u8 *src_l3, u8 *dst_l3, u8 *gw, int oif);
int nfp_fib_rule_del(int family, u8 *src_l3, u8 *dst_l3);
int nfp_fib_rule_age(int family, u8 *src_l3, u8 *dst_l3);
void nfp_clean_fib(int family);
void nfp_clean_arp(int family);

int nfp_arp_add(int family, u8 *ip, u8 *mac);
int nfp_arp_del(int family, u8 *ip);
int nfp_arp_age(int family, u8 *ip);
#endif /* NFP_FIB */

#ifdef NFP_VLAN
typedef enum {
	MV_NFP_VLAN_RX_TRANSPARENT                = 0,
	MV_NFP_VLAN_RX_DROP_UNTAGGED              = 1,
	MV_NFP_VLAN_RX_DROP_TAGGED                = 2,
	MV_NFP_VLAN_RX_DROP_UNKNOWN               = 3,
	MV_NFP_VLAN_RX_DROP_UNTAGGED_AND_UNKNOWN  = 4,

} MV_NFP_VLAN_RX_MODE;

typedef enum {
	MV_NFP_VLAN_TX_TRANSPARENT = 0,
	MV_NFP_VLAN_TX_UNTAGGED    = 1,
	MV_NFP_VLAN_TX_TAGGED      = 2,

} MV_NFP_VLAN_TX_MODE;

int nfp_vlan_pvid_set(int if_index, u16 pvid);
int nfp_vlan_vid_set(int if_index, u16 vid);
int nfp_vlan_rx_mode_set(int if_index, int mode);
int nfp_vlan_tx_mode_set(int if_index, int mode);
#endif /* NFP_VLAN */

#ifdef NFP_PPP
int nfp_ppp_add(int ppp_if, u16 sid, u8 *remoteMac);
int nfp_ppp_del(int ppp_if);
#endif /* NFP_PPP */

#ifdef NFP_CT
typedef enum {
	MV_ETH_NFP_CT_DROP = 0,
	MV_ETH_NFP_CT_FORWARD = 1,

} MV_ETH_NFP_CT_MODE;

/* Delete an existing 5-tuple connection tracking rule.			*/
/* The rule will be deleted with all the configured information, e.g.	*/
/* NAPT, Drop/Forward, Classification, Rate Limit etc.			*/
void nfp_ct_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto);

/* Return a 5 Tuple rule age */
int nfp_ct_age(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto);

/* Set a 5 Tuple filtering rule: drop or forward	*/
/* mode: 0 - drop, 1 - forward.				*/
int nfp_ct_filter_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int mode);

/* Set a UDP checksum calculation mode */
/* mode: 0 - set zero, 1 - recalculate.				*/
int nfp_ct_udp_csum_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int mode);

#ifdef NFP_CLASSIFY
/* Set a 5 Tuple DSCP Manipulation Rule */
/* Update the DSCP value in packets matching the 5 Tuple and dscp information to new_dscp */
int nfp_ct_dscp_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int dscp, int new_dscp);

/* Delete a 5 Tuple DSCP Manipulation Rule */
int nfp_ct_dscp_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int dscp);

/* Set a 5 Tuple VLAN Priority Manipulation Rule */
/* Update the VLAN Priority value in 802.1Q VLAN-tagged packets matching the 5 Tuple and dscp information to new_dscp */
int nfp_ct_vlan_prio_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int prio, int new_prio);

/* Delete a 5 Tuple VLAN Priority Manipulation Rule */
int nfp_ct_vlan_prio_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int prio);

/* Set a Tx queue value for a 5 Tuple rule */
int nfp_ct_txq_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int txq);

/* Delete a Tx queue value from a 5 Tuple rule */
int nfp_ct_txq_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto);

/* Set a Tx port value for a 5 Tuple rule */
int nfp_ct_txp_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int txp);

/* Delete a Tx port value from a 5 Tuple rule */
int nfp_ct_txp_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto);

/* Set a Marvell Header value for a 5 Tuple rule */
int nfp_ct_mh_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, u16 mh);

/* Delete a Marvell Header value from a 5 Tuple rule */
int nfp_ct_mh_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto);
#endif /* NFP_CLASSIFY */

/* Get hit counter for ct rule */
MV_STATUS nfp_ct_hit_cntr_get(MV_NFP_CT_KEY *ct_key, MV_U32 *hit_cntr);

/* Set hit counter for ct rule */
MV_STATUS nfp_ct_hit_cntr_set(MV_NFP_CT_KEY *ct_key, MV_U32 val);

/* Get all NFP information about specific ct rule */
MV_STATUS nfp_ct_info_get(MV_NFP_CT_KEY *ct_key, MV_NFP_CT_INFO *ct_info);

/* Get mostly used ct rule and its hit counter */
MV_STATUS nfp_ct_mostly_used_get(MV_NFP_CT_KEY *ct_key, MV_U32 *hit_cntr);

/* Get first valid ct rule in the NFP ct data-base and its hit counter */
MV_STATUS nfp_ct_first_get(MV_NFP_CT_KEY *ct_key, MV_U32 *hit_cntr);

/* Get first valid ct rule in the NFP ct data-base and its hit counter */
MV_STATUS nfp_ct_next_get(MV_NFP_CT_KEY *ct_key, MV_U32 *hit_cntr);

/* Inform NFP that specific ct rule added to HWF processing */
MV_STATUS nfp_ct_hwf_set(MV_NFP_CT_KEY *ct_key);

/* Inform NFP that specifiv ct rule removed from HWF processing */
MV_STATUS nfp_ct_hwf_clear(MV_NFP_CT_KEY *ct_key);

/* clean CT rules by family */
void nfp_ct_clean(int family);

#endif /* NFP_CT */

#ifdef NFP_CLASSIFY
int nfp_dscp_policy_set(int mode);
int nfp_vlan_prio_policy_set(int mode);
int nfp_txq_policy_set(int mode);
int nfp_txp_policy_set(int mode);
int nfp_mh_policy_set(int mode);

int nfp_dscp_policy_get(void);
int nfp_vlan_prio_policy_get(void);
int nfp_txq_policy_get(void);
int nfp_txp_policy_get(void);
int nfp_mh_policy_get(void);
#endif /* NFP_CLASSIFY */

#ifdef NFP_NAT

#define MV_ETH_NFP_NAT_MANIP_DST	0x01
#define MV_ETH_NFP_NAT_MANIP_SRC	0x02

/* Add a 5-tuple NAT rule */
int nfp_ct_nat_add(u32 sip, u32 dip, u16 sport, u16 dport, u8 proto,
		       u32 new_sip, u32 new_dip, u16 new_sport, u16 new_dport,
		       int manip_flags);

#endif /* NFP_NAT */

#ifdef NFP_LIMIT
void nfp_mgr_tbfs_dump(void);
/* Create a Token Bucket Filter */
/* Return an index representing the Token Bucket Filter, or -1 on failure */
int nfp_tbf_create(int limit, int burst_limit);

/* Delete a Token Bucket Filter */
int nfp_tbf_del(int tbf_index);

/* Create an Ingress Rate Limit rule - attach a 5 Tuple to an existing Token Bucket Filter */
int nfp_ct_rate_limit_set(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto, int tbf_index);

/* Delete an Ingress Rate Limit Rule - detach a 5 Tuple from its Token Bucket Filter */
int nfp_ct_rate_limit_del(int family, u8 *src_l3, u8 *dst_l3, u16 sport, u16 dport, u8 proto);
#endif /* NFP_LIMIT */

int nfp_mgr_if_unregister(int if_index);
#endif /* __mv_nfp_mgr_h__ */

