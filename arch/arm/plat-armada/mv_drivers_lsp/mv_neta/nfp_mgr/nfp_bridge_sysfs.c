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

#include "mvOs.h"
#include "gbe/mvNeta.h"
#include "nfp/mvNfp.h"

#include "mv_nfp_mgr.h"
#include "nfp_sysfs.h"
#include "net_dev/mv_netdev.h"




static ssize_t bridge_help(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char *name = attr->attr.name;
	if (!strcmp(name, "help")) {
		PRINT("cat                             help              - print this help\n");
		PRINT("echo br_name if               > if_add            - add an interface to bridge\n");
		PRINT("echo br_name if               > if_del            - delete an interface from bridge\n");

#ifdef NFP_FDB_LEARN
		PRINT("echo [0 | 1]                  > learn             - enable/disble NFP bridging dynamic learning\n");
		PRINT("echo [0 | 1]                  > age_enable        - enable/disble NFP bridging aging\n");
		PRINT("echo 1                        > sync              - sync Linux fdb database to NFP database\n");
#endif /* NFP_FDB_LEARN */
#ifdef NFP_FDB_MODE
		PRINT("echo br_name mac if is_local  > rule_add          - add a new bridging rule\n");
		PRINT("echo br_name mac              > rule_del          - delete an existing bridging rule\n");
		PRINT("echo br_name mac              > rule_age          - print number of packets match this rule\n\n");
#else
		PRINT("echo sa da iif oif            > rule_add          - add a new bridging rule\n");
		PRINT("echo sa da iif                > rule_del          - delete an existing bridging rule\n");
		PRINT("echo sa da iif                > rule_age          - print number of packets match this rule\n");
#endif /* NFP_FDB_MODE */
		PRINT("echo mode                     > clean             - clean NFP BRIDGE DB. \n");		
	} else {
#if defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE)
	PRINT("echo sa da iif eth_type prio  > vlan_prio         - set a new vlan prio for bridging rule of type (eth_type)\n");
	PRINT("                                                    if prio = -1, then delete former mapping of this rule and eth type.\n");
	PRINT("echo sa da iif prio           > vlan_prio_default - set a new vlan prio for bridging rule of type with default eth type.\n");
	PRINT("                                                    if prio = -1, then delete former mapping of this rule.\n");
	PRINT("echo sa da iif txq            > txq               - set transmit queue for existing bridging rule.\n");
	PRINT("                                                    if txq = -1, delete txq value previously set for existing bridging rule.\n\n");
	PRINT("echo sa da iif txp            > txp               - set transmit port for existing bridging rule.\n");
	PRINT("                                                    if txp = -1, delete txp value previously set for existing bridging rule.\n\n");
	PRINT("echo sa da iif mh             > mh                - set Marvell Header for existing bridging rule.\n");
	PRINT("                                                    if mh = -1, delete Marvell Header previously set for existing bridging rule.\n\n");
#endif /* defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE) */
	}
	PRINT("\nParameters: if - interface name(i.e. ethX), br_name - bridge interface name, iif - ingress interface name,\n");
	PRINT("            oif - egress interface name, br_mac - bridge's MAC, sa - source MAC, da - destination MAC\n");
	PRINT("            mode - when 1, perform bridge clean, else do nothing. \n\n");
	return off;
}

#ifdef NFP_FDB_LEARN
static ssize_t bridge_learn_age_sync(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int err, a;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = err = 0;
	sscanf(buf, "%x", &a);

	if (!strcmp(name, "learn")) {
		nfp_bridge_learn_enable(a);
	} else if (!strcmp(name, "age_enable")) {
		nfp_bridge_age_enable(a);
	} else if (!strcmp(name, "sync")) {
		fdb_sync();
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	return err ? -EINVAL : len;
}
#endif /* NFP_FDB_LEARN */

#ifdef NFP_FDB_MODE
static ssize_t bridge_add_del_age(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, br_idx, if_idx, is_local;
	char br_name[NAME_SIZE], if_name[NAME_SIZE], macStr[MAC_SIZE];
	MV_U8 mac[MV_MAC_ADDR_SIZE];
	const char *name = attr->attr.name;
	struct net_device *netdev;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	res = sscanf(buf, "%s %s %s %d", br_name, macStr, if_name, &is_local);
	if ((!strcmp(name, "rule_add") && (res < 4 || is_local > 1)) || (strcmp(name, "rule_del") && (res < 2))) {
		err = 1;
		goto bridge_rule_out;
	}

	netdev = dev_get_by_name(&init_net, br_name);
	if (netdev == NULL) {
		err = 1;
		goto bridge_rule_out;
	}
	br_idx = netdev->ifindex;
	dev_put(netdev);

	mvMacStrToHex(macStr, mac);

	if (!strcmp(name, "rule_add")) {
		netdev = dev_get_by_name(&init_net, if_name);
		if (netdev == NULL) {
			err = 1;
			goto bridge_rule_out;
		}
		if_idx = netdev->ifindex;
		dev_put(netdev);

		nfp_fdb_rule_add(br_idx, if_idx, mac, is_local);
	} else if (!strcmp(name, "rule_del")) {
		nfp_fdb_rule_del(br_idx, mac);
	} else if (!strcmp(name, "rule_age")) {
		printk(KERN_INFO "age: %d\n", nfp_fdb_rule_age(br_idx, mac));
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}

bridge_rule_out:
	return err ? -EINVAL : len;
}
#else
static ssize_t bridge_add_del_age(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, iif, oif;
	char iif_name[NAME_SIZE], oif_name[NAME_SIZE], daStr[MAC_SIZE], saStr[MAC_SIZE];
	MV_U8 da[MV_MAC_ADDR_SIZE], sa[MV_MAC_ADDR_SIZE];
	const char *name = attr->attr.name;
	struct net_device *netdev;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	res = sscanf(buf, "%s %s %s %s", saStr, daStr, iif_name, oif_name);
	if ((!strcmp(name, "rule_add") && (res < 4)) || (strcmp(name, "rule_del") && (res < 3))) {
		err = 1;
		goto bridge_rule_out;
	}

	netdev = dev_get_by_name(&init_net, iif_name);
	if (netdev == NULL) {
		err = 1;
		goto bridge_rule_out;
	}
	iif = netdev->ifindex;
	dev_put(netdev);
	mvMacStrToHex(daStr, da);
	mvMacStrToHex(saStr, sa);

	if (!strcmp(name, "rule_add")) {
		netdev = dev_get_by_name(&init_net, oif_name);
		if (netdev == NULL) {
			err = 1;
			goto bridge_rule_out;
		}
		oif = netdev->ifindex;
		dev_put(netdev);
		NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_rule_add( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT" , %d, %d )\n",
			MV_MACQUAD(sa), MV_MACQUAD(da), iif, oif));
		nfp_bridge_rule_add(sa, da, iif, oif);
	} else if (!strcmp(name, "rule_del")) {
		NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_rule_del( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT", %d )\n",
			MV_MACQUAD(sa), MV_MACQUAD(da), iif));
		nfp_bridge_rule_del(sa, da, iif);
	} else if (!strcmp(name, "rule_age")) {
		NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_rule_age( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT" , %d )\n",
			MV_MACQUAD(sa), MV_MACQUAD(da), iif));
		printk(KERN_INFO "age: %d\n", nfp_bridge_rule_age(sa, da, iif));
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}

bridge_rule_out:
	return err ? -EINVAL : len;
}
#endif /* NFP_FDB_MODE */

static ssize_t do_bridge_clean(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int mode;
	unsigned int res = 0, err = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	res = sscanf(buf, "%d", &mode);
	if ((res < 1) || (mode != 1))
		goto bridge_err;
	nfp_bridge_clean();

bridge_out:
	return err ? -EINVAL : len;
bridge_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto bridge_out;
}

static ssize_t bridge_if_add_del(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, if_index, br_index;
	char bridge_name[NAME_SIZE], if_name[NAME_SIZE];
	const char *name = attr->attr.name;
	struct net_device *if_netdev = NULL, *br_netdev = NULL;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	res = sscanf(buf, "%s %s", bridge_name, if_name);
	if (res < 2) {
		err = 1;
		goto bridge_rule_out;
	}
	br_netdev = dev_get_by_name(&init_net, bridge_name);
	if_netdev = dev_get_by_name(&init_net, if_name);
	if (br_netdev == NULL || if_netdev == NULL) {
		err = 1;
		goto bridge_rule_out;
	}
	if_index = if_netdev->ifindex;
	br_index = br_netdev->ifindex;

	if (!strcmp(name, "if_add")) {
		if (res < 2) {
			err = 1;
			goto bridge_rule_out;
		}
		NFP_SYSFS_DBG(printk(KERN_INFO "nfp_if_to_bridge_add( %d , %d )\n", br_index, if_index));
		nfp_if_to_bridge_add(br_index, if_index);
	} else if (!strcmp(name, "if_del")) {
		NFP_SYSFS_DBG(printk(KERN_INFO "nfp_if_to_bridge_del( %d , %d )\n", br_index, if_index));
		nfp_if_to_bridge_del(br_index, if_index);
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}

bridge_rule_out:
	if (br_netdev)
		dev_put(br_netdev);
	if (if_netdev)
		dev_put(if_netdev);
	return err ? -EINVAL : len;
}

/* classification */
#if defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE)

static ssize_t bridge_classify(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, iif, val, vlan_prio;
	char iif_name[NAME_SIZE], daStr[MAC_SIZE], saStr[MAC_SIZE];
	MV_U8 da[MV_MAC_ADDR_SIZE], sa[MV_MAC_ADDR_SIZE];
	const char *name = attr->attr.name;
	struct net_device *netdev;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	res = sscanf(buf, "%s %s %s %d %d", saStr, daStr, iif_name, &val, &vlan_prio);
	if (res < 4)
		goto class_err;

	netdev = dev_get_by_name(&init_net, iif_name);
	if (netdev == NULL)
		goto class_err;

	iif = netdev->ifindex;
	dev_put(netdev);
	mvMacStrToHex(daStr, da);
	mvMacStrToHex(saStr, sa);

	if (!strcmp(name, "vlan_prio")) {
		if (res < 5)
			goto class_err;

		if (vlan_prio == -1) {
			NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_vlan_prio_del( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT
					" , %d , %d )\n", MV_MACQUAD(sa), MV_MACQUAD(da), iif, val));
			nfp_bridge_vlan_prio_del(sa, da, iif, val);
		} else {
			NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_vlan_prio_set( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT
					" , %d , %d , %d )\n", MV_MACQUAD(sa), MV_MACQUAD(da), iif, val, vlan_prio));
			nfp_bridge_vlan_prio_set(sa, da, iif, val, vlan_prio);
		}
	} else if (!strcmp(name, "vlan_prio_default")) {
		if (val == -1) {
			NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_vlan_prio_del( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT
					" , %d , %d )\n", MV_MACQUAD(sa), MV_MACQUAD(da), iif, val));
			nfp_bridge_vlan_prio_del(sa, da, iif, MV_ETH_NFP_GLOBAL_MAP);
		} else {
			NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_vlan_prio_set( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT
				" , %d , %d , %d )\n", MV_MACQUAD(sa), MV_MACQUAD(da), iif, MV_ETH_NFP_GLOBAL_MAP, val));
			nfp_bridge_vlan_prio_set(sa, da, iif, MV_ETH_NFP_GLOBAL_MAP, val);
		}
	} else if (!strcmp(name, "txq")) {
		if (val == -1) {
			NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_txq_del( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT
					" , %d )\n", MV_MACQUAD(sa), MV_MACQUAD(da), iif));
			nfp_bridge_txq_del(sa, da, iif);
		} else {
			NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_txq_set( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT
					" , %d , %d )\n", MV_MACQUAD(sa), MV_MACQUAD(da), iif, val));
			nfp_bridge_txq_set(sa, da, iif, val);
		}
	} else if (!strcmp(name, "txp")) {
		if (val == -1) {
			NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_txp_del( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT
					" , %d )\n", MV_MACQUAD(sa), MV_MACQUAD(da), iif));
			nfp_bridge_txp_del(sa, da, iif);
		} else {
			NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_txp_set( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT
					" , %d , %d )\n", MV_MACQUAD(sa), MV_MACQUAD(da), iif, val));
			nfp_bridge_txp_set(sa, da, iif, val);
		}
	} else if (!strcmp(name, "mh")) {
		if (val == -1) {
			NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_mh_del( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT
					" , %d )\n", MV_MACQUAD(sa), MV_MACQUAD(da), iif));
			nfp_bridge_mh_del(sa, da, iif);
		} else {
			NFP_SYSFS_DBG(printk(KERN_INFO "nfp_bridge_mh_set( "MV_MACQUAD_FMT" , "MV_MACQUAD_FMT
					" , %d , %d )\n", MV_MACQUAD(sa), MV_MACQUAD(da), iif, val));
			nfp_bridge_mh_set(sa, da, iif, val);
		}
	}

class_out:
	return err ? -EINVAL : len;
class_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto class_out;
}

static DEVICE_ATTR(vlan_prio, S_IWUSR, NULL, bridge_classify);
static DEVICE_ATTR(vlan_prio_default, S_IWUSR, NULL, bridge_classify);
static DEVICE_ATTR(txq, S_IWUSR, NULL, bridge_classify);
static DEVICE_ATTR(txp, S_IWUSR, NULL, bridge_classify);
static DEVICE_ATTR(mh, S_IWUSR, NULL, bridge_classify);
static DEVICE_ATTR(classify_help,   S_IRUSR, bridge_help, NULL);
#endif /* defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE) */
/* end of classification */

static DEVICE_ATTR(rule_add, S_IWUSR, NULL, bridge_add_del_age);
static DEVICE_ATTR(rule_del, S_IWUSR, NULL, bridge_add_del_age);
static DEVICE_ATTR(rule_age, S_IWUSR, NULL, bridge_add_del_age);

static DEVICE_ATTR(if_add, S_IWUSR, NULL, bridge_if_add_del);
static DEVICE_ATTR(if_del, S_IWUSR, NULL, bridge_if_add_del);
static DEVICE_ATTR(help,   S_IRUSR, bridge_help, NULL);

#ifdef NFP_FDB_LEARN
static DEVICE_ATTR(learn,  S_IWUSR, NULL, bridge_learn_age_sync);
static DEVICE_ATTR(age_enable,    S_IWUSR, NULL, bridge_learn_age_sync);
static DEVICE_ATTR(sync,   S_IWUSR, NULL, bridge_learn_age_sync);
#endif /* NFP_FDB_LEARN */
static DEVICE_ATTR(clean, S_IWUSR, NULL, do_bridge_clean);


static struct attribute *nfp_bridge_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_if_add.attr,
	&dev_attr_if_del.attr,
	&dev_attr_rule_add.attr,
	&dev_attr_rule_del.attr,
	&dev_attr_rule_age.attr,
#ifdef NFP_FDB_LEARN
	&dev_attr_learn.attr,
	&dev_attr_age_enable.attr,
	&dev_attr_sync.attr,
#endif /* NFP_FDB_LEARN */
	&dev_attr_clean.attr,
	NULL
};

static struct attribute_group nfp_bridge_group = {
	.attrs = nfp_bridge_attrs,
};


#if defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE)
static struct attribute *nfp_bridge_cls_attrs[] = {
	&dev_attr_vlan_prio.attr,
	&dev_attr_vlan_prio_default.attr,
	&dev_attr_txq.attr,
	&dev_attr_txp.attr,
	&dev_attr_mh.attr,
	&dev_attr_classify_help.attr,
	NULL
};
static struct attribute_group nfp_bridge_cls_group = {
	.attrs = nfp_bridge_cls_attrs,
	.name = "classify",
};
#endif /* defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE) */

int __init nfp_bridge_sysfs_init(struct kobject *nfp_kobj)
{
	int err;
	struct kobject *bridge_kobj;

	bridge_kobj = kobject_create_and_add("bridge", nfp_kobj);
	if (!bridge_kobj) {
		printk(KERN_INFO "could not create bridge kobject \n");
		return -ENOMEM;
	}

	err = sysfs_create_group(bridge_kobj, &nfp_bridge_group);
	if (err) {
		printk(KERN_INFO "bridge sysfs group failed %d\n", err);
		goto out;
	}
#if defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE)
	err = sysfs_create_group(bridge_kobj, &nfp_bridge_cls_group);
	if (err) {
		printk(KERN_INFO "bridge classify sysfs group failed %d\n", err);
		goto out;
	}
#endif /* defined(NFP_CLASSIFY) && !defined(NFP_FDB_MODE) */

out:
	return err;
}
