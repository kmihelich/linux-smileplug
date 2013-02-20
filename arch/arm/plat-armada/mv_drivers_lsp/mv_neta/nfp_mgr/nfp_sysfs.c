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

#include <linux/module.h>
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


static ssize_t nfp_help(char *buf)
{
	int off = 0;

	off += mvOsSPrintf(buf+off, "cat                     help         - print this help\n");
	off += mvOsSPrintf(buf+off, "cat                     stats        - print NFP_MGR statistics\n");
	off += mvOsSPrintf(buf+off, "cat                     status       - print NFP_MGR status\n");
	off += mvOsSPrintf(buf+off, "cat                     dump         - print NFP databases\n");
	off += mvOsSPrintf(buf+off, "echo [0 | 1 | 2 | 3]    > dbg        - no / dbg / warn / dbg + warn prints\n");
	off += mvOsSPrintf(buf+off, "echo [0 | 1]            > nfp        - NFP state: OFF / ON\n");
	off += mvOsSPrintf(buf+off, "echo [0 | 1]            > mode       - NFP mode: 2 tuple / 5 tuple\n");
	off += mvOsSPrintf(buf+off, "echo [0 | 1]            > data_path  - NFP packet processing: disable / enable\n");
#ifdef NFP_LEARN
	off += mvOsSPrintf(buf+off, "echo [0 | 1]            > learn      - disble/enable NFP dynamic learning\n");
	off += mvOsSPrintf(buf+off, "echo [0 | 1]            > age        - disble/enable NFP aging for\n");
	off += mvOsSPrintf(buf+off, "echo 1                  > sync       - sync Linux database to NFP database\n");
#endif /* NFP_LEARN */
	off += mvOsSPrintf(buf+off, "echo 1                  > clean      - clean NFP data base\n");
	off += mvOsSPrintf(buf+off, "echo port               > pstats     - print NFP port statistics\n");
	off += mvOsSPrintf(buf+off, "echo if_name            > bind       - bind real interface <if_name> to NFP processing\n");
	off += mvOsSPrintf(buf+off, "echo if_name            > unbind     - unbind real interface <if_name> from NFP processing\n");
	off += mvOsSPrintf(buf+off, "echo if_name mac        > mac        - set a MAC address to an interface with <if_name>\n");
	off += mvOsSPrintf(buf+off, "echo if_name mtu        > mtu        - set a MTU to an interface with <if_name>\n");
	off += mvOsSPrintf(buf+off, "echo if_name if_virt    > virt_add   - add virtual interface <if_virt> to <if_name>\n");
	off += mvOsSPrintf(buf+off, "echo if_virt            > virt_del   - remove virtual interface <if_virt>\n");
	off += mvOsSPrintf(buf+off, "echo if_name            > flush      - flush rules associated with if_name\n");
#ifdef CONFIG_MV_ETH_NFP_HWF
	off += mvOsSPrintf(buf+off, "echo rxp p txp txq      > hwf        - ");
	off += mvOsSPrintf(buf+off, "use <txp/txq> for NFP HWF flows from <rxp> to <p>\n");
#endif /* CONFIG_MV_ETH_NFP_HWF */
#ifdef NFP_LIMIT
	off += mvOsSPrintf(buf+off, "cat                     tbf_dump     - print info about all existing tbfs\n");
	off += mvOsSPrintf(buf+off, "echo limit burst_limit  > tbf_create - create a new token buffer with the specified limit and\n");
	off += mvOsSPrintf(buf+off, "                                       burst limit, and print the index of the new tbf.\n");
	off += mvOsSPrintf(buf+off, "echo tbf_index          > tbf_delete - if there are no ingress limit rules attached to it\n");
	off += mvOsSPrintf(buf+off, "                                       delete the tbf matched by the specifed index.\n");
#endif /* NFP_LIMIT */
	return off;
}

static ssize_t nfp_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const char *name = attr->attr.name;
	int off = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "dump"))
		nfp_mgr_dump();
	else if (!strcmp(name, "stats"))
		nfp_mgr_stats();
	else if (!strcmp(name, "status"))
		nfp_mgr_status();
#ifdef NFP_LIMIT
	else if (!strcmp(name, "tbf_dump"))
		nfp_mgr_tbfs_dump();
#endif /* NFP_LIMIT */
	else
		off = nfp_help(buf);

	return off;
}

static ssize_t nfp_store(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int err, a, b, c, d;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = b = c = d = err = 0;
	sscanf(buf, "%x %x %x %x", &a, &b, &c, &d);

	if (!strcmp(name, "pstats"))
		mvNfpStats(a);
	else if (!strcmp(name, "dbg"))
		mvNfpDebugLevelSet(a);
	else if (!strcmp(name, "nfp"))
		nfp_mgr_enable(a);
	else if (!strcmp(name, "mode"))
		nfp_mgr_mode_set(a);
	else if (!strcmp(name, "data_path"))
		nfp_mgr_data_path_enable(a);
	else if (!strcmp(name, "clean"))
		nfp_mgr_clean();
#ifdef NFP_LEARN
	else if (!strcmp(name, "learn"))
		nfp_learn_enable(a);
	else if (!strcmp(name, "age"))
		nfp_age_enable(a);
	else if (!strcmp(name, "sync"))
		nfp_hook_sync();
#endif /* NFP_LEARN */
	else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}

	return (err ? (-EINVAL) : len);
}

#ifdef CONFIG_MV_ETH_NFP_HWF
static ssize_t nfp_hwf_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t len)
{
	const char              *name = attr->attr.name;
	int			err = 0, rxp = 0, p = 0, txp = 0, txq = 0;
	int			hwf_txp, hwf_txq;
	unsigned long		flags;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	sscanf(buf, "%d %d %d %d", &rxp, &p, &txp, &txq);

	local_irq_save(flags);

	if (!strcmp(name, "hwf")) {
		if (txq == -1) {
			/* Disable NFP HWF from rxp to p. Free ownership of txp/txq */
			nfp_hwf_txq_get(rxp, p, &hwf_txp, &hwf_txq);

			if (hwf_txq != -1) {
				mv_eth_ctrl_txq_hwf_own(p, hwf_txp, hwf_txq, -1);
				nfp_hwf_txq_set(rxp, p, -1, -1);
			}
			mvNetaHwfTxqEnable(rxp, p, hwf_txp, hwf_txq, 0);
		} else {
			/* Enable NFP HWF from rxp to p. Set txp/txq ownership to HWF from rxp */
			err = mv_eth_ctrl_txq_hwf_own(p, txp, txq, rxp);
			if (err) {
				printk(KERN_ERR "%s failed: p=%d, txp=%d, txq=%d\n",
					__func__, p, txp, txq);
			} else {
				nfp_hwf_txq_set(rxp, p, txp, txq);
				mvNetaHwfTxqEnable(rxp, p, txp, txq, 1);
			}
		}
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}

	local_irq_restore(flags);

	return err ? -EINVAL : len;
}
#endif /* CONFIG_MV_ETH_NFP_HWF */



/* Bind / Unbind interface to / from NFP */
static ssize_t nfp_if_name_store(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int err = 0, res = 0, if_index, virt_index;
	const char  *name = attr->attr.name;
	char if_name[NAME_SIZE], virt_name[NAME_SIZE];
	struct net_device *netdev;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	memset(if_name, 0, NAME_SIZE);
	memset(virt_name, 0, NAME_SIZE);

	res = sscanf(buf, "%s %s", if_name, virt_name);
	if (res < 1) {
		err = 1;
		goto nfp_bind_out;
	}

	netdev = dev_get_by_name(&init_net, if_name);
	if (netdev == NULL) {
		err = 1;
		goto nfp_bind_out;
	}
	if_index = netdev->ifindex;
	dev_put(netdev);

	if (!strcmp(name, "bind")) {
		NFP_SYSFS_DBG(printk(KERN_INFO "nfp_mgr_if_bind( %d )\n", if_index));
		nfp_mgr_if_bind(if_index);
	} else if (!strcmp(name, "unbind")) {
		NFP_SYSFS_DBG(printk(KERN_INFO "nfp_mgr_if_unbind( %d )\n", if_index));
		nfp_mgr_if_unbind(if_index);
	} else if (!strcmp(name, "virt_del")) {
		nfp_mgr_if_virt_del(if_index);
	} else if (!strcmp(name, "flush")) {
		nfp_mgr_flush(if_index);
	} else if (!strcmp(name, "virt_add")) {
		if (res < 2) {
			err = 1;
			goto nfp_bind_out;
		}
		netdev = dev_get_by_name(&init_net, virt_name);
		if (netdev == NULL) {
			err = 1;
			goto nfp_bind_out;
		}
		virt_index = netdev->ifindex;
		dev_put(netdev);
		nfp_mgr_if_virt_add(if_index, virt_index);
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}

nfp_bind_out:
	return err ? -EINVAL : len;
}

static ssize_t nfp_if_mac_store(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int err = 0, res = 0, if_index;
	const char   *name = attr->attr.name;
	char         if_name[NAME_SIZE], mac_str[MAC_SIZE];
	MV_U8        mac[MV_MAC_ADDR_SIZE];
	struct net_device *netdev;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	memset(if_name, 0, NAME_SIZE);
	memset(mac_str, 0, MAC_SIZE);

	res = sscanf(buf, "%s %s", if_name, mac_str);
	if (res < 2)
		return -EINVAL;

	netdev = dev_get_by_name(&init_net, if_name);
	if (netdev == NULL)
		return -EINVAL;

	if_index = netdev->ifindex;
	dev_put(netdev);
	if (!strcmp(name, "mac")) {
		mvMacStrToHex(mac_str, mac);
		err = nfp_mgr_if_mac_set(if_index, mac);
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	return err ? -EINVAL : len;
}

static ssize_t nfp_if_dec_store(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int err = 0, res = 0, if_index, mtu;
	const char   *name = attr->attr.name;
	char         if_name[NAME_SIZE];

	struct net_device *netdev;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	memset(if_name, 0, NAME_SIZE);

	res = sscanf(buf, "%s %d", if_name, &mtu);
	if (res < 2)
		return -EINVAL;

	netdev = dev_get_by_name(&init_net, if_name);
	if (netdev == NULL)
		return -EINVAL;

	if_index = netdev->ifindex;
	dev_put(netdev);
	if (!strcmp(name, "mtu")) {
		err = nfp_mgr_if_mtu_set(if_index, mtu);
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	return err ? -EINVAL : len;
}

#ifdef NFP_LIMIT
static ssize_t nfp_tbf(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, limit, burst, tbfIndex = 0;
	const char *name = attr->attr.name;
	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	if (!strcmp(name, "tbf_create")) {
		res = sscanf(buf, "%d %d", &limit, &burst);
		if (res < 2)
			goto tbf_err;
		NFP_SYSFS_DBG(printk(KERN_INFO "nfp_tbf_create( %d , %d )\n", limit, burst));
		tbfIndex = nfp_tbf_create(limit, burst);
		if (tbfIndex != -1)
			printk(KERN_INFO "%s: tbf created with index of (%d)\n", attr->attr.name, tbfIndex);
		else
			printk(KERN_INFO "%s: could not create new tbf\n", attr->attr.name);
	} else {
		res = sscanf(buf, "%d", &tbfIndex);
		if (res < 1)
			goto tbf_err;
		NFP_SYSFS_DBG(printk(KERN_INFO "nfp_tbf_delete( %d )\n", tbfIndex));
		nfp_tbf_del(tbfIndex);
	}
tbf_out:
	return err ? -EINVAL : len;
tbf_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto tbf_out;
}
#endif /* NFP_LIMIT */


#ifdef CONFIG_MV_ETH_NFP_HWF
static DEVICE_ATTR(hwf, S_IWUSR, nfp_show, nfp_hwf_store);
#endif /* CONFIG_MV_ETH_NFP_HWF */

static DEVICE_ATTR(bind,     S_IWUSR, NULL, nfp_if_name_store);
static DEVICE_ATTR(unbind,   S_IWUSR, NULL, nfp_if_name_store);
static DEVICE_ATTR(virt_add, S_IWUSR, NULL, nfp_if_name_store);
static DEVICE_ATTR(virt_del, S_IWUSR, NULL, nfp_if_name_store);
static DEVICE_ATTR(mac,      S_IWUSR, NULL, nfp_if_mac_store);
static DEVICE_ATTR(mtu,      S_IWUSR, NULL, nfp_if_dec_store);
#ifdef NFP_LEARN
static DEVICE_ATTR(learn,    S_IWUSR, NULL, nfp_store);
static DEVICE_ATTR(age,      S_IWUSR, NULL, nfp_store);
static DEVICE_ATTR(sync,     S_IWUSR, nfp_show, nfp_store);
#endif /* NFP_LEARN */

static DEVICE_ATTR(flush,   S_IWUSR, nfp_show, nfp_if_name_store);
#ifdef NFP_LIMIT
static DEVICE_ATTR(tbf_dump,  S_IRUSR, nfp_show, nfp_store);
static DEVICE_ATTR(tbf_create, S_IWUSR, NULL, nfp_tbf);
static DEVICE_ATTR(tbf_delete, S_IWUSR, NULL, nfp_tbf);
#endif /* NFP_LIMIT */
static DEVICE_ATTR(dump,   S_IRUSR, nfp_show, nfp_store);
static DEVICE_ATTR(stats,  S_IRUSR, nfp_show, nfp_store);
static DEVICE_ATTR(status, S_IRUSR, nfp_show, nfp_store);
static DEVICE_ATTR(pstats, S_IWUSR, nfp_show, nfp_store);
static DEVICE_ATTR(nfp,    S_IWUSR, nfp_show, nfp_store);
static DEVICE_ATTR(clean,  S_IWUSR, nfp_show, nfp_store);
static DEVICE_ATTR(mode,   S_IWUSR, nfp_show, nfp_store);
static DEVICE_ATTR(data_path, S_IWUSR, nfp_show, nfp_store);
static DEVICE_ATTR(dbg,    S_IWUSR, nfp_show, nfp_store);
static DEVICE_ATTR(help,   S_IRUSR, nfp_show, nfp_store);

static struct attribute *nfp_attrs[] = {
#ifdef CONFIG_MV_ETH_NFP_HWF
	&dev_attr_hwf.attr,
#endif /* CONFIG_MV_ETH_NFP_HWF */
#ifdef NFP_LIMIT
	&dev_attr_tbf_dump.attr,
	&dev_attr_tbf_create.attr,
	&dev_attr_tbf_delete.attr,
#endif /* NFP_LIMIT */
#ifdef NFP_LEARN
	&dev_attr_learn.attr,
	&dev_attr_age.attr,
	&dev_attr_sync.attr,
#endif /* NFP_LEARN */
	&dev_attr_dump.attr,
	&dev_attr_stats.attr,
	&dev_attr_status.attr,
	&dev_attr_pstats.attr,
	&dev_attr_nfp.attr,
	&dev_attr_clean.attr,
	&dev_attr_mode.attr,
	&dev_attr_data_path.attr,
	&dev_attr_dbg.attr,
	&dev_attr_help.attr,
	&dev_attr_unbind.attr,
	&dev_attr_bind.attr,
	&dev_attr_virt_add.attr,
	&dev_attr_virt_del.attr,
	&dev_attr_mac.attr,
	&dev_attr_mtu.attr,
	&dev_attr_flush.attr,
	NULL
};


static struct attribute_group nfp_group = {
	.attrs = nfp_attrs,
};


int nfp_sysfs_init(void)
{
	int err;
	struct device  *pd;
	struct kobject *nfp_kobj;

	pd = bus_find_device_by_name(&platform_bus_type, NULL, "neta");
	if (!pd) {
		platform_device_register_simple("neta", -1, NULL, 0);
		pd = bus_find_device_by_name(&platform_bus_type, NULL, "neta");
	}

	if (!pd) {
		printk(KERN_ERR"%s: cannot find neta device\n", __func__);
		pd = &platform_bus;
	}

	nfp_kobj = kobject_create_and_add("nfp", &pd->kobj);
	if (!nfp_kobj) {
		printk(KERN_INFO "could not create nfp kobject\n");
		return -ENOMEM;
	}

	err = sysfs_create_group(nfp_kobj, &nfp_group);
	if (err) {
		printk(KERN_INFO "create sysfs nfs group failed %d\n", err);
		goto out;
	}
#ifdef NFP_BRIDGE
	nfp_bridge_sysfs_init(nfp_kobj);
#endif /* NFP_BRIDGE */

#ifdef NFP_VLAN
	nfp_vlan_sysfs_init(nfp_kobj);
#endif /* NFP_VLAN */

#ifdef NFP_FIB
	nfp_fib_sysfs_init(nfp_kobj);
#ifdef CONFIG_IPV6
	nfp_fib6_sysfs_init(nfp_kobj);
#endif /* CONFIG_IPV6 */
#endif /* NFP_FIB */

#ifdef NFP_CT
	nfp_ct_sysfs_init(nfp_kobj);
#ifdef CONFIG_IPV6
	nfp_ct6_sysfs_init(nfp_kobj);
#endif /* CONFIG_IPV6 */
#endif
#ifdef NFP_CLASSIFY
	nfp_cls_sysfs_init(nfp_kobj);
#endif
#ifdef NFP_PPP
	nfp_ppp_sysfs_init(nfp_kobj);
#endif
out:
	return err;
}


MODULE_AUTHOR("Kostya Belezko");
MODULE_DESCRIPTION("NFP API");
MODULE_LICENSE("GPL");

