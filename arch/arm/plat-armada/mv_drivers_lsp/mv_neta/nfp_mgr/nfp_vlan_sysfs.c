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




static ssize_t vlan_help(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int off = 0;

	off += mvOsSPrintf(buf+off, "cat                help      - print this help\n");
	off += mvOsSPrintf(buf+off, "echo if_name pvid  > pvid    - set PVID for ingress interface\n");
	off += mvOsSPrintf(buf+off, "echo if_name vid   > vid     - set VID for VLAN interface\n");
	off += mvOsSPrintf(buf+off, "echo if_name mode  > rx_mode - set VLAN <mode> for ingress interface\n");
	off += mvOsSPrintf(buf+off, "                             0 - transparent, 1 - drop untagged, 2 - drop tagged,\n");
	off += mvOsSPrintf(buf+off, "                             3 - drop unknown, 4 - drop untagged and unknown\n");
	off += mvOsSPrintf(buf+off, "echo if_name mode  > tx_mode - set VLAN <mode> for egress interface\n");
	off += mvOsSPrintf(buf+off, "                             0 - transparent, 1 - send untagged, 2 - send tagged\n");
#ifdef NFP_VLAN_LEARN
	off += mvOsSPrintf(buf+off, "echo [0 | 1]       > learn   - enable/disble NFP vlan dynamic learning\n");
	off += mvOsSPrintf(buf+off, "echo 1             > sync    - sync Linux vlan database to NFP database\n");
#endif

	return off;
}



static ssize_t vlan_store(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, if_index, val;
	char if_name[NAME_SIZE];
	const char *name = attr->attr.name;
	struct net_device *netdev = NULL;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	res = sscanf(buf, "%s %d", if_name, &val);
	if (res < 2) {
		printk(KERN_ERR "%s: Wrong arguments number %d\n", __func__, res);
		return -EINVAL;
	}
	netdev = dev_get_by_name(&init_net, if_name);
	if (netdev == NULL) {
		err = 1;
		return -EINVAL;
	}
	dev_put(netdev);

	if_index = netdev->ifindex;

	if (!strcmp(name, "pvid")) {
		err = nfp_vlan_pvid_set(if_index, (u16)val);
	} else if (!strcmp(name, "vid")) {
		err = nfp_vlan_vid_set(if_index, (u16)val);
	} else if (!strcmp(name, "rx_mode")) {
		err = nfp_vlan_rx_mode_set(if_index, val);
	} else if (!strcmp(name, "tx_mode")) {
		err = nfp_vlan_tx_mode_set(if_index, val);
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	return err ? -EINVAL : len;
}

#ifdef NFP_VLAN_LEARN
static ssize_t vlan_learn_sync(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int err, a;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = err = 0;
	sscanf(buf, "%x", &a);

	if (!strcmp(name, "learn")) {
		nfp_vlan_learn_enable(a);
	} else if (!strcmp(name, "sync")) {
		vlan_sync();
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	return err ? -EINVAL : len;
}
#endif /* NFP_VLAN_LEARN */

static DEVICE_ATTR(pvid,    S_IWUSR, NULL, vlan_store);
static DEVICE_ATTR(vid,     S_IWUSR, NULL, vlan_store);
static DEVICE_ATTR(rx_mode, S_IWUSR, NULL, vlan_store);
static DEVICE_ATTR(tx_mode, S_IWUSR, NULL, vlan_store);
static DEVICE_ATTR(help,    S_IRUSR, vlan_help, NULL);
#ifdef NFP_VLAN_LEARN
static DEVICE_ATTR(learn, S_IWUSR, NULL, vlan_learn_sync);
static DEVICE_ATTR(sync,  S_IWUSR, NULL, vlan_learn_sync);
#endif /* NFP_VLAN_LEARN */



static struct attribute *nfp_vlan_attrs[] = {
	&dev_attr_pvid.attr,
	&dev_attr_vid.attr,
	&dev_attr_rx_mode.attr,
	&dev_attr_tx_mode.attr,
	&dev_attr_help.attr,
#ifdef NFP_VLAN_LEARN
	&dev_attr_learn.attr,
	&dev_attr_sync.attr,
#endif /* NFP_VLAN_LEARN */
	NULL
};

static struct attribute_group nfp_vlan_group = {
	.attrs = nfp_vlan_attrs,
};

int __init nfp_vlan_sysfs_init(struct kobject *nfp_kobj)
{
	int err;
	struct kobject *vlan_kobj;

	vlan_kobj = kobject_create_and_add("vlan", nfp_kobj);
	if (!vlan_kobj) {
		printk(KERN_INFO "could not create vlan kobject \n");
		return -ENOMEM;
	}

	err = sysfs_create_group(vlan_kobj, &nfp_vlan_group);
	if (err) {
		printk(KERN_INFO "vlan sysfs group failed %d\n", err);
		goto out;
	}
out:
	return err;
}
