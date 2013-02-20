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


static ssize_t ppp_help(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int off = 0;

	off += mvOsSPrintf(buf+off, "cat                    help    - print this help\n");
	off += mvOsSPrintf(buf+off, "echo if_ppp sid mac  > ppp_set - create PPPoE channel over <if_ppp>\n");
	off += mvOsSPrintf(buf+off, "echo if_ppp          > ppp_del - delete PPPoE channel over <if_ppp>\n");
#ifdef NFP_PPP_LEARN
	off += mvOsSPrintf(buf+off, "echo [0 | 1]         > learn   - enable/disble NFP pppoe dynamic learning\n");
#endif
	return off;
}



static ssize_t ppp_store(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0;
	unsigned int ppp_index=0;
	const char *name = attr->attr.name;

	unsigned int sid;
	char remoteMac[MAC_SIZE];
	MV_U8 mac[MV_MAC_ADDR_SIZE];
	char ppp_name[NAME_SIZE];
	struct net_device *netdev;

	memset(ppp_name,  0, NAME_SIZE);
	memset(remoteMac, 0, MAC_SIZE);

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	res = sscanf(buf, "%s %d %s", ppp_name, &sid, remoteMac);

	if (res >= 1) {
		netdev = dev_get_by_name(&init_net, ppp_name);
		if (netdev == NULL) {
			printk("%s: Unknown netdevice %s\n", __func__, ppp_name);
			err = 1;
			goto nfp_bind_out;
		}
		ppp_index = netdev->ifindex;
		dev_put(netdev);
	}

	if (!strcmp(name, "ppp_set")) {
		mvMacStrToHex(remoteMac, mac);
		NFP_SYSFS_DBG(printk(KERN_ERR "ppp_add : ppp_index=%d mac="MV_MACQUAD_FMT" \n", ppp_index, MV_MACQUAD(mac)));
		err = nfp_ppp_add(ppp_index, (u16)sid, mac);

	} else if (!strcmp(name, "ppp_del")) {
		NFP_SYSFS_DBG(printk(KERN_ERR "calling nfp_ppp_del ppp_index=%d\n", ppp_index));
		err = nfp_ppp_del(ppp_index);
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
nfp_bind_out:

	return err ? -EINVAL : len;
}

#ifdef NFP_PPP_LEARN
static ssize_t ppp_learn(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int err, a;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = err = 0;
	sscanf(buf, "%x", &a);

	if (!strcmp(name, "learn")) {
		nfp_ppp_learn_enable(a);
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	return err ? -EINVAL : len;
}
#endif /* NFP_PPP_LEARN */

static DEVICE_ATTR(ppp_set, S_IWUSR, NULL, ppp_store);
static DEVICE_ATTR(ppp_del, S_IWUSR, NULL, ppp_store);
static DEVICE_ATTR(help,    S_IRUSR, ppp_help, NULL);
#ifdef NFP_PPP_LEARN
static DEVICE_ATTR(learn,  S_IWUSR, NULL, ppp_learn);
#endif /* NFP_PPP_LEARN */

static struct attribute *nfp_ppp_attrs[] = {
	&dev_attr_ppp_set.attr,
	&dev_attr_ppp_del.attr,
	&dev_attr_help.attr,
#ifdef NFP_PPP_LEARN
	&dev_attr_learn.attr,
#endif /* NFP_PPP_LEARN */
	NULL
};

static struct attribute_group nfp_ppp_group = {
	.attrs = nfp_ppp_attrs,
};

int __init nfp_ppp_sysfs_init(struct kobject *nfp_kobj)
{
	int err;
	struct kobject *ppp_kobj;

	ppp_kobj = kobject_create_and_add("ppp", nfp_kobj);
	if (!ppp_kobj) {
		printk(KERN_INFO "could not create ppp kobject \n");
		return -ENOMEM;
	}

	err = sysfs_create_group(ppp_kobj, &nfp_ppp_group);
	if (err) {
		printk(KERN_INFO "ppp sysfs group failed %d\n", err);
		goto out;
	}
out:
	return err;
}
