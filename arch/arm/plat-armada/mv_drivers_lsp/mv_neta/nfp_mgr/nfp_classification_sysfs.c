/*******************************************************************************
Copyright (C) Marvell Interclsional Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
Interclsional Ltd. and/or its affiliates ("Marvell") under the following
alterclsive licensing terms.  Once you have made an eleclsion to distribute the
File under one of the following license alterclsives, please (i) delete this
introduclsory statement regarding license alterclsives, (ii) delete the two
license alterclsives that you have not eleclsed to use and (iii) preserve the
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


static ssize_t cls_help(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int off = 0;
	off += mvOsSPrintf(buf+off, "cat                      help                 - print this help.\n");
	off += mvOsSPrintf(buf+off, "cat                      dscp_policy_get      - print policy of choosing dscp value.\n");
	off += mvOsSPrintf(buf+off, "cat                      vlan_prio_policy_get - print policy of choosing vlan priority value.\n");
	off += mvOsSPrintf(buf+off, "cat                      txq_policy_get       - print policy of choosing txq value.\n");
	off += mvOsSPrintf(buf+off, "cat                      txp_policy_get       - print policy of choosing txp value.\n");
	off += mvOsSPrintf(buf+off, "cat                      mh_policy_get        - print policy of choosing mh value.\n");
	off += mvOsSPrintf(buf+off, "echo [0 | 1 | 2 | 3]   > dscp_policy_set      - define policy of choosing dscp value.\n");
	off += mvOsSPrintf(buf+off, "echo [0 | 1 | 2 | 3]   > vlan_prio_policy_set - define policy of choosing vlan priority value.\n");
	off += mvOsSPrintf(buf+off, "echo [0 | 1 | 2 | 3]   > txq_policy_set       - define policy of choosing txq value.\n");
	off += mvOsSPrintf(buf+off, "echo [0 | 1 | 2 | 3]   > txp_policy_set       - define policy of choosing txp value.\n");
	off += mvOsSPrintf(buf+off, "echo [0 | 1 | 2 | 3]   > mh_policy_set        - define policy of choosing mh value.\n");

	off += mvOsSPrintf(buf+off, "\n\nParameters: 0 = highest , 1 = lowest , 2 = first , 3 = last.\n");
	return off;
}


static ssize_t cls_policy_set(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, mode;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	res = sscanf(buf, "%d", &mode);
	if (res < 1)
		goto cls_err;

	if (mode > MV_NFP_CLASSIFY_LAST || mode < MV_NFP_CLASSIFY_HIGHEST)
		goto cls_err;

	if (!strcmp(name, "dscp_policy_set"))
		nfp_dscp_policy_set(mode);
	else if (!strcmp(name, "vlan_prio_policy_set"))
		nfp_vlan_prio_policy_set(mode);
	else if (!strcmp(name, "txq_policy_set"))
		nfp_txq_policy_set(mode);
	else if (!strcmp(name, "txp_policy_set"))
		nfp_txp_policy_set(mode);
	else if (!strcmp(name, "mh_policy_set"))
		nfp_mh_policy_set(mode);

cls_out:
	return err ? -EINVAL : len;
cls_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto cls_out;
}

static ssize_t cls_policy_get(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int off = 0, mode = 0;
	const char *name = attr->attr.name, *str;
	if (!strcmp(name, "dscp_policy_get"))
		mode = nfp_dscp_policy_get();
	else if (!strcmp(name, "vlan_prio_policy_get"))
		mode = nfp_vlan_prio_policy_get();
	else if (!strcmp(name, "txq_policy_get"))
		mode = nfp_txq_policy_get();
	else if (!strcmp(name, "txp_policy_get"))
		mode = nfp_txp_policy_get();
	else if (!strcmp(name, "mh_policy_get"))
		mode = nfp_mh_policy_get();

	if (mode == 0)
		str = "highest";
	else if (mode == 1)
		str = "lowest";
	else if (mode == 2)
		str = "first";
	else
		str = "last";

	off += mvOsSPrintf(buf+off, "%s: %s\n", name, str);
	return off;
}


static DEVICE_ATTR(help, S_IRUSR, cls_help, NULL);
static DEVICE_ATTR(dscp_policy_set, S_IWUSR, NULL, cls_policy_set);
static DEVICE_ATTR(vlan_prio_policy_set, S_IWUSR, NULL, cls_policy_set);
static DEVICE_ATTR(txq_policy_set, S_IWUSR, NULL, cls_policy_set);
static DEVICE_ATTR(txp_policy_set, S_IWUSR, NULL, cls_policy_set);
static DEVICE_ATTR(mh_policy_set, S_IWUSR, NULL, cls_policy_set);
static DEVICE_ATTR(dscp_policy_get, S_IRUSR, cls_policy_get, NULL);
static DEVICE_ATTR(vlan_prio_policy_get, S_IRUSR, cls_policy_get, NULL);
static DEVICE_ATTR(txq_policy_get, S_IRUSR, cls_policy_get, NULL);
static DEVICE_ATTR(txp_policy_get, S_IRUSR, cls_policy_get, NULL);
static DEVICE_ATTR(mh_policy_get, S_IRUSR, cls_policy_get, NULL);


static struct attribute *nfp_cls_attrs[] = {
	&dev_attr_dscp_policy_set.attr,
	&dev_attr_vlan_prio_policy_set.attr,
	&dev_attr_txq_policy_set.attr,
	&dev_attr_txp_policy_set.attr,
	&dev_attr_mh_policy_set.attr,
	&dev_attr_dscp_policy_get.attr,
	&dev_attr_vlan_prio_policy_get.attr,
	&dev_attr_txq_policy_get.attr,
	&dev_attr_txp_policy_get.attr,
	&dev_attr_mh_policy_get.attr,
	&dev_attr_help.attr,
	NULL
};
static struct attribute_group nfp_cls_group = {
	.attrs = nfp_cls_attrs,
};


int __init nfp_cls_sysfs_init(struct kobject *nfp_kobj)
{
		int err;
		struct kobject *cls_kobj = NULL;

		cls_kobj = kobject_create_and_add("classification", nfp_kobj);
		if (!cls_kobj) {
			printk(KERN_INFO "could not create classification kobjecls \n");
			return -ENOMEM;
		}

		err = sysfs_create_group(cls_kobj, &nfp_cls_group);
		if (err) {
			printk(KERN_INFO "classification sysfs group failed %d\n", err);
			goto out;
		}

out:
		return err;
}
