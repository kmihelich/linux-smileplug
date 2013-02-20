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

#ifdef CONFIG_IPV6

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


static ssize_t fib_help(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int off = 0;
	PRINT("cat                      help    - print this help\n");
	PRINT("echo sip dip oif gtw   > fib_add - add a new 2 tuple routing rule\n");
	PRINT("echo sip dip oif       > fib_add - add a new 2 tuple routing rule, use dip as gateway\n");
	PRINT("echo sip dip           > fib_del - delete an existing 2 tuple routing rule\n");
	PRINT("echo sip dip           > fib_age - print the number of packets processed by this\n");
	PRINT("                                   rule since the last call of this function\n");
	PRINT("echo gtw mac           > arp_add - add a new ARP rule\n");
	PRINT("echo gtw               > arp_del - delete an existing ARP rule\n");
	PRINT("echo gtw               > arp_age - print 1 if there are FIB rules based on this ARP\n");
	PRINT("                                   rule, and 0 if there are none\n");
#ifdef NFP_FIB_LEARN
	PRINT("echo [0 | 1]            > learn      - enable/disble NFP routing dynamic learning\n");
	PRINT("echo [0 | 1]            > age_enable - enable/disble NFP routing aging\n");
	PRINT("echo 1                  > sync       - sync Linux fib database to NFP database\n");
#endif
	PRINT("echo mode               > fib_clean - clean NFP FIB DB. \n");
	PRINT("echo mode               > arp_clean - clean NFP ARP DB. \n");
	PRINT("							 1 - perform clean operation \n");
	PRINT("\nParameters: sip = source ip, dip = destination ip\n");
	PRINT("            oif = egress interface name, gtw=gateway ip, mac=MAC address\n\n");
	return off;
}


static ssize_t fib_add_del_age(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0;
	unsigned char sip[16], dip[16], gtw[16], *gateway = dip;
	char oif[NAME_SIZE], ipv6StrSip[IPV6_MAX_SIZE], ipv6StrDip[IPV6_MAX_SIZE], ipv6StrGtw[IPV6_MAX_SIZE];
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	memset(oif, 0, NAME_SIZE);

	memset(ipv6StrSip, 0, sizeof(ipv6StrSip));
	memset(ipv6StrDip, 0, sizeof(ipv6StrDip));
	memset(ipv6StrGtw, 0, sizeof(ipv6StrGtw));
	res = sscanf(buf, "%s %s %s %s", ipv6StrSip, ipv6StrDip, oif, ipv6StrGtw);
	if (res < 3) {
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
		err = 1;
		goto fib_out;
	}
	/* parse addresses */
	ipv6Parser(ipv6StrSip, sip);
	ipv6Parser(ipv6StrDip, dip);
	if (res == 4) {
		ipv6Parser(ipv6StrGtw, gtw);
		gateway = gtw;
	}

	err = do_fib_add_del_age(name, AF_INET6, sip, dip, oif, gateway);
fib_out:
	return err ? -EINVAL : len;
}

static ssize_t do_fib_clean(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int mode;
	unsigned int res = 0, err = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	res = sscanf(buf, "%d", &mode);
	if ((res < 1) || (mode != 1))
		goto fib_err;

	nfp_clean_fib(MV_INET6);

fib_out:
	return err ? -EINVAL : len;
fib_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto fib_out;
}

static ssize_t do_arp_clean(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int mode;
	unsigned int res = 0, err = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	res = sscanf(buf, "%d", &mode);
	if ((res < 1) || (mode != 1))
		goto arp_err;

	nfp_clean_arp(MV_INET6);

arp_out:
	return err ? -EINVAL : len;
arp_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto arp_out;
}


static ssize_t arp_add_del_age(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, isArpAdd;
	unsigned char gtw[16];
	char macStr[MAC_SIZE], gtwStrIp6[IPV6_MAX_SIZE];
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	memset(macStr, 0, MAC_SIZE);
	memset(gtwStrIp6, 0, sizeof(gtwStrIp6));
	isArpAdd = (!strcmp(name, "arp_add"));

	res = sscanf(buf, "%s %s", gtwStrIp6, macStr);
	if ((isArpAdd && res < 2) || (!isArpAdd && res < 1)) {
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
		err = 1;
		goto arp_out;
	}
	ipv6Parser(gtwStrIp6, gtw);
	err = do_arp_add_del_age(name, AF_INET6, gtw, macStr);
arp_out:
	return err ? -EINVAL : len;
}

#ifdef NFP_FIB_LEARN
static ssize_t fib6_learn_age_sync(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int err, a;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = err = 0;
	sscanf(buf, "%x", &a);

	if (!strcmp(name, "learn")) {
		nfp_fib6_learn_enable(a);
	} else if (!strcmp(name, "age_enable")) {
		nfp_fib6_age_enable(a);
	} else if (!strcmp(name, "sync")) {
		neigh_sync(MV_INET6);
		nfp_fib6_sync();
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	return err ? -EINVAL : len;
}
#endif /* NFP_FIB_LEARN */

static DEVICE_ATTR(arp_del, S_IWUSR, NULL, arp_add_del_age);
static DEVICE_ATTR(arp_age, S_IWUSR, NULL, arp_add_del_age);
static DEVICE_ATTR(arp_add, S_IWUSR, NULL, arp_add_del_age);

static DEVICE_ATTR(fib_age, S_IWUSR, NULL, fib_add_del_age);
static DEVICE_ATTR(fib_add, S_IWUSR, NULL, fib_add_del_age);
static DEVICE_ATTR(fib_del, S_IWUSR, NULL, fib_add_del_age);
#ifdef NFP_FIB_LEARN
static DEVICE_ATTR(learn,  S_IWUSR, NULL, fib6_learn_age_sync);
static DEVICE_ATTR(age_enable,    S_IWUSR, NULL, fib6_learn_age_sync);
static DEVICE_ATTR(sync,   S_IWUSR, NULL, fib6_learn_age_sync);
#endif /* NFP_FIB_LEARN */
static DEVICE_ATTR(fib_clean, S_IWUSR, NULL, do_fib_clean);
static DEVICE_ATTR(arp_clean, S_IWUSR, NULL, do_arp_clean);
static DEVICE_ATTR(help,   S_IRUSR, fib_help, NULL);


static struct attribute *nfp_fib6_attrs[] = {
	&dev_attr_fib_add.attr,
	&dev_attr_fib_del.attr,
	&dev_attr_fib_age.attr,
	&dev_attr_arp_add.attr,
	&dev_attr_arp_del.attr,
	&dev_attr_arp_age.attr,
#ifdef NFP_FIB_LEARN
	&dev_attr_learn.attr,
	&dev_attr_age_enable.attr,
	&dev_attr_sync.attr,
#endif /* NFP_FIB_LEARN */
	&dev_attr_fib_clean.attr,
	&dev_attr_arp_clean.attr,
	&dev_attr_help.attr,
	NULL
};
static struct attribute_group nfp_fib6_group = {
	.attrs = nfp_fib6_attrs,
};



int __init nfp_fib6_sysfs_init(struct kobject *nfp_kobj)
{
		int err;
		struct kobject *fib_kobj = NULL;
		fib_kobj = kobject_create_and_add("fib6", nfp_kobj);

		if (!fib_kobj) {
			printk(KERN_INFO "could not create fib6 kobject \n");
			return -ENOMEM;
		}

		err = sysfs_create_group(fib_kobj, &nfp_fib6_group);
		if (err) {
			printk(KERN_INFO "fib6 sysfs group failed %d\n", err);
			goto out;
		}

out:
		return err;
}
#endif /* CONFIG_IPV6 */
