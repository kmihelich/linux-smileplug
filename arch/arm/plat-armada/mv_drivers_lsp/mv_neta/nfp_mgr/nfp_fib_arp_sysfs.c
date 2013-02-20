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

/* ip printers */
void print_fib_add(int f, unsigned char *sip, unsigned char *dip, int oif, unsigned char *gtw)
{
	if (f == AF_INET) {
		printk(KERN_INFO "add_fib_rule(IPV4 , "MV_IPQUAD_FMT" , "MV_IPQUAD_FMT" , %d, "MV_IPQUAD_FMT" )\n",
			MV_IPQUAD(sip), MV_IPQUAD(dip), oif, MV_IPQUAD(gtw));
	} else {
		printk(KERN_INFO "add_fib_rule(IPV6 , "MV_IP6_FMT" , ", MV_IP6_ARG(sip));
		printk(KERN_INFO MV_IP6_FMT" , %d , ", MV_IP6_ARG(dip), oif);
		printk(KERN_INFO MV_IP6_FMT" )\n", MV_IP6_ARG(gtw));
	}
}

void print_fib_del_age(int f, unsigned char *sip, unsigned char *dip, char *name)
{
	if (f == AF_INET) {
		printk(KERN_INFO "%s_fib_rule(IPV4 , "MV_IPQUAD_FMT" , "MV_IPQUAD_FMT" )\n", name,
			MV_IPQUAD(sip), MV_IPQUAD(dip));
	} else {
		printk(KERN_INFO "%s_fib_rule(IPV6 , "MV_IP6_FMT" , ", name, MV_IP6_ARG(sip));
		printk(KERN_INFO MV_IP6_FMT")\n", MV_IP6_ARG(dip));
	}
}

void print_arp_add(int f, unsigned char *gtw, MV_U8 *mac)
{
	if (f == AF_INET) {
		printk(KERN_INFO "nfp_arp_add(IPV4, "MV_IPQUAD_FMT" , "MV_MACQUAD_FMT" )\n",
			MV_IPQUAD(gtw), MV_MACQUAD(mac));
	} else {
		printk(KERN_INFO "nfp_arp_add(IPV6, "MV_IP6_FMT" , "MV_MACQUAD_FMT" )\n",
			MV_IP6_ARG(gtw), MV_MACQUAD(mac));
	}
}

void print_arp_del_age(int f, unsigned char *gtw, char *name)
{
	if (f == AF_INET)
		printk(KERN_INFO "nfp_arp_%s(IPV4, "MV_IPQUAD_FMT" )\n", name, MV_IPQUAD(gtw));
	else
		printk(KERN_INFO "nfp_arp_%s(IPV6, "MV_IP6_FMT" )\n", name, MV_IP6_ARG(gtw));
}



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
	PRINT("echo mode               > fib_clean  - clean NFP FIB DB. \n");
	PRINT("echo mode               > arp_clean  - clean NFP ARP DB. \n");
	PRINT("\nParameters: sip = source ip, dip = destination ip\n");
	PRINT("            oif = egress interface name, gtw=gateway ip, mac=MAC address\n\n");
	PRINT("							 1 - perform clean operation \n");
	return off;
}

/* this function call nfp_mgr_functions, and print debug messeges */
int do_fib_add_del_age(const char *name, int family, unsigned char *sip,
				unsigned char *dip, char *oif, unsigned char *gateway)
{
	unsigned int oif_index = 0;
	struct net_device *netdev;
	if (!strcmp(name, "fib_add")) {
		netdev = dev_get_by_name(&init_net, oif);
		if (netdev == NULL) {
			NFP_SYSFS_DBG(printk(KERN_ERR "fib_add: can't find net_device\n"));
			return 1;
		}
		oif_index = netdev->ifindex;
		dev_put(netdev);
		NFP_SYSFS_DBG(print_fib_add(family, sip, dip, oif_index, gateway));
		nfp_fib_rule_add(family, sip, dip, gateway, oif_index);
	} else if (!strcmp(name, "fib_del")) {
		NFP_SYSFS_DBG(print_fib_del_age(family, sip, dip, "del"));
		nfp_fib_rule_del(family, sip, dip);
	} else { /* fib_age */
		NFP_SYSFS_DBG(print_fib_del_age(family, sip, dip, "age"));
		printk(KERN_INFO "age: %d\n", nfp_fib_rule_age(family, sip, dip));
	}
	return 0;
}

/* this function call nfp_mgr_functions, and print debug messeges */
int do_arp_add_del_age(const char *name, int family, unsigned char *gtw, char *macStr)
{
	MV_U8 mac[MV_MAC_ADDR_SIZE];
	if (!strcmp(name, "arp_add")) {
		mvMacStrToHex(macStr, mac);
		NFP_SYSFS_DBG(print_arp_add(family, gtw, mac));
		nfp_arp_add(family, gtw, mac);
	} else {

		if (!strcmp(name, "arp_del")) {
			NFP_SYSFS_DBG(print_arp_del_age(family, gtw, "del"));
			nfp_arp_del(family, gtw);
		} else {
			NFP_SYSFS_DBG(print_arp_del_age(family, gtw, "age"));
			printk(KERN_INFO "age: %d\n", nfp_arp_age(family, gtw));
		}
	}
	return 0;
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

	nfp_clean_fib(MV_INET);

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

	nfp_clean_arp(MV_INET);

arp_out:
	return err ? -EINVAL : len;
arp_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto arp_out;
}

/* sysfs handle */
#define SCAN_FAIL(x, r) (((r < 9 || (r > 9 && r < 13)) && !strcmp(x, "fib_add")) \
				|| (r < 8 && strcmp(x, "fib_add")))
static ssize_t fib_add_del_age(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0;
	unsigned char sip[16], dip[16], gtw[16], *gateway = dip;
	char oif[NAME_SIZE];
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	memset(oif, 0, NAME_SIZE);
	res = sscanf(buf, "%hhu.%hhu.%hhu.%hhu %hhu.%hhu.%hhu.%hhu %s %hhu.%hhu.%hhu.%hhu",
			sip, sip+1, sip+2, sip+3, dip, dip+1, dip+2, dip+3, oif, gtw, gtw+1, gtw+2, gtw+3);

	if (SCAN_FAIL(name, res)) {
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
		err = 1;
		goto fib_out;
	}
	if (res == 13)
		gateway = gtw;

	err = do_fib_add_del_age(name, AF_INET, sip, dip, oif, gateway);
fib_out:
	return err ? -EINVAL : len;
}


/* sysfs handle */
static ssize_t arp_add_del_age(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, isArpAdd;
	unsigned char gtw[16];
	char macStr[MAC_SIZE];
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	memset(macStr, 0, MAC_SIZE);
	isArpAdd = (!strcmp(name, "arp_add"));

	res = sscanf(buf, "%hhu.%hhu.%hhu.%hhu %s", gtw, gtw+1, gtw+2, gtw+3, macStr);
	if ((isArpAdd && res < 5) || (!isArpAdd && res < 4)) {
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
		err = 1;
		goto arp_out;
	}

	err = do_arp_add_del_age(name, AF_INET, gtw, macStr);
arp_out:
	return err ? -EINVAL : len;
}

#ifdef NFP_FIB_LEARN
static ssize_t fib_learn_age_sync(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int err, a;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = err = 0;
	sscanf(buf, "%x", &a);

	if (!strcmp(name, "learn")) {
		nfp_fib_learn_enable(a);
	} else if (!strcmp(name, "age_enable")) {
		nfp_fib_age_enable(a);
	} else if (!strcmp(name, "sync")) {
		neigh_sync(MV_INET);
		nfp_fib_sync();
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

static DEVICE_ATTR(help,   S_IRUSR, fib_help, NULL);
#ifdef NFP_FIB_LEARN
static DEVICE_ATTR(learn,  S_IWUSR, NULL, fib_learn_age_sync);
static DEVICE_ATTR(age_enable,    S_IWUSR, NULL, fib_learn_age_sync);
static DEVICE_ATTR(sync,   S_IWUSR, NULL, fib_learn_age_sync);
#endif
static DEVICE_ATTR(fib_clean, S_IWUSR, NULL, do_fib_clean);
static DEVICE_ATTR(arp_clean, S_IWUSR, NULL, do_arp_clean);

static struct attribute *nfp_fib_attrs[] = {
	&dev_attr_fib_add.attr,
	&dev_attr_fib_del.attr,
	&dev_attr_fib_age.attr,
	&dev_attr_arp_add.attr,
	&dev_attr_arp_del.attr,
	&dev_attr_arp_age.attr,
	&dev_attr_help.attr,
#ifdef NFP_FIB_LEARN
	&dev_attr_learn.attr,
	&dev_attr_age_enable.attr,
	&dev_attr_sync.attr,
#endif
	&dev_attr_fib_clean.attr,
	&dev_attr_arp_clean.attr,
	NULL
};
static struct attribute_group nfp_fib_group = {
	.attrs = nfp_fib_attrs,
};



int __init nfp_fib_sysfs_init(struct kobject *nfp_kobj)
{
		int err;
		struct kobject *fib_kobj = NULL;
		fib_kobj = kobject_create_and_add("fib", nfp_kobj);
		if (!fib_kobj) {
			printk(KERN_INFO "could not create fib kobject \n");
			return -ENOMEM;
		}

		err = sysfs_create_group(fib_kobj, &nfp_fib_group);
		if (err) {
			printk(KERN_INFO "fib sysfs group failed %d\n", err);
			goto out;
		}


out:
		return err;
}
