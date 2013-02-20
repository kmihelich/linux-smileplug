/*******************************************************************************
Copyright (C) Marvell Interctional Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
Interctional Ltd. and/or its affiliates ("Marvell") under the following
alterctive licensing terms.  Once you have made an election to distribute the
File under one of the following license alterctives, please (i) delete this
introductory statement regarding license alterctives, (ii) delete the two
license alterctives that you have not elected to use and (iii) preserve the
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

static ssize_t do_ct_clean(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int mode;
	unsigned int res = 0, err = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	res = sscanf(buf, "%d", &mode);
	if ((res < 1) || (mode != 1))
		goto ct_err;

	nfp_ct_clean(MV_INET6);

ct_out:
	return err ? -EINVAL : len;
ct_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto ct_out;
}

static ssize_t ct_help(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int off = 0;
	PRINT("Local commands:\n");
	PRINT("cat                                             help           - print this help\n");

	PRINT("\n5 tuple commands:\n");
	PRINT("echo sip dip sport dport [tcp | udp] [0 | 1]  > forward        - set filtering mode of 5 tuple rule to drop(0) / forward(1).\n");
	PRINT("echo sip dip sport dport [tcp | udp]          > del            - delete 5 tuple rule.\n");
	PRINT("echo sip dip sport dport [tcp | udp]          > age            - print number of packets processed by this 5 tuple rule.\n");

	PRINT("\nIngress rate limit commands:\n");
	PRINT("echo sip dip sport dport [tcp | udp] tbf      > rate_limit_set - attach the specified 5 tuple rule to the\n");
	PRINT("                                                                 token buffer filter with tbf as its index.\n");
	PRINT("echo sip dip sport dport [tcp | udp]          > rate_limit_del - disable the ingress rate limiting for the\n");
	PRINT("                                                                 5 tuple rule, the tbf is not deleted\n");
#ifdef NFP_CT_LEARN
	PRINT("echo [0 | 1]                                  > learn          - enable/disble NFP 5 tuple dynamic learning\n");
	PRINT("echo [0 | 1]                                  > age_enable     - enable/disble NFP 5 tuple aging\n");
	PRINT("echo 1                                        > sync           - sync Linux ct database to NFP database\n");
#endif
	PRINT("echo mode                                     > ct_clean       - clean CT DB. \n");

	PRINT("\nParameters: sip = source, dip = destination ip, sport = source port, dport = destination port\n");
	PRINT("            [tcp | udp] = protocol (case insensitive).\n\n");
	PRINT("            1 - perform clean operation \n");
	return off;
}


static ssize_t ct_age_del_forward(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, mode, isForward;
	unsigned char sipArr[16], dipArr[16];
	char ipv6StrSip[IPV6_MAX_SIZE], ipv6StrDip[IPV6_MAX_SIZE], protoStr[4];
	MV_U16 sport, dport;
	const char *name = attr->attr.name;
	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	memset(sipArr, 0, sizeof(sipArr));
	memset(dipArr, 0, sizeof(dipArr));
	memset(ipv6StrSip, 0, sizeof(ipv6StrSip));
	memset(ipv6StrDip, 0, sizeof(ipv6StrDip));

	isForward = !strcmp(name, "forward");
	res = sscanf(buf, "%s %s %hu %hu %c%c%c %d", ipv6StrSip, ipv6StrDip,
			&sport, &dport, protoStr, protoStr+1, protoStr+2, &mode);

	if ((res < 8 && isForward) || (res < 7 && !isForward) || (isForward && mode > 1))
		goto forward_err;
	ipv6Parser(ipv6StrSip, sipArr);
	ipv6Parser(ipv6StrDip, dipArr);
	err = do_ct_age_del_forward(name, AF_INET6, sipArr, dipArr, sport, dport, protoStr, mode);

forward_out:
	return err ? -EINVAL : len;
forward_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto forward_out;
}


#ifdef NFP_LIMIT
static ssize_t ct_tbf(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, tbfIndex;
	unsigned char sipArr[16], dipArr[16];
	char ipv6StrSip[IPV6_MAX_SIZE], ipv6StrDip[IPV6_MAX_SIZE], protoStr[4];
	MV_U16 sport, dport;
	const char *name = attr->attr.name;
	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	memset(sipArr, 0, sizeof(sipArr));
	memset(dipArr, 0, sizeof(dipArr));
	memset(ipv6StrSip, 0, sizeof(ipv6StrSip));
	memset(ipv6StrDip, 0, sizeof(ipv6StrDip));
	res = sscanf(buf, "%s %s %hu %hu %c%c%c %d",
		ipv6StrSip, ipv6StrDip, &sport, &dport, protoStr, protoStr+1, protoStr+2, &tbfIndex);

	ipv6Parser(ipv6StrSip, sipArr);
	ipv6Parser(ipv6StrDip, dipArr);
	if ((res < 8 && !strcmp(name, "rate_limit_set")) || (res < 7 && strcmp(name, "rate_limit_set")))
		goto tbf_err;
	err = do_ct_tbf(name, AF_INET6, sipArr, dipArr, sport, dport, protoStr, tbfIndex);

tbf_out:
	return err ? -EINVAL : len;
tbf_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto tbf_out;
}

static DEVICE_ATTR(rate_limit_set, S_IWUSR, NULL, ct_tbf);
static DEVICE_ATTR(rate_limit_del, S_IWUSR, NULL, ct_tbf);
#endif /* NFP_LIMIT */


/* classification */
#ifdef NFP_CLASSIFY
static ssize_t ct_class_help(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int off = 0;
	PRINT("cat                                                help              - print this help\n");
	PRINT("echo sip dip sport dport [tcp | udp] dcsp ndscp  > dscp              - map a new dscp value for 5 tuple rule with the specified dscp\n");
	PRINT("                                                                       if ndscp = -1, then delete the specified dscp rule\n");
	PRINT("echo sip dip sport dport [tcp | udp] ndscp       > dscp_default      - map a new dscp value for 5 tuple rule, regardless of its dscp\n");
	PRINT("                                                                       value, if ndscp = -1 then delete former dscp mapping\n");
	PRINT("echo sip dip sport dport [tcp | udp] prio nprio  > vlan_prio         - set a new vlan prio for 5 tuple rule with specified prio.\n");
	PRINT("                                                                       if nprio = -1, then delete former mapping for this rule.\n");
	PRINT("echo sip dip sport dport [tcp | udp] nprio       > vlan_prio_default - set a new vlan prio for 5 tuple rule, regardless of its vlan\n");
	PRINT("                                                                       prio, if nprio = -1 then delete rule's former mapping.\n");
	PRINT("echo sip dip sport dport [tcp | udp] txq         > txq               - set transmit queue for 5 tuple rule.\n");
	PRINT("                                                                       if txq = -1, then delete former txq mapping for this rule.\n\n");

	PRINT("Parameters: sip = source, dip = destination ip, sport = source port, dport = destination port\n");
	PRINT("            [tcp | udp] = protocol (case insensitive), dscp = dscp value, ndscp = new dscp value\n");
	PRINT("            prio = vlan prio value, nprio = new vlan prio, txq = transmit queue value.\n\n");
	return off;
}



static ssize_t ct6_classify(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, val, nval;
	unsigned char sipArr[16], dipArr[16];
	char ipv6StrSip[IPV6_MAX_SIZE], ipv6StrDip[IPV6_MAX_SIZE], protoStr[4];
	MV_U16 sport, dport;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	memset(sipArr, 0, sizeof(sipArr));
	memset(dipArr, 0, sizeof(dipArr));
	memset(ipv6StrSip, 0, sizeof(ipv6StrSip));
	memset(ipv6StrDip, 0, sizeof(ipv6StrDip));

	res = sscanf(buf, "%s %s %hu %hu %c%c%c %d %d", ipv6StrSip, ipv6StrDip,
			&sport, &dport, protoStr, protoStr+1, protoStr+2, &val, &nval);
	if (res < 9 || (res < 10 && (!strcmp(name, "dscp") || !strcmp(name, "vlan_prio"))))
		goto class_err;

	ipv6Parser(ipv6StrSip, sipArr);
	ipv6Parser(ipv6StrDip, dipArr);

	err = do_ct_classify(name, AF_INET6, sipArr, dipArr, sport, dport, protoStr, val, nval);

class_out:
	return err ? -EINVAL : len;
class_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto class_out;
}

static DEVICE_ATTR(dscp, S_IWUSR, NULL, ct6_classify);
static DEVICE_ATTR(dscp_default, S_IWUSR, NULL, ct6_classify);
static DEVICE_ATTR(vlan_prio, S_IWUSR, NULL, ct6_classify);
static DEVICE_ATTR(vlan_prio_default, S_IWUSR, NULL, ct6_classify);
static DEVICE_ATTR(txq, S_IWUSR, NULL, ct6_classify);
static DEVICE_ATTR(txp, S_IWUSR, NULL, ct6_classify);
static DEVICE_ATTR(mh, S_IWUSR, NULL, ct6_classify);
static DEVICE_ATTR(classify_help,   S_IRUSR, ct_class_help, NULL);
#endif /* NFP_CLASSIFY */
/* end of classification */

#ifdef NFP_CT_LEARN
static ssize_t ct6_learn_age_sync(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int err, a;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = err = 0;
	sscanf(buf, "%x", &a);

	if (!strcmp(name, "learn")) {
		nfp_ct6_learn_enable(a);
	} else if (!strcmp(name, "age_enable")) {
		nfp_ct6_age_enable(a);
	} else if (!strcmp(name, "sync")) {
		nfp_ct_sync(MV_INET6);
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	return err ? -EINVAL : len;
}
#endif /* NFP_CT_LEARN */

static DEVICE_ATTR(forward, S_IWUSR, NULL, ct_age_del_forward);
static DEVICE_ATTR(del, S_IWUSR, NULL, ct_age_del_forward);
static DEVICE_ATTR(age, S_IWUSR, NULL, ct_age_del_forward);
#ifdef NFP_CT_LEARN
static DEVICE_ATTR(learn,   S_IWUSR, NULL, ct6_learn_age_sync);
static DEVICE_ATTR(age_enable,     S_IWUSR, NULL, ct6_learn_age_sync);
static DEVICE_ATTR(sync,    S_IWUSR, NULL, ct6_learn_age_sync);
#endif /* NFP_CT_LEARN */
static DEVICE_ATTR(ct_clean, S_IWUSR, NULL, do_ct_clean);
static DEVICE_ATTR(help,   S_IRUSR, ct_help, NULL);


static struct attribute *nfp_ct6_attrs[] = {
	&dev_attr_forward.attr,
	&dev_attr_age.attr,
	&dev_attr_del.attr,
#ifdef NFP_LIMIT
	&dev_attr_rate_limit_set.attr,
	&dev_attr_rate_limit_del.attr,
#endif /* NFP_LIMIT */
#ifdef NFP_CT_LEARN
	&dev_attr_learn.attr,
	&dev_attr_age_enable.attr,
	&dev_attr_sync.attr,
#endif /* NFP_CT_LEARN */
	&dev_attr_ct_clean.attr,
	&dev_attr_help.attr,
	NULL
};
static struct attribute_group nfp_ct6_group = {
	.attrs = nfp_ct6_attrs,
};

#ifdef NFP_CLASSIFY
static struct attribute *nfp_ct_cls_attrs[] = {
	&dev_attr_dscp.attr,
	&dev_attr_dscp_default.attr,
	&dev_attr_vlan_prio.attr,
	&dev_attr_vlan_prio_default.attr,
	&dev_attr_txq.attr,
	&dev_attr_txp.attr,
	&dev_attr_mh.attr,
	&dev_attr_classify_help.attr,
	NULL
};
static struct attribute_group nfp_ct_cls_group = {
	.attrs = nfp_ct_cls_attrs,
	.name = "classify",
};
#endif /* NFP_CLASSIFY */

/* HWF-NFP debug */

static ssize_t hwf_nfp_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int off = 0;
	MV_STATUS status;
	MV_U32 hit;
	MV_NFP_CT_KEY key;
	if (!strcmp(attr->attr.name, "first_get")) {
		status = nfp_ct_first_get(&key, &hit);
	} else if (!strcmp(attr->attr.name, "mostly_get")) {
		status = nfp_ct_mostly_used_get(&key, &hit);
	} else {
		status = nfp_ct_next_get(&key, &hit);
	}
	if (status != MV_OK) {
		off += mvOsSPrintf(buf+off, "No rules found.\n");
		return off;
	}
	if (key.family == AF_INET) {
		off += mvOsSPrintf(buf+off, "\nfamily: IPv4\n");
		off += mvOsSPrintf(buf+off, "sip: "MV_IPQUAD_FMT"\n", MV_IPQUAD(key.src_l3));
		off += mvOsSPrintf(buf+off, "dip: "MV_IPQUAD_FMT"\n", MV_IPQUAD(key.dst_l3));
	} else {
		off += mvOsSPrintf(buf+off, "\nfamily: IPv6\n");
		off += mvOsSPrintf(buf+off, "sip: "MV_IP6_FMT"\n", MV_IP6_ARG(key.src_l3));
		off += mvOsSPrintf(buf+off, "dip: "MV_IP6_FMT"\n", MV_IP6_ARG(key.dst_l3));
	}
	off += mvOsSPrintf(buf+off, "sport: %hu\n", key.sport);
	off += mvOsSPrintf(buf+off, "dport: %hu\n", key.dport);
	off += mvOsSPrintf(buf+off, "proto: %hhu\n\n", key.proto);
	off += mvOsSPrintf(buf+off, "hit counter: %d\n\n", hit);
	return off;
}

static ssize_t hwf_nfp_debug_store(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0;
	MV_U32 hit_cntr, val;
	MV_NFP_CT_KEY key;
	MV_NFP_CT_INFO info;
	MV_STATUS status;
	char ipv6StrSip[IPV6_MAX_SIZE], ipv6StrDip[IPV6_MAX_SIZE], protoStr[4];

	memset(ipv6StrSip, 0, sizeof(ipv6StrSip));
	memset(ipv6StrDip, 0, sizeof(ipv6StrDip));
	res = sscanf(buf, "%s %s %hu %hu %c%c%c %d", ipv6StrSip, ipv6StrDip,
			&key.sport, &key.dport, protoStr, protoStr+1, protoStr+2, &val);

	ipv6Parser(ipv6StrSip, key.src_l3);
	ipv6Parser(ipv6StrDip, key.dst_l3);
	key.family = AF_INET6;
	key.proto = get_protocol_num(protoStr);
	if (!strcmp(attr->attr.name, "hwf_add")) {
		status = nfp_ct_hwf_set(&key);
	} else if (!strcmp(attr->attr.name, "hwf_del")) {
		status = nfp_ct_hwf_clear(&key);
	} else if (!strcmp(attr->attr.name, "hit_get")) {
		status = nfp_ct_hit_cntr_get(&key, &hit_cntr);
		if (status == MV_OK)
			printk(KERN_INFO "hit_cntr: %d\n", hit_cntr);
	} else if (!strcmp(attr->attr.name, "hit_set")) {
		status = nfp_ct_hit_cntr_set(&key, val);
	} else if (!strcmp(attr->attr.name, "info")) {
		status = nfp_ct_info_get(&key, &info);
		if (status != MV_OK) {
			printk("rule not found\n");
			return len;
		}
		printk("info-flags: %d\n", info.flags);
		printk("info-nsip: %d   %s VALID\n", info.new_sip, (info.flags & NFP_F_CT_SNAT) ? "" : "NOT");
		printk("info-ndip: %d   %s VALID\n", info.new_dip, (info.flags & NFP_F_CT_DNAT) ? "" : "NOT");
		printk("info-nsport: %d   %s VALID\n", info.new_sport, (info.flags & NFP_F_CT_SNAT) ? "" : "NOT");
		printk("info-ndport: %d   %s VALID\n", info.new_dport, (info.flags & NFP_F_CT_DNAT) ? "" : "NOT");
		printk("info-sa: "MV_MACQUAD_FMT"   %s VALID\n", MV_MACQUAD(info.sa),
									!(info.flags & NFP_F_CT_FIB_INV) ? "" : "NOT");
		printk("info-da: "MV_MACQUAD_FMT"   %s VALID\n", MV_MACQUAD(info.da),
									!(info.flags & NFP_F_CT_FIB_INV) ? "" : "NOT");
		printk("info-out_port: %d   %s VALID\n", info.out_port, !(info.flags & NFP_F_CT_FIB_INV) ? "" : "NOT");
#ifdef NFP_CLASSIFY
		printk("info-mh: %d   %s VALID\n", info.mh, (info.flags & NFP_F_CT_SET_MH) ? "" : "NOT");
		printk("info-txp: %d   %s VALID\n", info.txp, (info.flags & NFP_F_CT_SET_TXP) ? "" : "NOT");
		printk("info-txq: %d   %s VALID\n", info.txq, (info.flags & NFP_F_CT_SET_TXQ) ? "" : "NOT");
		printk("info-dscp: %d   %s VALID\n", info.dscp, (info.flags & NFP_F_CT_SET_DSCP) ? "" : "NOT");
		printk("info-vprio: %d   %s VALID\n", info.vprio, (info.flags & NFP_F_CT_SET_VLAN_PRIO) ? "" : "NOT");
#endif /* NFP_CLASSIFY */
		printk("info-processed by: %s\n\n", (info.flags & NFP_F_CT_HWF) ? "HWF" : "NFP");
	}

	return err ? -EINVAL : len;
}

static ssize_t ct_hwf_debug_help(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int off = 0;
	PRINT("cat                                        help_hwf   - print this help\n");
	PRINT("cat                                        first_get  - start new CT-DB iteration, print first valid ct rule and its hit counter\n");
	PRINT("cat                                        next_get   - print next valid ct rule and its hit counter\n");
	PRINT("cat                                        mostly_get - print valid ct rule with max hit counter value\n");
	PRINT("echo sip dip sport dport [tcp | udp]     > hwf_add    - inform NFP that specific ct rule is now processed by HWF\n");
	PRINT("echo sip dip sport dport [tcp | udp]     > hwf_del    - inform NFP that specific ct rule is no longer processed by HWF\n");
	PRINT("echo sip dip sport dport [tcp | udp]     > hit_get    - get hit counter for specific ct rule\n");
	PRINT("echo sip dip sport dport [tcp | udp] val > hit_set    - clear hit counter for specific ct rule\n");
	PRINT("echo sip dip sport dport [tcp | udp]     > info_get   - print information for specific ct rule\n\n");

	PRINT("Parameters: sip = source ip, dip = destination ip, sport = source port, dport = destination port\n");
	PRINT("           [tcp | udp] = protocol (case insensitive)\n\n");

	return off;
}

static DEVICE_ATTR(first_get, S_IRUSR, hwf_nfp_debug_show, NULL);
static DEVICE_ATTR(next_get, S_IRUSR, hwf_nfp_debug_show, NULL);
static DEVICE_ATTR(hwf_add, S_IWUSR, NULL, hwf_nfp_debug_store);
static DEVICE_ATTR(hwf_del, S_IWUSR, NULL, hwf_nfp_debug_store);
static DEVICE_ATTR(hit_set, S_IWUSR, NULL, hwf_nfp_debug_store);
static DEVICE_ATTR(hit_get, S_IWUSR, NULL, hwf_nfp_debug_store);
static DEVICE_ATTR(mostly_get, S_IRUSR, hwf_nfp_debug_show, NULL);
static DEVICE_ATTR(info_get, S_IWUSR, NULL, hwf_nfp_debug_store);
static DEVICE_ATTR(help_hwf, S_IRUSR, ct_hwf_debug_help, NULL);

static struct attribute *nfp_ct_dbg_attrs[] = {
	&dev_attr_first_get.attr,
	&dev_attr_next_get.attr,
	&dev_attr_hwf_add.attr,
	&dev_attr_hwf_del.attr,
	&dev_attr_hit_set.attr,
	&dev_attr_hit_get.attr,
	&dev_attr_mostly_get.attr,
	&dev_attr_info_get.attr,
	&dev_attr_help_hwf.attr,
	NULL
};
static struct attribute_group nfp_ct_dbg_group = {
	.attrs = nfp_ct_dbg_attrs,
	.name = "hwf",
};

/*********/

int __init nfp_ct6_sysfs_init(struct kobject *nfp_kobj)
{
	int err;
	struct kobject *ct_kobj = NULL;

	ct_kobj = kobject_create_and_add("ct6", nfp_kobj);
	if (!ct_kobj) {
		printk(KERN_INFO "could not create ct6 kobject \n");
		return -ENOMEM;
	}

	err = sysfs_create_group(ct_kobj, &nfp_ct6_group);
	if (err) {
		printk(KERN_INFO "ct6 sysfs group failed %d\n", err);
		goto out;
	}
#ifdef NFP_CLASSIFY
	err = sysfs_create_group(ct_kobj, &nfp_ct_cls_group);
	if (err) {
		printk(KERN_INFO "ct6 classify sysfs group failed %d\n", err);
		goto out;
	}
#endif /* NFP_CLASSIFY */
	err = sysfs_create_group(ct_kobj, &nfp_ct_dbg_group);
	if (err) {
		printk(KERN_INFO "ct6 dbg sysfs group failed %d\n", err);
		goto out;
	}
out:
	return err;
}
#endif /* CONFIG_IPV6 */
