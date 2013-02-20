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

/* key type */
struct key_type {
	unsigned int family;
	unsigned char sip[16], dip[16];
	MV_U16 sport, dport;
	MV_U8 proto;
	int init;
} key;

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

	nfp_ct_clean(MV_INET);

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

	PRINT("\nNAT commands:\n");
	PRINT("echo sip dip sport dport [tcp | udp]          > local_key      - save a local 5 tuple key to work on.\n");
	PRINT("cat                                             print_key      - print the key saved by local_key.\n");
	PRINT("echo nsip nsport                              > snat           - add a new SNAT rule based on the local 5 tuple key.\n");
	PRINT("echo ndip ndport                              > dnat           - add a new DNAT rule based on the local 5 tuple key.\n");
	PRINT("echo nsip ndip nsport ndport                  > snat_dnat      - add a new SNAT + DNAT rule based on the local 5 tuple key.\n\n");
#ifdef NFP_LIMIT
	PRINT("Ingress rate limit commands:\n");
	PRINT("echo sip dip sport dport [tcp | udp] tbf      > rate_limit_set - attach the specified 5 tuple rule to the\n");
	PRINT("                                                                 token buffer filter with tbf as its index.\n");
	PRINT("echo sip dip sport dport [tcp | udp]          > rate_limit_del - disable the ingress rate limiting for the\n");
	PRINT("                                                                 5 tuple rule, the tbf is not deleted\n\n");
#endif /* NFP_LIMIT */
	PRINT("echo mode                                     > ct_clean       - clean CT DB. \n");
#ifdef NFP_CT_LEARN
	PRINT("echo [0 | 1]                                  > learn          - enable/disble NFP 5 tuple dynamic learning\n");
	PRINT("echo [0 | 1]                                  > age_enable     - enable/disble NFP 5 tuple aging\n");
	PRINT("echo 1                                        > sync           - sync Linux ct database to NFP database\n");
#endif
	PRINT("\nParameters: sip = source, dip = destination ip, sport = source port, dport = destination port\n");
	PRINT("            [tcp | udp] = protocol (case insensitive).\n\n");
	PRINT("            1 - perform clean operation \n");


	return off;
}

int get_protocol_num(char *str)
{
	if ((str[0] == 'T' || str[0] == 't') && (str[1] == 'C' || str[1] == 'c') && (str[2] == 'P' || str[2] == 'p'))
		return MV_IP_PROTO_TCP;
	else if ((str[0] == 'U' || str[0] == 'u') && (str[1] == 'D' || str[1] == 'd') && (str[2] == 'P' || str[2] == 'p'))
		return MV_IP_PROTO_UDP;
	return 0;
}

static ssize_t do_store_local_key(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0;
	char protoStr[4];

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	res = sscanf(buf, "%hhu.%hhu.%hhu.%hhu %hhu.%hhu.%hhu.%hhu %hu %hu %c%c%c", (key.sip),
			(key.sip)+1, (key.sip)+2, (key.sip)+3, (key.dip), (key.dip)+1, (key.dip)+2,
			(key.dip)+3, &(key.sport), &(key.dport), protoStr, protoStr+1, protoStr+2);

	if (res < 13) {
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
		err = 1;
		goto key_store_out;
	}
	key.proto = get_protocol_num(protoStr);
	if (!key.proto) {
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
		err = 1;
		goto key_store_out;
	}
	key.init = 1;
key_store_out:
	return err ? -EINVAL : len;
}


static ssize_t do_print_local_key(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	unsigned int off = 0;
	if (!key.init) {
		off += mvOsSPrintf(buf+off, "Print_local_key: local key is not initialized.\n");
		goto key_print_out;
	}
	off += mvOsSPrintf(buf+off, "Local key:\n");
	off += mvOsSPrintf(buf+off, "    family: IPv4\n");
	off += mvOsSPrintf(buf+off, "    sip: "MV_IPQUAD_FMT"\n", MV_IPQUAD(key.sip));
	off += mvOsSPrintf(buf+off, "    dip: "MV_IPQUAD_FMT"\n", MV_IPQUAD(key.dip));
	off += mvOsSPrintf(buf+off, "    sport: %hu\n", key.sport);
	off += mvOsSPrintf(buf+off, "    dport: %hu\n", key.dport);
	off += mvOsSPrintf(buf+off, "    proto: %hhu\n\n", key.proto);
key_print_out:
	return off;
}


static ssize_t ct_nat_snat_dnat(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, flags;
	unsigned char nsipArr[4], ndipArr[4], *nsip = key.sip, *ndip = key.dip;
	MV_U16 nsport = key.sport, ndport = key.dport;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	memset(nsipArr, 0, sizeof(nsipArr));
	memset(ndipArr, 0, sizeof(ndipArr));
	if (!key.init) {
		printk(KERN_ERR "%s: Local key is not initialized.\n", attr->attr.name);
		goto nat_err;
	}
	if (!strcmp(name, "dnat")) {
		res = sscanf(buf, "%hhu.%hhu.%hhu.%hhu %hu", ndipArr, ndipArr+1, ndipArr+2, ndipArr+3, &ndport);
		if (res < 5)
			goto nat_err;
		ndip = ndipArr;
		flags = MV_ETH_NFP_NAT_MANIP_DST;
	} else if (!strcmp(name, "snat")) {
		res = sscanf(buf, "%hhu.%hhu.%hhu.%hhu %hu", nsipArr, nsipArr+1, nsipArr+2, nsipArr+3, &nsport);
		if (res < 5)
			goto nat_err;
		nsip = nsipArr;
		flags = MV_ETH_NFP_NAT_MANIP_SRC;
	} else { /* SNAT + DNAT */
		res = sscanf(buf, "%hhu.%hhu.%hhu.%hhu %hu %hhu.%hhu.%hhu.%hhu %hu", nsipArr,
			nsipArr+1, nsipArr+2, nsipArr+3, &nsport, ndipArr, ndipArr+1, ndipArr+2, ndipArr+3, &ndport);
		if (res < 10)
			goto nat_err;
		ndip = ndipArr;
		nsip = nsipArr;
		flags = MV_ETH_NFP_NAT_MANIP_DST | MV_ETH_NFP_NAT_MANIP_SRC;
	}
	NFP_SYSFS_DBG(printk(KERN_INFO "nfp_ct_nat_add( "MV_IPQUAD_FMT" , "MV_IPQUAD_FMT" , %hu , %hu ,%hhu \n",
			key.sip[0], key.sip[1], key.sip[2], key.sip[3], key.dip[0], key.dip[1], key.dip[2],
			key.dip[3], key.sport, key.dport, key.proto));
	NFP_SYSFS_DBG(printk(KERN_INFO "                "MV_IPQUAD_FMT" , "MV_IPQUAD_FMT" , %hu , %hu ,%d )\n",
			nsip[0], nsip[1], nsip[2], nsip[3], ndip[0], ndip[1], ndip[2], ndip[3], nsport, ndport, flags));
	nfp_ct_nat_add(*(MV_U32 *)key.sip, *(MV_U32 *)key.dip, key.sport, key.dport, key.proto,
			*(MV_U32 *)nsip, *(MV_U32 *)ndip, nsport, ndport, flags);
nat_out:
	return err ? -EINVAL : len;
nat_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto nat_out;
}


int do_ct_age_del_forward(const char *name, int family, unsigned char *sip,
		unsigned char *dip, int sport, int dport, char *protoStr, int mode)
{
	int proto;

	proto = get_protocol_num(protoStr);
	if (!proto)
		return 1;

	NFP_SYSFS_DBG(printk(KERN_INFO "nfp_ct_%s( IPv%d , ", name, family));
	if (family == AF_INET)
		NFP_SYSFS_DBG(printk(MV_IPQUAD_FMT" , "MV_IPQUAD_FMT,
				MV_IPQUAD(sip), MV_IPQUAD(dip)));
	else
		NFP_SYSFS_DBG(printk(MV_IP6_FMT" , "MV_IP6_FMT, MV_IP6_ARG(sip), MV_IP6_ARG(dip)));
	if (!strcmp(name, "forward")) {
		NFP_SYSFS_DBG(printk(KERN_INFO " , %hu , %hu, %d , %d )\n", sport, dport, proto, mode));
		nfp_ct_filter_set(family, sip, dip, sport, dport, proto, mode);
	} else {
		NFP_SYSFS_DBG(printk(KERN_INFO " , %hu , %hu, %d )\n", sport, dport, proto));
		if (!strcmp(name, "del"))
			nfp_ct_del(family, sip, dip, sport, dport, proto);
		else
			printk(KERN_INFO "age: %d\n", nfp_ct_age(family, sip, dip, sport, dport, proto));
	}
	return 0;
}

static ssize_t ct_age_del_forward(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, mode = 0, isForward;
	unsigned char sipArr[4], dipArr[4];
	char protoStr[4];
	MV_U16 sport, dport;
	const char *name = attr->attr.name;
	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	memset(sipArr, 0, sizeof(sipArr));
	memset(dipArr, 0, sizeof(dipArr));

	isForward = !strcmp(name, "forward");
	res = sscanf(buf, "%hhu.%hhu.%hhu.%hhu %hhu.%hhu.%hhu.%hhu %hu %hu %c%c%c %d",
		sipArr, sipArr+1, sipArr+2, sipArr+3, dipArr, dipArr+1, dipArr+2, dipArr+3, &sport, &dport,
		protoStr, protoStr+1, protoStr+2, &mode);

	if ((res < 14 && isForward) || (res < 13 && !isForward) || (isForward && mode > 1))
		goto forward_err;

	err = do_ct_age_del_forward(name, AF_INET, sipArr, dipArr, sport, dport, protoStr, mode);
forward_out:
	return err ? -EINVAL : len;
forward_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto forward_out;
}


#ifdef NFP_LIMIT
int do_ct_tbf(const char *name, int family, unsigned char *sip,
		unsigned char *dip, int sport, int dport, char *protoStr, int tbfIndex)
{
	int proto;
	proto = get_protocol_num(protoStr);
	if (!proto)
		return 1;

	NFP_SYSFS_DBG(printk(KERN_INFO "nfp_ct_%s( IPv%d , ", name, family));
	if (family == AF_INET)
		NFP_SYSFS_DBG(printk(KERN_INFO MV_IPQUAD_FMT" , "MV_IPQUAD_FMT,
				MV_IPQUAD(sip), MV_IPQUAD(dip)));
	else
		NFP_SYSFS_DBG(printk(KERN_INFO MV_IP6_FMT" , "MV_IP6_FMT, MV_IP6_ARG(sip), MV_IP6_ARG(dip)));
	if (!strcmp(name, "rate_limit_set")) {
		NFP_SYSFS_DBG(printk(KERN_INFO " , %hu , %hu, %d , %d )\n", sport, dport, proto, tbfIndex));
		nfp_ct_rate_limit_set(family, sip, dip, sport, dport, proto, tbfIndex);
	} else {
		NFP_SYSFS_DBG(printk(KERN_INFO " , %hu , %hu, %d )\n", sport, dport, proto));
		nfp_ct_rate_limit_del(family, sip, dip, sport, dport, proto);
	}
	return 0;
}

static ssize_t ct_tbf(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, tbfIndex = 0;
	unsigned char sipArr[4], dipArr[4];
	char protoStr[4];
	MV_U16 sport, dport;
	const char *name = attr->attr.name;
	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	memset(sipArr, 0, sizeof(sipArr));
	memset(dipArr, 0, sizeof(dipArr));

	/* rate limit set / del  */
	res = sscanf(buf, "%hhu.%hhu.%hhu.%hhu %hhu.%hhu.%hhu.%hhu %hu %hu %c%c%c %d",
		sipArr, sipArr+1, sipArr+2, sipArr+3, dipArr, dipArr+1, dipArr+2, dipArr+3, &sport, &dport,
		protoStr, protoStr+1, protoStr+2, &tbfIndex);
	if ((res < 14 && !strcmp(name, "rate_limit_set")) || (res < 13 && strcmp(name, "rate_limit_set")))
		goto tbf_err;
	err = do_ct_tbf(name, AF_INET, sipArr, dipArr, sport, dport, protoStr, tbfIndex);

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
	PRINT("echo sip dip sport dport [tcp | udp] txp         > txp               - set transmit port for 5 tuple rule.\n");
	PRINT("                                                                       if txp = -1, then delete former txp mapping for this rule.\n\n");
	PRINT("echo sip dip sport dport [tcp | udp] mh          > mh                - set Marvell Header for 5 tuple rule.\n");
	PRINT("                                                                       if mh = -1, then delete former MH mapping for this rule.\n\n");

	PRINT("Parameters: sip = source, dip = destination ip, sport = source port, dport = destination port\n");
	PRINT("            [tcp | udp] = protocol (case insensitive), dscp = dscp value, ndscp = new dscp value\n");
	PRINT("            prio = vlan prio value, nprio = new vlan prio, txq = transmit queue value\n");
	PRINT("            txp = transmit port value, mh = Marvell Header value.\n\n");

	return off;
}


int do_ct_classify(const char *name, int family, unsigned char *sip,
		unsigned char *dip, int sport, int dport, char *protoStr, int val, int nval)
{
	int proto = get_protocol_num(protoStr);

	if (!proto)
		return 1;

	if (!strcmp(name, "dscp")) {
		if (nval == -1)
			nfp_ct_dscp_del(family, sip, dip, sport, dport, proto, val);
		else
			nfp_ct_dscp_set(family, sip, dip, sport, dport, proto, val, nval);
	} else if (!strcmp(name, "dscp_default")) {
		if (val == -1)
			nfp_ct_dscp_del(family, sip, dip, sport, dport, proto, val);
		else
			nfp_ct_dscp_set(family, sip, dip, sport, dport, proto, MV_ETH_NFP_GLOBAL_MAP, val);
	} else if (!strcmp(name, "vlan_prio")) {
		if (nval == -1)
			nfp_ct_vlan_prio_del(family, sip, dip, sport, dport, proto, val);
		else
			nfp_ct_vlan_prio_set(family, sip, dip, sport, dport, proto, val, nval);
	} else if (!strcmp(name, "vlan_prio_default")) {
		if (val == -1)
			nfp_ct_vlan_prio_del(family, sip, dip, sport, dport, proto, val);
		else
			nfp_ct_vlan_prio_set(family, sip, dip, sport, dport, proto, MV_ETH_NFP_GLOBAL_MAP, val);
	} else if (!strcmp(name, "txq")) {
		if (val == -1)
			nfp_ct_txq_del(family, sip, dip, sport, dport, proto);
		else
			nfp_ct_txq_set(family, sip, dip, sport, dport, proto, val);
	} else if (!strcmp(name, "txp")) {
		if (val == -1)
			nfp_ct_txp_del(family, sip, dip, sport, dport, proto);
		else
			nfp_ct_txp_set(family, sip, dip, sport, dport, proto, val);
	} else if (!strcmp(name, "mh")) {
		if (val == -1)
			nfp_ct_mh_del(family, sip, dip, sport, dport, proto);
		else
			nfp_ct_mh_set(family, sip, dip, sport, dport, proto, val);
	}

	return 0;
}


static ssize_t ct_classify(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int res = 0, err = 0, val, nval;
	unsigned char sipArr[16], dipArr[16];
	char protoStr[4];
	MV_U16 sport, dport;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	memset(sipArr, 0, sizeof(sipArr));
	memset(dipArr, 0, sizeof(dipArr));
	res = sscanf(buf, "%hhu.%hhu.%hhu.%hhu %hhu.%hhu.%hhu.%hhu %hu %hu %c%c%c %d %d",
			sipArr, sipArr+1, sipArr+2, sipArr+3, dipArr, dipArr+1, dipArr+2, dipArr+3,
			&sport, &dport, protoStr, protoStr+1, protoStr+2, &val, &nval);
	if (res < 14 || (res < 15 && (!strcmp(name, "dscp") || !strcmp(name, "vlan_prio"))))
		goto class_err;

	err = do_ct_classify(name, AF_INET, sipArr, dipArr, sport, dport, protoStr, val, nval);

class_out:
	return err ? -EINVAL : len;
class_err:
	printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	err = 1;
	goto class_out;
}

static DEVICE_ATTR(dscp, S_IWUSR, NULL, ct_classify);
static DEVICE_ATTR(dscp_default, S_IWUSR, NULL, ct_classify);
static DEVICE_ATTR(vlan_prio, S_IWUSR, NULL, ct_classify);
static DEVICE_ATTR(vlan_prio_default, S_IWUSR, NULL, ct_classify);
static DEVICE_ATTR(txq, S_IWUSR, NULL, ct_classify);
static DEVICE_ATTR(txp, S_IWUSR, NULL, ct_classify);
static DEVICE_ATTR(mh, S_IWUSR, NULL, ct_classify);
static DEVICE_ATTR(classify_help,   S_IRUSR, ct_class_help, NULL);
#endif /* NFP_CLASSIFY */
/* end of classification */

#ifdef NFP_CT_LEARN
static ssize_t ct_learn_age_sync(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int err, a;
	const char *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = err = 0;
	sscanf(buf, "%x", &a);

	if (!strcmp(name, "learn")) {
		nfp_ct_learn_enable(a);
	} else if (!strcmp(name, "age_enable")) {
		nfp_ct_age_enable(a);
	} else if (!strcmp(name, "sync")) {
		nfp_ct_sync(MV_INET);
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	return err ? -EINVAL : len;
}
#endif /* NFP_CT_LEARN */

static DEVICE_ATTR(snat, S_IWUSR, NULL, ct_nat_snat_dnat);
static DEVICE_ATTR(dnat, S_IWUSR, NULL, ct_nat_snat_dnat);
static DEVICE_ATTR(snat_dnat, S_IWUSR, NULL, ct_nat_snat_dnat);
static DEVICE_ATTR(forward, S_IWUSR, NULL, ct_age_del_forward);
static DEVICE_ATTR(del, S_IWUSR, NULL, ct_age_del_forward);
static DEVICE_ATTR(age, S_IWUSR, NULL, ct_age_del_forward);

static DEVICE_ATTR(print_key, S_IRUSR, do_print_local_key, NULL);
static DEVICE_ATTR(local_key, S_IWUSR, NULL, do_store_local_key);
static DEVICE_ATTR(help,   S_IRUSR, ct_help, NULL);
#ifdef NFP_CT_LEARN
static DEVICE_ATTR(learn,   S_IWUSR, NULL, ct_learn_age_sync);
static DEVICE_ATTR(age_enable,     S_IWUSR, NULL, ct_learn_age_sync);
static DEVICE_ATTR(sync,    S_IWUSR, NULL, ct_learn_age_sync);
#endif
static DEVICE_ATTR(ct_clean, S_IWUSR, NULL, do_ct_clean);
static struct attribute *nfp_ct_attrs[] = {
	&dev_attr_snat.attr,
	&dev_attr_dnat.attr,
	&dev_attr_snat_dnat.attr,
	&dev_attr_forward.attr,
	&dev_attr_age.attr,
	&dev_attr_del.attr,
#ifdef NFP_LIMIT
	&dev_attr_rate_limit_set.attr,
	&dev_attr_rate_limit_del.attr,
#endif /* NFP_LIMIT */
	&dev_attr_print_key.attr,
	&dev_attr_local_key.attr,
	&dev_attr_help.attr,
#ifdef NFP_CT_LEARN
	&dev_attr_learn.attr,
	&dev_attr_age_enable.attr,
	&dev_attr_sync.attr,
#endif
	&dev_attr_ct_clean.attr,
	NULL
};
static struct attribute_group nfp_ct_group = {
	.attrs = nfp_ct_attrs,
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
		off += mvOsSPrintf(buf+off,"No rules found.\n");
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
	char protoStr[4];

	res = sscanf(buf, "%hhu.%hhu.%hhu.%hhu %hhu.%hhu.%hhu.%hhu %hu %hu %c%c%c %d",
		key.src_l3, key.src_l3 + 1, key.src_l3 + 2, key.src_l3 + 3, key.dst_l3, key.dst_l3 + 1,
		key.dst_l3 + 2, key.dst_l3 + 3, &key.sport, &key.dport, protoStr, protoStr + 1, protoStr + 2, &val);
	key.family = AF_INET;
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

int __init nfp_ct_sysfs_init(struct kobject *nfp_kobj)
{
	int err;
	struct kobject *ct_kobj = NULL;

	ct_kobj = kobject_create_and_add("ct", nfp_kobj);
	if (!ct_kobj) {
		printk(KERN_INFO "could not create ct kobject \n");
		return -ENOMEM;
	}

	err = sysfs_create_group(ct_kobj, &nfp_ct_group);
	if (err) {
		printk(KERN_INFO "ct sysfs group failed %d\n", err);
		goto out;
	}
#ifdef NFP_CLASSIFY
	err = sysfs_create_group(ct_kobj, &nfp_ct_cls_group);
	if (err) {
		printk(KERN_INFO "ct classify sysfs group failed %d\n", err);
		goto out;
	}
#endif /* NFP_CLASSIFY */
	err = sysfs_create_group(ct_kobj, &nfp_ct_dbg_group);
	if (err) {
		printk(KERN_INFO "ct dbg sysfs group failed %d\n", err);
		goto out;
	}
	key.init = 0;
out:
	return err;
}
