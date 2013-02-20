/* Shared library add-on to iptables to add NFP target support. */
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

#include <xtables.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/ipt_NFP.h>

static void drop_option_error(struct ipt_nfp_info *info)
{
	if (info->mode == IPT_NFP_DROP)
		xtables_error(PARAMETER_PROBLEM,
			"--drop cannot be combined with other options");
}

static void NFP_init(struct xt_entry_target *target)
{
	struct ipt_nfp_info *info =
		(struct ipt_nfp_info *)target->data;

	info->mode = IPT_NFP_INIT;
	info->burst = -1;
	info->limit = -1;
	info->dscp = -1;
	info->new_dscp = -1;
	info->txq = -1;
	info->txp = -1;
	info->mh = -1;
	info->vlanprio = -1;
	info->new_vlanprio = -1;
}

static unsigned long get_argument(const char* arg, const char *param)
{
	long value;
	char *str;

	value = strtoll(arg, &str, 0);
	if (str	== arg)
		xtables_error(PARAMETER_PROBLEM,
			"unable to parse --%s option argument", param);

	return value;
}

static int NFP_parse(int c, char **argv, int invert, unsigned int *flags,
                     const void *entry, struct xt_entry_target **target)
{
	struct ipt_nfp_info *info =
		(struct ipt_nfp_info *)(*target)->data;
	long value, new_value;

	switch (c) {
	case 'B': /* --limit-burst <burst> */
		drop_option_error(info);
		value = get_argument(optarg, "limit-burst");

		if (value < 0)
			xtables_error(PARAMETER_PROBLEM,
				"invalid limit-burst value");

		info->mode = IPT_NFP_MANGLE;
		info->flags |= IPT_NFP_F_SET_BURST;
		info->burst = value;
		break;

	case 'D': /* --drop */
		if (info->mode == IPT_NFP_DROP)
			xtables_error(PARAMETER_PROBLEM,
				"--drop can be specified only once");

		if ((info->mode != IPT_NFP_DROP) && (info->mode != IPT_NFP_INIT))
			xtables_error(PARAMETER_PROBLEM,
				"--drop cannot be combined with other options");

		info->mode = IPT_NFP_DROP;
		break;

	case 'F': /* --forward */
		if (info->mode == IPT_NFP_FWD)
			xtables_error(PARAMETER_PROBLEM,
				"--forward can be specified only once");

		if (info->mode == IPT_NFP_DROP)
			xtables_error(PARAMETER_PROBLEM,
				"--forward cannot be combined with --drop");

		info->mode = IPT_NFP_FWD;
		break;

	case 'L': /* --limit <limit> */
		drop_option_error(info);
		value = get_argument(optarg, "limit");

		if (value < 0)
			xtables_error(PARAMETER_PROBLEM,
				"invalid limit value");

		info->mode = IPT_NFP_MANGLE;
		info->flags |= IPT_NFP_F_SET_LIMIT;
		info->limit = value;
		break;

	case 'T': /* --dscp <old> <new> */
		drop_option_error(info);
		if (info->flags & IPT_NFP_F_SET_DSCP)
			xtables_error(PARAMETER_PROBLEM,
				"--dscp can be specified only once");

		if (info->flags & IPT_NFP_F_SET_DSCP_DEF)
			xtables_error(PARAMETER_PROBLEM,
				"--dscp cannot be combined with --dscp-def");

		value = get_argument(optarg, "dscp");
		new_value = get_argument(argv[optind++], "new_dscp");

		if ((value < 0) || (value > 63))
			xtables_error(PARAMETER_PROBLEM,
				"invalid dscp value");

		if ((new_value < 0) || (new_value > 63))
			xtables_error(PARAMETER_PROBLEM,
				"invalid new_dscp value");

		info->mode = IPT_NFP_MANGLE;
		info->flags |= IPT_NFP_F_SET_DSCP;
		info->dscp = value;
		info->new_dscp = new_value;
		break;

	case 'U': /* --dscp-def <n> */
		drop_option_error(info);
		if (info->flags & IPT_NFP_F_SET_DSCP_DEF)
			xtables_error(PARAMETER_PROBLEM,
				"--dscp-def can be specified only once");

		if (info->flags & IPT_NFP_F_SET_DSCP)
			xtables_error(PARAMETER_PROBLEM,
				"--dscp-def cannot be combined with --dscp");

		value = get_argument(optarg, "dscp-def");

		if (value > 63)
			xtables_error(PARAMETER_PROBLEM,
				"invalid dscp value");

		info->mode = IPT_NFP_MANGLE;
		info->flags |= IPT_NFP_F_SET_DSCP_DEF;
		info->dscp = -1;
		info->new_dscp = value;
		break;

	case 'Q': /* --txq <n> */
		drop_option_error(info);
		if (info->flags & IPT_NFP_F_SET_TXQ)
			xtables_error(PARAMETER_PROBLEM,
				"--txq can be specified only once");

		value = get_argument(optarg, "txq");

		if (value < 0)
			xtables_error(PARAMETER_PROBLEM,
				"invalid txq value");

		/* TXQ value is corrected in kernel */
		info->mode = IPT_NFP_MANGLE;
		info->flags |= IPT_NFP_F_SET_TXQ;
		info->txq = value;
		break;

	case 'P': /* --txp <n> */
		drop_option_error(info);
		if (info->flags & IPT_NFP_F_SET_TXP)
			xtables_error(PARAMETER_PROBLEM,
				"--txp can be specified only once");

		value = get_argument(optarg, "txp");

		if (value < 0)
			xtables_error(PARAMETER_PROBLEM,
				"invalid txp value");

		/* TXP value is corrected in kernel */
		info->mode = IPT_NFP_MANGLE;
		info->flags |= IPT_NFP_F_SET_TXP;
		info->txp = value;
		break;

	case 'M': /* --mh <n> */
		drop_option_error(info);
		if (info->flags & IPT_NFP_F_SET_MH)
			xtables_error(PARAMETER_PROBLEM,
				"--mh can be specified only once");

		value = get_argument(optarg, "mh");

		if ((value < 0) || (value > 0xFFFF))
			xtables_error(PARAMETER_PROBLEM,
				"invalid Marvell Header");

		info->mode = IPT_NFP_MANGLE;
		info->flags |= IPT_NFP_F_SET_MH;
		info->mh = value;
		break;

	case 'V': /* --vlan-prio <old> <new> */
		drop_option_error(info);
		if (info->flags & IPT_NFP_F_SET_VLAN_PRIO)
			xtables_error(PARAMETER_PROBLEM,
				"--vlan-prio can be specified only once");

		if (info->flags & IPT_NFP_F_SET_VLAN_PRIO_DEF)
			xtables_error(PARAMETER_PROBLEM,
				"--vlan-prio cannot be be combined with --vlan-prio-def");

		value = get_argument(optarg, "vlan-prio");
		new_value = get_argument(argv[optind++], "new-vlan-prio");

		if ((value < 0) || (value > 7))
			xtables_error(PARAMETER_PROBLEM,
				"invalid 802.1p priority");

		if ((new_value < 0) || (new_value > 7))
			xtables_error(PARAMETER_PROBLEM,
				"invalid new 802.1p priority");

		info->mode = IPT_NFP_MANGLE;
		info->flags |= IPT_NFP_F_SET_VLAN_PRIO;
		info->vlanprio = value;
		info->new_vlanprio = new_value;
		break;

	case 'W': /* --vlan-prio-def <n> */
		drop_option_error(info);
		if (info->flags & IPT_NFP_F_SET_VLAN_PRIO_DEF)
			xtables_error(PARAMETER_PROBLEM,
				"--vlan-prio-def can be specified only once");

		if (info->flags & IPT_NFP_F_SET_VLAN_PRIO)
			xtables_error(PARAMETER_PROBLEM,
				"--vlan-prio-def cannot be combined with --vlan-prio");

		value = get_argument(optarg, "vlan-prio-def");

		if (value > 7)
			xtables_error(PARAMETER_PROBLEM,
				"invalid 802.1p priority");

		info->mode = IPT_NFP_MANGLE;
		info->flags |= IPT_NFP_F_SET_VLAN_PRIO_DEF;
		info->vlanprio = -1;
		info->new_vlanprio = value;
		break;

	default:
		return 0;
	}

	return 1;
}

static void NFP_print(const void *ip, const struct xt_entry_target *target,
		      int numeric)
{
	const struct ipt_nfp_info *info = (const void *)target->data;

	if (info->mode == IPT_NFP_DROP) {
		printf("drop ");
		return;
	}

	if (info->mode == IPT_NFP_FWD)
		printf("forward ");

	if (info->limit >= 0)
		printf("limit=%ikbps ", info->limit);

	if (info->limit >= 0)
		printf("burst=%ikB ", info->burst);

	if ((info->dscp >= 0) || (info->new_dscp >= 0))
		printf("dscp=0x%02X new_dscp=0x%02X ", info->dscp, info->new_dscp);

	if (info->txq >= 0)
		printf("txq=%i ", info->txq);

	if (info->txp >= 0)
		printf("txp=%i ", info->txp);

	if (info->mh >= 0)
		printf("mh=0x%X ", info->mh);

	if ((info->vlanprio >= 0) || (info->new_vlanprio >= 0))
		printf("vlan-prio=%i new-vlan-prio=%i ", info->vlanprio, info->new_vlanprio);
}

static void NFP_save(const void *ip, const struct xt_entry_target *target)
{
	const struct ipt_nfp_info *info = (const void *)target->data;

	if (info->mode == IPT_NFP_DROP) {
		printf("--drop");
		return;
	}

	if (info->mode == IPT_NFP_FWD)
		printf("--forward ");

	if (info->limit >= 0)
		printf("--limit %i ", info->limit);

	if (info->limit >= 0)
		printf("--limit-burst %i ", info->burst);

	if ((info->dscp >= 0) || (info->new_dscp >= 0))
		printf("dscp=0x%02X new_dscp=0x%02X ", info->dscp, info->new_dscp);

	if (info->txq >= 0)
		printf("--txq %i ", info->txq);

	if (info->txp >= 0)
		printf("--txp %i ", info->txp);

	if (info->mh >= 0)
		printf("--mh %i ", info->mh);

	if ((info->vlanprio >= 0) || (info->new_vlanprio >= 0))
		printf("vlan-prio=%i new-vlan-prio=%i ", info->vlanprio, info->new_vlanprio);
}

static void NFP_help(void)
{
	printf("NFP target options:\n"
		"--drop\n"
		"				generate NFP drop rules.\n"
		"--forward\n"
		"				generate NFP forward rules.\n"
		"--limit <n>\n"
		"				limit traffic to <n> kbps\n"
		"--limit-burst <n>\n"
		"				set maximum burst size to <n> kB\n"
		"--dscp <old> <new>\n"
		"				map old DSCP field in\n"
		"				IP header to a given value\n"
		"--dscp-def <n>\n"
		"				set DSCP field in\n"
		"				IP header to a given value\n"
		"--txq <n>\n"
		"				use selected TX queue.\n"
		"--txp <n>\n"
		"				use selected TX port.\n"
		"--mh  <n>\n"
		"				set Marvell Header field in\n"
		"				the packet to given value\n"
		"--vlan-prio <old> <new>\n"
		"				map old 802.1p priority in\n"
		"				802.1q header to given value.\n"
		"--vlan-prio-def <n>\n"
		"				set 802.1p priority in\n"
		"				802.1q header to given value.\n");
}

static const struct option NFP_opts[] = {
	{ .name = "drop", .has_arg = false, .val = 'D' },
	{ .name = "forward", .has_arg = false, .val = 'F' },
	{ .name = "limit", .has_arg = true, .val = 'L' },
	{ .name = "limit-burst", .has_arg = true, .val = 'B' },
	{ .name = "dscp", .has_arg = true, .val = 'T' },
	{ .name = "dscp-def", .has_arg = true, .val = 'U' },
	{ .name = "txq", .has_arg = true, .val = 'Q' },
	{ .name = "txp", .has_arg = true, .val = 'P' },
	{ .name = "mh", .has_arg = true, .val = 'M' },
	{ .name = "vlan-prio", .has_arg = true, .val = 'V' },
	{ .name = "vlan-prio-def", .has_arg = true, .val = 'W' },
	{ .name = NULL },
};

static struct xtables_target nfp6_target = {
	.family		= NFPROTO_IPV6,
	.name		= "NFP",
	.version	= XTABLES_VERSION,
	.size		= XT_ALIGN(sizeof(struct ipt_nfp_info)),
	.userspacesize	= offsetof(struct ipt_nfp_info, entry),
	.help		= NFP_help,
	.init		= NFP_init,
	.parse		= NFP_parse,
	.print		= NFP_print,
	.save		= NFP_save,
	.extra_opts	= NFP_opts,
};

void _init(void)
{
	xtables_register_target(&nfp6_target);
}
