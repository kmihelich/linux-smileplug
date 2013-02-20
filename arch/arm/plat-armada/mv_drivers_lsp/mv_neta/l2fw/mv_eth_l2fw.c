/* mv_eth_l2fw.c */
#include <linux/ctype.h>

#include "xor/mvXor.h"
#include "xor/mvXorRegs.h"
#include "mv_hal_if/mvSysXorApi.h"

#include "mvOs.h"
#include "mv_eth_l2fw.h"
#include "mv_neta/net_dev/mv_netdev.h"
#include "gbe/mvNeta.h"
#include "gbe/mvNetaRegs.h"
#include "nfp/mvNfp.h"
#include "mv_eth_l2fw.h"
#include "ctrlEnv/mvCtrlEnvLib.h"
#include "cesa/mvCesa.h"
#include "eth/nfp/mvNfpSec.h"

atomic_t req_count[2];

static int Qis_0_Full = 0;
static int Qis_1_Full = 0;

#define MV_NFP_SEC_REQ_Q_SIZE 1000
#define CESA_DEF_REQ_SIZE       (256*4)

extern u32 mv_crypto_virt_base_get(u8 chan);
enum {
	L2FW_1CORE=1,	
	L2FW_2CORES=2, 
	L4FW_4CORES=4,
};

static int l2fwMode = L2FW_2CORES;

static MV_NFP_SEC_SA_ENTRY sa;

typedef struct _mv_nfp_sec_cesa_priv_l2fw {
	MV_NFP_SEC_SA_ENTRY *pSaEntry;
	MV_PKT_INFO *pPktInfo;
	MV_U8 orgDigest[MV_CESA_MAX_DIGEST_SIZE];
	MV_CESA_COMMAND *pCesaCmd;
	struct eth_pbuf *pPkt;
	int ifout;
	int ownerId;
	int inPort;
} MV_NFP_SEC_CESA_PRIV_L2FW;


static MV_PKT_INFO *pPktInfoNewArray_0;
static MV_PKT_INFO *pPktInfoNewArray_1;
static MV_BUF_INFO *pBufInfoArray_0;
static MV_BUF_INFO *pBufInfoArray_1;

MV_BUF_INFO cesaBufs_0[CESA_DEF_REQ_SIZE];
MV_BUF_INFO cesaBufs_1[CESA_DEF_REQ_SIZE];

spinlock_t cesa_lock[2];
static int cesaPrivIndx_0 = 0;
static int cesaPrivIndx_1 = 0;

MV_NFP_SEC_CESA_PRIV_L2FW *cesaPrivArray_0;
MV_NFP_SEC_CESA_PRIV_L2FW *cesaPrivArray_1;

static int cesaCmdIndx_0 = 0;
static int cesaCmdIndx_1 = 0;


void *cesaOSHandle = NULL;
static MV_CESA_MBUF *cesaMbufArray_0;
static MV_CESA_MBUF *cesaMbufArray_1;

static MV_CESA_COMMAND *cesaCmdArray_0;
static MV_CESA_COMMAND *cesaCmdArray_1;

static int mv_eth_ports_l2fw_num;
static int espEnabled = 0;

static L2FW_RULE **l2fw_hash = NULL;
#define CESA_RESULT_Q_SIZE	1024
#define	L2FW_HASH_MASK   (L2FW_HASH_SIZE - 1)
unsigned int req_empty[2] = {0, 0};
unsigned int req_ready[2] = {0, 0};

static MV_U32 l2fw_jhash_iv;

static int numHashEntries;

static int counterNoResources[4] = {0, 0, 0, 0};

struct eth_port_l2fw **mv_eth_ports_l2fw;
static inline MV_STATUS mv_eth_l2fw_tx(struct eth_pbuf *pkt, struct eth_port *pp,
									   int withXor, struct neta_rx_desc *rx_desc);

void printkPkt(struct eth_pbuf *pkt);
void cesaStart(void);
static inline MV_STATUS mv_eth_cesa_l2fw_tx(struct eth_pbuf *pkt, struct eth_port *pp);


INLINE void  nfp_sec_pkt_info_free(struct eth_pbuf *pkt, int inPort)
{
	struct bm_pool *pool;
	pool = &mv_eth_pool[pkt->pool];

	mvStackPush(pool->stack, (MV_U32) pkt);

}

void mv_eth_pkt_print_new(struct eth_pbuf *pkt)
{
	printk(KERN_ERR "pkt: pkt=%p len=%d off=%d cmd=0x%x pool=%d "
	       "tos=%d gpon=0x%x\n skb=%p pa=%lx buf=%p\n",
	       pkt, pkt->bytes, pkt->offset, pkt->tx_cmd, pkt->pool,
	       pkt->tos, pkt->hw_cmd, pkt->osInfo, pkt->physAddr, pkt->pBuf);
}


void printBufVirtPtr(MV_BUF_INFO *pBuf)
{
	int i;
	if (pBuf->bufVirtPtr == NULL) {
		printk(KERN_INFO "pBuf->bufVirtPt==NULL in %s\n", __func__);
		return;
	}
	for (i = 0; i < 40; i++) {
		printk(KERN_INFO "KERN_INFO [%d]=%x ", i, pBuf->bufVirtPtr[i]);
		if (!(i%10) && i > 1)
			printk(KERN_INFO "\n");
	}
	printk(KERN_INFO "\n****************** %s\n", __func__);

}
void printBufInfo(MV_BUF_INFO *pbuf)
{
	printk(KERN_INFO "bufSize=%d\n"      , pbuf->bufSize);
	printk(KERN_INFO "dataSize=%d\n"     , pbuf->dataSize);
	printk(KERN_INFO "memHandle=%d\n"    , pbuf->memHandle);
	printk(KERN_INFO "bufAddrShift=%d\n" , pbuf->bufAddrShift);
	printk(KERN_INFO "*****************************************\n\n");

}


static inline void nfp_sec_complete_out(unsigned long data)

{
	MV_NFP_SEC_CESA_PRIV_L2FW *nfp_sec_cesa_priv = (MV_NFP_SEC_CESA_PRIV_L2FW *)data;		MV_U32            ifout;
	MV_PKT_INFO       *pkt;
	MV_BUF_INFO       *pBuf;
	struct eth_port   *pp;
	struct eth_pbuf   *pPkt;
	int oldOfsset;
	MV_STATUS status = MV_FAIL;
	static int counterOfFailed = 0;
	if (!nfp_sec_cesa_priv) {
		printk(KERN_INFO "nfp_sec_cesa_priv is NULL in %s\n", __func__);
		return;
	}
	ifout = nfp_sec_cesa_priv->ifout;

	pkt = nfp_sec_cesa_priv->pPktInfo;
	if (!pkt) {
		printk(KERN_INFO "pPktInfo is NULL in %s\n", __func__);
		return;
	}
	pBuf = pkt->pFrags;
	if (!pBuf) {
		printk(KERN_INFO "pBuf is NULL in %s\n", __func__);
		return;
	}
	pPkt = nfp_sec_cesa_priv->pPkt;
	if (!pPkt) {
		printk(KERN_INFO "!pPkt) in %s\n", __func__);
		return;
	}
	pPkt->bytes    = pBuf->dataSize;
	pPkt->bytes += MV_NFP_SEC_ESP_OFFSET;
	oldOfsset      = pPkt->offset;
	pPkt->offset   = pPkt->offset - (sizeof(MV_ESP_HEADER) + sizeof(MV_IP_HEADER) + MV_CESA_AES_BLOCK_SIZE);

	pp     = mv_eth_ports[ifout];

	status = 	mv_eth_cesa_l2fw_tx(pPkt, pp);
	if (status == MV_DROPPED)
		counterOfFailed++;
	 else
		pPkt->offset = oldOfsset;
}

void openCesaSession(void)
{
	unsigned char sha1Key[]  = {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
								0x24, 0x68, 0xac, 0xe0, 0x24, 0x68, 0xac, 0xe0,
								0x13, 0x57, 0x9b, 0xdf};
	/* sizeof(cryptoKey) should be 128 for AES-128 */
	unsigned char cryptoKey[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
									0x02, 0x46, 0x8a, 0xce, 0x13, 0x57, 0x9b, 0xdf};

	int i;
	MV_NFP_SEC_SA_ENTRY sa;
	MV_CESA_OPEN_SESSION os;
	unsigned short digest_size = 0;
	memset(&sa, 0, sizeof(MV_NFP_SEC_SA_ENTRY));
	memset(&os, 0, sizeof(MV_CESA_OPEN_SESSION));

	os.operation 		= MV_CESA_MAC_THEN_CRYPTO;
	os.cryptoAlgorithm  = MV_CESA_CRYPTO_AES;
	os.macMode  		= MV_CESA_MAC_HMAC_SHA1;
	digest_size 		= MV_CESA_SHA1_DIGEST_SIZE;
	os.cryptoMode 		= MV_CESA_CRYPTO_ECB;
	for (i = 0; i < sizeof(cryptoKey); i++)
		os.cryptoKey[i] = cryptoKey[i];

	os.cryptoKeyLength = sizeof(cryptoKey);

	for (i = 0; i < sizeof(sha1Key); i++)
		os.macKey[i] = sha1Key[i];
	os.macKeyLength = sizeof(sha1Key);
	os.digestSize = digest_size;

	if (mvCesaSessionOpen(&os, (short *)&(sa.sid)))
		printk(KERN_INFO "mvCesaSessionOpen failed in %s\n", __func__);
}


static s32 atoi(char *psz_buf)
{
	char *pch = psz_buf;
	s32 base = 0;
	unsigned long res;
	int ret_val;

	while (isspace(*pch))
			pch++;

	if (*pch == '-' || *pch == '+') {
			base = 10;
			pch++;
	} else if (*pch && tolower(pch[strlen(pch) - 1]) == 'h') {
			base = 16;
	}

	ret_val = strict_strtoul(pch, base, &res);

	return ret_val ? : res;
}



static L2FW_RULE *l2fw_lookup(MV_U32 srcIP, MV_U32 dstIP)
{
	MV_U32 hash;
	L2FW_RULE *rule;

	hash = mv_jhash_3words(srcIP, dstIP, (MV_U32) 0, l2fw_jhash_iv);
	hash &= L2FW_HASH_MASK;

	rule = l2fw_hash[hash];
#ifdef CONFIG_MV_ETH_L2FW_DEBUG
	printk(KERN_INFO "hash=%d in %s\n", hash, __func__);
	if (rule)
		printk(KERN_INFO "rule is not NULL in %s\n", __func__);
	else
		printk(KERN_INFO "rule is NULL in %s\n", __func__);
#endif
	while (rule) {
		if ((rule->srcIP == srcIP) && (rule->dstIP == dstIP))
			return rule;

		rule = rule->next;
	}
	return NULL;
}
void l2fw_mode_show(void)
{
	mvOsPrintf("mode is %d\n", l2fwMode);
}

void l2fw_stats(void)
{
	int i;
	for (i = 0; i < 4; i++) {
		mvOsPrintf("number of Cesa No Resources error is port[%d]=%d \n", i, counterNoResources[i]);
		counterNoResources[i] = 0;
	}

	mvOsPrintf("cesaCmdIndx_0 = %d \n",  cesaCmdIndx_0);
	mvOsPrintf("cesaCmdIndx_1 = %d \n",  cesaCmdIndx_1);
	mvOsPrintf("Qis_0_Full = %d \n", Qis_0_Full);
	mvOsPrintf("Qis_1_Full = %d \n", Qis_1_Full);
}

void l2fw_show_numHashEntries(void)
{
	mvOsPrintf("number of Hash Entries is %d \n", numHashEntries);

}


void l2fw_flush(void)
{
	MV_U32 i = 0;
	mvOsPrintf("\nFlushing L2fw Rule Database: \n");
	mvOsPrintf("*******************************\n");
	for (i = 0; i < L2FW_HASH_SIZE; i++)
		l2fw_hash[i] = NULL;
	numHashEntries = 0;
}

void l2fw_mode(int mode)
{
	mvOsPrintf("setting mode to %d\n", mode);
	if ( (mode != 1) && (mode != 2) && (mode != 4) )
		printk("mode must be 2 or 4, operation failed;  in %s\n",__FUNCTION__);
	else
		l2fwMode = mode;
}

void l2fw_dump(void)
{
	MV_U32 i = 0;
	L2FW_RULE *currRule;

	mvOsPrintf("\nPrinting L2fw Rule Database: \n");
	mvOsPrintf("*******************************\n");

	for (i = 0; i < L2FW_HASH_SIZE; i++) {
		currRule = l2fw_hash[i];
		while (currRule != NULL) {
			mvOsPrintf("%u.%u.%u.%u->%u.%u.%u.%u    out port=%d (hash=%x)\n",
				NIPQUAD(currRule->srcIP), NIPQUAD(currRule->dstIP),
				currRule->port, i);
			currRule = currRule->next;
		}
	}

}


MV_STATUS l2fw_add(MV_U32 srcIP, MV_U32 dstIP, int port)
{
	L2FW_RULE *l2fw_rule;

	MV_U32 hash = mv_jhash_3words(srcIP, dstIP, (MV_U32) 0, l2fw_jhash_iv);
	hash &= L2FW_HASH_MASK;
	if (numHashEntries == L2FW_HASH_SIZE) {
		printk(KERN_INFO "cannot add entry, hash table is full, there are %d entires \n", L2FW_HASH_SIZE);
		return MV_ERROR;
	}

#ifdef CONFIG_MV_ETH_L2FW_DEBUG
	mvOsPrintf("srcIP=%x dstIP=%x in %s\n", srcIP, dstIP, __func__);
	mvOsPrintf("srcIp = %u.%u.%u.%u in %s\n", NIPQUAD(srcIP), __func__);
	mvOsPrintf("dstIp = %u.%u.%u.%u in %s\n", NIPQUAD(dstIP), __func__);
#endif

	l2fw_rule = l2fw_lookup(srcIP, dstIP);
	if (l2fw_rule)
		return MV_OK;

	l2fw_rule = (L2FW_RULE *)mvOsMalloc(sizeof(L2FW_RULE));
	if (!l2fw_rule) {
		mvOsPrintf("%s: OOM\n", __func__);
		return MV_FAIL;
	}
#ifdef CONFIG_MV_ETH_L2FW_DEBUG
	mvOsPrintf("adding a rule to l2fw hash in %s\n", __func__);
#endif
	l2fw_rule->srcIP = srcIP;
	l2fw_rule->dstIP = dstIP;
	l2fw_rule->port = port;

	l2fw_rule->next = l2fw_hash[hash];
	l2fw_hash[hash] = l2fw_rule;
	numHashEntries++;
    return MV_OK;
}

MV_STATUS l2fw_add_ip(const char *buf)
{
	char *addr1, *addr2;
	L2FW_RULE *l2fw_rule;
	MV_U32 srcIP;
	MV_U32 dstIP;
	char dest1[15];
	char dest2[15];
	char *portStr;
	int offset1, offset2, port;
	MV_U32 hash    = 0;
	if (numHashEntries == L2FW_HASH_SIZE) {
		printk(KERN_INFO "cannot add entry, hash table is full, there are %d entires \n", L2FW_HASH_SIZE);
		return MV_ERROR;
	}

	memset(dest1,   0, sizeof(dest1));
	memset(dest2,   0, sizeof(dest2));

	addr1 = strchr(buf, ',');
	addr2 =	strchr(addr1+1, ',');
	offset1 = addr1-buf;
	offset2 = addr2-addr1;
	if (!addr1) {
			printk(KERN_INFO "first separating comma (',') missing in input in %s\n", __func__);
			return MV_FAIL;
	}
	if (!addr2) {
			printk(KERN_INFO "second separating comma (',') missing in input in %s\n", __func__);
			return MV_FAIL;
	}

	strncpy(dest1, buf, addr1-buf);
	srcIP = in_aton(dest1);
	strncpy(dest2, buf+offset1+1, addr2-addr1-1);
	dstIP = in_aton(dest2);
	portStr = addr2+1;
	if (*portStr == 'D') {
		L2FW_RULE *l2fw_rule_to_del, *prev;
		hash = mv_jhash_3words(srcIP, dstIP, (MV_U32) 0, l2fw_jhash_iv);
		hash &= L2FW_HASH_MASK;
		l2fw_rule_to_del = l2fw_hash[hash];
		prev = NULL;

		while (l2fw_rule_to_del) {
		if ((l2fw_rule_to_del->srcIP == srcIP) &&
			(l2fw_rule_to_del->dstIP == dstIP)) {
			if (prev)
				prev->next = l2fw_rule_to_del->next;
			else
				l2fw_hash[hash] = l2fw_rule_to_del->next;
			mvOsPrintf("%u.%u.%u.%u->%u.%u.%u.%u deleted\n", NIPQUAD(srcIP), NIPQUAD(dstIP));
			mvOsFree(l2fw_rule_to_del);
			numHashEntries--;
			return MV_OK;
		}

		prev = l2fw_rule_to_del;
		l2fw_rule_to_del = l2fw_rule_to_del->next;
	}
		mvOsPrintf("%u.%u.%u.%u->%u.%u.%u.%u : entry not found\n", NIPQUAD(srcIP), NIPQUAD(dstIP));
		return MV_NOT_FOUND;
	}

	port = atoi(portStr);
	hash = mv_jhash_3words(srcIP, dstIP, (MV_U32) 0, l2fw_jhash_iv);
	hash &= L2FW_HASH_MASK;

	l2fw_rule = l2fw_lookup(srcIP, dstIP);
	if (l2fw_rule) {
		mvOsPrintf("%u.%u.%u.%u->%u.%u.%u.%u : entry already exist\n",
				NIPQUAD(srcIP), NIPQUAD(dstIP));
		return MV_OK;
	}

	l2fw_rule = (L2FW_RULE *)mvOsMalloc(sizeof(L2FW_RULE));
	if (!l2fw_rule) {
		mvOsPrintf("%s: OOM\n", __func__);
		return MV_FAIL;
	}
#ifdef CONFIG_MV_ETH_L2FW_DEBUG
	mvOsPrintf("adding a rule to l2fw hash in %s\n", __func__);
#endif
	l2fw_rule->srcIP = srcIP;
	l2fw_rule->dstIP = dstIP;
	l2fw_rule->port = port;

	l2fw_rule->next = l2fw_hash[hash];
	l2fw_hash[hash] = l2fw_rule;
	numHashEntries++;
    return MV_OK;

}

void l2fw_esp_show(void)
{
	if (espEnabled)
		printk(KERN_INFO "ESP is enabled in %s\n", __func__);
	else
		printk(KERN_INFO "ESP is not enabled in %s\n", __func__);
}

void l2fw_esp_set(int enableEsp)
{
	if (enableEsp) {
		openCesaSession();
		printk(KERN_INFO "calling cesaStart() in %s\n", __func__);
		cesaStart();
	} else
		printk(KERN_INFO "enableEsp=%d disabling ESP in %s\n", enableEsp, __func__);
	espEnabled = enableEsp;
}


static inline
struct neta_tx_desc *mv_eth_tx_desc_get(struct tx_queue *txq_ctrl, int num)
{
	/* Is enough TX descriptors to send packet */
	if ((txq_ctrl->txq_count + num) >= txq_ctrl->txq_size) {
		/*
		printk("eth_tx: txq_ctrl->txq=%d - no_resource: txq_count=%d,
				txq_size=%d, num=%d\n",
				txq_ctrl->txq, txq_ctrl->txq_count,
				txq_ctrl->txq_size, num);
		*/
		STAT_DBG(txq_ctrl->stats.txq_err++);
		return NULL;
	}
	return mvNetaTxqNextDescGet(txq_ctrl->q);
}


static int mv_ctrl_txdone = CONFIG_MV_ETH_TXDONE_COAL_PKTS;
static void dump_xor(void)
{
	mvOsPrintf(" CHANNEL_ARBITER_REG %08x\n",
		MV_REG_READ(XOR_CHANNEL_ARBITER_REG(1)));
	mvOsPrintf(" CONFIG_REG          %08x\n",
		MV_REG_READ(XOR_CONFIG_REG(1, XOR_CHAN(0))));
	mvOsPrintf(" ACTIVATION_REG      %08x\n",
		MV_REG_READ(XOR_ACTIVATION_REG(1, XOR_CHAN(0))));
	mvOsPrintf(" CAUSE_REG           %08x\n",
		MV_REG_READ(XOR_CAUSE_REG(1)));
	mvOsPrintf(" MASK_REG            %08x\n",
		MV_REG_READ(XOR_MASK_REG(1)));
	mvOsPrintf(" ERROR_CAUSE_REG     %08x\n",
		MV_REG_READ(XOR_ERROR_CAUSE_REG(1)));
	mvOsPrintf(" ERROR_ADDR_REG      %08x\n",
		MV_REG_READ(XOR_ERROR_ADDR_REG(1)));
	mvOsPrintf(" NEXT_DESC_PTR_REG   %08x\n",
		MV_REG_READ(XOR_NEXT_DESC_PTR_REG(1, XOR_CHAN(0))));
	mvOsPrintf(" CURR_DESC_PTR_REG   %08x\n",
		MV_REG_READ(XOR_CURR_DESC_PTR_REG(1, XOR_CHAN(0))));
	mvOsPrintf(" BYTE_COUNT_REG      %08x\n\n",
		MV_REG_READ(XOR_BYTE_COUNT_REG(1, XOR_CHAN(0))));
	mvOsPrintf("  %08x\n\n", XOR_WINDOW_CTRL_REG(1, XOR_CHAN(0))) ;
		mvOsPrintf(" XOR_WINDOW_CTRL_REG      %08x\n\n",
		MV_REG_READ(XOR_WINDOW_CTRL_REG(1, XOR_CHAN(0)))) ;
}



/* L2fw defines */
#define L2FW_DISABLE				0
#define TX_AS_IS					1
#define SWAP_MAC					2
#define COPY_AND_SWAP		        3

#define XOR_CAUSE_DONE_MASK(chan) ((BIT0|BIT1) << (chan * 16))

static int         l2fw_xor_threshold = 200;
static MV_XOR_DESC *eth_xor_desc = NULL;
static MV_LONG      eth_xor_desc_phys_addr;


static int mv_eth_poll_l2fw(struct napi_struct *napi, int budget)
{
	int rx_done = 0;
	MV_U32 causeRxTx;
	static int counter = 0;

	struct eth_port *pp =
		container_of(napi, struct eth_port, napi[smp_processor_id()]);
	counter++;
	read_lock(&pp->rwlock);

	STAT_INFO(pp->stats.poll++);

	/* Read cause register */
	causeRxTx = MV_REG_READ(NETA_INTR_NEW_CAUSE_REG(pp->port)) &
	    (MV_ETH_MISC_SUM_INTR_MASK | MV_ETH_TXDONE_INTR_MASK |
		 MV_ETH_RX_INTR_MASK);

	if (causeRxTx & MV_ETH_MISC_SUM_INTR_MASK) {
		MV_U32 causeMisc;

		/* Process MISC events - Link, etc ??? */
		causeRxTx &= ~MV_ETH_MISC_SUM_INTR_MASK;
		causeMisc = MV_REG_READ(NETA_INTR_MISC_CAUSE_REG(pp->port));

		if (causeMisc & NETA_CAUSE_LINK_CHANGE_MASK)
			mv_eth_link_event(pp, 1);
		MV_REG_WRITE(NETA_INTR_MISC_CAUSE_REG(pp->port), 0);
	}

	causeRxTx |= pp->causeRxTx[smp_processor_id()];
#ifdef CONFIG_MV_ETH_TXDONE_ISR
	if (causeRxTx & MV_ETH_TXDONE_INTR_MASK) {
		/* TX_DONE process */

		mv_eth_tx_done_gbe(pp,
				(causeRxTx & MV_ETH_TXDONE_INTR_MASK));

		causeRxTx &= ~MV_ETH_TXDONE_INTR_MASK;
	}
#endif /* CONFIG_MV_ETH_TXDONE_ISR */

#if (CONFIG_MV_ETH_RXQ > 1)
	while ((causeRxTx != 0) && (budget > 0)) {
		int count, rx_queue;

		rx_queue = mv_eth_rx_policy(causeRxTx);
		if (rx_queue == -1)
			break;

		count = mv_eth_rx_l2f(pp, budget, rx_queue);
		rx_done += count;
		budget -= count;
		if (budget > 0)
			causeRxTx &=
			 ~((1 << rx_queue) << NETA_CAUSE_RXQ_OCCUP_DESC_OFFS);
	}
#else
	rx_done = mv_eth_rx_l2f(pp, budget, CONFIG_MV_ETH_RXQ_DEF);
	budget -= rx_done;
#endif /* (CONFIG_MV_ETH_RXQ > 1) */


	if (budget > 0) {
		unsigned long flags;
		causeRxTx = 0;

		napi_complete(&pp->napi[smp_processor_id()]);
		STAT_INFO(pp->stats.poll_exit++);

		local_irq_save(flags);
		MV_REG_WRITE(NETA_INTR_NEW_MASK_REG(pp->port),
			(MV_ETH_MISC_SUM_INTR_MASK | MV_ETH_TXDONE_INTR_MASK |
				  MV_ETH_RX_INTR_MASK));

		local_irq_restore(flags);
	}
	pp->causeRxTx[smp_processor_id()] = causeRxTx;

	read_unlock(&pp->rwlock);

	return rx_done;
}


void mv_eth_set_l2fw(int cmd, int rx_port, int out_tx_port)
{
	int cpu;
	struct eth_port *pp;
	struct net_device *dev;
	pp     = mv_eth_ports[rx_port];

	if (!pp) {
		mvOsPrintf("pp is NULL in setting L2FW (%s)\n", __func__);
		return;
	}
	clear_bit(MV_ETH_F_CONNECT_LINUX_BIT, &(pp->flags));

	dev = pp->dev;
	if (dev == NULL) {
		mvOsPrintf("device is NULL in in setting L2FW (%s)\n", __func__);
		return;
	}
	if (!test_bit(MV_ETH_F_STARTED_BIT, &(pp->flags))) {
		mvOsPrintf("Device is down for port=%d ; MV_ETH_F_STARTED_BIT is not set in %s\n", rx_port, __func__);
		mvOsPrintf("Cannot set to L2FW mode in %s\n", __func__);
		return;
	}

	for_each_possible_cpu(cpu)
		{
		if (cmd == L2FW_DISABLE) {
			if (test_bit(MV_ETH_F_STARTED_BIT, &(pp->flags)))
				napi_disable(&pp->napi[cpu]);
			netif_napi_del(&pp->napi[cpu]);
			netif_napi_add(dev, &pp->napi[cpu], mv_eth_poll,
				weightArr[pp->port]);
			if (test_bit(MV_ETH_F_STARTED_BIT, &(pp->flags)))
				napi_enable(&pp->napi[cpu]);
		} else {
			if (test_bit(MV_ETH_F_STARTED_BIT, &(pp->flags)))
				napi_disable(&pp->napi[cpu]);
			netif_napi_del(&pp->napi[cpu]);
			netif_napi_add(dev, &pp->napi[cpu], mv_eth_poll_l2fw,
				weightArr[pp->port]);
			if (test_bit(MV_ETH_F_STARTED_BIT, &(pp->flags)))
				napi_enable(&pp->napi[cpu]);
			}
		}
}


static inline struct eth_pbuf *l2fw_swap_mac(struct eth_pbuf *pRxPktInfo)
{
	MV_U16 *pSrc;
	int i;
	MV_U16 swap;
	pSrc = (MV_U16 *)(pRxPktInfo->pBuf + pRxPktInfo->offset + MV_ETH_MH_SIZE);

	for (i = 0; i < 3; i++) {
		swap = pSrc[i];
		pSrc[i] = pSrc[i+3];
		pSrc[i+3] = swap;
		}

	return  pRxPktInfo;
}

static inline void l2fw_copy_mac(struct eth_pbuf *pRxPktInfo,
					 struct eth_pbuf *pTxPktInfo)
	{
	/* copy 30 bytes (start after MH header) */
    /* 12 for SA + DA */
	/* 18 for the rest */
	MV_U16 *pSrc;
	MV_U16 *pDst;
	int i;
	pSrc = (MV_U16 *)(pRxPktInfo->pBuf + pRxPktInfo->offset + MV_ETH_MH_SIZE);
	pDst = (MV_U16 *)(pTxPktInfo->pBuf + pTxPktInfo->offset + MV_ETH_MH_SIZE);
	/* swap mac SA and DA */
	for (i = 0; i < 3; i++) {
		pDst[i]   = pSrc[i+3];
		pDst[i+3] = pSrc[i];
		}
	for (i = 6; i < 15; i++)
		pDst[i] = pSrc[i];
	}

static inline void l2fw_copy_and_swap_mac(struct eth_pbuf *pRxPktInfo, struct eth_pbuf *pTxPktInfo)
{
	MV_U16 *pSrc;
	MV_U16 *pDst;
	int i;

	pSrc = (MV_U16 *)(pRxPktInfo->pBuf +  pRxPktInfo->offset + MV_ETH_MH_SIZE);
	pDst = (MV_U16 *)(pTxPktInfo->pBuf +  pTxPktInfo->offset + MV_ETH_MH_SIZE);
	for (i = 0; i < 3; i++) {
		pDst[i]   = pSrc[i+3];
		pDst[i+3] = pSrc[i];
		}
}

static inline
struct eth_pbuf *eth_l2fw_copy_packet_withoutXor(struct eth_pbuf *pRxPktInfo)
{
	MV_U8 *pSrc;
	MV_U8 *pDst;
	struct bm_pool *pool;
	struct eth_pbuf *pTxPktInfo;

	mvOsCacheInvalidate(NULL, pRxPktInfo->pBuf + pRxPktInfo->offset,
						pRxPktInfo->bytes);

	pool = &mv_eth_pool[pRxPktInfo->pool];
	pTxPktInfo = mv_eth_pool_get(pool);
	if (pTxPktInfo == NULL) {
		mvOsPrintf("pTxPktInfo == NULL in %s\n", __func__);
		return NULL;
		}
	pSrc = pRxPktInfo->pBuf +  pRxPktInfo->offset + MV_ETH_MH_SIZE;
	pDst = pTxPktInfo->pBuf +  pTxPktInfo->offset + MV_ETH_MH_SIZE;

	memcpy(pDst+12, pSrc+12, pRxPktInfo->bytes-12);
	l2fw_copy_and_swap_mac(pRxPktInfo, pTxPktInfo);
	pTxPktInfo->bytes = pRxPktInfo->bytes;
	mvOsCacheFlush(NULL, pTxPktInfo->pBuf + pTxPktInfo->offset, pTxPktInfo->bytes);

	return pTxPktInfo;
}

static inline
struct eth_pbuf *eth_l2fw_copy_packet_withXor(struct eth_pbuf *pRxPktInfo)
{
	struct bm_pool *pool;
	struct eth_pbuf *pTxPktInfo;

	pool = &mv_eth_pool[pRxPktInfo->pool];
	pTxPktInfo = mv_eth_pool_get(pool);
	if (pTxPktInfo == NULL) {
		mvOsPrintf("pTxPktInfo == NULL in %s\n", __func__);
		return NULL;
		}
	eth_xor_desc->srcAdd0    = pRxPktInfo->physAddr + pRxPktInfo->offset + MV_ETH_MH_SIZE + 30;
	eth_xor_desc->phyDestAdd = pTxPktInfo->physAddr + pTxPktInfo->offset + MV_ETH_MH_SIZE + 30;

	eth_xor_desc->byteCnt    = pRxPktInfo->bytes - 30;

	eth_xor_desc->phyNextDescPtr = 0;
	eth_xor_desc->status         = BIT31;
	/* we had changed only the first part of eth_xor_desc, so flush only one
	 line of cache */
	mvOsCacheLineFlush(NULL, eth_xor_desc);

	MV_REG_WRITE(XOR_NEXT_DESC_PTR_REG(1, XOR_CHAN(0)), eth_xor_desc_phys_addr);

	MV_REG_WRITE(XOR_ACTIVATION_REG(1, XOR_CHAN(0)), XEXACTR_XESTART_MASK);

	mvOsCacheLineInv(NULL, pRxPktInfo->pBuf + pRxPktInfo->offset);
	l2fw_copy_mac(pRxPktInfo, pTxPktInfo);
	mvOsCacheLineFlush(NULL, pTxPktInfo->pBuf + pTxPktInfo->offset);

    /* Update TxPktInfo */
	pTxPktInfo->bytes = pRxPktInfo->bytes;
	return pTxPktInfo;
}

void setXorDesc(void)
{
	unsigned int mode;
	eth_xor_desc = mvOsMalloc(sizeof(MV_XOR_DESC) + XEXDPR_DST_PTR_DMA_MASK + 32);
	eth_xor_desc = (MV_XOR_DESC *)MV_ALIGN_UP((MV_U32)eth_xor_desc, XEXDPR_DST_PTR_DMA_MASK+1);
	eth_xor_desc_phys_addr = mvOsIoVirtToPhys(NULL, eth_xor_desc);
	mvSysXorInit();

	mode = MV_REG_READ(XOR_CONFIG_REG(1, XOR_CHAN(0)));
	mode &= ~XEXCR_OPERATION_MODE_MASK;
	mode |= XEXCR_OPERATION_MODE_DMA;
	MV_REG_WRITE(XOR_CONFIG_REG(1, XOR_CHAN(0)), mode);

    MV_REG_WRITE(XOR_NEXT_DESC_PTR_REG(1, XOR_CHAN(0)), eth_xor_desc_phys_addr);
	dump_xor();
}


static inline int xorReady(void)
{
	int timeout = 0;

	while (!(MV_REG_READ(XOR_CAUSE_REG(1)) & XOR_CAUSE_DONE_MASK(XOR_CHAN(0)))) {
		if (timeout > 0x100000) {
			mvOsPrintf("XOR timeout\n");
			return 0;
			}
		timeout++;
	}

	/* Clear int */
	MV_REG_WRITE(XOR_CAUSE_REG(1), ~(XOR_CAUSE_DONE_MASK(XOR_CHAN(0))));

	return 1;
}


void l2fw(int cmd, int rx_port, int tx_port)
{
	struct eth_port_l2fw *ppl2fw;
	int port;
	MV_U32 regVal;
	mv_eth_ports_l2fw_num = mvCtrlEthMaxPortGet();
	ppl2fw = mv_eth_ports_l2fw[rx_port];
	mvOsPrintf("cmd=%d rx_port=%d tx_port=%d in %s \n",
				cmd, rx_port, tx_port, __func__);
	ppl2fw->txPort = tx_port;
	ppl2fw->cmd	= cmd;
	mv_eth_set_l2fw(cmd, rx_port, tx_port);
	regVal = 0;
	regVal |= ETH_TX_BURST_SIZE_MASK(ETH_BURST_SIZE_16_64BIT_VALUE);
	regVal |= ETH_RX_BURST_SIZE_MASK(ETH_BURST_SIZE_16_64BIT_VALUE);
	regVal |= (ETH_RX_NO_DATA_SWAP_MASK | ETH_TX_NO_DATA_SWAP_MASK | ETH_NO_DESC_SWAP_MASK);
	for (port = 0; port < mv_eth_ports_l2fw_num; port++) {
	/* Assign port SDMA configuration */

	/* setting Tx/Rx burst
		TBD - unset BURST size back to default when exiting L2FW mode */

		MV_REG_WRITE(ETH_SDMA_CONFIG_REG(port), regVal);
	}
}

void l2fw_xor(int threshold)
{
	mvOsPrintf("setting threshold to %d in %s\n", threshold, __func__);
	l2fw_xor_threshold = threshold;
}

void printCesaCmd(MV_CESA_COMMAND *pCesaCmd)
{
	printk(KERN_INFO "pCesaCmd->sessionId = %d in %s\n", pCesaCmd->sessionId, __func__);
	printk(KERN_INFO "ivFromUser =%d\n", pCesaCmd->ivFromUser);
	printk(KERN_INFO"ivOffset=%d\n", pCesaCmd->ivOffset);
	printk(KERN_INFO "cryptoOffset=%d\n", pCesaCmd->cryptoOffset);
	printk(KERN_INFO "cryptoLength=%d\n", pCesaCmd->cryptoLength);
	printk(KERN_INFO "digestOffset=%d\n", pCesaCmd->digestOffset);
	printk(KERN_INFO "macOffset=%d\n", pCesaCmd->macOffset);
	printk(KERN_INFO "macLength=%d\n", pCesaCmd->macLength);
	printk(KERN_INFO "reqId=%d\n", pCesaCmd->reqId);
	printk(KERN_INFO "pCesaCmd->pDst->pFrags[0].bufSize=%d\n", pCesaCmd->pDst->pFrags[0].bufSize);
}

INLINE MV_VOID mvNfpSecBuildMac(MV_PKT_INFO *pPktInfo, MV_NFP_SEC_SA_ENTRY* pSAEntry)
{
	MV_802_3_HEADER *pMacHdr;

	pMacHdr = (MV_802_3_HEADER *)((MV_U8 *)(pPktInfo->pFrags[0].bufVirtPtr));
	memcpy(pMacHdr, &pSAEntry->tunnelHdr.dstMac, 12);
	/* stands for IP protocol code 16bit swapped */
	pMacHdr->typeOrLen = 0x08;
	return;
}

INLINE MV_VOID mvNfpSecBuildIPTunnel(MV_PKT_INFO *pPktInfo, MV_NFP_SEC_SA_ENTRY *pSAEntry)
{
	MV_IP_HEADER *pIpHdr, *pIntIpHdr;
	MV_U16 newIpTotalLength;

	newIpTotalLength = pPktInfo->pFrags[0].dataSize - sizeof(MV_802_3_HEADER);

	pIpHdr = (MV_IP_HEADER *)(pPktInfo->pFrags[0].bufVirtPtr +
				sizeof(MV_802_3_HEADER));
	pIntIpHdr = (MV_IP_HEADER *)((MV_U8 *)(pIpHdr) + sizeof(MV_IP_HEADER) + sizeof(MV_ESP_HEADER) +
			pSAEntry->ivSize);

	pIpHdr->version = 0x45;
	pIpHdr->tos = 0;
	pIpHdr->checksum = 0;
	pIpHdr->totalLength = MV_16BIT_BE(newIpTotalLength);
	pIpHdr->identifier = 0;
	pIpHdr->fragmentCtrl = 0;
	pIpHdr->ttl = pIntIpHdr->ttl - 1 ;
	pIpHdr->protocol = MV_IP_PROTO_ESP;
	pIpHdr->srcIP = pSAEntry->tunnelHdr.sIp;
	pIpHdr->dstIP = pSAEntry->tunnelHdr.dIp;
	return;
}

/* Append sequence number and spi, save some space for IV */
INLINE MV_VOID mvNfpSecBuildEspHdr(MV_PKT_INFO *pPktInfo, MV_NFP_SEC_SA_ENTRY* pSAEntry)
{
	MV_ESP_HEADER *pEspHdr;

	pEspHdr = (MV_ESP_HEADER *)(pPktInfo->pFrags[0].bufVirtPtr +
			sizeof(MV_802_3_HEADER) + sizeof(MV_IP_HEADER));
	pEspHdr->spi = pSAEntry->spi;
	pSAEntry->seqNum = (pSAEntry->seqNum++);
	pEspHdr->seqNum = pSAEntry->seqNum;
}

void printEspHdr(MV_ESP_HEADER *pEspHdr)
{
	printk(KERN_INFO "pEspHdr->spi=%d in %s\n"  , pEspHdr->spi, __func__);
	printk(KERN_INFO "pEspHdr->seqNum=%d in %s\n", pEspHdr->seqNum, __func__);
}

void printIpHdr(MV_IP_HEADER *pIpHdr)
{
	printk(KERN_INFO "%u.%u.%u.%u->%u.%u.%u.%u in %s\n", NIPQUAD(pIpHdr->srcIP), NIPQUAD(pIpHdr->dstIP), __func__);
	printk(KERN_INFO "MV_16BIT_BE(pIpHdr->totalLength)=%d  in %s\n", MV_16BIT_BE(pIpHdr->totalLength), __func__);
	printk(KERN_INFO "pIpHdr->protocol=%d \n", pIpHdr->protocol);
}

void printCesaMbuf(MV_CESA_MBUF *pCesaMbuf)
{
	printk(KERN_INFO "pCesaMbuf->pFrags->bufSize=%d  \n", pCesaMbuf->pFrags->bufSize);
	printk(KERN_INFO "pCesaMbuf->pFrags->dataSize=%d  \n", pCesaMbuf->pFrags->dataSize);
	printk(KERN_INFO "pCesaMbuf->pFrags->bufAddrShift=%d  \n", pCesaMbuf->pFrags->bufAddrShift);
	printk(KERN_INFO "pCesaMbuf->mbufSize=%d  \n", pCesaMbuf->mbufSize);
}


inline MV_STATUS mvSecEspProcess_0(struct eth_pbuf *pPkt, MV_PKT_INFO *pPktInfo,
							MV_NFP_SEC_SA_ENTRY *pSAEntry, struct eth_port *newpp,
							MV_U8 channel, int inPort)
{
	MV_CESA_COMMAND	*pCesaCmd;
	MV_CESA_MBUF *pCesaMbuf;
	MV_NFP_SEC_CESA_PRIV_L2FW *pCesaPriv;
	MV_STATUS status;
	MV_IP_HEADER *pIpHdr;
	MV_BUF_INFO  *pBuf;

	pCesaCmd  = &cesaCmdArray_0[cesaCmdIndx_0];
	pCesaMbuf = &cesaMbufArray_0[cesaCmdIndx_0];
	cesaCmdIndx_0++;

	cesaCmdIndx_0 %= CESA_DEF_REQ_SIZE;
	pCesaPriv = &cesaPrivArray_0[cesaPrivIndx_0++];

	cesaPrivIndx_0 = cesaPrivIndx_0%(CESA_DEF_REQ_SIZE + MV_NFP_SEC_REQ_Q_SIZE);

	pCesaPriv->pPktInfo = pPktInfo;
	pCesaPriv->pSaEntry = pSAEntry;
	pCesaPriv->pCesaCmd = pCesaCmd;

	pCesaPriv->pPkt   = pPkt;
	pCesaPriv->ifout  = newpp->port;
	pCesaPriv->inPort = inPort;
	/*
	 *  Fix, encrypt/decrypt the IP payload only, --BK 20091027
	 */
	pBuf = pPktInfo->pFrags;
	pIpHdr = (MV_IP_HEADER *)(pBuf->bufVirtPtr + sizeof(MV_802_3_HEADER));
	pBuf->dataSize = MV_16BIT_BE(pIpHdr->totalLength) + sizeof(MV_802_3_HEADER);
	/* after next command, pBuf->bufVirtPtr will point to ESP */
	pBuf->bufVirtPtr += MV_NFP_SEC_ESP_OFFSET;
	pBuf->bufPhysAddr += MV_NFP_SEC_ESP_OFFSET;
	pBuf->dataSize -= MV_NFP_SEC_ESP_OFFSET;

	pBuf->bufAddrShift -= MV_NFP_SEC_ESP_OFFSET;
	pCesaMbuf->pFrags = pPktInfo->pFrags;
	pCesaMbuf->numFrags = 1;
	pCesaMbuf->mbufSize = pBuf->dataSize;

	pCesaMbuf->pFrags->bufSize = pBuf->dataSize;

	pCesaCmd->pReqPrv = (MV_VOID *)pCesaPriv;
	pCesaCmd->sessionId = pSAEntry->sid;
	pCesaCmd->pSrc = pCesaMbuf;
	pCesaCmd->pDst = pCesaMbuf;
	pCesaCmd->skipFlush = MV_TRUE;

	/* Assume ESP */
	pCesaCmd->cryptoOffset = sizeof(MV_ESP_HEADER) + pSAEntry->ivSize;
	pCesaCmd->cryptoLength =  pBuf->dataSize - (sizeof(MV_ESP_HEADER)
				  + pSAEntry->ivSize + pSAEntry->digestSize);
	pCesaCmd->ivFromUser = 0; /* relevant for encode only */
	pCesaCmd->ivOffset = sizeof(MV_ESP_HEADER);
	pCesaCmd->macOffset = 0;
	pCesaCmd->macLength = pBuf->dataSize - pSAEntry->digestSize;
	if ((pCesaCmd->digestOffset != 0) && ((pCesaCmd->digestOffset%4)))  {
		printk(KERN_INFO "pBuf->dataSize=%d pSAEntry->digestSize=%d in %s\n",
			pBuf->dataSize, pSAEntry->digestSize, __func__);
		printk(KERN_INFO "pCesaCmd->digestOffset=%d in %s\n",
			pCesaCmd->digestOffset, __func__);
	}
	pCesaCmd->digestOffset = pBuf->dataSize - pSAEntry->digestSize ;

#ifdef CONFIG_MV_ETH_L2FW_DEBUG
	printCesaCmd(pCesaCmd);
	printCesaMbuf(pCesaMbuf);
#endif

	disable_irq(CESA_IRQ(channel));
	status = mvCesaAction(channel, pCesaCmd);
	enable_irq(CESA_IRQ(channel));
	if (status != MV_OK) {
		pSAEntry->stats.rejected++;
		mvOsPrintf("%s: mvCesaAction failed %d\n", __func__, status);
	}
	return status;
}

inline MV_STATUS mvSecEspProcess_1(struct eth_pbuf *pPkt, MV_PKT_INFO *pPktInfo,
						  MV_NFP_SEC_SA_ENTRY *pSAEntry, struct eth_port *newpp,
						  MV_U8 channel, int inPort)

{
	MV_CESA_COMMAND	*pCesaCmd;
	MV_CESA_MBUF *pCesaMbuf;
	MV_NFP_SEC_CESA_PRIV_L2FW *pCesaPriv;
	MV_STATUS status;
	MV_IP_HEADER *pIpHdr;
	MV_BUF_INFO  *pBuf;
	pCesaCmd  = &cesaCmdArray_1[cesaCmdIndx_1];
	pCesaMbuf = &cesaMbufArray_1[cesaCmdIndx_1];
	cesaCmdIndx_1++;
	cesaCmdIndx_1 %= CESA_DEF_REQ_SIZE;
	pCesaPriv = &cesaPrivArray_1[cesaPrivIndx_1++];
	cesaPrivIndx_1 = cesaPrivIndx_1%(CESA_DEF_REQ_SIZE + MV_NFP_SEC_REQ_Q_SIZE);

	pCesaPriv->pPktInfo = pPktInfo;
	pCesaPriv->pSaEntry = pSAEntry;
	pCesaPriv->pCesaCmd = pCesaCmd;

	pCesaPriv->pPkt   = pPkt;
	pCesaPriv->ifout  = newpp->port;
	pCesaPriv->inPort = inPort;
	/*
	 *  Fix, encrypt/decrypt the IP payload only, --BK 20091027
	 */
	pBuf = pPktInfo->pFrags;
	pIpHdr = (MV_IP_HEADER *)(pBuf->bufVirtPtr + sizeof(MV_802_3_HEADER));
	pBuf->dataSize = MV_16BIT_BE(pIpHdr->totalLength) + sizeof(MV_802_3_HEADER);
	/* after next command, pBuf->bufVirtPtr will point to ESP */
	pBuf->bufVirtPtr += MV_NFP_SEC_ESP_OFFSET;
	pBuf->bufPhysAddr += MV_NFP_SEC_ESP_OFFSET;
	pBuf->dataSize -= MV_NFP_SEC_ESP_OFFSET;
	pBuf->bufAddrShift -= MV_NFP_SEC_ESP_OFFSET;
	pCesaMbuf->pFrags = pPktInfo->pFrags;
	pCesaMbuf->numFrags = 1;
	pCesaMbuf->mbufSize = pBuf->dataSize;
	pCesaMbuf->pFrags->bufSize = pBuf->dataSize;

	pCesaCmd->pReqPrv = (MV_VOID *)pCesaPriv;
	pCesaCmd->sessionId = pSAEntry->sid;
	pCesaCmd->pSrc = pCesaMbuf;
	pCesaCmd->pDst = pCesaMbuf;
	pCesaCmd->skipFlush = MV_TRUE;

	/* Assume ESP */
	pCesaCmd->cryptoOffset = sizeof(MV_ESP_HEADER) + pSAEntry->ivSize;
	pCesaCmd->cryptoLength =  pBuf->dataSize - (sizeof(MV_ESP_HEADER)
				  + pSAEntry->ivSize + pSAEntry->digestSize);
	pCesaCmd->ivFromUser = 0; /* relevant for encode only */
	pCesaCmd->ivOffset = sizeof(MV_ESP_HEADER);
	pCesaCmd->macOffset = 0;
	pCesaCmd->macLength = pBuf->dataSize - pSAEntry->digestSize;
	if ((pCesaCmd->digestOffset != 0) && ((pCesaCmd->digestOffset%4)))  {
		printk(KERN_INFO "pBuf->dataSize=%d pSAEntry->digestSize=%d in %s\n",
			pBuf->dataSize, pSAEntry->digestSize, __func__);
		printk(KERN_INFO "pCesaCmd->digestOffset=%d in %s\n",
			pCesaCmd->digestOffset, __func__);
	}
	pCesaCmd->digestOffset = pBuf->dataSize - pSAEntry->digestSize ;

#ifdef CONFIG_MV_ETH_L2FW_DEBUG
	printCesaCmd(pCesaCmd);
	printCesaMbuf(pCesaMbuf);
	printk(KERN_INFO "Calling mvCesaAction in %s\n", __func__);
#endif
	disable_irq(CESA_IRQ(channel));
	status = mvCesaAction(channel, pCesaCmd);
	enable_irq(CESA_IRQ(channel));
	if (status != MV_OK) {
		pSAEntry->stats.rejected++;
		mvOsPrintf("%s: mvCesaAction failed %d\n", __func__, status);
	}

	return status;
}



inline MV_STATUS mvSecOutgoing(struct eth_pbuf *pkt, MV_PKT_INFO *pPktInfo,
						MV_NFP_SEC_SA_ENTRY *pSAEntry, struct eth_port *new_pp,
						int inPort, MV_U8 chan)
{
	MV_U8 *pTmp;
	MV_U32 cryptoSize, encBlockMod, dSize;
	MV_BUF_INFO *pBuf = pPktInfo->pFrags;
	/* CESA Q is full drop. */
	if (cesaReqResources[chan] <= 1) {
		counterNoResources[inPort]++;
		return MV_DROPPED;
	}
	cryptoSize = pBuf->dataSize - sizeof(MV_802_3_HEADER);

	/* Align buffer address to beginning of new packet - TBD handle VLAN tag, LLC */
	dSize = pSAEntry->ivSize + sizeof(MV_ESP_HEADER) + sizeof(MV_IP_HEADER);
	pBuf->bufVirtPtr -= dSize;
	pBuf->bufPhysAddr -= dSize;
	pBuf->dataSize += dSize;
	pBuf->bufAddrShift += dSize;

	encBlockMod = (cryptoSize % MV_NFP_SEC_ENC_BLOCK_SIZE);
	/* leave space for padLen + Protocol */
	if (encBlockMod > 14) {
		encBlockMod =  MV_NFP_SEC_ENC_BLOCK_SIZE - encBlockMod;
		encBlockMod += MV_NFP_SEC_ENC_BLOCK_SIZE;
	} else
		encBlockMod =  MV_NFP_SEC_ENC_BLOCK_SIZE - encBlockMod;
	/* expected frame size */
	dSize = pBuf->dataSize + encBlockMod + pSAEntry->digestSize;

	pBuf->dataSize += encBlockMod;
	pTmp = pBuf->bufVirtPtr + pBuf->dataSize;
	memset(pTmp - encBlockMod, 0, encBlockMod - 2);
	*((MV_U8 *)(pTmp-2)) = (MV_U8)(encBlockMod-2);
	*((MV_U8 *)(pTmp-1)) = (MV_U8)4;

	pBuf->dataSize += pSAEntry->digestSize;

	mvNfpSecBuildEspHdr(pPktInfo, pSAEntry);
	mvNfpSecBuildIPTunnel(pPktInfo, pSAEntry);
	mvNfpSecBuildMac(pPktInfo, pSAEntry);

	/* flush & invalidate new MAC, IP, & ESP headers + old ip*/
	dSize = pBuf->bufAddrShift + sizeof(MV_IP_HEADER) + sizeof(MV_802_3_HEADER);

	if (chan == 0)
	  return mvSecEspProcess_0(pkt, pPktInfo, pSAEntry, new_pp, chan, inPort);
	else
	  return mvSecEspProcess_1(pkt, pPktInfo, pSAEntry, new_pp, chan, inPort);
}


static inline MV_STATUS handleEsp(struct eth_pbuf *pkt, struct neta_rx_desc *rx_desc,
							struct eth_port  *new_pp, int inPort, MV_U8 chan)
{
	if (chan == 0) {
		pBufInfoArray_0[cesaCmdIndx_0].bufAddrShift = 0;
		pBufInfoArray_0[cesaCmdIndx_0].dataSize    = pkt->bytes;

		pBufInfoArray_0[cesaCmdIndx_0].bufSize     = pkt->bytes;
		pBufInfoArray_0[cesaCmdIndx_0].bufVirtPtr  = pkt->pBuf + pkt->offset + MV_ETH_MH_SIZE;

		pBufInfoArray_0[cesaCmdIndx_0].bufPhysAddr = mvOsIoVirtToPhy(NULL, pkt->pBuf + pkt->offset + MV_ETH_MH_SIZE);
		pBufInfoArray_0[cesaCmdIndx_0].memHandle   = 0;

		pPktInfoNewArray_0[cesaCmdIndx_0].pFrags = &pBufInfoArray_0[cesaCmdIndx_0];
		pPktInfoNewArray_0[cesaCmdIndx_0].numFrags = 1;
	} else {
		pBufInfoArray_1[cesaCmdIndx_1].bufAddrShift = 0;
		pBufInfoArray_1[cesaCmdIndx_1].dataSize    = pkt->bytes;

		pBufInfoArray_1[cesaCmdIndx_1].bufSize     = pkt->bytes;
		pBufInfoArray_1[cesaCmdIndx_1].bufVirtPtr  = pkt->pBuf + pkt->offset + MV_ETH_MH_SIZE;

		pBufInfoArray_1[cesaCmdIndx_1].bufPhysAddr = mvOsIoVirtToPhy(NULL, pkt->pBuf + pkt->offset + MV_ETH_MH_SIZE);
		pBufInfoArray_1[cesaCmdIndx_1].memHandle   = 0;

		pPktInfoNewArray_1[cesaCmdIndx_1].pFrags = &pBufInfoArray_1[cesaCmdIndx_1];
		pPktInfoNewArray_1[cesaCmdIndx_1].numFrags = 1;
	}

	if (chan == 0)
		return mvSecOutgoing(pkt, &pPktInfoNewArray_0[cesaCmdIndx_0], &sa, new_pp, inPort, chan);
	else
		return mvSecOutgoing(pkt, &pPktInfoNewArray_1[cesaCmdIndx_1], &sa, new_pp, inPort, chan);
}

void printkPkt(struct eth_pbuf *pkt)
{
	int i;
	MV_IP_HEADER  *pIpHdr  = (MV_IP_HEADER *)(pkt->pBuf+16);
	MV_ESP_HEADER *pEspHdr = (MV_ESP_HEADER *)(pkt->pBuf+16+sizeof(MV_IP_HEADER));
	printk(KERN_INFO "****************** \n");
	printk(KERN_INFO "printkPkt, pkt->bytes=%d \n", pkt->bytes);
	for (i = 0; i < pkt->bytes; i++) {
		printk(KERN_INFO "[%d]=%x", i, pkt->pBuf[i]);
		if (!(i%10) && (i > 1))
			printk(KERN_INFO "\n");
	}
	printk(KERN_INFO "\n");
	printIpHdr(pIpHdr);

	printEspHdr(pEspHdr);
	printk(KERN_INFO "****************** \n");
}



static inline MV_STATUS mv_eth_l2fw_tx(struct eth_pbuf *pkt, struct eth_port *pp, int withXor,
									   struct neta_rx_desc *rx_desc)
{
	struct neta_tx_desc *tx_desc;
	u32 tx_cmd = 0;
	struct tx_queue *txq_ctrl;
	/* assigning different txq for each rx port , to avoid waiting on the
	same txq lock when traffic on several rx ports are destined to the same
	outgoing interface */
	int txq = 0;
	read_lock(&pp->rwlock);
	txq_ctrl = &pp->txq_ctrl[pp->txp * CONFIG_MV_ETH_TXQ + txq];
	spin_lock(&txq_ctrl->queue_lock);

	if (txq_ctrl->txq_count >= mv_ctrl_txdone)
		mv_eth_txq_done(pp, txq_ctrl);
	/* Get next descriptor for tx, single buffer, so FIRST & LAST */
	tx_desc = mv_eth_tx_desc_get(txq_ctrl, 1);
	if (tx_desc == NULL) {
		spin_unlock(&txq_ctrl->queue_lock);
		read_unlock(&pp->rwlock);
		/* No resources: Drop */
		pp->dev->stats.tx_dropped++;
		if (withXor)
			xorReady();
		return MV_DROPPED;
	}
	txq_ctrl->txq_count++;

	tx_cmd |= NETA_TX_BM_ENABLE_MASK | NETA_TX_BM_POOL_ID_MASK(pkt->pool);
	txq_ctrl->shadow_txq[txq_ctrl->shadow_txq_put_i] = (u32) NULL;
	mv_eth_shadow_inc_put(txq_ctrl);

	tx_desc->command = tx_cmd | NETA_TX_L4_CSUM_NOT |
		NETA_TX_FLZ_DESC_MASK | NETA_TX_F_DESC_MASK
		| NETA_TX_L_DESC_MASK |
		NETA_TX_PKT_OFFSET_MASK(pkt->offset + MV_ETH_MH_SIZE);

	tx_desc->dataSize    = pkt->bytes;
	tx_desc->bufPhysAddr = pkt->physAddr;

	mv_eth_tx_desc_flush(tx_desc);

	if (withXor) {
		if (!xorReady()) {
			mvOsPrintf("MV_DROPPED in %s\n", __func__);
			return MV_DROPPED;
		}
	}
	mvNetaTxqPendDescAdd(pp->port, pp->txp, 0, 1);

	spin_unlock(&txq_ctrl->queue_lock);
	read_unlock(&pp->rwlock);

	return MV_OK;
}

static inline MV_STATUS mv_eth_cesa_l2fw_tx(struct eth_pbuf *pkt, struct eth_port *pp)
{
	struct neta_tx_desc *tx_desc;
	u32 tx_cmd = 0;
	struct tx_queue *txq_ctrl;

	/* assigning different txq for each rx port , to avoid waiting on the
	same txq lock when traffic on several rx ports are destined to the same
	outgoing interface */
	int txq = 0;
	txq_ctrl = &pp->txq_ctrl[pp->txp * CONFIG_MV_ETH_TXQ + txq];
	spin_lock(&txq_ctrl->queue_lock);

	if (txq_ctrl->txq_count >= mv_ctrl_txdone)
		mv_eth_txq_done(pp, txq_ctrl);
	/* Get next descriptor for tx, single buffer, so FIRST & LAST */
	tx_desc = mv_eth_tx_desc_get(txq_ctrl, 1);
	if (tx_desc == NULL) {
		/* printk("tx_desc == NULL pp->port=%d in %s\n", pp->port, ,__func__); */
		spin_unlock(&txq_ctrl->queue_lock);
		/* No resources: Drop */
		pp->dev->stats.tx_dropped++;
		return MV_DROPPED;
	}
	txq_ctrl->txq_count++;

	tx_cmd |= NETA_TX_BM_ENABLE_MASK | NETA_TX_BM_POOL_ID_MASK(pkt->pool);
	txq_ctrl->shadow_txq[txq_ctrl->shadow_txq_put_i] = (u32) NULL;
	mv_eth_shadow_inc_put(txq_ctrl);

	tx_desc->command = tx_cmd | NETA_TX_L4_CSUM_NOT |
		NETA_TX_FLZ_DESC_MASK | NETA_TX_F_DESC_MASK
		| NETA_TX_L_DESC_MASK |
		NETA_TX_PKT_OFFSET_MASK(pkt->offset + MV_ETH_MH_SIZE);

	tx_desc->dataSize    = pkt->bytes;
	tx_desc->bufPhysAddr = pkt->physAddr;
	mv_eth_tx_desc_flush(tx_desc);
	mvNetaTxqPendDescAdd(pp->port, pp->txp, 0, 1);
	spin_unlock(&txq_ctrl->queue_lock);
	return MV_OK;
}


inline int mv_eth_rx_l2f(struct eth_port *pp, int rx_todo, int rxq)
{
	struct eth_port  *new_pp;
	L2FW_RULE *l2fw_rule;
	MV_NETA_RXQ_CTRL *rx_ctrl = pp->rxq_ctrl[rxq].q;
	int rx_done, rx_filled;
	struct neta_rx_desc *rx_desc;
	u32 rx_status = MV_OK;
	struct eth_pbuf *pkt;
	struct eth_pbuf *newpkt = NULL;
	struct bm_pool *pool;
	MV_STATUS status = MV_OK;
	struct eth_port_l2fw *ppl2fw = mv_eth_ports_l2fw[pp->port];
	MV_IP_HEADER *pIph = NULL;
	MV_U8 *pData;
	int	ipOffset;
	MV_U8 chan = 0;
	rx_done = mvNetaRxqBusyDescNumGet(pp->port, rxq);
	mvOsCacheIoSync();
	if (rx_todo > rx_done)
		rx_todo = rx_done;

	rx_done = 0;
	rx_filled = 0;

	/* Fairness NAPI loop */
	while (rx_done < rx_todo) {
#ifdef CONFIG_MV_ETH_RX_DESC_PREFETCH
		rx_desc = mv_eth_rx_prefetch(pp, rx_ctrl, rx_done, rx_todo);
		if (!rx_desc)
			printk(KERN_INFO "rx_desc is NULL in %s\n", __func__);
#else
		rx_desc = mvNetaRxqNextDescGet(rx_ctrl);
		mvOsCacheLineInv(NULL, rx_desc);
		prefetch(rx_desc);
#endif /* CONFIG_MV_ETH_RX_DESC_PREFETCH */

		rx_done++;
		rx_filled++;

		pkt = (struct eth_pbuf *)rx_desc->bufCookie;
		if (!pkt) {
			printk(KERN_INFO "pkt is NULL in ; rx_done=%d %s\n", rx_done, __func__);
			return rx_done;
		}

		pool = &mv_eth_pool[pkt->pool];
		rx_status = rx_desc->status;
		if (((rx_status & NETA_RX_FL_DESC_MASK) != NETA_RX_FL_DESC_MASK) ||
			(rx_status & NETA_RX_ES_MASK)) {
			STAT_ERR(pp->stats.rx_error++);

			if (pp->dev)
				pp->dev->stats.rx_errors++;

			mv_eth_rxq_refill(pp, rxq, pkt, pool, rx_desc);
			continue;
		}

		pkt->bytes = rx_desc->dataSize - (MV_ETH_CRC_SIZE + MV_ETH_MH_SIZE);

		pData = pkt->pBuf + pkt->offset;
		if ((rx_desc->status & ETH_RX_VLAN_TAGGED_FRAME_MASK))
			ipOffset = MV_ETH_MH_SIZE + sizeof(MV_802_3_HEADER) + MV_VLAN_HLEN;
		else
			ipOffset = MV_ETH_MH_SIZE + sizeof(MV_802_3_HEADER);
		pIph = (MV_IP_HEADER *)(pData + ipOffset);
		if (pIph == NULL) {
			printk(KERN_INFO "pIph==NULL in %s\n", __func__);
			continue;
		}
#ifdef CONFIG_MV_ETH_L2FW_DEBUG
		if (pIph) {
			printk(KERN_INFO "%u.%u.%u.%u->%u.%u.%u.%u in %s\n", NIPQUAD(pIph->srcIP), NIPQUAD(pIph->dstIP), __func__);
			printk(KERN_INFO "ipOffset=%d in %s\n", ipOffset, __func__);
		} else
			printk(KERN_INFO "pIph is NULL in %s\n", __func__);
#endif
		if (espEnabled)
			new_pp  = mv_eth_ports[ppl2fw->txPort];
		else {
			 l2fw_rule = l2fw_lookup(pIph->srcIP, pIph->dstIP);

			 if (!l2fw_rule) {

#ifdef CONFIG_MV_ETH_L2FW_DEBUG
				printk(KERN_INFO "l2fw_lookup() failed in %s\n", __func__);
#endif
				mv_eth_rxq_refill(pp, rxq, pkt, pool, rx_desc);
				continue;
			 }

#ifdef CONFIG_MV_ETH_L2FW_DEBUG
				printk(KERN_INFO "l2fw_lookup() is ok l2fw_rule->port=%d in %s\n", l2fw_rule->port, __func__);
#endif
			new_pp  = mv_eth_ports[l2fw_rule->port];
			}

		switch (ppl2fw->cmd) {
		case TX_AS_IS:
					if (espEnabled) {
						int cpu = smp_processor_id();
						switch (l2fwMode) 
						{
						case L2FW_2CORES:
							/* for working with 2 cpus - CPU#0 and CPU#1
							 on 4 ports */
							if (cpu == 0)
								chan = 0;
							else {
								if (cpu == 1)
									chan = 1;
								}
							break;

						/* for working with 4 cpus - CPU#0,..., CPU#4
							 on 4 ports */
							
						case L4FW_4CORES:
							if (cpu<2)
								chan = 0;
							else {
								chan = 1;
								}
							break;
						case L2FW_1CORE:
							chan=pp->port;
							break;
						}
							
						spin_lock(&cesa_lock[chan]);
						status = handleEsp(pkt, rx_desc, new_pp, pp->port, chan);
						spin_unlock(&cesa_lock[chan]);
					}
				else
					status = mv_eth_l2fw_tx(pkt, new_pp, 0, rx_desc);
				break;

		case SWAP_MAC:
				mvOsCacheLineInv(NULL, pkt->pBuf + pkt->offset);
				l2fw_swap_mac(pkt);
				mvOsCacheLineFlush(NULL, pkt->pBuf+pkt->offset);
				status = mv_eth_l2fw_tx(pkt, new_pp, 0, rx_desc);
				break;

		case COPY_AND_SWAP:
				if (pkt->bytes >= l2fw_xor_threshold) {
					newpkt = eth_l2fw_copy_packet_withXor(pkt);
					if (newpkt)
						status = mv_eth_l2fw_tx(newpkt, new_pp, 1, rx_desc);
					else
						status = MV_ERROR;
				} else {
						newpkt = eth_l2fw_copy_packet_withoutXor(pkt);
						if (newpkt)
							status = mv_eth_l2fw_tx(newpkt, new_pp, 0, rx_desc);
						else
							status = MV_ERROR;
				}
		}
		if (status == MV_OK) {
			mvOsCacheLineInv(NULL, rx_desc);
			/* we do not need the pkt , we do not do anything with it*/
			if  ((ppl2fw->cmd	== COPY_AND_SWAP) && !(espEnabled))
				mv_eth_pool_put(pool, pkt);
			continue;
		} else if (status == MV_DROPPED) {
			mv_eth_rxq_refill(pp, rxq, pkt, pool, rx_desc);
			if ((ppl2fw->cmd	== COPY_AND_SWAP) && !(espEnabled))
				mv_eth_pool_put(pool, newpkt);

			continue;
		} else if (status == MV_ERROR) {
			printk(KERN_INFO "MV_ERROR in %s\n", __func__);
			mv_eth_rxq_refill(pp, rxq, pkt, pool, rx_desc);
		}

	} /* of while */
	/* Update RxQ management counters */
	mvOsCacheIoSync();

	mvNetaRxqDescNumUpdate(pp->port, rxq, rx_done, rx_filled);

	return rx_done;
}

static irqreturn_t nfp_sec_interrupt_handler_0(int irq, void *arg)
{
	MV_CESA_RESULT  	result;
	MV_STATUS           status;
	MV_U8 chan = 0;
    MV_REG_WRITE(MV_CESA_ISR_CAUSE_REG(chan), 0);

	while (1) {
	/* Get Ready requests */

	status = mvCesaReadyGet(chan, &result);
	if (status != MV_OK)
		break;
	nfp_sec_complete_out((unsigned long)((MV_NFP_SEC_CESA_PRIV_L2FW *)result.pReqPrv));
	}
	return IRQ_HANDLED;
}

static irqreturn_t nfp_sec_interrupt_handler_1(int irq, void *arg)
{
	MV_CESA_RESULT  	result;
	MV_STATUS           status;
	MV_U8 chan = 1;
    MV_REG_WRITE(MV_CESA_ISR_CAUSE_REG(chan), 0);
	while (1) {
	/* Get Ready requests */
	status = mvCesaReadyGet(chan, &result);
	if (status != MV_OK)
		break;
	nfp_sec_complete_out((unsigned long)((MV_NFP_SEC_CESA_PRIV_L2FW *)result.pReqPrv));
	}

	return IRQ_HANDLED;
}



MV_STATUS my_mvSysCesaInit(int numOfSession, int queueDepth, void *osHandle)
{
	MV_CESA_HAL_DATA halData;
	MV_UNIT_WIN_INFO addrWinMap[MAX_TARGETS + 1];
	MV_STATUS status;
	MV_U8 chan;

	status = mvCtrlAddrWinMapBuild(addrWinMap, MAX_TARGETS + 1);

	if (status == MV_OK) {
		for (chan = 0; chan < MV_CESA_CHANNELS; chan++) {
			status = mvCesaTdmaWinInit(chan, addrWinMap);
			if (status != MV_OK) {
				mvOsPrintf("Error, unable to initialize CESA windows for channel(%d)\n", chan);
				break;
			}
			halData.sramPhysBase[chan] = (MV_ULONG)mv_crypto_virt_base_get(chan);
			halData.sramVirtBase[chan] = (MV_U8 *)mv_crypto_virt_base_get(chan);
			halData.sramOffset[chan] = 0;
		}

		if (status == MV_OK) {
		halData.ctrlModel = mvCtrlModelGet();
		halData.ctrlRev = mvCtrlRevGet();
			status = mvCesaHalInit(numOfSession, queueDepth,
					osHandle, &halData);
	}
	}

	return status;
}

void cesaStart(void)
{
	int bufNum, bufSize;
	int i, j, idx;
	MV_CESA_MBUF *pMbufSrc_0, *pMbufDst_0;
	MV_BUF_INFO *pFragsSrc_0, *pFragsDst_0;
	char *pBuf_0;

	MV_CESA_MBUF *pMbufSrc_1, *pMbufDst_1;
	MV_BUF_INFO *pFragsSrc_1, *pFragsDst_1;
	char *pBuf_1;

	printk(KERN_INFO "in %s\n", __func__);

	cesaCmdArray_0 = 	mvOsMalloc(sizeof(MV_CESA_COMMAND) * CESA_DEF_REQ_SIZE);

	if (cesaCmdArray_0 == NULL) {
		mvOsPrintf("Can't allocate %d bytes of memory\n",
			   (int)(sizeof(MV_CESA_COMMAND) * CESA_DEF_REQ_SIZE));
		return;
	}
	memset(cesaCmdArray_0, 0, sizeof(MV_CESA_COMMAND) * CESA_DEF_REQ_SIZE);
	/* CESA_DEF_BUF_NUM */
	bufNum    =  1;
	/* CESA_DEF_BUF_SIZE */
	bufSize   = 1500;

	pMbufSrc_0  = mvOsMalloc(sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);
	pFragsSrc_0 = mvOsMalloc(sizeof(MV_BUF_INFO) * bufNum * CESA_DEF_REQ_SIZE);

	pMbufDst_0  = mvOsMalloc(sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);
	pFragsDst_0 = mvOsMalloc(sizeof(MV_BUF_INFO) * bufNum * CESA_DEF_REQ_SIZE);

	if ((pMbufSrc_0 == NULL) || (pFragsSrc_0 == NULL) ||
		(pMbufDst_0 == NULL) || (pFragsDst_0 == NULL)) {
		mvOsPrintf(" Can't malloc Src and Dst pMbuf and pFrags structures.\n");
		return;
	}

	memset(pMbufSrc_0,  0, sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);
	memset(pFragsSrc_0, 0, sizeof(MV_BUF_INFO) * bufNum * CESA_DEF_REQ_SIZE);

	memset(pMbufDst_0,  0, sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);
	memset(pFragsDst_0, 0, sizeof(MV_BUF_INFO) * bufNum * CESA_DEF_REQ_SIZE);

	idx = 0;
	for (i = 0; i < CESA_DEF_REQ_SIZE; i++) {
		pBuf_0 = mvOsIoCachedMalloc(cesaOSHandle, bufSize * bufNum * 2,
					  &cesaBufs_0[i].bufPhysAddr, &cesaBufs_0[i].memHandle);
		if (pBuf_0 == NULL) {
			mvOsPrintf("testStart: Can't malloc %d bytes for pBuf\n", bufSize * bufNum * 2);
			return;
		}

		memset(pBuf_0, 0, bufSize * bufNum * 2);
		mvOsCacheFlush(cesaOSHandle, pBuf_0, bufSize * bufNum * 2);
		if (pBuf_0 == NULL) {
			mvOsPrintf("Can't allocate %d bytes for req_%d buffers\n",
				   bufSize * bufNum * 2, i);
			return;
		}

		cesaBufs_0[i].bufVirtPtr = (MV_U8 *) pBuf_0;
		cesaBufs_0[i].bufSize = bufSize * bufNum * 2;

		cesaCmdArray_0[i].pSrc = &pMbufSrc_0[i];
		cesaCmdArray_0[i].pSrc->pFrags = &pFragsSrc_0[idx];
		cesaCmdArray_0[i].pSrc->numFrags = bufNum;
		cesaCmdArray_0[i].pSrc->mbufSize = 0;

		cesaCmdArray_0[i].pDst = &pMbufDst_0[i];
		cesaCmdArray_0[i].pDst->pFrags = &pFragsDst_0[idx];
		cesaCmdArray_0[i].pDst->numFrags = bufNum;
		cesaCmdArray_0[i].pDst->mbufSize = 0;

		for (j = 0; j < bufNum; j++) {
			cesaCmdArray_0[i].pSrc->pFrags[j].bufVirtPtr = (MV_U8 *) pBuf_0;
			cesaCmdArray_0[i].pSrc->pFrags[j].bufSize = bufSize;
			pBuf_0 += bufSize;
			cesaCmdArray_0[i].pDst->pFrags[j].bufVirtPtr = (MV_U8 *) pBuf_0;

			cesaCmdArray_0[i].pDst->pFrags[j].bufSize = bufSize;
			pBuf_0 += bufSize;
		}
		idx += bufNum;
	}

	cesaMbufArray_0 = mvOsMalloc(sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);
	if (cesaMbufArray_0 == NULL) {
		mvOsPrintf("Can't allocate %d bytes of memory\n",
			   (int)(sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE));
		return;
	}
	memset(cesaMbufArray_0, 0, sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);

	cesaPrivArray_0 = mvOsMalloc(sizeof(MV_NFP_SEC_CESA_PRIV_L2FW) * (CESA_DEF_REQ_SIZE + MV_NFP_SEC_REQ_Q_SIZE));
	memset(cesaPrivArray_0, 0, sizeof(MV_NFP_SEC_CESA_PRIV_L2FW) * (CESA_DEF_REQ_SIZE + MV_NFP_SEC_REQ_Q_SIZE));

	/* second engine */
	cesaCmdArray_1 = 	mvOsMalloc(sizeof(MV_CESA_COMMAND) * CESA_DEF_REQ_SIZE);

	if (cesaCmdArray_1 == NULL) {
		mvOsPrintf("Can't allocate %d bytes of memory\n",
			   (int)(sizeof(MV_CESA_COMMAND) * CESA_DEF_REQ_SIZE));
		return;
	}
	memset(cesaCmdArray_1, 0, sizeof(MV_CESA_COMMAND) * CESA_DEF_REQ_SIZE);

	/* CESA_DEF_BUF_NUM */
	bufNum    =  1;
	/* CESA_DEF_BUF_SIZE */
	bufSize   = 1500;

	pMbufSrc_1  = mvOsMalloc(sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);
	pFragsSrc_1 = mvOsMalloc(sizeof(MV_BUF_INFO) * bufNum * CESA_DEF_REQ_SIZE);

	pMbufDst_1  = mvOsMalloc(sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);
	pFragsDst_1 = mvOsMalloc(sizeof(MV_BUF_INFO) * bufNum * CESA_DEF_REQ_SIZE);

	if ((pMbufSrc_1 == NULL) || (pFragsSrc_1 == NULL) || (pMbufDst_1 == NULL)
		|| (pFragsDst_1 == NULL)) {
		mvOsPrintf(" Can't malloc Src and Dst pMbuf and pFrags structures.\n");
		return;
	}

	memset(pMbufSrc_1,  0, sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);
	memset(pFragsSrc_1, 0, sizeof(MV_BUF_INFO) * bufNum * CESA_DEF_REQ_SIZE);

	memset(pMbufDst_1,  0, sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);
	memset(pFragsDst_1, 0, sizeof(MV_BUF_INFO) * bufNum * CESA_DEF_REQ_SIZE);

	idx = 0;
	for (i = 0; i < CESA_DEF_REQ_SIZE; i++) {
		pBuf_1 = mvOsIoCachedMalloc(cesaOSHandle, bufSize * bufNum * 2,
					  &cesaBufs_1[i].bufPhysAddr, &cesaBufs_1[i].memHandle);
		if (pBuf_1 == NULL) {
			mvOsPrintf("testStart: Can't malloc %d bytes for pBuf\n", bufSize * bufNum * 2);
			return;
		}

		memset(pBuf_1, 0, bufSize * bufNum * 2);
		mvOsCacheFlush(cesaOSHandle, pBuf_1, bufSize * bufNum * 2);
		if (pBuf_1 == NULL) {
			mvOsPrintf("Can't allocate %d bytes for req_%d buffers\n",
				   bufSize * bufNum * 2, i);
			return;
		}

		cesaBufs_1[i].bufVirtPtr = (MV_U8 *) pBuf_1;
		cesaBufs_1[i].bufSize = bufSize * bufNum * 2;

		cesaCmdArray_1[i].pSrc = &pMbufSrc_1[i];
		cesaCmdArray_1[i].pSrc->pFrags = &pFragsSrc_1[idx];
		cesaCmdArray_1[i].pSrc->numFrags = bufNum;
		cesaCmdArray_1[i].pSrc->mbufSize = 0;

		cesaCmdArray_1[i].pDst = &pMbufDst_1[i];
		cesaCmdArray_1[i].pDst->pFrags = &pFragsDst_1[idx];
		cesaCmdArray_1[i].pDst->numFrags = bufNum;
		cesaCmdArray_1[i].pDst->mbufSize = 0;

		for (j = 0; j < bufNum; j++) {
			cesaCmdArray_1[i].pSrc->pFrags[j].bufVirtPtr = (MV_U8 *) pBuf_1;
			cesaCmdArray_1[i].pSrc->pFrags[j].bufSize = bufSize;
			pBuf_1 += bufSize;
			cesaCmdArray_1[i].pDst->pFrags[j].bufVirtPtr = (MV_U8 *) pBuf_1;

			cesaCmdArray_1[i].pDst->pFrags[j].bufSize = bufSize;
			pBuf_1 += bufSize;
		}
		idx += bufNum;
	}

	cesaMbufArray_1 = mvOsMalloc(sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);
	if (cesaMbufArray_1 == NULL) {
		mvOsPrintf("Can't allocate %d bytes of memory\n",
			   (int)(sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE));
		return;
	}
	memset(cesaMbufArray_1, 0, sizeof(MV_CESA_MBUF) * CESA_DEF_REQ_SIZE);

	cesaPrivArray_1 = mvOsMalloc(sizeof(MV_NFP_SEC_CESA_PRIV_L2FW) * (CESA_DEF_REQ_SIZE + MV_NFP_SEC_REQ_Q_SIZE));
	memset(cesaPrivArray_1, 0, sizeof(MV_NFP_SEC_CESA_PRIV_L2FW) * (CESA_DEF_REQ_SIZE + MV_NFP_SEC_REQ_Q_SIZE));

	pPktInfoNewArray_0 = mvOsMalloc(sizeof(MV_PKT_INFO) * MV_NFP_SEC_REQ_Q_SIZE);

	if (!pPktInfoNewArray_0) {
		printk(KERN_INFO "mvOsMalloc() failed in %s\n", __func__);
		return;
	}

	pBufInfoArray_0 = mvOsMalloc(sizeof(MV_BUF_INFO) * MV_NFP_SEC_REQ_Q_SIZE);
	if (!pBufInfoArray_0) {
		printk(KERN_INFO "could not allocate MV_BUF_INFO in %s\n", __func__);
		return;
	}

	pPktInfoNewArray_1 = mvOsMalloc(sizeof(MV_PKT_INFO) * MV_NFP_SEC_REQ_Q_SIZE);

	if (!pPktInfoNewArray_1) {
		printk(KERN_INFO "mvOsMalloc() failed in %s\n", __func__);
		return;
	}
	pBufInfoArray_1 = mvOsMalloc(sizeof(MV_BUF_INFO) * MV_NFP_SEC_REQ_Q_SIZE);
	if (!pBufInfoArray_0) {
		printk(KERN_INFO "could not allocate MV_BUF_INFO in %s\n", __func__);
		return;
	}
	printk(KERN_INFO "start finished in %s\n", __func__);
}


static int cesa_init(void)
{
	u8 chan = 0;
	int i;
	const char *irq_str[] = {"cesa0", "cesa1"};
	printk(KERN_INFO "in %s\n", __func__);
	for (i = 0; i < 2; i++)
		spin_lock_init(&cesa_lock[i]);
	if (mvCtrlPwrClckGet(CESA_UNIT_ID, 0) == MV_FALSE)
		return 0;
	if (MV_OK != my_mvSysCesaInit(1, 256, NULL)) {
		printk(KERN_INFO "%s,%d: mvCesaInit Failed. \n", __FILE__, __LINE__);
		return EINVAL;
	}

	/* clear and unmask Int */
	MV_REG_WRITE(MV_CESA_ISR_CAUSE_REG(chan), 0);
	MV_REG_WRITE(MV_CESA_ISR_MASK_REG(chan), MV_CESA_CAUSE_ACC_DMA_MASK);
	if (request_irq(CESA_IRQ(0), nfp_sec_interrupt_handler_0,
							(IRQF_DISABLED) , irq_str[chan], NULL)) {
				printk(KERN_INFO "%s,%d: cannot assign irq %x\n", __FILE__, __LINE__, CESA_IRQ(chan));
		return EINVAL;
	}

	chan = 1;
	MV_REG_WRITE(MV_CESA_ISR_CAUSE_REG(chan), 0);
	MV_REG_WRITE(MV_CESA_ISR_MASK_REG(chan), MV_CESA_CAUSE_ACC_DMA_MASK);

	if (request_irq(CESA_IRQ(1), nfp_sec_interrupt_handler_1,
							(IRQF_DISABLED) , irq_str[chan], NULL)) {
				printk(KERN_INFO "%s,%d: cannot assign irq %x\n", __FILE__, __LINE__, CESA_IRQ(chan));
		return EINVAL;
		}

	atomic_set(&req_count[0], 0);
	atomic_set(&req_count[1], 0);
	mvOsPrintf("MV_CESA_TDMA_CTRL_REG address 0 %08x\n\n", MV_CESA_TDMA_CTRL_REG(0));
	mvOsPrintf("MV_CESA_TDMA_CTRL_REG address 1 %08x\n\n", MV_CESA_TDMA_CTRL_REG(1));
	mvOsPrintf("MV_CESA_TDMA_CTRL_REG(0)  %08x\n",
		MV_REG_READ(MV_CESA_TDMA_CTRL_REG(0)));
	mvOsPrintf("MV_CESA_TDMA_CTRL_REG(1)  %08x\n",
		MV_REG_READ(MV_CESA_TDMA_CTRL_REG(1)));

	memset(&sa, 0, sizeof(MV_NFP_SEC_SA_ENTRY));
	sa.digestSize = MV_CESA_SHA1_DIGEST_SIZE;
	sa.ivSize = MV_CESA_AES_BLOCK_SIZE;
	sa.spi = 3;

	sa.tunProt = MV_NFP_SEC_TUNNEL;
	sa.encap   = MV_NFP_SEC_ESP;
	sa.seqNum  = 4;
	sa.tunnelHdr.sIp = 0x6400A8C0;
	sa.tunnelHdr.dIp = 0x6401A8C0;
	sa.tunnelHdr.outIfIndex = 0;
	sa.lifeTime = 0;

	sa.secOp = MV_NFP_SEC_ENCRYPT;
	strcpy(sa.tunnelHdr.dstMac, "aabbccddeeff");
	strcpy(sa.tunnelHdr.srcMac, "abacadaeafaa");

	return 0;
}


#ifdef CONFIG_MV_ETH_L2FW
int __devinit mv_l2fw_init(void)
{
	int size, port;
	MV_U32 bytes;
	MV_U32 regVal;
	mv_eth_ports_l2fw_num = mvCtrlEthMaxPortGet();
	mvOsPrintf("in %s: mv_eth_ports_l2fw_num=%d\n", __func__, mv_eth_ports_l2fw_num);
	size = mv_eth_ports_l2fw_num * sizeof(struct eth_port_l2fw *);
	mv_eth_ports_l2fw = mvOsMalloc(size);
	if (!mv_eth_ports_l2fw)
		goto oom;
	memset(mv_eth_ports_l2fw, 0, size);
	for (port = 0; port < mv_eth_ports_l2fw_num; port++) {
		mv_eth_ports_l2fw[port] =
			mvOsMalloc(sizeof(struct eth_port_l2fw));
		if (!mv_eth_ports_l2fw[port])
			goto oom1;
		mv_eth_ports_l2fw[port]->cmd    = L2FW_DISABLE;
		mv_eth_ports_l2fw[port]->txPort = -1;
	}

	bytes = sizeof(L2FW_RULE *) * L2FW_HASH_SIZE;
	l2fw_jhash_iv = mvOsRand();

	l2fw_hash = (L2FW_RULE **)mvOsMalloc(bytes);
	if (l2fw_hash == NULL) {
		mvOsPrintf("l2fw hash: not enough memory\n");
		return MV_NO_RESOURCE;
	}

	mvOsMemset(l2fw_hash, 0, bytes);

	mvOsPrintf("L2FW hash init %d entries, %d bytes\n", L2FW_HASH_SIZE, bytes);
	regVal = 0;

	cesa_init();
	return 0;
oom:
	mvOsPrintf("%s: out of memory in L2FW initialization\n", __func__);
oom1:
	mvOsFree(mv_eth_ports_l2fw);
	return -ENOMEM;

}
#endif

module_init(mv_l2fw_init);

MODULE_AUTHOR("Rami Rosen");
MODULE_DESCRIPTION("l2fw module");
MODULE_LICENSE("GPL");

