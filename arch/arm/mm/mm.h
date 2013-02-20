#ifdef CONFIG_MMU

/* the upper-most page table pointer */
extern pmd_t *top_pmd;

#ifdef CONFIG_MV_SUPPORT_64KB_PAGE_SIZE
#define TOP_PTE(x)	pte_offset_kernel(pmd_off_k(x), x)
#else
#define TOP_PTE(x)	pte_offset_kernel(top_pmd, x)
#endif

static inline pmd_t *pmd_off(pgd_t *pgd, unsigned long virt)
{
	return pmd_offset(pgd, virt);
}

static inline pmd_t *pmd_off_k(unsigned long virt)
{
	return pmd_off(pgd_offset_k(virt), virt);
}

struct mem_type {
	unsigned int prot_pte;
	unsigned int prot_l1;
	unsigned int prot_sect;
	unsigned int domain;
};

const struct mem_type *get_mem_type(unsigned int type);

extern void __flush_dcache_page(struct address_space *mapping, struct page *page);

#endif

struct pglist_data;

void __init bootmem_init(void);
void reserve_node_zero(struct pglist_data *pgdat);