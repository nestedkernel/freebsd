/*===-- LICENSE ------------------------------------------------------------===
 *
 * University of Illinois/NCSA Open Source License
 *
 * Copyright (C) 2014, The Board of Trustees of the University of Illinois.
 * All rights reserved.
 *
 * Developed by:
 *
 *    Research Group of Professor Vikram Adve in the Department of Computer
 *    Science The University of Illinois at Urbana-Champaign
 *    http://web.engr.illinois.edu/~vadve/Home.html
 *
 * Copyright (c) 2014, Nathan Dautenhahn, John Criswell, Will Dietz, Theodoros
 * Kasampalis
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the Software), to deal
 * with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimers.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimers in the documentation
 * and/or other materials provided with the distribution.  Neither the names of
 * Sam King or the University of Illinois, nor the names of its contributors
 * may be used to endorse or promote products derived from this Software
 * without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 *
 *===-----------------------------------------------------------------------===
 *
 *       Filename:  pmmu.c
 *
 *    Description:  File that holds the low level implmentation of writes to
 *                  the physical mmu.
 *
 *===-----------------------------------------------------------------------===
 */

#include "common.h"
#include "debug.h"
#include "cpufunc.h"
#include "stack.h"
#include "pmmu.h"
#include "wr_prot.h"

#include <sys/libkern.h>
#include <sys/systm.h>

/* These are to obtain the macros and defs for the pmap init system */
//#include <machine/pmap.h>
//#include <machine/param.h>

/* Define whether or not the mmu_init code assumes virtual addresses */
#define	USE_VIRT	0

/*----------------------------------------------------------------------------*
 *===-- pMMU Forward Declarattions ----------------------------------------===*
 *----------------------------------------------------------------------------*/
void	declare_ptp_and_walk_pt_entries(page_entry_t *pageEntry,
					unsigned long numPgEntries,
					enum page_type_t pageLevel);

/*----------------------------------------------------------------------------*
 *===-- Local Data Definitions --------------------------------------------===*
 *----------------------------------------------------------------------------*/
/* Size of the physical memory and page size in bytes */
#define	MEMSIZE			0x0000000800000000u
#define	PAGESIZE		4096
#define	NUMPGDESCENTRIES	(MEMSIZE / PAGESIZE)

#if REVIEW_OBSOLETE
/* Start and end addresses of the secure memory */
#define	SECMEMSTART		0xffffff0000000000u
#define	SECMEMEND		0xffffff8000000000u

/*
 * Offset into the PML4E at which the mapping for the secure memory region can
 * be found.
 */
static const uintptr_t secmemOffset = ((SECMEMSTART >> 39) << 3) & vmask;
#endif

/*
 * ===========================================================================
 * BEGIN FreeBSD CODE BLOCK
 *
 * $FreeBSD: release/9.0.0/sys/amd64/include/pmap.h 222813 2011-06-07 08:46:13Z attilio $
 * ===========================================================================
 */

/* MMU Flags ---- Intel Nomenclature ---- */
#define PG_V        0x001   /* P    Valid               */
#define PG_RW       0x002   /* R/W  Read/Write          */
#define PG_U        0x004   /* U/S  User/Supervisor     */
#define PG_NC_PWT   0x008   /* PWT  Write through       */
#define PG_NC_PCD   0x010   /* PCD  Cache disable       */
#define PG_A        0x020   /* A    Accessed            */
#define PG_M        0x040   /* D    Dirty               */
#define PG_PS       0x080   /* PS   Page size (0=4k,1=2M)   */
#define PG_PTE_PAT  0x080   /* PAT  PAT index           */
#define PG_G        0x100   /* G    Global              */
#define PG_AVAIL1   0x200   /*    / Available for system    */
#define PG_AVAIL2   0x400   /*   <  programmers use     */
#define PG_AVAIL3   0x800   /*    \                     */
#define PG_PDE_PAT  0x1000  /* PAT  PAT index           */
#define PG_NX       (1ul<<63) /* No-execute             */

/* Various interpretations of the above */
#define PG_W        PG_AVAIL1   /* "Wired" pseudoflag */
#define PG_MANAGED  PG_AVAIL2
#define PG_FRAME    (0x000ffffffffff000ul)
#define PG_PS_FRAME (0x000fffffffe00000ul)
#define PG_PROT     (PG_RW|PG_U)    /* all protection bits . */
#define PG_N        (PG_NC_PWT|PG_NC_PCD)   /* Non-cacheable */

/* Size of the level 1 page table units */
#define PAGE_SHIFT  12      /* LOG2(PAGE_SIZE) */
#define PAGE_SIZE   (1<<PAGE_SHIFT) /* bytes/page */
#define NPTEPG      (PAGE_SIZE/(sizeof (pte_t)))
#define NPTEPGSHIFT 9       /* LOG2(NPTEPG) */
#define PAGE_MASK   (PAGE_SIZE-1)
/* Size of the level 2 page directory units */
#define NPDEPG      (PAGE_SIZE/(sizeof (pde_t)))
#define NPDEPGSHIFT 9       /* LOG2(NPDEPG) */
#define PDRSHIFT    21              /* LOG2(NBPDR) */
#define NBPDR       (1<<PDRSHIFT)   /* bytes/page dir */
#define PDRMASK     (NBPDR-1)
/* Size of the level 3 page directory pointer table units */
#define NPDPEPG     (PAGE_SIZE/(sizeof (pdpte_t)))
#define NPDPEPGSHIFT    9       /* LOG2(NPDPEPG) */
#define PDPSHIFT    30      /* LOG2(NBPDP) */
#define NBPDP       (1<<PDPSHIFT)   /* bytes/page dir ptr table */
#define PDPMASK     (NBPDP-1)
/* Size of the level 4 page-map level-4 table units */
#define NPML4EPG    (PAGE_SIZE/(sizeof (pml4e_t)))
#define NPML4EPGSHIFT   9       /* LOG2(NPML4EPG) */
#define PML4SHIFT   39      /* LOG2(NBPML4) */
#define NBPML4      (1UL<<PML4SHIFT)/* bytes/page map lev4 table */
#define PML4MASK    (NBPML4-1)

/*
 * ===========================================================================
 * END FreeBSD CODE BLOCK
 * ===========================================================================
 */

/* Array describing the physical pages */
DECLARE_NK_DATA_ARR(static page_desc_t, page_desc, NUMPGDESCENTRIES);

/*
 * Description:
 *   This is a pointer to the PerspicuOS SuperSpace stack, which is used on
 *   calls to SuperSpace or SuperSpace calls.
 */
DECLARE_NK_DATA_ARR(char, SecureStack, 1<<12);
DECLARE_NK_DATA(uintptr_t, SecureStackBase) = (uintptr_t) SecureStack + sizeof(SecureStack);

/* Start and end addresses of user memory */
DECLARE_NK_DATA(static const uintptr_t, USERSTART) = 0x0000000000000000u;
DECLARE_NK_DATA(static const uintptr_t, USEREND) = 0x00007fffffffffffu;

/* Mask to get the proper number of bits from the virtual address */
DECLARE_NK_DATA(static const uintptr_t, vmask) = 0x0000000000000ff8u;

/* The number of references allowed per page table page */
DECLARE_NK_DATA(static const int, maxPTPVARefs) = 1;

/* The count must be at least this value to remove a mapping to a page */
DECLARE_NK_DATA(static const int, minRefCountToRemoveMapping) = 1;

/* Size of the smallest page frame in bytes */
DECLARE_NK_DATA(static const uintptr_t, X86_PAGE_SIZE) = 4096u;

/* Number of bits to shift to get the page number out of a PTE entry */
DECLARE_NK_DATA(static const unsigned, PAGESHIFT) = 12;

/* Zero mapping is the mapping that eliminates the previous entry */
DECLARE_NK_DATA(static const uintptr_t, ZERO_MAPPING) = 0;

/* Mask to get the address bits out of a PTE, PDE, etc. */
DECLARE_NK_DATA(static const uintptr_t, addrmask) = 0x000ffffffffff000u;


/*****************************************************************************
 * Define helper functions for MMU operations
 *****************************************************************************/

/*
 * Function: _mm_flush_tlb()
 *
 * Description:
 *  Flush all TLB's holding translations for the specified virtual address.
 *
 * Notes:
 *  I had to look at the FreeBSD implementation of invlpg() to figure out that
 *  you need to "dereference" the address to get the operand to the inline asm
 *  constraint to work properly.  While perhaps not necessary (because I don't
 *  think such a trivial thing can by copyrighted), the fact that I referenced
 *  the FreeBSD code is why we have the BSD copyright and attribute comment at
 *  the top of this file.
 */
static inline void
_mm_flush_tlb (void *address)
{
	__asm __volatile("invlpg %0" : : "m" (*((char *)address)) : "memory");
}

/*
 * Function: page_entry_store
 *
 * Description:
 *  This function takes a pointer to a page table entry and updates its value
 *  to the new value provided.
 *
 * Assumptions:
 *  - This function assumes that write protection is enabled in CR0 (WP bit set
 *    to 1).
 *
 * Inputs:
 *  *page_entry -: A pointer to the page entry to store the new value to, a
 *                 valid VA for accessing the page_entry.
 *  newVal      -: The new value to store, including the address of the
 *                 referenced page.
 *
 * Side Effect:
 *  - This function enables system wide write protection in CR0.
 */
static inline void
page_entry_store(unsigned long *page_entry, page_entry_t newVal)
{
	/* Write the new value to the page_entry */
	*page_entry = newVal;

	/*
	 * TODO: Add a check here to make sure the value matches
	 * the one passed in
	 */
}

/*----------------------------------------------------------------------------*
 *===-- pMMU Utility Functions --------------------------------------------===*
 *----------------------------------------------------------------------------*/

/*
 * Function: getVirtual()
 *
 * Description:
 *  This function takes a physical address and converts it into a virtual
 *  address that the SVA VM can access.
 *
 *  In a real system, this is done by having the SVA VM create its own
 *  virtual-to-physical mapping of all of physical memory within its own
 *  reserved portion of the virtual address space.  However, for now, we'll
 *  take advantage of FreeBSD's direct map of physical memory so that we don't
 *  have to set one up.
 */
static inline unsigned char *
getVirtual(uintptr_t physical)
{
	return (unsigned char *)(physical | 0xfffffe0000000000u);
}

/*
 * Description:
 *  This function takes a page table mapping and set's the flag to read only.
 *
 * Inputs:
 *  - mapping: the mapping to add read only flag to
 *
 * Return:
 *  - A new mapping set to read only
 *
 *  Note that setting the read only flag does not necessarily mean that the
 *  read only protection is enabled in the system. It just indicates that if
 *  the system has the write protection enabled then the value of this bit is
 *  considered.
 */
static inline page_entry_t
setMappingReadOnly(page_entry_t mapping)
{
	return (mapping & ~((uintptr_t)(PG_RW)));
}

/*
 * Description:
 *  This function takes a page table mapping and set's the flag to read/write.
 *
 * Inputs:
 *  - mapping: the mapping to which to add read/write permission
 *
 * Return:
 *  - A new mapping set with read/write permission
 */
static inline page_entry_t
setMappingReadWrite(page_entry_t mapping)
{
	return (mapping | PG_RW);
}

/*
 * Description:
 *  Given a page table entry value, return the page description associate with
 *  the frame being addressed in the mapping.
 *
 * Inputs:
 *  mapping: the mapping with the physical address of the referenced frame
 *
 * Return:
 *  Pointer to the page_desc for this frame
 */
static page_desc_t *
getPageDescPtr(unsigned long mapping)
{
	unsigned long frameIndex = (mapping & PG_FRAME) / PAGESIZE;
	if (frameIndex >= NUMPGDESCENTRIES)
		panic("Nested Kernel: getPageDescPtr: %lx %lx\n",
		      frameIndex, NUMPGDESCENTRIES);
	return page_desc + frameIndex;
}

 /*
 * Function: get_pagetable()
 *
 * Description:
 *  Return a physical address that can be used to access the current page table.
 */
static inline unsigned char *
get_pagetable(void)
{
	/* Value of the CR3 register */
	uintptr_t cr3;

	/* Get the page table value out of CR3 */
	__asm__ __volatile__ ("movq %%cr3, %0\n" : "=r" (cr3));

	/*
	 * Shift the value over 12 bits. The lower-order 12 bits of the page
	 * table pointer are assumed to be zero, and so they are reserved or
	 * used by the hardware.
	 */
	return (unsigned char *)((((uintptr_t)cr3) & 0x000ffffffffff000u));
}

/*===-- Functions for finding the virtual address of page table components ===*/
static inline pml4e_t *
get_pml4eVaddr(uintptr_t cr3, uintptr_t vaddr)
{
	/* Offset into the page table */
	uintptr_t offset = (vaddr >> (39 - 3)) & vmask;
	return (pml4e_t *) getVirtual(cr3 | offset);
}

static inline pdpte_t *
get_pdpteVaddr(pml4e_t *pml4e, uintptr_t vaddr)
{
	uintptr_t base   = (*pml4e) & 0x000ffffffffff000u;
	uintptr_t offset = (vaddr >> (30 - 3)) & vmask;
	return (pdpte_t *) getVirtual(base | offset);
}

static inline pde_t *
get_pdeVaddr(pdpte_t *pdpte, uintptr_t vaddr)
{
	uintptr_t base   = (*pdpte) & 0x000ffffffffff000u;
	uintptr_t offset = (vaddr >> (21 - 3)) & vmask;
	return (pde_t *) getVirtual(base | offset);
}

static inline pte_t *
get_pteVaddr(pde_t *pde, uintptr_t vaddr)
{
	uintptr_t base   = (*pde) & 0x000ffffffffff000u;
	uintptr_t offset = (vaddr >> (12 - 3)) & vmask;
	return (pte_t *) getVirtual(base | offset);
}

/*
 * Functions for returing the physical address of page table pages.
 */
static inline uintptr_t
get_pml4ePaddr (unsigned char *cr3, uintptr_t vaddr)
{
	/* Offset into the page table */
	uintptr_t offset = ((vaddr >> 39) << 3) & vmask;
	return (((uintptr_t)cr3) | offset);
}

static inline uintptr_t
get_pdptePaddr(pml4e_t *pml4e, uintptr_t vaddr)
{
	uintptr_t offset = ((vaddr  >> 30) << 3) & vmask;
	return ((*pml4e & 0x000ffffffffff000u) | offset);
}

static inline uintptr_t
get_pdePaddr(pdpte_t *pdpte, uintptr_t vaddr)
{
	uintptr_t offset = ((vaddr  >> 21) << 3) & vmask;
	return ((*pdpte & 0x000ffffffffff000u) | offset);
}

static inline uintptr_t
get_ptePaddr(pde_t *pde, uintptr_t vaddr)
{
	uintptr_t offset = ((vaddr >> 12) << 3) & vmask;
	return ((*pde & 0x000ffffffffff000u) | offset);
}

/*
 * Function: get_pgeVaddr
 *
 * Description:
 *  This function does page walk to find the entry controlling access to the
 *  specified address. The function takes into consideration the potential use
 *  of larger page sizes.
 *
 * Inputs:
 *  vaddr - Virtual Address to find entry for
 *
 * Return value:
 *  0 - There is no mapping for this virtual address.
 *  Otherwise, a pointer to the PTE that controls the mapping of this virtual
 *  address is returned.
 */
static inline page_entry_t *
get_pgeVaddr(uintptr_t vaddr)
{
	uintptr_t	 cr3;
	pml4e_t		*pml4e;
	pdpte_t		*pdpte;
	pde_t		*pde;

	/* Get the base of the pml4 to traverse */
	cr3 = get_pagetable();
	if ((cr3 & 0xfffffffffffff000u) == 0)
		return 0;

	/* Get the VA of the pml4e for this vaddr */
	pml4e = get_pml4eVaddr(cr3, vaddr);
	if (!(*pml4e & PG_V))
		return 0;

	/* Get the VA of the pdpte for this vaddr */
	pdpte = get_pdpteVaddr(pml4e, vaddr);
	if (!(*pdpte & PG_V))
		return 0;

	/*
	 * The PDPE can be configurd in large page mode. If it is then we have
	 * the entry corresponding to the given vaddr If not then we go deeper
	 * in the page walk.
	 */
	if (*pdpte & PG_PS)
		return pdpte;

	/* Get the pde associated with this vaddr */
	pde = get_pdeVaddr(pdpte, vaddr);
	if (!(*pde & PG_V))
		return 0;

	/*
	 * As is the case with the pdpte, if the pde is configured for large
	 * page size then we have the corresponding entry. Otherwise we need
	 * to traverse one more level, which is the last.
	 */
	if (*pde & PG_PS)
		return pde;

	return get_pteVaddr(pde, vaddr);
}

/*
 * Function: getPhysicalAddr()
 *
 * Description:
 *  Find the physical page number of the specified virtual address.
 */
static uintptr_t
getPhysicalAddr(void *v)
{
	/* Mask to get the proper number of bits from the virtual address */
#if NOT_PORTED_YET
	static const uintptr_t vmask = 0x0000000000000fffu;
#endif

	/* Virtual address to convert */
	uintptr_t vaddr = ((uintptr_t) v);

	/* Offset into the page table */
	uintptr_t offset = 0;

	/*
	 * Get the currently active page table.
	 */
	unsigned char *cr3 = get_pagetable();

	/*
	 * Get the address of the PML4e.
	 */
	pml4e_t *pml4e = get_pml4eVaddr(cr3, vaddr);

	/*
	 * Use the PML4E to get the address of the PDPTE.
	 */
	pdpte_t *pdpte = get_pdpteVaddr(pml4e, vaddr);

	/*
	 * Determine if the PDPTE has the PS flag set.  If so, then it's
	 * pointing to a 1 GB page; return the physical address of that page.
	 */
	if ((*pdpte) & PG_PS)
		return (*pdpte & 0x000fffffffffffffu) >> 30;

	/*
	 * Find the page directory entry table from the PDPTE value.
	 */
	pde_t *pde = get_pdeVaddr(pdpte, vaddr);

	/*
	 * Determine if the PDE has the PS flag set.  If so, then it's pointing
	 * to a 2 MB page; return the physical address of that page.
	 */
	if ((*pde) & PG_PS)
		return (*pde & 0x000fffffffe00000u) + (vaddr & 0x1fffffu);

	/*
	 * Find the PTE pointed to by this PDE.
	 */
	pte_t *pte = get_pteVaddr(pde, vaddr);

	/*
	 * Compute the physical address.
	 */
	offset = vaddr & vmask;
	uintptr_t paddr = (*pte & 0x000ffffffffff000u) + offset;
	return paddr;
}

/*
 * Function: declare_ptp_and_walk_pt_entries
 *
 * Descriptions:
 *  This function recursively walks a page table and it's entries to initalize
 *  the nested kernel data structures for the given page. This function is
 *  meant to initialize SVA data structures so they mirror the static page
 *  table setup by a kernel. However, it uses the paging structure itself to
 *  walk the pages, which means it should be agnostic to the operating system
 *  being employed upon. The function only walks into page table pages that are
 *  valid or enabled. It also makes sure that if a given page table is already
 *  active in NK then it skips over initializing its entries as that could
 *  cause an infinite loop of recursion. This is an issue in FreeBSD as they
 *  have a recursive mapping in the pml4 top level page table page.
 *
 *  If a given page entry is marked as having a larger page size, such as may
 *  be the case with a 2MB page size for PD entries, then it doesn't traverse
 *  the page. Therefore, if the kernel page tables are configured correctly
 *  this won't initialize any NK page descriptors that aren't in use.
 *
 *  The primary objective of this code is to for each valid page table page:
 *      [1] Initialize the page_desc for the given page
 *      [2] Set the page permissions as read only
 *
 * Assumptions:
 *  - The number of entries per page assumes a amd64 paging hardware mechanism.
 *    As such the number of entires per a 4KB page table page is 2^9 or 512
 *    entries.
 *  - This page referenced in pageMapping has already been determined to be
 *    valid and requires SVA metadata to be created.
 *
 * Inputs:
 *   pageMapping: Page mapping associated with the given page being traversed.
 *                This mapping identifies the physical address/frame of the
 *                page table page so that SVA can initialize it's data
 *                structures then recurse on each entry in the page table page.
 *  numPgEntries: The number of entries for a given level page table.
 *     pageLevel: The page level of the given mapping {1,2,3,4}.
 *
 *
 * TODO:
 *  - Modify the page entry number to be dynamic in some way to accomodate
 *    differing numbers of entries. This only impacts how we traverse the
 *    address structures. The key issue is that we don't want to traverse an
 *    entry that randomly has the valid bit set, but not have it point to a
 *    real page. For example, if the kernel did not zero out the entire page
 *    table page and only inserted a subset of entries in the page table, the
 *    non set entries could be identified as holding valid mappings, which
 *    would then cause this function to traverse down truly invalid page table
 *    pages. In FreeBSD this isn't an issue given the way they initialize the
 *    static mapping, but could be a problem given differnet intialization
 *    methods.
 *
 *  - Add code to mark direct map page table pages to prevent the OS from
 *    modifying them.
 *
 */
#define DEBUG_INIT 0
void
declare_ptp_and_walk_pt_entries(page_entry_t *pageEntry,
				unsigned long numPgEntries,
				enum page_type_t pageLevel)
{
    int i;
    int traversedPTEAlready;
    enum page_type_t subLevelPgType;
    unsigned long numSubLevelPgEntries;
    page_desc_t *thisPg;
    page_entry_t pageMapping;
    page_entry_t *pagePtr;

    /* Store the pte value for the page being traversed */
    pageMapping = *pageEntry;

    /* Set the page pointer for the given page */
#if USE_VIRT
    uintptr_t pagePhysAddr = pageMapping & PG_FRAME;
    pagePtr = (page_entry_t *) getVirtual(pagePhysAddr);
#else
    pagePtr = (page_entry_t *) (pageMapping & PG_FRAME);
#endif

    /* Get the page_desc for this page */
    thisPg = getPageDescPtr(pageMapping);

    /* Mark if we have seen this traversal already */
    traversedPTEAlready = (thisPg->type != PG_UNUSED);

#if DEBUG_INIT >= 1
    /* Character inputs to make the printing pretty for debugging */
    char *indent = "";
    char *l4s = "L4:";
    char *l3s = "\tL3:";
    char *l2s = "\t\tL2:";
    char *l1s = "\t\t\tL1:";

    switch (pageLevel) {
    case PG_L4:
        indent = l4s;
        printf("%sSetting L4 Page: mapping:0x%lx\n", indent, pageMapping);
        break;
    case PG_L3:
        indent = l3s;
        printf("%sSetting L3 Page: mapping:0x%lx\n", indent, pageMapping);
        break;
    case PG_L2:
        indent = l2s;
        printf("%sSetting L2 Page: mapping:0x%lx\n", indent, pageMapping);
        break;
    case PG_L1:
        indent = l1s;
        printf("%sSetting L1 Page: mapping:0x%lx\n", indent, pageMapping);
        break;
    default:
        break;
    }
#endif

    /*
     * For each level of page we do the following:
     *  - Set the page descriptor type for this page table page
     *  - Set the sub level page type and the number of entries for the
     *    recursive call to the function.
     */
    switch(pageLevel) {
    case PG_L4:
        thisPg->type = PG_L4;       /* Set the page type to L4 */
        thisPg->user = 0;           /* Set the priv flag to kernel */
        ++(thisPg->count);
        subLevelPgType = PG_L3;
        numSubLevelPgEntries = NPML4EPG;//    numPgEntries;
        break;

    case PG_L3:
        if (thisPg->type != PG_L4)
            thisPg->type = PG_L3;       /* Set the page type to L3 */
        thisPg->user = 0;           /* Set the priv flag to kernel */
        ++(thisPg->count);
        subLevelPgType = PG_L2;
        numSubLevelPgEntries = NPDPEPG; //numPgEntries;
        break;

    case PG_L2:
        /*
         * If the L2 page mapping signifies that this mapping references a
         * 1GB page frame, then get the frame address using the correct
         * page mask for a L3 page entry and initialize the page_desc for
         * this entry. Then return as we don't need to traverse frame
         * pages.
         */
        if ((pageMapping & PG_PS) != 0) {
#if DEBUG_INIT >= 1
            printf("\tIdentified 1GB page...\n");
#endif
            unsigned long index = (pageMapping & ~PDPMASK) / PAGESIZE;
            page_desc[index].type = PG_TKDATA;
            page_desc[index].user = 0;           /* Set the priv flag to kernel */
            ++(page_desc[index].count);
            return;
        } else {
            thisPg->type = PG_L2;       /* Set the page type to L2 */
            thisPg->user = 0;           /* Set the priv flag to kernel */
            ++(thisPg->count);
            subLevelPgType = PG_L1;
            numSubLevelPgEntries = NPDEPG; // numPgEntries;
        }
        break;

    case PG_L1:
        /*
         * If my L1 page mapping signifies that this mapping references a 2MB
         * page frame, then get the frame address using the correct page mask
         * for a L2 page entry and initialize the page_desc for this entry.
         * Then return as we don't need to traverse frame pages.
         */
        if ((pageMapping & PG_PS) != 0) {
#if DEBUG_INIT >= 1
            printf("\tIdentified 2MB page...\n");
#endif
            /* TODO: this should use thisPg... */
            /* The frame address referencing the page obtained */
            unsigned long index = (pageMapping & ~PDRMASK) / PAGESIZE;
            page_desc[index].type = PG_TKDATA;
            page_desc[index].user = 0;           /* Set the priv flag to kernel */
            ++(page_desc[index].count);
            return;
        } else {
            thisPg->type = PG_L1;       /* Set the page type to L1 */
            thisPg->user = 0;           /* Set the priv flag to kernel */
            ++(thisPg->count);
            subLevelPgType = PG_TKDATA;
            numSubLevelPgEntries = NPTEPG;//      numPgEntries;
        }
        break;

    default:
        printf("SVA: page type %d. Frame addr: %p\n",thisPg->type, pagePtr);
        panic("SVA: walked an entry with invalid page type.");
    }

    /*
     * There is one recursive mapping, which is the last entry in the PML4 page
     * table page. Thus we return before traversing the descriptor again.
     * Notice though that we keep the last assignment to the page as the page
     * type information.
     */
    if (traversedPTEAlready) {
#if DEBUG_INIT >= 1
        printf("%sRecursed on already initialized page_desc\n", indent);
#endif
        return;
    }

#if DEBUG_INIT >= 1
    u_long nNonValPgs=0;
    u_long nValPgs=0;
#endif
    /*
     * Iterate through all the entries of this page, recursively calling the
     * walk on all sub entries.
     */
    for (i = 0; i < numSubLevelPgEntries; i++) {
#if 0
        /*
         * Do not process any entries that implement the direct map.  This prevents
         * us from marking physical pages in the direct map as kernel data pages.
         */
        if ((pageLevel == PG_L4) && (i == (0xfffffe0000000000 / 0x1000))) {
            continue;
        }
#endif
        page_entry_t *nextEntry = & pagePtr[i];

#if DEBUG_INIT >= 5
        printf("%sPagePtr in loop: %p, val: 0x%lx\n", indent, nextEntry, *nextEntry);
#endif

        /*
         * If this entry is valid then recurse the page pointed to by this page
         * table entry.
         */
        if (*nextEntry & PG_V) {
#if DEBUG_INIT >= 1
            nValPgs++;
#endif

            /*
             * If we hit the level 1 pages we have hit our boundary condition for
             * the recursive page table traversals. Now we just mark the leaf page
             * descriptors.
             */
            if (pageLevel == PG_L1) {
#if DEBUG_INIT >= 2
                printf("%sInitializing leaf entry: pteaddr: %p, mapping: 0x%lx\n",
                        indent, nextEntry, *nextEntry);
#endif
            } else {
#if DEBUG_INIT >= 2
                printf("%sProcessing:pte addr: %p, newPgAddr: %p, mapping: 0x%lx\n",
                        indent, nextEntry, (*nextEntry & PG_FRAME), *nextEntry );
#endif
                declare_ptp_and_walk_pt_entries(nextEntry,
                        numSubLevelPgEntries, subLevelPgType);
            }
        }
#if DEBUG_INIT >= 1
        else {
            nNonValPgs++;
        }
#endif
    }

#if DEBUG_INIT >= 1
    SVA_ASSERT((nNonValPgs + nValPgs) == 512, "Wrong number of entries traversed");

    printf("%sThe number of || non valid pages: %lu || valid pages: %lu\n",
            indent, nNonValPgs, nValPgs);
#endif
}

/*
 * Function: declare_kernel_code_pages()
 *
 * Description:
 *  Mark all kernel code pages as code pages.
 *
 * Inputs:
 *  startVA    - The first virtual address of the memory region.
 *  endVA      - The last virtual address of the memory region.
 *  pgType     - The nested kernel page type
 */
void
init_protected_pages(uintptr_t startVA, uintptr_t endVA,
		     enum page_type_t pgType)
{
	/* Get pointers for the pages */
	uintptr_t page;
	uintptr_t startPA = getPhysicalAddr(startVA) & PG_FRAME;
	uintptr_t endPA   = getPhysicalAddr(endVA)   & PG_FRAME;

#if NOT_PORTED_YET
	PERSPDEBUG(dec_ker_cod_pgs, "\nDeclaring pages for range: %p -- %p\n",
		   startVA, endVA);

	/*
	 * Scan through each page in the text segment.  Note that it is a pgType
	 * page, and make the page read-only within the page table.
	 */
	for (page = startPA; page < endPA; page += PAGESIZE) {
		/* Mark the page as both a code page and kernel level */
		page_desc[page / PAGESIZE].type = PG_CODE;
		page_desc[page / PAGESIZE].user = 0;

		/* Configure the MMU so that the page is read-only */
		page_entry_t *page_entry = get_pgeVaddr(startVA + (page - startPA));
		page_entry_store(page_entry, setMappingReadOnly(*page_entry));
	}

	PERSPDEBUG(dec_ker_cod_pgs, "\nFinished decl pages for range: %p -- %p\n",
		   startVA, endVA);
#endif
}

/*
 * Function: pmmu_init()
 *
 * Description:
 *  This function initializes the nk vmmu unit by zeroing out the page
 *  descriptors, capturing the statically allocated initial kernel mmu state,
 *  and identifying all kernel code pages, and setting them in the page
 *  descriptor array.
 *
 *  To initialize the sva page descriptors, this function takes the pml4 base
 *  mapping and walks down each level of the page table tree.
 *
 *  NOTE: In this function we assume that the page mapping for the kpml4 has
 *  physical addresses in it. We then dereference by obtaining the virtual
 *  address mapping of this page. This works whether or not the processor is in
 *  a virtually addressed or physically addressed mode.
 *
 * Inputs:
 *  - kpml4Mapping : Mapping referencing the base kernel pml3 page table page
 *  - nkpml4e      : The number of entries in the pml4
 *  - firstpaddr   : A pointer to the physical address of the first free frame
 *  - btext        : The first virtual address of the text segment
 *  - etext        : The last virtual address of the text segment
 */
void
pmmu_init(pml4e_t *kpml4Mapping, unsigned long nkpml4e,
	  uintptr_t *firstpaddr, uintptr_t btext, uintptr_t etext)
{
	NKDEBUG(mmu_init, "Initializing MMU");

	/* Get the virtual address of the pml4e mapping */
#if USE_VIRT
	pml4e_t * kpml4eVA = (pml4e_t *)getVirtual((uintptr_t)kpml4Mapping);
#else
	pml4e_t * kpml4eVA = kpml4Mapping;
#endif

	/* Zero out the page descriptor array */
	memset (page_desc, 0, NUMPGDESCENTRIES * sizeof(page_desc_t));

	/* Walk the kernel page tables and initialize the sva page_desc */
	declare_ptp_and_walk_pt_entries(kpml4eVA, nkpml4e, PG_L4);

	/* Identify kernel code pages and intialize the descriptors */
	init_protected_pages(btext, etext, PG_CODE);

#if NOT_PORTED_YET
	/* Now load the initial value of the cr3 to complete kernel init */
	_load_cr3(*kpml4Mapping & PG_FRAME);

	/* Make existing page table pages read-only */
	makePTReadOnly();

	/* Make all SuperSpace pages read-only */
	//makeSuperSpaceRO();
	declare_kernel_code_pages(_pspacestart, _pspaceend);

	/* Note that the MMU is now initialized */
	mmuIsInitialized = 1;
#endif

	NKDEBUG(mmu_init, "Completed MMU init");
}

/*
 * Function: updateNewPageData
 *
 * Description:
 *  This function is called whenever we are inserting a new mapping into a page
 *  entry. The goal is to manage any SVA page data that needs to be set for
 *  tracking the new mapping with the existing page data. This is essential to
 *  enable the MMU verification checks.
 *
 * Inputs:
 *  mapping - The new mapping to be inserted in x86_64 page table format.
 */
static inline void
updateNewPageData(page_entry_t mapping)
{
	uintptr_t newPA = mapping & PG_FRAME;
	unsigned long newFrame = newPA >> PAGESHIFT;
	uintptr_t newVA = (uintptr_t) getVirtual(newPA);
	page_desc_t *newPG = getPageDescPtr(mapping);

	/*
	 * If the new mapping is valid, update the counts for it.
	 */
	if (mapping & PG_V) {
#if 0
		/*
		 * If the new page is to a page table page and this is the first
		 * reference to the page, we need to set the VA mapping this
		 * page so that the verification routine can enforce that this
		 * page is only mapped to a single VA. Note that if we have
		 * gotten here, we know that we currently do not have a mapping
		 * to this page already, which means this is the first mapping
		 * to the page.
		 */
		if (newPG->type >= PG_L1 && newPG->type <= PG_L4)
			newPG->pgVaddr = newVA;
#endif

		/*
		 * Update the reference count for the new page frame. Check
		 * that we aren't overflowing the counter.
		 */
		if (newPG->count >= ((1u << 13) - 1))
			panic("MMU: overflow for the mapping count");
		newPG->count++;

		/*
		 * Set the VA of this entry if it is the first mapping to a
		 * page table page.
		 */
	}
}

/*
 * Function: updateOrigPageData
 *
 * Description:
 *  This function updates the metadata for a page that is being removed from
 *  the mapping.
 *
 * Inputs:
 *  mapping - An x86_64 page table entry describing the old mapping of the page
 */
static inline void
updateOrigPageData(page_entry_t mapping)
{
	page_desc_t *origPG = getPageDescPtr(mapping);

	/*
	 * Only decrement the mapping count if the page has an existing valid
	 * mapping.  Ensure that we don't drop the reference count below zero.
	 */
	if ((mapping & PG_V) && (origPG->count)) {
		--(origPG->count);
	}
}

/*
 * Function: _do_mmu_update
 *
 * Description:
 *  If the update has been validated, this function manages metadata by
 *  updating the internal SVA reference counts for pages and then performs the
 *  actual update.
 *
 * Inputs:
 *  *page_entry  - VA pointer to the page entry being modified
 *  newVal       - Representes the mapping to insert into the page_entry
 */
static inline void
_do_mmu_update(pte_t *pteptr, page_entry_t mapping)
{
	uintptr_t origPA = *pteptr & PG_FRAME;
	uintptr_t newPA = mapping & PG_FRAME;

	/*
	 * If we have a new mapping as opposed to just changing the flags of an
	 * existing mapping, then update the SVA meta data for the pages. We
	 * know that we have passed the validation checks so these updates have
	 * been vetted.
	 */
	if (newPA != origPA) {
		updateOrigPageData(*pteptr);
		updateNewPageData(mapping);
	} else if ((*pteptr & PG_V) && ((mapping & PG_V) == 0)) {
		/*
		 * If the old mapping is marked valid but the new mapping is
		 * not, then decrement the reference count of the old page.
		 */
		updateOrigPageData(*pteptr);
	} else if (((*pteptr & PG_V) == 0) && (mapping & PG_V)) {
		/*
		 * Contrariwise, if the old mapping is invalid but the new
		 * mapping is valid, then increment the reference count of
		 * the new page.
		 */
		updateNewPageData(mapping);
	}

	/* Perform the actual write to into the page table entry */
	page_entry_store((page_entry_t *) pteptr, mapping);
	return;
}

/*
 * Function: initDeclaredPage
 *
 * Description:
 *  This function zeros out the physical page pointed to by frameAddr and
 *  changes the permissions of the page in the direct map to read-only.
 *  This function is agnostic as to which level page table entry we are
 *  modifying because the format of the entry is the same in all cases.
 *
 * Assumption: This function should only be called by a declare intrinsic.
 *      Otherwise it has side effects that may break the system.
 *
 * Inputs:
 *  frameAddr: represents the physical address of this frame
 *
 *  *page_entry: A pointer to a page table entry that will be used to
 *      initialize the mapping to this newly created page as read only. Note
 *      that the address of the page_entry must be a virtually accessible
 *      address.
 */
static inline void
initDeclaredPage(unsigned long frameAddr)
{
	/*
	 * Get the direct map virtual address of the physical address.
	 */
	unsigned char *vaddr = getVirtual(frameAddr);

	/*
	 * Get a pointer to the page table entry that maps the physical page
	 * into the direct map.
	 */
	page_entry_t *page_entry = get_pgeVaddr(vaddr);

	/*
	 * Initialize the contents of the page to zero.  This will ensure that
	 * no existing page translations which have not been vetted exist
	 * within the page.
	 */
	memset(vaddr, 0, PAGESIZE);

	if (page_entry && (*page_entry & PG_PS) == 0) {
		/*
		 * Make the direct map entry for the page read-only to ensure
		 * that the OS goes through SVA to make page table changes.
		 *
		 * Also be sure to flush the TLBs for the direct map address
		 * to ensure that it's made read-only right away.
		 */
		page_entry_store(page_entry, setMappingReadOnly(*page_entry));
		_mm_flush_tlb(vaddr);
	}
}

static void _nk_pmmu_declare_page(uintptr_t frameAddr, enum page_type_t type)
{
	/* Get the page_desc for the newly declared <type> page frame */
	page_desc_t *pgDesc = getPageDescPtr(frameAddr);

	/*
	 * Make sure that this is already a <type> page, an unused page, or a
	 * kernel data page.
	 */
	if (pgDesc->type != type &&
	    pgDesc->type != PG_UNUSED &&
	    pgDesc->type != PG_TKDATA) {
		printf("SVA: %lx %lx\n", page_desc, page_desc + NUMPGDESCENTRIES);
		panic("SVA: Declaring L%d for wrong page: frameAddr = %lx, pgDesc=%lx, type=%x\n",
		      type, frameAddr, pgDesc, pgDesc->type);
	}

	/*
	 * Declare the page as a <type> page (unless it is already
	 * a <type> page).
	 */
	if (pgDesc->type != type) {
		/* Mark this page frame as an L1 page frame.*/
		pgDesc->type = type;

		/* Reset the VA which can point to this page table page. */
		pgDesc->pgVaddr = 0;

		/*
		 * Initialize the page data and page entry. Note that we pass
		 * a general page_entry_t to the function as it enables reuse of
		 * code for each of the entry declaration functions.
		 */
		initDeclaredPage(frameAddr);
	}
}

/*
 * Intrinsic: nk_pmmu_declare_l1_page()
 *
 * Description:
 *  This intrinsic marks the specified physical frame as a Level 1 page table
 *  frame.  It will zero out the contents of the page frame so that stale
 *  mappings within the frame are not used by the MMU.
 *
 * Inputs:
 *  frameAddr - The address of the physical page frame that will be used as a
 *              Level 1 page frame.
 */
void nk_pmmu_declare_l1_page(uintptr_t frameAddr)
{
	_nk_pmmu_declare_page(frameAddr, PG_L1);
}

/*
 * Intrinsic: nk_pmmu_declare_l2_page()
 *
 * Description:
 *  This intrinsic marks the specified physical frame as a Level 2 page table
 *  frame.  It will zero out the contents of the page frame so that stale
 *  mappings within the frame are not used by the MMU.
 *
 * Inputs:
 *  frameAddr - The address of the physical page frame that will be used as a
 *              Level 2 page frame.
 */
void nk_pmmu_declare_l2_page(uintptr_t frameAddr)
{
	_nk_pmmu_declare_page(frameAddr, PG_L2);
}

/*
 * Intrinsic: nk_pmmu_declare_l3_page()
 *
 * Description:
 *  This intrinsic marks the specified physical frame as a Level 3 page table
 *  frame.  It will zero out the contents of the page frame so that stale
 *  mappings within the frame are not used by the MMU.
 *
 * Inputs:
 *  frameAddr - The address of the physical page frame that will be used as a
 *              Level 3 page frame.
 */
void nk_pmmu_declare_l3_page(uintptr_t frameAddr)
{
	_nk_pmmu_declare_page(frameAddr, PG_L3);
}

/*
 * Intrinsic: nk_pmmu_declare_l4_page()
 *
 * Description:
 *  This intrinsic marks the specified physical frame as a Level 4 page table
 *  frame.  It will zero out the contents of the page frame so that stale
 *  mappings within the frame are not used by the MMU.
 *
 * Inputs:
 *  frameAddr - The address of the physical page frame that will be used as a
 *              Level 4 page frame.
 */
void nk_pmmu_declare_l4_page(uintptr_t frameAddr)
{
	_nk_pmmu_declare_page(frameAddr, PG_L4);
}

/*
 * Function: nk_pmmu_remove_page()
 *
 * Description:
 *  This function informs the NK VM that the system software no longer wants
 *  to use the specified page as a page table page.
 *
 * Inputs:
 *  paddr - The physical address of the page table page.
 */
void nk_pmmu_remove_page(uintptr_t paddr)
{
	/* Get the entry controlling the permissions for this pte PTP */
	page_entry_t *pte = get_pgeVaddr(getVirtual(paddr));

	/* Get the page_desc for the l1 page frame */
	page_desc_t *pgDesc = getPageDescPtr(paddr);

	/*
	 * Make sure that this is a page table page.  We don't want the system
	 * software to trick us.
	 */
	switch (pgDesc->type)  {
	case PG_L1:
	case PG_L2:
	case PG_L3:
	case PG_L4:
		break;
	default:
		panic("NK: undeclare bad page type: %lx %lx\n",
		      paddr, pgDesc->type);
	}

	/*
	 * Check that there are no references to this page (i.e.: there is no
	 * page table entry that refers to this physical page frame). If there
	 * is a mapping, then someone is still using it as a page table page. In
	 * that case, ignore the request.
	 *
	 * Note that we check for a reference count of 1 because the page is
	 * always mapped into the direct map.
	 */
	if (pgDesc->count == 1 || pgDesc->count == 0) {
		/*
		 * Mark the page frame as an unused page.
		 */
		pgDesc->type = PG_UNUSED;

		/*
		 * Make the page writeable again.  Be sure to flush the TLBs to
		 * make the change take effect right away.
		 */
		page_entry_store((page_entry_t *) pte,
				 setMappingReadWrite(*pte));
		_mm_flush_tlb(getVirtual(paddr));
	} else {
		printf("NK: remove_page: type=%x count %x\n",
			pgDesc->type, pgDesc->count);
	}
}

#define PT_UPDATE_INVALID	0
#define PT_UPDATE_VALID_RO	1
#define PT_UPDATE_VALID		2

static inline int
isFramePg(page_desc_t *page)
{
	return
		(page->type == PG_UNUSED) ||	/* Defines an unused page */
		(page->type == PG_TKDATA) ||	/* Defines a kernel data page */
		(page->type == PG_TUDATA) ||	/* Defines a user data page */
		(page->type == PG_CODE);	/* Defines a code page */
}

/*
 * Function: _pt_update_is_valid()
 *
 * Description:
 *  This function assesses a potential page table update for a valid mapping.
 *
 *  NOTE: This function assumes that the page being mapped in has already been
 *  declared and has its intial page metadata captured as defined in the
 *  initial mapping of the page.
 *
 * Inputs:
 *  *page_entry  - VA pointer to the page entry being modified
 *  newVal       - Representes the new value to write including the reference
 *                 to the underlying mapping.
 *
 * Return:
 *  PT_UPDATE_INVALID  - The update is not valid and should not be performed.
 *  PT_UPDATE_VALID_RO - The update is valid but should disable write access.
 *  PT_UPDATE_VALID    - The update is valid and can be performed.
 */
static int _pt_update_is_valid(page_entry_t *page_entry, page_entry_t newVal)
{
	/* Collect associated information for the existing mapping */
	unsigned long origPA = *page_entry & PG_FRAME;
	unsigned long origFrame = origPA >> PAGESHIFT;
	uintptr_t origVA = (uintptr_t) getVirtual(origPA);

	page_desc_t *origPG = getPageDescPtr(origPA);

	/* Get associated information for the new page being mapped */
	unsigned long newPA = newVal & PG_FRAME;
	unsigned long newFrame = newPA >> PAGESHIFT;
	uintptr_t newVA = (uintptr_t) getVirtual(newPA);
	page_desc_t *newPG = getPageDescPtr(newVal);

	/* Get the page table page descriptor. The page_entry is the viratu */
	uintptr_t ptePAddr = getPhysicalAddr (page_entry);
	page_desc_t *ptePG = getPageDescPtr(ptePAddr);

	/* Return value */
	unsigned char retValue = PT_UPDATE_VALID_RO;

	/*
	 * Determine if the page table pointer is within the direct map. If not,
	 * then it's an error.
	 *
	 * TODO: This check can cause a panic because the SVA VM does not set
	 *       up the direct map before starting the kernel. As a result, we
	 *       get page table addresses that don't fall into the direct map.
	 */
#if REVIEW_OBSOLETE // nk doesn't require DMAP only aliases
	SVA_NOOP_ASSERT(isDirectMap (page_entry), "SVA: MMU: Not direct map\n");
#endif

	/*
	 * Add check that the direct map is not being modified.
	 *
	 * TODO: This should be a check to make sure that we are updating a
	 * PTP page.
	 */
	if (PG_DML1 <= ptePG->type && ptePG->type <= PG_DML4)
		panic ("SVA: MMU: Modifying direct map!\n");

	/*
	 * If we aren't mapping a new page then we can skip several checks, and in
	 * some cases we must, otherwise, the checks will fail. For example if this
	 * is a mapping in a page table page then we allow a zero mapping.
	 */
	if (newVal & PG_V) {
#if REVIEW_OBSOLETE
		/* If the mapping is to an SVA page then fail */
		SVA_ASSERT(!isSVAPg(newPG), "Kernel attempted to map an SVA page");
#endif

		/*
		 * New mappings to code pages are permitted as long as they are
		 * either for user-space pages or do not permit write access.
		 */
		if (newPG->type == PG_CODE) {
			if ((newVal & (PG_RW | PG_U)) == (PG_RW)) {
				panic("SVA: Making kernel code writeable: %lx %lx\n",
				      newVA, newVal);
			}
		}

		/*
		 * If the new page is a page table page, then we verify some
		 * page table page specific checks.
		 */
		if (newPG->type >= PG_L1 && newPG->type <= PG_L4) {
			/*
			 * If we have a page table page being mapped in and it
			 * currently has a mapping to it, then we verify that
			 * the new VA from the new mapping matches the existing
			 * currently mapped VA.
			 *
			 * This guarantees that we each page table page (and the
			 * translations within it) maps a singular region of the
			 * address space.
			 *
			 * Otherwise, this is the first mapping of the page, and
			 * we should record in what virtual address it is being
			 * placed.
			 */
#if 0
			if (newPG->count > 1) {
				if (newPG->pgVaddr != page_entry) {
					panic("SVA: PG: %lx %lx: type=%x\n",
					      newPG->pgVaddr, page_entry,
					      newPG->type);
				}
				SVA_ASSERT (newPG->pgVaddr == page_entry, "MMU: Map PTP to second VA");
			} else {
				newPG->pgVaddr = page_entry;
			}
#endif
		}

		/*
		 * Verify that that the mapping matches the correct type of page
		 * allowed to be mapped into this page table. Verify that the new
		 * PTP is of the correct type given the page level of the page
		 * entry.
		 */
		switch (ptePG->type) {
		case PG_L1:
			if (!isFramePg(newPG)) {
				/*
				 * If it is a page table page, just ensure that it is not writeable.
				 * The kernel may be modifying the direct map, and we will permit
				 * that as long as it doesn't make page tables writeable.
				 *
				 * Note: The SVA VM really should have its own direct map that the
				 *       kernel cannot use or modify, but that is too much work, so
				 *       we make this compromise.
				 */
				if (newPG->type >= PG_L1 && newPG->type <= PG_L4)
					retValue = PT_UPDATE_VALID_RO;
				else
					panic("SVA: MMU: Map bad page type into L1: %x\n", newPG->type);
			}
			break;
		case PG_L2:
			if (newVal & PG_PS) {
				if (!isFramePg(newPG)) {
					/*
					 * If it is a page table page, just ensure that it is not writeable.
					 * The kernel may be modifying the direct map, and we will permit
					 * that as long as it doesn't make page tables writeable.
					 *
					 * Note: The SVA VM really should have its own direct map that the
					 *       kernel cannot use or modify, but that is too much work, so
					 *       we make this compromise.
					 */
					if (newPG->type >= PG_L1 && newPG->type <= PG_L4)
						retValue = PT_UPDATE_VALID_RO;
					else
						panic("SVA: MMU: Map bad page type into L2: %x\n",
						      newPG->type);
				}
			} else {
				if (newPG->type != PG_L1)
					panic("MMU: Mapping non-L1 page into L2.");
			}
			break;
		case PG_L3:
			if (newVal & PG_PS) {
				if (!isFramePg(newPG)) {
					/*
					 * If it is a page table page, just ensure that it is not writeable.
					 * The kernel may be modifying the direct map, and we will permit
					 * that as long as it doesn't make page tables writeable.
					 *
					 * Note: The SVA VM really should have its own direct map that the
					 *       kernel cannot use or modify, but that is too much work, so
					 *       we make this compromise.
					 */
					if (newPG->type >= PG_L1 && newPG->type <= PG_L4)
						retValue = PT_UPDATE_VALID_RO;
					else
						panic("SVA: MMU: Map bad page type into L2: %x\n",
						      newPG->type);
				}
			} else {
				if (newPG->type != PG_L2)
					panic("MMU: Mapping non-L2 page into L3.");
			}
			break;
		case PG_L4:
			/*
			 * FreeBSD inserts a self mapping into the pml4, therefore it is
			 * valid to map in an L4 page into the L4.
			 *
			 * TODO: Consider the security implications of allowing an L4 to map
			 *       an L4.
			 */
			if (newPG->type != PG_L3 && newPG->type != PG_L4)
				panic("MMU: Mapping non-L3/L4 page into L4.");
			break;

		default:
			break;
		}
	}

	/*
	 * If the new mapping is set for user access, but the VA being used is
	 * to kernel space, fail. Also capture in this check is if the new
	 * mapping is set for super user access, but the VA being used is to
	 * user space, fail.
	 *
	 * 3 things to assess for matches:
	 *  - U/S Flag of new mapping
	 *  - Type of the new mapping frame
	 *  - Type of the PTE frame
	 *
	 * Ensures the new mapping U/S flag matches the PT page frame type and
	 * the mapped in frame's page type, as well as no mapping kernel code
	 * pages into userspace.
	 */

	/*
	 * If the original PA is not equivalent to the new PA then we are
	 * creating an entirely new mapping, thus make sure that this is a valid
	 * new page reference. Also verify that the reference counts to the old
	 * page are sane, i.e., there is at least a current count of 1 to it.
	 */
	if (origPA != newPA) {
		/*
		 * If the old mapping was to a code page then we know we
		 * shouldn't be pointing this entry to another code page,
		 * thus fail.
		 */
		if (origPG->type == PG_CODE) {
			if (!(*page_entry & PG_U))
				panic("Kernel attempting to modify code page mapping");
		}
	}

	return retValue;
}

/*
 * Function: _update_mapping
 *
 * Description:
 *  Mapping update function that is agnostic to the level of page table. Most
 *  of the verification code is consistent regardless of which level page
 *  update we are doing.
 *
 * Inputs:
 *  - pageEntryPtr : reference to the page table entry to insert the mapping
 *      into
 *  - val : new entry value
 */
static void _update_mapping(pte_t *pageEntryPtr, page_entry_t val)
{
	int ret;

	/*
	 * If the given page update is valid then store the new value to the page
	 * table entry, else raise an error.
	 */
	ret = _pt_update_is_valid((page_entry_t *) pageEntryPtr, val);
	switch (ret) {
	case PT_UPDATE_VALID_RO:
		/*
		 * Kernel thinks these should be RW, since it wants to
		 * write to them.
		 * Convert to read-only and carry on.
		 */
		val = setMappingReadOnly(val);
	case PT_UPDATE_VALID:
		_do_mmu_update((page_entry_t *) pageEntryPtr, val);
		break;
	default:
		panic("##### NK invalid page update!!!\n");
	}
}

/*
 * Function: nk_pmmu_update_l1_mapping()
 *
 * Description:
 *  This function updates a Level-1 Mapping.  In other words, it adds a
 *  a direct translation from a virtual page to a physical page.
 *
 *  This function makes different checks to ensure the mapping
 *  does not bypass the type safety proved by the compiler.
 *
 * Inputs:
 *  pteptr - The location within the L1 page in which the new translation
 *           should be place.
 *  val    - The new translation to insert into the page table.
 */
void nk_pmmu_update_l1_mapping(pte_t *pteptr, page_entry_t val)
{
	/*
	 * Ensure that the PTE pointer points to an L1 page table.
	 * If it does not, then report an error.
	 */
	page_desc_t *ptDesc = getPageDescPtr(getPhysicalAddr(pteptr));

	if (ptDesc->type != PG_L1)
		panic("NK: MMU: update_l1 not an L1: %lx %lx: %lx\n",
		      pteptr, val, ptDesc->type);

	/*
	 * Update the page table with the new mapping.
	 */
	// printf("[NK] update_l1: pteptr=%p\n", pteptr);
	_update_mapping(pteptr, val);
}

/*
 * Updates a level2 mapping (a mapping to a l1 page).
 *
 * This function checks that the pages involved in the mapping
 * are correct, ie pmdptr is a level2, and val corresponds to
 * a level1.
 */
void nk_pmmu_update_l2_mapping(pde_t *pdePtr, page_entry_t val)
{
	/*
	 * Ensure that the PTE pointer points to an L1 page table.
	 * If it does not, then report an error.
	 */
	page_desc_t *ptDesc = getPageDescPtr(getPhysicalAddr(pdePtr));

	if (ptDesc->type != PG_L2)
		panic("NK: MMU: update_l2 not an L2: %lx %lx: %lx\n",
		      pdePtr, val, ptDesc->type);

	/*
	 * Update the page mapping.
	 */
	// printf("[NK] update_l2: pdeptr=%p\n", pdePtr);
	_update_mapping(pdePtr, val);
}

/*
 * Updates a level3 mapping
 */
void nk_pmmu_update_l3_mapping(pdpte_t *pdptePtr, page_entry_t val)
{
	/*
	 * Ensure that the PTE pointer points to an L1 page table.
	 * If it does not, then report an error.
	 */
	page_desc_t *ptDesc = getPageDescPtr(getPhysicalAddr(pdptePtr));

	if (ptDesc->type != PG_L3)
		panic("SVA: MMU: update_l3 not an L3: %lx %lx: %lx\n",
		      pdptePtr, val, ptDesc->type);

	// printf("[NK] update_l3: pdpteptr=%p\n", pdptePtr);
	_update_mapping(pdptePtr, val);
}

/*
 * Updates a level4 mapping
 */
void nk_pmmu_update_l4_mapping(pml4e_t *pml4ePtr, page_entry_t val)
{
	/*
	 * Ensure that the PTE pointer points to an L1 page table.
	 * If it does not, then report an error.
	 */
	page_desc_t *ptDesc = getPageDescPtr(getPhysicalAddr(pml4ePtr));
	if (ptDesc->type != PG_L4)
		panic("NK: MMU: update_l4 not an L4: %lx %lx: %lx\n",
		      pml4ePtr, val, ptDesc->type);

	// printf("[NK] update_l4: pml4ePtr=%p\n", pml4ePtr);
	_update_mapping(pml4ePtr, val);
}

/*
 * Function: nk_pmmu_remove_mapping()
 *
 * Description:
 *  This function updates the entry to the page table page and is agnostic to
 *  the level of page table. The particular needs for each page table level are
 *  handled in the _update_mapping function. The primary function here is to
 *  set the mapping to zero, if the page was a PTP then zero it's data and set
 *  it to writeable.
 *
 * Inputs:
 *  pteptr - The location within the page table page for which the translation
 *           should be removed.
 */
void nk_pmmu_remove_mapping(page_entry_t *pteptr)
{
	page_desc_t *pgDesc = getPageDescPtr(*pteptr);

	/* Update the page table mapping to zero */
	_update_mapping (pteptr, ZERO_MAPPING);
}
