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
 *       Filename:  vmmu.c
 *
 *    Description:  This file includes the primary interface to the outer
 *                  kernel for modifying page-table-pages. It does this through
 *                  the use of declare and update PTS as well as remove and set
 *                  PTP base pointer (cr3 in amd64).
 *
 *===-----------------------------------------------------------------------===
 */

#include "cpufunc.h"
#include "common.h"
#include "stack.h"
#include "pmmu.h"
#include "vmmu.h"

/*
 * Function: nk_load_pgtbl_base_ptr
 *
 * Description:
 *  Set the current page table and verify that the page being loaded has been
 *  declard as a top-level page table page.
 *
 * Inputs:
 *  pg - The physical address of the top-level page table page.
 */
void
nk_load_pgtbl_base_ptr(register_t val)
{
	/* Execute the load */
	_load_cr3(val);

	/*
	 * Check that the new page table is an L4 page table page.
	 */
#if NOT_PORTED_YET
	if ((mmuIsInitialized) && (getPageDescPtr(pg)->type != PG_L4))
		panic("SVA: Loading non-L4 page into CR3: %lx %x\n",
		      pg, getPageDescPtr(pg)->type);
#endif
}

/*
 * Function: nk_vmmu_init
 *
 * Description:
 *  This function initializes the nk vmmu unit by zeroing out the page
 *  descriptors, capturing the statically allocated initial kernel mmu state,
 *  and identifying all kernel code pages, and setting them in the page
 *  descriptor array.
 *
 * Inputs:
 *  - kpml4Mapping  : Mapping referencing the base kernel pml4 page table page
 *  - nkpml4e       : The number of entries in the pml4
 *  - firstpaddr    : A pointer to the physical address of the first free frame.
 *  - btext         : The first virtual address of the text segment.
 *  - etext         : The last virtual address of the text segment.
 */
SECURE_WRAPPER(void, nk_vmmu_init,
	       pml4e_t *kpml4Mapping, unsigned long nkpml4e,
	       uintptr_t *firstpaddr, uintptr_t btext, uintptr_t etext)
{
	pmmu_init(kpml4Mapping, nkpml4e, firstpaddr, btext, etext);
}


/*
 * Intrinsic: nk_vmmu_declare_l1_page()
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
SECURE_WRAPPER(void, nk_vmmu_declare_l1_page,
	       uintptr_t frameAddr)
{
	nk_pmmu_declare_l1_page(frameAddr);
}

/*
 * Intrinsic: nk_vmmu_declare_l2_page()
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
SECURE_WRAPPER(void, nk_vmmu_declare_l2_page,
	       uintptr_t frameAddr)
{
	nk_pmmu_declare_l2_page(frameAddr);
}

/*
 * Intrinsic: nk_vmmu_declare_l3_page()
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
SECURE_WRAPPER(void, nk_vmmu_declare_l3_page,
	       uintptr_t frameAddr)
{
	nk_pmmu_declare_l2_page(frameAddr);
}

/*
 * Intrinsic: nk_vmmu_declare_l4_page()
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
SECURE_WRAPPER(void,
nk_vmmu_declare_l4_page, uintptr_t frameAddr)
{
	nk_pmmu_declare_l4_page(frameAddr);
}

/*
 * Function: nk_vmmu_remove_page()
 *
 * Description:
 *  This function informs the NK VM that the system software no longer wants
 *  to use the specified page as a page table page.
 *
 * Inputs:
 *  paddr - The physical address of the page table page.
 */
SECURE_WRAPPER(void, nk_vmmu_remove_page,
	       uintptr_t paddr)
{
	nk_pmmu_remove_page(paddr);
}

/*
 * Function: nk_vmmu_update_l1_mapping()
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
SECURE_WRAPPER(void, nk_vmmu_update_l1_mapping,
	       pte_t *pteptr, page_entry_t val)
{
	nk_pmmu_update_l1_mapping(pteptr, val);
}

/*
 * Updates a level2 mapping (a mapping to a l1 page).
 *
 * This function checks that the pages involved in the mapping
 * are correct, ie pmdptr is a level2, and val corresponds to
 * a level1.
 */
SECURE_WRAPPER(void, nk_vmmu_update_l2_mapping,
	       pde_t *pdePtr, page_entry_t val)
{
	nk_pmmu_update_l2_mapping(pdePtr, val);
}

/*
 * Updates a level3 mapping
 */
SECURE_WRAPPER(void, nk_vmmu_update_l3_mapping,
	       pdpte_t *pdptePtr, page_entry_t val)
{
	nk_pmmu_update_l3_mapping(pdptePtr, val);
}

/*
 * Updates a level4 mapping
 */
SECURE_WRAPPER(void, nk_vmmu_update_l4_mapping,
	       pml4e_t *pml4ePtr, page_entry_t val)
{
	nk_pmmu_update_l4_mapping(pml4ePtr, val);
}

/*
 * Function: nk_vmmu_remove_mapping()
 *
 * Description:
 *  This function updates the entry to the page table page and is agnostic to
 *  the level of page table. The particular needs for each page table level are
 *  handled in the __update_mapping function. The primary function here is to
 *  set the mapping to zero, if the page was a PTP then zero it's data and set
 *  it to writeable.
 *
 * Inputs:
 *  pteptr - The location within the page table page for which the translation
 *           should be removed.
 */
SECURE_WRAPPER(void, nk_vmmu_remove_mapping,
	       page_entry_t *pteptr)
{
	nk_pmmu_remove_mapping(pteptr);
}
