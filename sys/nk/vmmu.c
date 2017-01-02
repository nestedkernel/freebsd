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
