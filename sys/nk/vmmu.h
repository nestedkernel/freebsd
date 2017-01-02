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
 *       Filename:  vmmu.h
 *
 *    Description:
 *
 *===-----------------------------------------------------------------------===
 */

#ifndef __VMMU_H_
#define __VMMU_H_

#include "pmmu.h"

void	nk_load_pgtbl_base_ptr(register_t val);
void	nk_vmmu_init(pml4e_t *kpml4Mapping, unsigned long nkpml4e,
		     uintptr_t *firstpaddr, uintptr_t btext, uintptr_t etext);

void	nk_vmmu_declare_l1_page(uintptr_t frame);
void	nk_vmmu_declare_l2_page(uintptr_t frame);
void	nk_vmmu_declare_l3_page(uintptr_t frame);
void	nk_vmmu_declare_l4_page(uintptr_t frame);
void	nk_vmmu_remove_page(uintptr_t frame);

void	nk_vmmu_update_l1_mapping(pte_t *ptePtr, page_entry_t val);
void	nk_vmmu_update_l2_mapping(pde_t *pdePtr, page_entry_t val);
void	nk_vmmu_update_l3_mapping(pdpte_t *pdptePtr, page_entry_t val);
void	nk_vmmu_update_l4_mapping(pml4e_t *pml4ePtr, page_entry_t val);
void	nk_vmmu_remove_mapping(page_entry_t *ptePtr);

#endif /* !__VMMU_H_ */
