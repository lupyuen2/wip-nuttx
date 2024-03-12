/****************************************************************************
 * arch/risc-v/src/common/riscv_exception.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#ifdef CONFIG_PAGING
#  include <nuttx/pgalloc.h>
#endif

#ifdef CONFIG_PAGING
#  include "pgalloc.h"
#  include "riscv_mmu.h"
#endif

#include "riscv_internal.h"
#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_reasons_str[RISCV_MAX_EXCEPTION + 1] =
{
  "Instruction address misaligned",
  "Instruction access fault",
  "Illegal instruction",
  "Breakpoint",
  "Load address misaligned",
  "Load access fault",
  "Store/AMO address misaligned",
  "Store/AMO access fault",
  "Environment call from U-mode",
  "Environment call from S-mode",
  "Environment call from H-mode",
  "Environment call from M-mode",
  "Instruction page fault",
  "Load page fault",
  "Reserved",
  "Store/AMO page fault"
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_exception
 *
 * Description:
 *   This is the exception handler.
 *
 ****************************************************************************/

int riscv_exception(int mcause, void *regs, void *args)
{
  uintptr_t cause = mcause & RISCV_IRQ_MASK;

  _alert("EXCEPTION: %s. MCAUSE: %" PRIxREG ", EPC: %" PRIxREG
         ", MTVAL: %" PRIxREG "\n",
         mcause > RISCV_MAX_EXCEPTION ? "Unknown" : g_reasons_str[cause],
         cause, READ_CSR(CSR_EPC), READ_CSR(CSR_TVAL));

  _alert("PANIC!!! Exception = %" PRIxREG "\n", cause);
  up_irq_save();
  CURRENT_REGS = regs;
  PANIC_WITH_REGS("panic", regs);

  return 0;
}

/****************************************************************************
 * Name: riscv_fillpage
 *
 * Description:
 *   This function is an exception handler for page faults in a RISC-V.
 *   It is invoked when a page fault exception occurs, which is typically
 *   when a process tries to access a page that is not currently in memory.
 *
 *   The function takes as arguments the machine cause (mcause) which
 *   indicates the cause of the exception, a pointer to the register state
 *   at the time of the exception (regs), and a pointer to any additional
 *   arguments (args).
 *
 *   The function should handle the exception appropriately, typically by
 *   loading the required page into memory and updating the page table.
 *
 * Input Parameters:
 *   mcause - The machine cause of the exception.
 *   regs   - A pointer to the register state at the time of the exception.
 *   args   - A pointer to any additional arguments.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING
int riscv_fillpage(int mcause, void *regs, void *args)
{
  uintptr_t cause = mcause & RISCV_IRQ_MASK;
  uintptr_t ptlast;
  uintptr_t ptprev;
  uintptr_t paddr;
  uintptr_t vaddr;
  uint32_t  ptlevel;
  uintptr_t satp;
  uint32_t mmuflags;

  _info("EXCEPTION: %s. MCAUSE: %" PRIxREG ", EPC: %" PRIxREG
        ", MTVAL: %" PRIxREG "\n",
        mcause > RISCV_MAX_EXCEPTION ? "Unknown" : g_reasons_str[cause],
        cause, READ_CSR(CSR_EPC), READ_CSR(CSR_TVAL));
  vaddr = MM_PGALIGNDOWN(READ_CSR(CSR_TVAL));

  // CONFIG_ARCH_TEXT_VBASE=0x80000000
  // CONFIG_ARCH_TEXT_NPAGES=128
  // CONFIG_MM_PGSIZE=4096
  // #define ARCH_TEXT_SIZE  (CONFIG_ARCH_TEXT_NPAGES * CONFIG_MM_PGSIZE)
  // #define ARCH_TEXT_VEND  (CONFIG_ARCH_TEXT_VBASE + ARCH_TEXT_SIZE) ////
  _info("ARCH_TEXT_SIZE=%p\n", ARCH_TEXT_SIZE);////
  _info("ARCH_TEXT_VEND=%p\n", ARCH_TEXT_VEND);////
  _info("vaddr=%p\n", vaddr);////

  #define FIX_CONFIG_ARCH_TEXT_VBASE 0x80000000ul
  #define FIX_ARCH_TEXT_VEND  (FIX_CONFIG_ARCH_TEXT_VBASE + ARCH_TEXT_SIZE) ////
  _info("FIX_ARCH_TEXT_VEND=%p\n", FIX_ARCH_TEXT_VEND);////
  up_mdelay(1000);////

  if (vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr <= ARCH_TEXT_VEND)
    {
      _info("vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr <= ARCH_TEXT_VEND\n");////
      mmuflags = MMU_UTEXT_FLAGS;

      /* Write access to .text region needs to be set according to
       * https://github.com/apache/nuttx/pull/6193.
       */

      mmuflags |= PTE_W;
    }
  else if (vaddr >= CONFIG_ARCH_DATA_VBASE && vaddr <= ARCH_DATA_VEND)
    {
      _info("vaddr >= CONFIG_ARCH_DATA_VBASE && vaddr <= ARCH_DATA_VEND\n");////
      mmuflags = MMU_UDATA_FLAGS;
    }
  else if (vaddr >= CONFIG_ARCH_HEAP_VBASE && vaddr <= ARCH_HEAP_VEND)
    {
      _info("vaddr >= CONFIG_ARCH_HEAP_VBASE && vaddr <= ARCH_HEAP_VEND\n");////
      mmuflags = MMU_UDATA_FLAGS;
    }
  else
    {
      _info("else\n");////
      _alert("PANIC!!! virtual address not mappable: %" PRIxPTR "\n", vaddr);
      up_irq_save();
      CURRENT_REGS = regs;
      PANIC_WITH_REGS("panic", regs);
      for(;;) {} ////
    }

  satp    = READ_CSR(CSR_SATP);
  ptprev  = riscv_pgvaddr(mmu_satp_to_paddr(satp));
  ptlevel = ARCH_SPGTS;
  paddr = mmu_pte_to_paddr(mmu_ln_getentry(ptlevel, ptprev, vaddr));
  if (!paddr)
    {
      _info("!paddr1\n");////
      /* Nothing yet, allocate one page for final level page table */

      paddr = mm_pgalloc(1);
      if (!paddr)
        {
          _info("!paddr2\n");////
          return -ENOMEM;
        }

      /* Map the page table to the prior level */

      _info("mmu_ln_setentry1: ptlevel=%p, ptprev=%p, paddr=%p, vaddr=%p, MMU_UPGT_FLAGS=%p\n", ptlevel, ptprev, paddr, vaddr, MMU_UPGT_FLAGS);////
      up_mdelay(1000);////
      mmu_ln_setentry(ptlevel, ptprev, paddr, vaddr, MMU_UPGT_FLAGS);

      /* This is then used to map the final level */

      _info("riscv_pgwipe1\n");////
      up_mdelay(1000);////
      riscv_pgwipe(paddr);
    }

  _info("riscv_pgvaddr\n");////
  ptlast = riscv_pgvaddr(paddr);
  _info("mm_pgalloc\n");////
  paddr = mm_pgalloc(1);
  if (!paddr)
    {
      _info("!paddr3\n");////
      return -ENOMEM;
    }

  /* Wipe the physical page memory */

  _info("riscv_pgwipe2\n");////
  riscv_pgwipe(paddr);

  /* Then map the virtual address to the physical address */

  _info("mmu_ln_setentry2: mmuflags=0x%x\n", mmuflags);////
  mmu_ln_setentry(ptlevel + 1, ptlast, paddr, vaddr, mmuflags);

  _info("return\n");////
  return 0;
}
#endif /* CONFIG_PAGING */

/****************************************************************************
 * Name: riscv_exception_attach
 *
 * Description:
 *   Attach standard exception with suitable handler
 *
 ****************************************************************************/

void riscv_exception_attach(void)
{
  irq_attach(RISCV_IRQ_IAMISALIGNED, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_IAFAULT, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_IINSTRUCTION, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_BPOINT, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_LAFAULT, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_SAFAULT, riscv_exception, NULL);

#ifdef CONFIG_RISCV_MISALIGNED_HANDLER
  irq_attach(RISCV_IRQ_LAMISALIGNED, riscv_misaligned, NULL);
  irq_attach(RISCV_IRQ_SAMISALIGNED, riscv_misaligned, NULL);
#else
  irq_attach(RISCV_IRQ_LAMISALIGNED, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_SAMISALIGNED, riscv_exception, NULL);
#endif

  /* Attach the ecall interrupt handler */

#ifndef CONFIG_BUILD_FLAT
  irq_attach(RISCV_IRQ_ECALLU, riscv_swint, NULL);
#else
  irq_attach(RISCV_IRQ_ECALLU, riscv_exception, NULL);
#endif

  irq_attach(RISCV_IRQ_ECALLS, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_ECALLH, riscv_exception, NULL);

#ifndef CONFIG_ARCH_USE_S_MODE
  irq_attach(RISCV_IRQ_ECALLM, riscv_swint, NULL);
#else
  irq_attach(RISCV_IRQ_ECALLM, riscv_exception, NULL);
#endif

  irq_attach(RISCV_IRQ_INSTRUCTIONPF, riscv_exception, NULL);

#ifdef CONFIG_PAGING
  irq_attach(RISCV_IRQ_LOADPF, riscv_fillpage, NULL);
  irq_attach(RISCV_IRQ_STOREPF, riscv_fillpage, NULL);
#else
  irq_attach(RISCV_IRQ_LOADPF, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_STOREPF, riscv_exception, NULL);
#endif

  irq_attach(RISCV_IRQ_RESERVED, riscv_exception, NULL);

#ifdef CONFIG_SMP
  irq_attach(RISCV_IRQ_SOFT, riscv_pause_handler, NULL);
#else
  irq_attach(RISCV_IRQ_MSOFT, riscv_exception, NULL);
#endif
}
