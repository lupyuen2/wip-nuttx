/****************************************************************************
 * arch/arm/src/lpc2378/lpc2378.h
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2010,2015 Rommel Marcelo. All rights reserved.
 * SPDX-FileCopyrightText: 2010 Gregory Nutt. All rights reserved.
 * SPDX-FileContributor: Rommel Marcelo
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC2378_LPC2378_H
#define __ARCH_ARM_SRC_LPC2378_LPC2378_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* #define CONFIG_VECTORED_INTERRUPTS */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_ARCH_LEDS
void lpc2378_statledoff(void);
void lpc2378_statledon(void);
#endif

#endif

#endif /* __ARCH_ARM_SRC_LPC2378_LPC2378_H */
