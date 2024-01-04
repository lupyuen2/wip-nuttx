/****************************************************************************
 * arch/risc-v/src/bl602/bl602_gpio.c
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

#include <stdint.h>
#include <sys/param.h>

#include "riscv_internal.h"
#include "hardware/bl602_glb.h"
#include "bl602_gpio.h"

////TODO
#define BL602_GLB_BASE        0x20000000ul  /* glb */
#define reg_gpio_xx_o 24
#define reg_gpio_xx_i 28

////TODO: Can pinset map 46 pins?

/****************************************************************************
 * Private Data
 ****************************************************************************/

////TODO: Rename g_ to static
static const uintptr_t g_gpio_base[] =
{
  BL602_GPIO_CFG0,
  BL602_GPIO_CFG1,
  // BL602_GPIO_CFGCTL2,
  // BL602_GPIO_CFGCTL3,
  // BL602_GPIO_CFGCTL4,
  // BL602_GPIO_CFGCTL5,
  // BL602_GPIO_CFGCTL6,
  // BL602_GPIO_CFGCTL7,
  // BL602_GPIO_CFGCTL8,
  // BL602_GPIO_CFGCTL9,
  // BL602_GPIO_CFGCTL10,
  // BL602_GPIO_CFGCTL11,
  // BL602_GPIO_CFGCTL12,
  // BL602_GPIO_CFGCTL13,
  // BL602_GPIO_CFGCTL14,
  // gpio_cfg141
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int bl602_configgpio(gpio_pinset_t cfgset)
{
  uint32_t mask;
  uintptr_t regaddr;
  uint32_t cfg = 0;
  uint8_t pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  if (pin >= nitems(g_gpio_base))
    {
      return ERROR;
    }

  /* The configuration masks will just use the defines for GPIO0 which is
   * the same as all GPIO config bit fields besides the offset.
   */

  if (cfgset & GPIO_INPUT)
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_IE;
    }

  if (cfgset & GPIO_PULLUP)
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_PU;
    }

  if (cfgset & GPIO_PULLDOWN)
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_PD;
    }

  if (cfgset & GPIO_DRV_MASK)
    {
      cfg |= ((cfgset & GPIO_DRV_MASK) >> GPIO_DRV_SHIFT) << \
        GPIO_CFGCTL0_GPIO_0_DRV_SHIFT;
    }

  if (cfgset & GPIO_SMT_EN)
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_SMT;
    }

  if (cfgset & GPIO_FUNC_MASK)
    {
      cfg |= ((cfgset & GPIO_FUNC_MASK) >> GPIO_FUNC_SHIFT) << \
        GPIO_CFGCTL0_GPIO_0_FUNC_SEL_SHIFT;
    }

  regaddr = g_gpio_base[pin];
  mask = 0xffff;
  if ((pin & 1) == 1)
    {
      cfg = cfg << 16;
      mask = mask << 16;
    }

  modifyreg32(regaddr, mask, cfg);

  // TODO
  // /* Enable pin output if requested */

  // if (!(cfgset & GPIO_INPUT))
  //   {
  //     modifyreg32(BL602_GPIO_CFGCTL34, 0, (1 << pin));
  //     modifyreg32(regaddr, 0, (1 << reg_gpio_xx_o));
  //   }

  return OK;
}

/****************************************************************************
 * Name: bl602_gpio_deinit
 *
 * Description:
 *   Deinit a GPIO (Set GPIO to floating input state)
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int bl602_gpio_deinit(uint8_t pin)
{
  bl602_configgpio(GPIO_INPUT | GPIO_FLOAT | pin);
  return OK;
}

/****************************************************************************
 * Name: bl602_config_uart_sel
 *
 * Description:
 *   Configure the GPIO UART pin selection mux based on bit-encoded
 *   description of the pin and the selection signal
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int bl602_config_uart_sel(gpio_pinset_t pinset, uint8_t sig_sel)
{
  irqstate_t flags;
  uint8_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint8_t sel_idx;
  uint32_t reg;

  if ((pin >= nitems(g_gpio_base)) || sig_sel > UART_SIG_SEL_UART1_RXD)
    {
      return ERROR;
    }

  sel_idx = pin % 8;
  flags = enter_critical_section();

  reg = getreg32(BL602_UART_SIG_SEL_0);
  reg &= ~(0xf << (sel_idx * 4));
  reg |= sig_sel << (sel_idx * 4);
  putreg32(reg, BL602_UART_SIG_SEL_0);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: bl602_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void bl602_gpiowrite(gpio_pinset_t pinset, bool value)
{
  uintptr_t regaddr;
  uint8_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT(pin < nitems(g_gpio_base));
  regaddr = g_gpio_base[pin];
  if (value)
    {
      modifyreg32(regaddr, 0, (1 << reg_gpio_xx_o));
    }
  else
    {
      modifyreg32(regaddr, (1 << reg_gpio_xx_o), 0);
    }
}

/****************************************************************************
 * Name: bl602_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool bl602_gpioread(gpio_pinset_t pinset)
{
  uintptr_t regaddr;
  uint8_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT(pin < nitems(g_gpio_base));
  regaddr = g_gpio_base[pin];
  return ((getreg32(regaddr) & (1 << reg_gpio_xx_i)) ? 1 : 0);
}
