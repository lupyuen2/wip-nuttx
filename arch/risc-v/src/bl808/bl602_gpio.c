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

#include <nuttx/config.h>

#include <debug.h>
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
  BL602_GPIO_CFG2,
  BL602_GPIO_CFG3,
  BL602_GPIO_CFG4,
  BL602_GPIO_CFG5,
  BL602_GPIO_CFG6,
  BL602_GPIO_CFG7,
  BL602_GPIO_CFG8,
  BL602_GPIO_CFG9,

  BL602_GPIO_CFG10,
  BL602_GPIO_CFG11,
  BL602_GPIO_CFG12,
  BL602_GPIO_CFG13,
  BL602_GPIO_CFG14,
  BL602_GPIO_CFG15,
  BL602_GPIO_CFG16,
  BL602_GPIO_CFG17,
  BL602_GPIO_CFG18,
  BL602_GPIO_CFG19,

  BL602_GPIO_CFG20,
  BL602_GPIO_CFG21,
  BL602_GPIO_CFG22,
  BL602_GPIO_CFG23,
  BL602_GPIO_CFG24,
  BL602_GPIO_CFG25,
  BL602_GPIO_CFG26,
  BL602_GPIO_CFG27,
  BL602_GPIO_CFG28,
  BL602_GPIO_CFG29,

  BL602_GPIO_CFG30,
  BL602_GPIO_CFG31,
  BL602_GPIO_CFG32,
  BL602_GPIO_CFG33,
  BL602_GPIO_CFG34,
  BL602_GPIO_CFG35,
  BL602_GPIO_CFG36,
  BL602_GPIO_CFG37,
  BL602_GPIO_CFG38,
  BL602_GPIO_CFG39,

  BL602_GPIO_CFG40,
  BL602_GPIO_CFG41,
  BL602_GPIO_CFG42,
  BL602_GPIO_CFG43,
  BL602_GPIO_CFG44,
  BL602_GPIO_CFG45
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
  uintptr_t regaddr;
  uint32_t cfg = 0;
  uint8_t pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  if (pin >= nitems(g_gpio_base))
    {
      return ERROR;
    }

  //// TODO: Change GPIO_CFGCTL0_GPIO_0_IE to GPIO_CFG_GPIO_IE
  if (cfgset & GPIO_INPUT)
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_IE;
    }
  else
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_OE;
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
      cfg |= ((cfgset & GPIO_DRV_MASK) >> GPIO_DRV_SHIFT) <<
        GPIO_CFGCTL0_GPIO_0_DRV_SHIFT;
    }

  if (cfgset & GPIO_SMT_EN)
    {
      cfg |= GPIO_CFGCTL0_GPIO_0_SMT;
    }

  if (cfgset & GPIO_FUNC_MASK)
    {
      cfg |= ((cfgset & GPIO_FUNC_MASK) >> GPIO_FUNC_SHIFT) <<
        GPIO_CFGCTL0_GPIO_0_FUNC_SEL_SHIFT;
    }

  regaddr = g_gpio_base[pin];
  _info("regaddr=%p, cfg=0x%x\n", regaddr, cfg);////
  putreg32(cfg, regaddr);
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
      _info("regaddr=%p, set=0x%x\n", regaddr, (1 << reg_gpio_xx_o));////
      modifyreg32(regaddr, 0, (1 << reg_gpio_xx_o));
    }
  else
    {
      _info("regaddr=%p, clear=0x%x\n", regaddr, (1 << reg_gpio_xx_o));////
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
