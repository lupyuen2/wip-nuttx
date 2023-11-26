/****************************************************************************
 * arch/risc-v/src/bl602/bl602_serial.c
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
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/tioctl.h>

#include "bl602_lowputc.h"
////#include "bl602_gpio.h"

#include "hardware/bl602_uart.h"
////#include "hardware/bl602_glb.h"
#include "riscv_internal.h"
////#include "bl602_config.h"
#include "chip.h"

////TODO
#define HAVE_SERIAL_CONSOLE 1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define BL602_CONSOLE_IDX    0
#define BL602_CONSOLE_BAUD   CONFIG_UART0_BAUD
#define BL602_CONSOLE_BITS   CONFIG_UART0_BITS
#define BL602_CONSOLE_PARITY CONFIG_UART0_PARITY
#define BL602_CONSOLE_2STOP  CONFIG_UART0_2STOP
#define BL602_CONSOLE_TX     GPIO_UART0_TX
#define BL602_CONSOLE_RX     GPIO_UART0_RX
#define HAVE_UART
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#define BL602_CONSOLE_IDX    1
#define BL602_CONSOLE_BAUD   CONFIG_UART1_BAUD
#define BL602_CONSOLE_BITS   CONFIG_UART1_BITS
#define BL602_CONSOLE_PARITY CONFIG_UART1_PARITY
#define BL602_CONSOLE_2STOP  CONFIG_UART1_2STOP
#define BL602_CONSOLE_TX     GPIO_UART1_TX
#define BL602_CONSOLE_RX     GPIO_UART1_RX
#define HAVE_UART
#endif
#endif /* HAVE_CONSOLE */

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#ifdef HAVE_SERIAL_CONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart0port /* UART0 is console */
#define TTYS0_DEV   g_uart0port /* UART0 is ttyS0 */
#undef TTYS1_DEV                /* No ttyS1 */
#define SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#define CONSOLE_DEV g_uart1port /* UART0 is console */
#error "I'm confused... Do we have a serial console or not?"
#endif
#else
#undef CONSOLE_DEV /* No console */
#undef CONFIG_UART0_SERIAL_CONSOLE
#if defined(CONFIG_BL602_UART0)
#define TTYS0_DEV g_uart0port /* UART0 is ttyS0 */
#undef TTYS1_DEV              /* No ttyS1 */
#define SERIAL_CONSOLE 1
#else
#undef TTYS0_DEV
#undef TTYS1_DEV
#endif
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of riscv_earlyserialinit(), riscv_serialinit(), and
 * up_putc().
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bl602_uart_s
{
  uint8_t              irq;      /* IRQ associated with this UART */
  struct uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Serial driver methods */

static int  bl602_setup(struct uart_dev_s *dev);
static void bl602_shutdown(struct uart_dev_s *dev);
static int  bl602_attach(struct uart_dev_s *dev);
static void bl602_detach(struct uart_dev_s *dev);
static int  bl602_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  bl602_receive(struct uart_dev_s *dev, unsigned int *status);
static void bl602_rxint(struct uart_dev_s *dev, bool enable);
static bool bl602_rxavailable(struct uart_dev_s *dev);
static void bl602_send(struct uart_dev_s *dev, int ch);
static void bl602_txint(struct uart_dev_s *dev, bool enable);
static bool bl602_txready(struct uart_dev_s *dev);
static bool bl602_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup       = bl602_setup,
  .shutdown    = bl602_shutdown,
  .attach      = bl602_attach,
  .detach      = bl602_detach,
  .ioctl       = bl602_ioctl,
  .receive     = bl602_receive,
  .rxint       = bl602_rxint,
  .rxavailable = bl602_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send    = bl602_send,
  .txint   = bl602_txint,
  .txready = bl602_txready,
  .txempty = bl602_txempty,
};

// UART3 Int = (IRQ_NUM_BASE + 4)
// IRQ_NUM_BASE = 16
// RISC-V IRQ = 20
// NuttX IRQ = 45 (Offset by 25)
#define BL602_IRQ_UART0 45 ////TODO

/* I/O buffers */

#ifdef CONFIG_BL602_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct bl602_uart_s g_uart0priv =
{
  .irq      = BL602_IRQ_UART0,
  .config =
    {
      .idx       = 0,
      .baud      = CONFIG_UART0_BAUD,
      .parity    = CONFIG_UART0_PARITY,
      .data_bits = CONFIG_UART0_BITS,
      .stop_bits = CONFIG_UART0_2STOP,

#ifdef CONFIG_UART0_IFLOWCONTROL
      .iflow_ctl = CONFIG_UART0_IFLOWCONTROL,
#else
      .iflow_ctl = 0,
#endif

#ifdef CONFIG_UART0_OFLOWCONTROL
      .oflow_ctl = CONFIG_UART0_OFLOWCONTROL,
#else
      .oflow_ctl = 0,
#endif
    },
};

static uart_dev_t g_uart0port =
{
#ifdef CONFIG_UART0_SERIAL_CONSOLE
  .isconsole = 1,
#endif
  .recv =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },
  .xmit =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },
  .ops  = &g_uart_ops,
  .priv = (void *)&g_uart0priv,
};
#endif

#ifdef CONFIG_BL602_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

static struct bl602_uart_s g_uart1priv =
{
  .irq      = BL602_IRQ_UART1,
  .config =
    {
      .idx       = 1,
      .baud      = CONFIG_UART1_BAUD,
      .parity    = CONFIG_UART1_PARITY,
      .data_bits = CONFIG_UART1_BITS,
      .stop_bits = CONFIG_UART1_2STOP,

#ifdef CONFIG_UART1_IFLOWCONTROL
      .iflow_ctl = CONFIG_UART1_IFLOWCONTROL,
#else
      .iflow_ctl = 0,
#endif

#ifdef CONFIG_UART1_OFLOWCONTROL
      .oflow_ctl = CONFIG_UART1_OFLOWCONTROL,
#else
      .oflow_ctl = 0,
#endif
    },
};

static uart_dev_t g_uart1port =
{
#ifdef CONFIG_UART1_SERIAL_CONSOLE
  .isconsole = 1,
#endif
  .recv =
    {
      .size   = CONFIG_UART1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },
  .xmit =
    {
      .size   = CONFIG_UART1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },
  .ops  = &g_uart_ops,
  .priv = (void *)&g_uart1priv,
};
#endif

static struct uart_dev_s *const g_uart_devs[] =
{
#ifdef CONFIG_BL602_UART0
  [0] = &g_uart0port,
#endif
#ifdef CONFIG_BL602_UART1
  [1] = &g_uart1port
#endif
};

/****************************************************************************
 * Name: uart_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int __uart_interrupt(int irq, void *context, void *arg)
{
  uart_dev_t *dev           = (uart_dev_t *)arg;
  struct bl602_uart_s *priv = dev->priv;
  uint8_t uart_idx          = priv->config.idx;
  uint32_t int_status;
  uint32_t int_mask;

  int_status = getreg32(BL602_UART_INT_STS(uart_idx));
  int_mask = getreg32(BL602_UART_INT_MASK(uart_idx));

  /* Length of uart rx data transfer arrived interrupt */

  if ((int_status & UART_INT_STS_URX_END_INT) &&
      !(int_mask & UART_INT_MASK_CR_URX_END_MASK))
    {
      putreg32(UART_INT_CLEAR_CR_URX_END_CLR,
               BL602_UART_INT_CLEAR(uart_idx));

      /* Receive Data ready */

      uart_recvchars(dev);
    }

  /* Tx fifo ready interrupt,auto-cleared when data is pushed */

  if ((int_status & UART_INT_STS_UTX_FIFO_INT) &&
      !(int_mask & UART_INT_MASK_CR_UTX_FIFO_MASK))
    {
      /* Transmit data request interrupt */

      uart_xmitchars(dev);
    }

  /* Rx fifo ready interrupt,auto-cleared when data is popped */

  if ((int_status & UART_INT_STS_URX_FIFO_INT) &&
      !(int_mask & UART_INT_MASK_CR_URX_FIFO_MASK))
    {
      /* Receive Data ready */

      uart_recvchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: bl602_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int bl602_setup(struct uart_dev_s *dev)
{
  struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;

  bl602_uart_configure(&priv->config);
  return OK;
}

/****************************************************************************
 * Name: bl602_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void bl602_shutdown(struct uart_dev_s *dev)
{
  struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;
  uint8_t uart_idx = priv->config.idx;

  /* Disable uart before config */

  modifyreg32(BL602_UART_UTX_CONFIG(uart_idx), UART_UTX_CONFIG_CR_EN, 0);
  modifyreg32(BL602_UART_URX_CONFIG(uart_idx), UART_URX_CONFIG_CR_EN, 0);
}

#include "riscv_mmu.h" ////TODO

/****************************************************************************
 * Name: bl602_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/

static int bl602_attach(struct uart_dev_s *dev)
{
  int                  ret;
  struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;

  ret = irq_attach(priv->irq, __uart_interrupt, (void *)dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  ////Begin
  _info("BL602_UART_INT_STS=0x%x\n", getreg32(BL602_UART_INT_STS(0)));
  _info("BL602_UART_INT_MASK=0x%x\n", getreg32(BL602_UART_INT_MASK(0)));
  _info("BL602_UART_INT_CLEAR=0x%x\n", getreg32(BL602_UART_INT_CLEAR(0)));
  _info("BL602_UART_INT_EN=0x%x\n", getreg32(BL602_UART_INT_EN(0)));

  // Clear RX FIFO
  _info("BL602_UART_FIFO_CONFIG_0=0x%x\n", getreg32(BL602_UART_FIFO_CONFIG_0(0)));
  putreg32(1 << 3, BL602_UART_FIFO_CONFIG_0(0));
  _info("BL602_UART_FIFO_CONFIG_0=0x%x\n", getreg32(BL602_UART_FIFO_CONFIG_0(0)));

  // Dump the UART and PLIC
  infodumpbuffer("UART Registers", 0x30002000, 0x36 * 4);
  infodumpbuffer("PLIC Interrupt Priority", 0xe0000004, 0x50 * 4);
  infodumpbuffer("PLIC Interrupt Pending", 0xe0001000, 2 * 4);
  infodumpbuffer("PLIC Hart 0 S-Mode Interrupt Enable", 0xe0002080, 2 * 4);
  infodumpbuffer("PLIC Hart 0 S-Mode Priority Threshold", 0xe0201000, 2 * 4);
  infodumpbuffer("PLIC Hart 0 S-Mode Claim / Complete", 0xe0201004, 1 * 4);
  infodumpbuffer("Interrupt Pending", 0xe0001000, 2 * 4);
  infodumpbuffer("PLIC Hart 0 M-Mode Interrupt Enable", 0xe0002000, 2 * 4);
  infodumpbuffer("PLIC Hart 0 M-Mode Priority Threshold", 0xe0200000, 2 * 4);
  infodumpbuffer("PLIC Hart 0 M-Mode Claim / Complete", 0xe0200004, 1 * 4);

  // Test Interrupt Priority
  _info("Test Interrupt Priority\n");
  void test_interrupt_priority(void);
  test_interrupt_priority();

  // Set PLIC Interrupt Priority to 1
  _info("Set PLIC Interrupt Priority to 1\n");
  putreg32(1, (uintptr_t)0xe0000050); // IRQ 20
  infodumpbuffer("PLIC Interrupt Priority", 0xe0000004, 0x50 * 4);
  infodumpbuffer("PLIC Interrupt Pending", 0xe0001000, 2 * 4);
  ////End
  return ret;
}

// Test the setting of PLIC Interrupt Priority
void test_interrupt_priority(void)
{
  // Read the values before setting Interrupt Priority
  uint32_t before50 = *(volatile uint32_t *) 0xe0000050UL;
  uint32_t before54 = *(volatile uint32_t *) 0xe0000054UL;

  // Set the Interrupt Priority
  // for 50 but NOT 54
  *(volatile uint32_t *) 0xe0000050UL = 1;

  // Read the values after setting Interrupt Priority
  uint32_t after50 = *(volatile uint32_t *) 0xe0000050UL;
  uint32_t after54 = *(volatile uint32_t *) 0xe0000054UL;

  // Dump before and after values:
  // before50=0 before54=0
  // after50=1  after54=1
  // Why after54=1 ???
  _info("before50=%u, before54=%u, after50=%u, after54=%u\n",
    before50, before54, after50, after54);
}

// Test in RISC-V Assembly the setting of PLIC Interrupt Priority
void test_interrupt_priority2(void)
{
  __asm__ __volatile__
    (  
      // Read the values before setting Interrupt Priority
      // A0 = *(volatile uint32_t *) 0xe0000050UL;
      // A1 = *(volatile uint32_t *) 0xe0000054UL;
      "li   t0, 0xe0000000\n"  // PLIC Base Address
      "lw   a0, 0x50(t0)\n"    // Read A0 from Offset 0x50
      "lw   a1, 0x54(t0)\n"    // Read A0 from Offset 0x54

      // Print the values
      "li   t0, 0x30002000\n"  // UART3 Base Address
      "addi t1, a0, 0x30\n"    // Add `0` to A0
      "sb   t1, 0x88(t0)\n"    // Write to UART TX
      "addi t1, a1, 0x30\n"    // Add `0` to A1
      "sb   t1, 0x88(t0)\n"    // Write to UART TX

      // Set the Interrupt Priority in RISC-V Assembly
      // *(volatile uint32_t *) 0xe0000050UL = 1;
      "li   t0, 0xe0000000\n"  // PLIC Base Address
      "li   t1, 0x01\n"        // Value 1
      "sw   t1, 0x50(t0)\n"    // Write to Offset 0x50

      // Read the values after setting Interrupt Priority
      // A0 = *(volatile uint32_t *) 0xe0000050UL;
      // A0 = *(volatile uint32_t *) 0xe0000054UL;
      "li   t0, 0xe0000000\n"  // PLIC Base Address
      "lw   a0, 0x50(t0)\n"    // Read A0 from Offset 0x50
      "lw   a1, 0x54(t0)\n"    // Read A0 from Offset 0x54

      // Print the values
      "li   t0, 0x30002000\n"  // UART3 Base Address
      "addi t1, a0, 0x30\n"    // Add `0` to A0
      "sb   t1, 0x88(t0)\n"    // Write to UART TX
      "addi t1, a1, 0x30\n"    // Add `0` to A1
      "sb   t1, 0x88(t0)\n"    // Write to UART TX

      ::: "memory"
    );  
    // Prints [before1][before2][after1][after2]: 0011
}

/****************************************************************************
 * Name: bl602_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void bl602_detach(struct uart_dev_s *dev)
{
  struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Detach from the interrupt */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: bl602_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int bl602_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode *     inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int ret = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      do
        {
          struct termios * termiosp = (struct termios *)arg;
          struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }
          termiosp->c_cflag = 0;

          /* Return parity */

          termiosp->c_cflag = ((priv->config.parity != 0) ? PARENB : 0) |
                              ((priv->config.parity == 1) ? PARODD : 0);

          /* Return stop bits */

          termiosp->c_cflag |= (priv->config.stop_bits) ? CSTOPB : 0;

          /* Return flow control */

          termiosp->c_cflag |= (priv->config.iflow_ctl) ? CRTS_IFLOW : 0;
          termiosp->c_cflag |= (priv->config.oflow_ctl) ? CCTS_OFLOW : 0;

          /* Return baud */

          cfsetispeed(termiosp, priv->config.baud);

          /* Return number of bits */

          switch (priv->config.data_bits)
            {
            case 5:
              termiosp->c_cflag |= CS5;
              break;

            case 6:
              termiosp->c_cflag |= CS6;
              break;

            case 7:
              termiosp->c_cflag |= CS7;
              break;

            default:
            case 8:
              termiosp->c_cflag |= CS8;
              break;
            }
        }
      while (0);
      break;

    case TCSETS:
      do
        {
          struct termios *     termiosp = (struct termios *)arg;
          struct bl602_uart_s *    priv = (struct bl602_uart_s *)dev->priv;
          struct uart_config_s config;
          uint32_t             tmp_val;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* Decode baud. */

          ret         = OK;
          config.baud = cfgetispeed(termiosp);

          /* Decode number of bits */

          switch (termiosp->c_cflag & CSIZE)
            {
            case CS5:
              config.data_bits = 5;
              break;

            case CS6:
              config.data_bits = 6;
              break;

            case CS7:
              config.data_bits = 7;
              break;

            case CS8:
              config.data_bits = 8;
              break;

            default:
              ret = -EINVAL;
              break;
            }

          /* Decode parity */

          if ((termiosp->c_cflag & PARENB) != 0)
            {
              config.parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
            }
          else
            {
              config.parity = 0;
            }

          /* Decode stop bits */

          config.stop_bits = (termiosp->c_cflag & CSTOPB) != 0;

          /* Decode flow control */

          if (priv->config.idx == 0)
            {
#ifdef CONFIG_UART0_IFLOWCONTROL
              config.iflow_ctl = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
#ifdef CONFIG_UART0_OFLOWCONTROL
              config.oflow_ctl = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
            }
          else
            {
#ifdef CONFIG_UART1_IFLOWCONTROL
              config.iflow_ctl = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
#ifdef CONFIG_UART1_OFLOWCONTROL
              config.oflow_ctl = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
            }

          /* Verify that all settings are valid before committing */

          if (ret == OK)
            {
              /* Commit */

              memcpy(&priv->config, &config, sizeof(config));

              /* effect the changes immediately - note that we do not
               * implement TCSADRAIN / TCSAFLUSH
               */

              tmp_val = getreg32(BL602_UART_INT_MASK(config.idx));
              bl602_uart_configure(&config);
              putreg32(tmp_val, BL602_UART_INT_MASK(config.idx));
            }
        }
      while (0);
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: bl602_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int bl602_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;
  uint8_t uart_idx = priv->config.idx;
  int rxdata;

  /* Return status information */

  if (status)
    {
      *status = 0; /* We are not yet tracking serial errors */
    }

  /* if uart fifo cnts > 0 */

  if (getreg32(BL602_UART_FIFO_CONFIG_1(uart_idx)) & \
      UART_FIFO_CONFIG_1_RX_CNT_MASK)
    {
      rxdata = getreg32(BL602_UART_FIFO_RDATA(uart_idx)) & \
        UART_FIFO_RDATA_MASK;
      _info("rxdata=0x%x\n", rxdata);////
    }
  else
    {
      rxdata = -1;
      _info("rxdata=-1\n");////
      ////TODO
      rxdata = getreg32(BL602_UART_FIFO_RDATA(uart_idx)) & \
        UART_FIFO_RDATA_MASK;
      _info("rxdata=0x%x\n", rxdata);////
    }
  return rxdata;
}

/****************************************************************************
 * Name: bl602_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void bl602_rxint(struct uart_dev_s *dev, bool enable)
{
  uint32_t int_mask;
  struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;
  uint8_t uart_idx = priv->config.idx;
  irqstate_t       flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      int_mask = getreg32(BL602_UART_INT_MASK(uart_idx));
      int_mask &= ~(UART_INT_MASK_CR_URX_FIFO_MASK);
      int_mask &= ~(UART_INT_MASK_CR_URX_END_MASK);
      putreg32(int_mask, BL602_UART_INT_MASK(uart_idx));
#endif
    }
  else
    {
      int_mask = getreg32(BL602_UART_INT_MASK(uart_idx));
      int_mask |= UART_INT_MASK_CR_URX_FIFO_MASK;
      int_mask |= UART_INT_MASK_CR_URX_END_MASK;
      putreg32(int_mask, BL602_UART_INT_MASK(uart_idx));
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bl602_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool bl602_rxavailable(struct uart_dev_s *dev)
{
  struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;
  uint8_t uart_idx          = priv->config.idx;

  /* Return true is data is available in the receive data buffer */

  return (getreg32(BL602_UART_FIFO_CONFIG_1(uart_idx)) & \
          UART_FIFO_CONFIG_1_RX_CNT_MASK) != 0;
}

/****************************************************************************
 * Name: bl602_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void bl602_send(struct uart_dev_s *dev, int ch)
{
  struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;
  uint8_t uart_idx          = priv->config.idx;

  /* Wait for FIFO to be empty */

  while ((getreg32(BL602_UART_FIFO_CONFIG_1(uart_idx)) & \
         UART_FIFO_CONFIG_1_TX_CNT_MASK) == 0);

  putreg32(ch, BL602_UART_FIFO_WDATA(uart_idx));
}

/****************************************************************************
 * Name: bl602_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void bl602_txint(struct uart_dev_s *dev, bool enable)
{
  struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;
  uint8_t uart_idx          = priv->config.idx;
  irqstate_t       flags;
  uint32_t         int_mask;

  flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable the TX interrupt */

      int_mask = getreg32(BL602_UART_INT_MASK(uart_idx));
      int_mask &= ~(UART_INT_MASK_CR_UTX_FIFO_MASK);
      putreg32(int_mask, BL602_UART_INT_MASK(uart_idx));

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      int_mask = getreg32(BL602_UART_INT_MASK(uart_idx));
      int_mask |= UART_INT_MASK_CR_UTX_FIFO_MASK;
      putreg32(int_mask, BL602_UART_INT_MASK(uart_idx));
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bl602_txready
 *
 * Description:
 *   Return true if the transmit data register is not full
 *
 ****************************************************************************/

static bool bl602_txready(struct uart_dev_s *dev)
{
  struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;
  uint8_t uart_idx          = priv->config.idx;

  /* Return TRUE if the TX FIFO is not full */

  return (getreg32(BL602_UART_FIFO_CONFIG_1(uart_idx)) & \
          UART_FIFO_CONFIG_1_TX_CNT_MASK) != 0;
}

/****************************************************************************
 * Name: bl602_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool bl602_txempty(struct uart_dev_s *dev)
{
  struct bl602_uart_s *priv = (struct bl602_uart_s *)dev->priv;
  uint8_t uart_idx          = priv->config.idx;

  return (getreg32(BL602_UART_FIFO_CONFIG_1(uart_idx)) & \
          UART_FIFO_CONFIG_1_TX_CNT_MASK) == 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void bl602_earlyserialinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Configuration whichever one is the console */

  CONSOLE_DEV.isconsole = true;
  bl602_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: bl602_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void bl602_serialinit(void)
{
  int  i;
  char devname[16];

#ifdef HAVE_SERIAL_CONSOLE
  /* Register the console */

  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  strlcpy(devname, "/dev/ttySx", sizeof(devname));
  for (i = 0; i < sizeof(g_uart_devs) / sizeof(g_uart_devs[0]); i++)
    {
      if (g_uart_devs[i] == 0)
        {
          continue;
        }

      /* Don't create a device for the console - we did that above */

      if (g_uart_devs[i]->isconsole)
        {
          continue;
        }

      /* Register USARTs as devices in increasing order */

      devname[9] = '0' + i;
      uart_register(devname, g_uart_devs[i]);
    }
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
int up_putc(int ch)
{
  struct bl602_uart_s *priv = (struct bl602_uart_s *)CONSOLE_DEV.priv;
  irqstate_t flags = enter_critical_section();

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      bl602_send(priv, '\r');
    }

  bl602_send(priv, ch);
  leave_critical_section(flags);
  return ch;
}
#endif

////TODO
void bl602_uart_configure(const struct uart_config_s *config) {}
