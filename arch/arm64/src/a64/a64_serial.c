/***************************************************************************
 * arch/arm64/src/a64/a64_serial.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h> ////TODO

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "a64_pio.h"
#include "a64_serial.h"
#include "arm64_arch_timer.h"
#include "a64_boot.h"
#include "arm64_gic.h"

/* UART0 Settings should be same as U-Boot Bootloader */

#ifndef CONFIG_UART0_BAUD
#  define CONFIG_UART0_BAUD 115200
#endif

#ifndef CONFIG_UART0_BITS
#  define CONFIG_UART0_BITS 8
#endif

#ifndef CONFIG_UART0_PARITY
#  define CONFIG_UART0_PARITY 0
#endif

#ifndef CONFIG_UART0_2STOP
#  define CONFIG_UART0_2STOP 0
#endif

#ifndef CONFIG_UART0_RXBUFSIZE
#  define CONFIG_UART0_RXBUFSIZE 256
#endif

#ifndef CONFIG_UART0_TXBUFSIZE
#  define CONFIG_UART0_TXBUFSIZE 256
#endif

//// TODO
#define CONFIG_A64_UART3
#define CONFIG_UART3_RXBUFSIZE 256
#define CONFIG_UART3_TXBUFSIZE 256
#define CONFIG_UART3_BAUD 115200
#define CONFIG_UART3_BITS 8
#define CONFIG_UART3_PARITY 0
#define CONFIG_UART3_2STOP 0
#define CONFIG_A64_UART3_BASE      0x01C28C00  /* A64 UART3 Base Address */
#define CONFIG_A64_UART3_IRQ       35          /* A64 UART3 IRQ */

#ifdef USE_SERIALDRIVER

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* UART0 is console and ttys0, follows U-Boot Bootloader */

#define CONSOLE_DEV     g_uart0port         /* UART0 is console */
#define TTYS0_DEV       g_uart0port         /* UART0 is ttyS0 */
#define UART0_ASSIGNED  1

/* A64 UART Registers */

#define UART_THR(uart_addr) (uart_addr + 0x00)  /* Tx Holding */
#define UART_RBR(uart_addr) (uart_addr + 0x00)  /* Rx Buffer */
#define UART_IER(uart_addr) (uart_addr + 0x04)  /* Interrupt Enable */
#define UART_IIR(uart_addr) (uart_addr + 0x08)  /* Interrupt Identity */
#define UART_LSR(uart_addr) (uart_addr + 0x14)  /* Line Status */

/* A64 UART Register Bit Definitions */

#define UART_IER_ERBFI (1 << 0)  /* Enable Rx Data Interrupt */
#define UART_IER_ETBEI (1 << 1)  /* Enable Tx Empty Interrupt */
#define UART_IIR_INTID (0b1111)  /* Interrupt ID */
#define UART_LSR_DR    (1 << 0)  /* Rx Data Ready */
#define UART_LSR_THRE  (1 << 5)  /* Tx Empty */

/***************************************************************************
 * Private Types
 ***************************************************************************/

// struct up_dev_s
// {
//   uint32_t uartbase;  /* Base address of UART registers */
//   uint32_t baud;      /* Configured baud */
//   uint32_t ier;       /* Saved IER value */
//   uint8_t  irq;       /* IRQ associated with this UART */
//   uint8_t  parity;    /* 0=none, 1=odd, 2=even */
//   uint8_t  bits;      /* Number of bits (7 or 8) */
//   bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
// };

/* A64 UART Configuration */

struct a64_uart_config
{
  unsigned long uart;  /* UART Base Address */
};

/* A64 UART Device Data */

struct a64_uart_data
{
  uint32_t baud_rate;  /* UART Baud Rate */
  uint32_t ier;        /* Saved IER value */
  uint8_t  parity;     /* 0=none, 1=odd, 2=even */
  uint8_t  bits;       /* Number of bits (7 or 8) */
  bool     stopbits2;  /* true: Configure with 2 stop bits instead of 1 */
};

/* A64 UART Port */

struct a64_uart_port_s
{
  struct a64_uart_data data;     /* UART Device Data */
  struct a64_uart_config config; /* UART Configuration */
  unsigned int irq_num;          /* UART IRQ Number */
  bool is_console;               /* 1 if this UART is console */
};

/***************************************************************************
 * Private Functions
 ***************************************************************************/

#ifdef NOTUSED //// TODO
/***************************************************************************
 * Name: a64_uart_irq_handler
 *
 * Description:
 *   This is the common UART interrupt handler.  It should call
 *   uart_xmitchars or uart_recvchars to perform the appropriate data
 *   transfers.
 *
 * Input Parameters:
 *   irq     - IRQ Number
 *   context - Interrupt Context
 *   arg     - UART Device
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ***************************************************************************/
#endif  // NOTUSED

//// TODO

/* Register offsets *********************************************************/

#define A1X_UART_RBR_OFFSET       0x0000 /* UART Receive Buffer Register */
#define A1X_UART_THR_OFFSET       0x0000 /* UART Transmit Holding Register */
#define A1X_UART_DLL_OFFSET       0x0000 /* UART Divisor Latch Low Register */
#define A1X_UART_DLH_OFFSET       0x0004 /* UART Divisor Latch High Register */
#define A1X_UART_IER_OFFSET       0x0004 /* UART Interrupt Enable Register */
#define A1X_UART_IIR_OFFSET       0x0008 /* UART Interrupt Identity Register */
#define A1X_UART_FCR_OFFSET       0x0008 /* UART FIFO Control Register */
#define A1X_UART_LCR_OFFSET       0x000c /* UART Line Control Register */
#define A1X_UART_MCR_OFFSET       0x0010 /* UART Modem Control Register */
#define A1X_UART_LSR_OFFSET       0x0014 /* UART Line Status Register */
#define A1X_UART_MSR_OFFSET       0x0018 /* UART Modem Status Register */
#define A1X_UART_SCH_OFFSET       0x001c /* UART Scratch Register */
#define A1X_UART_USR_OFFSET       0x007c /* UART Status Register */
#define A1X_UART_TFL_OFFSET       0x0080 /* UART Transmit FIFO Level */
#define A1X_UART_RFL_OFFSET       0x0084 /* UART Receive FIFO Level */
#define A1X_UART_HALT_OFFSET      0x00a4 /* UART Halt TX Register */

/* UART Interrupt Identity Register */

#define UART_IIR_IID_SHIFT        (0) /* Bits: 0-3: Interrupt ID */
#define UART_IIR_IID_MASK         (15 << UART_IIR_IID_SHIFT)
#  define UART_IIR_IID_MODEM      (0 << UART_IIR_IID_SHIFT)  /* Modem status */
#  define UART_IIR_IID_NONE       (1 << UART_IIR_IID_SHIFT)  /* No interrupt pending */
#  define UART_IIR_IID_TXEMPTY    (2 << UART_IIR_IID_SHIFT)  /* THR empty */
#  define UART_IIR_IID_RECV       (4 << UART_IIR_IID_SHIFT)  /* Received data available */
#  define UART_IIR_IID_LINESTATUS (6 << UART_IIR_IID_SHIFT)  /* Receiver line status */
#  define UART_IIR_IID_BUSY       (7 << UART_IIR_IID_SHIFT)  /* Busy detect */
#  define UART_IIR_IID_TIMEOUT    (12 << UART_IIR_IID_SHIFT) /* Character timeout */

#define UART_IIR_FEFLAG_SHIFT     (6) /* Bits 6-7: FIFOs Enable Flag */
#define UART_IIR_FEFLAG_MASK      (3 << UART_IIR_FEFLAG_SHIFT)
#  define UART_IIR_FEFLAG_DISABLE (0 << UART_IIR_FEFLAG_SHIFT)
#  define UART_IIR_FEFLAG_ENABLE  (3 << UART_IIR_FEFLAG_SHIFT)

/* UART FIFO Control Register */

#define UART_FCR_FIFOE            (1 << 0)  /* Bit 0:  Enable FIFOs */
#define UART_FCR_RFIFOR           (1 << 1)  /* Bit 1:  RCVR FIFO Reset */
#define UART_FCR_XFIFOR           (1 << 2)  /* Bit 2:  XMIT FIFO reset */
#define UART_FCR_DMAM             (1 << 3)  /* Bit 3:  DMA mode */
#define UART_FCR_TFT_SHIFT        (4)       /* Bits 4-5: TX Empty Trigger */
#define UART_FCR_TFT_MASK         (3 << UART_FCR_TFT_SHIFT)
#  define UART_FCR_TFT_EMPTY      (0 << UART_FCR_TFT_SHIFT) /* FIFO empty */
#  define UART_FCR_TFT_TWO        (1 << UART_FCR_TFT_SHIFT) /* 2 characters in the FIFO */
#  define UART_FCR_TFT_QUARTER    (2 << UART_FCR_TFT_SHIFT) /* FIFO 1/4 full */
#  define UART_FCR_TFT_HALF       (3 << UART_FCR_TFT_SHIFT) /* FIFO 1/2 full */

#define UART_FCR_RT_SHIFT         (6)       /* Bits 6-7: RCVR Trigger */
#define UART_FCR_RT_MASK          (3 << UART_FCR_RT_SHIFT)
#  define UART_FCR_RT_ONE         (0 << UART_FCR_RT_SHIFT) /* 1 character in the FIFO */
#  define UART_FCR_RT_QUARTER     (1 << UART_FCR_RT_SHIFT) /* FIFO 1/4 full */
#  define UART_FCR_RT_HALF        (2 << UART_FCR_RT_SHIFT) /* FIFO 1/2 full */
#  define UART_FCR_RT_MINUS2      (3 << UART_FCR_RT_SHIFT) /* FIFO-2 less than full */

/* UART Line Control Register */

#define UART_LCR_DLS_SHIFT        (0)       /* Bits 0-1: Data Length Select */
#define UART_LCR_DLS_MASK         (3 << UART_LCR_DLS_SHIFT)
#  define UART_LCR_DLS_5BITS      (0 << UART_LCR_DLS_SHIFT) /* 5 bits */
#  define UART_LCR_DLS_6BITS      (1 << UART_LCR_DLS_SHIFT) /* 6 bits */
#  define UART_LCR_DLS_7BITS      (2 << UART_LCR_DLS_SHIFT) /* 7 bits */
#  define UART_LCR_DLS_8BITS      (3 << UART_LCR_DLS_SHIFT) /* 8 bits */

#define UART_LCR_STOP             (1 << 2)  /* Bit 2:  Number of stop bits */
#define UART_LCR_PEN              (1 << 3)  /* Bit 3:  Parity Enable */
#define UART_LCR_EPS              (1 << 4)  /* Bit 4:  Even Parity Select */
#define UART_LCR_BC               (1 << 6)  /* Bit 6:  Break Control Bit */
#define UART_LCR_DLAB             (1 << 7)  /* Bit 7:  Divisor Latch Access Bit */

/* SCLK is the UART input clock.
 *
 * Through experimentation, it has been found that the serial clock is
 * OSC24M
 */

#define A1X_SCLK 24000000

/****************************************************************************
 * Name: a1x_uartdl
 *
 * Description:
 *   Select a divider to produce the BAUD from the UART PCLK.
 *
 *     BAUD = PCLK / (16 * DL), or
 *     DL   = PCLK / BAUD / 16
 *
 ****************************************************************************/

static inline uint32_t a64_uartdl(uint32_t baud)
{
  return A1X_SCLK / (baud << 4);
}

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(const struct a64_uart_config *config, int offset)
{
  return getreg32(config->uart + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(const struct a64_uart_config *config, int offset,
                                uint32_t value)
{
  putreg32(value, config->uart + offset);
}

/****************************************************************************
 * Name: a64_uart_irq_handler
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int a64_uart_irq_handler(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;
  uint32_t status;
  int passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status */

      status = up_serialin(config, A1X_UART_IIR_OFFSET);

      /* Handle the interrupt by its interrupt ID field */

      switch (status & UART_IIR_IID_MASK)
        {
          /* Handle incoming, receive bytes (with or without timeout) */

          case UART_IIR_IID_RECV:
          case UART_IIR_IID_TIMEOUT:
            {
              uart_recvchars(dev);
              break;
            }

          /* Handle outgoing, transmit bytes */

          case UART_IIR_IID_TXEMPTY:
            {
              uart_xmitchars(dev);
              break;
            }

          /* Just clear modem status interrupts (UART1 only) */

          case UART_IIR_IID_MODEM:
            {
              /* Read the modem status register (MSR) to clear */

              status = up_serialin(config, A1X_UART_MSR_OFFSET);
              _info("MSR: %02" PRIx32 "\n", status);
              break;
            }

          /* Just clear any line status interrupts */

          case UART_IIR_IID_LINESTATUS:
            {
              /* Read the line status register (LSR) to clear */

              status = up_serialin(config, A1X_UART_LSR_OFFSET);
              _info("LSR: %02" PRIx32 "\n", status);
              break;
            }

          /* Busy detect.
           * Just ignore.
           * Cleared by reading the status register
           */

          case UART_IIR_IID_BUSY:
            {
              /* Read from the UART status register
               * to clear the BUSY condition
               */

              status = up_serialin(config, A1X_UART_USR_OFFSET);
              break;
            }

          /* No further interrupts pending... return now */

          case UART_IIR_IID_NONE:
            {
              return OK;
            }

            /* Otherwise we have received an interrupt
             * that we cannot handle
             */

          default:
            {
              _err("ERROR: Unexpected IIR: %02" PRIx32 "\n", status);
              break;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;
  struct a64_uart_data *data = &port->data;
  uint16_t dl;
  uint32_t lcr;

  DEBUGASSERT(data != NULL);
  // _info("baud_rate=%d\n", data->baud_rate);////

  /* Clear fifos */

  // _info("Clear fifos"); ////
  up_serialout(config, A1X_UART_FCR_OFFSET,
              (UART_FCR_RFIFOR | UART_FCR_XFIFOR));

  /* Set trigger */

  // _info("Set trigger"); ////
  up_serialout(config, A1X_UART_FCR_OFFSET,
              (UART_FCR_FIFOE | UART_FCR_RT_HALF));

  /* Set up the IER */

  // _info("Set up the IER"); ////
  data->ier = up_serialin(config, A1X_UART_IER_OFFSET);

  /* Set up the LCR */

  lcr = 0;

  switch (data->bits)
    {
    case 5:
      lcr |= UART_LCR_DLS_5BITS;
      break;

    case 6:
      lcr |= UART_LCR_DLS_6BITS;
      break;

    case 7:
      lcr |= UART_LCR_DLS_7BITS;
      break;

    case 8:
    default:
      lcr |= UART_LCR_DLS_8BITS;
      break;
    }

  if (data->stopbits2)
    {
      lcr |= UART_LCR_STOP;
    }

  if (data->parity == 1)
    {
      lcr |= UART_LCR_PEN;
    }
  else if (data->parity == 2)
    {
      lcr |= (UART_LCR_PEN | UART_LCR_EPS);
    }

  /* Enter DLAB=1 */

  // _info("Enter DLAB=1 and set BAUD divisor"); ////

  // Wait for UART not busy: UART_USR[0] must be zero
  for (;;)
    {
      uint32_t status = getreg32(config->uart + A1X_UART_USR_OFFSET); ////
      if ((status & 1) == 0) { break; }
    }
  
  up_serialout(config, A1X_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

  // Wait for UART not busy: UART_USR[0] must be zero
  for (;;)
    {
      uint32_t status = getreg32(config->uart + A1X_UART_USR_OFFSET); ////
      if ((status & 1) == 0) { break; }
    }
  
  /* Set the BAUD divisor */

  uint32_t before0 = getreg32(config->uart + A1X_UART_DLH_OFFSET); ////
  uint32_t before1 = getreg32(config->uart + A1X_UART_DLL_OFFSET); ////

  dl = a64_uartdl(data->baud_rate);
  up_serialout(config, A1X_UART_DLH_OFFSET, dl >> 8);
  up_serialout(config, A1X_UART_DLL_OFFSET, dl & 0xff);

  uint32_t after0 = getreg32(config->uart + A1X_UART_DLH_OFFSET); ////
  uint32_t after1 = getreg32(config->uart + A1X_UART_DLL_OFFSET); ////

  /* Clear DLAB */

  up_serialout(config, A1X_UART_LCR_OFFSET, lcr);
  _info("Clear DLAB"); ////

  _info("addr=0x%x, before=0x%x, after=0x%x\n", config->uart + A1X_UART_DLH_OFFSET, before0, after0); ////
  _info("addr=0x%x, before=0x%x, after=0x%x\n", config->uart + A1X_UART_DLL_OFFSET, before1, after1); ////

  /* Configure the FIFOs */

  _info("Configure the FIFOs"); ////
  up_serialout(config, A1X_UART_FCR_OFFSET,
               (UART_FCR_RT_HALF | UART_FCR_XFIFOR | UART_FCR_RFIFOR |
                UART_FCR_FIFOE));

  /* Enable Auto-Flow Control in the Modem Control Register */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
#  warning Missing logic
#endif

#endif
  return OK;
}

/***************************************************************************
 * Name: a64_uart_setup
 *
 * Description:
 *   Set up the UART Port.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ***************************************************************************/

static int a64_uart_setup(struct uart_dev_s *dev)
{
  int ret = up_setup(dev);
  DEBUGASSERT(ret == OK);

  return ret;
}

/***************************************************************************
 * Name: a64_uart_shutdown
 *
 * Description:
 *   Disable the UART Port.  This method is called when the serial
 *   port is closed.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void a64_uart_shutdown(struct uart_dev_s *dev)
{
  /* Should never be called */

  UNUSED(dev);
  sinfo("%s: call unexpected\n", __func__);
}

/***************************************************************************
 * Name: a64_uart_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.
 *   This method is called when the serial port is opened.
 *   Normally, this is just after the setup() method is called,
 *   however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method
 *   (unless the hardware supports multiple levels of interrupt
 *   enabling).  The RX and TX interrupts are not enabled until
 *   the txint() and rxint() methods are called.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int a64_uart_attach(struct uart_dev_s *dev)
{
  int ret;
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;

  DEBUGASSERT(port != NULL);

  /* Attach UART Interrupt Handler */

  ret = irq_attach(port->irq_num, a64_uart_irq_handler, dev);

  /* Set Interrupt Priority in Generic Interrupt Controller v2 */

  arm64_gic_irq_set_priority(port->irq_num, IRQ_TYPE_LEVEL, 0);

  /* Enable UART Interrupt */

  if (ret == OK)
    {
      up_enable_irq(port->irq_num);
    }
  else
    {
      sinfo("error ret=%d\n", ret);
    }

  return ret;
}

/***************************************************************************
 * Name: a64_uart_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void a64_uart_detach(struct uart_dev_s *dev)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;

  DEBUGASSERT(port != NULL);

  /* Disable UART Interrupt */

  up_disable_irq(port->irq_num);

  /* Detach UART Interrupt Handler */

  irq_detach(port->irq_num);
}

/***************************************************************************
 * Name: a64_uart_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   filep - File Struct
 *   cmd   - ioctl Command
 *   arg   - ioctl Argument
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int a64_uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  UNUSED(filep);
  UNUSED(arg);

  switch (cmd)
    {
      case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/***************************************************************************
 * Name: a64_uart_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 * Input Parameters:
 *   dev    - UART Device
 *   status - Return status, zero on success
 *
 * Returned Value:
 *   Received character
 *
 ***************************************************************************/

static int a64_uart_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;
  uint32_t rbr;

  *status = up_serialin(config, A1X_UART_LSR_OFFSET);
  rbr     = up_serialin(config, A1X_UART_RBR_OFFSET);
  return rbr;
}

#ifdef NOTUSED ////
static int a64_uart_receive(struct uart_dev_s *dev, unsigned int *status)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;

  /* Status is always OK */

  *status = 0;

  /* Read received char from Receiver Buffer Register (UART_RBR) */

  return getreg8(UART_RBR(config->uart));
}
#endif  // NOTUSED

/***************************************************************************
 * Name: a64_uart_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 * Input Parameters:
 *   dev    - UART Device
 *   enable - True to enable RX interrupts; false to disable
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void a64_uart_rxint(struct uart_dev_s *dev, bool enable)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;

  /* Write to Interrupt Enable Register (UART_IER) */

  if (enable)
    {
      /* Set ERBFI bit (Enable Rx Data Available Interrupt) */

      modreg8(UART_IER_ERBFI, UART_IER_ERBFI, UART_IER(config->uart));
    }
  else
    {
      /* Clear ERBFI bit (Disable Rx Data Available Interrupt) */

      modreg8(0, UART_IER_ERBFI, UART_IER(config->uart));
    }
}

/***************************************************************************
 * Name: a64_uart_rxavailable
 *
 * Description:
 *   Return true if the Receive FIFO is not empty
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Receive FIFO is not empty; false otherwise
 *
 ***************************************************************************/

static bool a64_uart_rxavailable(struct uart_dev_s *dev)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;

  /* Data Ready Bit (Line Status Register) is 1 if Rx Data is ready */

  return getreg8(UART_LSR(config->uart)) & UART_LSR_DR;
}

/***************************************************************************
 * Name: a64_uart_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 * Input Parameters:
 *   dev - UART Device
 *   ch  - Character to be sent
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void a64_uart_send(struct uart_dev_s *dev, int ch)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;

////#define BUFFER_UART
#ifdef BUFFER_UART
  //// TODO: Fix the garbled output. Buffer the chars until we see CR or LF.
  static char buf[256];
  static int pos = 0;
  if (ch != '>' && ch != '\r' && ch != '\n' && pos < sizeof(buf))
    {
      buf[pos] = ch;
      pos += 1;
      return;
    }
  for (int i = 0; i < pos; i++)
    {
      up_putc(buf[i]);      
    }
  pos = 0;
  up_putc(ch);
  UNUSED(config);
#else
  /* Write char to Transmit Holding Register (UART_THR) */

  putreg8(ch, UART_THR(config->uart));
#endif // BUFFER_UART
}

/***************************************************************************
 * Name: a64_uart_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 * Input Parameters:
 *   dev    - UART Device
 *   enable - True to enable TX interrupts; false to disable
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void a64_uart_txint(struct uart_dev_s *dev, bool enable)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;

  /* Write to Interrupt Enable Register (UART_IER) */

  if (enable)
    {
      /* Set ETBEI bit (Enable Tx Holding Register Empty Interrupt) */

      modreg8(UART_IER_ETBEI, UART_IER_ETBEI, UART_IER(config->uart));
    }
  else
    {
      /* Clear ETBEI bit (Disable Tx Holding Register Empty Interrupt) */

      modreg8(0, UART_IER_ETBEI, UART_IER(config->uart));
    }
}

/***************************************************************************
 * Name: a64_uart_txready
 *
 * Description:
 *   Return true if the Transmit FIFO is not full
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Transmit FIFO is not full; false otherwise
 *
 ***************************************************************************/

static bool a64_uart_txready(struct uart_dev_s *dev)
{
  const struct a64_uart_port_s *port = (struct a64_uart_port_s *)dev->priv;
  const struct a64_uart_config *config = &port->config;

  /* Tx FIFO is ready if THRE Bit is 1 (Tx Holding Register Empty) */

  return (getreg8(UART_LSR(config->uart)) & UART_LSR_THRE) != 0;
}

/***************************************************************************
 * Name: a64_uart_txempty
 *
 * Description:
 *   Return true if the Transmit FIFO is empty
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Transmit FIFO is empty; false otherwise
 *
 ***************************************************************************/

static bool a64_uart_txempty(struct uart_dev_s *dev)
{
  /* Tx FIFO is empty if Tx FIFO is not full (for now) */

  return a64_uart_txready(dev);
}

#ifdef CONFIG_A64_UART1
// TODO
static int a64_uart1config(void)
{
  irqstate_t flags;
  int ret;

  /* Step 1: Enable power to UART1 */

  flags   = enter_critical_section();
#warning Missing logic

  /* Step 2: Enable clocking to UART1 */
#warning Missing logic

  /* Step 3: Configure I/O pins */

  a64_pio_config(PIO_UART1_TX);
  a64_pio_config(PIO_UART1_RX);
  leave_critical_section(flags);
  return OK;
};
#endif /* CONFIG_A64_UART1 */

#ifdef CONFIG_A64_UART2
// TODO
static int a64_uart2config(void)
{
  irqstate_t flags;
  int ret;

  /* Step 1: Enable power to UART2 */

  flags   = enter_critical_section();
#warning Missing logic

  /* Step 2: Enable clocking on UART2 */
#warning Missing logic

  /* Step 3: Configure I/O pins */

  a64_pio_config(PIO_UART2_TX);
  a64_pio_config(PIO_UART2_RX);
  leave_critical_section(flags);
  return OK;
};
#endif /* CONFIG_A64_UART2 */

// Enable clocking to UART3: Set UART3_GATING to High (Pass)
// CCU Base Address: 0x01C20000
// Bus Clock Gating Register3
// Offset: 0x006C, Register Name: BUS_CLK_GATING_REG3
// Bit 19
// A64 User Manual Page 105
#define CCU_BASE_ADDR 0x01C20000
#define BUS_CLK_GATING_REG3 (CCU_BASE_ADDR + 0x006C)
#define UART3_GATING (1 << 19)

// Compare with UART0_GATING (Bit 16)
#define UART0_GATING (1 << 16)

// Deassert reset for UART3: Set UART3_RST to High
// CCU Base Address: 0x01C20000
// Bus Software Reset Register 4
// Offset: 0x02D8, Register Name: BUS_SOFT_RST_REG4
// Bit 19
// A64 User Manual Page 142
#define BUS_SOFT_RST_REG4 (CCU_BASE_ADDR + 0x02D8)
#define UART3_RST (1 << 19)

// Compare with UART0_RST (Bit 16)
#define UART0_RST (1 << 16)

// PIO Base Address: 0x01C20800
// PD Configure Register 0
// Offset: 0x6C, Register Name: PD_CFG0_REG
// Bits 0 to 2: PD0_SELECT
// Bits 4 to 6: PD1_SELECT
// A64 User Manual Page 385
#define PIO_BASE_ADDR 0x01C20800
#define PD_CFG0_REG (PIO_BASE_ADDR + 0x6C)
#define PD0_SELECT (0b111 << 0)
#define PD1_SELECT (0b111 << 4)

// Enable UART3 on PD0 and PD1: PD0_SELECT and PD1_SELECT
#define PIO_UART3_TX  (PIO_PERIPH3 | PIO_PORT_PIOD | PIO_PIN0)
#define PIO_UART3_RX  (PIO_PERIPH3 | PIO_PORT_PIOD | PIO_PIN1)

#ifdef CONFIG_A64_UART3
// TODO
static int a64_uart3config(void)
{
  irqstate_t flags;
  int ret;

  /* Step 1: Enable power to UART3 */

  flags   = enter_critical_section();

  // Enable clocking to UART3: Set UART3_GATING to High (Pass)
  // CCU Base Address: 0x01C20000
  // Bus Clock Gating Register3
  // Offset: 0x006C, Register Name: BUS_CLK_GATING_REG3
  // Bit 19
  // A64 User Manual Page 105
  uint32_t before = getreg32(BUS_CLK_GATING_REG3) & UART3_GATING;
  modreg32(UART3_GATING, UART3_GATING, BUS_CLK_GATING_REG3);
  uint32_t after = getreg32(BUS_CLK_GATING_REG3) & UART3_GATING;
  _info("Enable clocking to UART3: Set UART3_GATING to High (Pass): addr=0x%x, before=0x%x, after=0x%x\n", BUS_CLK_GATING_REG3, before, after);

  // Compare with UART0_GATING (Bit 16)
  _info("Compare with UART0_GATING: addr=0x%x, val=0x%x\n", BUS_CLK_GATING_REG3, getreg32(BUS_CLK_GATING_REG3) & UART0_GATING);

  // Deassert reset for UART3: Set UART3_RST to High
  // CCU Base Address: 0x01C20000
  // Bus Software Reset Register 4
  // Offset: 0x02D8, Register Name: BUS_SOFT_RST_REG4
  // Bit 19
  // A64 User Manual Page 142
  before = getreg32(BUS_SOFT_RST_REG4) & UART3_RST;
  modreg32(UART3_RST, UART3_RST, BUS_SOFT_RST_REG4);
  after = getreg32(BUS_SOFT_RST_REG4) & UART3_RST;
  _info("Deassert reset for UART3: Set UART3_RST to High: addr=0x%x, before=0x%x, after=0x%x\n", BUS_SOFT_RST_REG4, before, after);

  // Compare with UART0_RST (Bit 16)
  _info("Compare with UART0_RST: addr=0x%x, val=0x%x\n", BUS_SOFT_RST_REG4, getreg32(BUS_SOFT_RST_REG4) & UART0_RST);

  // PIO Base Address: 0x01C20800
  // PD Configure Register 0
  // Offset: 0x6C, Register Name: PD_CFG0_REG
  // Bits 0 to 2: PD0_SELECT
  // Bits 4 to 6: PD1_SELECT
  // A64 User Manual Page 385
  uint32_t before0 = getreg32(PD_CFG0_REG) & PD0_SELECT;
  uint32_t before1 = getreg32(PD_CFG0_REG) & PD1_SELECT;

  // Enable UART3 on PD0 and PD1: PD0_SELECT and PD1_SELECT
  ret = a64_pio_config(PIO_UART3_TX);
  DEBUGASSERT(ret == OK);
  ret = a64_pio_config(PIO_UART3_RX);
  DEBUGASSERT(ret == OK);

  uint32_t after0 = getreg32(PD_CFG0_REG) & PD0_SELECT;
  uint32_t after1 = getreg32(PD_CFG0_REG) & PD1_SELECT;
  _info("Enable UART3 on PD0: PD0_SELECT: addr=0x%x, before=0x%x, after=0x%x\n", PD_CFG0_REG, before0, after0);
  _info("Enable UART3 on PD1: PD0_SELECT: addr=0x%x, before=0x%x, after=0x%x\n", PD_CFG0_REG, before1, after1);

  /* Step 2: Enable clocking to UART3 */

  /* Step 3: Configure I/O pins */

  // a64_pio_config(PIO_UART3_TX);
  // a64_pio_config(PIO_UART3_RX);
  leave_critical_section(flags);
  return OK;
};
#endif /* CONFIG_A64_UART3 */

#ifdef CONFIG_A64_UART4
// TODO
static int a64_uart4config(void)
{
  irqstate_t flags;
  int ret;

  /* Step 1: Enable power to UART4 */

  flags   = enter_critical_section();
#warning Missing logic

  /* Step 2: Enable clocking on UART4 */
#warning Missing logic

  /* Step 3: Configure I/O pins */

  a64_pio_config(PIO_UART4_TX);
  a64_pio_config(PIO_UART4_RX);
  leave_critical_section(flags);
  return OK;
};
#endif /* CONFIG_A64_UART4 */

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* UART Operations for Serial Driver */

static const struct uart_ops_s g_uart_ops =
{
  .setup    = a64_uart_setup,
  .shutdown = a64_uart_shutdown,
  .attach   = a64_uart_attach,
  .detach   = a64_uart_detach,
  .ioctl    = a64_uart_ioctl,
  .receive  = a64_uart_receive,
  .rxint    = a64_uart_rxint,
  .rxavailable = a64_uart_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol    = NULL,
#endif
  .send     = a64_uart_send,
  .txint    = a64_uart_txint,
  .txready  = a64_uart_txready,
  .txempty  = a64_uart_txempty,
};

/* UART0 Port State (Console) */

static struct a64_uart_port_s g_uart0priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART0_BAUD,
      .parity     = CONFIG_UART0_PARITY,
      .bits       = CONFIG_UART0_BITS,
      .stopbits2  = CONFIG_UART0_2STOP
    },

  .config =
    {
      .uart       = CONFIG_A64_UART0_BASE
    },

    .irq_num      = CONFIG_A64_UART0_IRQ,
    .is_console   = 1
};

#ifdef CONFIG_A64_UART

/* UART0 I/O Buffers (Console) */

static char                 g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char                 g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

/* UART0 Port Definition (Console) */

static struct uart_dev_s    g_uart0port =
{
  .recv  =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart0priv,
};

#endif /* CONFIG_A64_UART */

#ifdef CONFIG_A64_UART1 ////

/* UART1 Port State */

static struct a64_uart_port_s g_uart1priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART1_BAUD,
      .parity     = CONFIG_UART1_PARITY,
      .bits       = CONFIG_UART1_BITS,
      .stopbits2  = CONFIG_UART1_2STOP
    },

  .config =
    {
      .uart       = CONFIG_A64_UART1_BASE
    },

    .irq_num      = CONFIG_A64_UART1_IRQ,
    .is_console   = 0
};

/* UART1 I/O Buffers */

static char                 g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char                 g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

/* UART1 Port Definition */

static struct uart_dev_s    g_uart1port =
{
  .recv  =
    {
      .size   = CONFIG_UART1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart1priv,
};

#endif /* CONFIG_A64_UART1 */

#ifdef CONFIG_A64_UART2 ////

/* UART2 Port State */

static struct a64_uart_port_s g_uart2priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART2_BAUD,
      .parity     = CONFIG_UART2_PARITY,
      .bits       = CONFIG_UART2_BITS,
      .stopbits2  = CONFIG_UART2_2STOP
    },

  .config =
    {
      .uart       = CONFIG_A64_UART2_BASE
    },

    .irq_num      = CONFIG_A64_UART2_IRQ,
    .is_console   = 0
};

/* UART2 I/O Buffers */

static char                 g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char                 g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];

/* UART2 Port Definition */

static struct uart_dev_s    g_uart2port =
{
  .recv  =
    {
      .size   = CONFIG_UART2_RXBUFSIZE,
      .buffer = g_uart2rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART2_TXBUFSIZE,
      .buffer = g_uart2txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart2priv,
};

#endif /* CONFIG_A64_UART2 */

#ifdef CONFIG_A64_UART3 ////

/* UART3 Port State */

static struct a64_uart_port_s g_uart3priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART3_BAUD,
      .parity     = CONFIG_UART3_PARITY,
      .bits       = CONFIG_UART3_BITS,
      .stopbits2  = CONFIG_UART3_2STOP
    },

  .config =
    {
      .uart       = CONFIG_A64_UART3_BASE
    },

    .irq_num      = CONFIG_A64_UART3_IRQ,
    .is_console   = 0
};

/* UART3 I/O Buffers */

static char                 g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char                 g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];

/* UART3 Port Definition */

static struct uart_dev_s    g_uart3port =
{
  .recv  =
    {
      .size   = CONFIG_UART3_RXBUFSIZE,
      .buffer = g_uart3rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART3_TXBUFSIZE,
      .buffer = g_uart3txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart3priv,
};

#endif /* CONFIG_A64_UART3 */

#ifdef CONFIG_A64_UART4 ////

/* UART4 Port State */

static struct a64_uart_port_s g_uart4priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART4_BAUD,
      .parity     = CONFIG_UART4_PARITY,
      .bits       = CONFIG_UART4_BITS,
      .stopbits2  = CONFIG_UART4_2STOP
    },

  .config =
    {
      .uart       = CONFIG_A64_UART4_BASE
    },

    .irq_num      = CONFIG_A64_UART4_IRQ,
    .is_console   = 0
};

/* UART4 I/O Buffers */

static char                 g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char                 g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];

/* UART4 Port Definition */

static struct uart_dev_s    g_uart4port =
{
  .recv  =
    {
      .size   = CONFIG_UART4_RXBUFSIZE,
      .buffer = g_uart4rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART4_TXBUFSIZE,
      .buffer = g_uart4txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart4priv,
};

#endif /* CONFIG_A64_UART4 */

/* Pick ttys1.  This could be any of UART1-4. */

#if defined(CONFIG_A64_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#elif defined(CONFIG_A64_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS1_DEV           g_uart2port /* UART2 is ttyS1 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_A64_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS1_DEV           g_uart3port /* UART3 is ttyS1 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_A64_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS1_DEV           g_uart4port /* UART4 is ttyS1 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys2.  This could be one of UART2-4. */

#if defined(CONFIG_A64_UART2) && !defined(UART2_ASSIGNED)
#  define TTYS2_DEV           g_uart2port /* UART2 is ttyS2 */
#  define UART2_ASSIGNED      1
#elif defined(CONFIG_A64_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS2_DEV           g_uart3port /* UART3 is ttyS2 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_A64_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS2_DEV           g_uart4port /* UART4 is ttyS2 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys3.  This could be one of UART3-4. */

#if defined(CONFIG_A64_UART3) && !defined(UART3_ASSIGNED)
#  define TTYS3_DEV           g_uart3port /* UART3 is ttyS3 */
#  define UART3_ASSIGNED      1
#elif defined(CONFIG_A64_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS3_DEV           g_uart4port /* UART4 is ttyS3 */
#  define UART4_ASSIGNED      1
#endif

/* Pick ttys4.  This could only be UART4. */

#if defined(CONFIG_A64_UART4) && !defined(UART4_ASSIGNED)
#  define TTYS4_DEV           g_uart4port /* UART4 is ttyS4 */
#  define UART4_ASSIGNED      1
#endif

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: arm64_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm64_serialinit.
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void arm64_earlyserialinit(void)
{
  int ret;

  /* NOTE: This function assumes that UART0 low level hardware configuration
   * -- including all clocking and pin configuration -- was performed
   * earlier by U-Boot Bootloader.
   */

#ifdef CONFIG_A64_UART1 ////
  /* Configure UART1 */

  ret = a64_uart1config();

  if (ret < 0)
    {
      sinfo("UART1 config failed, ret=%d\n", ret);
    }
#endif /* CONFIG_A64_UART1 */

#ifdef CONFIG_A64_UART2 ////
  /* Configure UART2 */

  ret = a64_uart2config();

  if (ret < 0)
    {
      sinfo("UART2 config failed, ret=%d\n", ret);
    }
#endif /* CONFIG_A64_UART2 */

#ifdef CONFIG_A64_UART3 ////
  /* Configure UART3 */

  ret = a64_uart3config();

  if (ret < 0)
    {
      sinfo("UART3 config failed, ret=%d\n", ret);
    }
#endif /* CONFIG_A64_UART3 */

#ifdef CONFIG_A64_UART4 ////
  /* Configure UART4 */

  ret = a64_uart4config();

  if (ret < 0)
    {
      sinfo("UART4 config failed, ret=%d\n", ret);
    }
#endif /* CONFIG_A64_UART4 */

#ifdef CONSOLE_DEV
  /* Enable the console at UART0 */

  CONSOLE_DEV.isconsole = true;
  a64_uart_setup(&CONSOLE_DEV);
#endif
}

/***************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 * Input Parameters:
 *   ch - Character to be transmitted over UART
 *
 * Returned Value:
 *   Character that was transmitted
 *
 ***************************************************************************/

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm64_lowputc('\r');
    }

  arm64_lowputc((uint8_t)ch);
  return ch;
}

/***************************************************************************
 * Name: arm64_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that a64_earlyserialinit was called previously.
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void arm64_serialinit(void)
{
  int ret;

  ret = uart_register("/dev/console", &CONSOLE_DEV);
  if (ret < 0)
    {
      sinfo("Register /dev/console failed, ret=%d\n", ret);
    }

  ret = uart_register("/dev/ttyS0", &TTYS0_DEV);

  if (ret < 0)
    {
      sinfo("Register /dev/ttyS0 failed, ret=%d\n", ret);
    }

#ifdef TTYS1_DEV
  ret = uart_register("/dev/ttyS1", &TTYS1_DEV);

  if (ret < 0)
    {
      sinfo("Register /dev/ttyS1 failed, ret=%d\n", ret);
    }
#endif /* TTYS1_DEV */

#ifdef TTYS2_DEV
  ret = uart_register("/dev/ttyS2", &TTYS2_DEV);

  if (ret < 0)
    {
      sinfo("Register /dev/ttyS2 failed, ret=%d\n", ret);
    }
#endif /* TTYS2_DEV */

#ifdef TTYS3_DEV
  ret = uart_register("/dev/ttyS3", &TTYS3_DEV);

  if (ret < 0)
    {
      sinfo("Register /dev/ttyS3 failed, ret=%d\n", ret);
    }
#endif /* TTYS3_DEV */

#ifdef TTYS4_DEV
  ret = uart_register("/dev/ttyS4", &TTYS4_DEV);

  if (ret < 0)
    {
      sinfo("Register /dev/ttyS4 failed, ret=%d\n", ret);
    }
#endif /* TTYS4_DEV */
}

#else /* USE_SERIALDRIVER */

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 * Input Parameters:
 *   ch - Character to be transmitted over UART
 *
 * Returned Value:
 *   Character that was transmitted
 *
 ***************************************************************************/

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm64_lowputc('\r');
    }

  arm64_lowputc((uint8_t)ch);
  return ch;
}

#endif /* USE_SERIALDRIVER */
