==================
Bouffalo Lab BL808
==================

`Bouffalo Lab BL808 <https://github.com/bouffalolab/bl_docs/tree/main/BL808_RM/en>`_ is a 64-bit / 32-bit RISC-V SoC with 3 RISC-V Cores:

- **D0 Multimedia Core:** T-Head C906 480 MHz 64-bit RISC-V CPU
    - RV64IMAFCV
    - Level 1 Instruction and Data Cache (Harvard architecture)
    - Sv39 Memory Management Unit
    - jTLB (128 entries)
    - AXI 4.0 128-bit master interface
    - Core Local Interrupt (CLINT) and Platform-Level Interrupt Controller (PLIC)
    - 80 External Interrupt Sources
    - BHT (8K) and BTB
    - RISC-V PMP (8 configurable areas)

- **M0 Wireless Core:** T-Head E907 320 MHz 32-bit RISC-V CPU
    - RV32IMAFCP
    - 32-bit / 16-bit Mixed Instruction Set
    - RISC-V Machine Mode and User Mode
    - 32 x 32-bit Integer General Purpose Registers (GPR)
    - 32 x 32-bit / 64-bit Floating-Point GPRs
    - AXI 4.0 main device interface and AHB 5.0 peripheral interface
    - Instruction and Data Cache

- **LP Low Power Core:** T-Head E902 150 MHz 32-bit RISC-V CPU
    - RV32E[M]C

- **RAM:** Embedded 64 MB PSRAM
- **Wireless:** 2.4 GHz 1T1R WiFi 802.11 b/g/n, Bluetooth 5.2, Zigbee
- **Ethernet:** 10 / 100 Mbps

TODO

- **Storage:** On-board 16 Mbit (2 MB) or 128 Mbit (16 MB) XSPI NOR Flash Memory, MicroSD (SDHC and SDXC)
- **USB:** USB 2.0 OTG
- **Expansion Ports:** 26 GPIO pins (including SPI, I2C and UART)
- **Audio:** Microphone and Speaker (optional)
- **MIPI CSI Port:** Dual-lane MIPI CSI port for Camera Module (USB-C)

TODO

- **Peripherals:** TODO

Network
2.4 GHz 1T1R WiFi 802.11 b/g/n
Bluetooth 5.2
Zigbee
10/100 Mbit/s Ethernet (optional, on expansion board)

Storage
On-board 16 Mbit (2 MB) or 128 Mbit (16 MB) XSPI NOR flash memory
MicroSD, supports SDHC and SDXC (only on the 128 Mbit version)

Expansion Ports
USB 2.0 OTG port
26 GPIO pins, including SPI, I2C and UART functionality, possible I2S and GMII expansion

Dual-lane MiPi CSI port, located at USB-C port, for camera module

TODO

- **CPU:** SiFive RISC-V U74 Application Cores (4 cores, RV64GCB) and SiFive RISC-V S7 Monitor Core (single core, RV64IMACB)
- **GPU:** Imagination Technology BXE-4-32
- **RAM:** 32-bit LPDDR4 / DDR4 / LPDDR3 / DDR3
- **Video Decoder:** 4K @ 30 fps multi-stream for H.264/H.265
- **Video Encoder:** 1080p @ 30 fps multi-stream for H.265
- **Video Input:** 1 x DVP and 1 x MIPI-CSI with 4D1C
- **Video Output:** MIPI DSI with 4D1C
- **HDMI:** 1 x HDMI 2.0 port display up to 4K @ 30 fps
- **Parallel Interface:**: 24-bit RGB parallel interface
- **PCIe:** 2 x PCIe 2.0, 1 lane
- **USB:** USB 3.0 Host / Device
- **Ethernet:** 2 x Ethernet MAC 1,000 Mbps, 2 x CAN 2.0B
- **Security:** TRNG, OTP
- **Peripherals:** UART, I2C, SPI, SDIO, DPI, PCM / I2S, Timers, Temperature Sensor, INTC, PWM, WDT, GPIO, DVP, GPCLK

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
