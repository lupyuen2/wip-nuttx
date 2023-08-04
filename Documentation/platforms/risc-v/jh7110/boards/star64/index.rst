=============
PINE64 Star64
=============

`Star64 <https://wiki.pine64.org/wiki/STAR64>`_ is a 64-bit RISC-V based
Single Board Computer powered by StarFive JH7110 Quad-Core SiFive U74 64-Bit CPU,
Imagination Technology BX-4-32 GPU and supports up to 8GB 1866MHz LPDDR4 memory.

It provides an eMMC module socket, MicroSD Card slot, PCI-e, Pi-2 Bus, USB 3.0
and many other peripheral interfaces for makers to integrate with sensors
and other devices.

Features
========

- **System on Chip:** StarFive JH7110
    - **CPU:** SiFive RISC-V U74 Application Cores (4 cores, RV64GCB) and SiFive RISC-V S7 Monitor Core (single core, RV64IMACB)
    - **GPU:** Imagination Technology BXE-4-32
    - **RAM:** LPDDR4 2GB / 4GB / 8GB
- **Video:** Digital Video Output up to 4K@30Hz, 4K HDR @ 60fps
- **Audio:** 3.5mm Audio Jack
- **Ethernet:** Single or Dual 10 / 100 / 1000Mbps
- **Wireless:** 2.4 GHz / 5 Ghz MIMO WiFi 802.11 b/g/n/ac with Bluetooth 5.2 (Realtek RTL8852BU)
- **Storage:** 128 Mbit (16 MByte) XSPI NOR flash Memory, Bootable
microSD (SDHC and SDXC up to 256 GB), Bootable eMMC
- **USB:** 1 x USB 3.0 Dedicated Host Port, 3 x USB 2.0 Host Ports
- **Expansion Ports:** PCIe 2.0 x 1 lane, 2 x 20 pins "Pi2" GPIO Header
- **MIPI DSI Port:** 4-lane MIPI DSI port for LCD Panel
- **MIPI CSI Port:** 4-lane MIPI CSI port for Camera Module

Serial Console
==============

A **USB Serial Adapter** (like `CH340G Serial Console <https://pine64.com/product/serial-console-woodpecker-edition/>`_)
is required to run NuttX on Star64.

Connect the USB Serial Adapter to Star64's GPIO Header at:

================== ==================
USB Serial Adapter Star64 GPIO Header
================== ==================
GND                Pin 6 (GND)
RX                 Pin 8 (TX)
TX                 Pin 10 (RX)
================== ==================

On the USB Serial Adapter, set the **Voltage Level** to 3V3.

Connect Star64 to our computer with the USB Serial Adapter.
On our computer, start a Serial Terminal and connect to the USB Serial Port
at **115.2 kbps**.

NuttX will appear in the Serial Console when it boots on Star64.

TODO

ARM64 Toolchain
===============

Before building NuttX for PinePhone, download the ARM64 Toolchain for
**AArch64 Bare-Metal Target** ``aarch64-none-elf`` from
`Arm GNU Toolchain Downloads <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>`_.
(Skip the section for Beta Releases)

Add the downloaded toolchain ``gcc-arm-...-aarch64-none-elf/bin``
to the ``PATH`` Environment Variable.

Check the ARM64 Toolchain:

.. code:: console

   $ aarch64-none-elf-gcc -v

Building
========

To build NuttX for PinePhone, :doc:`install the prerequisites </quickstart/install>` and
:doc:`clone the git repositories </quickstart/install>` for ``nuttx`` and ``apps``.

Configure the NuttX project and build the project:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh pinephone:lvgl
   $ make
   $ cp nuttx.bin Image
   $ rm -f Image.gz
   $ gzip Image

This produces the file ``Image.gz``, which will be copied to PinePhone in the next step.

If the build fails with the error ``token "@" is not valid in preprocessor``,
`apply this patch <https://github.com/apache/nuttx/pull/7284/commits/518b0eb31cb66f25b590ae9a79ab16c319b96b94#diff-12291efd8a0ded1bc38bad733d99e4840ae5112b465c04287f91ba5169612c73>`_
to ``gcc-arm-none-eabi/arm-none-eabi/include/_newlib_version.h``
in the ARM64 Toolchain.

Booting
=======

NuttX boots on PinePhone via a microSD Card. To prepare the microSD Card, download the
**PinePhone Jumpdrive Image** ``pine64-pinephone.img.xz`` from
`dreemurrs-embedded/Jumpdrive <https://github.com/dreemurrs-embedded/Jumpdrive/releases>`_.

Write the downloaded image to a microSD Card with
`Balena Etcher <https://www.balena.io/etcher/>`_.

Copy the file ``Image.gz`` from the previous section
and overwrite the file on the microSD Card.

Check that PinePhone is connected to our computer via a
`Serial Debug Cable <https://wiki.pine64.org/index.php/PinePhone#Serial_console>`_ at 115.2 kbps.
`Privacy Switch 6 (Headphone) <https://wiki.pine64.org/index.php/PinePhone#Privacy_switch_configuration>`_
should be set to **Off**.

Insert the microSD Card into PinePhone and power up PinePhone.
NuttX boots on PinePhone and NuttShell (nsh) appears in the Serial Console.

To see the available commands in NuttShell:

.. code:: console

   $ help

To run the LVGL Touchscreen Demo:

.. code:: console

   $ lvgldemo widgets

LEDs
====

The supported PinePhone LEDs are:

===== ========= ===
Index LED       PIO
===== ========= ===
LED1  Green LED PD18
LED2  Red LED   PD19
LED3  Blue LED  PD20
===== ========= ===

Configurations
==============

lcd
___

Supports LCD Display (XBD599) with LCD Controller (ST7703),
Display Engine 2.0, MIPI Display Serial Interface (DSI),
Power Management Integrated Circuit (AXP803) and
Reduced Serial Bus (RSB).
Serial Console is enabled on UART0 at 115.2 kbps.

lvgl
____

Supports all the features in ``lcd``,
plus LVGL Graphics Library and Touch Panel (GT917S).
Serial Console is enabled on UART0 at 115.2 kbps.

nsh
---

Basic configuration that runs NuttShell (nsh).
This configuration is focused on low level, command-line driver testing.
Built-in applications are supported, but none are enabled.
Serial Console is enabled on UART0 at 115.2 kbps.

sensor
------

Supports Accelerometer / Gyroscope (MPU-6050),
Power Management Integrated Circuit (AXP803) and
Reduced Serial Bus (RSB).
Serial Console is enabled on UART0 at 115.2 kbps.

Peripheral Support
==================

NuttX for PinePhone supports these peripherals:

======================== ======= =====
Peripheral               Support NOTES
======================== ======= =====
Accelerometer (MPU-6050) Yes
Backlight                Yes
Display Engine           Yes
Frame Buffer             Yes
LCD Controller (ST7703)  Yes
LCD Panel (XBD599)       Yes
MIPI D-PHY               Yes
MIPI DSI                 Yes
PIO                      Yes
PMIC (AXP803)            Yes
RSB                      Yes
TCON0                    Yes
TWI / I2C                Yes
Touch Panel (GT917S)     Yes
UART                     Yes
======================== ======= =====
