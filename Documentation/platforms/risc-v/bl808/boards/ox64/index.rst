===========
PINE64 Ox64
===========

TODO

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
Audio
Microphone (optional, on the camera module)
Speaker (optional, on the camera module)

TODO: `Star64 <https://wiki.pine64.org/wiki/STAR64>`_ is a 64-bit RISC-V based
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
- **Video:** Digital Video Output up to 4K @ 30 Hz, 4K HDR @ 60 fps
- **Audio:** 3.5mm Audio Jack
- **Ethernet:** Single or Dual 10 / 100 / 1000Mbps
- **Wireless:** 2.4 GHz / 5 Ghz MIMO WiFi 802.11 b/g/n/ac with Bluetooth 5.2 (Realtek RTL8852BU)
- **Storage:** 128 Mbit (16 MByte) XSPI NOR flash Memory, Bootable microSD (SDHC and SDXC up to 256 GB), Bootable eMMC
- **USB:** 1 x USB 3.0 Dedicated Host Port, 3 x USB 2.0 Host Ports
- **Expansion Ports:** PCIe 2.0 x 1 lane, 2 x 20 pins "Pi2" GPIO Header
- **MIPI DSI Port:** 4-lane MIPI DSI port for LCD Panel
- **MIPI CSI Port:** 4-lane MIPI CSI port for Camera Module

Serial Console
==============

A **USB Serial Adapter** that supports 2 Mbps (like `CH340G Serial Adapter <https://lupyuen.github.io/articles/ox64#test-the-usb-serial-adapter>`_)
is required to run NuttX on Ox64.

Connect the USB Serial Adapter to Ox64 Serial Console at:

========== ========
USB Serial Ox64 Pin
========== ========
TX         Pin 31 (GPIO 17 / UART3 RX)
RX         Pin 32 (GPIO 16 / UART3 TX)
GND        Pin 33 (GND)
========== ========

On the USB Serial Adapter, set the **Voltage Level** to 3V3.

Connect Ox64 to our computer with the USB Serial Adapter.
On our computer, start a Serial Terminal and connect to the USB Serial Port
at **2 Mbps**.

NuttX will appear in the Serial Console when it boots on Ox64.

RISC-V Toolchain
================

Before building NuttX for Ox64, download the **RISC-V Toolchain riscv64-unknown-elf**
from `SiFive RISC-V Tools <https://github.com/sifive/freedom-tools/releases/tag/v2020.12.0>`_.

Add the downloaded toolchain ``riscv64-unknown-elf-toolchain-.../bin``
to the ``PATH`` Environment Variable.

Check the RISC-V Toolchain:

.. code:: console

   $ riscv64-unknown-elf-gcc -v

Building
========

To build NuttX for Ox64, :doc:`install the prerequisites </quickstart/install>` and
:doc:`clone the git repositories </quickstart/install>` for ``nuttx`` and ``apps``.

Configure the NuttX project and build the project:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh ox64:nsh
   $ make
   $ riscv64-unknown-elf-objcopy -O binary nuttx nuttx.bin

This produces the NuttX Kernel ``nuttx.bin``.  Next, build the NuttX Apps Filesystem:

.. code:: console

   $ make export
   $ pushd ../apps
   $ tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
   $ make import
   $ popd
   $ genromfs -f initrd -d ../apps/bin -V "NuttXBootVol"

This generates the Initial RAM Disk ``initrd``.

Package the NuttX Kernel and Initial RAM Disk into a NuttX Image:

.. code:: console

   $ head -c 65536 /dev/zero >/tmp/nuttx.pad
   $ cat nuttx.bin /tmp/nuttx.pad initrd >Image

The NuttX Image ``Image`` will be copied to a microSD Card in the next step.

Booting
=======

Flash `OpenSBI and U-Boot Bootloader <https://lupyuen.github.io/articles/ox64>`_ to Ox64.

Prepare a `Linux microSD Card <https://lupyuen.github.io/articles/ox64>`_ for Ox64.

Copy the file ``Image`` from the previous section
and overwrite the file on the microSD Card.

Check that Ox64 is connected to our computer via a USB Serial Adapter at 2 Mbps.

Insert the microSD Card into Ox64 and power up Ox64 via the Micro USB Port.
NuttX boots on Ox64 and NuttShell (nsh) appears in the Serial Console.

To see the available commands in NuttShell:

.. code:: console

   $ help

Configurations
==============

nsh
---

Basic configuration that runs NuttShell (nsh).
This configuration is focused on low level, command-line driver testing.
Built-in applications are supported, but none are enabled.
Serial Console is enabled on UART3 at 2 Mbps.

Peripheral Support
==================

NuttX for Ox64 supports these peripherals:

======================== ======= =====
Peripheral               Support NOTES
======================== ======= =====
UART                     Yes
======================== ======= =====
