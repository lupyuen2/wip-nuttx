================
PINE64 StarPro64
================

`PINE64 StarPro64 <https://lupyuen.github.io/articles/starpro64>`_ is a RISC-V Single-Board Computer

TODO

based on the SOPHGO SG2000 RISC-V SoC with T-Head C906 64-bit Main Processor,
512 MB of SIP DRAM memory and 100 Mbps Ethernet.

Features
========

TODO

- **System on Chip:** SOPHGO SG2000
    - **CPU:** 
        - 64-bit T-Head C906 1.0 GHz (Main Processor)
        - 64-bit T-Head C906 700 MHz (Co-Processor)
        - 64-bit Arm Cortex-A53 1.0 GHz (Alternate Main Processor)

- **MCU:** 8051 with 6 KB SRAM
- **Memory:** SIP DRAM 512 MB
- **TPU:** 0.5 TOPS INT8
- **Storage:** 1 x microSD Connector, 1 x eMMC Pad
- **USB:** 1 x Type-C for Power and Data or 1 x USB 2.0 A Port Host
- **MIPI CSI:** 1 x 16P FPC Connector (MIPI CSI 2-Lane), 1 x 15P FPC Connector (MIPI CSI 2-Lane)	
- **Sensor Support:** 5M @ 30 FPS
- **MIPI DSI:** Via GPIO Header (MIPI DSI 4-Lane)	
- **Ethernet:** 100 Mbps Ethernet Port (RJ45)
- **Wireless:** Optional Wi-Fi 6 / Bluetooth 5
- **Audio:** Via GPIO Pin	
- **GPIO:** Up to 39 x GPIO Pin (via 2 x 26-Pin GPIO Header)
- **Others:** 1 x Boot Switch, 1 x Recovery Key, 1 x Reset Key

Serial Console
==============

A **USB Serial Adapter** (CH340 or CP2102) is required to run NuttX
on StarPro64.

Connect the USB Serial Adapter to StarPro64 Serial Console at:

========== =================
USB Serial StarPro64 Pin
========== =================
GND        Pin 6 (GND)
RX         Pin 8 (UART0 TX)
TX         Pin 10 (UART0 RX)
========== =================

On the USB Serial Adapter, set the **Voltage Level** to 3V3.

Connect StarPro64 to our computer with the USB Serial Adapter.
On our computer, start a Serial Terminal and connect to the USB Serial Port
at **115.2 kbps**:

.. code:: console

   $ screen /dev/ttyUSB0 115200

NuttX will appear in the Serial Console when it boots on StarPro64.

RISC-V Toolchain
================

Before building NuttX for StarPro64, download the toolchain for
`xPack GNU RISC-V Embedded GCC (riscv-none-elf) <https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases>`_.

Add the downloaded toolchain ``xpack-riscv-none-elf-gcc-.../bin``
to the ``PATH`` Environment Variable.

Check the RISC-V Toolchain:

.. code:: console

   $ riscv-none-elf-gcc -v

Building
========

To build NuttX for StarPro64, :doc:`install the prerequisites </quickstart/install>` and
:doc:`clone the git repositories </quickstart/install>` for ``nuttx`` and ``apps``.

Configure the NuttX project and build the project:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh starpro64:nsh
   $ make

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
   $ cat nuttx.bin /tmp/nuttx.pad initrd >Image-starpro64

The NuttX Image ``Image-starpro64`` will be copied to the TFTP Server in the next step.

Booting
=======

To boot NuttX on StarPro64, `install TFTP Server <https://lupyuen.github.io/articles/starpro64#boot-nuttx-over-tftp>`_
on our computer.

Copy the file ``Image-starpro64`` from the previous section to the TFTP Server,
together with the Device Tree:

.. code:: console

   $ wget https://github.com/lupyuen2/wip-nuttx/releases/download/sg2000-1/TODO.dtb
   $ scp Image-starpro64 \
      tftpserver:/tftpfolder/Image-starpro64
   $ scp TODO.dtb \
      tftpserver:/tftpfolder/TODO.dtb

Check that StarPro64 is connected to our computer via a USB Serial Adapter at 115.2 kbps:

.. code:: console

   $ screen /dev/ttyUSB0 115200

When StarPro64 boots, press Ctrl-C to see the U-Boot Prompt.
Run these commands to `boot NuttX over TFTP <https://lupyuen.github.io/articles/starpro64#boot-nuttx-over-tftp>`_:

.. code:: console

   # Change to your TFTP Server
   $ setenv tftp_server 192.168.x.x
   $ saveenv
   $ dhcp ${kernel_addr_r} ${tftp_server}:Image-starpro64
   $ tftpboot ${fdt_addr_r} ${tftp_server}:TODO.dtb
   $ fdt addr ${fdt_addr_r}
   $ booti ${kernel_addr_r} - ${fdt_addr_r}

Or configure U-Boot to `boot NuttX automatically <https://lupyuen.github.io/articles/starpro64#boot-nuttx-over-tftp>`_.

NuttX boots on StarPro64 and NuttShell (nsh) appears in the Serial Console.
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
Serial Console is enabled on UART0 at 115.2 kbps.

Peripheral Support
==================

NuttX for StarPro64 supports these peripherals:

======================== ======= =====
Peripheral               Support NOTES
======================== ======= =====
UART                     Yes
======================== ======= =====
