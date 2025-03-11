/****************************************************************************
 * boards/arm64/a527/avaota-a1/src/a527_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/types.h>
#include <sys/mount.h>
#include <sys/boardctl.h>
#include <syslog.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/virtio/virtio-mmio.h>
#include <nuttx/fdt.h>
#include <nuttx/pci/pci_ecam.h>
#include <nuttx/drivers/ramdisk.h>

#ifdef CONFIG_LIBC_FDT
#  include <libfdt.h>
#endif

#include "avaota-a1.h"

//// TODO
extern uint8_t __ramdisk_start[];
extern uint8_t __ramdisk_size[];

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef A527_SPI_IRQ_BASE
#define A527_SPI_IRQ_BASE     32
#endif

#define FDT_PCI_TYPE_IO              0x01000000
#define FDT_PCI_TYPE_MEM32           0x02000000
#define FDT_PCI_TYPE_MEM64           0x03000000
#define FDT_PCI_TYPE_MASK            0x03000000
#define FDT_PCI_PREFETCH             0x40000000

/* RAM Disk Definition */

#define SECTORSIZE   512
#define NSECTORS(b)  (((b) + SECTORSIZE - 1) / SECTORSIZE)
#define RAMDISK_DEVICE_MINOR 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_LIBC_FDT) && defined(CONFIG_DEVICE_TREE)

/****************************************************************************
 * Name: register_pci_host_from_fdt
 ****************************************************************************/

#ifdef CONFIG_PCI
static void register_pci_host_from_fdt(const void *fdt)
{
  struct pci_resource_s prefetch;
  struct pci_resource_s cfg;
  struct pci_resource_s mem;
  struct pci_resource_s io;
  const fdt32_t *ranges;
  int offset;

  /* #address-size must be 3
   * defined in the PCI Bus Binding to IEEE Std 1275-1994 :
   * Bit#
   *
   * phys.hi cell:  npt000ss bbbbbbbb dddddfff rrrrrrrr
   * phys.mid cell: hhhhhhhh hhhhhhhh hhhhhhhh hhhhhhhh
   * phys.lo cell:  llllllll llllllll llllllll llllllll
   */

  const int na = 3;

  /* #size-cells must be 2 */

  const int ns = 2;
  int rlen;
  int pna;

  memset(&prefetch, 0, sizeof(prefetch));
  memset(&cfg, 0, sizeof(cfg));
  memset(&mem, 0, sizeof(mem));
  memset(&io, 0, sizeof(io));

  offset = fdt_node_offset_by_compatible(fdt, -1,
                                         "pci-host-ecam-generic");
  if (offset < 0)
    {
      return;
    }

  /* Get the reg address, 64 or 32 */

  cfg.start = fdt_get_reg_base(fdt, offset, 0);
  cfg.end = cfg.start + fdt_get_reg_size(fdt, offset);

  /* Get the ranges address */

  ranges = fdt_getprop(fdt, offset, "ranges", &rlen);
  if (ranges < 0)
    {
      return;
    }

  pna = fdt_get_parent_address_cells(fdt, offset);

  for (rlen /= 4; (rlen -= na + pna + ns) >= 0; ranges += na + pna + ns)
    {
      uint32_t type = fdt32_ld(ranges);

      if ((type & FDT_PCI_TYPE_MASK) == FDT_PCI_TYPE_IO)
        {
          io.start = fdt_ld_by_cells(ranges + na, pna);
          io.end = io.start + fdt_ld_by_cells(ranges + na + pna, ns);
        }
      else if ((type & FDT_PCI_PREFETCH) == FDT_PCI_PREFETCH ||
               (type & FDT_PCI_TYPE_MASK) == FDT_PCI_TYPE_MEM64)
        {
          prefetch.start = fdt_ld_by_cells(ranges + na, pna);
          prefetch.end = prefetch.start +
                         fdt_ld_by_cells(ranges + na + pna, ns);
        }
      else
        {
          mem.start = fdt_ld_by_cells(ranges + na, pna);
          mem.end = mem.start + fdt_ld_by_cells(ranges + na + pna, ns);
        }
    }

  pci_ecam_register(&cfg, &io, &mem, NULL);
}
#endif

/****************************************************************************
 * Name: register_devices_from_fdt
 ****************************************************************************/

static void register_devices_from_fdt(void)
{
  const void *fdt = fdt_get();
  int ret;

  if (fdt == NULL)
    {
      return;
    }

#ifdef CONFIG_DRIVERS_VIRTIO_MMIO

  ret = fdt_virtio_mmio_devices_register(fdt, A527_SPI_IRQ_BASE);
  if (ret < 0)
    {
      syslog(LOG_ERR, "fdt_virtio_mmio_devices_register failed, ret=%d\n",
             ret);
    }
#endif

#ifdef CONFIG_PCI
  register_pci_host_from_fdt(fdt);
#endif

  UNUSED(ret);
}

#endif

/****************************************************************************
 * Name: mount_ramdisk
 *
 * Description:
 *  Mount a RAM Disk defined in ld.script to /dev/ramX.  The RAM Disk
 *  contains a ROMFS filesystem with applications that can be spawned at
 *  runtime.
 *
 * Returned Value:
 *   OK is returned on success.
 *   -ERRORNO is returned on failure.
 *
 ****************************************************************************/

static int mount_ramdisk(void)
{
  int ret;
  struct boardioc_romdisk_s desc;

  desc.minor    = RAMDISK_DEVICE_MINOR;
  desc.nsectors = NSECTORS((ssize_t)__ramdisk_size);
  desc.sectsize = SECTORSIZE;
  desc.image    = __ramdisk_start;

  ret = boardctl(BOARDIOC_ROMDISK, (uintptr_t)&desc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Ramdisk register failed: %s\n", strerror(errno));
      syslog(LOG_ERR, "Ramdisk mountpoint /dev/ram%d\n",
             RAMDISK_DEVICE_MINOR);
      syslog(LOG_ERR, "Ramdisk length %lu, origin %lx\n",
             (ssize_t)__ramdisk_size, (uintptr_t)__ramdisk_start);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a527_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int a527_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmp file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at /tmp: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_LIBC_FDT) && defined(CONFIG_DEVICE_TREE)
  register_devices_from_fdt();
#endif

  /* Mount the RAM Disk */

  mount_ramdisk();

  UNUSED(ret);
  return OK;
}
