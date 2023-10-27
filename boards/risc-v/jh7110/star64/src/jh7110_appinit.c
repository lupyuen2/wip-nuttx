/****************************************************************************
 * boards/risc-v/jh7110/star64/src/jh7110_appinit.c
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

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/drivers/ramdisk.h>
#include <sys/mount.h>
#include <sys/boardctl.h>
#include <arch/board/board_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ramdisk Definition */

#define SECTORSIZE   512
#define NSECTORS(b)  (((b) + SECTORSIZE - 1) / SECTORSIZE)
#define RAMDISK_DEVICE_MINOR 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mount_ramdisk
 *
 * Description:
 *  Mount a ramdisk defined in the ld.script to /dev/ramX.  The ramdisk is
 *  intended to contain a romfs with applications which can be spawned at
 *  runtime.
 *
 * Returned Value:
 *   OK is returned on success.
 *   -ERRORNO is returned on failure.
 *
 ****************************************************************************/

int mount_ramdisk(void)
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
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifdef CONFIG_BOARD_LATE_INITIALIZE
  /* Board initialization already performed by board_late_initialize() */

  return OK;
#else
  /* Perform board-specific initialization */

#ifdef CONFIG_NSH_ARCHINIT

  mount(NULL, "/proc", "procfs", 0, NULL);

#endif

  return OK;
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called after up_initialize() and board_early_initialize() and just
 *   before the initial application is started.  This additional
 *   initialization phase may be used, for example, to initialize board-
 *   specific device drivers for which board_early_initialize() is not
 *   suitable.
 *
 *   Waiting for events, use of I2C, SPI, etc are permissible in the context
 *   of board_late_initialize().  That is because board_late_initialize()
 *   will run on a temporary, internal kernel thread.
 *
 ****************************************************************************/

void board_late_initialize(void)
{
  /* Mount the RAM Disk */

  mount_ramdisk();

  /* Perform board-specific initialization */

#ifdef CONFIG_NSH_ARCHINIT

  mount(NULL, "/proc", "procfs", 0, NULL);

#endif

  // Make an ecall to OpenSBI
  int test_opensbi(void);
  int ret = test_opensbi();
  DEBUGASSERT(ret == OK);
}

// SBI Definitions
// https://github.com/riscv-software-src/opensbi/blob/master/include/sbi/sbi_ecall_interface.h

/* SBI Extension IDs */
#define SBI_EXT_0_1_SET_TIMER			0x0
#define SBI_EXT_0_1_CONSOLE_PUTCHAR		0x1
#define SBI_EXT_0_1_CONSOLE_GETCHAR		0x2
#define SBI_EXT_0_1_CLEAR_IPI			0x3
#define SBI_EXT_0_1_SEND_IPI			0x4
#define SBI_EXT_0_1_REMOTE_FENCE_I		0x5
#define SBI_EXT_0_1_REMOTE_SFENCE_VMA		0x6
#define SBI_EXT_0_1_REMOTE_SFENCE_VMA_ASID	0x7
#define SBI_EXT_0_1_SHUTDOWN			0x8
#define SBI_EXT_BASE				0x10
#define SBI_EXT_TIME				0x54494D45
#define SBI_EXT_IPI				0x735049
#define SBI_EXT_RFENCE				0x52464E43
#define SBI_EXT_HSM				0x48534D
#define SBI_EXT_SRST				0x53525354
#define SBI_EXT_PMU				0x504D55
#define SBI_EXT_DBCN				0x4442434E
#define SBI_EXT_SUSP				0x53555350
#define SBI_EXT_CPPC				0x43505043

/* SBI function IDs for BASE extension*/
#define SBI_EXT_BASE_GET_SPEC_VERSION		0x0
#define SBI_EXT_BASE_GET_IMP_ID			0x1
#define SBI_EXT_BASE_GET_IMP_VERSION		0x2
#define SBI_EXT_BASE_PROBE_EXT			0x3
#define SBI_EXT_BASE_GET_MVENDORID		0x4
#define SBI_EXT_BASE_GET_MARCHID		0x5
#define SBI_EXT_BASE_GET_MIMPID			0x6

/* SBI function IDs for TIME extension*/
#define SBI_EXT_TIME_SET_TIMER			0x0

/* SBI function IDs for IPI extension*/
#define SBI_EXT_IPI_SEND_IPI			0x0

/* SBI function IDs for RFENCE extension*/
#define SBI_EXT_RFENCE_REMOTE_FENCE_I		0x0
#define SBI_EXT_RFENCE_REMOTE_SFENCE_VMA	0x1
#define SBI_EXT_RFENCE_REMOTE_SFENCE_VMA_ASID	0x2
#define SBI_EXT_RFENCE_REMOTE_HFENCE_GVMA_VMID	0x3
#define SBI_EXT_RFENCE_REMOTE_HFENCE_GVMA	0x4
#define SBI_EXT_RFENCE_REMOTE_HFENCE_VVMA_ASID	0x5
#define SBI_EXT_RFENCE_REMOTE_HFENCE_VVMA	0x6

/* SBI function IDs for HSM extension */
#define SBI_EXT_HSM_HART_START			0x0
#define SBI_EXT_HSM_HART_STOP			0x1
#define SBI_EXT_HSM_HART_GET_STATUS		0x2
#define SBI_EXT_HSM_HART_SUSPEND		0x3

/* SBI function IDs for SRST extension */
#define SBI_EXT_SRST_RESET			0x0

#define SBI_SRST_RESET_TYPE_SHUTDOWN		0x0
#define SBI_SRST_RESET_TYPE_COLD_REBOOT	0x1
#define SBI_SRST_RESET_TYPE_WARM_REBOOT	0x2
#define SBI_SRST_RESET_TYPE_LAST	SBI_SRST_RESET_TYPE_WARM_REBOOT

#define SBI_SRST_RESET_REASON_NONE	0x0
#define SBI_SRST_RESET_REASON_SYSFAIL	0x1

/* SBI function IDs for PMU extension */
#define SBI_EXT_PMU_NUM_COUNTERS	0x0
#define SBI_EXT_PMU_COUNTER_GET_INFO	0x1
#define SBI_EXT_PMU_COUNTER_CFG_MATCH	0x2
#define SBI_EXT_PMU_COUNTER_START	0x3
#define SBI_EXT_PMU_COUNTER_STOP	0x4
#define SBI_EXT_PMU_COUNTER_FW_READ	0x5
#define SBI_EXT_PMU_COUNTER_FW_READ_HI	0x6

/* SBI function IDs for DBCN extension */
#define SBI_EXT_DBCN_CONSOLE_WRITE		0x0
#define SBI_EXT_DBCN_CONSOLE_READ		0x1
#define SBI_EXT_DBCN_CONSOLE_WRITE_BYTE		0x2

/* SBI function IDs for SUSP extension */
#define SBI_EXT_SUSP_SUSPEND			0x0

#define SBI_SUSP_SLEEP_TYPE_SUSPEND		0x0
#define SBI_SUSP_SLEEP_TYPE_LAST		SBI_SUSP_SLEEP_TYPE_SUSPEND
#define SBI_SUSP_PLATFORM_SLEEP_START		0x80000000

/* SBI function IDs for CPPC extension */
#define SBI_EXT_CPPC_PROBE			0x0
#define SBI_EXT_CPPC_READ			0x1
#define SBI_EXT_CPPC_READ_HI			0x2
#define SBI_EXT_CPPC_WRITE			0x3

/* SBI return error codes */
#define SBI_SUCCESS				0
#define SBI_ERR_FAILED				-1
#define SBI_ERR_NOT_SUPPORTED			-2
#define SBI_ERR_INVALID_PARAM			-3
#define SBI_ERR_DENIED				-4
#define SBI_ERR_INVALID_ADDRESS			-5
#define SBI_ERR_ALREADY_AVAILABLE		-6
#define SBI_ERR_ALREADY_STARTED			-7
#define SBI_ERR_ALREADY_STOPPED			-8

// SBI Return Type. From
// https://github.com/riscv-software-src/opensbi/blob/master/firmware/payloads/test_main.c
struct sbiret {
  unsigned long error;
  unsigned long value;
};

static struct sbiret sbi_ecall(unsigned int extid, unsigned int fid,
                               uintptr_t parm0, uintptr_t parm1,
                               uintptr_t parm2, uintptr_t parm3,
                               uintptr_t parm4, uintptr_t parm5);

// Make an ecall to OpenSBI. Based on
// https://github.com/riscv-software-src/opensbi/blob/master/firmware/payloads/test_main.c
// https://www.thegoodpenguin.co.uk/blog/an-overview-of-opensbi/
int test_opensbi(void)
{
  // Print `123` to Debug Console with Legacy Console Putchar.
  // Call sbi_console_putchar: EID 0x01, FID 0
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/master/src/ext-legacy.adoc
  sbi_ecall(SBI_EXT_0_1_CONSOLE_PUTCHAR, 0, '1', 0, 0, 0, 0, 0);
  sbi_ecall(SBI_EXT_0_1_CONSOLE_PUTCHAR, 0, '2', 0, 0, 0, 0, 0);
  sbi_ecall(SBI_EXT_0_1_CONSOLE_PUTCHAR, 0, '3', 0, 0, 0, 0, 0);

  // TODO: Not supported by SBI v1.0, this will return SBI_ERR_NOT_SUPPORTED
  // Print `456` to Debug Console.
  // Call sbi_debug_console_write: EID 0x4442434E "DBCN", FID 0
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/master/src/ext-debug-console.adoc#function-console-write-fid-0
  const char *str = "456";
  struct sbiret sret = sbi_ecall(
    SBI_EXT_DBCN,  // Extension ID
    SBI_EXT_DBCN_CONSOLE_WRITE,  // Function ID
    strlen(str),         // Number of bytes
    (unsigned long)str,  // Address Low
    0,                   // Address High
    0, 0, 0              // Unused
  );
  _info("debug_console_write: value=0x%x, error=%d\n", sret.value, sret.error);
  // DEBUGASSERT(sret.error == SBI_SUCCESS);
  // DEBUGASSERT(sret.value == strlen(str));

  // TODO: Not supported by SBI v1.0, this will return SBI_ERR_NOT_SUPPORTED
  // Print `789` to Debug Console.
  // Call sbi_debug_console_write_byte: EID 0x4442434E "DBCN", FID 2
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/master/src/ext-debug-console.adoc#function-console-write-byte-fid-2
  sret = sbi_ecall(SBI_EXT_DBCN, SBI_EXT_DBCN_CONSOLE_WRITE_BYTE, '7', 0, 0, 0, 0, 0);
  sret = sbi_ecall(SBI_EXT_DBCN, SBI_EXT_DBCN_CONSOLE_WRITE_BYTE, '8', 0, 0, 0, 0, 0);
  sret = sbi_ecall(SBI_EXT_DBCN, SBI_EXT_DBCN_CONSOLE_WRITE_BYTE, '9', 0, 0, 0, 0, 0);
  _info("debug_console_write_byte: value=0x%x, error=%d\n", sret.value, sret.error);
  // DEBUGASSERT(sret.error == SBI_SUCCESS);
  // DEBUGASSERT(sret.value == 0);

  // Get SBI Spec Version
  // Call sbi_get_spec_version: EID 0x10, FID 0
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/v1.0.0/riscv-sbi.adoc#41-function-get-sbi-specification-version-fid-0
  sret = sbi_ecall(SBI_EXT_BASE, SBI_EXT_BASE_GET_SPEC_VERSION, 0, 0, 0, 0, 0, 0);
  _info("get_spec_version: value=0x%x, error=%d\n", sret.value, sret.error);

  // Get SBI Implementation ID
  // Call sbi_get_impl_id: EID 0x10, FID 1
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/v1.0.0/riscv-sbi.adoc#42-function-get-sbi-implementation-id-fid-1
  sret = sbi_ecall(SBI_EXT_BASE, SBI_EXT_BASE_GET_IMP_ID, 0, 0, 0, 0, 0, 0);
  _info("get_impl_id: value=0x%x, error=%d\n", sret.value, sret.error);

  // Get SBI Implementation Version
  // Call sbi_get_impl_version: EID 0x10, FID 2
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/v1.0.0/riscv-sbi.adoc#43-function-get-sbi-implementation-version-fid-2
  sret = sbi_ecall(SBI_EXT_BASE, SBI_EXT_BASE_GET_IMP_VERSION, 0, 0, 0, 0, 0, 0);
  _info("get_impl_version: value=0x%x, error=%d\n", sret.value, sret.error);

  // Get Machine Vendor ID
  // Call sbi_get_mvendorid: EID 0x10, FID 4
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/v1.0.0/riscv-sbi.adoc#45-function-get-machine-vendor-id-fid-4
  sret = sbi_ecall(SBI_EXT_BASE, SBI_EXT_BASE_GET_MVENDORID, 0, 0, 0, 0, 0, 0);
  _info("get_mvendorid: value=0x%x, error=%d\n", sret.value, sret.error);

  // Get Machine Architecture ID
  // Call sbi_get_marchid: EID 0x10, FID 5
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/v1.0.0/riscv-sbi.adoc#46-function-get-machine-architecture-id-fid-5
  sret = sbi_ecall(SBI_EXT_BASE, SBI_EXT_BASE_GET_MARCHID, 0, 0, 0, 0, 0, 0);
  _info("get_marchid: value=0x%x, error=%d\n", sret.value, sret.error);

  // Get Machine Implementation ID
  // Call sbi_get_mimpid: EID 0x10, FID 6
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/v1.0.0/riscv-sbi.adoc#47-function-get-machine-implementation-id-fid-6
  sret = sbi_ecall(SBI_EXT_BASE, SBI_EXT_BASE_GET_MIMPID, 0, 0, 0, 0, 0, 0);
  _info("get_mimpid: value=0x%x, error=%d\n", sret.value, sret.error);

  // Probe SBI Extension: Base Extension
  // Call sbi_probe_extension: EID 0x10, FID 3
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/v1.0.0/riscv-sbi.adoc#44-function-probe-sbi-extension-fid-3
  sret = sbi_ecall(SBI_EXT_BASE, SBI_EXT_BASE_PROBE_EXT, SBI_EXT_BASE, 0, 0, 0, 0, 0);
  _info("probe_extension[0x10]: value=0x%x, error=%d\n", sret.value, sret.error);

  // Probe SBI Extension: Debug Console Extension
  sret = sbi_ecall(SBI_EXT_BASE, SBI_EXT_BASE_PROBE_EXT, SBI_EXT_DBCN, 0, 0, 0, 0, 0);
  _info("probe_extension[0x4442434E]: value=0x%x, error=%d\n", sret.value, sret.error);

  // HART Get Status
  // Call sbi_hart_get_status: EID 0x48534D "HSM", FID 2
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/v1.0.0/riscv-sbi.adoc#93-function-hart-get-status-fid-2
  for (uintptr_t hart = 0; hart < 6; hart++) {
    sret = sbi_ecall(SBI_EXT_HSM, SBI_EXT_HSM_HART_GET_STATUS, hart, 0, 0, 0, 0, 0);
    _info("hart_get_status[%d]: value=0x%x, error=%d\n", hart, sret.value, sret.error);
  }

  // Set Timer
  // Call sbi_set_timer: EID 0x54494D45 "TIME", FID 0
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/v1.0.0/riscv-sbi.adoc#61-function-set-timer-fid-0
  sret = sbi_ecall(SBI_EXT_TIME, SBI_EXT_TIME_SET_TIMER, 0, 0, 0, 0, 0, 0);
  _info("set_timer: value=0x%x, error=%d\n", sret.value, sret.error);

  // System Reset: Shutdown
  // Call sbi_system_reset: EID 0x53525354 "SRST", FID 0
  // https://github.com/riscv-non-isa/riscv-sbi-doc/blob/v1.0.0/riscv-sbi.adoc#101-function-system-reset-fid-0
  // sret = sbi_ecall(SBI_EXT_SRST, SBI_EXT_SRST_RESET, SBI_SRST_RESET_TYPE_SHUTDOWN, SBI_SRST_RESET_REASON_NONE, 0, 0, 0, 0);
  // _info("system_reset[shutdown]: value=0x%x, error=%d\n", sret.value, sret.error);

  // System Reset: Cold Reboot
  // sret = sbi_ecall(SBI_EXT_SRST, SBI_EXT_SRST_RESET, SBI_SRST_RESET_TYPE_COLD_REBOOT, SBI_SRST_RESET_REASON_NONE, 0, 0, 0, 0);
  // _info("system_reset[cold_reboot]: value=0x%x, error=%d\n", sret.value, sret.error);

  // System Reset: Warm Reboot
  sret = sbi_ecall(SBI_EXT_SRST, SBI_EXT_SRST_RESET, SBI_SRST_RESET_TYPE_WARM_REBOOT, SBI_SRST_RESET_REASON_NONE, 0, 0, 0, 0);
  _info("system_reset[warm_reboot]: value=0x%x, error=%d\n", sret.value, sret.error);

  return OK;
}

// Make an ecall to OpenSBI. Based on
// https://github.com/apache/nuttx/blob/master/arch/risc-v/src/common/supervisor/riscv_sbi.c#L52-L77
// https://github.com/riscv-software-src/opensbi/blob/master/firmware/payloads/test_main.c
static struct sbiret sbi_ecall(unsigned int extid, unsigned int fid,
                               uintptr_t parm0, uintptr_t parm1,
                               uintptr_t parm2, uintptr_t parm3,
                               uintptr_t parm4, uintptr_t parm5)
{
  struct sbiret ret;
  register long r0 asm("a0") = (long)(parm0);
  register long r1 asm("a1") = (long)(parm1);
  register long r2 asm("a2") = (long)(parm2);
  register long r3 asm("a3") = (long)(parm3);
  register long r4 asm("a4") = (long)(parm4);
  register long r5 asm("a5") = (long)(parm5);
  register long r6 asm("a6") = (long)(fid);
  register long r7 asm("a7") = (long)(extid);

  asm volatile
    (
     "ecall"
     : "+r"(r0), "+r"(r1)
     : "r"(r2), "r"(r3), "r"(r4), "r"(r5), "r"(r6), "r"(r7)
     : "memory"
     );

  ret.error = r0;
  ret.value = r1;
  return ret;
}

/* Output Log:
Starting kernel ...

clk u5_dw_i2c_clk_core already disabled
clk u5_dw_i2c_clk_apb already disabled
BC123test_opensbi: debug_console_write: value=0x0, error=-2
test_opensbi: debug_console_write_byte: value=0x0, error=-2
test_opensbi: get_spec_version: value=0x1000000, error=0
test_opensbi: get_impl_id: value=0x1, error=0
test_opensbi: get_impl_version: value=0x10002, error=0
test_opensbi: get_mvendorid: value=0x489, error=0
test_opensbi: get_marchid: value=0x7, error=0
test_opensbi: get_mimpid: value=0x4210427, error=0
test_opensbi: probe_extension[0x10]: value=0x1, error=0
test_opensbi: probe_extension[0x4442434E]: value=0x0, error=0
test_opensbi: hart_get_status[0]: value=0x1, error=0
test_opensbi: hart_get_status[1]: value=0x0, error=0
test_opensbi: hart_get_status[2]: value=0x1, error=0
test_opensbi: hart_get_status[3]: value=0x1, error=0
test_opensbi: hart_get_status[4]: value=0x1, error=0
test_opensbi: hart_get_status[5]: value=0x0, error=-3
test_opensbi: set_timer: value=0x0, error=0
test_opensbi: system_reset[warm_reboot]: value=0x0, error=-2

NuttShell (NSH) NuttX-12.0.3
nsh> 
*/
