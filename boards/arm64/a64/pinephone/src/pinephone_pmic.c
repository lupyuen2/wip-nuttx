#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm64_internal.h"
#include "a64_rsb.h"
#include "pinephone_pmic.h"

/// Address of AXP803 PMIC on Reduced Serial Bus
#define AXP803_RT_ADDR 0x2d

static int pmic_write(
  uint8_t reg,
  uint8_t val
);
static int pmic_clrsetbits(
  uint8_t reg, 
  uint8_t clr_mask, 
  uint8_t set_mask
);

/// Init PMIC.
/// Based on https://lupyuen.github.io/articles/de#appendix-power-management-integrated-circuit
int pinephone_pmic_init(void)
{
  int ret;

  // Set DLDO1 Voltage to 3.3V
  // DLDO1 powers the Front Camera / USB HSIC / I2C Sensors
  // Register 0x15: DLDO1 Voltage Control (AXP803 Page 52)
  // Set Voltage (Bits 0 to 4) to 26 (2.6V + 0.7V = 3.3V)
  batinfo("Set DLDO1 Voltage to 3.3V\n");
  #define DLDO1_Voltage_Control 0x15
  #define DLDO1_Voltage (26 << 0)
  ret = pmic_write(DLDO1_Voltage_Control, DLDO1_Voltage);
  if (ret < 0)
    {
      baterr("Set DLDO1 failed: %d\n", ret);
      return ret;
    }

  // Power on DLDO1
  // Register 0x12: Output Power On-Off Control 2 (AXP803 Page 51)
  // Set DLDO1 On-Off Control (Bit 3) to 1 (Power On)
  #define Output_Power_On_Off_Control2 0x12
  #define DLDO1_On_Off_Control (1 << 3)
  ret = pmic_clrsetbits(Output_Power_On_Off_Control2, 0, DLDO1_On_Off_Control);
  if (ret < 0)
    {
      baterr("Power on DLDO1 failed: %d\n", ret);
      return ret;
    }

  // Set LDO Voltage to 3.3V
  // GPIO0LDO powers the Capacitive Touch Panel
  // Register 0x91: GPIO0LDO and GPIO0 High Level Voltage Setting (AXP803 Page 77)
  // Set GPIO0LDO and GPIO0 High Level Voltage (Bits 0 to 4) to 26 (2.6V + 0.7V = 3.3V)
  batinfo("Set LDO Voltage to 3.3V\n");
  #define GPIO0LDO_High_Level_Voltage_Setting 0x91
  #define GPIO0LDO_High_Level_Voltage (26 << 0)
  ret = pmic_write(GPIO0LDO_High_Level_Voltage_Setting, GPIO0LDO_High_Level_Voltage);
  if (ret < 0)
    {
      baterr("Set LDO failed: %d\n", ret);
      return ret;
    }

  // Enable LDO Mode on GPIO0
  // Register 0x90: GPIO0 (GPADC) Control (AXP803 Page 76)
  // Set GPIO0 Pin Function Control (Bits 0 to 2) to 0b11 (Low Noise LDO on)
  batinfo("Enable LDO mode on GPIO0\n");
  #define GPIO0_Control 0x90
  #define GPIO0_Pin_Function (0b11 << 0)
  ret = pmic_write(GPIO0_Control, GPIO0_Pin_Function);
  if (ret < 0)
    {
      baterr("Enable LDO failed: %d\n", ret);
      return ret;
    }

  // Set DLDO2 Voltage to 1.8V
  // DLDO2 powers the MIPI DSI Connector
  // Register 0x16: DLDO2 Voltage Control (AXP803 Page 52)
  // Set Voltage (Bits 0 to 4) to 11 (1.1V + 0.7V = 1.8V)
  batinfo("Set DLDO2 Voltage to 1.8V\n");
  #define DLDO2_Voltage_Control 0x16
  #define DLDO2_Voltage (11 << 0)
  ret = pmic_write(DLDO2_Voltage_Control, DLDO2_Voltage);
  if (ret < 0)
    {
      baterr("Set DLDO2 failed: %d\n", ret);
      return ret;
    }

  // Power on DLDO2
  // Register 0x12: Output Power On-Off Control 2 (AXP803 Page 51)
  // Set DLDO2 On-Off Control (Bit 4) to 1 (Power On)
  DEBUGASSERT(Output_Power_On_Off_Control2 == 0x12);
  #define DLDO2 (1 << 4)
  ret = pmic_clrsetbits(Output_Power_On_Off_Control2, 0x0, DLDO2);
  if (ret < 0)
    {
      baterr("Power on DLDO2 failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/// Write value to PMIC Register
static int pmic_write(
  uint8_t reg,
  uint8_t val
)
{
  // Write to AXP803 PMIC on Reduced Serial Bus
  batinfo("reg=0x%x, val=0x%x\n", reg, val);
  int ret = a64_rsb_write(AXP803_RT_ADDR, reg, val);
  if (ret < 0)
    {
      baterr("PMIC Write failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/// Clear and Set the PMIC Register Bits
static int pmic_clrsetbits(
  uint8_t reg, 
  uint8_t clr_mask, 
  uint8_t set_mask
)
{
  // Read from AXP803 PMIC on Reduced Serial Bus
  batinfo("reg=0x%x, clr_mask=0x%x, set_mask=0x%x\n", reg, clr_mask, set_mask);
  int ret = a64_rsb_read(AXP803_RT_ADDR, reg);
  if (ret < 0)
    {
      baterr("PMIC Read failed: %d\n", ret);
      return ret;
    }

  // Write to AXP803 PMIC on Reduced Serial Bus
  uint8_t regval = (ret & ~clr_mask) | set_mask;
  ret = a64_rsb_write(AXP803_RT_ADDR, reg, regval);
  if (ret < 0)
    {
      baterr("PMIC Write failed: %d\n", ret);
      return ret;
    }

  return OK;
}
