/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_bringup.c
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
#include <syslog.h>
#ifdef CONFIG_I2C
#  include <nuttx/i2c/i2c_master.h>
#endif
#include <nuttx/kmalloc.h>
#ifdef CONFIG_MPU60X0_I2C
#  include <nuttx/sensors/mpu60x0.h>
#endif

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#  include "pinephone_display.h"
#endif

#include "a64_twi.h"
#include "pinephone.h"
#include "pinephone_pmic.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int pinephone_bringup(void)
{
  int ret;
#ifdef CONFIG_I2C
  int i2c_bus;
  struct i2c_master_s *i2c;
#ifdef CONFIG_MPU60X0_I2C
  struct mpu_config_s *mpu_config;
#endif
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
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

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }

  /* Render the Test Pattern */

  pinephone_display_test_pattern();
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_A64_TWI1)
  i2c_bus = 1;
  i2c = a64_i2cbus_initialize(i2c_bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", i2c_bus);
    }
  else
    {
#if defined(CONFIG_SYSTEM_I2CTOOL)
      ret = i2c_register(i2c, i2c_bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 i2c_bus, ret);
        }
#endif

#ifdef CONFIG_MPU60X0_I2C
      /* Init PMIC */

      ret = pinephone_pmic_init();
      if (ret < 0)
        {
          syslog(LOG_ERR, "Init PMIC failed: %d\n", ret);
          return ret;
        }

      /* Wait 15 milliseconds for power supply and power-on init */

      up_mdelay(15);

      mpu_config = kmm_zalloc(sizeof(struct mpu_config_s));
      if (mpu_config == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to allocate mpu60x0 driver\n");
        }
      else
        {
          mpu_config->i2c = i2c;
          mpu_config->addr = 0x68;
          mpu60x0_register("/dev/imu0", mpu_config);
        }
#endif
    }
#endif

  ////TODO: Begin
#if defined(CONFIG_I2C) && defined(CONFIG_A64_TWI0)
  i2c_bus = 0;
  i2c = a64_i2cbus_initialize(i2c_bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", i2c_bus);
    }
  else
    {
      void touch_panel_initialize(struct i2c_master_s *i2c);
      touch_panel_initialize(i2c);
    }
#endif
  ////TODO: End

  UNUSED(ret);
  return OK;
}

// Testing Touch Panel
#include <nuttx/arch.h>
#include <debug.h>
#include "arm64_arch.h"

////#define TEST_INTERRUPT
#ifdef TEST_INTERRUPT
// Test Touch Panel Interrupt
// Touch Panel Interrupt (CTP-INT) is at PH4
#define CTP_INT (PIO_EINT | PIO_PORT_PIOH | PIO_PIN4)
#define CTP_INT_PIN 4

// IRQ for Touch Panel Interrupt (PH)
#define PH_EINT 53

// Interrupt Handler for Touch Panel
static int touch_panel_interrupt(int irq, void *context, void *arg)
{
  // Poll the Touch Panel Interrupt as GPIO Input
  static bool prev_val = false;

  // Read the GPIO Input
  bool val = a64_pio_read(CTP_INT);

  // Print if value has changed
  if (val != prev_val) {
    if (val) { up_putc('+'); }
    else     { up_putc('-'); }
  }
  prev_val = val;
  return OK;
}

// Register the Interrupt Handler for Touch Panel
void touch_panel_initialize(void)
{
  int ret;

  // Configure the Touch Panel Interrupt
  ret = a64_pio_config(CTP_INT);
  DEBUGASSERT(ret == 0);

  // Un-mask the interrupt by setting the corresponding bit in the PIO INT CTL register.
  int pin = CTP_INT_PIN;

  // PH_EINT_CTL_REG (Interrupt Control Register for PH4) at Offset 0x250
  #define PH_EINT_CTL_REG (0x1c20800 + 0x250)
  _info("v=0x%x, m=0x%x, a=0x%x\n", PIO_INT_CTL(pin), PIO_INT_CTL(pin), PH_EINT_CTL_REG);
  // Shows touch_panel_initialize: v=0x10, m=0x10, a=0x1c20a50

  // Enter Critical Section
  irqstate_t flags;
  flags = enter_critical_section();

  // Enable the Touch Panel Interrupt
  modreg32(
    PIO_INT_CTL(pin),  // Value
    PIO_INT_CTL(pin),  // Mask
    PH_EINT_CTL_REG    // Address
  );

  // Leave Critical Section
  leave_critical_section(flags);

  // TODO: Disable all external PIO interrupts
  // putreg32(0, A1X_PIO_INT_CTL);

  // Attach the PIO interrupt handler
  if (irq_attach(PH_EINT, touch_panel_interrupt, NULL) < 0)
    {
      _err("irq_attach failed\n");
      return;
    }

  // And enable the PIO interrupt
  up_enable_irq(PH_EINT);
}

#else
// Test Touch Panel Interrupt by Polling as GPIO Input.
// Touch Panel Interrupt (CTP-INT) is at PH4.
// Configure for GPIO Input
#define CTP_INT (PIO_INPUT | PIO_PORT_PIOH | PIO_PIN4)

static void touch_panel_read(struct i2c_master_s *i2c);
static int touch_panel_i2c_read(
  struct i2c_master_s *i2c,  // I2C Bus
  uint16_t reg,  // I2C Register
  uint8_t *buf,  // Receive Buffer
  size_t buflen  // Receive Buffer Size
);

// Poll for Touch Panel Interrupt (PH4) by reading as GPIO Input
void touch_panel_initialize(struct i2c_master_s *i2c)
{

  // Configure the Touch Panel Interrupt for GPIO Input
  int ret = a64_pio_config(CTP_INT);
  DEBUGASSERT(ret == 0);

  // Poll the Touch Panel Interrupt as GPIO Input
  bool prev_val = false;
  for (int i = 0; i < 6000; i++) {  // Poll for 60 seconds

    // Read the GPIO Input
    bool val = a64_pio_read(CTP_INT);

    // If value has changed...
    if (val != prev_val) {

      // Print the value
      if (val) { up_putc('+'); }
      else     { up_putc('-'); }
      prev_val = val;

      // If we have just transitioned from Low to High...
      if (val) {

        // Read the Touch Panel over I2C
        touch_panel_read(i2c);
      }
    }

    // Wait a while
    up_mdelay(10);
  }
}

// ReadOnly registers (device and coordinates info)
// Product ID (LSB 4 bytes)
#define GOODIX_REG_ID 0x8140
// Firmware version (LSB 2 bytes)
#define GOODIX_REG_FW_VER 0x8144

// Current output X resolution (LSB 2 bytes)
#define GOODIX_READ_X_RES 0x8146
// Current output Y resolution (LSB 2 bytes)
#define GOODIX_READ_Y_RES 0x8148
// Module vendor ID
#define GOODIX_READ_VENDOR_ID 0x814A

#define GOODIX_READ_COORD_ADDR 0x814E

#define GOODIX_POINT1_X_ADDR 0x8150

// Read Touch Panel over I2C
static void touch_panel_read(struct i2c_master_s *i2c)
{
  // Read the Product ID
  uint8_t id[4];
  touch_panel_i2c_read(i2c, GOODIX_REG_ID, id, sizeof(id));
  // Shows "39 31 37 53" or "917S"

  // Request for Touch Coordinates
  uint8_t req[1];
  touch_panel_i2c_read(i2c, GOODIX_READ_COORD_ADDR, req, sizeof(req));
  // Shows "80"

  // Read the Touch Coordinates
  uint8_t touch[6];
  touch_panel_i2c_read(i2c, GOODIX_POINT1_X_ADDR, touch, sizeof(touch));
  // Shows "80 00 00 00 00 00"
}

static int touch_panel_i2c_read(
  struct i2c_master_s *i2c,  // I2C Bus
  uint16_t reg,  // I2C Register
  uint8_t *buf,  // Receive Buffer
  size_t buflen  // Receive Buffer Size
) {
  uint32_t freq = 400000;  // 400 kHz
  uint16_t addr = 0x5d;  // Default I2C Address for Goodix GT917S
  uint8_t regbuf[2] = { reg >> 8, reg & 0xff };  // Flip the bytes

  // Erase the receive buffer
  memset(buf, 0xff, buflen);

  // Compose the I2C Messages
  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = freq,
      .addr      = addr,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    },
    {
      .frequency = freq,
      .addr      = addr,
      .flags     = I2C_M_READ,
      .buffer    = buf,
      .length    = buflen
    }
  };

  // Execute the I2C Transfer
  int ret = I2C_TRANSFER(i2c, msgv, 2);
  if (ret < 0) { _err("I2C Error: %d\n", ret); return ret; }

  // Dump the receive buffer
  infodumpbuffer("buf", buf, buflen);
  return OK;
}

///////////////////////////////////////////////////////////////////////////////

static uint8_t GT911_Config[] = {
		0x81, 0x00, 0x04, 0x58, 0x02, 0x0A, 0x0C, 0x20, 0x01, 0x08, 0x28, 0x05, 0x50, // 0x8047 - 0x8053
		0x3C, 0x0F, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x8054 - 0x8060
		0x00, 0x89, 0x2A, 0x0B, 0x2D, 0x2B, 0x0F, 0x0A, 0x00, 0x00, 0x01, 0xA9, 0x03, // 0x8061 - 0x806D
		0x2D, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, // 0x806E - 0x807A
		0x59, 0x94, 0xC5, 0x02, 0x07, 0x00, 0x00, 0x04, 0x93, 0x24, 0x00, 0x7D, 0x2C, // 0x807B - 0x8087
		0x00, 0x6B, 0x36, 0x00, 0x5D, 0x42, 0x00, 0x53, 0x50, 0x00, 0x53, 0x00, 0x00, // 0x8088	- 0x8094
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x8095 - 0x80A1
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x80A2 - 0x80AD
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, // 0x80AE - 0x80BA
		0x0C, 0x0E, 0x10, 0x12, 0x14, 0x16, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, // 0x80BB - 0x80C7
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x80C8 - 0x80D4
		0x02, 0x04, 0x06, 0x08, 0x0A, 0x0F, 0x10, 0x12, 0x16, 0x18, 0x1C, 0x1D, 0x1E, // 0x80D5 - 0x80E1
		0x1F, 0x20, 0x21, 0x22, 0x24, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, // 0x80E2 - 0x80EE
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x80EF - 0x80FB
		0x00, 0x00, 0xD6, 0x01 }; // 0x80FC - 0x8100
static GT911_Status_t CommunicationResult;
static uint8_t TxBuffer[200];
static uint8_t RxBuffer[200];

static void GT911_Reset(void);
static void GT911_CalculateCheckSum(void);
static GT911_Status_t GT911_SetCommandRegister(uint8_t command);
static GT911_Status_t GT911_GetProductID(uint32_t* id);
static GT911_Status_t GT911_SendConfig(void);
static GT911_Status_t GT911_GetStatus(uint8_t* status);
static GT911_Status_t GT911_SetStatus(uint8_t status);
/* API Implementation --------------------------------------------------------*/
GT911_Status_t GT911_Init(GT911_Config_t config){
//	Set X resolution
	GT911_Config[1] = config.X_Resolution & 0x00FF;
	GT911_Config[2] = (config.X_Resolution >> 8) & 0x00FF;
//	Set Y resolution
	GT911_Config[3] = config.Y_Resolution & 0x00FF;
	GT911_Config[4] = (config.Y_Resolution >> 8) & 0x00FF;
//  Set touch number
	GT911_Config[5] = config.Number_Of_Touch_Support;
//  set reverse Y
	GT911_Config[6] = 0;
	GT911_Config[6] |= config.ReverseY << 7;
//  set reverse X
	GT911_Config[6] |= config.ReverseX << 6;
//  set switch X2Y
	GT911_Config[6] |= config.SwithX2Y << 3;
//  set Sito
	GT911_Config[6] |= config.SoftwareNoiseReduction << 2;

	//Reset chip
	GT911_Reset();
	//Get product ID
	uint32_t productID = 0;
	CommunicationResult = GT911_GetProductID(&productID);
	if(CommunicationResult != GT911_OK){
		return CommunicationResult;
	}
	if(productID == 0){
		return GT911_NotResponse;
	}
	//Reset chip
	GT911_Reset();
	CommunicationResult = GT911_SendConfig();
	if(CommunicationResult != GT911_OK){
		return CommunicationResult;
	}
	GT911_SetCommandRegister(0x00);
	return GT911_OK;
}

GT911_Status_t GT911_ReadTouch(TouchCordinate_t *cordinate, uint8_t *number_of_cordinate) {
	uint8_t StatusRegister;
	GT911_Status_t Result = GT911_NotResponse;
	Result = GT911_GetStatus(&StatusRegister);
	if (Result != GT911_OK) {
		return Result;
	}
	if ((StatusRegister & 0x80) != 0) {
		*number_of_cordinate = StatusRegister & 0x0F;
		if (*number_of_cordinate != 0) {
			for (uint8_t i = 0; i < *number_of_cordinate; i++) {
				TxBuffer[0] = ((GOODIX_POINT1_X_ADDR + (i* 8)) & 0xFF00) >> 8;
				TxBuffer[1] = (GOODIX_POINT1_X_ADDR + (i* 8)) & 0xFF;
				GT911_I2C_Write(GOODIX_ADDRESS, TxBuffer, 2);
				GT911_I2C_Read(GOODIX_ADDRESS, RxBuffer, 6);
				cordinate[i].x = RxBuffer[0];
				cordinate[i].x = (RxBuffer[1] << 8) + cordinate[i].x;
				cordinate[i].y = RxBuffer[2];
				cordinate[i].y = (RxBuffer[3] << 8) + cordinate[i].y;
			}
		}
		GT911_SetStatus(0);
	}
	return GT911_OK;
}

//Private functions Implementation ---------------------------------------------------------*/
static void GT911_Reset(void){
	GT911_INT_Input();
	GT911_RST_Control(0);
	GT911_Delay(20);
	GT911_INT_Control(0);
	GT911_Delay(50);
	GT911_RST_Control(1);
	GT911_Delay(100);
	GT911_INT_Output();
	GT911_Delay(100);
}

static void GT911_CalculateCheckSum(void){
	GT911_Config[184] = 0;
	for(uint8_t i = 0 ; i < 184 ; i++){
		GT911_Config[184] += GT911_Config[i];
	}
	GT911_Config[184] = (~GT911_Config[184]) + 1;
}

static GT911_Status_t GT911_SetCommandRegister(uint8_t command){
	TxBuffer[0] = (GOODIX_REG_COMMAND & 0xFF00) >> 8;
	TxBuffer[1] = GOODIX_REG_COMMAND & 0xFF;
	TxBuffer[2] = command;
	return GT911_I2C_Write(GOODIX_ADDRESS, TxBuffer, 3);
}

static GT911_Status_t GT911_GetProductID(uint32_t* id){
	TxBuffer[0] = (GOODIX_REG_ID & 0xFF00) >> 8;
	TxBuffer[1] = GOODIX_REG_ID & 0xFF;
	GT911_Status_t Result = GT911_NotResponse;
	Result = GT911_I2C_Write(GOODIX_ADDRESS, TxBuffer, 2);
	if(Result == GT911_OK){
		Result = GT911_I2C_Read(GOODIX_ADDRESS, RxBuffer, 4);
		if( Result == GT911_OK){
			memcpy(id, RxBuffer, 4);
		}
	}
	return Result;
}

static GT911_Status_t GT911_SendConfig(void){
	GT911_CalculateCheckSum();
	TxBuffer[0] = (GOODIX_REG_CONFIG_DATA & 0xFF00) >> 8;
	TxBuffer[1] = GOODIX_REG_CONFIG_DATA & 0xFF;
	memcpy(&TxBuffer[2], GT911_Config, sizeof(GT911_Config));
	return GT911_I2C_Write(GOODIX_ADDRESS, TxBuffer, sizeof(GT911_Config) + 2);
}

static GT911_Status_t GT911_GetStatus(uint8_t* status){
	TxBuffer[0] = (GOODIX_READ_COORD_ADDR & 0xFF00) >> 8;
	TxBuffer[1] = GOODIX_READ_COORD_ADDR & 0xFF;
	GT911_Status_t Result = GT911_NotResponse;
	Result = GT911_I2C_Write(GOODIX_ADDRESS, TxBuffer, 2);
	if(Result == GT911_OK){
		Result = GT911_I2C_Read(GOODIX_ADDRESS, RxBuffer, 1);
		if( Result == GT911_OK){
			*status = RxBuffer[0];
		}
	}
	return Result;
}

static GT911_Status_t GT911_SetStatus(uint8_t status){
	TxBuffer[0] = (GOODIX_READ_COORD_ADDR & 0xFF00) >> 8;
	TxBuffer[1] = GOODIX_READ_COORD_ADDR & 0xFF;
	TxBuffer[2] = status;
	return GT911_I2C_Write(GOODIX_ADDRESS, TxBuffer, 3);
}

#ifdef NOTUSED
struct goodix_point_t {
	int id;
	int x;
	int y;
	int w;
	int p;
	int tool_type;
};

struct goodix_config_data {
	int length;
	u8 data[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH];
};

struct goodix_ts_platform_data {
	int irq_gpio;
	int rst_gpio;
	u32 irq_flags;
	u32 abs_size_x;
	u32 abs_size_y;
	u32 max_touch_id;
	u32 max_touch_width;
	u32 max_touch_pressure;
	u32 key_map[MAX_KEY_NUMS];
	u32 key_nums;
	u32 int_sync;
	u32 driver_send_cfg;
	u32 swap_x2y;
	u32 slide_wakeup;
	u32 auto_update;
	u32 auto_update_cfg;
	u32 esd_protect;
	u32 type_a_report;
	u32 power_off_sleep;
	u32 resume_in_workqueue;
	u32 pen_suppress_finger;
	struct goodix_config_data config;
};

static void touch_panel_setup(void)
{
	/* set parameters at here if you platform doesn't DTS */
	pdata->rst_gpio = GTP_RST_PORT;
	pdata->irq_gpio = GTP_INT_PORT;
	pdata->slide_wakeup = false;
	pdata->auto_update = true;
	pdata->auto_update_cfg = false;
	pdata->type_a_report = false;
	pdata->esd_protect = false;
	pdata->max_touch_id = GTP_MAX_TOUCH_ID;
	pdata->abs_size_x = GTP_DEFAULT_MAX_X;
	pdata->abs_size_y = GTP_DEFAULT_MAX_Y;
	pdata->max_touch_width = GTP_DEFAULT_MAX_WIDTH;
	pdata->max_touch_pressure = GTP_DEFAULT_MAX_PRESSURE;
}
#endif  // NOTUSED

#endif  // !TEST_INTERRUPT
