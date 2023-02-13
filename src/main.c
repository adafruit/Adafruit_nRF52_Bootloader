/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * -# Receive start data packet.
 * -# Based on start packet, prepare NVM area to store received data.
 * -# Receive data packet.
 * -# Validate data packet.
 * -# Write Data packet to NVM.
 * -# If not finished - Wait for next packet.
 * -# Receive stop data packet.
 * -# Activate Image, boot application.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

#include "nrfx.h"
#include "nrf_clock.h"
#include "nrfx_power.h"
#include "nrfx_pwm.h"

#include "nordic_common.h"
#include "sdk_common.h"
#include "dfu_transport.h"
#include "bootloader.h"
#include "bootloader_util.h"

#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_nvic.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "nrf.h"
#include "ble_hci.h"
#include "app_scheduler.h"
#include "nrf_error.h"

#include "boards.h"

#include "pstorage_platform.h"
#include "nrf_mbr.h"
#include "pstorage.h"
#include "nrfx_nvmc.h"

#ifdef NRF_USBD
#include "uf2/uf2.h"
#include "nrf_usbd.h"
#include "tusb.h"

void usb_init(bool cdc_only);
void usb_teardown(void);

// tinyusb function that handles power event (detected, ready, removed)
// We must call it within SD's SOC event handler, or set it as power event handler if SD is not enabled.
extern void tusb_hal_nrf_power_event(uint32_t event);

#else

#define usb_init(x)       led_state(STATE_USB_MOUNTED) // mark nrf52832 as mounted
#define usb_teardown()

#endif

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

/*
 * Blinking patterns:
 * - DFU Serial     : LED Status blink
 * - DFU OTA        : LED Status & Conn blink at the same time
 * - DFU Flashing   : LED Status blink 2x fast
 * - Factory Reset  : LED Status blink 2x fast
 * - Fatal Error    : LED Status & Conn blink one after another
 */

/* Magic that written to NRF_POWER->GPREGRET by application when it wish to go into DFU
 * - DFU_MAGIC_OTA_APPJUM        : used by BLEDfu service, SD is already inited
 * - DFU_MAGIC_OTA_RESET         : entered by soft reset, SD is not inited yet
 * - DFU_MAGIC_SERIAL_ONLY_RESET : with CDC interface only
 * - DFU_MAGIC_UF2_RESET         : with CDC and MSC interfaces
 * - DFU_MAGIC_SKIP              : skip DFU entirely including double reset delay,
 *                                 Can be used with systemoff or quick reset to app
 *
 * Note: for DFU_MAGIC_OTA_APPJUM Softdevice must not initialized.
 * since it is already in application. In all other case of OTA SD must be initialized
 */
#define DFU_MAGIC_OTA_APPJUM            BOOTLOADER_DFU_START  // 0xB1
#define DFU_MAGIC_OTA_RESET             0xA8
#define DFU_MAGIC_SERIAL_ONLY_RESET     0x4e
#define DFU_MAGIC_UF2_RESET             0x57
#define DFU_MAGIC_SKIP                  0x6d

#define DFU_DBL_RESET_MAGIC             0x5A1AD5      // SALADS
#define DFU_DBL_RESET_APP               0x4ee5677e
#define DFU_DBL_RESET_DELAY             500
#define DFU_DBL_RESET_MEM               0x20007F7C

#define BOOTLOADER_VERSION_REGISTER     NRF_TIMER2->CC[0]
#define DFU_SERIAL_STARTUP_INTERVAL     1000

// Allow for using reset button essentially to swap between application and bootloader.
// This is controlled by a flag in the app and is the behavior of CPX and all Arcade boards when using MakeCode.
// DFU_DBL_RESET magic is used to determined which mode is entered
#define APP_ASKS_FOR_SINGLE_TAP_RESET() (*((uint32_t*)(DFU_BANK_0_REGION_START + 0x200)) == 0x87eeb07c)

// These value must be the same with one in dfu_transport_ble.c
#define BLEGAP_EVENT_LENGTH             6
#define BLEGATT_ATT_MTU_MAX             23
enum { BLE_CONN_CFG_HIGH_BANDWIDTH = 1 };

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
uint32_t* dbl_reset_mem = ((uint32_t*)  DFU_DBL_RESET_MEM );

// true if ble, false if serial
bool _ota_dfu = false;
bool _ota_connected = false;
bool _sd_inited = false;

bool is_ota(void)
{
  return _ota_dfu;
}

static void check_dfu_mode(void);
static uint32_t ble_stack_init(void);

// The SoftDevice must only be initialized if a chip reset has occurred.
// Soft reset (jump ) from application must not reinitialize the SoftDevice.
static void mbr_init_sd(void)
{
  PRINTF("SD_MBR_COMMAND_INIT_SD\r\n");
  sd_mbr_command_t com = { .command = SD_MBR_COMMAND_INIT_SD };
  sd_mbr_command(&com);
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
int main(void)
{
  // Populate Boot Address and MBR Param into MBR if not already
  // MBR_BOOTLOADER_ADDR/MBR_PARAM_PAGE_ADDR are used if available, else UICR registers are used
  // Note: skip it for now since this will prevent us to change the size of bootloader in the future
  // bootloader_mbr_addrs_populate();

  // Save bootloader version to pre-defined register, retrieved by application
  // TODO move to CF2
  BOOTLOADER_VERSION_REGISTER = (MK_BOOTLOADER_VERSION);

  board_init();
  bootloader_init();

  PRINTF("Bootloader Start\r\n");

  led_state(STATE_BOOTLOADER_STARTED);

  // When updating SoftDevice, bootloader will reset before swapping SD
  if (bootloader_dfu_sd_in_progress())
  {
    led_state(STATE_WRITING_STARTED);

    bootloader_dfu_sd_update_continue();
    bootloader_dfu_sd_update_finalize();

    led_state(STATE_WRITING_FINISHED);
  }

  // Check all inputs and enter DFU if needed
  // Return when DFU process is complete (or not entered at all)
  check_dfu_mode();

  // Reset peripherals
  board_teardown();

  /* Jump to application if valid
   * "Master Boot Record and SoftDevice initializaton procedure"
   * - SD_MBR_COMMAND_INIT_SD (if not already)
   * - sd_softdevice_disable()
   * - sd_softdevice_vector_table_base_set(APP_ADDR)
   * - jump to App reset
   */

  if (bootloader_app_is_valid() && !bootloader_dfu_sd_in_progress())
  {
    PRINTF("App is valid\r\n");
    if ( is_sd_existed() )
    {
      // MBR forward IRQ to SD (if not already)
      if ( !_sd_inited ) mbr_init_sd();

      // Make sure SD is disabled
      sd_softdevice_disable();
    }

    // clear in case we kept DFU_DBL_RESET_APP there
    (*dbl_reset_mem) = 0;

    // start application
    bootloader_app_start();
  }

  NVIC_SystemReset();
}

static void check_dfu_mode(void)
{
  uint32_t const gpregret = NRF_POWER->GPREGRET;

  // SD is already Initialized in case of BOOTLOADER_DFU_OTA_MAGIC
  _sd_inited = (gpregret == DFU_MAGIC_OTA_APPJUM);

  // Start Bootloader in BLE OTA mode
  _ota_dfu = (gpregret == DFU_MAGIC_OTA_APPJUM) || (gpregret == DFU_MAGIC_OTA_RESET);

  // Serial only mode
  bool const serial_only_dfu = (gpregret == DFU_MAGIC_SERIAL_ONLY_RESET);
  bool const uf2_dfu         = (gpregret == DFU_MAGIC_UF2_RESET);
  bool const dfu_skip        = (gpregret == DFU_MAGIC_SKIP);

  bool const reason_reset_pin = (NRF_POWER->RESETREAS & POWER_RESETREAS_RESETPIN_Msk) ? true : false;

  // start either serial, uf2 or ble
  bool dfu_start = _ota_dfu || serial_only_dfu || uf2_dfu ||
                    (((*dbl_reset_mem) == DFU_DBL_RESET_MAGIC) && reason_reset_pin);

  // Clear GPREGRET if it is our values
  if (dfu_start || dfu_skip) NRF_POWER->GPREGRET = 0;

  // skip dfu entirely
  if (dfu_skip) return;

  /*------------- Determine DFU mode (Serial, OTA, FRESET or normal) -------------*/
  // DFU button pressed
  dfu_start = dfu_start || button_pressed(BUTTON_DFU);

  // DFU + FRESET are pressed --> OTA
  _ota_dfu = _ota_dfu  || ( button_pressed(BUTTON_DFU) && button_pressed(BUTTON_FRESET) ) ;

  bool const valid_app = bootloader_app_is_valid();
  bool const just_start_app = valid_app && !dfu_start && (*dbl_reset_mem) == DFU_DBL_RESET_APP;

  if (!just_start_app && APP_ASKS_FOR_SINGLE_TAP_RESET()) dfu_start = 1;

  // App mode: Double Reset detection or DFU startup for nrf52832
  if ( ! (just_start_app || dfu_start || !valid_app) )
  {
#ifdef NRF52832_XXAA
    /* Even DFU is not active, we still force an 1000 ms dfu serial mode when startup
     * to support auto programming from Arduino IDE
     *
     * Note: Double Reset WONT work with nrf52832 since all its SRAM got cleared with GPIO reset.
     */
    bootloader_dfu_start(false, DFU_SERIAL_STARTUP_INTERVAL, false);
#else
    // Note: RESETREAS is not clear by bootloader, it should be cleared by application upon init()
    if (reason_reset_pin)
    {
      // Register our first reset for double reset detection
      (*dbl_reset_mem) = DFU_DBL_RESET_MAGIC;

      // if RST is pressed during this delay (double reset)--> if will enter dfu
      NRFX_DELAY_MS(DFU_DBL_RESET_DELAY);
    }
#endif
  }

  if (APP_ASKS_FOR_SINGLE_TAP_RESET())
  {
    (*dbl_reset_mem) = DFU_DBL_RESET_APP;
  }
  else
  {
    (*dbl_reset_mem) = 0;
  }

  // Enter DFU mode accordingly to input
  if ( dfu_start || !valid_app )
  {
    if ( _ota_dfu )
    {
      led_state(STATE_BLE_DISCONNECTED);

      if (!_sd_inited ) mbr_init_sd();
      _sd_inited = true;

      ble_stack_init();
    }
    else
    {
      led_state(STATE_USB_UNMOUNTED);
      usb_init(serial_only_dfu);
    }

    // Initiate an update of the firmware.
    if (APP_ASKS_FOR_SINGLE_TAP_RESET() || uf2_dfu || serial_only_dfu)
    {
      // If USB is not enumerated in 3s (eg. because we're running on battery), we restart into app.
       bootloader_dfu_start(_ota_dfu, 3000, true);
    }
    else
    {
      // No timeout if bootloader requires user action (double-reset).
       bootloader_dfu_start(_ota_dfu, 0, false);
    }

    if ( _ota_dfu )
    {
      sd_softdevice_disable();
    }else
    {
      usb_teardown();
    }
  }
}


// Initializes the SotdDevice by following SD specs section
// "Master Boot Record and SoftDevice initializaton procedure"
static uint32_t ble_stack_init(void)
{
  // Forward vector table to bootloader address so that we can handle BLE events
  sd_softdevice_vector_table_base_set(BOOTLOADER_REGION_START);

  // Enable Softdevice, Use Internal OSC to compatible with all boards
  nrf_clock_lf_cfg_t clock_cfg =
  {
      .source       = NRF_CLOCK_LF_SRC_RC,
      .rc_ctiv      = 16,
      .rc_temp_ctiv = 2,
      .accuracy     = NRF_CLOCK_LF_ACCURACY_250_PPM
  };

  sd_softdevice_enable(&clock_cfg, app_error_fault_handler);
  sd_nvic_EnableIRQ(SD_EVT_IRQn);

  /*------------- Configure BLE params  -------------*/
  extern uint32_t  __data_start__[]; // defined in linker
  uint32_t ram_start = (uint32_t) __data_start__;

  ble_cfg_t blecfg;

  // Configure the maximum number of connections.
  varclr(&blecfg);
  blecfg.gap_cfg.role_count_cfg.adv_set_count = 1;
  blecfg.gap_cfg.role_count_cfg.periph_role_count  = 1;
  blecfg.gap_cfg.role_count_cfg.central_role_count = 0;
  blecfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
  sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &blecfg, ram_start);

  // NRF_DFU_BLE_REQUIRES_BONDS
  varclr(&blecfg);
  blecfg.gatts_cfg.service_changed.service_changed = 1;
  sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &blecfg, ram_start);

  // ATT MTU
  varclr(&blecfg);
  blecfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_HIGH_BANDWIDTH;
  blecfg.conn_cfg.params.gatt_conn_cfg.att_mtu = BLEGATT_ATT_MTU_MAX;
  sd_ble_cfg_set(BLE_CONN_CFG_GATT, &blecfg, ram_start);

  // Event Length + HVN queue + WRITE CMD queue setting affecting bandwidth
  varclr(&blecfg);
  blecfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_HIGH_BANDWIDTH;
  blecfg.conn_cfg.params.gap_conn_cfg.conn_count   = 1;
  blecfg.conn_cfg.params.gap_conn_cfg.event_length = BLEGAP_EVENT_LENGTH;
  sd_ble_cfg_set(BLE_CONN_CFG_GAP, &blecfg, ram_start);

  // Enable BLE stack.
  // Note: Interrupt state (enabled, forwarding) is not work properly if not enable ble
  sd_ble_enable(&ram_start);

#if 0
  ble_opt_t  opt;
  varclr(&opt);
  opt.common_opt.conn_evt_ext.enable = 1; // enable Data Length Extension
  sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
#endif

  return NRF_SUCCESS;
}


//--------------------------------------------------------------------+
// Error Handler
//--------------------------------------------------------------------+
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
  volatile uint32_t* ARM_CM_DHCSR =  ((volatile uint32_t*) 0xE000EDF0UL); /* Cortex M CoreDebug->DHCSR */
  if ( (*ARM_CM_DHCSR) & 1UL ) __asm("BKPT #0\n"); /* Only halt mcu if debugger is attached */
  NVIC_SystemReset();
}

void assert_nrf_callback (uint16_t line_num, uint8_t const * p_file_name)
{
  app_error_fault_handler(0xDEADBEEF, 0, 0);
}

/*------------------------------------------------------------------*/
/* SoftDevice Event handler
 *------------------------------------------------------------------*/

// Process BLE event from SD
uint32_t proc_ble(void)
{
  __ALIGN(4) uint8_t ev_buf[ BLE_EVT_LEN_MAX(BLEGATT_ATT_MTU_MAX) ];
  uint16_t ev_len = BLE_EVT_LEN_MAX(BLEGATT_ATT_MTU_MAX);

  // Init header
  ble_evt_t* evt = (ble_evt_t*) ev_buf;
  evt->header.evt_id = BLE_EVT_INVALID;

  // Get BLE Event
  uint32_t err = sd_ble_evt_get(ev_buf, &ev_len);

  // Handle valid event, ignore error
  if( NRF_SUCCESS == err)
  {
    switch (evt->header.evt_id)
    {
      case BLE_GAP_EVT_CONNECTED:
        _ota_connected = true;
        led_state(STATE_BLE_CONNECTED);
      break;

      case BLE_GAP_EVT_DISCONNECTED:
        _ota_connected = false;
        led_state(STATE_BLE_DISCONNECTED);
      break;

      default: break;
    }

    // from dfu_transport_ble
    extern void ble_evt_dispatch(ble_evt_t * p_ble_evt);
    ble_evt_dispatch(evt);
  }

  return err;
}

// process SOC event from SD
uint32_t proc_soc(void)
{
  uint32_t soc_evt = 0;
  uint32_t err = sd_evt_get(&soc_evt);

  if (NRF_SUCCESS == err)
  {
    pstorage_sys_event_handler(soc_evt);

#ifdef NRF_USBD
    /*------------- usb power event handler -------------*/
    int32_t usbevt = (soc_evt == NRF_EVT_POWER_USB_DETECTED   ) ? NRFX_POWER_USB_EVT_DETECTED:
                     (soc_evt == NRF_EVT_POWER_USB_POWER_READY) ? NRFX_POWER_USB_EVT_READY   :
                     (soc_evt == NRF_EVT_POWER_USB_REMOVED    ) ? NRFX_POWER_USB_EVT_REMOVED : -1;

    if ( usbevt >= 0) tusb_hal_nrf_power_event((uint32_t) usbevt);
#endif
  }

  return err;
}

void proc_sd_task(void* evt_data, uint16_t evt_size)
{
  (void) evt_data;
  (void) evt_size;

  // process BLE and SOC until there is no more events
  while( (NRF_ERROR_NOT_FOUND != proc_ble()) || (NRF_ERROR_NOT_FOUND != proc_soc()) )
  {

  }
}

void SD_EVT_IRQHandler(void)
{
  // Use App Scheduler to defer handling code in non-isr context
  app_sched_event_put(NULL, 0, proc_sd_task);
}


//--------------------------------------------------------------------+
// RTT printf retarget for Debug
//--------------------------------------------------------------------+
#ifdef CFG_DEBUG

#include "SEGGER_RTT.h"

__attribute__ ((used))
int _write (int fhdl, const void *buf, size_t count)
{
  (void) fhdl;
  SEGGER_RTT_Write(0, (char*) buf, (int) count);
  return count;
}

__attribute__ ((used))
int _read (int fhdl, char *buf, size_t count)
{
  (void) fhdl;
  return SEGGER_RTT_Read(0, buf, count);
}


#endif
