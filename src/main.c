/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**@file
 *
 * @defgroup ble_sdk_app_bootloader_main main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file.
 *
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
#include "nrfx_power.h"

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
#include "app_timer.h"
#include "nrf_error.h"

#include "boards.h"

#include "pstorage_platform.h"
#include "nrf_mbr.h"
#include "nrf_wdt.h"
#include "pstorage.h"

#include "nrf_nvmc.h"

#ifdef NRF52840_XXAA
#include "nrf_usbd.h"
#include "tusb.h"
#include "usb/msc_uf2.h"

void usb_init(void);
void usb_teardown(void);

#else

#define usb_init()
#define usb_teardown()

#endif


#define BOOTLOADER_VERSION_REGISTER         NRF_TIMER2->CC[0]

#define LED_BLINK_INTERVAL                  100
#define BOOTLOADER_STARTUP_DFU_INTERVAL     1000

/* Magic that written to NRF_POWER->GPREGRET by application when it wish to go into DFU
 * - BOOTLOADER_DFU_OTA_MAGIC used by BLEDfu service : SD is already init
 * - BOOTLOADER_DFU_OTA_FULLRESET_MAGIC entered by soft reset : SD is not init
 * - BOOTLOADER_DFU_SERIAL_MAGIC entered by soft reset : SD is not init
 *
 * Note: for BOOTLOADER_DFU_OTA_MAGIC Softdevice should not initialized. In other case SD must be initialized
 */
#define BOOTLOADER_DFU_OTA_MAGIC            BOOTLOADER_DFU_START             // 0xB1
#define BOOTLOADER_DFU_OTA_FULLRESET_MAGIC  0xA8
#define BOOTLOADER_DFU_SERIAL_MAGIC         0x4e

#define BUTTON_DFU                          BUTTON_1                         // Button used to enter SW update mode.
#define BUTTON_FRESET                       BUTTON_2                         // Button used in addition to DFU button, to force OTA DFU

#define SCHED_MAX_EVENT_DATA_SIZE           sizeof(app_timer_event_t)        /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    30                               /**< Maximum number of events in the scheduler queue. */

// Helper function
#define memclr(buffer, size)                memset(buffer, 0, size)
#define varclr(_var)                        memclr(_var, sizeof(*(_var)))
#define arrclr(_arr)                        memclr(_arr, sizeof(_arr))
#define arrcount(_arr)                      ( sizeof(_arr) / sizeof(_arr[0]) )


// These value must be the same with one in dfu_transport_ble.c
#define BLEGAP_EVENT_LENGTH             6
#define BLEGATT_ATT_MTU_MAX             247
enum { BLE_CONN_CFG_HIGH_BANDWIDTH = 1 };

// Adafruit for factory reset
#define APPDATA_ADDR_START                  (BOOTLOADER_REGION_START-DFU_APP_DATA_RESERVED)

#ifdef NRF52840_XXAA
STATIC_ASSERT( APPDATA_ADDR_START == 0xED000);
#else
STATIC_ASSERT( APPDATA_ADDR_START == 0x6D000);
#endif


void adafruit_factory_reset(void);


// Adafruit for Blink pattern
bool _fast_blink = false;
bool _ota_connected = false;

// true if ble, false if serial
bool _ota_update = false;

bool is_ota(void) { return _ota_update; }



/*
 * Blinking function, there are a few patterns
 * - DFU Serial     : LED Status blink
 * - DFU OTA        : LED Status & Conn blink at the same time
 * - DFU Flashing   : LED Status blink 2x fast
 * - Factory Reset  : LED Status blink 2x fast
 * - Fatal Error    : LED Status & Conn blink one after another
 */
static void blinky_handler(void)
{
  static uint8_t state = 0;
  static uint32_t count = 0;

  count++;

  // if not uploading then blink slow (interval/2)
  if ( !_fast_blink && count%2 ) return;

  state = 1-state;

  led_control(LED_RED, state);

  // Blink LED BLUE if OTA mode and not connected
  if (is_ota() && !_ota_connected)
  {
    led_control(LED_BLUE, state);
  }

  // Feed all Watchdog just in case application enable it (WDT last through a jump from application to bootloader)
  if ( nrf_wdt_started() )
  {
    for (uint8_t i=0; i<8; i++) nrf_wdt_reload_request_set(i);
  }
}

void SysTick_Handler(void)
{
  blinky_handler();
}

void led_blink_fast(bool enable)
{
  _fast_blink = enable;
}

void board_init(void)
{
  button_init(BUTTON_DFU);
  button_init(BUTTON_FRESET);
  NRFX_DELAY_US(100); // wait for the pin state is stable

  // LED init
  nrf_gpio_cfg_output(LED_RED);
  nrf_gpio_cfg_output(LED_BLUE);
  led_off(LED_RED);
  led_off(LED_BLUE);

  // Init scheduler
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

  // Init app timer (use RTC1)
  app_timer_init();

  // Configure Systick for led blinky
  extern uint32_t SystemCoreClock;
  SysTick_Config(SystemCoreClock/(1000/LED_BLINK_INTERVAL));
}

void board_teardown(void)
{
  // Disable systick, turn off LEDs
  SysTick->CTRL = 0;

  led_off(LED_BLUE);
  led_off(LED_RED);

  // Stop RTC1 used by app_timer
  NVIC_DisableIRQ(RTC1_IRQn);
  NRF_RTC1->EVTENCLR    = RTC_EVTEN_COMPARE0_Msk;
  NRF_RTC1->INTENCLR    = RTC_INTENSET_COMPARE0_Msk;
  NRF_RTC1->TASKS_STOP  = 1;
  NRF_RTC1->TASKS_CLEAR = 1;

  // disable usb
  usb_teardown();
}

void softdev_mbr_init(void)
{
  sd_mbr_command_t com = { .command = SD_MBR_COMMAND_INIT_SD };
  sd_mbr_command(&com);
}

/**
 * Initializes the SoftDevice and the BLE event interrupt.
 * @param[in] init_softdevice  true if SoftDevice should be initialized. The SoftDevice must only
 *                             be initialized if a chip reset has occured. Soft reset (jump ) from
 *                             application must not reinitialize the SoftDevice.
 */
uint32_t softdev_init(bool init_softdevice)
{
  if (init_softdevice) softdev_mbr_init();

  // map vector table to bootloader address
  APP_ERROR_CHECK( sd_softdevice_vector_table_base_set(BOOTLOADER_REGION_START) );

  // Enable Softdevice
  nrf_clock_lf_cfg_t clock_cfg =
  {
#ifdef NRF52840_XXAA // TODO use xtal source for feather52832
      .source       = NRF_CLOCK_LF_SRC_XTAL,
      .rc_ctiv      = 0,
      .rc_temp_ctiv = 0,
      .accuracy     = NRF_CLOCK_LF_ACCURACY_20_PPM
#else
      .source       = NRF_CLOCK_LF_SRC_RC,
      .rc_ctiv      = 16,
      .rc_temp_ctiv = 2,
      .accuracy     = NRF_CLOCK_LF_ACCURACY_20_PPM
#endif
  };

  APP_ERROR_CHECK( sd_softdevice_enable(&clock_cfg, app_error_fault_handler) );
  sd_nvic_EnableIRQ(SD_EVT_IRQn);

  /*------------- Configure BLE params  -------------*/
  extern uint32_t  __data_start__[]; // defined in linker
  uint32_t ram_start = (uint32_t) __data_start__;

  ble_cfg_t blecfg;

  // Configure the maximum number of connections.
  varclr(&blecfg);
  blecfg.gap_cfg.role_count_cfg.periph_role_count  = 1;
  blecfg.gap_cfg.role_count_cfg.central_role_count = 0;
  blecfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
  APP_ERROR_CHECK( sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &blecfg, ram_start) );

  // NRF_DFU_BLE_REQUIRES_BONDS
  varclr(&blecfg);
  blecfg.gatts_cfg.service_changed.service_changed = 1;
  APP_ERROR_CHECK( sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &blecfg, ram_start) );

  // ATT MTU
  varclr(&blecfg);
  blecfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_HIGH_BANDWIDTH;
  blecfg.conn_cfg.params.gatt_conn_cfg.att_mtu = BLEGATT_ATT_MTU_MAX;
  APP_ERROR_CHECK( sd_ble_cfg_set(BLE_CONN_CFG_GATT, &blecfg, ram_start) );

  // Event Length + HVN queue + WRITE CMD queue setting affecting bandwidth
  varclr(&blecfg);
  blecfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_HIGH_BANDWIDTH;
  blecfg.conn_cfg.params.gap_conn_cfg.conn_count   = 1;
  blecfg.conn_cfg.params.gap_conn_cfg.event_length = BLEGAP_EVENT_LENGTH;
  APP_ERROR_CHECK( sd_ble_cfg_set(BLE_CONN_CFG_GAP, &blecfg, ram_start) );

  // Enable BLE stack.
  // Note: Interrupt state (enabled, forwarding) is not work properly if not enable ble
  APP_ERROR_CHECK( sd_ble_enable(&ram_start) );

  return NRF_SUCCESS;
}

void softdev_teardown(void)
{
  sd_softdevice_disable();
}



int main(void)
{
  // SD is already Initialized in case of BOOTLOADER_DFU_OTA_MAGIC
  bool sd_inited = (NRF_POWER->GPREGRET == BOOTLOADER_DFU_OTA_MAGIC);

  // Start Bootloader in BLE OTA mode
  _ota_update = (NRF_POWER->GPREGRET == BOOTLOADER_DFU_OTA_MAGIC) ||
                (NRF_POWER->GPREGRET == BOOTLOADER_DFU_OTA_FULLRESET_MAGIC);

  // start bootloader either serial or ble
  bool dfu_start = _ota_update || (NRF_POWER->GPREGRET == BOOTLOADER_DFU_SERIAL_MAGIC);

  // Clear GPREGRET if it is our values
  if (dfu_start) NRF_POWER->GPREGRET = 0;

  // Save bootloader version to pre-defined register, retrieved by application
  BOOTLOADER_VERSION_REGISTER = (MK_BOOTLOADER_VERSION);

  // This check ensures that the defined fields in the bootloader corresponds with actual setting in the chip.
  APP_ERROR_CHECK_BOOL(*((uint32_t *)NRF_UICR_BOOT_START_ADDRESS) == BOOTLOADER_REGION_START);

  board_init();
  bootloader_init();

  if (bootloader_dfu_sd_in_progress())
  {
    APP_ERROR_CHECK( bootloader_dfu_sd_update_continue() );
    softdev_init(!sd_inited);
    sd_inited = true;
    APP_ERROR_CHECK( bootloader_dfu_sd_update_finalize() );
  }
  else
  {
    // softdev_init();
  }

  /*------------- Determine DFU mode (Serial, OTA, FRESET or normal) -------------*/
  // DFU button pressed
  dfu_start  = dfu_start || button_pressed(BUTTON_DFU);

  // DFU + FRESET are pressed --> OTA
  _ota_update = _ota_update  || ( button_pressed(BUTTON_DFU) && button_pressed(BUTTON_FRESET) ) ;

  if ( dfu_start || !bootloader_app_is_valid(DFU_BANK_0_REGION_START) )
  {
    // Enable BLE if in OTA
    if ( _ota_update )
    {
      softdev_init(!sd_inited);
      sd_inited = true;
    }

    usb_init();

    // Initiate an update of the firmware.
    APP_ERROR_CHECK( bootloader_dfu_start(_ota_update, 0) );
  }
#ifdef NRF52832_XXAA
  else
  {
    /* Adafruit Modification
     * Even DFU is not active, we still force an 1000 ms dfu serial mode when startup
     * to support auto programming from Arduino IDE */
    bootloader_dfu_start(false, BOOTLOADER_STARTUP_DFU_INTERVAL);
  }
#endif

  // we are all done with DFU, disable soft device
  softdev_teardown();

  /*------------- Adafruit Factory reset -------------*/
  if ( !button_pressed(BUTTON_DFU) && button_pressed(BUTTON_FRESET) )
  {
    adafruit_factory_reset();
  }

  /*------------- Reset used prph and jump to application -------------*/
  board_teardown();

  if (bootloader_app_is_valid(DFU_BANK_0_REGION_START) && !bootloader_dfu_sd_in_progress())
  {
    // MBR must be init before start application
    if ( !sd_inited ) softdev_mbr_init();

    // Select a bank region to use as application region.
    // @note: Only applications running from DFU_BANK_0_REGION_START is supported.
    bootloader_app_start(DFU_BANK_0_REGION_START);
  }

  NVIC_SystemReset();
}


//--------------------------------------------------------------------+
// FACTORY RESET
//--------------------------------------------------------------------+

// Perform factory reset to erase Application + Data
void adafruit_factory_reset(void)
{
  // Blink fast RED and turn on BLUE when erasing
  led_blink_fast(true);
  led_on(LED_BLUE);

  // clear all App Data if any
  if ( DFU_APP_DATA_RESERVED )
  {
    nrf_nvmc_page_erase(APPDATA_ADDR_START);
  }

  // Only need to erase the 1st page of Application code to make it invalid
  nrf_nvmc_page_erase(DFU_BANK_0_REGION_START);

  // back to normal
  led_blink_fast(false);
  led_off(LED_BLUE);
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

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
  app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

uint32_t tusb_hal_millis(void)
{
  return ( ( ((uint64_t)app_timer_cnt_get())*1000*(APP_TIMER_CONFIG_RTC_FREQUENCY+1)) / APP_TIMER_CLOCK_FREQ );
}

/*------------------------------------------------------------------*/
/* SoftDevice Event handler
 *------------------------------------------------------------------*/

// Process BLE event from SD
uint32_t proc_ble(void)
{
  __ALIGN(4) uint8_t ev_buf[ BLE_EVT_LEN_MAX(BLEGATT_ATT_MTU_MAX) ];
  uint16_t ev_len = BLE_EVT_LEN_MAX(BLEGATT_ATT_MTU_MAX);

  // Get BLE Event
  uint32_t err = sd_ble_evt_get(ev_buf, &ev_len);

  // Handle valid event, ignore error
  if( NRF_SUCCESS == err)
  {
    ble_evt_t* evt = (ble_evt_t*) ev_buf;

    switch (evt->header.evt_id)
    {
      case BLE_GAP_EVT_CONNECTED:
        _ota_connected = true;
        led_on(LED_BLUE);
      break;

      case BLE_GAP_EVT_DISCONNECTED:
        _ota_connected = false;
        led_off(LED_BLUE);
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
  uint32_t soc_evt;
  uint32_t err = sd_evt_get(&soc_evt);

  if (NRF_SUCCESS == err)
  {
    pstorage_sys_event_handler(soc_evt);

#ifdef NRF52840_XXAA
    extern void tusb_hal_nrf_power_event(uint32_t event);
    /*------------- usb power event handler -------------*/
    int32_t usbevt = (soc_evt == NRF_EVT_POWER_USB_DETECTED   ) ? NRFX_POWER_USB_EVT_DETECTED:
                     (soc_evt == NRF_EVT_POWER_USB_POWER_READY) ? NRFX_POWER_USB_EVT_READY   :
                     (soc_evt == NRF_EVT_POWER_USB_REMOVED    ) ? NRFX_POWER_USB_EVT_REMOVED : -1;

    if ( usbevt >= 0) tusb_hal_nrf_power_event(usbevt);
#endif
  }

  return err;
}

void ada_sd_task(void* evt_data, uint16_t evt_size)
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
  app_sched_event_put(NULL, 0, ada_sd_task);
}


