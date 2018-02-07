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

#include "dfu_transport.h"
#include "bootloader.h"
#include "bootloader_util.h"
#include "nordic_common.h"
#include "sdk_common.h"

#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "nrf.h"
#include "ble_hci.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "nrf_error.h"
#include "boards.h"

#include "softdevice_handler_appsh.h"
//#include "nrf_sdh.h"
//#include "nrf_sdh_ble.h"
//#include "nrf_sdh_soc.h"


#include "pstorage_platform.h"
#include "nrf_mbr.h"
//#include "nrf_log.h"

#include "nrf_wdt.h"
#include "nrf_delay.h"
#include "pstorage.h"


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

#define BOOTLOADER_BUTTON                   BUTTON_1                         // Button used to enter SW update mode.
#define FRESET_BUTTON                       BUTTON_2                         // Button used in addition to DFU button, to force OTA DFU

#define APP_TIMER_PRESCALER                 0                                /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE             4                                /**< Size of timer operation queues. */

#define SCHED_MAX_EVENT_DATA_SIZE           MAX(APP_TIMER_SCHED_EVT_SIZE, 0) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    20                               /**< Maximum number of events in the scheduler queue. */

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
STATIC_ASSERT( APPDATA_ADDR_START == 0x6D000);

void adafruit_factory_reset(void);
volatile bool _freset_erased_complete = false;

// Adafruit for Blink pattern
bool isBlinkFast = false;
bool isOTAConnected = false;

APP_TIMER_DEF( blinky_timer_id );

// true if ble, false if serial
bool _ota_update = false;

bool is_ota(void) { return _ota_update; }

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

static void button_pin_init(uint32_t pin)
{
  nrf_gpio_cfg_sense_input(pin, BUTTON_PULL, NRF_GPIO_PIN_SENSE_LOW);
}

bool button_pressed(uint32_t pin)
{
  return (nrf_gpio_pin_read(pin) == 0) ? true : false;
}

static void led_pin_init(uint32_t pin)
{
#ifdef BOARD_METRO52
  // LED BLUE is muxed with FRESET. We need to make sure it is
  // not wired to GND before configuring it as output.
  // Only check if it is not yet configured as OUTPUT
  if (pin == LED_BLUE && !bit_test(NRF_GPIO->PIN_CNF[pin], GPIO_PIN_CNF_DIR_Pos))
  {
    // skip and configure as input if grounded instead of output !!!
    if ( button_pressed(pin) ) return;
  }
#endif

  nrf_gpio_cfg_output(pin);
  led_off(pin);
}

void led_control(uint32_t pin, bool state)
{
#ifdef BOARD_METRO52
  // Skip if LED_BLUE is configured as input and wiring to GND
  // Otherwise configure it as output (it may just transition from hardware GND to open)
  if ( pin == LED_2 && !bit_test(NRF_GPIO->PIN_CNF[pin], GPIO_PIN_CNF_DIR_Pos) )
  {
    if ( button_pressed(pin) ) return;

    // configure as output
    nrf_gpio_cfg_output(pin);
  }
#endif

  nrf_gpio_pin_write(pin, state ? LED_STATE_ON : (1-LED_STATE_ON));
}


/*
 * Blinking function, there are a few patterns
 * - DFU Serial     : LED Status blink
 * - DFU OTA        : LED Status & Conn blink at the same time
 * - DFU Flashing   : LED Status blink 2x fast
 * - Factory Reset  : LED Status blink 2x fast
 * - Fatal Error    : LED Status & Conn blink one after another
 */
static void blinky_handler(void * p_context)
{
  static uint8_t state = 0;
  static uint32_t count = 0;

  count++;

  // if not uploading then blink slow (interval/2)
  if ( !isBlinkFast && count%2 ) return;

  state = 1-state;

  led_control(LED_RED, state);

  // Blink LED BLUE if OTA mode and not connected
  if (is_ota() && !isOTAConnected)
  {
    led_control(LED_BLUE, state);
  }

  // Feed all Watchdog just in case application enable it (WDT last through a soft reboot to bootloader)
  if ( nrf_wdt_started() )
  {
    for (uint8_t i=0; i<8; i++) nrf_wdt_reload_request_set(i);
  }
}

void blinky_fast_set(bool isFast)
{
  isBlinkFast = isFast;
}

void blinky_ota_connected(void)
{
  isOTAConnected = true;
}

void blinky_ota_disconneted(void)
{
  isOTAConnected = false;
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void sys_evt_dispatch(uint32_t event)
{
  pstorage_sys_event_handler(event);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 *
 * @param[in] init_softdevice  true if SoftDevice should be initialized. The SoftDevice must only
 *                             be initialized if a chip reset has occured. Soft reset from
 *                             application must not reinitialize the SoftDevice.
 */
static uint32_t ble_stack_init(bool init_softdevice)
{
  uint32_t         err_code;
  nrf_clock_lf_cfg_t clock_lf_cfg =
  {
      .source       = NRF_CLOCK_LF_SRC_RC,
      .rc_ctiv      = 16,
      .rc_temp_ctiv = 2,
      .accuracy     = NRF_CLOCK_LF_ACCURACY_20_PPM
  };

  if (init_softdevice)
  {
    sd_mbr_command_t com = { .command = SD_MBR_COMMAND_INIT_SD };
    err_code = sd_mbr_command(&com);
    APP_ERROR_CHECK(err_code);
  }

  err_code = sd_softdevice_vector_table_base_set(BOOTLOADER_REGION_START);
  APP_ERROR_CHECK(err_code);

  // equivalent to nrf_sdh_enable_request()
  SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);

  /*------------- Configure BLE params  -------------*/
  extern uint32_t  __data_start__[]; // defined in linker
  uint32_t ram_start = (uint32_t) __data_start__;

  ble_cfg_t blecfg;

  // Configure the maximum number of connections.
  varclr(&blecfg);
  blecfg.gap_cfg.role_count_cfg.periph_role_count  = 1;
  blecfg.gap_cfg.role_count_cfg.central_role_count = 0;
  blecfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
  err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &blecfg, ram_start);

  // NRF_DFU_BLE_REQUIRES_BONDS
  varclr(&blecfg);
  blecfg.gatts_cfg.service_changed.service_changed = 1;
  err_code = sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &blecfg, ram_start);
  VERIFY_SUCCESS(err_code);

  // ATT MTU
  varclr(&blecfg);
  blecfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_HIGH_BANDWIDTH;
  blecfg.conn_cfg.params.gatt_conn_cfg.att_mtu = BLEGATT_ATT_MTU_MAX;
  err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &blecfg, ram_start);
  VERIFY_SUCCESS ( err_code );

  // Event Length + HVN queue + WRITE CMD queue setting affecting bandwidth
  varclr(&blecfg);
  blecfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_HIGH_BANDWIDTH;
  blecfg.conn_cfg.params.gap_conn_cfg.conn_count   = 1;
  blecfg.conn_cfg.params.gap_conn_cfg.event_length = BLEGAP_EVENT_LENGTH;
  err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &blecfg, ram_start);
  VERIFY_SUCCESS ( err_code );

  // Enable BLE stack.
  err_code = sd_ble_enable(&ram_start);
  VERIFY_SUCCESS(err_code);

  err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
  VERIFY_SUCCESS(err_code);

  return err_code;
}


/**
 * @brief Function for event scheduler initialization.
 */
static void scheduler_init(void)
{
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

  /* Initialize a blinky timer to show that we're in bootloader */
  (void) app_timer_create(&blinky_timer_id, APP_TIMER_MODE_REPEATED, blinky_handler);
  app_timer_start(blinky_timer_id, APP_TIMER_TICKS(LED_BLINK_INTERVAL, APP_TIMER_PRESCALER), NULL);
}


/**
 * @brief Function for bootloader main entry.
 */
int main(void)
{
  uint32_t err_code;

  // SD is already Initialized in case of BOOTLOADER_DFU_OTA_MAGIC
  bool sd_inited = (NRF_POWER->GPREGRET == BOOTLOADER_DFU_OTA_MAGIC);

  // Start Bootloader in BLE OTA mode
  _ota_update = (NRF_POWER->GPREGRET == BOOTLOADER_DFU_OTA_MAGIC) ||
                (NRF_POWER->GPREGRET == BOOTLOADER_DFU_OTA_FULLRESET_MAGIC);

  // start bootloader either serial or ble
  bool dfu_start = _ota_update || (NRF_POWER->GPREGRET == BOOTLOADER_DFU_SERIAL_MAGIC);

  if (dfu_start)
  {
    // Clear GPREGRET if it is our values
    NRF_POWER->GPREGRET = 0;
  }

  // Save bootloader version to pre-defined register, retrieved by application
  BOOTLOADER_VERSION_REGISTER = (BOOTLOADER_VERSION);

  // This check ensures that the defined fields in the bootloader corresponds with actual
  // setting in the chip.
  APP_ERROR_CHECK_BOOL(*((uint32_t *)NRF_UICR_BOOT_START_ADDRESS) == BOOTLOADER_REGION_START);
  APP_ERROR_CHECK_BOOL(NRF_FICR->CODEPAGESIZE == CODE_PAGE_SIZE);

  /* Initialize GPIOs
   * For metro52 LED_BLUE is muxed with FRESET
   */
  button_pin_init(BOOTLOADER_BUTTON);
  button_pin_init(FRESET_BUTTON);
  nrf_delay_us(100); // wait for the pin state is stable

  led_pin_init(LED_RED);
  led_pin_init(LED_BLUE); // on metro52 will override FRESET

  // Initialize timer module, already configred to use with the scheduler.
  APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

  (void) bootloader_init();

  if (bootloader_dfu_sd_in_progress())
  {
    err_code = bootloader_dfu_sd_update_continue();
    APP_ERROR_CHECK(err_code);

    ble_stack_init(!sd_inited);
    scheduler_init();

    err_code = bootloader_dfu_sd_update_finalize();
    APP_ERROR_CHECK(err_code);
  }
  else
  {
    // If stack is present then continue initialization of bootloader.
    ble_stack_init(!sd_inited);
    scheduler_init();
  }

  /* For metro52 LED_BLUE is muxed with FRESET. We only init FRESET BUTTON
   * as needed and reconfigure as LED BLUE when done. */
#ifdef BOARD_METRO52
  button_pin_init(FRESET_BUTTON);
  nrf_delay_us(100); // wait for the pin state is stable
#endif

  // DFU button pressed
  dfu_start  = dfu_start || button_pressed(BOOTLOADER_BUTTON);

  // DFU + FRESET are pressed --> OTA
  _ota_update = _ota_update  || ( button_pressed(BOOTLOADER_BUTTON) && button_pressed(FRESET_BUTTON) ) ;

#ifdef BOARD_METRO52
  led_pin_init(LED_BLUE);
#endif

  if (dfu_start || (!bootloader_app_is_valid(DFU_BANK_0_REGION_START)))
  {
    // Initiate an update of the firmware.
    err_code = bootloader_dfu_start(_ota_update, 0);
    APP_ERROR_CHECK(err_code);
  }
  else
  {
    /* Adafruit Modification
     * Even DFU is not active, we still force an 1000 ms dfu serial mode when startup
     * to support auto programming from Arduino IDE
     */
    (void) bootloader_dfu_start(false, BOOTLOADER_STARTUP_DFU_INTERVAL);
  }

  // Adafruit Factory reset
#ifdef BOARD_METRO52
  button_pin_init(FRESET_BUTTON);
  nrf_delay_us(100); // wait for the pin state is stable
#endif

  bool is_freset = ( !button_pressed(BOOTLOADER_BUTTON) && button_pressed(FRESET_BUTTON) );

#ifdef BOARD_METRO52
  led_pin_init(LED_BLUE);
#endif

  if (is_freset)
  {
    adafruit_factory_reset();
  }

  app_timer_stop(blinky_timer_id);

  if (bootloader_app_is_valid(DFU_BANK_0_REGION_START) && !bootloader_dfu_sd_in_progress())
  {
    // Select a bank region to use as application region.
    // @note: Only applications running from DFU_BANK_0_REGION_START is supported.
    bootloader_app_start(DFU_BANK_0_REGION_START);
  }

  NVIC_SystemReset();
}

/**
 * Pstorage callback, fired after erased  Application Data
 */
static void appdata_pstorage_cb(pstorage_handle_t * p_handle, uint8_t op_code, uint32_t result,
                                uint8_t  * p_data, uint32_t  data_len)
{
  if ( op_code == PSTORAGE_CLEAR_OP_CODE)
  {
    _freset_erased_complete = true;
  }
}

void freset_erase_and_wait(pstorage_handle_t* hdl, uint32_t addr, uint32_t size)
{
  _freset_erased_complete = false;

  // set address and start erasing
  hdl->block_id = addr;
  pstorage_clear(hdl, size);

  // Time to erase a page is 100 ms max
  // It is better to force a timeout to prevent lock-up
  uint32_t timeout_tck = (size/CODE_PAGE_SIZE)*100;
  timeout_tck = APP_TIMER_TICKS(timeout_tck, APP_TIMER_PRESCALER);

  uint32_t start_tck;
  app_timer_cnt_get(&start_tck);

  while(!_freset_erased_complete)
  {
    sd_app_evt_wait();
    app_sched_execute();

    uint32_t now_tck;
    app_timer_cnt_get(&now_tck);
    if ( (now_tck - start_tck) > timeout_tck ) break;
  }
}

/**
 * Perform factory reset to erase Application + Data
 */
void adafruit_factory_reset(void)
{
  // Blink fast RED and turn on BLUE when erasing
  blinky_fast_set(true);
  led_on(LED_BLUE);

  static pstorage_handle_t freset_handle = { .block_id = APPDATA_ADDR_START } ;
  pstorage_module_param_t  storage_params = { .cb = appdata_pstorage_cb};

  pstorage_register(&storage_params, &freset_handle);

  // clear all App Data
  freset_erase_and_wait(&freset_handle, APPDATA_ADDR_START, DFU_APP_DATA_RESERVED);

  // Only need to erase the 1st page of Application code to make it invalid
  freset_erase_and_wait(&freset_handle, DFU_BANK_0_REGION_START, CODE_PAGE_SIZE);

  // back to normal
  blinky_fast_set(false);
  led_off(LED_BLUE);
}
