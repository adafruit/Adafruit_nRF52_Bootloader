/**
 * Copyright (c) 2017 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */



#ifndef SDK_CONFIG_H
#define SDK_CONFIG_H

// <e> HCI_SLIP_ENABLED - hci_slip - SLIP protocol implementation used by HCI
//==========================================================
#ifndef HCI_SLIP_ENABLED
#define HCI_SLIP_ENABLED 1
#endif
// <o> HCI_UART_BAUDRATE  - Default Baudrate

// <323584=> 1200 baud
// <643072=> 2400 baud
// <1290240=> 4800 baud
// <2576384=> 9600 baud
// <3862528=> 14400 baud
// <5152768=> 19200 baud
// <7716864=> 28800 baud
// <10289152=> 38400 baud
// <15400960=> 57600 baud
// <20615168=> 76800 baud
// <30801920=> 115200 baud
// <61865984=> 230400 baud
// <67108864=> 250000 baud
// <121634816=> 460800 baud
// <251658240=> 921600 baud
// <268435456=> 1000000 baud

#ifndef HCI_UART_BAUDRATE
#define HCI_UART_BAUDRATE 30801920
#endif

// <o> HCI_UART_FLOW_CONTROL  - Hardware Flow Control

// <0=> Disabled
// <1=> Enabled

#ifndef HCI_UART_FLOW_CONTROL
#define HCI_UART_FLOW_CONTROL 0
#endif

// <o> HCI_UART_RX_PIN - UART RX pin
#ifndef HCI_UART_RX_PIN
#define HCI_UART_RX_PIN 8
#endif

// <o> HCI_UART_TX_PIN - UART TX pin
#ifndef HCI_UART_TX_PIN
#define HCI_UART_TX_PIN 6
#endif

// <o> HCI_UART_RTS_PIN - UART RTS pin
#ifndef HCI_UART_RTS_PIN
#define HCI_UART_RTS_PIN 5
#endif

// <o> HCI_UART_CTS_PIN - UART CTS pin
#ifndef HCI_UART_CTS_PIN
#define HCI_UART_CTS_PIN 7
#endif


#ifndef HCI_TRANSPORT_ENABLED
#define HCI_TRANSPORT_ENABLED 1
#endif
// <o> HCI_MAX_PACKET_SIZE_IN_BITS - Maximum size of a single application packet in bits.
#ifndef HCI_MAX_PACKET_SIZE_IN_BITS
#define HCI_MAX_PACKET_SIZE_IN_BITS 8000
#endif
 

#ifndef NRF_DFU_BLE_REQUIRES_BONDS
#define NRF_DFU_BLE_REQUIRES_BONDS 0
#endif

//==========================================================

// <h> nRF_Drivers 

//==========================================================
// <e> CLOCK_ENABLED - nrf_drv_clock - CLOCK peripheral driver
//==========================================================
#ifndef CLOCK_ENABLED
#define CLOCK_ENABLED 0
#endif
// <o> CLOCK_CONFIG_XTAL_FREQ  - HF XTAL Frequency
 
// <0=> Default (64 MHz) 

#ifndef CLOCK_CONFIG_XTAL_FREQ
#define CLOCK_CONFIG_XTAL_FREQ 0
#endif

// <o> CLOCK_CONFIG_LF_SRC  - LF Clock Source
 
// <0=> RC 
// <1=> XTAL 
// <2=> Synth 

#ifndef CLOCK_CONFIG_LF_SRC
#define CLOCK_CONFIG_LF_SRC 1
#endif

// <o> CLOCK_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef CLOCK_CONFIG_IRQ_PRIORITY
#define CLOCK_CONFIG_IRQ_PRIORITY 7
#endif

// </e>

// </e>

// <q> EGU_ENABLED  - nrf_drv_swi - SWI(EGU) peripheral driver
 

#ifndef EGU_ENABLED
#define EGU_ENABLED 0
#endif

// <e> GPIOTE_ENABLED - nrf_drv_gpiote - GPIOTE peripheral driver
//==========================================================
#ifndef GPIOTE_ENABLED
#define GPIOTE_ENABLED 0
#endif
// <o> GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS - Number of lower power input pins 
#ifndef GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS
#define GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS 4
#endif

// <o> GPIOTE_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef GPIOTE_CONFIG_IRQ_PRIORITY
#define GPIOTE_CONFIG_IRQ_PRIORITY 7
#endif

// </e>




// <q> PERIPHERAL_RESOURCE_SHARING_ENABLED  - nrf_drv_common - Peripheral drivers common module
 

#ifndef PERIPHERAL_RESOURCE_SHARING_ENABLED
#define PERIPHERAL_RESOURCE_SHARING_ENABLED 0
#endif

// <e> POWER_ENABLED - nrf_drv_power - POWER peripheral driver
//==========================================================
#ifndef POWER_ENABLED
#define POWER_ENABLED 1
#endif
// <o> POWER_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef POWER_CONFIG_IRQ_PRIORITY
#define POWER_CONFIG_IRQ_PRIORITY 7
#endif

// <q> POWER_CONFIG_DEFAULT_DCDCEN  - The default configuration of main DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#ifndef POWER_CONFIG_DEFAULT_DCDCEN
#define POWER_CONFIG_DEFAULT_DCDCEN 0
#endif

// <q> POWER_CONFIG_DEFAULT_DCDCENHV  - The default configuration of High Voltage DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#ifndef POWER_CONFIG_DEFAULT_DCDCENHV
#define POWER_CONFIG_DEFAULT_DCDCENHV 0
#endif

// </e>


// <e> RTC_ENABLED - nrf_drv_rtc - RTC peripheral driver
//==========================================================
#ifndef RTC_ENABLED
#define RTC_ENABLED 0
#endif
// <o> RTC_DEFAULT_CONFIG_FREQUENCY - Frequency  <16-32768> 


#ifndef RTC_DEFAULT_CONFIG_FREQUENCY
#define RTC_DEFAULT_CONFIG_FREQUENCY 32768
#endif

// <q> RTC_DEFAULT_CONFIG_RELIABLE  - Ensures safe compare event triggering
 

#ifndef RTC_DEFAULT_CONFIG_RELIABLE
#define RTC_DEFAULT_CONFIG_RELIABLE 0
#endif

// <o> RTC_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef RTC_DEFAULT_CONFIG_IRQ_PRIORITY
#define RTC_DEFAULT_CONFIG_IRQ_PRIORITY 7
#endif

// <q> RTC0_ENABLED  - Enable RTC0 instance
 

#ifndef RTC0_ENABLED
#define RTC0_ENABLED 0
#endif

// <q> RTC1_ENABLED  - Enable RTC1 instance
 

#ifndef RTC1_ENABLED
#define RTC1_ENABLED 0
#endif

// <q> RTC2_ENABLED  - Enable RTC2 instance
 

#ifndef RTC2_ENABLED
#define RTC2_ENABLED 0
#endif

// <o> NRF_MAXIMUM_LATENCY_US - Maximum possible time[us] in highest priority interrupt 
#ifndef NRF_MAXIMUM_LATENCY_US
#define NRF_MAXIMUM_LATENCY_US 2000
#endif

// </e>


// <e> UART_ENABLED - nrf_drv_uart - UART/UARTE peripheral driver
//==========================================================
#ifndef UART_ENABLED
#define UART_ENABLED 1
#endif
// <o> UART_DEFAULT_CONFIG_HWFC  - Hardware Flow Control
 
// <0=> Disabled 
// <1=> Enabled 

#ifndef UART_DEFAULT_CONFIG_HWFC
#define UART_DEFAULT_CONFIG_HWFC 0
#endif

// <o> UART_DEFAULT_CONFIG_PARITY  - Parity
 
// <0=> Excluded 
// <14=> Included 

#ifndef UART_DEFAULT_CONFIG_PARITY
#define UART_DEFAULT_CONFIG_PARITY 0
#endif

// <o> UART_DEFAULT_CONFIG_BAUDRATE  - Default Baudrate
 
// <323584=> 1200 baud 
// <643072=> 2400 baud 
// <1290240=> 4800 baud 
// <2576384=> 9600 baud 
// <3862528=> 14400 baud 
// <5152768=> 19200 baud 
// <7716864=> 28800 baud 
// <10289152=> 38400 baud 
// <15400960=> 57600 baud 
// <20615168=> 76800 baud 
// <30801920=> 115200 baud 
// <61865984=> 230400 baud 
// <67108864=> 250000 baud 
// <121634816=> 460800 baud 
// <251658240=> 921600 baud 
// <268435456=> 1000000 baud 

#ifndef UART_DEFAULT_CONFIG_BAUDRATE
#define UART_DEFAULT_CONFIG_BAUDRATE 30801920
#endif

// <o> UART_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef UART_DEFAULT_CONFIG_IRQ_PRIORITY
#define UART_DEFAULT_CONFIG_IRQ_PRIORITY 7
#endif

// <q> UART_EASY_DMA_SUPPORT  - Driver supporting EasyDMA
 

#ifndef UART_EASY_DMA_SUPPORT
#define UART_EASY_DMA_SUPPORT 1
#endif

// <q> UART_LEGACY_SUPPORT  - Driver supporting Legacy mode
 

#ifndef UART_LEGACY_SUPPORT
#define UART_LEGACY_SUPPORT 1
#endif

// <e> UART0_ENABLED - Enable UART0 instance
//==========================================================
#ifndef UART0_ENABLED
#define UART0_ENABLED 1
#endif
// <q> UART0_CONFIG_USE_EASY_DMA  - Default setting for using EasyDMA
 

#ifndef UART0_CONFIG_USE_EASY_DMA
#define UART0_CONFIG_USE_EASY_DMA 0
#endif

// </e>

// <e> UART1_ENABLED - Enable UART1 instance
//==========================================================
#ifndef UART1_ENABLED
#define UART1_ENABLED 0
#endif
// <q> UART1_CONFIG_USE_EASY_DMA  - Default setting for using EasyDMA
 

#ifndef UART1_CONFIG_USE_EASY_DMA
#define UART1_CONFIG_USE_EASY_DMA 1
#endif

// </e>

// </e>


// <e> WDT_ENABLED - nrf_drv_wdt - WDT peripheral driver
//==========================================================
#ifndef WDT_ENABLED
#define WDT_ENABLED 0
#endif
// <o> WDT_CONFIG_BEHAVIOUR  - WDT behavior in CPU SLEEP or HALT mode
 
// <1=> Run in SLEEP, Pause in HALT 
// <8=> Pause in SLEEP, Run in HALT 
// <9=> Run in SLEEP and HALT 
// <0=> Pause in SLEEP and HALT 

#ifndef WDT_CONFIG_BEHAVIOUR
#define WDT_CONFIG_BEHAVIOUR 1
#endif

// <o> WDT_CONFIG_RELOAD_VALUE - Reload value  <15-4294967295> 


#ifndef WDT_CONFIG_RELOAD_VALUE
#define WDT_CONFIG_RELOAD_VALUE 2000
#endif

// <o> WDT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef WDT_CONFIG_IRQ_PRIORITY
#define WDT_CONFIG_IRQ_PRIORITY 7
#endif

// </e>

// </h> 
//==========================================================

// <h> nRF_Libraries 

//==========================================================
// <e> APP_SCHEDULER_ENABLED - app_scheduler - Events scheduler
//==========================================================
#ifndef APP_SCHEDULER_ENABLED
#define APP_SCHEDULER_ENABLED 1
#endif
// <q> APP_SCHEDULER_WITH_PAUSE  - Enabling pause feature
 

#ifndef APP_SCHEDULER_WITH_PAUSE
#define APP_SCHEDULER_WITH_PAUSE 0
#endif

// <q> APP_SCHEDULER_WITH_PROFILER  - Enabling scheduler profiling
 

#ifndef APP_SCHEDULER_WITH_PROFILER
#define APP_SCHEDULER_WITH_PROFILER 0
#endif

// </e>

// <e> APP_TIMER_ENABLED - app_timer - Application timer functionality
//==========================================================
#ifndef APP_TIMER_ENABLED
#define APP_TIMER_ENABLED 1
#endif
// <o> APP_TIMER_CONFIG_RTC_FREQUENCY  - Configure RTC prescaler.
 
// <0=> 32768 Hz 
// <1=> 16384 Hz 
// <3=> 8192 Hz 
// <7=> 4096 Hz 
// <15=> 2048 Hz 
// <31=> 1024 Hz 

#ifndef APP_TIMER_CONFIG_RTC_FREQUENCY
#define APP_TIMER_CONFIG_RTC_FREQUENCY 0
#endif

// <o> APP_TIMER_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef APP_TIMER_CONFIG_IRQ_PRIORITY
#define APP_TIMER_CONFIG_IRQ_PRIORITY 7
#endif

// <o> APP_TIMER_CONFIG_OP_QUEUE_SIZE - Capacity of timer requests queue. 
// <i> Size of the queue depends on how many timers are used
// <i> in the system, how often timers are started and overall
// <i> system latency. If queue size is too small app_timer calls
// <i> will fail.

#ifndef APP_TIMER_CONFIG_OP_QUEUE_SIZE
#define APP_TIMER_CONFIG_OP_QUEUE_SIZE 10
#endif

// <q> APP_TIMER_CONFIG_USE_SCHEDULER  - Enable scheduling app_timer events to app_scheduler
 

#ifndef APP_TIMER_CONFIG_USE_SCHEDULER
#define APP_TIMER_CONFIG_USE_SCHEDULER 1
#endif

// <q> APP_TIMER_WITH_PROFILER  - Enable app_timer profiling
 

#ifndef APP_TIMER_WITH_PROFILER
#define APP_TIMER_WITH_PROFILER 0
#endif

// <q> APP_TIMER_KEEPS_RTC_ACTIVE  - Enable RTC always on
 

// <i> If option is enabled RTC is kept running even if there is no active timers.
// <i> This option can be used when app_timer is used for timestamping.

#ifndef APP_TIMER_KEEPS_RTC_ACTIVE
#define APP_TIMER_KEEPS_RTC_ACTIVE 0
#endif

// <o> APP_TIMER_CONFIG_SWI_NUMBER  - Configure SWI instance used.
 
// <0=> 0 
// <1=> 1 

#ifndef APP_TIMER_CONFIG_SWI_NUMBER
#define APP_TIMER_CONFIG_SWI_NUMBER 0
#endif

// </e>

// <e> APP_UART_ENABLED - app_uart - UART driver
//==========================================================
#ifndef APP_UART_ENABLED
#define APP_UART_ENABLED 1
#endif
// <o> APP_UART_DRIVER_INSTANCE  - UART instance used
 
// <0=> 0 

#ifndef APP_UART_DRIVER_INSTANCE
#define APP_UART_DRIVER_INSTANCE 0
#endif

// </e>


// <q> BUTTON_ENABLED  - app_button - buttons handling module
 

#ifndef BUTTON_ENABLED
#define BUTTON_ENABLED 0
#endif

// <q> CRC16_ENABLED  - crc16 - CRC16 calculation routines
 

#ifndef CRC16_ENABLED
#define CRC16_ENABLED 1
#endif

// <e> HCI_MEM_POOL_ENABLED - hci_mem_pool - memory pool implementation used by HCI
//==========================================================
#ifndef HCI_MEM_POOL_ENABLED
#define HCI_MEM_POOL_ENABLED 1
#endif
// <o> HCI_TX_BUF_SIZE - TX buffer size in bytes. 
#ifndef HCI_TX_BUF_SIZE
#define HCI_TX_BUF_SIZE 600
#endif

// <o> HCI_RX_BUF_SIZE - RX buffer size in bytes. 
#ifndef HCI_RX_BUF_SIZE
#define HCI_RX_BUF_SIZE 600
#endif

// <o> HCI_RX_BUF_QUEUE_SIZE - RX buffer queue size. 
#ifndef HCI_RX_BUF_QUEUE_SIZE
#define HCI_RX_BUF_QUEUE_SIZE 4
#endif

// </e>

// <e> NRF_FSTORAGE_ENABLED - nrf_fstorage - Flash abstraction library
//==========================================================
#ifndef NRF_FSTORAGE_ENABLED
#define NRF_FSTORAGE_ENABLED 1
#endif
// <h> nrf_fstorage_sd - Implementation using the SoftDevice.

// <i> Configuration options for the fstorage implementation using the SoftDevice.
//==========================================================
// <o> NRF_FSTORAGE_SD_QUEUE_SIZE - Size of the internal queue of operations. 
// <i> Increase this value if API calls frequently return the error @ref NRF_ERROR_NO_MEM.

#ifndef NRF_FSTORAGE_SD_QUEUE_SIZE
#define NRF_FSTORAGE_SD_QUEUE_SIZE 8
#endif

// <o> NRF_FSTORAGE_SD_MAX_RETRIES - Maximum number of attempts at executing an operation when the SoftDevice is busy. 
// <i> Increase this value if events frequently return the @ref NRF_ERROR_TIMEOUT error.
// <i> The SoftDevice might fail to schedule flash access due to high BLE activity.

#ifndef NRF_FSTORAGE_SD_MAX_RETRIES
#define NRF_FSTORAGE_SD_MAX_RETRIES 8
#endif

// <o> NRF_FSTORAGE_SD_MAX_WRITE_SIZE - Maximum number of bytes to be written to flash in a single operation. 
// <i> This value must be a multiple of four.
// <i> Lowering this value can increase the chances of the SoftDevice being able to execute flash operations in between radio activity.
// <i> This value is bound by the maximum number of bytes that can be written to flash in a single call to @ref sd_flash_write.
// <i> That is 1024 bytes for nRF51 ICs and 4096 bytes for nRF52 ICs.

#ifndef NRF_FSTORAGE_SD_MAX_WRITE_SIZE
#define NRF_FSTORAGE_SD_MAX_WRITE_SIZE 20
#endif

// </h> 
//==========================================================


// <q> NRF_STRERROR_ENABLED  - nrf_strerror - Library for converting error code to string.
#ifndef NRF_STRERROR_ENABLED
#define NRF_STRERROR_ENABLED 1
#endif

// <<< end of configuration section >>>
#endif //SDK_CONFIG_H

