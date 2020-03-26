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

#include "boards.h"

//==========================================================
// <e> HCI_SLIP_ENABLED - hci_slip - SLIP protocol implementation used by HCI
//==========================================================
#define HCI_SLIP_ENABLED                   1

#define HCI_UART_BAUDRATE                  UART_BAUDRATE_BAUDRATE_Baud115200
#define HCI_UART_FLOW_CONTROL              HWFC
#define HCI_UART_RX_PIN                    RX_PIN_NUMBER
#define HCI_UART_TX_PIN                    TX_PIN_NUMBER
#define HCI_UART_CTS_PIN                   CTS_PIN_NUMBER
#define HCI_UART_RTS_PIN                   RTS_PIN_NUMBER

#define HCI_TRANSPORT_ENABLED              1
#define HCI_MAX_PACKET_SIZE_IN_BITS        8000

//==========================================================
// <e> HCI_MEM_POOL_ENABLED - hci_mem_pool - memory pool implementation used by HCI
//==========================================================
#define HCI_MEM_POOL_ENABLED               1
#define HCI_TX_BUF_SIZE                    600 // not used
#define HCI_RX_BUF_SIZE                    600
#define HCI_RX_BUF_QUEUE_SIZE              8   // must be power of 2

//==========================================================
// <e> UART_ENABLED - nrf_drv_uart - UART/UARTE peripheral driver
//==========================================================
#define UART_ENABLED                       1
#define UART_DEFAULT_CONFIG_HWFC           0
#define UART_DEFAULT_CONFIG_PARITY         0
#define UART_DEFAULT_CONFIG_BAUDRATE       UART_BAUDRATE_BAUDRATE_Baud115200
#define UART_DEFAULT_CONFIG_IRQ_PRIORITY   7
#define UART_EASY_DMA_SUPPORT              1
#define UART_LEGACY_SUPPORT                1
#define UART_CONFIG_LOG_ENABLED            0

#define UART0_ENABLED                      1
#define UART0_CONFIG_USE_EASY_DMA          0


//==========================================================
// <e> APP_UART_ENABLED - app_uart - UART driver
//==========================================================
#define APP_UART_ENABLED                   1
#define APP_UART_DRIVER_INSTANCE           0

//==========================================================
// <e> APP_SCHEDULER_ENABLED - app_scheduler - Events scheduler
//==========================================================
#define APP_SCHEDULER_ENABLED              1
#define APP_SCHEDULER_WITH_PAUSE           0
#define APP_SCHEDULER_WITH_PROFILER        0

//==========================================================
// <e> APP_TIMER_ENABLED - app_timer - Application timer functionality
//==========================================================
#define APP_TIMER_ENABLED                  1

#define APP_TIMER_CONFIG_RTC_FREQUENCY     0
#define APP_TIMER_CONFIG_IRQ_PRIORITY      7
#define APP_TIMER_CONFIG_OP_QUEUE_SIZE     10
#define APP_TIMER_CONFIG_USE_SCHEDULER     1
#define APP_TIMER_WITH_PROFILER            0
#define APP_TIMER_CONFIG_SWI_NUMBER        0

// <q> APP_TIMER_KEEPS_RTC_ACTIVE - Enable RTC always on
// <i> If option is enabled RTC is kept running even if there is no active timers.
// <i> This option can be used when app_timer is used for timestamping.
#ifndef APP_TIMER_KEEPS_RTC_ACTIVE
#define APP_TIMER_KEEPS_RTC_ACTIVE         0
#endif

#define CRC16_ENABLED                      1
#define NRF_STRERROR_ENABLED               1


#endif //SDK_CONFIG_H

