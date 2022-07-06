#ifndef NRFX_CONFIG_H__
#define NRFX_CONFIG_H__

// Power
#define NRFX_POWER_ENABLED  1
#define NRFX_POWER_DEFAULT_CONFIG_IRQ_PRIORITY  7

#define NRFX_CLOCK_ENABLED  0
#define NRFX_NVMC_ENABLED   1

#define NRFX_PRS_ENABLED    0

#define NRFX_PWM_ENABLED    0
#define NRFX_PWM0_ENABLED   0
#define NRFX_PWM1_ENABLED   0
#define NRFX_PWM2_ENABLED   0
#define NRFX_PWM3_ENABLED   0

// UART
// mdm #ifdef NRF52832_XXAA
//#error mdm hey
#define NRFX_UART_ENABLED   1
#define NRFX_UART0_ENABLED  1

#define NRFX_UART_DEFAULT_CONFIG_IRQ_PRIORITY 7
#define NRFX_UART_DEFAULT_CONFIG_HWFC NRF_UART_HWFC_DISABLED
#define NRFX_UART_DEFAULT_CONFIG_PARITY NRF_UART_PARITY_EXCLUDED
#define NRFX_UART_DEFAULT_CONFIG_BAUDRATE NRF_UART_BAUDRATE_115200
// #endif

#endif
