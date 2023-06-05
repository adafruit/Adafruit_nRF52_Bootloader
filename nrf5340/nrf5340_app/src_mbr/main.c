
/**
 * After initialization done in thumb_crt0.s and sess_sturtup_nrf_common.s
 * program jumps to address BOOTLOADER_REGION_START to start executing bootloader
 */
#include <nrf5340_application.h>
#include <core_cm33.h>

#define BOOTLOADER_REGION_START             0x000EE000

static inline void jump_to_bootloader(unsigned int addr)
{
    __asm volatile(
        "ldr   r0, [%0]\t\n"            // Get App initial MSP for bootloader.
        "msr   msp, r0\t\n"             // Set the main stack pointer to the applications MSP.
        "ldr   r0, [%0, #0x04]\t\n"     // Load Reset handler into R0.
        "movs  r4, #0xFF\t\n"           // Move ones to R4.
        "sxtb  r4, r4\t\n"              // Sign extend R4 to obtain 0xFFFFFFFF instead of 0xFF.
        "mov   lr, r4\t\n"              // Clear the link register and set to ones to ensure no return.
        "bx    r0\t\n"                  // Branch to reset handler of bootloader.

        ".align\t\n"
        :: "r" (addr)                   // Argument list for the gcc assembly. start_addr is %0.
        :  "r0", "r4"                   // List of register maintained manually.
    );
}

int main(void)
{
  unsigned int bootloader_address = BOOTLOADER_REGION_START;

  /* set vector table to bootloader flash address */
  SCB->VTOR = bootloader_address;

  /* jump to bootloader */
  jump_to_bootloader(bootloader_address);
}

