/* Linker script for Nordic Semiconductor nRF devices
 *
 * Version: Sourcery G++ 4.5-1
 * Support: https://support.codesourcery.com/GNUToolchain/
 *
 * Copyright (c) 2007, 2008, 2009, 2010 CodeSourcery, Inc.
 *
 * The authors hereby grant permission to use, copy, modify, distribute,
 * and license this software and its documentation for any purpose, provided
 * that existing copyright notices are retained in all copies and that this
 * notice is included verbatim in any distributions.  No written agreement,
 * license, or royalty fee is required for any of the authorized uses.
 * Modifications to this software may be copyrighted by their authors
 * and need not follow the licensing terms described here, provided that
 * the new terms are clearly indicated on the first page of each file where
 * they apply.
 */
OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __init_start
 *   __init_end
 *   __text_start
 *   __CTOR_LIST__
 *   __CTOR_END__
 *   __DTOR_LIST__
 *   __DTOR_END__
 *   __exidx_start
 *   __exidx_end
 *   __etext
 *   _etext
 *   etext
 *   __data_load_start
 *   __data_start
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __data_end
 *   __data_end__
 *   __sdata_start
 *   __sdata_end
 *   __fast_start
 *   __fast_end
 *   __edata
 *   _edata
 *   edata
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __bss_start__
 *   __bss_start
 *   __bss_end__
 *   __bss_end
 *   __end
 *   _end
 *   __end__
 *   end
 *   __HeapBase
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 */
ENTRY(Reset_Handler)


SECTIONS
{
    /* Set the location counter to start of FLASH */
    . = ALIGN(ORIGIN(FLASH), 4);    

    __vectors_start = .;
    .vectors : {
        /* Vector table */
        KEEP(*(.isr_vector))
        KEEP(*(.vectors))
    } > FLASH
    __vectors_size = SIZEOF(.vectors);
    __vectors_end = __vectors_start + __vectors_size;
    ASSERT(__vectors_start == __vectors_end || (__vectors_end - ORIGIN(FLASH)) <= LENGTH(FLASH) , "error: .vectors is too large to fit in FLASH memory segment")


    /* .ctors section (global constructor function pointers) */
    . = ALIGN(4);
    .ctors :
    {
        __ctors_start = .;
        __CTOR_LIST__ = .;
        KEEP (*crtbegin*.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend*.o ) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*(.ctors))
        __CTOR_END__ = .;
    } > FLASH
    __ctors_size  = SIZEOF(.ctors);
    __ctors_end   = __ctors_start + __ctors_size;
    ASSERT(__ctors_start == __ctors_end || (__ctors_end - ORIGIN(FLASH)) <= LENGTH(FLASH) , "error: .ctors is too large to fit in FLASH memory segment")


    /* .detors section (global destructor function pointers) */
    . = ALIGN(4);
    .dtors :
    {
        __dtors_start = .;
        __DTOR_LIST__ = .;
        KEEP (*crtbegin*.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend*.o ) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*(.dtors))
        __DTOR_END__ = .;
    } > FLASH
    __dtors_size  = SIZEOF(.dtors);
    __dtors_end   = __dtors_start + __dtors_size;
    ASSERT(__dtors_start == __dtors_end || (__dtors_end - ORIGIN(FLASH)) <= LENGTH(FLASH) , "error: .dtors is too large to fit in FLASH memory segment")


    /* .rodata section */
    . = ALIGN(4);
    .rodata : {
        . = ALIGN(4);
        __rodata_start = .;
        *(.rodata*)
        *(.srodata .srodata.*)
    } > FLASH
    __rodata_size       = SIZEOF(.rodata);
    __rodata_end        = __rodata_start + __rodata_size;
    ASSERT(__rodata_start == __rodata_end || (__rodata_end - ORIGIN(FLASH)) <= LENGTH(FLASH) , "error: .rodata is too large to fit in FLASH memory segment")


    . = ALIGN(4);
    __text_start = .;
    .text :
    {
        /* Reset handler for C startup file */
        KEEP(*(.startup))
        *(.startup*)

        /* text: executable instructions of a program. */
        *(.text*)

        /* executable instructions that contribute to the process initialization code */
        . = ALIGN(4);
        KEEP(*(.init))
        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;

        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;

        /* executable instructions that contribute to the process termination code */
        KEEP(*(.fini))

        . = ALIGN(4);
        __fini_array_start = .;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        __fini_array_end = .;

        /* DWARF-based unwinding - exception handling. C++ stuff (disable with no-asynchronous-unwind-tables)*/
        . = ALIGN(4);
        KEEP(*(.eh_frame*))

        /* section contains a pointer to the .eh_frame section which is */
        /* accessible to the runtime support code of a C++ application  */
        . = ALIGN(4);
        *(.eh_frame_hdr)

        /* hash tables */
        . = ALIGN(4);
        *(.hash)
        *(.gnu.hash)

        . = ALIGN(4);
        *(.gnu_extab* .gnu.linkonce.extab.*)

        . = ALIGN(4);
        __exidx_start = .;
        *(.gnu_exidx* .gnu.linkonce.exidx.*)
        . = ALIGN(4);
        __exidx_end = .;
    } > FLASH
    __text_size = SIZEOF(.text);
    __text_end = __text_start + __text_size;
    ASSERT(__text_start == __text_end || (__text_end - ORIGIN(FLASH)) <= LENGTH(FLASH) , "error: .text is too large to fit in FLASH memory segment")
    
    .copy.table :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;

        LONG (__data_load_start)
        LONG (__data_start)
        LONG ((__data_end - __data_start) / 4)

        LONG (__sdata_load_start)
        LONG (__sdata_start)
        LONG ((__sdata_end - __sdata_start) / 4)

        LONG (__tdata_load_start)
        LONG (__tdata_start)
        LONG ((__tdata_end - __tdata_start) / 4)

        LONG (__fast_load_start)
        LONG (__fast_start)
        LONG ((__fast_end - __fast_start) / 4)

        /* Add aditional regions here as needed. */
        
        __copy_table_end__ = .;
    } > FLASH

    .zero.table :
    {
        . = ALIGN(4);
        __zero_table_start__ = .;
        
        LONG (__bss_start)
        LONG ((__bss_end - __bss_start) / 4)
        
        LONG (__sbss_start)
        LONG ((__sbss_end - __sbss_start) / 4)
        
        LONG (__tbss_start)
        LONG ((__tbss_end - __tbss_start) / 4)

        /* Add aditional regions here as needed. */

        __zero_table_end__ = .;
    } > FLASH

    /* End of text */
    . = ALIGN(4);
    __etext = .; PROVIDE(etext = .); PROVIDE(_etext = .);


    /* Define start of all sections that needs to be loaded on boot */
    . = ALIGN(4);
    __all_data_load_start = .;

    /* .data section */
    __data_load_start = .;
    .data : AT(__data_load_start){
        . = ALIGN(4);
        __data_start = .;
        __data_start__ = .;
        *(vtable)
        *(.data* .gnu.linkonce.d.*)

        . = ALIGN(4);
        KEEP(*(.jcr*))
    } > RAM
    __data_size       = SIZEOF(.data);
    __data_end        = __data_start + __data_size;
    __data_end__      = __data_end;
    __data_load_end   = __data_load_start + __data_size;
    ASSERT(__data_load_start == __data_load_end || (__data_load_end - ORIGIN(FLASH)) <= LENGTH(FLASH) , "error: .data is too large to fit in FLASH memory segment")
    ASSERT(__data_start == __data_end || (__data_end - ORIGIN(RAM)) <= LENGTH(RAM) , "error: .data is too large to fit in RAM memory segment")


    /* .sdata section */
    . = ALIGN(4);
    __sdata_load_start = ALIGN(__data_load_end, 4);
    .sdata : AT(__sdata_load_start){
        . = ALIGN(4);
        __sdata_start = .;
        *(.sdata .sdata.* .gnu.linkonce.s.*)
        *(.sdata2 .sdata2.* .gnu.linkonce.s2.*)
    } > RAM
    __sdata_size       = SIZEOF(.sdata);
    __sdata_end        = __sdata_start + __sdata_size;
    __sdata_load_end   = __sdata_load_start + __sdata_size;
    ASSERT(__sdata_load_start == __sdata_load_end || (__sdata_load_end - ORIGIN(FLASH)) <= LENGTH(FLASH) , "error: .sdata is too large to fit in FLASH memory segment")
    ASSERT(__sdata_start == __sdata_end || (__sdata_end - ORIGIN(RAM)) <= LENGTH(RAM) , "error: .sdata is too large to fit in RAM memory segment")


    /* .tdata section */
    . = ALIGN(4);
    __tdata_load_start = ALIGN(__sdata_load_end, 4);
    .tdata : AT(__tdata_load_start){
        . = ALIGN(4);
        __tdata_start = .;
        *(.tdata .tdata.* .gnu.linkonce.td.*)
        *(.stdata .stdata.*)
    } > RAM
    __tdata_size       = SIZEOF(.tdata);
    __tdata_end        = __tdata_start + __tdata_size;
    __tdata_load_end   = __tdata_load_start + __tdata_size;
    ASSERT(__tdata_load_start == __tdata_load_end || (__tdata_load_end - ORIGIN(FLASH)) <= LENGTH(FLASH) , "error: .tdata is too large to fit in FLASH memory segment")
    ASSERT(__tdata_start == __tdata_end || (__tdata_end - ORIGIN(RAM)) <= LENGTH(RAM) , "error: .tdata is too large to fit in RAM memory segment")


    /* .fast section */
    . = ALIGN(4);
    __fast_load_start = ALIGN(__tdata_load_end, 4);
    .fast : AT(__fast_load_start){
        . = ALIGN(4);
        __fast_start = .;
        *(.fast .fast.*)
        *(.fastcode .fastcode.*)
        *(.fastdata .fastdata.*)
        *(.ramcode*)
    } > RAM
    __fast_size       = SIZEOF(.fast);
    __fast_end        = __fast_start + __fast_size;
    __fast_load_end   = __fast_load_start + __fast_size;
    ASSERT(__fast_load_start == __fast_load_end || (__fast_load_end - ORIGIN(FLASH)) <= LENGTH(FLASH) , "error: .fast is too large to fit in FLASH memory segment")
    ASSERT(__fast_start == __fast_end || (__fast_end - ORIGIN(RAM)) <= LENGTH(RAM) , "error: .fast is too large to fit in RAM memory segment")

    /* end of data in RAM */
    . = ALIGN(4);
    _edata = .; PROVIDE (edata = .); PROVIDE (__edata = .);


    /* No-init section. */
    . = ALIGN(4);
    __noinit_start = .; PROVIDE(__noinit_start__ = .);
    .noinit (NOLOAD):
    {
        *(.noinit .noinit.*)
        *(.no_init .no_init.*)
        *(.noninit .noninit.*)
        *(.non_init .non_init.*)
        . = ALIGN(4);
    } > RAM
    __noinit_size         = SIZEOF(.noinit);
    __noinit_end          = __noinit_start + __noinit_size;
    __noinit_end__        = __noinit_end;
    ASSERT(__noinit_start == __noinit_end || (__noinit_end - ORIGIN(RAM)) <= LENGTH(RAM) , "error: .noinit is too large to fit in RAM memory segment")
    
    
    /* .bss section */
    . = ALIGN(4);
    __bss_start = .; PROVIDE(__bss_start__ = .);
    .bss (NOLOAD):
    {
        *(.bss* .gnu.linkonce.b.*)
        *(.dynbss)
        *(COMMON)
        . = ALIGN(4);
    } > RAM
    __bss_size         = SIZEOF(.bss);
    __bss_end          = __bss_start + __bss_size;
    __bss_end__        = __bss_end;
    ASSERT(__bss_start == __bss_end || (__bss_end - ORIGIN(RAM)) <= LENGTH(RAM) , "error: .bss is too large to fit in RAM memory segment")


    /* .sbss section */
    . = ALIGN(4);
    __sbss_start = .;
    .sbss (NOLOAD):
    {
        *(.sbss .sbss.* .gnu.linkonce.sb.*)
        *(.sbss2 .sbss2.* .gnu.linkonce.sb2.*)
        *(.dynsbss)
        . = ALIGN(4);
    } > RAM
    __sbss_size         = SIZEOF(.sbss);
    __sbss_end          = __sbss_start + __sbss_size;
    ASSERT(__sbss_start == __sbss_end || (__sbss_end - ORIGIN(RAM)) <= LENGTH(RAM) , "error: .sbss is too large to fit in RAM memory segment")


    /* .tbss section */
    . = ALIGN(4);
    __tbss_start = .;
    .tbss (NOLOAD):
    {
        . = ALIGN(4);
        *(.tbss .tbss.* .gnu.linkonce.tb.*)
        *(.tcommon)
        *(.stbss .stbss.*)
        . = ALIGN(4);
    } > RAM
    __tbss_size         = SIZEOF(.tbss);
    __tbss_end          = __tbss_start + __tbss_size;
    ASSERT(__tbss_start == __tbss_end || (__tbss_end - ORIGIN(RAM)) <= LENGTH(RAM) , "error: .tbss is too large to fit in RAM memory segment")

    . = ALIGN(4);
    . = SEGMENT_START("ldata-segment", .);

    . = ALIGN(4);
    __heap_start = .;
    .heap (COPY):
    {
        __HeapBase = .;
        __heap_base = .;
        __end = .; 
        PROVIDE(end = .);
        PROVIDE(_end = .);
        PROVIDE(__end__ = .);
        KEEP(*(.heap*))
        __HeapLimit = .;
        __heap_limit = .;
    } > RAM
    __heap_end = .;

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        KEEP(*(.stack*))
    } > RAM

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __stack_end = ORIGIN(RAM) + LENGTH(RAM);

    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

    /* Check if text sections + data exceeds FLASH limit */
    DataInitFlashUsed = __bss_start - __data_start;
    CodeFlashUsed = _etext - ORIGIN(FLASH);
    TotalFlashUsed = CodeFlashUsed + DataInitFlashUsed;
    ASSERT(TotalFlashUsed <= LENGTH(FLASH), "region FLASH overflowed with .data and user data")
}

