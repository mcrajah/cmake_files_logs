/**
 * Copyright (C) 2016-2019 KETOS INC Confidential
 * All Rights Reserved.
 * Author(s): Rajah Chandrasekhar (Ketos)
**/
/**
 * @file startup_ccs.c
 * @date Aug 5 2019
 * @brief Linker routine that defines the memory layout and allocates section (GCC)
 *
 * @see https://ketos7.atlassian.net/wiki/spaces/KH/pages/560463892/Shield+V3.x
 */
/******************************************************************************
 *
 * tiva4c1294_gcc.ld - Linker configuration file for tiva4c1294.
 *
 * Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 *
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's microcontroller products. The software is owned by
 * TI and/or its suppliers, and is protected under applicable copyright
 * laws. You may not combine this software with "viral" open-source
 * software in order to form a larger program.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
 *
 *****************************************************************************/

HEAP_SIZE = 1024;
SRAM_TOP = 0x20000000 + 0x00040000;

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00004000, LENGTH = 0x000FC000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00040000
    STACK (rwx): ORIGIN = 0x2003FFFF, LENGTH = 0x00000000
}

SECTIONS
{
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        KEEP(*(.init))
        KEEP(*(.fini))
        _etext = .;
    } > FLASH

    .gnu_build_id :
    {
        PROVIDE(g_note_build_id = .);
        *(.note.gnu.build-id)
        _etext = .;
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

   .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH
    __end_code = .;

   .data : AT(__end_code)
    {
        _data = .;
        _ldata = LOADADDR (.data);
        *(vtable)
        *(.data*)
        _edata = .;
    } > SRAM

   .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    .heap : AT(_ebss)
    {
        . = ALIGN(8);
        __end__ = .;
        PROVIDE(end = .);
        __HeapBase = .;
        . += HEAP_SIZE;
        __HeapLimit = .;
    } > SRAM

    .stack :
    {
        _stack_top = .;
    } > STACK

    __bss_start__ = _bss;
    __bss_end__ = _ebss;

}