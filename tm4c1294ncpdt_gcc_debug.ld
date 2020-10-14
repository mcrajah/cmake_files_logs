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

ENTRY(reset_isr)

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00100000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00040000
}

SRAM_TOP = 0x20000000 + 0x00040000;

SECTIONS
{
    /* The startup code into "FLASH" Rom type memory */
    .isr_vector :
    {
      . = ALIGN(4);
      KEEP(*(.isr_vector)) /* Startup code */
      . = ALIGN(4);
    } >FLASH
    .text : ALIGN(4)
    {
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        __bss_section_table_end = .;
        __section_table_end = . ;

        *(.after_vectors*)
        *(.text*)
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } > FLASH

    .gnu_build_id :
    {
        PROVIDE(g_note_build_id = .);
        *(.note.gnu.build-id)
    } > FLASH


    /*for exception handling/unwind - some Newlib functions (in common
    with C++ and STDC++) use this.*/
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH
    __exidx_start = .;

   .ARM : ALIGN(4)
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;
    _etext = .;

    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
    } > SRAM

   .data : ALIGN(4)
    {
        FILL(0xFF)
        _data = .;
        *(vtable)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM AT>FLASH

   .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        PROVIDE(end = .);
    } > SRAM

    /*NOTE: The below section should always be below bss*/
    .stack_overflow_magic_num : ALIGN(4)
    {
        KEEP(*(.stack_overflow_magic_num*))
    } > SRAM

    PROVIDE(_pvHeapStart = .);
    PROVIDE(_vStackTop = SRAM_TOP - 0);
}
/*
-----------------------------------------------------------------------
                       Syntax for Memory sections
Only one MEMORY section is allowed in a linker script, but each MEMORY
section can have many memory blocks

MEMORY
{
   name [(attr)] : ORIGIN = origin/org/o, LENGTH = length/len/l
   ...
   ...
}

name : name used to refer to the region, it has no meaning outside the
       script. (e.g. FLASH, SRAM etc)
attr : define the type of memory and is optional. Values are as below:
       'R' or 'r' Read-only section
       'W' or 'w' Read/Write setion
       'X' or 'x' Executable section
       'A' or 'a' Allocatable section
       'I' or 'i' Initialized section
       'L' or 'l' Same as 'I' or 'i'
       '!'        Invert the sense of any of the preceeding attributes
origin/org/o : start address of the memory region
length/len/l : size of bytes in the memory region

Example:
MEMORY
{
   rom (rx) : ORIGIN = 0, LENGTH = 256K
   ram (!rx) : org = 0x40000000, l = 4 M
}
In the above example, we specify that there are 2 memory regions available:
one starting at ‘0 ’ for 256 kilobytes , and the other starting at
‘0x40000000’ for 4 megabytes. The linker will place into the ‘rom ’ memory
region every section which is not explicitly mapped into a memory region,
and is either read-only or executable. The linker will place other sections
which are not explicitly mapped into a memory region into the ‘ram ’ memory
region.
-----------------------------------------------------------------------
                  Syntax for input and output sections
SECTIONS
{
   section [address] [(type)] : [AT(lma)]
   {
      output-section-command
      output-section-command
      ...
   } [>region]
}
section : .text (code), .data (initialized data), .bss (uninitialized data), etc
address : ., 0x10000 etc
type: NOLOAD, DSECT, COPY, INTO, OVERLAY (all these don't get loaded)
AT(lma) : If VMA is not equal to LMA, use AT symbol
region : FLASH, SRAM etc

Example:
SECTIONS
{
   .text 0x1000 :
   {
      *(.text) _etext = . ;
   }
   .mdata 0x2000 :   AT ( ADDR (.text) + SIZEOF (.text) )
   {
      _data = . ;
      *(.data);
      _edata = . ;
   }
   .bss 0x3000 :
   {
      _bstart = . ; *(.bss) *(COMMON) ; _bend = . ;
   }
}

The above linker script creates three output sections:
- one called ‘.text’, which starts at 0x1000,
- one called ‘.mdata’, which is loaded at the end of the
‘.text’ section even though its VMA is 0x2000, and
- one called ‘.bss ’ to hold uninitialized data at address 0x3000.

The symbol _data is deﬁned with the value 0x2000, which shows that
the location counter holds the VMA value, not the LMA value.

KEEP : Forces compiler to reseve memory for unused data (ex: finger print)
-----------------------------------------------------------------------
*/
