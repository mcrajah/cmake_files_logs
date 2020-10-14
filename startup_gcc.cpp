//*****************************************************************************
//
// startup_ccs.c - Startup code for use with TI's Code Composer Studio.
//
// Copyright (c) 2013-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.3.156 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "common.h"
#include "main.h"
//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
static void reset_isr(void);
static void nm_isr(void);
static void fault_isr(void);
static void default_isr_handler(void);
//*****************************************************************************
//
// External declaration for the reset handler that is to be called when the
// processor is started
//
//*****************************************************************************
extern int main(void);
extern uint32_t _etext;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss;
extern uint32_t _ebss;


//*****************************************************************************
//
// Linker variable that marks the top of the stack.
//
//*****************************************************************************
extern uint32_t _vStackTop;
//*****************************************************************************
//
// External declarations for the interrupt handlers used by the application.
//
//*****************************************************************************
extern void Timer0IntHandler(void);
extern void Timer1IntHandler(void);
extern void Timer2IntHandler(void);
extern void Timer3IntHandler(void);
extern void Timer4IntHandler(void);
extern void Timer5IntHandler(void);
extern void BB_Init_Timer(void);
extern void fwu_gpio_int_handler(void);
extern void UART6IntHandler(void);
extern void portM_Rising_Edge_IRQHandler(void);
#if defined(SAMPLE_VCC_33_ON_ADC1)
    extern void ADC1_IRQ_Handler(void);
#endif
//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000 or at the start of
// the program if located at a start address other than 0.
//
//*****************************************************************************
// #pragma DATA_SECTION(".isr_vector")
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))((unsigned long)&_vStackTop),  // The initial stack pointer
    reset_isr,                                    // The reset handler
    nm_isr,                                       // The NMI handler
    fault_isr,                                    // The hard fault handler
    default_isr_handler,                          // The MPU fault handler
    default_isr_handler,                          // The bus fault handler
    default_isr_handler,                          // The usage fault handler
    0,                                            // Reserved
    0,                                            // Reserved
    0,                                            // Reserved
    0,                                            // Reserved
    default_isr_handler,                          // SVCall handler
    default_isr_handler,                          // Debug monitor handler
    0,                                            // Reserved
    default_isr_handler,                          // The PendSV handler
    default_isr_handler,                          // The SysTick handler
    default_isr_handler,                          // GPIO Port A
    fwu_gpio_int_handler,                         // GPIO Port B
    default_isr_handler,                          // GPIO Port C
    default_isr_handler,                          // GPIO Port D
    default_isr_handler,                          // GPIO Port E
    default_isr_handler,                          // UART0 Rx and Tx
    default_isr_handler,                          // UART1 Rx and Tx
    default_isr_handler,                          // SSI0 Rx and Tx
    default_isr_handler,                          // I2C0 Master and Slave
    default_isr_handler,                          // PWM Fault
    default_isr_handler,                          // PWM Generator 0
    default_isr_handler,                          // PWM Generator 1
    default_isr_handler,                          // PWM Generator 2
    default_isr_handler,                          // Quadrature Encoder 0
    default_isr_handler,                          // ADC Sequence 0
    default_isr_handler,                          // ADC Sequence 1
    default_isr_handler,                          // ADC Sequence 2
    default_isr_handler,                          // ADC Sequence 3
    default_isr_handler,                          // Watchdog timer
    Timer0IntHandler,                             // Timer 0 subtimer A
    default_isr_handler,                          // Timer 0 subtimer B
    Timer1IntHandler,                             // Timer 1 subtimer A
    Timer1IntHandler,                             // Timer 1 subtimer B
    Timer2IntHandler,                             // Timer 2 subtimer A
    default_isr_handler,                          // Timer 2 subtimer B
    default_isr_handler,                          // Analog Comparator 0
    default_isr_handler,                          // Analog Comparator 1
    default_isr_handler,                          // Analog Comparator 2
    default_isr_handler,                          // System Control (PLL, OSC, BO)
    default_isr_handler,                          // FLASH Control
    default_isr_handler,                          // GPIO Port F
    default_isr_handler,                          // GPIO Port G
    default_isr_handler,                          // GPIO Port H
    default_isr_handler,                          // UART2 Rx and Tx
    default_isr_handler,                          // SSI1 Rx and Tx
    Timer3IntHandler,                             // Timer 3 subtimer A
    default_isr_handler,                          // Timer 3 subtimer B
    default_isr_handler,                          // I2C1 Master and Slave
    default_isr_handler,                          // CAN0
    default_isr_handler,                          // CAN1
    default_isr_handler,                          // Ethernet
    default_isr_handler,                          // Hibernate
    default_isr_handler,                          // USB0
    default_isr_handler,                          // PWM Generator 3
    default_isr_handler,                          // uDMA Software Transfer
    default_isr_handler,                          // uDMA Error
    default_isr_handler,                          // ADC1 Sequence 0
    default_isr_handler,                          // ADC1 Sequence 1
    default_isr_handler,                          // ADC1 Sequence 2
#if defined(SAMPLE_VCC_33_ON_ADC1)
    ADC1_IRQ_Handler,                             // ADC1 Sequence 3
#else
    default_isr_handler,                          // ADC1 Sequence 3
#endif
    default_isr_handler,                          // External Bus Interface 0
    default_isr_handler,                          // GPIO Port J
    default_isr_handler,                          // GPIO Port K
    default_isr_handler,                          // GPIO Port L
    default_isr_handler,                          // SSI2 Rx and Tx
    default_isr_handler,                          // SSI3 Rx and Tx
    default_isr_handler,                          // UART3 Rx and Tx
    default_isr_handler,                          // UART4 Rx and Tx
    default_isr_handler,                          // UART5 Rx and Tx
    UART6IntHandler,                              // UART6 Rx and Tx
    default_isr_handler,                          // UART7 Rx and Tx
    default_isr_handler,                          // I2C2 Master and Slave
    default_isr_handler,                          // I2C3 Master and Slave
    Timer4IntHandler,                             // Timer 4 subtimer A
    default_isr_handler,                          // Timer 4 subtimer B
    Timer5IntHandler,                             // Timer 5 subtimer A
    default_isr_handler,                          // Timer 5 subtimer B
    default_isr_handler,                          // FPU
    0,                                            // Reserved
    0,                                            // Reserved
    default_isr_handler,                          // I2C4 Master and Slave
    default_isr_handler,                          // I2C5 Master and Slave
    portM_Rising_Edge_IRQHandler,                 // GPIO Port M
    default_isr_handler,                          // GPIO Port N
    0,                                            // Reserved
    default_isr_handler,                          // Tamper
    default_isr_handler,                          // GPIO Port P (Summary or P0)
    default_isr_handler,                          // GPIO Port P1
    default_isr_handler,                          // GPIO Port P2
    default_isr_handler,                          // GPIO Port P3
    default_isr_handler,                          // GPIO Port P4
    default_isr_handler,                          // GPIO Port P5
    default_isr_handler,                          // GPIO Port P6
    default_isr_handler,                          // GPIO Port P7
    default_isr_handler,                          // GPIO Port Q (Summary or Q0)
    default_isr_handler,                          // GPIO Port Q1
    default_isr_handler,                          // GPIO Port Q2
    default_isr_handler,                          // GPIO Port Q3
    default_isr_handler,                          // GPIO Port Q4
    default_isr_handler,                          // GPIO Port Q5
    default_isr_handler,                          // GPIO Port Q6
    default_isr_handler,                          // GPIO Port Q7
    default_isr_handler,                          // GPIO Port R
    default_isr_handler,                          // GPIO Port S
    default_isr_handler,                          // SHA/MD5 0
    default_isr_handler,                          // AES 0
    default_isr_handler,                          // DES3DES 0
    default_isr_handler,                          // LCD Controller 0
    default_isr_handler,                          // Timer 6 subtimer A
    default_isr_handler,                          // Timer 6 subtimer B
    default_isr_handler,                          // Timer 7 subtimer A
    default_isr_handler,                          // Timer 7 subtimer B
    default_isr_handler,                          // I2C6 Master and Slave
    default_isr_handler,                          // I2C7 Master and Slave
    default_isr_handler,                          // HIM Scan Matrix Keyboard 0
    default_isr_handler,                          // One Wire 0
    default_isr_handler,                          // HIM PS/2 0
    default_isr_handler,                          // HIM LED Sequencer 0
    default_isr_handler,                          // HIM Consumer IR 0
    default_isr_handler,                          // I2C8 Master and Slave
    default_isr_handler,                          // I2C9 Master and Slave
    default_isr_handler                           // GPIO Port T
};

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
__attribute__ (( naked ))
static void reset_isr(void)
{
    uint32_t *pui32Src, *pui32Dest;

    //
    // Copy the data segment initializers from flash to SRAM.
    //
    pui32Src = &_etext;
    for(pui32Dest = &_data; pui32Dest < &_edata; )
    {
        *pui32Dest++ = *pui32Src++;
    }

    //
    // Zero fill the bss segment.
    //
    __asm("    ldr     r0, =_bss\n"
          "    ldr     r1, =_ebss\n"
          "    mov     r2, #0\n"
          "    .thumb_func\n"
          "zero_loop:\n"
          "        cmp     r0, r1\n"
          "        it      lt\n"
          "        strlt   r2, [r0], #4\n"
          "        blt     zero_loop");

    //
    // Enable the floating-point unit.  This must be done here to handle the
    // case where main() uses floating-point and the function prologue saves
    // floating-point registers (which will fault if floating-point is not
    // enabled).  Any configuration of the floating-point unit using DriverLib
    // APIs must be done here prior to the floating-point unit being enabled.
    //
    // Note that this does not use DriverLib since it might not be included in
    // this project.
    //
    HWREG(NVIC_CPAC) = ((HWREG(NVIC_CPAC) & (uint32_t)
                         ~(NVIC_CPAC_CP10_M | NVIC_CPAC_CP11_M)) |
                        NVIC_CPAC_CP10_FULL | NVIC_CPAC_CP11_FULL);

    //
    // Call the application's entry point.
    //
    main();
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
nm_isr(void)
{
    //
    // Enter an infinite loop.
    sendPacket((char *)"#9090, NM Interrupt occurred. \n");
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
fault_isr(void)
{
    //
    // Enter an infinite loop.
    UARTprintf(" Spurious Interrupt Fault.. \n");
    sendPacket((char *)"#9091, Hard Fault Interrupt occurred. \n");
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
default_isr_handler(void)
{
    //
    // Go into an infinite loop.
    UARTprintf(" Interrupt issue.. \n");
    sendPacket((char *)"#9092, Spurious Interrupt occurred. \n");
    //
    while(1)
    {
    }
}