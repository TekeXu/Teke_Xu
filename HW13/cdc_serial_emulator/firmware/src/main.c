/*******************************************************************************
  MPLAB Harmony Project Main Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    This file contains the "main" function for an MPLAB Harmony project.

  Description:
    This file contains the "main" function for an MPLAB Harmony project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state 
    machines of all MPLAB Harmony modules in the system and it calls the 
    "SYS_Tasks" function from within a system-wide "super" loop to maintain 
    their correct operation. These two functions are implemented in 
    configuration-specific files (usually "system_init.c" and "system_tasks.c")
    in a configuration-specific folder under the "src/system_config" folder 
    within this project's top-level folder.  An MPLAB Harmony project may have
    more than one configuration, each contained within it's own folder under
    the "system_config" folder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************
void init_OC(void);

int main ( void )
{
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );
    __builtin_disable_interrupts();
    init_OC();
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;     // output
    TRISBbits.TRISB4 = 1;     // input

    LATAbits.LATA4 = 1;       // initialize LED on
    
    __builtin_enable_interrupts();
    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
        
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}

void init_OC(void){
  TRISBbits.TRISB9 = 0;      // set RB9 output for phase A
  TRISBbits.TRISB13 = 0;     // set RB13 output for phase B
  LATBbits.LATB9 = 1;
  LATBbits.LATB13 = 1;
  TRISBbits.TRISB15 = 0;     // set RB15 output for OC1
  TRISBbits.TRISB8 = 0;     // set RB8 output for OC2
  RPB15Rbits.RPB15R = 0b0101; // assign RA0 for OC1
  RPB8Rbits.RPB8R = 0b0101; // assign RA1 for OC2

  T2CONbits.TCKPS = 0b011;     // prescale = 8 / N = 8
  PR2 = 5999;                  // period = (PR2 + 1) * N * ? ns = ?  / 1kHz
  OC1RS = 0;                // duty cycle = OC1RS/(PR2+1) = 50%
  OC2RS = 0;
  TMR2 = 0;
  OC1CONbits.OCM = 0b110;   // PWM
  OC2CONbits.OCM = 0b110;   // PWM
  OC1CONbits.OCTSEL = 0;  // use timer2 for OC1
  OC2CONbits.OCTSEL = 0;  // use timer2 for OC2
  T2CONbits.ON = 1;       // turn on time2
  OC1CONbits.ON = 1;      // turn on OC1
  OC2CONbits.ON = 1;      // turn on OC2


}
/*******************************************************************************
 End of File
*/

