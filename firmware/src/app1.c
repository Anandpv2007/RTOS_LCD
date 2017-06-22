/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app1.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
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

#include "app1.h" 

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP1_DATA app1Data;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP1_Initialize ( void )

  Remarks:
    See prototype in app1.h.
 */

void APP1_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app1Data.state = APP1_STATE_INIT;

    TRISEbits.TRISE1 = 0;
    TRISEbits.TRISE0 = 0;
    /////////////////
    
   
     TRIS_SCK = 0;
    TRIS_SDA = 0;
    TRIS_A0 = 0;
    TRIS_CS = 0;
    TRIS_RST = 0;
   
    
    
    SPI3CONbits.DISSDO = 0;
    SPI3CONbits.MODE16 = 0;
    SPI3CONbits.MODE32 = 0;
    SPI3CONbits.MSTEN = 1;  
    SPI3BRG = 0;
    SPI3CONbits.CKE = 1;    
    SPI3CONbits.ON = 1;
     
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

void DelayMs(int delay_mss)
{ 
    vTaskDelay(delay_mss / portTICK_PERIOD_MS);
 
    
}



///XMIT data
void  xmitbyte(unsigned char dat_a)
{
  //  int tempc;
   // unsigned char rxdat=0;
    while(SPI3STATbits.SPIBUSY);
    SPI3BUF = dat_a;
   ///Software SPI
   // for(tempc=0; tempc<8; tempc++)
   // {  
   //     SCK_ = 0;
  ////     MOSI_ = 0;
  //      if(dat_a & 0x80)
   //         MOSI_ = 1;   
   //     dat_a = dat_a<<1; 
    //    SCK_ = 1; 
   // }                   
}

void writecommand(unsigned char c)
{
    A0 = 0;
    xmitbyte(c); 
} 

void writedata(unsigned char c)
{
    A0  = 1;
    xmitbyte(c); 
} 


void init_lcd()
{
     CS = 0;
    ////////////////////////////////
     RST = 1;
    DelayMs(50);
    RST = 0;
     
    DelayMs(50);
    RST = 1;
    DelayMs(50);
    
     
    writecommand(ST7735_SWRESET); 
    DelayMs(120);
    writecommand(ST7735_SLPOUT); 
    DelayMs(120);     
    writecommand(ST7735_DISPON); 
    DelayMs(120);     
     
    
    writecommand(ST7735_CASET);  
	writedata(0x00); 
	writedata(0x00);   // XSTART = 2 
	writedata(0x00); 
	writedata(127);   // XEND = 128 
        DelayMs(100);
    writecommand(ST7735_RASET);  
	writedata(0x00); 
	writedata(0x00);   // XSTART = 2]] 
	writedata(0x00); 
	writedata(159);   // XEND = 160 
      DelayMs(100);    
    
    
}


/******************************************************************************
  Function:
    void APP1_Tasks ( void )

  Remarks:
    See prototype in app1.h.
 */
    char R = 0,G = 255,B;
void APP1_Tasks ( void )
{
    

      
    R  = ~R;
    G=~G;
    int x,y;
    /* Check the application's current state. */
    switch ( app1Data.state )
    {
        /* Application's initial state. */
        case APP1_STATE_INIT:
        {
            bool appInitialized = true;
            
        
            if (appInitialized)
            {
                init_lcd();
            
                app1Data.state = APP1_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP1_STATE_SERVICE_TASKS:
        {
             
            DelayMs(500); 
            LATEbits.LATE0 = !LATEbits.LATE0;       
            
            
            writecommand(ST7735_RAMWR); 
            DelayMs(10);
 
            for(x=0;x<128;x++)
                for(y=0;y<160;y++)
                { 
                    writedata(R);  
                    writedata(G);  
                    writedata(B);    
                }
        
                  
 
         
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
