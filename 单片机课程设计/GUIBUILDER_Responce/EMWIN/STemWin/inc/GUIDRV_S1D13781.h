/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2015  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.30 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software has been licensed to  ARM LIMITED whose registered office
is situated at  110 Fulbourn Road,  Cambridge CB1 9NJ,  England solely
for  the  purposes  of  creating  libraries  for  ARM7, ARM9, Cortex-M
series,  and   Cortex-R4   processor-based  devices,  sublicensed  and
distributed as part of the  MDK-ARM  Professional  under the terms and
conditions  of  the   End  User  License  supplied  with  the  MDK-ARM
Professional. 
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information

Licensor:                 SEGGER Software GmbH
Licensed to:              ARM Ltd
Licensed SEGGER software: emWin
License number:           GUI-00181
License model:            LES-SLA-20007, Agreement, effective since October 1st 2011 
Licensed product:         MDK-ARM Professional
Licensed platform:        ARM7/9, Cortex-M/R4
Licensed number of seats: -
----------------------------------------------------------------------
File        : GUIDRV_S1D13781.h
Purpose     : Interface definition for GUIDRV_S1D13781 driver
---------------------------END-OF-HEADER------------------------------
*/

#ifndef GUIDRV_S1D13781_H
#define GUIDRV_S1D13781_H

#define GUIDRV_S1D13781_USE_MAIN 0
#define GUIDRV_S1D13781_USE_PIP1 1
#define GUIDRV_S1D13781_USE_PIP2 2

/*********************************************************************
*
*       Configuration structure
*/
typedef struct {
  //
  // Driver specific configuration items
  //
  U32 BufferOffset;
  int UseLayer;
  int WriteBufferSize;
  int WaitUntilVNDP;
} CONFIG_S1D13781;

/*********************************************************************
*
*       Display drivers
*/
//
// Addresses
//
extern const GUI_DEVICE_API GUIDRV_S1D13781_8C0_API;
extern const GUI_DEVICE_API GUIDRV_S1D13781_OXY_8C0_API;
extern const GUI_DEVICE_API GUIDRV_S1D13781_OSY_8C0_API;
extern const GUI_DEVICE_API GUIDRV_S1D13781_OSX_8C0_API;

//
// Macros to be used in configuration files
//
#if defined(WIN32) && !defined(LCD_SIMCONTROLLER)

  #define GUIDRV_S1D13781_8C0       &GUIDRV_Win_API
  #define GUIDRV_S1D13781_OXY_8C0   &GUIDRV_Win_API
  #define GUIDRV_S1D13781_OSY_8C0   &GUIDRV_Win_API
  #define GUIDRV_S1D13781_OSX_8C0   &GUIDRV_Win_API

#else

  #define GUIDRV_S1D13781_8C0       &GUIDRV_S1D13781_8C0_API
  #define GUIDRV_S1D13781_OXY_8C0   &GUIDRV_S1D13781_OXY_8C0_API
  #define GUIDRV_S1D13781_OSY_8C0   &GUIDRV_S1D13781_OSY_8C0_API
  #define GUIDRV_S1D13781_OSX_8C0   &GUIDRV_S1D13781_OSX_8C0_API

#endif

/*********************************************************************
*
*       Public routines
*/
#if defined(WIN32) && !defined(LCD_SIMCONTROLLER)
  #define GUIDRV_S1D13781_Config(pDevice, pConfig)
  #define GUIDRV_S1D13781_SetBusSPI(pDevice, pHW_API)
#else
  void GUIDRV_S1D13781_Config   (GUI_DEVICE * pDevice, CONFIG_S1D13781 * pConfig);
  void GUIDRV_S1D13781_SetBusSPI(GUI_DEVICE * pDevice, GUI_PORT_API * pHW_API);
#endif

#endif

/*************************** End of file ****************************/
