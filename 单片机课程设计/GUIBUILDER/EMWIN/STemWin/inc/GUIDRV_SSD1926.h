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
File        : GUIDRV_SSD1926.h
Purpose     : Interface definition for GUIDRV_SSD1926 driver
---------------------------END-OF-HEADER------------------------------
*/

#ifndef GUIDRV_SSD1926_H
#define GUIDRV_SSD1926_H

#if defined(__cplusplus)
extern "C" {     /* Make sure we have C-declarations in C++ programs */
#endif

/*********************************************************************
*
*       Configuration structure
*/
typedef struct {
  //
  // Driver specific configuration items
  //
  int FirstSEG;
  int FirstCOM;
  int UseCache;
} CONFIG_SSD1926;

/*********************************************************************
*
*       Display drivers
*/
//
// Addresses
//
extern const GUI_DEVICE_API GUIDRV_SSD1926_8_API;
extern const GUI_DEVICE_API GUIDRV_SSD1926_OY_8_API;
extern const GUI_DEVICE_API GUIDRV_SSD1926_OX_8_API;
extern const GUI_DEVICE_API GUIDRV_SSD1926_OXY_8_API;
extern const GUI_DEVICE_API GUIDRV_SSD1926_OS_8_API;
extern const GUI_DEVICE_API GUIDRV_SSD1926_OSY_8_API;
extern const GUI_DEVICE_API GUIDRV_SSD1926_OSX_8_API;
extern const GUI_DEVICE_API GUIDRV_SSD1926_OSXY_8_API;

//
// Macros to be used in configuration files
//
#if defined(WIN32) && !defined(LCD_SIMCONTROLLER)

  #define GUIDRV_SSD1926_8       &GUIDRV_Win_API
  #define GUIDRV_SSD1926_OY_8    &GUIDRV_Win_API
  #define GUIDRV_SSD1926_OX_8    &GUIDRV_Win_API
  #define GUIDRV_SSD1926_OXY_8   &GUIDRV_Win_API
  #define GUIDRV_SSD1926_OS_8    &GUIDRV_Win_API
  #define GUIDRV_SSD1926_OSY_8   &GUIDRV_Win_API
  #define GUIDRV_SSD1926_OSX_8   &GUIDRV_Win_API
  #define GUIDRV_SSD1926_OSXY_8  &GUIDRV_Win_API

#else

  #define GUIDRV_SSD1926_8       &GUIDRV_SSD1926_8_API
  #define GUIDRV_SSD1926_OY_8    &GUIDRV_SSD1926_OY_8_API
  #define GUIDRV_SSD1926_OX_8    &GUIDRV_SSD1926_OX_8_API
  #define GUIDRV_SSD1926_OXY_8   &GUIDRV_SSD1926_OXY_8_API
  #define GUIDRV_SSD1926_OS_8    &GUIDRV_SSD1926_OS_8_API
  #define GUIDRV_SSD1926_OSY_8   &GUIDRV_SSD1926_OSY_8_API
  #define GUIDRV_SSD1926_OSX_8   &GUIDRV_SSD1926_OSX_8_API
  #define GUIDRV_SSD1926_OSXY_8  &GUIDRV_SSD1926_OSXY_8_API

#endif

/*********************************************************************
*
*       Public routines
*/
void GUIDRV_SSD1926_Config  (GUI_DEVICE * pDevice, CONFIG_SSD1926 * pConfig);
void GUIDRV_SSD1926_SetBus16(GUI_DEVICE * pDevice, GUI_PORT_API * pHW_API);

#if defined(__cplusplus)
}
#endif

#endif

/*************************** End of file ****************************/
