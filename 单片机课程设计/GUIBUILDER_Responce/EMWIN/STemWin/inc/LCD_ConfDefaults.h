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
File        : LCD_ConfDefaults.h
Purpose     : Valid LCD configuration and defaults
----------------------------------------------------------------------
*/

#ifndef LCD_CONFIG_DEFAULTS_H
#define LCD_CONFIG_DEFAULTS_H

#include "LCDConf.h"            /* Configuration header file */

/**********************************************************
*
*       Configuration defaults
*/
#ifndef   LCD_MIRROR_X
  #define LCD_MIRROR_X 0
#endif
#ifndef   LCD_MIRROR_Y
  #define LCD_MIRROR_Y 0
#endif
#ifndef   LCD_SWAP_XY
  #define LCD_SWAP_XY 0
#endif
#ifndef   LCD_FIRSTCOM0
  #define LCD_FIRSTCOM0 0
#endif
#ifndef   LCD_FIRSTSEG0
  #define LCD_FIRSTSEG0 0
#endif
#ifndef   LCD_SWAP_RB
  #define LCD_SWAP_RB 0
#endif
#ifndef   LCD_DISPLAY_INDEX
  #define LCD_DISPLAY_INDEX 0
#endif
#ifndef   LCD_ENDIAN_BIG
  #define LCD_ENDIAN_BIG 0
#endif
#ifndef   LCD_ALLOW_NON_OPTIMIZED_MODE
  #define LCD_ALLOW_NON_OPTIMIZED_MODE 1
#endif

#endif /* LCD_CONFIG_DEFAULTS_H */

/*************************** End of file ****************************/
