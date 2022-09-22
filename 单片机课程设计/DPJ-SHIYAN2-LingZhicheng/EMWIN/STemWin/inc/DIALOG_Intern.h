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
File        : Dialog.h
Purpose     : Dialog box include
--------------------END-OF-HEADER-------------------------------------
*/

#ifndef DIALOG_INTERN_H
#define DIALOG_INTERN_H

#include "WM.h"

#if GUI_WINSUPPORT

#if defined(__cplusplus)
  extern "C" {        // Make sure we have C-declarations in C++ programs
#endif

/*********************************************************************
*
*       Types
*
**********************************************************************
*/
typedef struct  GUI_WIDGET_CREATE_INFO_struct GUI_WIDGET_CREATE_INFO;
typedef WM_HWIN GUI_WIDGET_CREATE_FUNC        (const GUI_WIDGET_CREATE_INFO * pCreate, WM_HWIN hWin, int x0, int y0, WM_CALLBACK * cb);

/*********************************************************************
*
*       Structures
*
**********************************************************************
*/
struct GUI_WIDGET_CREATE_INFO_struct {
  GUI_WIDGET_CREATE_FUNC * pfCreateIndirect;
  const char             * pName;            // Text ... Not used on all widgets
  I16                      Id;               // ID ... should be unique in a dialog
  I16                      x0;               // x position
  I16                      y0;               // y position
  I16                      xSize;            // x size
  I16                      ySize;            // y size
  U16                      Flags;            // Widget specific create flags (opt.)
  I32                      Para;             // Widget specific parameter (opt.)
  U32                      NumExtraBytes;    // Number of extra bytes usable with <WIDGET>_SetUserData & <WIDGET>_GetUserData
};

/*********************************************************************
*
*       Public API functions
*
**********************************************************************
*/
WM_HWIN            GUI_CreateDialogBox   (const GUI_WIDGET_CREATE_INFO * paWidget, int NumWidgets, WM_CALLBACK * cb, WM_HWIN hParent, int x0, int y0);
void               GUI_EndDialog         (WM_HWIN hWin, int r);
int                GUI_ExecDialogBox     (const GUI_WIDGET_CREATE_INFO * paWidget, int NumWidgets, WM_CALLBACK * cb, WM_HWIN hParent, int x0, int y0);
int                GUI_ExecCreatedDialog (WM_HWIN hDialog);
WM_DIALOG_STATUS * GUI_GetDialogStatusPtr(WM_HWIN hDialog);                                    // Not to be documented
void               GUI_SetDialogStatusPtr(WM_HWIN hDialog, WM_DIALOG_STATUS * pDialogStatus);  // Not to be documented

/*********************************************************************
*
*       Obsolete
*/
LCD_COLOR          DIALOG_GetBkColor(void);
LCD_COLOR          DIALOG_SetBkColor(LCD_COLOR BkColor);

#if defined(__cplusplus)
  }
#endif

#endif  // GUI_WINSUPPORT
#endif  // DIALOG_INTERN_H

/*************************** End of file ****************************/
