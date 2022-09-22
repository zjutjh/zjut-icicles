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
File        : LCD_X.h
Purpose     : LCD port driver API
----------------------------------------------------------------------
*/

#ifndef LCD_X_H
#define LCD_X_H

extern void LCD_X_Config(void);

extern void LCD_X_Init(void);

extern void LCD_X_DisplayOn (void);
extern void LCD_X_DisplayOff(void);

/* 8-bit Interface */
extern void LCD_X_Write0_8(U8 c);
extern U8   LCD_X_Read0_8 (void);
extern void LCD_X_Write1_8(U8 c);
extern U8   LCD_X_Read1_8 (void);
extern void LCD_X_WriteM1_8(U8 * pData, int NumItems);
extern void LCD_X_ReadM1_8 (U8 * pData, int NumItems);

/* 16-bit Interface */
extern void LCD_X_Write0_16(U16 c);
extern U16  LCD_X_Read0_16 (void);
extern void LCD_X_Write1_16(U16 c);
extern U16  LCD_X_Read1_16 (void);
extern void LCD_X_WriteM1_16(U16 * pData, int NumItems);
extern void LCD_X_ReadM1_16 (U16 * pData, int NumItems);

/* 32-bit Interface */
extern void LCD_X_Write0_32(U32 c);
extern U32  LCD_X_Read0_32 (void);
extern void LCD_X_Write1_32(U32 c);
extern U32  LCD_X_Read1_32 (void);
extern void LCD_X_WriteM1_32(U32 * pData, int NumItems);
extern void LCD_X_ReadM1_32 (U32 * pData, int NumItems);

/* SPI Interface */
extern void LCD_X_ClrCS(void);
extern void LCD_X_SetCS(void);
extern void LCD_X_WriteM(U8 * pData, int NumBytes);
extern void LCD_X_ReadM (U8 * pData, int NumBytes);

extern void LCD_X_SetVRAMAddr(U32 addr);

extern volatile int LCD_X_PendingBuffer;

#endif /* LCD_X_H */

/*************************** End of file ****************************/
