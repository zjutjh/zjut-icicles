#ifndef _LCDCONF_H
#define _LCDCONF_H

#include "sys.h"
#include "GUI.h"
#include "GUIDRV_Lin.h"

typedef struct
{
  int32_t      address;          
  __IO int32_t pending_buffer;   
  int32_t      buffer_index;     
  int32_t      xSize;            
  int32_t      ySize;            
  int32_t      BytesPerPixel;
  LCD_API_COLOR_CONV   *pColorConvAPI;
}
LCD_LayerPropTypedef;

#endif

