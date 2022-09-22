#include "LCDConf.h"
#include "GUI_Private.h"
#include "sys.h"
#include "led.h" 
#include "ltdc.h"
#include "tftlcd.h"
#include "GUIDRV_Lin.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//STemWin LCD底层驱动函数 	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/3/15
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define XSIZE_PHYS 1024     //最大支持1024*768
#define YSIZE_PHYS 768

#define NUM_BUFFERS  3      //使用多缓冲时的缓冲数量
#define NUM_VSCREENS 1      //使用虚拟屏幕是的虚拟屏幕数量

#undef  GUI_NUM_LAYERS
#define GUI_NUM_LAYERS 1    //层数

#define COLOR_CONVERSION_0 GUICC_M565
#define DISPLAY_DRIVER_0   GUIDRV_LIN_16

#if (GUI_NUM_LAYERS > 1)
  #define COLOR_CONVERSION_1 GUICC_M1555I
  #define DISPLAY_DRIVER_1   GUIDRV_LIN_16
#endif

#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   NUM_VSCREENS
  #define NUM_VSCREENS 1
#else
  #if (NUM_VSCREENS <= 0)
    #error At least one screeen needs to be defined!
  #endif
#endif
#if (NUM_VSCREENS > 1) && (NUM_BUFFERS > 1)
  #error Virtual screens and multiple buffers are not allowed!
#endif

//LCD两层的缓冲区需要根据实际所使用的屏幕分辨率以及每层所使用的颜色格式来计算，
//最终确定最佳的缓冲区大小,这里默认设置最大分辨率为1024*600，颜色格式为8888，
//那么每层所需要的缓冲区就是1024*600*4=2457600=0X258000,注意mlloc.h和malloc.c
//中的外部SDRAM内存管理池地址和大小也需要做修改！！！！
#define LCD_LAYER0_FRAME_BUFFER  ((uint32_t)0xC0000000)     //第一层缓冲区
#define LCD_LAYER1_FRAME_BUFFER  ((uint32_t)0xC0300000)     //第二层缓冲区
  

#define DEFINEDMA2D_COLORCONVERSION(PFIX, PIXELFORMAT)                                                             \
static void Color2IndexBulk_##PFIX##DMA2D(LCD_COLOR * pColor, void * pIndex, U32 NumItems, U8 SizeOfIndex) { \
  DMA2D_Color2IndexBulk(pColor, pIndex, NumItems, SizeOfIndex, PIXELFORMAT);                                         \
}                                                                                                                   \
static void Index2ColorBulk_##PFIX##DMA2D(void * pIndex, LCD_COLOR * pColor, U32 NumItems, U8 SizeOfIndex) { \
  DMA2D_Index2ColorBulk(pIndex, pColor, NumItems, SizeOfIndex, PIXELFORMAT);  \
}
   
static LCD_LayerPropTypedef     layer_prop[GUI_NUM_LAYERS];
static const LCD_API_COLOR_CONV *apColorConvAPI[]= 
{
  COLOR_CONVERSION_0,
#if GUI_NUM_LAYERS > 1
  COLOR_CONVERSION_1,
#endif
};

static U32 aBufferDMA2D [XSIZE_PHYS * sizeof(U32)];
static U32 aBuffer_FG   [XSIZE_PHYS * sizeof(U32)];
static U32 aBuffer_BG   [XSIZE_PHYS * sizeof(U32)];

static uint32_t LCD_LL_GetPixelformat(uint32_t LayerIndex);
static void     DMA2D_CopyBuffer(uint32_t LayerIndex, void * pSrc, void * pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLineSrc, uint32_t OffLineDst);
static void     DMA2D_FillBuffer(uint32_t LayerIndex, void * pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex);
static void     LCD_LL_Init(void); 
static void     LCD_LL_LayerInit(uint32_t LayerIndex); 

static void     CUSTOM_CopyBuffer(int32_t LayerIndex, int32_t IndexSrc, int32_t IndexDst);
static void     CUSTOM_CopyRect(int32_t LayerIndex, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t xSize, int32_t ySize);
static void     CUSTOM_FillRect(int32_t LayerIndex, int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t PixelIndex);

static void     DMA2D_Index2ColorBulk(void * pIndex, LCD_COLOR * pColor, uint32_t NumItems, U8 SizeOfIndex, uint32_t PixelFormat);
static void     DMA2D_Color2IndexBulk(LCD_COLOR * pColor, void * pIndex, uint32_t NumItems, U8 SizeOfIndex, uint32_t PixelFormat);

static void LCD_DrawBitmap8bpp(int32_t LayerIndex, int32_t x, int32_t y, U8 const * p, int32_t xSize, int32_t ySize, int32_t BytesPerLine);
static void LCD_DrawBitmap16bpp(int32_t LayerIndex, int32_t x, int32_t y, U16 const * p, int32_t xSize, int32_t ySize, int32_t BytesPerLine);
static void DMA2D_AlphaBlendingBulk(LCD_COLOR * pColorFG, LCD_COLOR * pColorBG, LCD_COLOR * pColorDst, U32 NumItems);
static void DMA2D_AlphaBlending(LCD_COLOR * pColorFG, LCD_COLOR * pColorBG, LCD_COLOR * pColorDst, U32 NumItems);
static LCD_PIXELINDEX * _LCD_GetpPalConvTable(const LCD_LOGPALETTE GUI_UNI_PTR * pLogPal, const GUI_BITMAP GUI_UNI_PTR * pBitmap, int LayerIndex);
static LCD_COLOR DMA2D_MixColors(LCD_COLOR Color, LCD_COLOR BkColor, U8 Intens);
static void LCD_MixColorsBulk(U32 * pFG, U32 * pBG, U32 * pDst, unsigned OffFG, unsigned OffBG, unsigned OffDest, unsigned xSize, unsigned ySize, U8 Intens);

DEFINEDMA2D_COLORCONVERSION(M8888I, LTDC_PIXEL_FORMAT_ARGB8888)
DEFINEDMA2D_COLORCONVERSION(M888,   LTDC_PIXEL_FORMAT_ARGB8888)
//DEFINEDMA2D_COLORCONVERSION(M565,   LTDC_PIXEL_FORMAT_RGB565)
DEFINEDMA2D_COLORCONVERSION(M1555I, LTDC_PIXEL_FORMAT_ARGB1555)
DEFINEDMA2D_COLORCONVERSION(M4444I, LTDC_PIXEL_FORMAT_ARGB4444)

static uint32_t GetBufferSize(uint32_t LayerIndex);
 
//LTDC中断服务函数
void LTDC_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&LTDC_Handler);
}

//LTDC行事件回调函数
//LTDC_Handler:LTDC句柄
void HAL_LTDC_LineEvenCallback(LTDC_HandleTypeDef *LTDC_Handler)
{
    uint32_t Addr;
    uint32_t layer=0;

    for (layer=0;layer<GUI_NUM_LAYERS;layer++) 
    {
        if(layer_prop[layer].pending_buffer>=0) 
        {
            //计算显示的帧地址
            Addr=layer_prop[layer].address + \
                 layer_prop[layer].xSize * layer_prop[layer].ySize * layer_prop[layer].pending_buffer * layer_prop[layer].BytesPerPixel;
            HAL_LTDC_SetAddress(LTDC_Handler,Addr,layer);//设置地址
            __HAL_LTDC_RELOAD_CONFIG(LTDC_Handler);
            GUI_MULTIBUF_ConfirmEx(layer,layer_prop[layer].pending_buffer);
            layer_prop[layer].pending_buffer=-1;
        }
    }
    HAL_LTDC_ProgramLineEvent(LTDC_Handler,0);
}

//配置程序,用于创建显示驱动器件,设置颜色转换程序和显示尺寸
void LCD_X_Config(void) 
{
    uint32_t i;
  
    LCD_LL_Init ();                                 //LCD底层驱动(LTDC中断设置和DMA2D初始化)
#if (NUM_BUFFERS>1)                                 //多缓冲
    for (i=0;i<GUI_NUM_LAYERS; i++) 
    {
        GUI_MULTIBUF_ConfigEx(i, NUM_BUFFERS);
    }
#endif
    //设置第一层
    GUI_DEVICE_CreateAndLink(DISPLAY_DRIVER_0,COLOR_CONVERSION_0,0,0);//创建显示驱动器件
    GUI_SelectLayer(0);     //选中第0层
    if(lcddev.dir==0)//竖屏
    {
        LCD_SetSizeEx(0,lcddev.height,lcddev.width);    //设置可见区尺寸
        LCD_SetVSizeEx(0,lcddev.height,lcddev.width*NUM_VSCREENS);   //设置虚拟显示区尺寸
        GUI_SetOrientation(GUI_SWAP_XY|GUI_MIRROR_Y);                   //设置为竖屏
    }else            //横屏
    {
    	LCD_SetSizeEx(0,lcddev.width,lcddev.height);    //设置可见区尺寸
        LCD_SetVSizeEx(0,lcddev.width,lcddev.height*NUM_VSCREENS);   //设置虚拟显示区尺寸
    }
    GUI_TOUCH_Calibrate(GUI_COORD_X,0,lcddev.width,0,lcddev.width-1);   
    GUI_TOUCH_Calibrate(GUI_COORD_Y,0,lcddev.height,0,lcddev.height-1);
#if (GUI_NUM_LAYERS>1)
  
    //设置第二层
    GUI_DEVICE_CreateAndLink(DISPLAY_DRIVER_1,COLOR_CONVERSION_1,0,1);
    GUI_SelectLayer(1);     //选中第1层
    if(lcddev.dir==0)//竖屏
    {
        LCD_SetSizeEx(1,lcddev.height,lcddev.width);    //设置可见区尺寸
        LCD_SetVSizeEx(1,lcddev.height,lcddev.width*NUM_VSCREENS);   //设置虚拟显示区尺寸
        GUI_SetOrientation(GUI_SWAP_XY|GUI_MIRROR_Y);                   //设置为竖屏
    }else            //横屏
    {
    	LCD_SetSizeEx(1,lcddev.width,lcddev.height);    //设置可见区尺寸
        LCD_SetVSizeEx(1,lcddev.width,lcddev.height*NUM_VSCREENS);   //设置虚拟显示区尺寸
    }
    GUI_TOUCH_Calibrate(GUI_COORD_X,0,lcddev.width,0,lcddev.width-1);   
    GUI_TOUCH_Calibrate(GUI_COORD_Y,0,lcddev.height,0,lcddev.height-1);
#endif
  
    layer_prop[0].address=LCD_LAYER0_FRAME_BUFFER;           //缓冲区
#if (GUI_NUM_LAYERS>1)
    layer_prop[1].address=LCD_LAYER1_FRAME_BUFFER;     
#endif
    
    for (i=0;i<GUI_NUM_LAYERS;i++) 
    {
        layer_prop[i].pColorConvAPI=(LCD_API_COLOR_CONV *)apColorConvAPI[i];
        layer_prop[i].pending_buffer=-1;
        LCD_SetVRAMAddrEx(i,(void *)(layer_prop[i].address));
        layer_prop[i].BytesPerPixel=LCD_GetBitsPerPixelEx(i) >> 3;
        LCD_SetDevFunc(i,LCD_DEVFUNC_COPYBUFFER,(void(*)(void))CUSTOM_CopyBuffer);
        LCD_SetDevFunc(i,LCD_DEVFUNC_COPYRECT,(void(*)(void))CUSTOM_CopyRect);
        if (LCD_LL_GetPixelformat(i)<=LTDC_PIXEL_FORMAT_ARGB4444) 
        {
            LCD_SetDevFunc(i,LCD_DEVFUNC_FILLRECT,(void(*)(void))CUSTOM_FillRect);
            LCD_SetDevFunc(i,LCD_DEVFUNC_DRAWBMP_8BPP,(void(*)(void))LCD_DrawBitmap8bpp);
        }
        if(LCD_LL_GetPixelformat(i)==LTDC_PIXEL_FORMAT_RGB565) 
        {
           LCD_SetDevFunc(i,LCD_DEVFUNC_DRAWBMP_16BPP,(void(*)(void))LCD_DrawBitmap16bpp);    
        }
        GUICC_M1555I_SetCustColorConv(Color2IndexBulk_M1555IDMA2D,Index2ColorBulk_M1555IDMA2D); 
        //GUICC_M565_SetCustColorConv(Color2IndexBulk_M565DMA2D,Index2ColorBulk_M565DMA2D);  
        GUICC_M4444I_SetCustColorConv(Color2IndexBulk_M4444IDMA2D,Index2ColorBulk_M4444IDMA2D); 
        GUICC_M888_SetCustColorConv(Color2IndexBulk_M888DMA2D,Index2ColorBulk_M888DMA2D);   
        GUICC_M8888I_SetCustColorConv(Color2IndexBulk_M8888IDMA2D,Index2ColorBulk_M8888IDMA2D);
        GUI_SetFuncAlphaBlending(DMA2D_AlphaBlending);                                            
        GUI_SetFuncGetpPalConvTable(_LCD_GetpPalConvTable);
        GUI_SetFuncMixColors(DMA2D_MixColors);
        GUI_SetFuncMixColorsBulk(LCD_MixColorsBulk);
    }
}

//显示驱动回调函数
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) 
{
    int32_t r = 0;
    uint32_t addr;
    int32_t xPos, yPos;
    uint32_t Color;
    
    switch (Cmd) 
    {
      case LCD_X_INITCONTROLLER: 
            LCD_LL_LayerInit(LayerIndex); 
            break;
        case LCD_X_SETORG: 
            addr=layer_prop[LayerIndex].address+((LCD_X_SETORG_INFO *)pData)->yPos*layer_prop[LayerIndex].xSize*layer_prop[LayerIndex].BytesPerPixel;
            HAL_LTDC_SetAddress(&LTDC_Handler,addr,LayerIndex);
            break;
        case LCD_X_SHOWBUFFER: 
            layer_prop[LayerIndex].pending_buffer=((LCD_X_SHOWBUFFER_INFO *)pData)->Index;
            break;
        case LCD_X_SETLUTENTRY: 
            HAL_LTDC_ConfigCLUT(&LTDC_Handler,(uint32_t *)&(((LCD_X_SETLUTENTRY_INFO *)pData)->Color),1,LayerIndex);
            break;
        case LCD_X_ON: 
            __HAL_LTDC_ENABLE(&LTDC_Handler);
            break;
        case LCD_X_OFF: 
            __HAL_LTDC_DISABLE(&LTDC_Handler);
            break;
        case LCD_X_SETVIS:
            if(((LCD_X_SETVIS_INFO *)pData)->OnOff==ENABLE )
            {
                __HAL_LTDC_LAYER_ENABLE(&LTDC_Handler,LayerIndex); 
            }
            else
            {
                __HAL_LTDC_LAYER_DISABLE(&LTDC_Handler,LayerIndex); 
            }
            __HAL_LTDC_RELOAD_CONFIG(&LTDC_Handler); 
            break;
        case LCD_X_SETPOS: 
            HAL_LTDC_SetWindowPosition(&LTDC_Handler, 
                                      ((LCD_X_SETPOS_INFO *)pData)->xPos, 
                                      ((LCD_X_SETPOS_INFO *)pData)->yPos, 
                                      LayerIndex);
            break;
        case LCD_X_SETSIZE:
            GUI_GetLayerPosEx(LayerIndex,(int*)&xPos,(int*)&yPos);
            layer_prop[LayerIndex].xSize=((LCD_X_SETSIZE_INFO *)pData)->xSize;
            layer_prop[LayerIndex].ySize=((LCD_X_SETSIZE_INFO *)pData)->ySize;
            HAL_LTDC_SetWindowPosition(&LTDC_Handler,xPos,yPos,LayerIndex);
            break;
        case LCD_X_SETALPHA:
            HAL_LTDC_SetAlpha(&LTDC_Handler,((LCD_X_SETALPHA_INFO *)pData)->Alpha,LayerIndex);
            break;
        case LCD_X_SETCHROMAMODE:
            if(((LCD_X_SETCHROMAMODE_INFO *)pData)->ChromaMode != 0)
            {
                HAL_LTDC_EnableColorKeying(&LTDC_Handler,LayerIndex);
            }
            else
            {
                HAL_LTDC_DisableColorKeying(&LTDC_Handler,LayerIndex);      
            }
            break;
        case LCD_X_SETCHROMA:
            Color=((((LCD_X_SETCHROMA_INFO *)pData)->ChromaMin & 0xFF0000) >> 16) |\
                    (((LCD_X_SETCHROMA_INFO *)pData)->ChromaMin & 0x00FF00) |\
                    ((((LCD_X_SETCHROMA_INFO *)pData)->ChromaMin & 0x0000FF) << 16);
    
            HAL_LTDC_ConfigColorKeying(&LTDC_Handler,Color,LayerIndex);
            break;
        default:
            r = -1;
    }
    return r;
}

//LTDC层配置
static void LCD_LL_LayerInit(uint32_t LayerIndex) 
{
    uint32_t i;
    static uint32_t LUT[256];
    LTDC_LayerCfgTypeDef layer_cfg;
  
    if (LayerIndex<GUI_NUM_LAYERS) 
    {
        layer_cfg.WindowX0=0;                                   //窗口起始X坐标
        layer_cfg.WindowY0=0;                                   //窗口起始Y坐标
        layer_cfg.WindowX1=lcdltdc.pwidth;                      //窗口终止X坐标
        layer_cfg.WindowY1=lcdltdc.pheight;                     //窗口终止Y坐标
        layer_cfg.PixelFormat=LCD_LL_GetPixelformat(LayerIndex);//像素格式
        layer_cfg.Alpha=255;                                    //Alpha值设置，0~255,255为完全不透明
        layer_cfg.Alpha0=0;;                                    //默认Alpha值
        layer_cfg.Backcolor.Red=0;                              //背景颜色红色部分
        layer_cfg.Backcolor.Green=0;                            //背景颜色绿色部分
        layer_cfg.Backcolor.Blue=0;                             //背景颜色蓝色部分       
        layer_cfg.FBStartAdress=layer_prop[LayerIndex].address; //设置层颜色帧缓存起始地址
        layer_cfg.BlendingFactor1=LTDC_BLENDING_FACTOR1_PAxCA;  //设置层混合系数
        layer_cfg.BlendingFactor2=LTDC_BLENDING_FACTOR2_PAxCA;  //设置层混合系数
        layer_cfg.ImageWidth=lcdltdc.pwidth;                    //设置颜色帧缓冲区的宽度  
        layer_cfg.ImageHeight=lcdltdc.pheight;                  //设置颜色帧缓冲区的高度
        HAL_LTDC_ConfigLayer(&LTDC_Handler,&layer_cfg,LayerIndex);//设置所选中的层  
        if (LCD_GetBitsPerPixelEx(LayerIndex)<=8) 
        {
            HAL_LTDC_EnableCLUT(&LTDC_Handler,LayerIndex);
        } 
        else 
        {
            if (layer_prop[LayerIndex].pColorConvAPI==GUICC_88666I) 
            {
                HAL_LTDC_EnableCLUT(&LTDC_Handler,LayerIndex);
                for (i=0;i<256;i++) 
                {
                    LUT[i]=LCD_API_ColorConv_8666.pfIndex2Color(i);
                }
                HAL_LTDC_ConfigCLUT(&LTDC_Handler,LUT,256,LayerIndex);
            }
        }
    }
}

//LCD底层驱动,LTDC中断设置，DMA2D初始化
static void LCD_LL_Init(void) 
{
    //LTDC中断，抢占优先级1，子优先级1
    HAL_NVIC_SetPriority(LTDC_IRQn,1,1);    
    HAL_NVIC_EnableIRQ(LTDC_IRQn);
    HAL_LTDC_ProgramLineEvent(&LTDC_Handler,0);//开启LTDC的行中断
  
    //DMA2D默认设置
    DMA2D_Handler.Instance=DMA2D; 
    DMA2D_Handler.Init.Mode=DMA2D_R2M;          //内存到存储器模式
    DMA2D_Handler.Init.ColorMode=DMA2D_RGB565;  //RGB565模式
    DMA2D_Handler.Init.OutputOffset=0x0;        //输出偏移为0    
    HAL_DMA2D_Init(&DMA2D_Handler);
}

//返回层的颜色格式
//返回值:1，失败；其他，颜色格式
static uint32_t LCD_LL_GetPixelformat(uint32_t LayerIndex)
{
    const LCD_API_COLOR_CONV * pColorConvAPI;
    if (LayerIndex>=GUI_NUM_LAYERS) return 0;
    pColorConvAPI=layer_prop[LayerIndex].pColorConvAPI;
    if(pColorConvAPI==GUICC_M8888I) return LTDC_PIXEL_FORMAT_ARGB8888;
    else if (pColorConvAPI==GUICC_M888) return LTDC_PIXEL_FORMAT_RGB888;
    else if (pColorConvAPI==GUICC_M565) return LTDC_PIXEL_FORMAT_RGB565;
    else if (pColorConvAPI==GUICC_M1555I) return LTDC_PIXEL_FORMAT_ARGB1555;
    else if (pColorConvAPI==GUICC_M4444I) return LTDC_PIXEL_FORMAT_ARGB4444;
    return 1;
}


//使用DMA2D将数据从一个buffer拷贝到另一个buffer中
//LayerIndex:层索引
//pSrc:源buffer
//pDst:目标buffer
//xSize:X轴大小(每行像素数)
//ySize:Y轴大小(行数)
//OffLineSrc:源buffer偏移地址
//OffLineDst:目的fuffer偏移地址
static void DMA2D_CopyBuffer(uint32_t LayerIndex,void * pSrc,void * pDst,uint32_t xSize,uint32_t ySize,uint32_t OffLineSrc,uint32_t OffLineDst)
{
    uint32_t PixelFormat;
    u32 timeout=0;
    PixelFormat=LCD_LL_GetPixelformat(LayerIndex);  //获取像素格式
    DMA2D->CR=0x00000000UL|(1<<9);                  //存储器到存储器
    DMA2D->FGMAR=(uint32_t)pSrc;                    //设置前景层存储器地址                   
    DMA2D->OMAR=(uint32_t)pDst;                     //设置输出存储器地址    
    DMA2D->FGOR=OffLineSrc;                         //设置前景层偏移
    DMA2D->OOR=OffLineDst;                          //设置输出偏移
    DMA2D->FGPFCCR=PixelFormat;                     //设置颜色模式
    DMA2D->NLR=(uint32_t)(xSize<<16)|(U16)ySize;    //设置每行像素数和行数
    DMA2D->CR|=DMA2D_CR_START;                      //启动DMA2D传输
    while(DMA2D->CR&DMA2D_CR_START)                 //等待传输完成 
    {
        timeout++;
		if(timeout>0X1FFFFF)break;	                //超时退出
    }
}

//使用指定的颜色填充一个矩形块
//LayerIndex:层索引
//pDst:目标buffer，要填充的矩形
//xSize:X轴大小(每行像素数)
//ySize:Y轴大小(行数)
//OffLineSrc:源buffer偏移地址
//OffLineDst:目的fuffer偏移地址
//ColorIndex:要填充的颜色值
static void DMA2D_FillBuffer(uint32_t LayerIndex,void * pDst,uint32_t xSize,uint32_t ySize,uint32_t OffLine,uint32_t ColorIndex) 
{
    uint32_t PixelFormat;
    u32 timeout=0;
    PixelFormat=LCD_LL_GetPixelformat(LayerIndex);  //获取像素格式
    DMA2D->CR=0x00030000UL|(1<<9);                  //寄存器到存储器 
    DMA2D->OCOLR=ColorIndex;                        //设置要填充的颜色                   
    DMA2D->OMAR=(uint32_t)pDst;                     //设置输出寄存器地址
    DMA2D->OOR=OffLine;                             //设置输出偏移
    DMA2D->OPFCCR=PixelFormat;                      //设置颜色格式
    DMA2D->NLR=(uint32_t)(xSize << 16)|(U16)ySize;  //设置每行像素数和行数
    DMA2D->CR|=DMA2D_CR_START;                      //启动DMA2D传输
    while(DMA2D->CR&DMA2D_CR_START)                 //等待传输完成 
    {
        timeout++;
		if(timeout>0X1FFFFF)break;	                //超时退出
    }
}

//获取buffer大小
//返回值:buffer大小
static uint32_t GetBufferSize(uint32_t LayerIndex) 
{
    uint32_t BufferSize;
    BufferSize = layer_prop[LayerIndex].xSize*layer_prop[LayerIndex].ySize*layer_prop[LayerIndex].BytesPerPixel;
    return BufferSize;
}

//用户自定义的缓冲区复制函数
//LayerIndex:层索引
//IndexSrc:待复制的源帧缓冲器的索引
//IndexDst:待覆盖的目标帧缓冲器的索引
static void CUSTOM_CopyBuffer(int32_t LayerIndex,int32_t IndexSrc,int32_t IndexDst) 
{
    uint32_t BufferSize, AddrSrc, AddrDst;

    BufferSize=GetBufferSize(LayerIndex);
    AddrSrc=layer_prop[LayerIndex].address+BufferSize*IndexSrc;
    AddrDst=layer_prop[LayerIndex].address+BufferSize*IndexDst;
    DMA2D_CopyBuffer(LayerIndex,(void *)AddrSrc,(void *)AddrDst,layer_prop[LayerIndex].xSize,layer_prop[LayerIndex].ySize,0,0);
    layer_prop[LayerIndex].buffer_index=IndexDst;
}

//用户自定义的将屏幕的一个矩形区域复制到目标位置
//LayerIndex:层索引
//x0:源矩形最左边的像素
//y0:源矩形最上边的像素
//x1:目标矩形最左边的像素
//y1:目标矩形最上边的像素
//xSize:矩形的X尺寸
//ySize:矩形的Y尺寸
static void CUSTOM_CopyRect(int32_t LayerIndex, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t xSize, int32_t ySize) 
{
    uint32_t BufferSize, AddrSrc, AddrDst;

    BufferSize=GetBufferSize(LayerIndex);
    AddrSrc=layer_prop[LayerIndex].address+BufferSize*layer_prop[LayerIndex].pending_buffer+(y0*layer_prop[LayerIndex].xSize+x0)*layer_prop[LayerIndex].BytesPerPixel;
    AddrDst=layer_prop[LayerIndex].address+BufferSize*layer_prop[LayerIndex].pending_buffer+(y1*layer_prop[LayerIndex].xSize+x1)*layer_prop[LayerIndex].BytesPerPixel;
    DMA2D_CopyBuffer(LayerIndex,(void *)AddrSrc,(void *)AddrDst,xSize,ySize,layer_prop[LayerIndex].xSize-xSize,0);
}

//用户自定义的填充函数
//LayerIndex:层索引
//x0:屏幕坐标中待填充的最左边的坐标
//y0:屏幕坐标中待填充的最上边的坐标
//x1:屏幕坐标中待填充的最右边的坐标
//y1:屏幕坐标中待填充的最下边坐标
//PixelIndex:待填充的颜色
static void CUSTOM_FillRect(int32_t LayerIndex,int32_t x0,int32_t y0,int32_t x1,int32_t y1,uint32_t PixelIndex) 
{
    uint32_t BufferSize, AddrDst;
    int32_t xSize, ySize;

    if (GUI_GetDrawMode()==GUI_DM_XOR) 
    {
        LCD_SetDevFunc(LayerIndex,LCD_DEVFUNC_FILLRECT,NULL);
        LCD_FillRect(x0,y0,x1,y1);
        LCD_SetDevFunc(LayerIndex,LCD_DEVFUNC_FILLRECT,(void(*)(void))CUSTOM_FillRect);
    }else 
    {
        xSize=x1-x0+1;
        ySize=y1-y0+1;
        BufferSize=GetBufferSize(LayerIndex);
        AddrDst=layer_prop[LayerIndex].address+BufferSize*layer_prop[LayerIndex].buffer_index+(y0 * layer_prop[LayerIndex].xSize+x0)*layer_prop[LayerIndex].BytesPerPixel;
        DMA2D_FillBuffer(LayerIndex,(void *)AddrDst,xSize,ySize,layer_prop[LayerIndex].xSize-xSize,PixelIndex);
    }
}

//DMA2D的CLUT加载
//pColor:CLUT存储器地址
//NumItems:CLUT大小
static void DMA2D_LoadLUT(LCD_COLOR * pColor,uint32_t NumItems) 
{
    DMA2D->FGCMAR=(uint32_t)pColor; //设置前景层CLUT存储器地址
    DMA2D->FGPFCCR=LTDC_PIXEL_FORMAT_RGB888|((NumItems-1)&0xFF)<<8;//设置颜色模式和CLUT大小
    DMA2D->FGPFCCR|=(1<<5);         //启动加载
}

//交换指定地址颜色的RGB顺序并且翻转alpha值并将新得到的颜色值复制到目标地址中
//pColorSrc:源地址
//pColorDst:目的地址
//NumItems:要处理的数量
static void InvertAlpha_SwapRB(LCD_COLOR * pColorSrc, LCD_COLOR * pColorDst, uint32_t NumItems) 
{
    uint32_t Color;
    do 
    {
        Color=*pColorSrc++;
        *pColorDst++=((Color&0x000000FF)<<16)           //交换红色(R)和蓝色(B)的位置
                    |(Color&0x0000FF00)                 //绿色(G)的位置不变
                    |((Color&0x00FF0000)>>16)           //交换红色(R)和蓝色(B)的位置
                    |((Color&0xFF000000)^0xFF000000);   //翻转alpha值
    } while (--NumItems);
}


//反转alpha，因为DMA2D的颜色格式和STemWin的颜色格式不同，因此需要翻转alpha值使其适配STemWin
//pColorSrc:源地址
//pColorDst:目的地址
//NumItems:要处理的数量
static void InvertAlpha(LCD_COLOR * pColorSrc, LCD_COLOR * pColorDst, uint32_t NumItems) 
{
    uint32_t Color;
    do 
    {
        Color = *pColorSrc++;
        *pColorDst++=Color^0xFF000000;//翻转alpha
    } while (--NumItems);
}

//alpha混色，使用DMA2D
//pColorFG:前景层存储器地址 
//pColorBG:背景层存储器地址
//pColorDst:输出存储器地址
//NumItems:行数
static void DMA2D_AlphaBlendingBulk(LCD_COLOR *pColorFG,LCD_COLOR *pColorBG,LCD_COLOR *pColorDst,U32 NumItems) 
{
    u32 timeout=0;
    DMA2D->CR=0x00020000UL|(1<<9);              //存储器到存储器并执行混合
    DMA2D->FGMAR=(uint32_t)pColorFG;            //设置前景层存储器地址
    DMA2D->BGMAR=(uint32_t)pColorBG;            //设置背景层存储器地址
    DMA2D->OMAR=(uint32_t)pColorDst;            //设置输出存储器地址
    DMA2D->FGOR=0;                              //设置前景层偏移
    DMA2D->BGOR=0;                              //设置背景层偏移
    DMA2D->OOR=0;                               //设置输出偏移
    DMA2D->FGPFCCR=LTDC_PIXEL_FORMAT_ARGB8888;  //设置前景层颜色格式
    DMA2D->BGPFCCR=LTDC_PIXEL_FORMAT_ARGB8888;  //设置背景层颜色格式
    DMA2D->OPFCCR =LTDC_PIXEL_FORMAT_ARGB8888;  //设置输出颜色格式
    DMA2D->NLR=(uint32_t)(NumItems<<16)|1;      //设置行数 
    DMA2D->CR|=DMA2D_CR_START;                  //启动DMA2D传输
    while(DMA2D->CR&DMA2D_CR_START)             //等待传输完成 
    {
        timeout++;
		if(timeout>0X1FFFFF)break;	            //超时退出
    }
}

//混合两种颜色,混合前景色和背景色
//Color:前景色
//BkColor:背景色
//Intens:混合强度
//返回值:混合后的颜色
static LCD_COLOR DMA2D_MixColors(LCD_COLOR Color,LCD_COLOR BkColor,U8 Intens) 
{
    uint32_t ColorFG,ColorBG,ColorDst;
    u32 timeout=0;
    if((BkColor&0xFF000000)==0xFF000000)return Color;//背景色透明，不需要混色
    ColorFG=Color^0xFF000000;
    ColorBG=BkColor^0xFF000000;
  
    DMA2D->CR=0x00020000UL|(1<<9);              //存储器到存储器并执行混合  
    DMA2D->FGMAR=(uint32_t)&ColorFG;            //设置前景层存储器地址
    DMA2D->BGMAR=(uint32_t)&ColorBG;            //设置背景层存储器地址
    DMA2D->OMAR=(uint32_t)&ColorDst;            //设置输出存储器地址
    DMA2D->FGPFCCR=LTDC_PIXEL_FORMAT_ARGB8888|(1UL<<16)|((uint32_t)Intens<<24);         //设置前景层颜色格式
    DMA2D->BGPFCCR=LTDC_PIXEL_FORMAT_ARGB8888|(0UL<<16)|((uint32_t)(255-Intens)<<24);   //设置背景层颜色格式
    DMA2D->OPFCCR=LTDC_PIXEL_FORMAT_ARGB8888;   //设置背景层颜色格式
    DMA2D->NLR=(uint32_t)(1<<16)|1;             //设置每行像素数和行数 
    DMA2D->CR|=DMA2D_CR_START;                  //启动DMA2D传输
    while(DMA2D->CR&DMA2D_CR_START)             //等待传输完成 
    {
        timeout++;
		if(timeout>0X1FFFFF)break;	            //超时退出
    }
    return (ColorDst^0xFF000000);               //返回混合后的颜色
}

//使用DMA2D进行颜色转换
//pSrc:源颜色(前景色)
//pDst:转换后的颜色(输出颜色)
//PixelFormatSrc:前景层颜色格式
//PixelFormatDst:输出颜色格式
//NumItems:每行像素数
static void DMA2D_ConvertColor(void * pSrc,void * pDst,uint32_t PixelFormatSrc,uint32_t PixelFormatDst,uint32_t NumItems)
{
    u32 timeout=0;
    DMA2D->CR=0x00010000UL|(1<<9);              //存储器到存储器并执行FPC
    DMA2D->FGMAR=(uint32_t)pSrc;                //前景层存储器地址
    DMA2D->OMAR=(uint32_t)pDst;                 //输出存储器地址
    DMA2D->FGOR=0;                              //背景层偏移
    DMA2D->OOR=0;                               //输出偏移
    DMA2D->FGPFCCR=PixelFormatSrc;              //前景层颜色格式
    DMA2D->OPFCCR=PixelFormatDst;               //输出颜色格式
    DMA2D->NLR=(uint32_t)(NumItems<<16)|1;      //设置每行像素数和行数
    DMA2D->CR|=DMA2D_CR_START;                  //启动DMA2D传输
    while(DMA2D->CR&DMA2D_CR_START)             //等待传输完成 
    {
        timeout++;
		if(timeout>0X1FFFFF)break;	            //超时退出
    }
}

static LCD_PIXELINDEX * _LCD_GetpPalConvTable(const LCD_LOGPALETTE GUI_UNI_PTR * pLogPal,const GUI_BITMAP GUI_UNI_PTR * pBitmap,int LayerIndex)
{
    void (* pFunc)(void);
    int32_t DoDefault = 0;

    if (pBitmap->BitsPerPixel == 8) 
    {
        pFunc=LCD_GetDevFunc(LayerIndex, LCD_DEVFUNC_DRAWBMP_8BPP);
        if(pFunc) 
        {
            if(pBitmap->pPal) 
            {
                if(pBitmap->pPal->HasTrans) DoDefault = 1;
            }else DoDefault = 1;  
        }else DoDefault = 1;  
    } 
    else DoDefault = 1;
    if (DoDefault) return LCD_GetpPalConvTable(pLogPal);
    InvertAlpha_SwapRB((U32 *)pLogPal->pPalEntries, aBufferDMA2D, pLogPal->NumEntries);
    DMA2D_LoadLUT(aBufferDMA2D, pLogPal->NumEntries);
    return aBufferDMA2D;
}

//使用DMA2D进行颜色混合
//pColorFG:前景色存储器地址
//pColorBG:背景色存储器地址
//pColorDst:输出存储器地址
//Intens:混合强度
//NumItems:每行像素数
static void DMA2D_MixColorsBulk(LCD_COLOR * pColorFG, LCD_COLOR * pColorBG, LCD_COLOR * pColorDst, U8 Intens, uint32_t NumItems)
{
    u32 timeout=0;
    DMA2D->CR=0x00020000UL|(1<<9);              //存储器到存储器并执行混合
    DMA2D->FGMAR=(uint32_t)pColorFG;            //设置前景色存储器地址
    DMA2D->BGMAR=(uint32_t)pColorBG;            //设置背景色存储器地址
    DMA2D->OMAR=(uint32_t)pColorDst;            //设置输出存储器地址
    DMA2D->FGPFCCR=LTDC_PIXEL_FORMAT_ARGB8888   //设置前景色颜色格式
                    |(1UL<<16)
                    |((uint32_t)Intens<<24);
    DMA2D->BGPFCCR=LTDC_PIXEL_FORMAT_ARGB8888   //设置背景色颜色格式
                    |(0UL<<16)
                    |((uint32_t)(255-Intens)<<24);    
    DMA2D->OPFCCR=LTDC_PIXEL_FORMAT_ARGB8888;   //设置输出颜色格式
    DMA2D->NLR=(uint32_t)(NumItems<<16)|1;      //设置每行像素数和行数    
    DMA2D->CR|=DMA2D_CR_START;                  //启动DMA2D传输
    while(DMA2D->CR&DMA2D_CR_START)             //等待传输完成 
    {
        timeout++;
		if(timeout>0X1FFFFF)break;	            //超时退出
    }
}

//使用DMA2D进行alpha混色
//pColorFG:前景色
//pColorBG:背景色
//pColorDst:输出颜色
//NumItems:每行像素数
static void DMA2D_AlphaBlending(LCD_COLOR *pColorFG,LCD_COLOR *pColorBG,LCD_COLOR *pColorDst,U32 NumItems)
{
    InvertAlpha(pColorFG,aBuffer_FG,NumItems);
    InvertAlpha(pColorBG,aBuffer_BG,NumItems);
    DMA2D_AlphaBlendingBulk(aBuffer_FG,aBuffer_BG,aBufferDMA2D,NumItems);
    InvertAlpha(aBufferDMA2D,pColorDst,NumItems);
}

//颜色索引转换为颜色值
static void DMA2D_Index2ColorBulk(void *pIndex,LCD_COLOR *pColor,uint32_t NumItems, U8 SizeOfIndex,uint32_t PixelFormat)
{
    DMA2D_ConvertColor(pIndex,aBufferDMA2D,PixelFormat,LTDC_PIXEL_FORMAT_ARGB8888,NumItems);
    InvertAlpha_SwapRB(aBufferDMA2D,pColor,NumItems);
}

//颜色值转换为颜色索引
static void DMA2D_Color2IndexBulk(LCD_COLOR *pColor,void *pIndex,uint32_t NumItems,U8 SizeOfIndex,uint32_t PixelFormat) 
{
    InvertAlpha_SwapRB(pColor,aBufferDMA2D,NumItems);
    DMA2D_ConvertColor(aBufferDMA2D,pIndex,LTDC_PIXEL_FORMAT_ARGB8888,PixelFormat,NumItems);
}

static void LCD_MixColorsBulk(U32 *pFG,U32 *pBG,U32 *pDst,unsigned OffFG,unsigned OffBG,unsigned OffDest,unsigned xSize,unsigned ySize,U8 Intens)
{
    int32_t y;

    GUI_USE_PARA(OffFG);
    GUI_USE_PARA(OffDest);
    for(y=0;y<ySize;y++) 
    {
        InvertAlpha(pFG,aBuffer_FG,xSize);
        InvertAlpha(pBG,aBuffer_BG,xSize);
        DMA2D_MixColorsBulk(aBuffer_FG,aBuffer_BG,aBufferDMA2D,Intens,xSize);
        InvertAlpha(aBufferDMA2D,pDst,xSize);
        pFG+=xSize+OffFG;
        pBG+=xSize+OffBG;
        pDst+=xSize+OffDest;
    }
}

//使用DMA2D绘制L8颜色格式的位图
static void DMA2D_DrawBitmapL8(void * pSrc, void * pDst,  uint32_t OffSrc, uint32_t OffDst, uint32_t PixelFormatDst, uint32_t xSize, uint32_t ySize)
{
    u32 timeout=0;
    DMA2D->CR=0x00010000UL|(1<<9);    
    DMA2D->FGMAR=(uint32_t)pSrc;                    
    DMA2D->OMAR=(uint32_t)pDst;                       
    DMA2D->FGOR=OffSrc;                      
    DMA2D->OOR=OffDst;                          
    DMA2D->FGPFCCR=LTDC_PIXEL_FORMAT_L8;             
    DMA2D->OPFCCR=PixelFormatDst;           
    DMA2D->NLR=(uint32_t)(xSize<<16)|ySize;      
    DMA2D->CR|=DMA2D_CR_START;                  //启动DMA2D传输
    while(DMA2D->CR&DMA2D_CR_START)             //等待传输完成 
    {
        timeout++;
		if(timeout>0X1FFFFF)break;	            //超时退出
    }
}

//绘制16bpp位图
static void LCD_DrawBitmap16bpp(int32_t LayerIndex,int32_t x,int32_t y,U16 const * p,int32_t xSize,int32_t ySize,int32_t BytesPerLine)
{
    uint32_t BufferSize, AddrDst;
    int32_t OffLineSrc, OffLineDst;

    BufferSize=GetBufferSize(LayerIndex);
    AddrDst=layer_prop[LayerIndex].address+BufferSize*layer_prop[LayerIndex].buffer_index+(y*layer_prop[LayerIndex].xSize+x)*layer_prop[LayerIndex].BytesPerPixel;
    OffLineSrc=(BytesPerLine/2)-xSize;
    OffLineDst=layer_prop[LayerIndex].xSize-xSize;
    DMA2D_CopyBuffer(LayerIndex,(void *)p,(void *)AddrDst,xSize,ySize,OffLineSrc,OffLineDst);
}

//绘制8bpp位图
static void LCD_DrawBitmap8bpp(int32_t LayerIndex, int32_t x, int32_t y, U8 const * p, int32_t xSize, int32_t ySize, int32_t BytesPerLine)
{ 
    uint32_t BufferSize, AddrDst;
    int32_t OffLineSrc, OffLineDst;
    uint32_t PixelFormat;

    BufferSize=GetBufferSize(LayerIndex);
    AddrDst=layer_prop[LayerIndex].address+BufferSize*layer_prop[LayerIndex].buffer_index+(y*layer_prop[LayerIndex].xSize + x)*layer_prop[LayerIndex].BytesPerPixel;
    OffLineSrc=BytesPerLine-xSize;
    OffLineDst=layer_prop[LayerIndex].xSize-xSize;
    PixelFormat=LCD_LL_GetPixelformat(LayerIndex);
    DMA2D_DrawBitmapL8((void *)p,(void *)AddrDst,OffLineSrc,OffLineDst,PixelFormat,xSize,ySize);
}
