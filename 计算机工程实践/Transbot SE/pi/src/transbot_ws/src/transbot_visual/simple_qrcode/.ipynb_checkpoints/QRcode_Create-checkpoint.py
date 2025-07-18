#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import qrcode
from PIL import Image
from pathlib import Path
import matplotlib.pyplot as plt


def add_logo(img, logo_path):
    # 添加logo，打开logo照片
    # Add logo, open logo photo
    icon = Image.open(logo_path)
    # 获取图片的宽高
    # Get the width and height of the image
    img_w, img_h = img.size
    # 参数设置logo的大小
    # Set the size of the logo
    factor = 6
    size_w = int(img_w / factor)
    size_h = int(img_h / factor)
    icon_w, icon_h = icon.size
    if icon_w > size_w: icon_w = size_w
    if icon_h > size_h: icon_h = size_h
    # 重新设置logo的尺寸
    # Resize the logo
    icon = icon.resize((icon_w, icon_h), Image.ANTIALIAS)
    # 得到画图的x，y坐标，居中显示
    # Get the x and y coordinates of the drawing, and center them
    w = int((img_w - icon_w) / 2)
    h = int((img_h - icon_h) / 2)
    # 黏贴logo照
    # Paste the logo photo
    img.paste(icon, (w, h), mask=None)
    return img


def Create_QRcode(data, file_name, logo_path):
    '''
    参数含义：
    version：值为1~40的整数，控制二维码的大小（最小值是1，是个12×12的矩阵）。
             如果想让程序自动确定，将值设置为 None 并使用 fit 参数即可。
    error_correction：控制二维码的错误纠正功能。可取值下列4个常量。
    　　ERROR_CORRECT_L：大约7%或更少的错误能被纠正。
    　　ERROR_CORRECT_M（默认）：大约15%或更少的错误能被纠正。
    　　ROR_CORRECT_H：大约30%或更少的错误能被纠正。
    box_size：控制二维码中每个小格子包含的像素数。
    border：控制边框（二维码与图片边界的距离）包含的格子数（默认为4，是相关标准规定的最小值）
    Parameter meaning:
     version: An integer ranging from 1 to 40, which controls the size of the QR code (the minimum value is 1, which is a 12×12 matrix).
              If you want the program to determine it automatically, set the value to None and use the fit parameter.
     error_correction: Control the error correction function of the QR code. The following 4 constants can be used.
     　　ERROR_CORRECT_L: About 7% or less of errors can be corrected.
     　　ERROR_CORRECT_M (default): About 15% or less of errors can be corrected.
     　　 ROR_CORRECT_H: About 30% or less errors can be corrected.
     box_size: Control the number of pixels contained in each small grid in the QR code.
     border: Control the number of grids contained in the border (the distance between the QR code and the picture border) (the default is 4, which is the minimum value specified by the relevant standards)
    '''
    my_file = Path(logo_path)
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=5,
        border=4,)
    # 添加数据
    # add data
    qr.add_data(data)
    # 填充数据
    # fill data
    qr.make(fit=True)
    # 生成图片
    # generate images
    img = qr.make_image(fill_color="green", back_color="white")
    if my_file.is_file(): img = add_logo(img, logo_path)
    # 保存img
    # save img
    img.save(file_name)
    # 终端显示图片
    # Terminal display picture
    plt.imshow(img)
    plt.show()
    return img


if __name__ == '__main__':
    '''
    注意：使用中文时，需加中文字符
    例如：亚博智能科技有限公司！
    Note: Chinese characters must be added when using Chinese characters
    For example: Primus Technologies LTD!
    '''
    file_path = os.getcwd()
    # logo_path = ''
    logo_path = file_path+"/yahboom.jpg"
    out_img = file_path+'/myQRcode.jpg'
    input = input("Please enter:  ")
    Create_QRcode(input, out_img, logo_path)
