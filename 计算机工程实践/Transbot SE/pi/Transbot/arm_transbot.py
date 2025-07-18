#!/usr/bin/env python3
# coding=utf-8
# V3.2.1
class Transbot_ARM:
    # 初始化函数
    def __init__(self, debug=False):
        # 修改以下三个数据，修改机械臂的偏移量，修正夹子夹不紧等问题。
        # 举个例子：如果夹子设置180后，夹爪依然留有空隙（夹不紧），那么需要增大self.__offset_9的值，如设置成10或者15，这样调试得出最优结果。
        # 如果夹子设置160就已经夹紧，那么可以修改self.__offset_9的值为-20。
        # Modify the following three data, modify the offset of the mechanical arm, and correct the problems such as the clamp is not tight.  
        # For example:If the clamp is set to 180 and still has a gap, then increase self.__offset_9 to 10 or 15 for optimal debugging.  
        # Change self.__offset_9 to -20 if the clip is set to 160 and already clamped.  
        self.__offset_7 = 0
        self.__offset_8 = 0
        self.__offset_9 = 0
        
        self.__arm_offset = (int(self.__offset_7), int(self.__offset_8), int(self.__offset_9))
        if debug:
            print("arm_offset:", self.__arm_offset)
        
    # 获取机械臂偏移量 
    # Obtain the offset of the manipulator
    def get_arm_offset(self):
        return self.__arm_offset

if __name__ == '__main__':
    bot_arm = Transbot_ARM(debug=False)
    offset = bot_arm.get_arm_offset()
    print("offset:", offset)
    del bot_arm
    print('arm_transbot.py killed')

