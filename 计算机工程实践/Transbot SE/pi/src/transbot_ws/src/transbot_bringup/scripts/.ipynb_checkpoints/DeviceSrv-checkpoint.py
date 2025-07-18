#!/usr/bin/env python
# encoding: utf-8
import usb
import rospy
from transbot_msgs.srv import *


class Device:
    def __init__(self):
        self.srv_CamDevice = rospy.Service("/CamDevice", CamDevice, self.CamDevicecallback)

    def CamDevicecallback(self, request):
        if not isinstance(request, CamDeviceRequest): return
        response = CamDeviceResponse()
        response.camDevice = self.GetDevice()
        return response

    def GetDevice(self):
        idVendors = []
        for bus in usb.busses():
            devices = bus.devices
            for dev in devices: idVendors.append(dev.idVendor)
        if 0x2bc5 in idVendors: return "Astra"
        else: return "USBCam"

    def cancel(self): self.srv_CamDevice.shutdown()


if __name__ == '__main__':
    rospy.init_node('DeviceSrv', anonymous=False)
    device = Device()
    get_device = device.GetDevice()
    print ("camera_device: ",get_device)
    rospy.spin()
