#!/usr/bin/env python3
# encoding: utf-8
import usb


def GetDevice():
    idVendors = []
    for bus in usb.busses():
        devices = bus.devices
        for dev in devices: idVendors.append(dev.idVendor)
    if 0x2bc5 in idVendors:
        return "Astra"
    else:
        return "USBCam"
