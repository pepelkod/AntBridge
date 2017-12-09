/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except
in compliance with this license.

Copyright (c) Dynastream Innovations Inc. 2016
All rights reserved.
*/
#ifndef USB_DEVICE_IOKIT_HPP
#define USB_DEVICE_IOKIT_HPP

#include "types.h"
#include "usb_device.hpp"

#include "iokit_device.hpp"


//////////////////////////////////////////////////////////////////////////////////
// Public Definitions
//////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////
// Public Class Prototypes
//////////////////////////////////////////////////////////////////////////////////


class USBDeviceIOKit : public USBDevice
{

  public:

   USBDeviceIOKit(const IOKitDevice& clIOKitDevice_);
   USBDeviceIOKit(const USBDeviceIOKit& clDevice_);

   USBDeviceIOKit& operator=(const USBDeviceIOKit& clIOKitDevice_);

   const IOKitDevice& GetIOKitDevice() const { return clDevice; }

   //std::auto_ptr<USBDevice> MakeCopy() const { return auto_ptr<USBDevice>(new USBDeviceSI(*this)); }  //!!

   //Implementation of Device Interface

   BOOL USBReset() const;
   USHORT GetVid() const { return usVid; }
   USHORT GetPid() const { return usPid; }
   ULONG GetSerialNumber() const { return ulSerialNumber; }
   BOOL GetProductDescription(UCHAR* pucProductDescription_, USHORT usBufferSize_) const; //guaranteed to be null-terminated
   BOOL GetSerialString(UCHAR* pucSerialString_, USHORT usBufferSize_) const;
   BOOL UpdateSerialString(UCHAR* pucSerialString_, USHORT usLength_);

   DeviceType::Enum GetDeviceType() const;


  private:

   IOKitDevice clDevice;
   USHORT usVid;
   USHORT usPid;
   ULONG ulSerialNumber;

   UCHAR szProductDescription[USB_MAX_STRLEN];
   UCHAR szSerialString[USB_MAX_STRLEN];

};


#endif // !defined(USB_DEVICE_VCP_HPP)

