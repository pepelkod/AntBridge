/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except
in compliance with this license.

Copyright (c) Dynastream Innovations Inc. 2016
All rights reserved.
*/
#ifndef USB_DEVICE_VCP_HPP
#define USB_DEVICE_VCP_HPP

#include "types.h"
#include "usb_device.hpp"
#include "iokit_types.hpp"

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>

//////////////////////////////////////////////////////////////////////////////////
// Public Definitions
//////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////
// Public Class Prototypes
//////////////////////////////////////////////////////////////////////////////////


class USBDeviceSI : public USBDevice
{

  public:

   USBDeviceSI(const io_service_t& hService_);  //hService_ should be of IOUSBDevice class
   USBDeviceSI(UCHAR* pucBsdName_);
   USBDeviceSI(const USBDeviceSI& clDevice_);
   USBDeviceSI& operator=(const USBDeviceSI& clDevice_);

   BOOL USBReset() const;

   const UCHAR* GetBsdName() const { return szBsdName; }

   //std::auto_ptr<USBDevice> MakeCopy() const { return auto_ptr<USBDevice>(new USBDeviceSI(*this)); }  //!!

   //Implementation of Device Interface

   USHORT GetVid() const { return usVid; }
   USHORT GetPid() const { return usPid; }
   ULONG GetSerialNumber() const { return ulSerialNumber; }

   ULONG GetLocation() const { return ulLocation; }

   BOOL GetProductDescription(UCHAR* pucProductDescription_, USHORT usBufferSize_) const; //guaranteed to be null-terminated
   BOOL GetSerialString(UCHAR* pucSerialString_, USHORT usBufferSize_) const;
   usb_device_t** GetDeviceInterface() const { return ppstDeviceInterface; }

   DeviceType::Enum GetDeviceType() const { return DeviceType::SI_LABS; }


  private:

   static ULONG GetDeviceNumber(const io_service_t& hService_, CFStringRef hProperty_);
   static BOOL GetDeviceString(const io_service_t& hService_, CFStringRef hProperty_, UCHAR* pucBsdName_, ULONG ulSize_, BOOL bSearchChildren_ = FALSE);
   static ULONG GetSerialNumber(const io_service_t& hService_, UCHAR* pucSerialString_, ULONG ulSize_);
   static usb_device_t** CreateDeviceInterface(const io_service_t& hService_);


   USHORT usVid;
   USHORT usPid;
   ULONG ulSerialNumber;

   UCHAR szBsdName[255];
   UCHAR szProductDescription[USB_MAX_STRLEN];
   UCHAR szSerialString[USB_MAX_STRLEN];

   ULONG ulLocation;  //GUID for USB device

   usb_device_t** ppstDeviceInterface;

};


#endif // !defined(USB_DEVICE_VCP_HPP)

