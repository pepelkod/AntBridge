/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except
in compliance with this license.

Copyright (c) Dynastream Innovations Inc. 2016
All rights reserved.
*/

#include "types.h"
#if defined(DSI_TYPES_LINUX)

#include "usb_device_libusb_linux.hpp"

#include "macros.h"

#include "dsi_libusb_library_linux.hpp"

#include <memory>

#include <string.h>

#include <stdlib.h>

using namespace std;


USBDeviceLibusb::USBDeviceLibusb(struct libusb_device_node& stDevice_)
:
   pstDevice(stDevice_.device),
   usVid(stDevice_.descriptor.idVendor),
   usPid(stDevice_.descriptor.idProduct),
   ulSerialNumber(0)
{

   szProductDescription[0] = '\0';
   szSerialString[0] = '\0';

   //Get a reference to library
   auto_ptr<const LibusbLibrary> pclAutoLibusbLibrary(NULL);
   if(LibusbLibrary::Load(pclAutoLibusbLibrary) == FALSE)
      return;
   const LibusbLibrary& clLibusbLibrary = *pclAutoLibusbLibrary;

   libusb_device_handle *pstTempDeviceHandle;
   int ret = clLibusbLibrary.Open(stDevice_.device, &pstTempDeviceHandle);  //We can open the device to get the info even if someone else is using it, so this is okay.
   if(ret < 0)
      return;

   ret = clLibusbLibrary.GetStringDescriptorAscii(pstTempDeviceHandle, stDevice_.descriptor.iProduct, szProductDescription, sizeof(szProductDescription));
   if(ret < 0)
   {
      szProductDescription[0] = '\0';
   }

   ret = clLibusbLibrary.GetStringDescriptorAscii(pstTempDeviceHandle, stDevice_.descriptor.iSerialNumber, szSerialString, sizeof(szSerialString));
   if(ret < 0)
   {
      szSerialString[0] = '\0';
      ulSerialNumber = 0;
   }
   else
   {
      USBDeviceLibusb::GetDeviceSerialNumber(ulSerialNumber);
   }

   clLibusbLibrary.Close(pstTempDeviceHandle);

   return;
}

USBDeviceLibusb::USBDeviceLibusb(const USBDeviceLibusb& clDevice_)
:
   pstDevice(clDevice_.pstDevice),
   usVid(clDevice_.usVid),
   usPid(clDevice_.usPid),
   ulSerialNumber(clDevice_.ulSerialNumber)
{
   STRNCPY((char*)szProductDescription, (char*)clDevice_.szProductDescription, sizeof(szProductDescription));
   memcpy(szSerialString, clDevice_.szSerialString, sizeof(szSerialString));
   return;
}


USBDeviceLibusb& USBDeviceLibusb::operator=(const USBDeviceLibusb& clDevice_)
{
   if(this == &clDevice_)
      return *this;

   pstDevice = clDevice_.pstDevice;
   usVid = clDevice_.usVid;
   usPid = clDevice_.usPid;
   ulSerialNumber = clDevice_.ulSerialNumber;
   STRNCPY((char*)szProductDescription, (char*)clDevice_.szProductDescription, sizeof(szProductDescription));
   memcpy(szSerialString, clDevice_.szSerialString, sizeof(szSerialString));

   return *this;
}

BOOL USBDeviceLibusb::USBReset() const  //!!make static?
{
   return TRUE;
}

BOOL USBDeviceLibusb::GetProductDescription(UCHAR* pucProductDescription_, USHORT usBufferSize_) const
{
   return(STRNCPY((char*) pucProductDescription_, (char*) szProductDescription, usBufferSize_));
}

BOOL USBDeviceLibusb::GetSerialString(UCHAR* pucSerialString_, USHORT usBufferSize_) const
{
   if(sizeof(szSerialString) > usBufferSize_)
   {
      memcpy(pucSerialString_, szSerialString, usBufferSize_);
      return FALSE;
   }

   memcpy(pucSerialString_, szSerialString, sizeof(szSerialString));
   return TRUE;
}

//The serial number actually is not limited to a ULONG by USB specs,
//so, our range here is determined by whatever we do in our products.
//For now we have it defined as 1 to (ULONG_MAX-1)
BOOL USBDeviceLibusb::GetDeviceSerialNumber(ULONG& ulSerialNumber_)
{
   ULONG ulSerial = strtoul((char*)szSerialString, NULL, 10);
   if(ulSerial == 0 || ulSerial == ULONG_MAX)
      return FALSE;

   ulSerialNumber_ = ulSerial;
   return TRUE;
}

#endif //defined(DSI_TYPES_LINUX)
