/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except
in compliance with this license.

Copyright (c) Dynastream Innovations Inc. 2016
All rights reserved.
*/

#include "types.h"

#if defined(DSI_TYPES_LINUX)
#include "dsi_libusb_library_linux.hpp"

#include <memory>

using namespace std;

//Static variable initializations
std::auto_ptr<LibusbLibrary> LibusbLibrary::clAutoInstance(NULL);
BOOL LibusbLibrary::bStaticSet = FALSE;

                                                                     //return an auto_ptr?
BOOL LibusbLibrary::Load(auto_ptr<const LibusbLibrary>& clAutoLibrary_)  //!!Should we lose the dependency on auto_ptr and just return the pointer (and let the user make their own auto_ptr)?
{
   try
   {
      clAutoLibrary_.reset(new LibusbLibrary());
   }
   catch(...)
   {
      clAutoLibrary_.reset(NULL);
      return FALSE;
   }

   return TRUE;
}


LibusbLibrary::LibusbLibrary() //throw(LibusbError::Enum&)
:
#if defined(DSI_TYPES_LINUX)
   Init(NULL),
   Exit(NULL),
   SetDebug(NULL),
   Open(NULL),
   Close(NULL),
   ResetDevice(NULL),
   GetDeviceDescriptor(NULL),
   GetStringDescriptorAscii(NULL),
   GetDevice(NULL),
   GetDeviceList(NULL),
   FreeDeviceList(NULL),
   ClaimInterface(NULL),
   ReleaseInterface(NULL),
   ReferenceDevice(NULL),
   UnreferenceDevice(NULL),
   AllocTransfer(NULL),
   SubmitTransfer(NULL),
   CancelTransfer(NULL),
   FreeTransfer(NULL),
   FillBulkTransfer(NULL),
   DetachKernelDriver(NULL),
   AttachKernelDriver(NULL),
   KernelDriverActive(NULL),
   HandleEventsTimeoutCompleted(NULL)
#endif

{
   //Check static instance
   if(bStaticSet == FALSE)
   {
      bStaticSet = TRUE;
      try
      {
         clAutoInstance.reset(new LibusbLibrary());
      }
      catch(...)
      {
         bStaticSet = FALSE;
         throw;
      }
   }

   //load library
   LibusbError::Enum ret = LoadFunctions();
   if(ret != LibusbError::NONE)
      throw(ret);

   return;
}

LibusbLibrary::~LibusbLibrary() throw()
{
   FreeFunctions();
}


///////////////////////////////////////////////////////////////////////
// Loads USB interface functions from the DLLs.
///////////////////////////////////////////////////////////////////////
LibusbError::Enum LibusbLibrary::LoadFunctions()
{

   BOOL bStatus = TRUE;

#if defined(DSI_TYPES_LINUX)
   //libusb-1.0 functions

   Init = (Init_t)&libusb_init;
   if(Init == NULL)
      bStatus = FALSE;

   Exit = (Exit_t)&libusb_exit;
   if(Exit == NULL)
      bStatus = FALSE;

   SetDebug = (SetDebug_t)&libusb_set_debug;
   if(SetDebug == NULL)
      bStatus = FALSE;

   Open = (Open_t)&libusb_open;
   if(Open == NULL)
      bStatus = FALSE;

   Close = (Close_t)&libusb_close;
   if(Close == NULL)
      bStatus = FALSE;

   ResetDevice = (ResetDevice_t)&libusb_reset_device;
   if(ResetDevice == NULL)
      bStatus = FALSE;

   GetDeviceDescriptor = (GetDeviceDescriptor_t)&libusb_get_device_descriptor;
   if(GetDeviceDescriptor == NULL)
      bStatus = FALSE;

   GetStringDescriptorAscii = (GetStringDescriptorAscii_t)&libusb_get_string_descriptor_ascii;
   if(GetStringDescriptorAscii == NULL)
      bStatus = FALSE;

   GetDevice = (GetDevice_t)&libusb_get_device;
    if(GetDevice == NULL)
      bStatus = FALSE;

   GetDeviceList = (GetDeviceList_t)&libusb_get_device_list;
   if(GetDeviceList == NULL)
      bStatus = FALSE;

   FreeDeviceList = (FreeDeviceList_t)&libusb_free_device_list;
   if(FreeDeviceList == NULL)
      bStatus = FALSE;

   ClaimInterface = (ClaimInterface_t)&libusb_claim_interface;
   if(ClaimInterface == NULL)
      bStatus = FALSE;

   ReleaseInterface = (ReleaseInterface_t)&libusb_release_interface;
   if(ReleaseInterface == NULL)
      bStatus = FALSE;

   ReferenceDevice = (ReferenceDevice_t)&libusb_ref_device;
   if(ReferenceDevice == NULL)
      bStatus = FALSE;

   UnreferenceDevice = (UnreferenceDevice_t)&libusb_ref_device;
   if(UnreferenceDevice == NULL)
      bStatus = FALSE;

   AllocTransfer = (AllocTransfer_t)&libusb_alloc_transfer;
   if(AllocTransfer == NULL)
      bStatus = FALSE;

   SubmitTransfer = (SubmitTransfer_t)&libusb_submit_transfer;
   if(SubmitTransfer == NULL)
      bStatus = FALSE;

   CancelTransfer = (CancelTransfer_t)&libusb_cancel_transfer;
   if(CancelTransfer == NULL)
      bStatus = FALSE;

   FreeTransfer = (FreeTransfer_t)&libusb_free_transfer;
   if(FreeTransfer == NULL)
      bStatus = FALSE;

   FillBulkTransfer = (FillBulkTransfer_t)&libusb_fill_bulk_transfer;
   if(FillBulkTransfer == NULL)
      bStatus = FALSE;

   DetachKernelDriver = (DetachKernelDriver_t)&libusb_detach_kernel_driver;
   if(DetachKernelDriver == NULL)
      bStatus = FALSE;

   AttachKernelDriver = (AttachKernelDriver_t)&libusb_attach_kernel_driver;
   if(AttachKernelDriver == NULL)
      bStatus = FALSE;

   KernelDriverActive = (KernelDriverActive_t)&libusb_kernel_driver_active;
   if(KernelDriverActive == NULL)
      bStatus = FALSE;

   HandleEventsTimeoutCompleted = (HandleEventsTimeoutCompleted_t)&libusb_handle_events_timeout_completed;
   if(HandleEventsTimeoutCompleted == NULL)
      bStatus = FALSE;
#endif

   if(bStatus == FALSE)
   {
      FreeFunctions();
      return LibusbError::NO_FUNCTION;
   }

   return LibusbError::NONE;
}

///////////////////////////////////////////////////////////////////////
// Unloads USB DLLs.
///////////////////////////////////////////////////////////////////////
void LibusbLibrary::FreeFunctions()
{
   return;
}

#endif //defined(DSI_TYPES_LINUX)
