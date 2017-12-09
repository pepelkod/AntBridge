/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except
in compliance with this license.

Copyright (c) Dynastream Innovations Inc. 2016
All rights reserved.
*/

#include "types.h"
#if defined(DSI_TYPES_LINUX)

#include "usb_device_handle_libusb_linux.hpp"

#include "macros.h"
#include "usb_device_list.hpp"
#include "dsi_debug.hpp"
#include "antmessage.h"

#include <libusb-1.0/libusb.h>

#include <memory>

using namespace std;

//////////////////////////////////////////////////////////////////////////////////
// Static declarations
//////////////////////////////////////////////////////////////////////////////////

USBDeviceList<const USBDeviceLibusb> USBDeviceHandleLibusb::clDeviceList;
libusb_context* USBDeviceHandleLibusb::ctx = 0;

//////////////////////////////////////////////////////////////////////////////////
// Private Definitions
//////////////////////////////////////////////////////////////////////////////////

const UCHAR USB_ANT_CONFIGURATION = 1;
const UCHAR USB_ANT_INTERFACE = 0;
const UCHAR USB_ANT_EP_IN  = 0x81;
const UCHAR USB_ANT_EP_OUT = 0x01;

BOOL LibusbDeviceMatch(const USBDeviceLibusb* const & pclDevice_)
{
   //!!Can also find device by it's description string?
   USHORT usVid = pclDevice_->GetVid();
   return (usVid == USBDeviceHandle::USB_ANT_VID || usVid == USBDeviceHandle::USB_ANT_VID_TWO);
}

BOOL CanOpenDevice(const USBDeviceLibusb*const & pclDevice_)  //!!Should we make a static member function that does a more efficient try open (doesn't start a receive thread, etc.)
{
   if(pclDevice_ == FALSE)
      return FALSE;

   return USBDeviceHandleLibusb::TryOpen(*pclDevice_);
}

const USBDeviceListLibusb USBDeviceHandleLibusb::GetAllDevices()  //!!List needs to be deleted!
{
   USBDeviceListLibusb clList;
   USBDeviceListLibusb clFilteredList;
   libusb_device **list;
   ssize_t count;
   int ret;

   clDeviceList = USBDeviceList<const USBDeviceLibusb>();  //clear device list

   //Get a reference to library
   auto_ptr<const LibusbLibrary> pclAutoLibusbLibrary(NULL);
   if(LibusbLibrary::Load(pclAutoLibusbLibrary) == FALSE)
      return clList;
   const LibusbLibrary& clLibusbLibrary = *pclAutoLibusbLibrary;

   //Initialize libusb
   if(ctx == NULL)
   {
      clLibusbLibrary.Init(&ctx);
   }

   #if defined(_DEBUG) && defined(DEBUG_FILE)
     clLibusbLibrary.SetDebug(ctx, 255);
   #endif

   count = clLibusbLibrary.GetDeviceList(ctx, &list);

   struct libusb_device_node *dev = new libusb_device_node;

   for(ssize_t index = 0; index < count; index++)
   {
      dev->device = list[index];
      clLibusbLibrary.ReferenceDevice(dev->device);
      ret = clLibusbLibrary.GetDeviceDescriptor(dev->device, &dev->descriptor);
      if(ret < 0){break;}
     clDeviceList.Add( USBDeviceLibusb(*dev) );
     clList.Add( clDeviceList.GetAddress(clDeviceList.GetSize()-1) );
   }

    delete dev;
   clLibusbLibrary.FreeDeviceList(list, 0); //Free the list, do not unref the devices

   //Filter list here so that the returned list only contains
   // Libusb ANT devices regardless of where GetAllDevices is called from
   clFilteredList.Add(clList.GetSubList(LibusbDeviceMatch));

   return clFilteredList;
}


//!!Will we have a problem if someone asks for a device list twice? (will it dereference the pointers in the first list?)
const USBDeviceListLibusb USBDeviceHandleLibusb::GetAvailableDevices()  //!!List needs to be deleted!
{
   return USBDeviceHandleLibusb::GetAllDevices().GetSubList(CanOpenDevice);
}


BOOL USBDeviceHandleLibusb::Open(const USBDeviceLibusb& clDevice_, USBDeviceHandleLibusb*& pclDeviceHandle_)
{
   try
   {
      pclDeviceHandle_ = new USBDeviceHandleLibusb(clDevice_);
   }
   catch(...)
   {
      pclDeviceHandle_ = NULL;
      return FALSE;
   }

    // Workaround for USB2/m firmware error mishandling USB 'clear feature' request (See Jira ANTPC-45)
    // Firmware data toggle syncronization and busy bit become out of sync, and can be fixed by
    // sending two ANT requests.
    // In the case that the USB pipe is not totally out of sync, and some responses are received,
    // we make sure we read all that data so the workaround is invisible to the app.
    UCHAR aucReqCapabilitiesMsg[MESG_FRAME_SIZE + 2] = {0xA4, 0x02, 0x4D, 0x00, 0x54, 0xBF};
    UCHAR aucCapabilitiesMsg[MESG_MAX_SIZE];
    ULONG ulBytesWritten, ulBytesRead = 0;

    pclDeviceHandle_->Write(aucReqCapabilitiesMsg, sizeof(aucReqCapabilitiesMsg), ulBytesWritten);
    pclDeviceHandle_->Write(aucReqCapabilitiesMsg, sizeof(aucReqCapabilitiesMsg), ulBytesWritten);
    pclDeviceHandle_->Read(aucCapabilitiesMsg, sizeof(aucCapabilitiesMsg), ulBytesRead, 10);
    pclDeviceHandle_->Read(aucCapabilitiesMsg, sizeof(aucCapabilitiesMsg), ulBytesRead, 10);

   return TRUE;
}



BOOL USBDeviceHandleLibusb::Close(USBDeviceHandleLibusb*& pclDeviceHandle_, BOOL bReset_)
{
   if(pclDeviceHandle_ == NULL)
      return FALSE;

   pclDeviceHandle_->PClose(bReset_);
   delete pclDeviceHandle_;
   pclDeviceHandle_ = NULL;

   return TRUE;
}

//A more efficient way to test if you can open a device.  For instance, this function won't create a receive loop, etc.)
BOOL USBDeviceHandleLibusb::TryOpen(const USBDeviceLibusb& clDevice_)
{
   //Get a reference to library
   auto_ptr<const LibusbLibrary> pclAutoLibusbLibrary(NULL);
   if(LibusbLibrary::Load(pclAutoLibusbLibrary) == FALSE)
      return FALSE;
   const LibusbLibrary& clLibusbLibrary = *pclAutoLibusbLibrary;

   if(ctx == NULL)
   {
      clLibusbLibrary.Init(&ctx);
   }

   int ret;
   BOOL bDetachedKernelDriver = FALSE;

   libusb_device_handle* pclTempDeviceHandle;
   ret = clLibusbLibrary.Open(&clDevice_.GetRawDevice(), &pclTempDeviceHandle);
   if(pclTempDeviceHandle == NULL)
      return FALSE;  //Doesn't need to call Close because it wouldn't do anything

   //Only try to detach ant devices
   if(clDevice_.GetVid() == USB_ANT_VID_TWO || clDevice_.GetVid() == USB_ANT_VID)
   {
      ret = clLibusbLibrary.KernelDriverActive(pclTempDeviceHandle, USB_ANT_INTERFACE);
      if(ret == 1)
      {
         ret = clLibusbLibrary.DetachKernelDriver(pclTempDeviceHandle, USB_ANT_INTERFACE);
         if(ret != 0)
         {
            clLibusbLibrary.Close(pclTempDeviceHandle);
            return FALSE;
         }
         bDetachedKernelDriver = TRUE;
      }
   }

   ret = clLibusbLibrary.ClaimInterface(pclTempDeviceHandle, USB_ANT_INTERFACE);
   if(ret != 0)
   {
       clLibusbLibrary.Close(pclTempDeviceHandle);

      return FALSE;
   }

   ret = clLibusbLibrary.ReleaseInterface(pclTempDeviceHandle, USB_ANT_INTERFACE);
   if(ret != 0) {}  //this would be an error

   if(bDetachedKernelDriver)
   {
      ret = clLibusbLibrary.AttachKernelDriver(pclTempDeviceHandle, USB_ANT_INTERFACE);
      if(ret != 0) {}  //this would be an error
   }

   clLibusbLibrary.Close(pclTempDeviceHandle);

   return TRUE;
}


///////////////////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////////////////

USBDeviceHandleLibusb::USBDeviceHandleLibusb(const USBDeviceLibusb& clDevice_)
try
:
   USBDeviceHandle(),
   clDevice(clDevice_), //!!Copy?
   bDeviceGone(TRUE)
{
   hReceiveThread = NULL;
   bStopReceiveThread = TRUE;
   device_handle = NULL;

   if(ctx == NULL)
   {
      clLibusbLibrary.Init(&ctx);
   }

   if(POpen() == FALSE)
      throw 0; //!!We need something to throw

   return;
}
catch(...)
{
   throw;
}

///////////////////////////////////////////////////////////////////////
// Destructor
///////////////////////////////////////////////////////////////////////
USBDeviceHandleLibusb::~USBDeviceHandleLibusb()
{
   //!!Delete all the elements in the clOverflowQueue!
}



///////////////////////////////////////////////////////////////////////
// Opens port, starts receive thread.
///////////////////////////////////////////////////////////////////////
BOOL USBDeviceHandleLibusb::POpen()
{
   int ret;

   //Make sure we are not open before opening again.
   PClose();  //!!Do we want to reset here?

   ret = clLibusbLibrary.Open(&clDevice.GetRawDevice(), &device_handle);
   if(ret < 0)
      return FALSE;  //Doesn't need to call Close because it wouldn't do anything

   bDeviceGone = FALSE;

   //Detatch kernel driver to ensure we can set the configuration
   ret = clLibusbLibrary.KernelDriverActive(device_handle, USB_ANT_INTERFACE);
   if(ret == 1)
   {
      ret = clLibusbLibrary.DetachKernelDriver(device_handle, USB_ANT_INTERFACE);
      if(ret != 0)
      {
         PClose();
         return FALSE;
      }
   }
   else if (ret < 0)
   {
      PClose();
      return FALSE;
   }

   ret = clLibusbLibrary.ClaimInterface(device_handle, USB_ANT_INTERFACE);
   if(ret != 0)
   {
      PClose();
      return FALSE;
   }

   if(DSIThread_MutexInit(&stMutexCriticalSection) != DSI_THREAD_ENONE)
   {
      PClose();
      return FALSE;
   }

   if(DSIThread_CondInit(&stEventReceiveThreadExit) != DSI_THREAD_ENONE)
   {
      DSIThread_MutexDestroy(&stMutexCriticalSection);
      PClose();
      return FALSE;
   }

   bStopReceiveThread = FALSE;
   hReceiveThread = DSIThread_CreateThread(&USBDeviceHandleLibusb::ProcessThread, this);
   if (hReceiveThread == NULL)
   {
      DSIThread_CondDestroy(&stEventReceiveThreadExit);
      DSIThread_MutexDestroy(&stMutexCriticalSection);
      PClose();
      return FALSE;
   }

   return TRUE;
}


///////////////////////////////////////////////////////////////////////
// Closes the USB connection, kills receive thread.
///////////////////////////////////////////////////////////////////////
void USBDeviceHandleLibusb::PClose(BOOL bReset_)
{

   bDeviceGone = TRUE;

   if (hReceiveThread)
   {
      DSIThread_MutexLock(&stMutexCriticalSection);
      if(bStopReceiveThread == FALSE)
      {
         bStopReceiveThread = TRUE;

         if (DSIThread_CondTimedWait(&stEventReceiveThreadExit, &stMutexCriticalSection, 3000) != DSI_THREAD_ENONE)
         {
            // We were unable to stop the thread normally.
            DSIThread_DestroyThread(hReceiveThread);
         }
      }
      DSIThread_MutexUnlock(&stMutexCriticalSection);

      DSIThread_ReleaseThreadID(hReceiveThread);
      hReceiveThread = NULL;

      DSIThread_MutexDestroy(&stMutexCriticalSection);
      DSIThread_CondDestroy(&stEventReceiveThreadExit);
   }

    if(device_handle != (libusb_device_handle*)NULL)
    {

      int ret;

      ret = clLibusbLibrary.ReleaseInterface(device_handle, USB_ANT_INTERFACE);
      if(ret != 0) {}  //this would be an error

      ret = clLibusbLibrary.AttachKernelDriver(device_handle, USB_ANT_INTERFACE);
      if(ret != 0) {}  //this would be an error

      if(bReset_)
      {
         //ret = clLibusbLibrary.ResetDevice(device_handle);  //The library reset function can sometimes cause the device/driver to go into an unusable state, long term stability testing shows there is no benifit to including reset for this device/driver combination.
         clLibusbLibrary.Close(device_handle);
         //if(ret != 0) {}  //this would be an error
      }
      else
      {
         clLibusbLibrary.Close(device_handle);
         //if(ret != 0) {}  //this would be an error
      }

      device_handle = (libusb_device_handle*)NULL;
   }

}

static void LIBUSB_CALL Callback(struct libusb_transfer *transfer)
{
    int *completed = (int*)transfer->user_data;
    *completed = 1;
}
///////////////////////////////////////////////////////////////////////
// Writes usSize_ bytes to USB, returns TRUE if successful.
///////////////////////////////////////////////////////////////////////
//!!Return true if we wrote half the bytes successfully?
USBError::Enum USBDeviceHandleLibusb::Write(void* pvData_, ULONG ulSize_, ULONG& ulBytesWritten_)
{
    #if defined(DEBUG_FILE) && defined(_DEBUG)
    BOOL bTxDebug = DSIDebug::ThreadInit("ao_libusb_transmit");
    DSIDebug::ThreadEnable(TRUE);
    #endif

    if(bDeviceGone)
        return USBError::DEVICE_GONE;

    if(pvData_ == NULL)
        return USBError::INVALID_PARAM;

    int iRet = 0;
    int iCompleted = 0;
    struct libusb_transfer *asyncTransfer;
    struct timeval tvHandleEventsTimeout;
    tvHandleEventsTimeout.tv_sec = 3;
    tvHandleEventsTimeout.tv_usec = 0;

    asyncTransfer = clLibusbLibrary.AllocTransfer(0);
    clLibusbLibrary.FillBulkTransfer(asyncTransfer, device_handle, USB_ANT_EP_OUT, (UCHAR*)pvData_, (int)ulSize_, Callback, &iCompleted, 3000);
    asyncTransfer->type = LIBUSB_TRANSFER_TYPE_BULK;
    iRet = clLibusbLibrary.SubmitTransfer(asyncTransfer);
    if(iRet < 0)
    {
        clLibusbLibrary.FreeTransfer(asyncTransfer);
        return USBError::FAILED;
    }

    while(!iCompleted)
        clLibusbLibrary.HandleEventsTimeoutCompleted(NULL, &tvHandleEventsTimeout, &iCompleted);

    if(asyncTransfer->status != LIBUSB_TRANSFER_COMPLETED)
    {
        return USBError::FAILED;
    }

    ulBytesWritten_ = asyncTransfer->actual_length;

    return USBError::NONE;
}

///////////////////////////////////////////////////////////////////////
USBError::Enum USBDeviceHandleLibusb::Read(void* pvData_, ULONG ulSize_, ULONG& ulBytesRead_, ULONG ulWaitTime_)
{
    if(bDeviceGone)
        return USBError::DEVICE_GONE;

    ulBytesRead_ = clRxQueue.PopArray(reinterpret_cast<UCHAR* const>(pvData_), ulSize_, ulWaitTime_);
    return USBError::NONE;
}

void USBDeviceHandleLibusb::ReceiveThread()
{
    #if defined(DEBUG_FILE)
    BOOL bRxDebug = DSIDebug::ThreadInit("ao_libusb_receive");
    DSIDebug::ThreadEnable(TRUE);
    #endif

    int iRet = 0;
    int iConsecIoErrors = 0;
    int iCompleted = 0;
    BOOL bSubmitTransfer = TRUE;
    BOOL bReallocTransfer = TRUE;
    UCHAR aucData[4096];
    struct libusb_transfer *asyncTransfer;
    struct timeval tvHandleEventsTimeout;
    tvHandleEventsTimeout.tv_sec = 1;
    tvHandleEventsTimeout.tv_usec = 0;

    while(!bStopReceiveThread)
    {
        if(bSubmitTransfer)
        {
            bSubmitTransfer = FALSE;
            if(bReallocTransfer)
            {
                asyncTransfer = clLibusbLibrary.AllocTransfer(0);
                clLibusbLibrary.FillBulkTransfer(asyncTransfer, device_handle, USB_ANT_EP_IN, aucData, sizeof(aucData), Callback, &iCompleted, 0);
                asyncTransfer->type = LIBUSB_TRANSFER_TYPE_BULK;
                bReallocTransfer = FALSE;
            }
            iRet = clLibusbLibrary.SubmitTransfer(asyncTransfer);
            iCompleted = 0;
            if(iRet < 0)
            {
                clLibusbLibrary.FreeTransfer(asyncTransfer);
                bStopReceiveThread = TRUE;
                return;
            }
        }

        clLibusbLibrary.HandleEventsTimeoutCompleted(NULL, &tvHandleEventsTimeout, &iCompleted);

        if(iCompleted)
        {
            switch(asyncTransfer->status)
            {
                case LIBUSB_TRANSFER_COMPLETED:
                    clRxQueue.PushArray(aucData, asyncTransfer->actual_length);
                    bSubmitTransfer = TRUE;
                    iConsecIoErrors = 0;
                    #if defined(_DEBUG) && defined(DEBUG_FILE)
                    if(bRxDebug)
                    {
                        char acMesg[255];
                        SNPRINTF(acMesg, 255, "ReceiveThread(): %d Bytes Read From USB", asyncTransfer->actual_length);
                        DSIDebug::ThreadWrite(acMesg);
                    }
                    #endif
                    break;
                default:
                    clLibusbLibrary.FreeTransfer(asyncTransfer);
                    if(iConsecIoErrors == 10)
                    {
                        bStopReceiveThread = TRUE;
                        break;
                    }
                    iConsecIoErrors++;
                    bReallocTransfer = TRUE;
                    bSubmitTransfer = TRUE;
                    #if defined(_DEBUG) && defined(DEBUG_FILE)
                    if(bRxDebug)
                    {
                        char acMesg2[255];
                        SNPRINTF(acMesg2, 255, "ReceiveThread(): Transfer Unsuccessful - Error %d", asyncTransfer->status);
                        DSIDebug::ThreadWrite(acMesg2);
                    }
                    #endif
                    break;
            }
        }
    }

    if(!iCompleted)
    {
        clLibusbLibrary.CancelTransfer(asyncTransfer);
        while(!iCompleted)
            clLibusbLibrary.HandleEventsTimeoutCompleted(NULL, &tvHandleEventsTimeout, &iCompleted);
        clLibusbLibrary.FreeTransfer(asyncTransfer);
        asyncTransfer = NULL;
    }

    bDeviceGone = TRUE;  //The read loop is dead, since we can't get any info, the device might as well be gone

    DSIThread_MutexLock(&stMutexCriticalSection);
    bStopReceiveThread = TRUE;
    DSIThread_CondSignal(&stEventReceiveThreadExit);                       // Set an event to alert the main process that Rx thread is finished and can be closed.
    DSIThread_MutexUnlock(&stMutexCriticalSection);
}

///////////////////////////////////////////////////////////////////////
DSI_THREAD_RETURN USBDeviceHandleLibusb::ProcessThread(void* pvParameter_)
{
    USBDeviceHandleLibusb* This = reinterpret_cast<USBDeviceHandleLibusb*>(pvParameter_);
    This->ReceiveThread();

    return 0;
}

#endif //defined(DSI_TYPES_LINUX)
