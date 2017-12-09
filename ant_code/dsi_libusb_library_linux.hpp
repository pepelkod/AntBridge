/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except
in compliance with this license.

Copyright (c) Dynastream Innovations Inc. 2016
All rights reserved.
*/

#ifndef DSI_LIBUSB_LIBRARY_LINUX_HPP
#define DSI_LIBUSB_LIBRARY_LINUX_HPP

#include "types.h"

#include <libusb-1.0/libusb.h>

#include <memory>

//Struct to minimize code changes moving from libusb-win32/0.1 to libusb 1.0
struct libusb_device_node {

  libusb_device *device;

  struct libusb_device_descriptor descriptor;
};

struct LibusbError
{
   enum Enum
   {
      NONE,
      NO_LIBRARY,
      NO_FUNCTION
   };

   private: LibusbError();
};

/*
 * This class started out as a singleton, but you cannot control when a static variable destroys
 * itself at the end of an application.  Therefore, the class was changed to a normal wrapper around
 * the library.  This means that if you retrieve an instance of the class and then destroy it, you will
 * unload the whole library.  Since we would rather the library stay loaded for the rest of the application if
 * we use it, there is an extra call to load the library so that the library is guaranteed to not unload until
 * the end of the application.  As well, since each instance also calls to load the library, the library is
 * not freed until everyone is done using it.
 */

//NOTE: Make sure this class isn't being depended upon by another global variable!
//NOTE: Not thread-safe.
class LibusbLibrary
{
  public:
                                                                  //!!maybe return another smart pointer like shared_ptr?
   static BOOL Load(std::auto_ptr<const LibusbLibrary>& clAutoLibrary_);  //!! Alternative to creating directly and having to worry about try statements
                                                            /* Otherwise you'd have to do this...
                                                            //Get a reference to library
                                                            auto_ptr<LibusbLibrary> pclAutoSiLibrary(NULL);
                                                            try { pclAutoSiLibrary.reset(new LibusbLibrary); }
                                                            catch(...) { return clList; }
                                                            const LibusbLibrary& clSiLibrary = *pclAutoSiLibrary;
                                                            */

   //could do...
   //static const LibusbLibrary& Load(BOOL& bSuccess_);

   LibusbLibrary(); //throw(LibusbError::Enum)
   virtual ~LibusbLibrary() throw();

#if defined(DSI_TYPES_LINUX)
   //Prototypes for functions found in the libusb.so
   typedef int                                 (*Init_t)(libusb_context**);
   typedef void                                (*Exit_t)(struct libusb_context*);
   typedef void                                (*SetDebug_t)(libusb_context*, int);
   typedef int                                 (*Open_t)(libusb_device*, libusb_device_handle**);
   typedef void                                (*Close_t)(libusb_device_handle*);
   typedef int                                 (*ResetDevice_t)(libusb_device_handle*);
   typedef int                                 (*GetDeviceDescriptor_t)(libusb_device*, struct libusb_device_descriptor*);
   typedef int                                 (*GetStringDescriptorAscii_t)(libusb_device_handle*, uint8_t, unsigned char*, int);
   typedef libusb_device*                      (*GetDevice_t)(libusb_device_handle*);
   typedef ssize_t                             (*GetDeviceList_t)(libusb_context*, libusb_device***);
   typedef void                                (*FreeDeviceList_t)(libusb_device**, int);
   typedef int                                 (*ClaimInterface_t)(libusb_device_handle*, int);
   typedef int                                 (*ReleaseInterface_t)(libusb_device_handle*, int);
   typedef libusb_device*                      (*ReferenceDevice_t)(libusb_device*);
   typedef void                                (*UnreferenceDevice_t)(libusb_device*);
   typedef struct libusb_transfer *            (*AllocTransfer_t)(int);
   typedef int                                 (*SubmitTransfer_t)(struct libusb_transfer*);
   typedef int                                 (*CancelTransfer_t)(struct libusb_transfer*);
   typedef void                                (*FreeTransfer_t)(struct libusb_transfer*);
   typedef void                                (*FillBulkTransfer_t)(struct libusb_transfer*, libusb_device_handle*, unsigned char, unsigned char*, int, libusb_transfer_cb_fn, void*, unsigned int);
   typedef int                                 (*DetachKernelDriver_t)(libusb_device_handle*, int);
   typedef int                                 (*AttachKernelDriver_t)(libusb_device_handle*, int);
   typedef int                                 (*KernelDriverActive_t)(libusb_device_handle*, int);
   typedef int                                 (*HandleEventsTimeoutCompleted_t)(libusb_context*, struct timeval*, int*);

#endif

#if defined(DSI_TYPES_LINUX)
   //Needs cleanup, do not use all of these
   Init_t Init;
   Exit_t Exit;
   SetDebug_t SetDebug;
   Open_t Open;
   Close_t Close;
   ResetDevice_t ResetDevice;
   GetDeviceDescriptor_t GetDeviceDescriptor;
   GetStringDescriptorAscii_t GetStringDescriptorAscii;
   GetDevice_t GetDevice;
   GetDeviceList_t GetDeviceList;
   FreeDeviceList_t FreeDeviceList;
   ClaimInterface_t ClaimInterface;
   ReleaseInterface_t ReleaseInterface;
   ReferenceDevice_t ReferenceDevice;
   UnreferenceDevice_t UnreferenceDevice;
   AllocTransfer_t AllocTransfer;
   SubmitTransfer_t SubmitTransfer;
   CancelTransfer_t CancelTransfer;
   FreeTransfer_t FreeTransfer;
   FillBulkTransfer_t FillBulkTransfer;
   DetachKernelDriver_t DetachKernelDriver;
   AttachKernelDriver_t AttachKernelDriver;
   KernelDriverActive_t KernelDriverActive;
   HandleEventsTimeoutCompleted_t HandleEventsTimeoutCompleted;
#endif

  private:

   LibusbError::Enum LoadFunctions();
   void FreeFunctions();

   static std::auto_ptr<LibusbLibrary> clAutoInstance;  //keeps the library loaded for the duration of the application
                                                               //NOTE: There is no control when this gets destroyed at end of program
                                                               //       but it doesn't matter because it's main purpose is to keep the library loaded
                                                               //       during the duration of the whole application.

   static BOOL bStaticSet;

   //!!Could dynamically make all instances and push them onto a static list to delete when we get to it
};

#endif //DSI_LIBUSB_LIBRARY_LINUX_HPP
