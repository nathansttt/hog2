/*
 * $Id: HID_Queue_Utilities.cpp,v 1.3 2006/10/18 23:53:10 nathanst Exp $
 *
 *  HID_Queue_Utilities.c
 *  HID Explorer
 *
 *  Created by ggs on Tue May 08 2001.
 
 	Copyright:	Copyright © 2001 Apple Computer, Inc., All Rights Reserved

	Disclaimer:	IMPORTANT:  This Apple software is supplied to you by Apple Computer, Inc.
			("Apple") in consideration of your agreement to the following terms, and your
			use, installation, modification or redistribution of this Apple software
			constitutes acceptance of these terms.  If you do not agree with these terms,
			please do not use, install, modify or redistribute this Apple software.

			In consideration of your agreement to abide by the following terms, and subject
			to these terms, Apple grants you a personal, non-exclusive license, under AppleÕs
			copyrights in this original Apple software (the "Apple Software"), to use,
			reproduce, modify and redistribute the Apple Software, with or without
			modifications, in source and/or binary forms; provided that if you redistribute
			the Apple Software in its entirety and without modifications, you must retain
			this notice and the following text and disclaimers in all such redistributions of
			the Apple Software.  Neither the name, trademarks, service marks or logos of
			Apple Computer, Inc. may be used to endorse or promote products derived from the
			Apple Software without specific prior written permission from Apple.  Except as
			expressly stated in this notice, no other rights or licenses, express or implied,
			are granted by Apple herein, including but not limited to any patent rights that
			may be infringed by your derivative works or by other works in which the Apple
			Software may be incorporated.

			The Apple Software is provided by Apple on an "AS IS" basis.  APPLE MAKES NO
			WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED
			WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR
			PURPOSE, REGARDING THE APPLE SOFTWARE OR ITS USE AND OPERATION ALONE OR IN
			COMBINATION WITH YOUR PRODUCTS.

			IN NO EVENT SHALL APPLE BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR
			CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
			GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
			ARISING IN ANY WAY OUT OF THE USE, REPRODUCTION, MODIFICATION AND/OR DISTRIBUTION
			OF THE APPLE SOFTWARE, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT
			(INCLUDING NEGLIGENCE), STRICT LIABILITY OR OTHERWISE, EVEN IF APPLE HAS BEEN
			ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "HID_Utilities_Internal.h"
#include "HID_Utilities_External.h"

static IOReturn HIDCreateQueue (pRecDevice pDevice);
static unsigned char HIDIsDeviceQueueEmpty (pRecDevice pDevice);
static IOReturn HIDDisposeReleaseQueue (pRecDevice pDevice);

// ==================================
// private functions

// creates a queue for a device, creates and opens device interface if required

static IOReturn HIDCreateQueue (pRecDevice pDevice)
{
    IOReturn result = kIOReturnSuccess;
	if (NULL == pDevice->queue) // do we already have a queue
	{	
		if (NULL != pDevice->interface) 
		{
			pDevice->queue =(void *) (*(IOHIDDeviceInterface **) pDevice->interface)->allocQueue (pDevice->interface); // alloc queue
			if (pDevice->queue)
			{
				result = (*(IOHIDQueueInterface **) pDevice->queue)->create (pDevice->queue, 0, kDeviceQueueSize); // create actual queue
				if (kIOReturnSuccess != result)
					HIDReportErrorNum ("Failed to create queue via create", result);
			}
			else
			{
				HIDReportError ("Failed to alloc IOHIDQueueInterface ** via allocQueue");
				result = kIOReturnError; // synthesis error
			}
		}
		else
			HIDReportErrorNum ("Device inteface does not exist for queue creation", result);
	}
    return result;
}

// ---------------------------------

// returns true if queue is empty false otherwise
// error if no device, empty if no queue

static unsigned char HIDIsDeviceQueueEmpty (pRecDevice pDevice)
{
    if (pDevice && pDevice->queue) // need device and queue
    {
        pRecElement pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO);
        while (pElement)
        {
            if ((*(IOHIDQueueInterface **) pDevice->queue)->hasElement (pDevice->queue, pElement->cookie))
                return false;
            pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO);
        } 
    }
    else if (NULL == pDevice) // if no device (if just no queue then queue must be empty)
        HIDReportError ("NULL device passed to DeviceQueueEmpty.");
    return true;   
}

// ---------------------------------

// disposes and releases queue, sets queue to NULL,.
// Note: will have no effect if device or queue do not exist

static IOReturn HIDDisposeReleaseQueue (pRecDevice pDevice)
{
    IOReturn result = kIOReturnSuccess;

    if ((NULL != pDevice) && (NULL != pDevice->queue))
	{
		// stop queue
		result = (*(IOHIDQueueInterface **) pDevice->queue)->stop (pDevice->queue);
		if (kIOReturnSuccess != result)
			HIDReportErrorNum ("Failed to stop queue.", result);
		// dispose of queue
		result = (*(IOHIDQueueInterface **) pDevice->queue)->dispose (pDevice->queue);
		if (kIOReturnSuccess != result)
			HIDReportErrorNum ("Failed to dipose queue.", result);
		// release the queue
		result = (*(IOHIDQueueInterface **) pDevice->queue)->Release (pDevice->queue);
		if (kIOReturnSuccess != result)
			HIDReportErrorNum ("Failed to release queue.", result);
			
		pDevice->queue = NULL;
	}
    return result;
}

// ==================================
// public functions

// Create and open an interface to device, required prior to extracting values or building queues
// Note: appliction now owns the device and must close and release it prior to exiting

IOReturn HIDCreateOpenDeviceInterface (io_object_t hidDevice, pRecDevice pDevice)
{
    IOReturn result = kIOReturnSuccess;
    HRESULT plugInResult = S_OK;
    SInt32 score = 0;
    IOCFPlugInInterface ** ppPlugInInterface = NULL;
	
	if (NULL == pDevice->interface)
	{
		result = IOCreatePlugInInterfaceForService (hidDevice, kIOHIDDeviceUserClientTypeID,
													kIOCFPlugInInterfaceID, &ppPlugInInterface, &score);
		if (kIOReturnSuccess == result)
		{
			// Call a method of the intermediate plug-in to create the device interface
			plugInResult = (*ppPlugInInterface)->QueryInterface (ppPlugInInterface, 
								CFUUIDGetUUIDBytes (kIOHIDDeviceInterfaceID), (void **) &(pDevice->interface));
			if (S_OK != plugInResult)
				HIDReportErrorNum ("CouldnÕt query HID class device interface from plugInInterface", plugInResult);
			IODestroyPlugInInterface (ppPlugInInterface); // replace (*ppPlugInInterface)->Release (ppPlugInInterface)
		}
		else
			HIDReportErrorNum ("Failed to create **plugInInterface via IOCreatePlugInInterfaceForService.", result);
	}
	if (NULL != pDevice->interface)
	{
		result = (*(IOHIDDeviceInterface **)pDevice->interface)->open (pDevice->interface, 0);
		if (kIOReturnSuccess != result)
			HIDReportErrorNum ("Failed to open pDevice->interface via open.", result);
	}
    return result;
}

// ---------------------------------

// queues specific element, performing any device queue set up required
// queue is started and ready to return events on exit from this function

unsigned long  HIDQueueElement (pRecDevice pDevice, pRecElement pElement)
{
    IOReturn result = kIOReturnSuccess;

	// error checking
    if ((NULL == pDevice) || (NULL == pElement))
	{
		HIDReportError ("Device or element does not exist, cannot queue element.");
        return kIOReturnBadArgument;
	}
    if (NULL == pDevice->interface) // must have interface
	{
        HIDReportError ("Device does not have interface, cannot queue element.");
		return kIOReturnError;
	}
    if (NULL == pDevice->queue) // if no queue create queue
        result = HIDCreateQueue (pDevice);
    if ((kIOReturnSuccess != result) || (NULL == pDevice->queue))
	{
        HIDReportErrorNum ("Could not queue element due to problem creating queue.", result);
		if (kIOReturnSuccess != result)
			return result;
		else
			return kIOReturnError;
	}

    // stop queue
    result = (*(IOHIDQueueInterface **) pDevice->queue)->stop (pDevice->queue);
    if (kIOReturnSuccess != result)
        HIDReportError ("Failed to stop queue.");

    // queue element
    if (!(*(IOHIDQueueInterface **) pDevice->queue)->hasElement (pDevice->queue, pElement->cookie))
    {
        result = (*(IOHIDQueueInterface **) pDevice->queue)->addElement (pDevice->queue, pElement->cookie, 0);
        if (kIOReturnSuccess != result)
            HIDReportError ("Failed to add element to queue via addElement.");
    }

    // restart queue
    result = (*(IOHIDQueueInterface **) pDevice->queue)->start (pDevice->queue);
    if (kIOReturnSuccess != result)
        HIDReportError ("Failed to start queue.");

   return result;
}

// ---------------------------------

// adds all elements to queue, performing any device queue set up required
// queue is started and ready to return events on exit from this function

unsigned long  HIDQueueDevice (pRecDevice pDevice)
{
    IOReturn result = kIOReturnSuccess;
    pRecElement pElement;
    
	// error checking
    if (NULL == pDevice)
	{
		HIDReportError ("Device does not exist, cannot queue device.");
        return kIOReturnBadArgument;
	}
    if (NULL == pDevice->interface) // must have interface
	{
        HIDReportError ("Device does not have interface, cannot queue device.");
		return kIOReturnError;
	}
    if (NULL == pDevice->queue) // if no queue create queue
        result = HIDCreateQueue (pDevice);
    if ((kIOReturnSuccess != result) || (NULL == pDevice->queue))
	{
        HIDReportErrorNum ("Could not queue device due to problem creating queue.", result);
		if (kIOReturnSuccess != result)
			return result;
		else
			return kIOReturnError;
	}

	// stop queue
	result = (*(IOHIDQueueInterface **) pDevice->queue)->stop (pDevice->queue);
	if (kIOReturnSuccess != result)
		HIDReportErrorNum ("Failed to stop queue.", result);

	// queue element
	pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO);
	while (pElement)
	{
		if (!(*(IOHIDQueueInterface **) pDevice->queue)->hasElement (pDevice->queue, pElement->cookie))
		{
			result = (*(IOHIDQueueInterface **) pDevice->queue)->addElement (pDevice->queue, pElement->cookie, 0);
			if (kIOReturnSuccess != result)
				HIDReportErrorNum ("Failed to add element to queue via addElement", result);
		}
		pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO);
	}    

	// start queue
	result = (*(IOHIDQueueInterface **) pDevice->queue)->start (pDevice->queue);
	if (kIOReturnSuccess != result)
		HIDReportErrorNum ("Failed to start queue.", result);

    return result;
}

// ---------------------------------

// removes element for queue, if last element in queue will release queue and closes device interface

unsigned long  HIDDequeueElement (pRecDevice pDevice, pRecElement pElement)
{
    IOReturn result = kIOReturnSuccess;
    if (!pDevice || !pElement)
        result = kIOReturnBadArgument;
	else
	{
		if ((pDevice->interface) && (pDevice->queue))
		{
			// stop queue
			result = (*(IOHIDQueueInterface **) pDevice->queue)->stop (pDevice->queue);
			if (kIOReturnSuccess != result)
				HIDReportErrorNum ("Failed to stop queue.", result);
		
			if ((*(IOHIDQueueInterface **) pDevice->queue)->hasElement (pDevice->queue, pElement->cookie)) // if has element then remove
			{
				result = (*(IOHIDQueueInterface **) pDevice->queue)->removeElement (pDevice->queue, pElement->cookie);
				if (kIOReturnSuccess != result)
					HIDReportErrorNum ("Failed to add element to queue via addElement", result);
			}
			
			if (HIDIsDeviceQueueEmpty (pDevice)) // release device queue and close interface if queue empty
			{
				result = HIDDisposeReleaseQueue (pDevice);
				if (kIOReturnSuccess != result)
					HIDReportErrorNum ("Failed to dispose and release queue.", result);
			}
			else // not empty so restart queue
			{
				result = (*(IOHIDQueueInterface **) pDevice->queue)->start (pDevice->queue);
				if (kIOReturnSuccess != result)
					HIDReportErrorNum ("Failed to start queue.", result);
			}
		}
		else
		{
			HIDReportError ("No device inteface or queue.");
			return kIOReturnError;
		}
	}
    return result;
}

// ---------------------------------

// completely removes all elements from queue and releases queue and closes device interface
// does not release device interfaces, application must call ReleaseHIDDeviceList on exit

unsigned long  HIDDequeueDevice (pRecDevice pDevice)
{
    IOReturn result = kIOReturnSuccess;
    if (!pDevice)
        result = kIOReturnBadArgument;
	else
	{
		if ((pDevice->interface) && (pDevice->queue))
		{
			// iterate through elements and if queued, remove
			pRecElement pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO);
			while (pElement)
			{
				if ((*(IOHIDQueueInterface **) pDevice->queue)->hasElement (pDevice->queue, pElement->cookie))
				{
					result = (*(IOHIDQueueInterface **) pDevice->queue)->removeElement (pDevice->queue, pElement->cookie);
					if (kIOReturnSuccess != result)
						HIDReportErrorNum ("Failed to add element to queue via addElement", result);
				}
				pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO);
			}    
		}
		// ensure queue is disposed and released
		// interface will be closed and released on call to ReleaseHIDDeviceList
		result = HIDDisposeReleaseQueue (pDevice);
		if (kIOReturnSuccess != result)
			HIDReportErrorNum ("Failed to dispose and release queue.", result);
	}
    return result;
}

// ---------------------------------

// releases all device queues for quit or rebuild (must be called)
// does not release device interfaces, application must call ReleaseHIDDeviceList on exit

IOReturn HIDReleaseAllDeviceQueues (void)
{
    IOReturn result = kIOReturnSuccess;
    pRecDevice pDevice = HIDGetFirstDevice ();
    while (pDevice)
    {
        result = HIDDequeueDevice (pDevice);
        if (kIOReturnSuccess != result)
            HIDReportErrorNum ("Could not dequeue device.", result);
        pDevice = HIDGetNextDevice (pDevice);
    }
    return result;
}

// ---------------------------------

// Closes and releases interface to device, should be done prior to exting application
// Note: will have no affect if device or interface do not exist
// application will "own" the device if interface is not closed
// (device may have to be plug and re-plugged in different location to get it working again without a restart)

IOReturn HIDCloseReleaseInterface (pRecDevice pDevice)
{
	IOReturn result = kIOReturnSuccess;
	
	if ((NULL != pDevice) && (NULL != pDevice->interface))
	{
		// close the interface
		result = (*(IOHIDDeviceInterface **) pDevice->interface)->close (pDevice->interface);
		if (kIOReturnNotOpen == result)
		{
			//  do nothing as device was not opened, thus can't be closed
		}
		else if (kIOReturnSuccess != result)
			HIDReportErrorNum ("Failed to close IOHIDDeviceInterface.", result);
		//release the interface
		result = (*(IOHIDDeviceInterface **) pDevice->interface)->Release (pDevice->interface);
		if (kIOReturnSuccess != result)
			HIDReportErrorNum ("Failed to release IOHIDDeviceInterface.", result);
		pDevice->interface = NULL;
	}	
	return result;
}      

// ---------------------------------

// Get the next event in the queue for a device
// elements or entire device should be queued prior to calling this with HIDQueueElement or HIDQueueDevice
// returns true if an event is avialable for the element and fills out *pHIDEvent structure, returns false otherwise
// Note: kIOReturnUnderrun returned from getNextEvent indicates an empty queue not an error condition
// Note: application should pass in a pointer to a IOHIDEventStruct cast to a void (for CFM compatibility)

unsigned char HIDGetEvent (pRecDevice pDevice, void * pHIDEvent)
{
    IOReturn result = kIOReturnSuccess;
    AbsoluteTime zeroTime = {0,0};
    if (pDevice)
	{
		if (pDevice->queue)
		{
			result = (*(IOHIDQueueInterface **) pDevice->queue)->getNextEvent (pDevice->queue, (IOHIDEventStruct *)pHIDEvent, zeroTime, 0);
			if (kIOReturnUnderrun == result)
				return false;  // no events in queue not an error per say
			else if (kIOReturnSuccess != result) // actual error versus just an empty queue
				HIDReportErrorNum ("Could not get HID event via getNextEvent.", result);
			else
				return true;
		}
		else
			HIDReportError ("Could not get HID event, queue does not exist.");
	}
	else
		HIDReportError ("Could not get HID event, device does not exist.");
    return false; // did not get event
}

// ---------------------------------

// returns current value for element, polling element
// will return 0 on error conditions which should be accounted for by application

long HIDGetElementValue (pRecDevice pDevice, pRecElement pElement)
{
    IOReturn result = kIOReturnSuccess;
    IOHIDEventStruct hidEvent;
    hidEvent.value = 0;
    
    if (NULL != pDevice)
    {
        if (NULL != pElement)
        {
 			if (NULL != pDevice->interface)
            {
                result = (*(IOHIDDeviceInterface **) pDevice->interface)->getElementValue (pDevice->interface, pElement->cookie, &hidEvent);
                if (kIOReturnSuccess != result)
                        HIDReportErrorNum ("Could not get HID element value via getElementValue.", result);
				// on 10.0.x this returns the incorrect result for negative ranges, so fix it!!!
				// this is not required on Mac OS X 10.1+
				if ((pElement->min < 0) && (hidEvent.value > pElement->max)) // assume range problem
					hidEvent.value = hidEvent.value + pElement->min - pElement->max - 1;
			}
            else
                HIDReportError ("Did not have interface for device prior to getting element value.");
        }
        else
            HIDReportError ("Bad element passed to GetElementValue.");
    }
    else
        HIDReportError ("Bad device passed to GetElementValue.");
    // record min and max for auto scale and auto ...
    if (hidEvent.value < pElement->minReport)
        pElement->minReport = hidEvent.value; 
    if (hidEvent.value > pElement->maxReport)
        pElement->maxReport = hidEvent.value; 

    return hidEvent.value;
}
