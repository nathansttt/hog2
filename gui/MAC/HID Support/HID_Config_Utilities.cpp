/*
 * $Id: HID_Config_Utilities.cpp,v 1.3 2007/03/22 23:22:23 nathanst Exp $
 *
 *  HID_Config_Utilities.c
 *  HID Config Save
 *
 *  Created by ggs on Thu May 24 2001.
 *  Copyright (c) 2001 Apple. All rights reserved.
 *
 */


#include <stdlib.h> // malloc
#include <time.h> // clock

#include "HID_Utilities_Internal.h"
#include "HID_Utilities_External.h"

// ---------------------------------

// polls single device's elements for a change greater than kPercentMove.  Times out after given time
// returns 1 and pointer to element if found
// returns 0 and NULL for both parameters if not found

unsigned char HIDConfigureSingleDeviceAction (pRecDevice pDevice, pRecElement * ppElement, float timeout)
{
    unsigned long maxElements = 0;
    long * saveValueArray;
    pRecElement pElement = NULL;
    unsigned char found = 0, done = 0;
	clock_t start = clock (), end;
    unsigned long i;

	if (!pDevice)
		return 0;
     if (0 == HIDHaveDeviceList ())   // if we do not have a device list
		return 0; // return 0

    // build list of device and elements to save current values
	maxElements = HIDCountDeviceElements (pDevice, kHIDElementTypeIO);
	saveValueArray = (long *) malloc (sizeof (long) * maxElements); // 2D array to save values
	for (i = 0; i <maxElements; i++) // clear array
		*(saveValueArray + i) = 0x00000000;
		
	// store current values
	short elementNum = 0;
	pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO);
	while (pElement)
	{
		*(saveValueArray + elementNum) = HIDGetElementValue (pDevice, pElement);
		pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); 
		elementNum++;
	}
    
    // poll all devices and elements, compare current value to save +/- kPercentMove
    while ((!found) && (!done))
    {
		double secs;
		// are we done?
		end = clock();
		secs = (double)(end - start) / CLOCKS_PER_SEC;
		if (secs > timeout)
			done = 1;
		elementNum = 0;
		pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO);
		while (pElement)
		{
			long initialValue = *(saveValueArray + elementNum);
			long value = HIDGetElementValue (pDevice, pElement);
			long delta = (long)((float)(pElement->max - pElement->min) * kPercentMove * 0.01);
			if (((initialValue + delta) < value) || ((initialValue - delta) > value)) {
				found = 1;
				break;
			}
			pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); 
			elementNum++;
		}
    }
    
    // return device and element moved
    if (found) {
		*ppElement = pElement;
		return 1;
    } else {
		*ppElement = NULL;
		return 0;
	}
}

// ---------------------------------

// polls all devices and elements for a change greater than kPercentMove.  Times out after given time
// returns 1 and pointer to device and element if found
// returns 0 and NULL for both parameters if not found

unsigned char HIDConfigureAction (pRecDevice * ppDevice, pRecElement * ppElement, float timeout)
{
    unsigned long devices, maxElements = 0;
    long * saveValueArray;
    pRecDevice pDevice = NULL;
    pRecElement pElement = NULL;
    short deviceNum = 0;
    unsigned char found = 0, done = 0;
	clock_t start = clock (), end;
    unsigned long i;
    
     if (0 == HIDHaveDeviceList ())   // if we do not have a device list
		if (0 == HIDBuildDeviceList (0, 0)) // if we could not build anorther list (use generic usage and page)
			return 0; // return 0

    // build list of device and elements to save current values
    devices = HIDCountDevices ();
    pDevice = HIDGetFirstDevice ();
    while (pDevice)
    {
		if (HIDCountDeviceElements (pDevice, kHIDElementTypeIO) > maxElements)
			maxElements = HIDCountDeviceElements (pDevice, kHIDElementTypeIO);
		pDevice = HIDGetNextDevice (pDevice);
	}
	saveValueArray = (long *) malloc (sizeof (long) * devices * maxElements); // 2D array to save values
	for (i = 0; i < devices * maxElements; i++) // clear array
		*(saveValueArray + i) = 0x00000000;
		
	// store current values
	deviceNum = 0;
	pDevice = HIDGetFirstDevice ();
	while (pDevice)
	{
		short elementNum = 0;
		pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO);
		while (pElement)
		{
			*(saveValueArray + (deviceNum * maxElements) + elementNum) = HIDGetElementValue (pDevice, pElement);
			pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); 
			elementNum++;
		}
		pDevice = HIDGetNextDevice (pDevice);
		deviceNum++;
    }
    
    // poll all devices and elements, compare current value to save +/- kPercentMove
    while ((!found) && (!done))
    {
		double secs;
		// are we done?
		end = clock();
		secs = (double)(end - start) / CLOCKS_PER_SEC;
		if (secs > timeout)
			done = 1;
		deviceNum = 0;
		pDevice = HIDGetFirstDevice ();
		while (pDevice)
		{
			short elementNum = 0;
			pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO);
			while (pElement)
			{
				long initialValue = *(saveValueArray + (deviceNum * maxElements) + elementNum);
				long value = HIDGetElementValue (pDevice, pElement);
				long delta = (long)((float)(pElement->max - pElement->min) * kPercentMove * 0.01);
				if (((initialValue + delta) < value) || ((initialValue - delta) > value))
				{
					found = 1;
					break;
				}
				pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); 
				elementNum++;
			}
			if (found)
				break;
			pDevice = HIDGetNextDevice (pDevice);
			deviceNum++;
		}
    }
    
    // return device and element moved
    if (found)
    {
		*ppDevice = pDevice;
		*ppElement = pElement;
		return 1;
    }
	else
	{
		*ppDevice = NULL;
		*ppElement = NULL;
		return 0;
	}
}

// ---------------------------------

// takes input records, save required info
// assume file is open and at correct position.
// will always write to file (if file exists) size of recSaveHID, even if device and or element is bad

void HIDSaveElementConfig (FILE * fileRef, pRecDevice pDevice, pRecElement pElement, long actionCookie)
{
    // must save:
    // actionCookie
    // Device: serial,vendorID, productID, location, usagePage, usage
    // Element: cookie, usagePage, usage,
    recSaveHID saveRec;
	HIDSetElementConfig (&saveRec, pDevice, pElement, actionCookie);
    // write to file
    if (fileRef)
    	fwrite ((void *)&saveRec, sizeof (recSaveHID), 1, fileRef);
}

// ---------------------------------

// take file, read one record (assume file position is correct and file is open)
// search for matching device
// return pDevice, pElement and cookie for action
 
long HIDRestoreElementConfig (FILE * fileRef, pRecDevice * ppDevice, pRecElement * ppElement)
{
    // Device: serial,vendorID, productID, location, usagePage, usage
    // Element: cookie, usagePage, usage,
    
    recSaveHID restoreRec;
    fread ((void *) &restoreRec, 1, sizeof (recSaveHID), fileRef);
	return HIDGetElementConfig (&restoreRec, ppDevice, ppElement);
}

// ---------------------------------

// Set up a config record for saving
// takes an input records, returns record user can save as they want 
// Note: the save rec must be pre-allocated by the calling app and will be filled out
void HIDSetElementConfig (pRecSaveHID pConfigRec, pRecDevice pDevice, pRecElement pElement, long actionCookie)
{
	// must save:
    // actionCookie
    // Device: serial,vendorID, productID, location, usagePage, usage
    // Element: cookie, usagePage, usage,
    pConfigRec->actionCookie = actionCookie;
    // device
    // need to add serial number when I have a test case
	if (pDevice && pElement) {
		pConfigRec->vendorID = pDevice->vendorID;
		pConfigRec->productID = pDevice->productID;
		pConfigRec->locID = pDevice->locID;
		pConfigRec->usage = pDevice->usage;
		pConfigRec->usagePage = pDevice->usagePage;

		pConfigRec->usagePageE = pElement->usagePage;
		pConfigRec->usageE = pElement->usage;
		pConfigRec->minReport = pElement->minReport;
		pConfigRec->maxReport = pElement->maxReport;
		pConfigRec->cookie = pElement->cookie;
	} else {
		pConfigRec->vendorID = 0;
		pConfigRec->productID = 0;
		pConfigRec->locID = 0;
		pConfigRec->usage = 0;
		pConfigRec->usagePage = 0;

		pConfigRec->usagePageE = 0;
		pConfigRec->usageE = 0;
		pConfigRec->minReport = 0;
		pConfigRec->maxReport = 0;
		pConfigRec->cookie = 0;
	}
}

// ---------------------------------
#if 0
void HIDDumpConfig (pRecSaveHID pConfigRec)
{
	printf ("Config Record for action: %ld\n  vendor: %ld    product: %ld    location: %ld\n  usage: %ld    usagePage: %ld\n  usagePageE: %ld    usageE: %ld\n  minReport: %ld    maxReport: %ld\n  cookie: %ld\n", pConfigRec->actionCookie, pConfigRec->vendorID, pConfigRec->productID, pConfigRec->locID, pConfigRec->usage, pConfigRec->usagePage, pConfigRec->usagePageE, pConfigRec->usageE, pConfigRec->minReport, pConfigRec->maxReport, pConfigRec->cookie);
}
#endif // 0
// ---------------------------------

// Get matching element from config record
// takes a pre-allocated and filled out config record
// search for matching device
// return pDevice, pElement and cookie for action
long HIDGetElementConfig (pRecSaveHID pConfigRec, pRecDevice * ppDevice, pRecElement * ppElement)
{
	if (!pConfigRec->locID && !pConfigRec->vendorID && !pConfigRec->productID && !pConfigRec->usage && !pConfigRec->usagePage) { // early out
		*ppDevice = NULL;
		*ppElement = NULL;
		return pConfigRec->actionCookie;
	}

    pRecDevice pDevice, pFoundDevice = NULL;
    pRecElement pElement, pFoundElement = NULL;
     // compare to current device list for matches
    // look for device
    if (pConfigRec->locID && pConfigRec->vendorID && pConfigRec->productID)
    { // look for specific device type plug in to same port
		pDevice = HIDGetFirstDevice ();
		while (pDevice)
		{
			if ((pConfigRec->locID == pDevice->locID) &&
			(pConfigRec->vendorID == pDevice->vendorID) &&
			(pConfigRec->productID == pDevice->productID))
			pFoundDevice = pDevice;
			if (pFoundDevice)
				break;
			pDevice = HIDGetNextDevice (pDevice);
		}
		if (pFoundDevice)
		{
			pElement = HIDGetFirstDeviceElement (pFoundDevice, kHIDElementTypeIO);
			while (pElement)
			{
				if (pConfigRec->cookie == pElement->cookie)
					pFoundElement = pElement;
				if (pFoundElement)
					break;
				pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); 
			}
			// if no cookie match (should NOT occur) match on usage
			pElement = HIDGetFirstDeviceElement (pFoundDevice, kHIDElementTypeIO);
			while (pElement)
			{
				if ((pConfigRec->usageE == pElement->usage) &&
					(pConfigRec->usagePageE == pElement->usagePage))
					pFoundElement = pElement;
				if (pFoundElement)
					break;
				pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); 
			}
			if (pElement) {
				// set min and max values if same device
				pElement->minReport = pConfigRec->minReport;
				pElement->maxReport = pConfigRec->maxReport;
			}
		}
    }
    // if we have not found a match, look at just vendor and product
    if ((NULL == pFoundDevice) && (pConfigRec->vendorID && pConfigRec->productID))
    {
		pDevice = HIDGetFirstDevice ();
		while (pDevice)
		{
			if ((pConfigRec->vendorID == pDevice->vendorID) &&
			(pConfigRec->productID == pDevice->productID))
			pFoundDevice = pDevice;
			if (pFoundDevice)
			break;
			pDevice = HIDGetNextDevice (pDevice);
		}
		// match elements by cookie since same device type
		if (pFoundDevice)
		{
			pElement = HIDGetFirstDeviceElement (pFoundDevice, kHIDElementTypeIO);
			while (pElement)
			{
				if (pConfigRec->cookie == pElement->cookie)
					pFoundElement = pElement;
				if (pFoundElement)
					break;
				pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); 
			}
			// if no cookie match (should NOT occur) match on usage
			pElement = HIDGetFirstDeviceElement (pFoundDevice, kHIDElementTypeIO);
			while (pElement)
			{
				if ((pConfigRec->usageE == pElement->usage) &&
					(pConfigRec->usagePageE == pElement->usagePage))
					pFoundElement = pElement;
				if (pFoundElement)
					break;
				pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); 
			}
			if (pElement) {
				// set min and max values if same device
				pElement->minReport = pConfigRec->minReport;
				pElement->maxReport = pConfigRec->maxReport;
				
			}
		}
    }
	// can't find matching device return NULL, do not return first device
    if ((NULL == pFoundDevice) || (NULL == pFoundElement))
    {
		// no HID device
		*ppDevice = NULL;
		*ppElement = NULL;
		return pConfigRec->actionCookie;
    }
    else
    {
		// HID device
		*ppDevice = pFoundDevice;
		*ppElement = pFoundElement;
		return pConfigRec->actionCookie;
    }
}
