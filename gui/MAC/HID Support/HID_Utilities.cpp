/*
 * $Id: HID_Utilities.cpp,v 1.3 2006/10/18 23:53:10 nathanst Exp $
 *
 *	HID Utilities.c
 *	HID Explorer
 *
 *	Created by ggs on Fri Apr 20 2001.

	Copyright:	Copyright © 2001 Apple Computer, Inc., All Rights Reserved

	Disclaimer: IMPORTANT:	This Apple software is supplied to you by Apple Computer, Inc.
				("Apple") in consideration of your agreement to the following terms, and your
				use, installation, modification or redistribution of this Apple software
				constitutes acceptance of these terms.	If you do not agree with these terms,
				please do not use, install, modify or redistribute this Apple software.

				In consideration of your agreement to abide by the following terms, and subject
				to these terms, Apple grants you a personal, non-exclusive license, under Apple’s
				copyrights in this original Apple software (the "Apple Software"), to use,
				reproduce, modify and redistribute the Apple Software, with or without
				modifications, in source and/or binary forms; provided that if you redistribute
				the Apple Software in its entirety and without modifications, you must retain
				this notice and the following text and disclaimers in all such redistributions of
				the Apple Software.	 Neither the name, trademarks, service marks or logos of
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

// ==================================
// private functions

// path to follow to next element
static char MapChar (char c);
static void CleanString (char * targetString);
static void HIDGetElementInfo (CFTypeRef refElement, pRecElement pElement);
static void HIDAddElement (CFTypeRef refElement, pRecElement * ppElementCurrent);
static void HIDGetElementsCFArrayHandler (const void * value, void * parameter);
static void HIDGetElements (CFTypeRef refElementCurrent, pRecElement * ppElementCurrent);
static void HIDGetCollectionElements (CFMutableDictionaryRef deviceProperties, pRecElement * ppElementCurrent);
static void HIDGetDeviceInfo (io_object_t hidDevice, CFMutableDictionaryRef hidProperties, pRecDevice pDevice);
static void HIDAddDevice (pRecDevice *ppListDeviceHead, pRecDevice pNewDevice);
static pRecDevice HIDMoveDevice (pRecDevice *ppListDeviceHead, pRecDevice pNewDevice, pRecDevice *ppOldListDeviceHead);
static pRecDevice HIDBuildDevice (io_object_t hidDevice);
static pRecDevice HIDCreateSingleTypeDeviceList (io_iterator_t hidObjectIterator);
static pRecDevice HIDCreateMultiTypeDeviceList (UInt32 *usagePage, UInt32 *usage, UInt32 numDeviceTypes);
static Boolean HIDFindDeviceInList (pRecDevice pDeviceList, pRecDevice pFindDevice);
static Boolean HIDCompareUpdateDeviceList (pRecDevice *ppListDeviceHead, pRecDevice *ppNewDeviceList);
static void HIDMergeDeviceList (pRecDevice *ppNewDeviceList, pRecDevice *ppDeviceList);
static io_iterator_t HIDGetIterator (const mach_port_t masterPort, UInt32 usagePage, UInt32 usage);
static CFMutableDictionaryRef HIDSetUpMatchingDictionary (UInt32 usagePage, UInt32 usage);
static void HIDDisposeDeviceElements (pRecElement pElement);
static pRecDevice HIDDisposeDevice (pRecDevice *ppDevice);
static UInt32 HIDCountCurrentDevices (pRecDevice pDeviceList);
static Boolean HIDMatchElementTypeMask (IOHIDElementType type, HIDElementTypeMask typeMask);
static pRecElement HIDGetDeviceElement (pRecElement pElement, HIDElementTypeMask typeMask);

// ==================================
// globals

// for element retrieval
pRecDevice gCurrentGetDevice = NULL;
Boolean gfAddAsChild = true;

pRecDevice gpDeviceList = NULL;
unsigned long gNumDevices = 0;

// ==================================
// private functions

// Maps bad chars to good chars for html/printing ASCII

static char MapChar (char c)
{
	unsigned char uc = (unsigned char) c;
	switch (uc)
	{
		case 0x01: return ' ';
		case 0x02: return ' ';
		case 0x03: return ' ';
		case 0x04: return ' ';
		case 0x05: return ' ';
		case 0x06: return ' ';
		case 0x07: return ' ';
		case 0x08: return ' ';
		case 0x09: return ' ';
		case 0x0A: return ' ';
		case 0x0B: return ' ';
		case 0x0C: return ' ';
		case 0x0D: return ' ';
		case 0x0E: return ' ';
		case 0x0F: return ' ';
		case 0x10: return ' ';
		case 0x11: return ' ';
		case 0x12: return ' ';
		case 0x13: return ' ';
		case 0x14: return ' ';
		case 0x15: return ' ';
		case 0x16: return ' ';
		case 0x17: return ' ';
		case 0x18: return ' ';
		case 0x19: return ' ';
		case 0x1A: return ' ';
		case 0x1B: return ' ';
		case 0x1C: return ' ';
		case 0x1D: return ' ';
		case 0x1E: return ' ';
		case 0x1F: return ' ';
		case '/': return '-'; // use dash instead of slash
		
		case 0x7F: return ' ';
		case 0x80: return 'A';
		case 0x81: return 'A';
		case 0x82: return 'C';
		case 0x83: return 'E';
		case 0x84: return 'N';
		case 0x85: return 'O';
		case 0x86: return 'U';
		case 0x87: return 'a';
		case 0x88: return 'a';
		case 0x89: return 'a';
		case 0x8A: return 'a';
		case 0x8B: return 'a';
		case 0x8C: return 'a';
		case 0x8D: return 'c';
		case 0x8E: return 'e';
		case 0x8F: return 'e';
		case 0x90: return ' ';
		case 0x91: return ' '; // ? '
		case 0x92: return ' '; // ? '
		case 0x93: return ' '; // ? "
		case 0x94: return ' '; // ? "
		case 0x95: return ' ';
		case 0x96: return ' ';
		case 0x97: return ' ';
		case 0x98: return ' ';
		case 0x99: return ' ';
		case 0x9A: return ' ';
		case 0x9B: return 0x27;
		case 0x9C: return 0x22;
		case 0x9D: return ' ';
		case 0x9E: return ' ';
		case 0x9F: return ' ';
		case 0xA0: return ' ';
		case 0xA1: return ' ';
		case 0xA2: return ' ';
		case 0xA3: return ' ';
		case 0xA4: return ' ';
		case 0xA5: return ' ';
		case 0xA6: return ' ';
		case 0xA7: return ' ';
		case 0xA8: return ' ';
		case 0xA9: return ' ';
		case 0xAA: return ' ';
		case 0xAB: return ' ';
		case 0xAC: return ' ';
		case 0xAD: return ' ';
		case 0xAE: return ' ';
		case 0xAF: return ' ';
		case 0xB0: return ' ';
		case 0xB1: return ' ';
		case 0xB2: return ' ';
		case 0xB3: return ' ';
		case 0xB4: return ' ';
		case 0xB5: return ' ';
		case 0xB6: return ' ';
		case 0xB7: return ' ';
		case 0xB8: return ' ';
		case 0xB9: return ' ';
		case 0xBA: return ' ';
		case 0xBB: return ' ';
		case 0xBC: return ' ';
		case 0xBD: return ' ';
		case 0xBE: return ' ';
		case 0xBF: return ' ';
		case 0xC0: return ' ';
		case 0xC1: return ' ';
		case 0xC2: return ' ';
		case 0xC3: return ' ';
		case 0xC4: return ' ';
		case 0xC5: return ' ';
		case 0xC6: return ' ';
		case 0xC7: return ' ';
		case 0xC8: return ' ';
		case 0xC9: return ' ';
		case 0xCA: return ' ';
		case 0xCB: return 'A';
		case 0xCC: return 'A';
		case 0xCD: return 'O';
		case 0xCE: return ' ';
		case 0xCF: return ' ';
		case 0xD0: return '-';
		case 0xD1: return '-';
		case 0xD2: return 0x22;
		case 0xD3: return 0x22;
		case 0xD4: return 0x27;
		case 0xD5: return 0x27;
		case 0xD6: return '-'; // use dash instead of slash
		case 0xD7: return ' ';
		case 0xD8: return 'y';
		case 0xD9: return 'Y';
		case 0xDA: return '-'; // use dash instead of slash
		case 0xDB: return ' ';
		case 0xDC: return '<';
		case 0xDD: return '>';
		case 0xDE: return ' ';
		case 0xDF: return ' ';
		case 0xE0: return ' ';
		case 0xE1: return ' ';
		case 0xE2: return ',';
		case 0xE3: return ',';
		case 0xE4: return ' ';
		case 0xE5: return 'A';
		case 0xE6: return 'E';
		case 0xE7: return 'A';
		case 0xE8: return 'E';
		case 0xE9: return 'E';
		case 0xEA: return 'I';
		case 0xEB: return 'I';
		case 0xEC: return 'I';
		case 0xED: return 'I';
		case 0xEE: return 'O';
		case 0xEF: return 'O';
		case 0xF0: return ' ';
		case 0xF1: return 'O';
		case 0xF2: return 'U';
		case 0xF3: return 'U';
		case 0xF4: return 'U';
		case 0xF5: return '|';
		case 0xF6: return ' ';
		case 0xF7: return ' ';
		case 0xF8: return ' ';
		case 0xF9: return ' ';
		case 0xFA: return '.';
		case 0xFB: return ' ';
		case 0xFC: return ' ';
		case 0xFD: return 0x22;
		case 0xFE: return ' ';
		case 0xFF: return ' ';
	}
	return c;
}

// ---------------------------------

// ensures the string only contains printable ASCII characters
// using varition of my html conversion source
// input is null terminated string, change is made in place

static void CleanString (char * targetString)
{
	char * charIt = targetString;
	while (*charIt) {
		*charIt = MapChar (*charIt);
		charIt++;
	}
}

// ---------------------------------

// extracts actual specific element information from each element CF dictionary entry

static void HIDGetElementInfo (CFTypeRef refElement, pRecElement pElement)
{
	long number;
	CFTypeRef refType;
	// type, usagePage, usage already stored
	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, (void*)CFSTR(kIOHIDElementCookieKey));
	if (refType && CFNumberGetValue ((CFNumberRef)refType, kCFNumberLongType, &number))
		pElement->cookie = (IOHIDElementCookie) number;
	else
		pElement->cookie = (IOHIDElementCookie) 0;
		
	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementMinKey));
	if (refType && CFNumberGetValue ((CFNumberRef)refType, kCFNumberLongType, &number))
		pElement->min = number;
	else
		pElement->min = 0;
	pElement->maxReport = pElement->min;
	pElement->userMin = kDefaultUserMin;

	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementMaxKey));
	if (refType && CFNumberGetValue ((CFNumberRef)refType, kCFNumberLongType, &number))
		pElement->max = number;
	else
		pElement->max = 0;

	pElement->minReport = pElement->max;
	pElement->userMax = kDefaultUserMax;
	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementScaledMinKey));
	if (refType && CFNumberGetValue ((CFNumberRef)refType, kCFNumberLongType, &number))
		pElement->scaledMin = number;
	else
		pElement->scaledMin = 0;

	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementScaledMaxKey));
	if (refType && CFNumberGetValue ((CFNumberRef)refType, kCFNumberLongType, &number))
		pElement->scaledMax = number;
	else
		pElement->scaledMax = 0;

	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementSizeKey));
	if (refType && CFNumberGetValue ((CFNumberRef)refType, kCFNumberLongType, &number))
		pElement->size = number;
	else
		pElement->size = 0;

	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementIsRelativeKey));
	if (refType)
		pElement->relative = CFBooleanGetValue ((CFBooleanRef)refType);
	else
		pElement->relative = 0;

	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementIsWrappingKey));
	if (refType)
		pElement->wrapping = CFBooleanGetValue ((CFBooleanRef)refType);
	else
		pElement->wrapping = false;

	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementIsNonLinearKey));
	if (refType)
		pElement->nonLinear = CFBooleanGetValue ((CFBooleanRef)refType);
	else
		pElement->nonLinear = false;


	#ifdef kIOHIDElementHasPreferredStateKey
		refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementHasPreferredStateKey));
	#else // Mac OS X 10.0 has spelling error
		refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementHasPreferedStateKey));
	#endif
	if (refType)
		pElement->preferredState = CFBooleanGetValue ((CFBooleanRef)refType);
	else
		pElement->preferredState = false;

	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementHasNullStateKey));
	if (refType)
		pElement->nullState = CFBooleanGetValue ((CFBooleanRef)refType);
	else
		pElement->nullState = false;

	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementUnitKey));
	if (refType && CFNumberGetValue ((CFNumberRef)refType, kCFNumberLongType, &number))
		pElement->units = number;
	else
		pElement->units = 0;

	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementUnitExponentKey));
	if (refType && CFNumberGetValue ((CFNumberRef)refType, kCFNumberLongType, &number))
		pElement->unitExp = number;
	else
		pElement->unitExp = 0;

	refType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementNameKey));
	if (refType)
		if (!CFStringGetCString ((CFStringRef)refType, pElement->name, 256, CFStringGetSystemEncoding ()))
			HIDReportError ("CFStringGetCString error retrieving pElement->name.");
		CleanString (pElement->name);
	if (!*pElement->name) { // set name from vendor id/product id look up
		GetElementNameFromVendorProduct (gCurrentGetDevice->vendorID,  gCurrentGetDevice->productID, (long) pElement->cookie, pElement->name);
		if (!*pElement->name) { // if no name
			HIDGetUsageName (pElement->usagePage, pElement->usage, pElement->name);
			if (!*pElement->name) // if not usage
				sprintf (pElement->name, "Element");
		}
	}
}			 

// ---------------------------------

// examines CF dictionary value in device element hierarchy to determine if it is element of interest or a collection of more elements
// if element of interest allocate storage, add to list and retrieve element specific info
// if collection then pass on to deconstruction collection into additional individual elements

static void HIDAddElement (CFTypeRef refElement, pRecElement * ppElementCurrent)
{
	pRecDevice pDevice = gCurrentGetDevice;
	pRecElement pElement = NULL;
	long elementType, usagePage, usage;
	CFTypeRef refElementType = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementTypeKey));	 
	CFTypeRef refUsagePage = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementUsagePageKey));	
	CFTypeRef refUsage = CFDictionaryGetValue ((CFDictionaryRef)refElement, CFSTR(kIOHIDElementUsageKey));
	
	if (refElementType)
		CFNumberGetValue ((CFNumberRef)refElementType, kCFNumberLongType, &elementType);
	if (refUsagePage)
		CFNumberGetValue ((CFNumberRef)refUsagePage, kCFNumberLongType, &usagePage);
	if (refUsage)
		CFNumberGetValue ((CFNumberRef)refUsage, kCFNumberLongType, &usage);
		
	if (NULL == pDevice)
		return;
	if (elementType)
	{
		// look at types of interest
		if (elementType != kIOHIDElementTypeCollection)		   
		{
			switch (usagePage) // only interested in kHIDPage_GenericDesktop and  kHIDPage_Button
			{
				case kHIDPage_GenericDesktop:
					{
						switch (usage) // look at usage to determine function
						{
							case kHIDUsage_GD_X: 
							case kHIDUsage_GD_Y: 
							case kHIDUsage_GD_Z: 
							case kHIDUsage_GD_Rx: 
							case kHIDUsage_GD_Ry: 
							case kHIDUsage_GD_Rz: 
								pElement = (pRecElement) malloc (sizeof (recElement));
								if (pElement)
									pDevice->axis++; 
							break;
							case kHIDUsage_GD_Slider: 
								pElement = (pRecElement) malloc (sizeof (recElement));
								if (pElement)
									pDevice->sliders++; 
							break;
							case kHIDUsage_GD_Dial: 
								pElement = (pRecElement) malloc (sizeof (recElement));
								if (pElement)
									pDevice->dials++; 
							break;
							case kHIDUsage_GD_Wheel: 
								pElement = (pRecElement) malloc (sizeof (recElement));
								if (pElement)
									pDevice->wheels++; 
							break;
							case kHIDUsage_GD_Hatswitch: 
								pElement = (pRecElement) malloc (sizeof (recElement));
								if (pElement)
									pDevice->hats++; 
							break;
						}							 
					}
					break;
				case kHIDPage_Button:
					pElement = (pRecElement) malloc (sizeof (recElement));
					if (pElement)
						pDevice->buttons++;
					break;
				default:
					// just add a generic element
					pElement = (pRecElement) malloc (sizeof (recElement));
					break;
			}
		}
		else // collection
			pElement = (pRecElement) malloc (sizeof (recElement));
	}
	else
		HIDReportError ("CFNumberGetValue error when getting value for refElementType.");
	if (pElement) // add to list
	{
		// this code builds a binary tree based on the collection hierarchy of inherent in the device element layout
		// it preserves the structure of the lements as collections have children and lements are siblings to each other

		// clear record
		unsigned long i;
		char * temp = (char *) pElement;
		for (i = 0; i < sizeof (recElement); i++)
			*temp++ = 0x00;
		
		// get element info
		pElement->type = elementType;
		pElement->usagePage = usagePage;
		pElement->usage = usage;
		HIDGetElementInfo (refElement, pElement);
		
		// count elements
		pDevice->totalElements++;
		switch (pElement->type)
		{
			case kIOHIDElementTypeInput_Misc:
			case kIOHIDElementTypeInput_Button:
			case kIOHIDElementTypeInput_Axis:
			case kIOHIDElementTypeInput_ScanCodes:
				pDevice->inputs++;
				break;
			case kIOHIDElementTypeOutput:
				pDevice->outputs++;
				break;
			case kIOHIDElementTypeFeature:
				pDevice->features++;
				break;
			case kIOHIDElementTypeCollection:
				pDevice->collections++;
				break;
			default:
				HIDReportErrorNum ("Unknown element type : ", pElement->type);
		}
		// if a type that is normally an axis and has a preferred state and is not relative
		pElement->hasCenter = false;
		if ((pElement->preferredState) && (!pElement->relative) && (pElement->usagePage == kHIDPage_GenericDesktop))
			switch (pElement->usage) {
				case kHIDUsage_GD_X: 
				case kHIDUsage_GD_Y: 
				case kHIDUsage_GD_Z: 
				case kHIDUsage_GD_Rx: 
				case kHIDUsage_GD_Ry: 
				case kHIDUsage_GD_Rz: 
//				case kHIDUsage_GD_Slider: // should not have center
//				case kHIDUsage_GD_Dial:  // should not have center
				case kHIDUsage_GD_Wheel: 
				case kHIDUsage_GD_Hatswitch:
					pElement->hasCenter = true; // respect center
					pElement->initialCenter = HIDGetElementValue (pDevice, pElement);
			}

		if (NULL == *ppElementCurrent) // if at list head
		{
			pDevice->pListElements = pElement; // add current element
			*ppElementCurrent = pElement; // set current element to element we just added
			if (elementType == kIOHIDElementTypeCollection) // if this element is a collection of other elements
			{
				gfAddAsChild = true; // the next element is a child of this element
				HIDGetCollectionElements ((CFMutableDictionaryRef) refElement, &pElement); // recursively process the collection
			}
			gfAddAsChild = false; // when returning from the recursive processing or the element is not a collection, add the next as a sibling
		}
		else // have existing structure
		{
			if (gfAddAsChild) // if the previous element was a collection, let's add this as a child of the previous
			{
				// this iteration should not be needed but there maybe some untested degenerate case which this code will ensure works
				while ((*ppElementCurrent)->pChild) // step down tree until free child node found
					*ppElementCurrent = (*ppElementCurrent)->pChild;
				(*ppElementCurrent)->pChild = pElement; // insert there
			}
			else // add as sibling
			{
				// this iteration should not be needed but there maybe some untested degenerate case which this code will ensure works
				while ((*ppElementCurrent)->pSibling) // step down tree until free sibling node found
					*ppElementCurrent = (*ppElementCurrent)->pSibling;
				(*ppElementCurrent)->pSibling = pElement; // insert there
			}
			pElement->pPrevious = *ppElementCurrent; // point to previous
			*ppElementCurrent = pElement; // set current to our collection				
			if (elementType == kIOHIDElementTypeCollection) // if this element is a collection of other elements
			{
				gfAddAsChild = true; // add next set as children to this element
				HIDGetCollectionElements ((CFMutableDictionaryRef) refElement, &pElement); // recursively process the collection
			}
			gfAddAsChild = false; // add next as this elements sibling (when return from a collection or with non-collections)
		}
	}
}

// ---------------------------------

// collects information from each array member in device element list (each array memeber = element)

static void HIDGetElementsCFArrayHandler (const void * value, void * parameter)
{
	if (CFGetTypeID (value) == CFDictionaryGetTypeID ()) 
		HIDAddElement ((CFTypeRef) value, (pRecElement *) parameter);
}

// ---------------------------------

// handles retrieval of element information from arrays of elements in device IO registry information

static void HIDGetElements (CFTypeRef refElementCurrent, pRecElement *ppCurrentElement)
{
	CFTypeID type = CFGetTypeID (refElementCurrent);
	if (type == CFArrayGetTypeID()) // if element is an array
	{
		CFRange range = {0, CFArrayGetCount ((CFArrayRef)refElementCurrent)};
		// CountElementsCFArrayHandler called for each array member
		CFArrayApplyFunction ((CFArrayRef)refElementCurrent, range, HIDGetElementsCFArrayHandler, ppCurrentElement);
	}
}			 

// ---------------------------------

// handles extracting element information from element collection CF types
// used from top level element decoding and hierarchy deconstruction to flatten device element list

static void HIDGetCollectionElements (CFMutableDictionaryRef deviceProperties, pRecElement *ppCurrentCollection)
{
	CFTypeRef refElementTop = CFDictionaryGetValue (deviceProperties, CFSTR(kIOHIDElementKey));
	if (refElementTop)
		HIDGetElements (refElementTop, ppCurrentCollection);
	else
		HIDReportError ("CFDictionaryGetValue error when creating CFTypeRef for kIOHIDElementKey.");
}

// ---------------------------------

// use top level element usage page and usage to discern device usage page and usage setting appropriate values in device record

static void HIDTopLevelElementHandler (const void * value, void * parameter)
{
	CFTypeRef refCF = 0;
	if ((NULL == value) || (NULL == parameter)) 
		return; // (paramErr)
	if (CFGetTypeID (value) != CFDictionaryGetTypeID ()) 
		return;
	refCF = CFDictionaryGetValue ((CFDictionaryRef)value, CFSTR(kIOHIDElementUsagePageKey));
	if (!CFNumberGetValue ((CFNumberRef)refCF, kCFNumberLongType, &((pRecDevice) parameter)->usagePage))
		HIDReportError ("CFNumberGetValue error retrieving pDevice->usagePage.");
	refCF = CFDictionaryGetValue ((CFDictionaryRef)value, CFSTR(kIOHIDElementUsageKey));
	if (!CFNumberGetValue ((CFNumberRef)refCF, kCFNumberLongType, &((pRecDevice) parameter)->usage))
		HIDReportError ("CFNumberGetValue error retrieving pDevice->usage.");
}

// ---------------------------------

// extracts device info from CF dictionary records in IO registry

static void HIDGetDeviceInfo (io_object_t hidDevice, CFMutableDictionaryRef hidProperties, pRecDevice pDevice)
{
	CFMutableDictionaryRef usbProperties = 0;
	io_registry_entry_t parent1, parent2;
	
	// Mac OS X currently is not mirroring all USB properties to HID page so need to look at USB device page also
	// get dictionary for usb properties: step up two levels and get CF dictionary for USB properties
	if ((KERN_SUCCESS == IORegistryEntryGetParentEntry (hidDevice, kIOServicePlane, &parent1)) &&
		(KERN_SUCCESS == IORegistryEntryGetParentEntry (parent1, kIOServicePlane, &parent2)) &&
		(KERN_SUCCESS == IORegistryEntryCreateCFProperties (parent2, &usbProperties, kCFAllocatorDefault, kNilOptions)))
	{
		if (usbProperties)
		{
			CFTypeRef refCF = 0;
			// get device info
			// try hid dictionary first, if fail then go to usb dictionary
			
			
			// get transport
			refCF = CFDictionaryGetValue ((CFDictionaryRef)hidProperties, CFSTR(kIOHIDTransportKey));
			if (refCF)
			{
				if (!CFStringGetCString ((CFStringRef)refCF, pDevice->transport, 256, CFStringGetSystemEncoding ()))
					HIDReportError ("CFStringGetCString error retrieving pDevice->transport.");
				CleanString (pDevice->transport);
			}
			
			// get vendorID
			refCF = CFDictionaryGetValue ((CFDictionaryRef)hidProperties, CFSTR(kIOHIDVendorIDKey));
			if (!refCF)
				refCF = CFDictionaryGetValue (usbProperties, CFSTR("idVendor"));
			if (refCF)
			{
				if (!CFNumberGetValue ((CFNumberRef)refCF, kCFNumberLongType, &pDevice->vendorID))
					HIDReportError ("CFNumberGetValue error retrieving pDevice->vendorID.");
			}
			
			// get product ID
			refCF = CFDictionaryGetValue (hidProperties, CFSTR(kIOHIDProductIDKey));
			if (!refCF)
				refCF = CFDictionaryGetValue (usbProperties, CFSTR("idProduct"));
			if (refCF)
			{
				if (!CFNumberGetValue ((CFNumberRef)refCF, kCFNumberLongType, &pDevice->productID))
					HIDReportError ("CFNumberGetValue error retrieving pDevice->productID.");
			}
			
			// get product version
			refCF = CFDictionaryGetValue (hidProperties, CFSTR(kIOHIDVersionNumberKey));
			if (refCF)
			{
				if (!CFNumberGetValue ((CFNumberRef)refCF, kCFNumberLongType, &pDevice->version))
					HIDReportError ("CFNumberGetValue error retrieving pDevice->version.");
			}
			
			// get manufacturer name
			refCF = CFDictionaryGetValue (hidProperties, CFSTR(kIOHIDManufacturerKey));
			if (!refCF)
				refCF = CFDictionaryGetValue (usbProperties, CFSTR("USB Vendor Name"));
			if (refCF)
			{
				if (!CFStringGetCString ((CFStringRef)refCF, pDevice->manufacturer, 256, CFStringGetSystemEncoding ()))
					HIDReportError ("CFStringGetCString error retrieving pDevice->manufacturer.");
				CleanString (pDevice->manufacturer);
			}
			
			// get product name
			refCF = CFDictionaryGetValue (hidProperties, CFSTR(kIOHIDProductKey));
			if (!refCF)
				refCF = CFDictionaryGetValue (usbProperties, CFSTR("USB Product Name"));
			if (refCF)
			{
				if (!CFStringGetCString ((CFStringRef)refCF, pDevice->product, 256, CFStringGetSystemEncoding ()))
					HIDReportError ("CFStringGetCString error retrieving pDevice->product.");
				CleanString (pDevice->product);
			}
			
			// get serial
			refCF = CFDictionaryGetValue (hidProperties, CFSTR(kIOHIDSerialNumberKey));
			if (refCF)
			{
				if (!CFStringGetCString ((CFStringRef)refCF, pDevice->serial, 256, CFStringGetSystemEncoding ()))
					HIDReportError ("CFStringGetCString error retrieving pDevice->serial.");
				CleanString (pDevice->serial);
			}
			
			// get location ID
			refCF = CFDictionaryGetValue (hidProperties, CFSTR(kIOHIDLocationIDKey));
			if (!refCF)
				refCF = CFDictionaryGetValue (usbProperties, CFSTR("locationID"));
			if (refCF)
			{
				if (!CFNumberGetValue ((CFNumberRef)refCF, kCFNumberLongType, &pDevice->locID))
					HIDReportError ("CFNumberGetValue error retrieving pDevice->locID.");
			}
			
			// get usage page and usage
			refCF = CFDictionaryGetValue (hidProperties, CFSTR(kIOHIDPrimaryUsagePageKey));
			if (refCF)
			{
				if (!CFNumberGetValue ((CFNumberRef)refCF, kCFNumberLongType, &pDevice->usagePage))
					HIDReportError ("CFNumberGetValue error retrieving pDevice->usagePage.");
				refCF = CFDictionaryGetValue (hidProperties, CFSTR(kIOHIDPrimaryUsageKey));
				if (refCF)
					if (!CFNumberGetValue ((CFNumberRef)refCF, kCFNumberLongType, &pDevice->usage))
						HIDReportError ("CFNumberGetValue error retrieving pDevice->usage.");
			}
			if (NULL == refCF) // get top level element HID usage page or usage
			{
				// use top level element instead
				CFTypeRef refCFTopElement = 0;
				refCFTopElement = CFDictionaryGetValue (hidProperties, CFSTR(kIOHIDElementKey));
				{
					// refCFTopElement points to an array of element dictionaries
					CFRange range = {0, CFArrayGetCount ((CFArrayRef)refCFTopElement)};
					CFArrayApplyFunction ((CFArrayRef)refCFTopElement, range, HIDTopLevelElementHandler, NULL);
				}
			}
		}
		else
			HIDReportError ("IORegistryEntryCreateCFProperties failed to create usbProperties.");

		CFRelease (usbProperties);
		if (kIOReturnSuccess != IOObjectRelease (parent2))
			HIDReportError ("IOObjectRelease error with parent2.");
		if (kIOReturnSuccess != IOObjectRelease (parent1))
			HIDReportError ("IOObjectRelease error with parent1.");
	}
}

// ---------------------------------

// adds device to linked list of devices passed in (handles NULL lists properly)

static void HIDAddDevice (pRecDevice *ppListDeviceHead, pRecDevice pNewDevice)
{
	if (NULL == *ppListDeviceHead)
		*ppListDeviceHead = pNewDevice;
	else
	{
		pRecDevice pDevicePrevious = NULL, pDevice = *ppListDeviceHead;
		while (pDevice)
		{
			pDevicePrevious = pDevice;
			pDevice = pDevicePrevious->pNext;
		}
		pDevicePrevious->pNext = pNewDevice;
	}
	pNewDevice->pNext = NULL;
}

// ---------------------------------

// adds device to linked list of devices passed in (handles NULL lists properly)
// returns next device is old list (for properly iterating)

static pRecDevice HIDMoveDevice (pRecDevice *ppListDeviceHead, pRecDevice pNewDevice, pRecDevice *ppOldListDeviceHead)
{
	pRecDevice pDeviceNext = NULL;
	if (!pNewDevice || !ppOldListDeviceHead || !ppListDeviceHead) { // handle NULL pointers
		HIDReportError ("HIDMoveDevice, NULL input error.");
		return pDeviceNext;
	}

	// remove from old
	if (pNewDevice == *ppOldListDeviceHead) { // replacing head
		*ppOldListDeviceHead = pNewDevice->pNext;
		pDeviceNext = *ppOldListDeviceHead;
	} else {
		pRecDevice pDevicePrevious = NULL, pDevice = *ppOldListDeviceHead;
		while (pDevice && (pDevice != pNewDevice)) { // step through list until match or end
			pDevicePrevious = pDevice;
			pDevice = pDevicePrevious->pNext;
		}
		if (pDevice == pNewDevice) { // if there was a match
			pDevicePrevious->pNext = pDevice->pNext; // skip this device
			pDeviceNext = pDevice->pNext;
		} else
			HIDReportError ("HIDMoveDevice device not found when moving.");
	}

	// add to new list
	HIDAddDevice (ppListDeviceHead, pNewDevice);
	return pDeviceNext; // return next device
}

// ---------------------------------

// given a IO device object build a flat device record including device info and elements

static pRecDevice HIDBuildDevice (io_object_t hidDevice)
{
	pRecDevice pDevice = (pRecDevice) malloc (sizeof (recDevice));
	if (pDevice)
	{
		unsigned long i;
		// get dictionary for HID properties
		CFMutableDictionaryRef hidProperties = 0;
		kern_return_t result = IORegistryEntryCreateCFProperties (hidDevice, &hidProperties, kCFAllocatorDefault, kNilOptions);
		// clear record
		for (i = 0; i < sizeof (recDevice); i++) // clear array
			*((char *) pDevice + i) = 0x00;
		if ((result == KERN_SUCCESS) && hidProperties)
		{
			pRecElement pCurrentElement = NULL;
			// create device interface
			result = HIDCreateOpenDeviceInterface (hidDevice, pDevice);
			if (kIOReturnSuccess != result)
				HIDReportErrorNum ("HIDCreateOpenDeviceInterface failed.", result);
			HIDGetDeviceInfo (hidDevice, hidProperties, pDevice); // hidDevice used to find parents in registry tree
			// set current device for use in getting elements
			gCurrentGetDevice = pDevice;
			HIDGetCollectionElements (hidProperties, &pCurrentElement);
			gCurrentGetDevice = NULL;
			CFRelease (hidProperties);
		}
		else
			HIDReportErrorNum ("IORegistryEntryCreateCFProperties error when creating deviceProperties.", result);
	}
	else
		HIDReportError ("malloc error when allocating pRecDevice.");
	return pDevice;
}

// ---------------------------------

// build flat linked list of devices from device iterator

static pRecDevice HIDCreateSingleTypeDeviceList (io_iterator_t hidObjectIterator)
{
	IOReturn result = kIOReturnSuccess;
	pRecDevice pListDeviceHead = NULL;
	pRecDevice pNewDevice = NULL;
	io_object_t ioHIDDeviceObject = (io_object_t)NULL;

	while ((ioHIDDeviceObject = IOIteratorNext (hidObjectIterator)))
	{
		pNewDevice = HIDBuildDevice (ioHIDDeviceObject);
		// dump device object, it is no longer needed
		result = IOObjectRelease (ioHIDDeviceObject);
		if (KERN_SUCCESS != result)
			HIDReportErrorNum ("IOObjectRelease error with ioHIDDeviceObject.", result);
		HIDAddDevice (&pListDeviceHead, pNewDevice);
	}
	result = IOObjectRelease (hidObjectIterator); // release the iterator
	if (kIOReturnSuccess != result) 
		HIDReportErrorNum ("IOObjectRelease error with hidObjectIterator.", result);
	return pListDeviceHead;
}

// ---------------------------------

// build flat linked list of devices from list of usages and usagePages

static pRecDevice HIDCreateMultiTypeDeviceList (UInt32 *usagePage, UInt32 *usage, UInt32 numDeviceTypes)
{
	mach_port_t masterPort = (mach_port_t)NULL;
	io_iterator_t hidObjectIterator = (io_iterator_t)NULL;
	pRecDevice pNewDeviceList = NULL; // build new list
	UInt32 i;

	if (!usage || !usagePage || (numDeviceTypes == 0))
		HIDReportError ("HIDCreateMultiTypeDeviceList: NULL usage, usagePage or numDeviceTypes.");
	else {
		if (kIOReturnSuccess != IOMasterPort (bootstrap_port, &masterPort))
			HIDReportError ("HIDCreateMultiTypeDeviceList: IOMasterPort error with bootstrap_port.");
		else
			for (i = 0; i < numDeviceTypes; i++) { // for all usage and usage page types
				pRecDevice pDeviceList = NULL;
				hidObjectIterator = HIDGetIterator (masterPort, usagePage[i], usage[i]); // get iterator
				if (0 != hidObjectIterator) 
					pDeviceList = HIDCreateSingleTypeDeviceList (hidObjectIterator); // build device list
				if (0 != pDeviceList) // if there are devices to merge
					HIDMergeDeviceList (&pNewDeviceList, &pDeviceList); // merge into new list
				while (0 != pDeviceList) // dump what is left of source list (head first no clean up needed)
					pDeviceList = HIDDisposeDevice (&pDeviceList); // dispose current device return next device, will set pDeviceList to NULL
			}
	}
	return pNewDeviceList;
}

// ---------------------------------

// given a device list and a device find if device is in list
// returns true if in list, false otherwise
// this is problematic for generic devices as location id could be zero
// also match number of elements
static Boolean HIDFindDeviceInList (pRecDevice pDeviceList, pRecDevice pFindDevice)
{
	Boolean found = false; // not found
	pRecDevice pDevice = pDeviceList; // not really needed but is clearer this way
	while (pDevice && !found) { // while we still have device to look at and have not found the target device
		if ((pDevice->vendorID == pFindDevice->vendorID) &&   // if we match same vendor, product & location
			(pDevice->productID == pFindDevice->productID) && // this is not quite right for same tyes plugged into the same location but different physical devices
			(pDevice->locID == pFindDevice->locID) &&		  // since this is a corner and impossible to detect without serial numbers case we will ignore it
			(pDevice->usage == pFindDevice->usage) &&		 // ensure all device parameters really match (helps with generic devices)		
			(pDevice->usagePage == pFindDevice->usagePage) &&		  			
			(pDevice->totalElements == pFindDevice->totalElements) &&		  			
			(pDevice->features == pFindDevice->features) &&		  			
			(pDevice->inputs == pFindDevice->inputs) &&		  			
			(pDevice->outputs == pFindDevice->outputs) &&		  			
			(pDevice->collections == pFindDevice->collections) &&		  			
			(pDevice->axis == pFindDevice->axis) &&		  			
			(pDevice->buttons == pFindDevice->buttons) &&		  			
			(pDevice->hats == pFindDevice->hats) &&		  			
			(pDevice->sliders == pFindDevice->sliders) &&		  			
			(pDevice->wheels == pFindDevice->wheels) &&		  			
			(pDevice->dials == pFindDevice->dials))		  			
			found = true; // found device
		pDevice = pDevice->pNext; // step to next device
	}
	return found;
}

// ---------------------------------

// update main list with new devices and remove unplugged devices
// startegy is to update based on location of device, if this changed then previous device was
// at least unplugged and plugged in somewhere else, thus can't be considered the same
// return true if any changes
 
static Boolean HIDCompareUpdateDeviceList (pRecDevice *ppListDeviceHead, pRecDevice *ppNewDeviceList)
{
	Boolean changedList = false;
	pRecDevice pDevice = *ppListDeviceHead;
	pRecDevice pPrevDevice = NULL;
	while (pDevice)	{ // for all the devices in main list
		Boolean present = false;
		// ensure they are in new list
		present = HIDFindDeviceInList (*ppNewDeviceList, pDevice);
		// remove those that are not
		if (!present) {
			changedList = true;
			if (pDevice == *ppListDeviceHead) // if we are removing the list head
				*ppListDeviceHead = pDevice->pNext; // move list head
			pDevice = HIDDisposeDevice (&pDevice); // remove device from list and dispose device and step to next device
			if (pPrevDevice) // next device clean up required for HIDDisposeDevice 
				pPrevDevice->pNext = pDevice;
		 } else {
			pPrevDevice = pDevice; // only advance here since other case removes a device and prev stays constant
			pDevice = pDevice->pNext; // step to next device
		}
	}
	pDevice = *ppNewDeviceList;
	while (pDevice)	{ // for all the devices in new list
		Boolean present = false;
		present = HIDFindDeviceInList (*ppListDeviceHead, pDevice); // ensure they are in original list
		if (!present) { // not found in old list so move to old list from new 
			changedList = true;
			pDevice = HIDMoveDevice (ppListDeviceHead, pDevice, ppNewDeviceList); // move those that are not
		} else // found
			pDevice = pDevice->pNext; // just step to next device
	}
	return changedList;
}

// ---------------------------------

// merges two devicelist into single *ppNewDeviceList
// note: ppNewDeviceList may have head device modified (such as if it is NULL) thus pointer to pointer to device
// devices are matched on vendorID, productID, locID by HIDFindDeviceInList
// device record in pNewDeviceList maintained

static void HIDMergeDeviceList (pRecDevice *ppNewDeviceList, pRecDevice *ppDeviceList)
{
	pRecDevice pDevice = *ppDeviceList;
	while (pDevice)	{ // for all the devices in old list
		Boolean present = false;
		present = HIDFindDeviceInList (*ppNewDeviceList, pDevice); // ensure they are in new list
		if (!present) // not found in new list
			pDevice = HIDMoveDevice (ppNewDeviceList, pDevice, ppDeviceList); // move to new list and get next
		else // found in new list (so don't do anything
			pDevice = pDevice->pNext; // just step to next device
	}
}

// ---------------------------------

// builds a matching dictionary based on usage page and usage

static CFMutableDictionaryRef HIDSetUpMatchingDictionary (UInt32 usagePage, UInt32 usage)
{
	CFNumberRef refUsage = NULL, refUsagePage = NULL;
	CFMutableDictionaryRef refHIDMatchDictionary = NULL;

	// Set up a matching dictionary to search I/O Registry by class name for all HID class devices.
	refHIDMatchDictionary = IOServiceMatching (kIOHIDDeviceKey);
	if ((refHIDMatchDictionary != NULL) && (usagePage) && (usage))
	{
		// Add key for device type (joystick, in this case) to refine the matching dictionary.
		refUsagePage = CFNumberCreate (kCFAllocatorDefault, kCFNumberIntType, &usagePage);
		CFDictionarySetValue (refHIDMatchDictionary, CFSTR (kIOHIDPrimaryUsagePageKey), refUsagePage);
		CFRelease (refUsagePage);
		refUsage = CFNumberCreate (kCFAllocatorDefault, kCFNumberIntType, &usage);
		CFDictionarySetValue (refHIDMatchDictionary, CFSTR (kIOHIDPrimaryUsageKey), refUsage);
		CFRelease (refUsage);
	}
	else if (NULL == refHIDMatchDictionary)
		HIDReportError ("Failed to get HID CFMutableDictionaryRef via IOServiceMatching.");
	return refHIDMatchDictionary;
}

// ---------------------------------

// sets up a matching dictionary based on usage page and usage passed in and uses it to get a device iterator
// 

static io_iterator_t HIDGetIterator (const mach_port_t masterPort, UInt32 usagePage, UInt32 usage)
{
	IOReturn result = kIOReturnSuccess;
	CFMutableDictionaryRef hidMatchDictionary = (CFMutableDictionaryRef)NULL;
	io_iterator_t hidObjectIterator = (io_iterator_t)NULL;

	// Set up matching dictionary to search the I/O Registry for HID devices we are interested in. Dictionary reference is NULL if error.
	hidMatchDictionary = HIDSetUpMatchingDictionary (usagePage, usage);
	if (NULL == hidMatchDictionary)
	{
		HIDReportError ("Couldn’t create a matching dictionary.");
		return hidObjectIterator;
	}

	// Now search I/O Registry for matching devices.
	result = IOServiceGetMatchingServices (masterPort, hidMatchDictionary, &hidObjectIterator);
	if (kIOReturnSuccess != result)
		 HIDReportErrorNum ("Failed to create IO object iterator, error:", result);
// --- this is a "normal" failure condition (such as no devices of a certain type plugged in) so do not issue a warning
//	else if (NULL == hidObjectIterator) // likely no HID devices which matched selection criteria are connected
//		 HIDReportError ("Warning: Could not find any matching devices, thus iterator creation failed.");
 
	// IOServiceGetMatchingServices consumes a reference to the dictionary, so we don't need to release the dictionary ref.
	hidMatchDictionary = NULL;
	return hidObjectIterator;
}

// ---------------------------------

// disposes of the element list associated with a device and the memory associated with the list
// uses depthwise recursion to dispose both collections and elements.

static void HIDDisposeDeviceElements (pRecElement pElement)
{
	if (pElement)
	{
		if (pElement->pChild)
			HIDDisposeDeviceElements (pElement->pChild);
		if (pElement->pSibling)
			HIDDisposeDeviceElements (pElement->pSibling);
		free (pElement);
	}
	
}

// ---------------------------------

// disposes of a single device, closing and releasing interface, freeing memory for device and elements, setting device pointer to NULL
// all your device no longer belong to us... (i.e., you do not 'own' the device anymore)
// routines calling this need to fix up previous previous device next device pointer to point to the device returned by this routine

static pRecDevice HIDDisposeDevice (pRecDevice *ppDevice)
{
	kern_return_t result = KERN_SUCCESS;
	pRecDevice pDeviceNext = NULL;
	if (*ppDevice)
	{
		// save next device prior to disposing of this device
		pDeviceNext = (*ppDevice)->pNext;
		HIDDisposeDeviceElements ((*ppDevice)->pListElements);
		(*ppDevice)->pListElements = NULL;
		result = HIDCloseReleaseInterface (*ppDevice); // function sanity checks interface value (now application does not own device)
		if (kIOReturnSuccess != result)
			HIDReportErrorNum ("HIDCloseReleaseInterface failed when trying to dipose device.", result);
		free (*ppDevice);
		*ppDevice = NULL;
	}
	return pDeviceNext;
}

// ---------------------------------

// count number of devices in global device list (gpDeviceList)

static UInt32 HIDCountCurrentDevices (pRecDevice pDeviceList)
{
	pRecDevice pDevice = pDeviceList;
	UInt32 devices = 0;
	while (pDevice)
	{
		devices++;
		pDevice = pDevice->pNext;
	}
	return devices;
}

// ---------------------------------

// matches type masks passed in to actual element types (which are not set up to be used as a mask
static Boolean HIDMatchElementTypeMask (IOHIDElementType type, HIDElementTypeMask typeMask)
{
	if (typeMask & kHIDElementTypeInput)
		if ((type == kIOHIDElementTypeInput_Misc) || (type == kIOHIDElementTypeInput_Button) || (type == kIOHIDElementTypeInput_Axis) || (type == kIOHIDElementTypeInput_ScanCodes))
			return true;
	if (typeMask & kHIDElementTypeOutput)
		if (type == kIOHIDElementTypeOutput)
			return true;
	if (typeMask & kHIDElementTypeFeature)
		if (type == kIOHIDElementTypeFeature)
			return true;
	if (typeMask & kHIDElementTypeCollection)
		if (type == kIOHIDElementTypeCollection)
			return true;
	return false;
}

// ---------------------------------

static pRecElement HIDGetDeviceElement (pRecElement pElement, HIDElementTypeMask typeMask)
{
	// we are asking for this element
	if (NULL != pElement)
	{
		if (HIDMatchElementTypeMask ((IOHIDElementType)pElement->type, typeMask)) // if the type match what we are looking for
			return pElement; // return the element
		else
			return HIDGetNextDeviceElement (pElement, typeMask); // else get the next one
	}
	return NULL;
}

// ---------------------------------

void HIDDumpDevice (pRecDevice pDevice)
{
	printf ("    %s %s: 0x%lX", pDevice->manufacturer, pDevice->product, (unsigned long)pDevice);	
	if (!pDevice->pListElements)
		printf (" (NO elements)");		
	else
		printf (" (%ld elements)", pDevice->inputs);		
}

// ---------------------------------

void HIDDumpDeviceList (pRecDevice pDeviceList)
{
	printf ("HIDDumpDeviceList (0x%lX):\n", (unsigned long)pDeviceList);
	pRecDevice pDevice = pDeviceList;
	while (pDevice) {
		HIDDumpDevice (pDevice);
		printf("\n");
		pDevice = pDevice->pNext;
	}
}

#pragma mark -
// =================================
// public functions

// builds list of devices with elements (allocates memory and captures devices) 
// in which the devices could be of different types/usages
// list is allocated internally within HID Utilites and can be accessed via accessor functions
// structures within list are considered flat and user accessable, but not user modifiable
// can be called again to rebuild list to account for new devices (will do the right thing in case of disposing existing list)
// usagePage, usage are each a numDeviceTypes sized array of matching usage and usage pages
// returns true if succesful

Boolean HIDBuildMultiDeviceList (UInt32 *pUsagePage, UInt32 *pUsage, UInt32 numDeviceTypes)
{
	gpDeviceList = HIDCreateMultiTypeDeviceList (pUsagePage, pUsage, numDeviceTypes);	
	gNumDevices = HIDCountCurrentDevices (gpDeviceList);  // set count of main list
	if (gpDeviceList)
		return true; 
	else
		return false;
}

// ---------------------------------

// same as above but this uses a single usagePage and usage

Boolean HIDBuildDeviceList (UInt32 usagePage, UInt32 usage)
{
	return HIDBuildMultiDeviceList (&usagePage, &usage, 1); // call HIDBuildMultiDeviceList with a single usage
}

// ---------------------------------

// updates the current device list for any new/removed devices
// if this is called before HIDBuildDeviceList the it functions like HIDBuildMultiDeviceList
// usagePage, usage are each a numDeviceTypes sized array of matching usage and usage pages
// returns true if successful which means if any device were added or removed (the device config changed)

Boolean HIDUpdateDeviceList (UInt32 *pUsagePage, UInt32 *pUsage, UInt32 numDeviceTypes)
{
	Boolean updateResult = false;
	pRecDevice pNewDeviceList = HIDCreateMultiTypeDeviceList (pUsagePage, pUsage, numDeviceTypes); // get current list
	if (gpDeviceList) { // if we have a list already
		updateResult = HIDCompareUpdateDeviceList (&gpDeviceList, &pNewDeviceList); // update main list
		while (NULL != pNewDeviceList) // dispose new list after compare and update of main list (heaad first distruction to no next device clean up)
			pNewDeviceList = HIDDisposeDevice (&pNewDeviceList); // dispose current device return next device, will set pNewDeviceList to NULL
	} else
		gpDeviceList = pNewDeviceList;
	gNumDevices = HIDCountCurrentDevices (gpDeviceList);  // set count
	return updateResult;
}

// ---------------------------------

// release list built by above function
// MUST be called prior to application exit to properly release devices
// if not called (or app crashes) devices can be recovered by pluging into different location in USB chain

void HIDReleaseDeviceList (void)
{
	IOReturn result = kIOReturnSuccess;
	if (NULL != gpDeviceList) // release queues and device
	{
		result = HIDReleaseAllDeviceQueues ();
		if (kIOReturnSuccess != result) 
			HIDReportErrorNum ("Could not Dequeue Device elements and release devices.", result);
	}
	while (NULL != gpDeviceList) // head first destruction no clean needed
		 gpDeviceList = HIDDisposeDevice (&gpDeviceList); // dispose current device return next device will set gpDeviceList to NULL
	gNumDevices = 0;
}

// ---------------------------------

// does a device list exist

Boolean HIDHaveDeviceList (void)
{
	if (NULL != gpDeviceList)
		return true;
	return false;
}

// ---------------------------------

// how many HID devices have been found
// returns 0 if no device list exist

UInt32 HIDCountDevices (void)
{
	return gNumDevices;
}

// ---------------------------------

// how many elements does a specific device have
// returns 0 if device is invlaid or NULL

UInt32 HIDCountDeviceElements (pRecDevice pDevice, HIDElementTypeMask typeMask)
{
	long count = 0;
	if (NULL != pDevice)
	{
		if (typeMask & kHIDElementTypeInput)
			count += pDevice->inputs;
		if (typeMask & kHIDElementTypeOutput)
			count += pDevice->outputs;
		if (typeMask & kHIDElementTypeFeature)
			count += pDevice->features;
		if (typeMask & kHIDElementTypeCollection)
			count += pDevice->collections;
	}
	return count;
}

// ---------------------------------

// get the first device in the device list
// returns NULL if no list exists

pRecDevice HIDGetFirstDevice (void)
{
	return gpDeviceList;
}

// ---------------------------------

// get next device in list given current device as parameter
// returns NULL if end of list

pRecDevice HIDGetNextDevice (pRecDevice pDevice)
{
	if (NULL != pDevice)
		return pDevice->pNext;
	else
		return NULL;
}

// ---------------------------------

// get the first element of device passed in as parameter
// returns NULL if no list exists or device does not exists or is NULL
pRecElement HIDGetFirstDeviceElement (pRecDevice pDevice, HIDElementTypeMask typeMask)
{
	if (NULL != pDevice)
	{
		if (HIDMatchElementTypeMask ((IOHIDElementType)pDevice->pListElements->type, typeMask)) // ensure first type matches
			return pDevice->pListElements;
		else
			return HIDGetNextDeviceElement (pDevice->pListElements, typeMask);
	}
	else
		return NULL;
}

// ---------------------------------

// get next element of given device in list given current element as parameter
// will walk down each collection then to next element or collection (depthwise traverse)
// returns NULL if end of list
// uses mask of HIDElementTypeMask to restrict element found
// use kHIDElementTypeIO to get previous HIDGetNextDeviceElement functionality
pRecElement HIDGetNextDeviceElement (pRecElement pElement, HIDElementTypeMask typeMask)
{
	// should only have elements passed in (though someone could mix calls and pass us a collection)
	// collection means return the next child or sibling (in that order)
	// element means returnt he next sibling (as elements can't have children
	if (NULL != pElement)
	{
		if (pElement->pChild)
		{
			if (pElement->type != kIOHIDElementTypeCollection)
				HIDReportError ("Malformed element list: found child of element.");
			else	
				return HIDGetDeviceElement (pElement->pChild, typeMask); // return the child of this element
		}
		else if (pElement->pSibling)
		{
			return HIDGetDeviceElement (pElement->pSibling, typeMask); //return the sibling of this element
		}
		else // at end back up correctly
		{
			pRecElement pPreviousElement = NULL;
			// malformed device ending in collection
			if (pElement->type == kIOHIDElementTypeCollection)
				HIDReportError ("Malformed device: found collection at end of element chain.");
			// walk back up tree to element prior to first collection ecountered and take next element 
			while (pElement)
			{
				pPreviousElement = pElement;
				pElement = pElement->pPrevious; // look at previous element
				if (!pElement) // if we are at the top and did not find another branch
					return NULL;
				// if the previous element is the branch element
				// which means the previous the we are coming back up the child path and we have a sibling
				if ((pPreviousElement != pElement->pSibling) && pElement->pSibling)
					break;
			}
			// now we must have been down the child route so go down the sibling route
			pElement = pElement->pSibling; // element of interest
			return HIDGetDeviceElement (pElement, typeMask); // otherwise return this element
		}
	}
	return NULL;
}

// ---------------------------------

// get previous element of given device in list given current element as parameter
// this wlaks directly up the tree to the top element and does not search at each level
// returns NULL if beginning of list
// uses mask of HIDElementTypeMask to restrict element found
// use kHIDElementTypeIO to get non-collection elements
pRecElement HIDGetPreviousDeviceElement (pRecElement pElement, HIDElementTypeMask typeMask)
{
	pRecElement pPreviousElement = pElement->pPrevious;
	// walk back up tree to element prior
	while (pPreviousElement && !HIDMatchElementTypeMask ((IOHIDElementType)pPreviousElement->type, typeMask))
	{
		pElement = pPreviousElement; // look at previous element
		pPreviousElement = pElement->pPrevious;
	}
	return pPreviousElement; // return this element
}

// ---------------------------------

// returns C string type name given a type enumeration passed in as parameter (see IOHIDKeys.h)
// returns "Unknown Type" for invalid types

void HIDGetTypeName (unsigned long type, char * cstrName)
{
	switch (type)
	{
		case kIOHIDElementTypeInput_Misc:
			sprintf(cstrName, "Miscellaneous Input");
			break;
		case kIOHIDElementTypeInput_Button:
			sprintf(cstrName, "Button Input");
			break;
		case kIOHIDElementTypeInput_Axis:
			sprintf(cstrName, "Axis Input");
			break;
		case kIOHIDElementTypeInput_ScanCodes:
			sprintf(cstrName, "Scan Code Input");
			break;
		case kIOHIDElementTypeOutput:
			sprintf(cstrName, "Output");
			break;
		case kIOHIDElementTypeFeature:
			sprintf(cstrName, "Feature");
			break;
		case kIOHIDElementTypeCollection:
			sprintf(cstrName, "Collection");
			break;
		default:
			sprintf(cstrName, "Unknown Type");
			break;
	}
}

// ---------------------------------

// returns C string usage given usage page and usage passed in as parameters (see IOUSBHIDParser.h)
// returns usage page and usage values in string form for unknown values

void HIDGetUsageName (long valueUsagePage, long valueUsage, char * cstrName)
{
	switch (valueUsagePage)
	{
		case kHIDPage_Undefined:
			 switch (valueUsage)
			{
				default: sprintf (cstrName, "Undefined Page, Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_GenericDesktop:
			switch (valueUsage)
			{
				case kHIDUsage_GD_Pointer: sprintf (cstrName, "Pointer"); break;
				case kHIDUsage_GD_Mouse: sprintf (cstrName, "Mouse"); break;
				case kHIDUsage_GD_Joystick: sprintf (cstrName, "Joystick"); break;
				case kHIDUsage_GD_GamePad: sprintf (cstrName, "GamePad"); break;
				case kHIDUsage_GD_Keyboard: sprintf (cstrName, "Keyboard"); break;
				case kHIDUsage_GD_Keypad: sprintf (cstrName, "Keypad"); break;
				case kHIDUsage_GD_MultiAxisController: sprintf (cstrName, "Multi-Axis Controller"); break;

				case kHIDUsage_GD_X: sprintf (cstrName, "X-Axis"); break;
				case kHIDUsage_GD_Y: sprintf (cstrName, "Y-Axis"); break;
				case kHIDUsage_GD_Z: sprintf (cstrName, "Z-Axis"); break;
				case kHIDUsage_GD_Rx: sprintf (cstrName, "X-Rotation"); break;
				case kHIDUsage_GD_Ry: sprintf (cstrName, "Y-Rotation"); break;
				case kHIDUsage_GD_Rz: sprintf (cstrName, "Z-Rotation"); break;
				case kHIDUsage_GD_Slider: sprintf (cstrName, "Slider"); break;
				case kHIDUsage_GD_Dial: sprintf (cstrName, "Dial"); break;
				case kHIDUsage_GD_Wheel: sprintf (cstrName, "Wheel"); break;
				case kHIDUsage_GD_Hatswitch: sprintf (cstrName, "Hatswitch"); break;
				case kHIDUsage_GD_CountedBuffer: sprintf (cstrName, "Counted Buffer"); break;
				case kHIDUsage_GD_ByteCount: sprintf (cstrName, "Byte Count"); break;
				case kHIDUsage_GD_MotionWakeup: sprintf (cstrName, "Motion Wakeup"); break;
				case kHIDUsage_GD_Start: sprintf (cstrName, "Start"); break;
				case kHIDUsage_GD_Select: sprintf (cstrName, "Select"); break;

				case kHIDUsage_GD_Vx: sprintf (cstrName, "X-Velocity"); break;
				case kHIDUsage_GD_Vy: sprintf (cstrName, "Y-Velocity"); break;
				case kHIDUsage_GD_Vz: sprintf (cstrName, "Z-Velocity"); break;
				case kHIDUsage_GD_Vbrx: sprintf (cstrName, "X-Rotation Velocity"); break;
				case kHIDUsage_GD_Vbry: sprintf (cstrName, "Y-Rotation Velocity"); break;
				case kHIDUsage_GD_Vbrz: sprintf (cstrName, "Z-Rotation Velocity"); break;
				case kHIDUsage_GD_Vno: sprintf (cstrName, "Vno"); break;

				case kHIDUsage_GD_SystemControl: sprintf (cstrName, "System Control"); break;
				case kHIDUsage_GD_SystemPowerDown: sprintf (cstrName, "System Power Down"); break;
				case kHIDUsage_GD_SystemSleep: sprintf (cstrName, "System Sleep"); break;
				case kHIDUsage_GD_SystemWakeUp: sprintf (cstrName, "System Wake Up"); break;
				case kHIDUsage_GD_SystemContextMenu: sprintf (cstrName, "System Context Menu"); break;
				case kHIDUsage_GD_SystemMainMenu: sprintf (cstrName, "System Main Menu"); break;
				case kHIDUsage_GD_SystemAppMenu: sprintf (cstrName, "System App Menu"); break;
				case kHIDUsage_GD_SystemMenuHelp: sprintf (cstrName, "System Menu Help"); break;
				case kHIDUsage_GD_SystemMenuExit: sprintf (cstrName, "System Menu Exit"); break;
				case kHIDUsage_GD_SystemMenu: sprintf (cstrName, "System Menu"); break;
				case kHIDUsage_GD_SystemMenuRight: sprintf (cstrName, "System Menu Right"); break;
				case kHIDUsage_GD_SystemMenuLeft: sprintf (cstrName, "System Menu Left"); break;
				case kHIDUsage_GD_SystemMenuUp: sprintf (cstrName, "System Menu Up"); break;
				case kHIDUsage_GD_SystemMenuDown: sprintf (cstrName, "System Menu Down"); break;

				case kHIDUsage_GD_DPadUp: sprintf (cstrName, "DPad Up"); break;
				case kHIDUsage_GD_DPadDown: sprintf (cstrName, "DPad Down"); break;
				case kHIDUsage_GD_DPadRight: sprintf (cstrName, "DPad Right"); break;
				case kHIDUsage_GD_DPadLeft: sprintf (cstrName, "DPad Left"); break;

				case kHIDUsage_GD_Reserved: sprintf (cstrName, "Reserved"); break;

				default: sprintf (cstrName, "Generic Desktop Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_Simulation:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Simulation Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_VR:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "VR Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_Sport:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Sport Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_Game:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Game Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_KeyboardOrKeypad:
			switch (valueUsage)
			{
				case kHIDUsage_KeyboardErrorRollOver: sprintf (cstrName, "Error Roll Over"); break;
				case kHIDUsage_KeyboardPOSTFail: sprintf (cstrName, "POST Fail"); break;
				case kHIDUsage_KeyboardErrorUndefined: sprintf (cstrName, "Error Undefined"); break;
				case kHIDUsage_KeyboardA: sprintf (cstrName, "A"); break;
				case kHIDUsage_KeyboardB: sprintf (cstrName, "B"); break;
				case kHIDUsage_KeyboardC: sprintf (cstrName, "C"); break;
				case kHIDUsage_KeyboardD: sprintf (cstrName, "D"); break;
				case kHIDUsage_KeyboardE: sprintf (cstrName, "E"); break;
				case kHIDUsage_KeyboardF: sprintf (cstrName, "F"); break;
				case kHIDUsage_KeyboardG: sprintf (cstrName, "G"); break;
				case kHIDUsage_KeyboardH: sprintf (cstrName, "H"); break;
				case kHIDUsage_KeyboardI: sprintf (cstrName, "I"); break;
				case kHIDUsage_KeyboardJ: sprintf (cstrName, "J"); break;
				case kHIDUsage_KeyboardK: sprintf (cstrName, "K"); break;
				case kHIDUsage_KeyboardL: sprintf (cstrName, "L"); break;
				case kHIDUsage_KeyboardM: sprintf (cstrName, "M"); break;
				case kHIDUsage_KeyboardN: sprintf (cstrName, "N"); break;
				case kHIDUsage_KeyboardO: sprintf (cstrName, "O"); break;
				case kHIDUsage_KeyboardP: sprintf (cstrName, "P"); break;
				case kHIDUsage_KeyboardQ: sprintf (cstrName, "Q"); break;
				case kHIDUsage_KeyboardR: sprintf (cstrName, "R"); break;
				case kHIDUsage_KeyboardS: sprintf (cstrName, "S"); break;
				case kHIDUsage_KeyboardT: sprintf (cstrName, "T"); break;
				case kHIDUsage_KeyboardU: sprintf (cstrName, "U"); break;
				case kHIDUsage_KeyboardV: sprintf (cstrName, "V"); break;
				case kHIDUsage_KeyboardW: sprintf (cstrName, "W"); break;
				case kHIDUsage_KeyboardX: sprintf (cstrName, "X"); break;
				case kHIDUsage_KeyboardY: sprintf (cstrName, "Y"); break;
				case kHIDUsage_KeyboardZ: sprintf (cstrName, "Z"); break;
				case kHIDUsage_Keyboard1: sprintf (cstrName, "1"); break;
				case kHIDUsage_Keyboard2: sprintf (cstrName, "2"); break;
				case kHIDUsage_Keyboard3: sprintf (cstrName, "3"); break;
				case kHIDUsage_Keyboard4: sprintf (cstrName, "4"); break;
				case kHIDUsage_Keyboard5: sprintf (cstrName, "5"); break;
				case kHIDUsage_Keyboard6: sprintf (cstrName, "6"); break;
				case kHIDUsage_Keyboard7: sprintf (cstrName, "7"); break;
				case kHIDUsage_Keyboard8: sprintf (cstrName, "8"); break;
				case kHIDUsage_Keyboard9: sprintf (cstrName, "9"); break;
				case kHIDUsage_Keyboard0: sprintf (cstrName, "0"); break;
				case kHIDUsage_KeyboardReturnOrEnter: sprintf (cstrName, "Return"); break;
				case kHIDUsage_KeyboardEscape: sprintf (cstrName, "Escape"); break;
				case kHIDUsage_KeyboardDeleteOrBackspace: sprintf (cstrName, "Delete"); break;
				case kHIDUsage_KeyboardTab: sprintf (cstrName, "Tab"); break;
				case kHIDUsage_KeyboardSpacebar: sprintf (cstrName, "Spacebar"); break;
				case kHIDUsage_KeyboardHyphen: sprintf (cstrName, "Dash"); break;
				case kHIDUsage_KeyboardEqualSign: sprintf (cstrName, "Equal"); break;
				case kHIDUsage_KeyboardOpenBracket: sprintf (cstrName, "Left Square Bracket"); break;
				case kHIDUsage_KeyboardCloseBracket: sprintf (cstrName, "Right Square Bracket"); break;
				case kHIDUsage_KeyboardBackslash: sprintf (cstrName, "Slash"); break;
				case kHIDUsage_KeyboardNonUSPound: sprintf (cstrName, "Non-US #"); break;
				case kHIDUsage_KeyboardSemicolon: sprintf (cstrName, "Semi-Colon"); break;
				case kHIDUsage_KeyboardQuote: sprintf (cstrName, "Single Quote"); break;
				case kHIDUsage_KeyboardGraveAccentAndTilde: sprintf (cstrName, "Grave Accent"); break;
				case kHIDUsage_KeyboardComma: sprintf (cstrName, "Comma"); break;
				case kHIDUsage_KeyboardPeriod: sprintf (cstrName, "Period"); break;
				case kHIDUsage_KeyboardSlash: sprintf (cstrName, "Slash"); break;
				case kHIDUsage_KeyboardCapsLock: sprintf (cstrName, "Caps Lock"); break;
				case kHIDUsage_KeyboardF1: sprintf (cstrName, "F1"); break;
				case kHIDUsage_KeyboardF2: sprintf (cstrName, "F2"); break;
				case kHIDUsage_KeyboardF3: sprintf (cstrName, "F3"); break;
				case kHIDUsage_KeyboardF4: sprintf (cstrName, "F4"); break;
				case kHIDUsage_KeyboardF5: sprintf (cstrName, "F5"); break;
				case kHIDUsage_KeyboardF6: sprintf (cstrName, "F6"); break;
				case kHIDUsage_KeyboardF7: sprintf (cstrName, "F7"); break;
				case kHIDUsage_KeyboardF8: sprintf (cstrName, "F8"); break;
				case kHIDUsage_KeyboardF9: sprintf (cstrName, "F9"); break;
				case kHIDUsage_KeyboardF10: sprintf (cstrName, "F10"); break;
				case kHIDUsage_KeyboardF11: sprintf (cstrName, "F11"); break;
				case kHIDUsage_KeyboardF12: sprintf (cstrName, "F12"); break;
				case kHIDUsage_KeyboardPrintScreen: sprintf (cstrName, "Print Screen"); break;
				case kHIDUsage_KeyboardScrollLock: sprintf (cstrName, "Scroll Lock"); break;
				case kHIDUsage_KeyboardPause: sprintf (cstrName, "Pause"); break;
				case kHIDUsage_KeyboardInsert: sprintf (cstrName, "Insert"); break;
				case kHIDUsage_KeyboardHome: sprintf (cstrName, "Home"); break;
				case kHIDUsage_KeyboardPageUp: sprintf (cstrName, "Page Up"); break;
				case kHIDUsage_KeyboardDeleteForward: sprintf (cstrName, "Delete Forward"); break;
				case kHIDUsage_KeyboardEnd: sprintf (cstrName, "End"); break;
				case kHIDUsage_KeyboardPageDown: sprintf (cstrName, "Page Down"); break;
				case kHIDUsage_KeyboardRightArrow: sprintf (cstrName, "Right Arrow"); break;
				case kHIDUsage_KeyboardLeftArrow: sprintf (cstrName, "Left Arrow"); break;
				case kHIDUsage_KeyboardDownArrow: sprintf (cstrName, "Down Arrow"); break;
				case kHIDUsage_KeyboardUpArrow: sprintf (cstrName, "Up Arrow"); break;
				case kHIDUsage_KeypadNumLock: sprintf (cstrName, "Keypad NumLock"); break;
				case kHIDUsage_KeypadSlash: sprintf (cstrName, "Keypad Slash"); break;
				case kHIDUsage_KeypadAsterisk: sprintf (cstrName, "Keypad Asterisk"); break;
				case kHIDUsage_KeypadHyphen: sprintf (cstrName, "Keypad Dash"); break;
				case kHIDUsage_KeypadPlus: sprintf (cstrName, "Keypad Plus"); break;
				case kHIDUsage_KeypadEnter: sprintf (cstrName, "Keypad Enter"); break;
				case kHIDUsage_Keypad1: sprintf (cstrName, "Keypad 1"); break;
				case kHIDUsage_Keypad2: sprintf (cstrName, "Keypad 2"); break;
				case kHIDUsage_Keypad3: sprintf (cstrName, "Keypad 3"); break;
				case kHIDUsage_Keypad4: sprintf (cstrName, "Keypad 4"); break;
				case kHIDUsage_Keypad5: sprintf (cstrName, "Keypad 5"); break;
				case kHIDUsage_Keypad6: sprintf (cstrName, "Keypad 6"); break;
				case kHIDUsage_Keypad7: sprintf (cstrName, "Keypad 7"); break;
				case kHIDUsage_Keypad8: sprintf (cstrName, "Keypad 8"); break;
				case kHIDUsage_Keypad9: sprintf (cstrName, "Keypad 9"); break;
				case kHIDUsage_Keypad0: sprintf (cstrName, "Keypad 0"); break;
				case kHIDUsage_KeypadPeriod: sprintf (cstrName, "Keypad Period"); break;
				case kHIDUsage_KeyboardNonUSBackslash: sprintf (cstrName, "Non-US Backslash"); break;
				case kHIDUsage_KeyboardApplication: sprintf (cstrName, "Application"); break;
				case kHIDUsage_KeyboardPower: sprintf (cstrName, "Power"); break;
				case kHIDUsage_KeypadEqualSign: sprintf (cstrName, "Keypad Equal"); break;
				case kHIDUsage_KeyboardF13: sprintf (cstrName, "F13"); break;
				case kHIDUsage_KeyboardF14: sprintf (cstrName, "F14"); break;
				case kHIDUsage_KeyboardF15: sprintf (cstrName, "F15"); break;
				case kHIDUsage_KeyboardF16: sprintf (cstrName, "F16"); break;
				case kHIDUsage_KeyboardF17: sprintf (cstrName, "F17"); break;
				case kHIDUsage_KeyboardF18: sprintf (cstrName, "F18"); break;
				case kHIDUsage_KeyboardF19: sprintf (cstrName, "F19"); break;
				case kHIDUsage_KeyboardF20: sprintf (cstrName, "F20"); break;
				case kHIDUsage_KeyboardF21: sprintf (cstrName, "F21"); break;
				case kHIDUsage_KeyboardF22: sprintf (cstrName, "F22"); break;
				case kHIDUsage_KeyboardF23: sprintf (cstrName, "F23"); break;
				case kHIDUsage_KeyboardF24: sprintf (cstrName, "F24"); break;
				case kHIDUsage_KeyboardExecute: sprintf (cstrName, "Execute"); break;
				case kHIDUsage_KeyboardHelp: sprintf (cstrName, "Help"); break;
				case kHIDUsage_KeyboardMenu: sprintf (cstrName, "Menu"); break;
				case kHIDUsage_KeyboardSelect: sprintf (cstrName, "Select"); break;
				case kHIDUsage_KeyboardStop: sprintf (cstrName, "Stop"); break;
				case kHIDUsage_KeyboardAgain: sprintf (cstrName, "Again"); break;
				case kHIDUsage_KeyboardUndo: sprintf (cstrName, "Undo"); break;
				case kHIDUsage_KeyboardCut: sprintf (cstrName, "Cut"); break;
				case kHIDUsage_KeyboardCopy: sprintf (cstrName, "Copy"); break;
				case kHIDUsage_KeyboardPaste: sprintf (cstrName, "Paste"); break;
				case kHIDUsage_KeyboardFind: sprintf (cstrName, "Find"); break;
				case kHIDUsage_KeyboardMute: sprintf (cstrName, "Mute"); break;
				case kHIDUsage_KeyboardVolumeUp: sprintf (cstrName, "Volume Up"); break;
				case kHIDUsage_KeyboardVolumeDown: sprintf (cstrName, "Volume Down"); break;
				case kHIDUsage_KeyboardLockingCapsLock: sprintf (cstrName, "Locking Caps Lock"); break;
				case kHIDUsage_KeyboardLockingNumLock: sprintf (cstrName, "Locking Num Lock"); break;
				case kHIDUsage_KeyboardLockingScrollLock: sprintf (cstrName, "Locking Scroll Lock"); break;
				case kHIDUsage_KeypadComma: sprintf (cstrName, "Keypad Comma"); break;
				case kHIDUsage_KeypadEqualSignAS400: sprintf (cstrName, "Keypad Equal Sign for AS-400"); break;
				case kHIDUsage_KeyboardInternational1: sprintf (cstrName, "International1"); break;
				case kHIDUsage_KeyboardInternational2: sprintf (cstrName, "International2"); break;
				case kHIDUsage_KeyboardInternational3: sprintf (cstrName, "International3"); break;
				case kHIDUsage_KeyboardInternational4: sprintf (cstrName, "International4"); break;
				case kHIDUsage_KeyboardInternational5: sprintf (cstrName, "International5"); break;
				case kHIDUsage_KeyboardInternational6: sprintf (cstrName, "International6"); break;
				case kHIDUsage_KeyboardInternational7: sprintf (cstrName, "International7"); break;
				case kHIDUsage_KeyboardInternational8: sprintf (cstrName, "International8"); break;
				case kHIDUsage_KeyboardInternational9: sprintf (cstrName, "International9"); break;
				case kHIDUsage_KeyboardLANG1: sprintf (cstrName, "LANG1"); break;
				case kHIDUsage_KeyboardLANG2: sprintf (cstrName, "LANG2"); break;
				case kHIDUsage_KeyboardLANG3: sprintf (cstrName, "LANG3"); break;
				case kHIDUsage_KeyboardLANG4: sprintf (cstrName, "LANG4"); break;
				case kHIDUsage_KeyboardLANG5: sprintf (cstrName, "LANG5"); break;
				case kHIDUsage_KeyboardLANG6: sprintf (cstrName, "LANG6"); break;
				case kHIDUsage_KeyboardLANG7: sprintf (cstrName, "LANG7"); break;
				case kHIDUsage_KeyboardLANG8: sprintf (cstrName, "LANG8"); break;
				case kHIDUsage_KeyboardLANG9: sprintf (cstrName, "LANG9"); break;
				case kHIDUsage_KeyboardAlternateErase: sprintf (cstrName, "Alternate Erase"); break;
				case kHIDUsage_KeyboardSysReqOrAttention: sprintf (cstrName, "SysReq or Attention"); break;
				case kHIDUsage_KeyboardCancel: sprintf (cstrName, "Cancel"); break;
				case kHIDUsage_KeyboardClear: sprintf (cstrName, "Clear"); break;
				case kHIDUsage_KeyboardPrior: sprintf (cstrName, "Prior"); break;
				case kHIDUsage_KeyboardReturn: sprintf (cstrName, "Return"); break;
				case kHIDUsage_KeyboardSeparator: sprintf (cstrName, "Separator"); break;
				case kHIDUsage_KeyboardOut: sprintf (cstrName, "Out"); break;
				case kHIDUsage_KeyboardOper: sprintf (cstrName, "Oper"); break;
				case kHIDUsage_KeyboardClearOrAgain: sprintf (cstrName, "Clear or Again"); break;
				case kHIDUsage_KeyboardCrSelOrProps: sprintf (cstrName, "CrSel or Props"); break;
				case kHIDUsage_KeyboardExSel: sprintf (cstrName, "ExSel"); break;
				case kHIDUsage_KeyboardLeftControl: sprintf (cstrName, "Left Control"); break;
				case kHIDUsage_KeyboardLeftShift: sprintf (cstrName, "Left Shift"); break;
				case kHIDUsage_KeyboardLeftAlt: sprintf (cstrName, "Left Alt"); break;
				case kHIDUsage_KeyboardLeftGUI: sprintf (cstrName, "Left GUI"); break;
				case kHIDUsage_KeyboardRightControl: sprintf (cstrName, "Right Control"); break;
				case kHIDUsage_KeyboardRightShift: sprintf (cstrName, "Right Shift"); break;
				case kHIDUsage_KeyboardRightAlt: sprintf (cstrName, "Right Alt"); break;
				case kHIDUsage_KeyboardRightGUI: sprintf (cstrName, "Right GUI"); break;
				case kHIDUsage_Keyboard_Reserved: sprintf (cstrName, "Reserved"); break;
				default: sprintf (cstrName, "Keyboard Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_LEDs:
			switch (valueUsage)
			{
				// some LED usages
				case kHIDUsage_LED_IndicatorRed: sprintf (cstrName, "Red LED"); break;
				case kHIDUsage_LED_IndicatorGreen: sprintf (cstrName, "Green LED"); break;
				case kHIDUsage_LED_IndicatorAmber: sprintf (cstrName, "Amber LED"); break;
				case kHIDUsage_LED_GenericIndicator: sprintf (cstrName, "Generic LED"); break;
				case kHIDUsage_LED_SystemSuspend: sprintf (cstrName, "System Suspend LED"); break;
				case kHIDUsage_LED_ExternalPowerConnected: sprintf (cstrName, "External Power LED"); break;
				default: sprintf (cstrName, "LED Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_Button:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Button #%ld", valueUsage); break;
			}
			break;
		case kHIDPage_Ordinal:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Ordinal Instance %lx", valueUsage); break;
			}
			break;
		case kHIDPage_Telephony:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Telephony Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_Consumer:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Consumer Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_Digitizer:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Digitizer Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_Unicode:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Unicode Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_AlphanumericDisplay:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Alphanumeric Display Usage 0x%lx", valueUsage); break;
			}
			break;
	   case kHIDPage_BarCodeScanner:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Bar Code Scanner Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_Scale:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Scale Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_CameraControl:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Camera Control Usage 0x%lx", valueUsage); break;
			}
			break;
		case kHIDPage_Arcade:
			switch (valueUsage)
			{
				default: sprintf (cstrName, "Arcade Usage 0x%lx", valueUsage); break;
			}
			break;
		default:
			if (valueUsagePage > kHIDPage_VendorDefinedStart)
				sprintf (cstrName, "Vendor Defined Usage 0x%lx", valueUsage);
			else	
				sprintf (cstrName, "Page: 0x%lx, Usage: 0x%lx", valueUsagePage, valueUsage);
			break;
	}
}

// ---------------------------------

// returns calibrated value given raw value passed in
// calibrated value is equal to min and max values returned by HIDGetElementValue since device list built scaled to element reported min and max values
// 2002-05-01: ggs: add support for center calibration

SInt32 HIDCalibrateValue (SInt32 value, pRecElement pElement)
{
	if (NULL != pElement)
	{
		float readScale, deviceScale = pElement->max - pElement->min;
		if (!pElement->hasCenter) { // no center for this element type
			if (pElement->maxReport == pElement->minReport)
				return value; // no scaling as 
			else {
				readScale = pElement->maxReport - pElement->minReport;
				return (SInt32)(((value - pElement->minReport) * deviceScale / readScale) + pElement->min);
			}
		} else { // has center
			if (value < pElement->initialCenter) {
				readScale = pElement->initialCenter - pElement->minReport;
				if (readScale > 0.5f) { // if we have a positve range and reasonable value
					return (SInt32)(((value - pElement->minReport) * deviceScale * 0.5f / readScale ) + pElement->min);
				} else return value; // bad scaling
			} else if (value > pElement->initialCenter) {
				readScale = pElement->maxReport - pElement->initialCenter;
				if (readScale > 0.5f) { // if we have a positve range and reasonable value
					return (SInt32)(((value - pElement->initialCenter) * deviceScale * 0.5f / readScale ) + pElement->min + deviceScale / 2.0);
				} else return value; // bad scaling
			} else // at center so return center of range
				return (SInt32)(pElement->min + deviceScale / 2.0);
		}
	}
	else
		return 0; // bad element passed in 
}

// ---------------------------------

// returns scaled value given raw value passed in
// scaled value is equal to current value assumed to be in the range of element reported min and max values scaled to user min and max scaled values

SInt32 HIDScaleValue (SInt32 value, pRecElement pElement)
{
	float deviceScale = pElement->userMax - pElement->userMin;
	float readScale = pElement->max - pElement->min;
	if (readScale == 0)
		return value;
	return (SInt32)((value - pElement->min) * deviceScale / readScale + pElement->userMin);
}
