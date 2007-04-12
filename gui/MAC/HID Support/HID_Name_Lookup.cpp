/*
 * $Id: HID_Name_Lookup.cpp,v 1.2 2006/10/18 23:53:10 nathanst Exp $
 *
 *  HID_Name_Lookup.c
 *  HID Utilities Slib
 *
 *  Created by Geoffrey Stahl on Mon Mar 18 2002.
 *  Copyright (c) 2002 Apple Computer, Inc. All rights reserved.
 *
 */

#include "HID_Name_Lookup.h"
#include "HID_Utilities_Internal.h"

// ---------------------------------
#if 0
// not used
static CFPropertyListRef XML_HIDCookieStringLoad (void)
{
	CFPropertyListRef tCFPropertyListRef = NULL;
	CFURLRef resFileCFURLRef = CFBundleCopyResourceURL(CFBundleGetMainBundle(),CFSTR("HID_cookie_strings"),CFSTR("plist"),NULL);
	CFDataRef resCFDataRef;

	if (CFURLCreateDataAndPropertiesFromResource(kCFAllocatorDefault,resFileCFURLRef,&resCFDataRef,nil,nil,nil))
	{
		if (NULL != resCFDataRef)
		{
			CFStringRef errorString;

			tCFPropertyListRef = CFPropertyListCreateFromXMLData(kCFAllocatorDefault,resCFDataRef,kCFPropertyListImmutable,&errorString);
			if (NULL == tCFPropertyListRef)
				CFShow(errorString);
			CFRelease(resCFDataRef);
		}
	}
	if (NULL != resFileCFURLRef)
		CFRelease(resFileCFURLRef);
	return tCFPropertyListRef;
}

// ---------------------------------

static Boolean XML_NameSearch(const long pVendorID,const long pProductID,const long pCookie,char* pCstr)
{
	static CFPropertyListRef tCFPropertyListRef = NULL;
	Boolean results = false;

	if (NULL == tCFPropertyListRef)
		tCFPropertyListRef = XML_HIDCookieStringLoad ();
	if (NULL != tCFPropertyListRef)
	{
		if (CFDictionaryGetTypeID() == CFGetTypeID(tCFPropertyListRef))
		{
			CFDictionaryRef vendorCFDictionaryRef;
			CFStringRef	vendorKeyCFStringRef;
			vendorKeyCFStringRef = CFStringCreateWithFormat(kCFAllocatorDefault,NULL,CFSTR("%ld"),pVendorID);

			if (CFDictionaryGetValueIfPresent(tCFPropertyListRef,vendorKeyCFStringRef,(const void**) &vendorCFDictionaryRef))
			{
				CFDictionaryRef productCFDictionaryRef;
				CFStringRef	productKeyCFStringRef;
				CFStringRef	vendorCFStringRef;

				if (CFDictionaryGetValueIfPresent(vendorCFDictionaryRef,CFSTR("Name"),(const void**) &vendorCFStringRef))
				{
					//CFShow(vendorCFStringRef);
				}
				productKeyCFStringRef = CFStringCreateWithFormat(kCFAllocatorDefault,NULL,CFSTR("%ld"),pProductID);

				if (CFDictionaryGetValueIfPresent(vendorCFDictionaryRef,productKeyCFStringRef,(const void**) &productCFDictionaryRef))
				{
					CFStringRef fullCFStringRef;
					CFStringRef	cookieKeyCFStringRef;
					CFStringRef	productCFStringRef;
					CFStringRef	cookieCFStringRef;

					if (CFDictionaryGetValueIfPresent(productCFDictionaryRef,CFSTR("Name"),(const void**) &productCFStringRef))
					{
						//CFShow(productCFStringRef);
					}
					cookieKeyCFStringRef = CFStringCreateWithFormat(kCFAllocatorDefault,NULL,CFSTR("%ld"),pCookie);

					if (CFDictionaryGetValueIfPresent(productCFDictionaryRef,cookieKeyCFStringRef,(const void**) &cookieCFStringRef))
					{
						fullCFStringRef = CFStringCreateWithFormat(kCFAllocatorDefault,NULL,CFSTR("%@ %@ %@"),
												 vendorCFStringRef,productCFStringRef,cookieCFStringRef);
						// CFShow(cookieCFStringRef);
					}
#if 1	// set true while debugging to create a "fake" device, element, cookie string.
					else
					{
						fullCFStringRef = CFStringCreateWithFormat(kCFAllocatorDefault,NULL,CFSTR("%@ %@ #%@"),
												 vendorCFStringRef,productCFStringRef,cookieKeyCFStringRef);
					}
#endif
					if (fullCFStringRef)
					{
						// CFShow(fullCFStringRef);
						results = CFStringGetCString(
								   fullCFStringRef,pCstr,CFStringGetLength(fullCFStringRef) * sizeof(UniChar) + 1,kCFStringEncodingMacRoman);
						CFRelease(fullCFStringRef);
					}
					CFRelease(cookieKeyCFStringRef);
				}
				CFRelease(productKeyCFStringRef);
			}
			CFRelease(vendorKeyCFStringRef);
		}
		//CFRelease(tCFPropertyListRef);
	}
	return results;
}
#endif

void GetElementNameFromVendorProduct (long vendorID, long productID, long cookie, char * pName)
{
#if 0
	XML_NameSearch (vendorID, productID, cookie, pName); // load from plist
#else // old static code
	*pName = 0; // clear name
	switch (vendorID) {
		case kMacallyID:
			switch (productID) {
				case kiShockID:
					switch (cookie) {
						case 3: sprintf (pName, "D-Pad Up"); break;
						case 4: sprintf (pName, "D-Pad Down"); break;
						case 5: sprintf (pName, "D-Pad Left"); break;
						case 6: sprintf (pName, "D-Pad Right"); break;
						case 7: sprintf (pName, "Up Button"); break;
						case 8: sprintf (pName, "Right Button"); break;
						case 9: sprintf (pName, "Down Button"); break;
						case 10: sprintf (pName, "Left Button"); break;
						case 11: sprintf (pName, "C Button"); break;
						case 12: sprintf (pName, "B Button [Select]"); break;
						case 13: sprintf (pName, "A Button [Start]"); break;
						case 14: sprintf (pName, "F Button"); break;
						case 15: sprintf (pName, "R1 Trigger"); break;
						case 16: sprintf (pName, "R2 Trigger"); break;
						case 17: sprintf (pName, "L1 Trigger"); break;
						case 18: sprintf (pName, "L2 Trigger"); break;
						case 19: sprintf (pName, "Left Stick Button"); break;
						case 20: sprintf (pName, "Right Stick Button"); break;
						case 21: sprintf (pName, "D Button"); break;
						case 22: sprintf (pName, "E Button"); break;
						case 23: sprintf (pName, "Left Stick X-Axis"); break;
						case 24: sprintf (pName, "Left Stick Y-Axis"); break;
						case 25: sprintf (pName, "Right Stick X-Axis"); break;
						case 26: sprintf (pName, "Right Stick Y-Axis"); break;
					}
					break;
			}
			break;
	}
#endif
}
