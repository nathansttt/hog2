/*
 * $Id: HID_Utilities_External.h,v 1.2 2006/10/18 23:53:10 nathanst Exp $
 *
 *  HID_Utilities_External.h
 *     External interface for HID Utilities, can be used with either library or source
 *     Check notes below for usage.  Some type casting is required so library is framework and carbon free
 *
 *  Created by ggs on Fri Apr 20 2001.
 
    Copyright:	Copyright © 2001 Apple Computer, Inc., All Rights Reserved

	Disclaimer:	IMPORTANT:  This Apple software is supplied to you by Apple Computer, Inc.
			("Apple") in consideration of your agreement to the following terms, and your
			use, installation, modification or redistribution of this Apple software
			constitutes acceptance of these terms.  If you do not agree with these terms,
			please do not use, install, modify or redistribute this Apple software.

			In consideration of your agreement to abide by the following terms, and subject
			to these terms, Apple grants you a personal, non-exclusive license, under Apple’s
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

#ifndef _HID_Utilities_External_h_
#define _HID_Utilities_External_h_

// ==================================

#ifdef __cplusplus
extern "C" {
#endif

// ==================================

//includes

#include <stdio.h>

// ==================================

// Device and Element Interfaces

enum HIDElementTypeMask
{
	kHIDElementTypeInput				= 1 << 1,
	kHIDElementTypeOutput            	= 1 << 2,
	kHIDElementTypeFeature           	= 1 << 3,
	kHIDElementTypeCollection        	= 1 << 4,
	kHIDElementTypeIO					= kHIDElementTypeInput | kHIDElementTypeOutput | kHIDElementTypeFeature,
	kHIDElementTypeAll					= kHIDElementTypeIO | kHIDElementTypeCollection
};
typedef enum HIDElementTypeMask HIDElementTypeMask;


struct recElement
{
    unsigned long type;						// the type defined by IOHIDElementType in IOHIDKeys.h
    long usagePage;							// usage page from IOUSBHIDParser.h which defines general usage
    long usage;								// usage within above page from IOUSBHIDParser.h which defines specific usage
    void * cookie;							// unique value (within device of specific vendorID and productID) which identifies element, will NOT change
    long min;								// reported min value possible
    long max;								// reported max value possible
    long scaledMin;							// reported scaled min value possible
    long scaledMax;							// reported scaled max value possible
    long size;								// size in bits of data return from element
    unsigned char relative;						// are reports relative to last report (deltas)
    unsigned char wrapping;						// does element wrap around (one value higher than max is min)
    unsigned char nonLinear;						// are the values reported non-linear relative to element movement
    unsigned char preferredState;					// does element have a preferred state (such as a button)
    unsigned char nullState;						// does element have null state
    long units;								// units value is reported in (not used very often)
    long unitExp;							// exponent for units (also not used very often)
    char name[256];							// name of element (c string)

// runtime variables
    long initialCenter; 					// center value at start up
    unsigned char  hasCenter; 				// whether or not to use center for calibration
    long minReport; 						// min returned value
    long maxReport; 						// max returned value (calibrate call)
    long userMin; 							// user set value to scale to (scale call)
    long userMax;							
    
	struct recElement * pPrevious;			// previous element (NULL at list head)
    struct recElement * pChild;				// next child (only of collections)
    struct recElement * pSibling;			// next sibling (for elements and collections)
	
};
typedef struct recElement recElement;
typedef recElement* pRecElement;

struct recDevice
{
    void * interface;						// interface to device, NULL = no interface
    void * queue;							// device queue, NULL = no queue
    char transport[256];					// device transport (c string)
    long vendorID;							// id for device vendor, unique across all devices
    long productID;							// id for particular product, unique across all of a vendors devices
    long version;							// version of product
    char manufacturer[256];					// name of manufacturer
    char product[256];						// name of product
    char serial[256];						// serial number of specific product, can be assumed unique across specific product or specific vendor (not used often)
    long locID;								// long representing location in USB (or other I/O) chain which device is pluged into, can identify specific device on machine
    long usage;								// usage page from IOUSBHID Parser.h which defines general usage
    long usagePage;							// usage within above page from IOUSBHID Parser.h which defines specific usage
    long totalElements;						// number of total elements (should be total of all elements on device including collections) (calculated, not reported by device)
	long features;							// number of elements of type kIOHIDElementTypeFeature
	long inputs;							// number of elements of type kIOHIDElementTypeInput_Misc or kIOHIDElementTypeInput_Button or kIOHIDElementTypeInput_Axis or kIOHIDElementTypeInput_ScanCodes
	long outputs;							// number of elements of type kIOHIDElementTypeOutput
	long collections;						// number of elements of type kIOHIDElementTypeCollection
    long axis;								// number of axis (calculated, not reported by device)
    long buttons;							// number of buttons (calculated, not reported by device)
    long hats;								// number of hat switches (calculated, not reported by device)
    long sliders;							// number of sliders (calculated, not reported by device)
    long dials;								// number of dials (calculated, not reported by device)
    long wheels;							// number of wheels (calculated, not reported by device)
    recElement* pListElements; 				// head of linked list of elements 
    struct recDevice* pNext; 				// next device
};
typedef struct recDevice recDevice;
typedef recDevice* pRecDevice;

// ==================================

// builds list of devices with elements (allocates memory and captures devices) 
// in which the devices could be of different types/usages
// list is allocated internally within HID Utilites and can be accessed via accessor functions
// structures within list are considered flat and user accessable, but not user modifiable
// can be called again to rebuild list to account for new devices (will do the right thing in case of disposing existing list)
// usagePage, usage are each a numDeviceTypes sized array of matching usage and usage pages
// returns true if succesful
Boolean HIDBuildMultiDeviceList (UInt32 *pUsagePage, UInt32 *pUsage, UInt32 numDeviceTypes);

// same as above but this uses a single usagePage and usage
Boolean HIDBuildDeviceList (UInt32 usagePage, UInt32 usage);

// updates the current device list for any new/removed devices
// if this is called before HIDBuildDeviceList the it functions like HIDBuildMultiDeviceList
// usagePage, usage are each a numDeviceTypes sized array of matching usage and usage pages
// returns true if successful which means if any device were added or removed (the device config changed)
Boolean HIDUpdateDeviceList (UInt32 *pUsagePage, UInt32 *pUsage, UInt32 numDeviceTypes);

// release list built by above function
// MUST be called prior to application exit to properly release devices
// if not called (or app crashes) devices can be recovered by pluging into different location in USB chain
void HIDReleaseDeviceList (void);

// does a device list exist
unsigned char HIDHaveDeviceList (void);

// how many HID devices have been found
// returns 0 if no device list exist
unsigned long HIDCountDevices (void);

// how many elements does a specific device have
// returns 0 if device is invalid or NULL
// uses mask of HIDElementTypeMask to restrict element found
// use kHIDElementTypeIO to get non-collection elements
unsigned long HIDCountDeviceElements (pRecDevice pDevice, HIDElementTypeMask typeMask);

// get the first device in the device list
// returns NULL if no list exists
pRecDevice HIDGetFirstDevice (void);

// get next device in list given current device as parameter
// returns NULL if end of list
pRecDevice HIDGetNextDevice (pRecDevice pDevice);

// get the first element of device passed in as parameter
// returns NULL if no list exists or device does not exists or is NULL
// uses mask of HIDElementTypeMask to restrict element found
// use kHIDElementTypeIO to get previous HIDGetFirstDeviceElement functionality
pRecElement HIDGetFirstDeviceElement (pRecDevice pDevice, HIDElementTypeMask typeMask);

// get next element of given device in list given current element as parameter
// will walk down each collection then to next element or collection (depthwise traverse)
// returns NULL if end of list
// uses mask of HIDElementTypeMask to restrict element found
// use kHIDElementTypeIO to get previous HIDGetNextDeviceElement functionality
pRecElement HIDGetNextDeviceElement (pRecElement pElement, HIDElementTypeMask typeMask);

// get previous element of given device in list given current element as parameter
// this walks directly up the tree to the top element and does not search at each level
// returns NULL if beginning of list
// uses mask of HIDElementTypeMask to restrict element found
// use kHIDElementTypeIO to get non-collection elements
pRecElement HIDGetPreviousDeviceElement (pRecElement pElement, HIDElementTypeMask typeMask);

// returns C string type name given a type enumeration passed in as parameter (see IOHIDKeys.h)
// returns empty string for invlid types
void HIDGetTypeName (unsigned long type, char * cstrName);

// returns C string usage given usage page and usage passed in as parameters (see IOUSBHIDParser.h)
// returns usage page and usage values in string form for unknown values
void HIDGetUsageName (long valueUsagePage, long valueUsage, char * cstrName);

// ==================================

// Element Event Queue and Value Interfaces

enum
{
    kDefaultUserMin = 0,					// default user min and max used for scaling
    kDefaultUserMax = 255
};

enum
{
    kDeviceQueueSize = 50	// this is wired kernel memory so should be set to as small as possible
							// but should account for the maximum possible events in the queue
							// USB updates will likely occur at 100 Hz so one must account for this rate of
							// if states change quickly (updates are only posted on state changes)
};

// ==================================

// queues specific element, performing any device queue set up required
unsigned long  HIDQueueElement (pRecDevice pDevice, pRecElement pElement);

// adds all elements to queue, performing any device queue set up required
unsigned long  HIDQueueDevice (pRecDevice pDevice);

// removes element for queue, if last element in queue will release queue and device
unsigned long  HIDDequeueElement (pRecDevice pDevice, pRecElement pElement);

// completely removes all elements from queue and releases queue and device
unsigned long  HIDDequeueDevice (pRecDevice pDevice);

// returns true if an event is avialable for the element and fills out *pHIDEvent structure, returns false otherwise
// pHIDEvent is a poiner to a IOHIDEventStruct, using void here for compatibility, users can cast a required
unsigned char HIDGetEvent (pRecDevice pDevice, void * pHIDEvent);

// returns current value for element, creating device interface as required, polling element
// Note: this DOES NOT release the inteface so applications must properly release devices via ReleaseHIDDeviceList
long HIDGetElementValue (pRecDevice pDevice, pRecElement pElement);

// returns calibrated value given raw value passed in
// calibrated value is equal to min and max values returned by HIDGetElementValue since device list built scaled to element reported min and max values
long HIDCalibrateValue (long value, pRecElement pElement);

// returns scaled value given raw value passed in
// scaled value is equal to current value assumed to be in the range of element reported min and max values scaled to user min and max scaled values
long HIDScaleValue (long value, pRecElement pElement);

// ==================================

// Conguration and Save Interfaces

enum
{
    kPercentMove = 10 // precent of overall range a element must move to register
};

struct recSaveHID
{
    long actionCookie;
    // device
		// need to add serial number when I have a test case
    long vendorID;
    long productID;
    long locID;
    long usage;
    long usagePage;
    // elements
    long usagePageE;
    long usageE;
	long minReport;
	long maxReport;
    void * cookie;
};
typedef struct recSaveHID recSaveHID;
typedef recSaveHID * pRecSaveHID;

// polls single device's elements for a change greater than kPercentMove.  Times out after given time
// returns 1 and pointer to element if found
// returns 0 and NULL for both parameters if not found
unsigned char HIDConfigureSingleDeviceAction (pRecDevice pDevice, pRecElement * ppElement, float timeout);

// polls all devices and elements for a change greater than kPercentMove.  Times out after given time
// returns true and pointer to device and element if found
// returns false and NULL for both parameters if not found
unsigned char HIDConfigureAction (pRecDevice * ppDevice, pRecElement * ppElement, float timeout);

// -- These are routines to use if the applcationwants HID Utilities to do the file handling --
// Note: the FILE * is a MachO posix FILE and will not likely work directly with MW MSL FILE * type.

// take input records, save required info
// assume file is open and at correct position.
void HIDSaveElementConfig (FILE * fileRef, pRecDevice pDevice, pRecElement pElement, long actionCookie);

// takes a file, reads one record (assume file position is correct and file is open)
// search for matching device
// return pDevice, pElement and cookie for action
long HIDRestoreElementConfig (FILE * fileRef, pRecDevice * ppDevice, pRecElement * ppElement);

// -- These are routines to use if the client wants to use their own file handling -- 

// Set up a config record for saving
// takes an input records, returns record user can save as they want 
// Note: the save rec must be pre-allocated by the calling app and will be filled out
void HIDSetElementConfig (pRecSaveHID pConfigRec, pRecDevice pDevice, pRecElement pElement, long actionCookie);

// Get matching element from config record
// takes a pre-allocated and filled out config record
// search for matching device
// return pDevice, pElement and cookie for action
long HIDGetElementConfig (pRecSaveHID pConfigRec, pRecDevice * ppDevice, pRecElement * ppElement);

// ==================================


#ifdef __cplusplus
}
#endif

#endif // _HID_Utilities_External_h_
