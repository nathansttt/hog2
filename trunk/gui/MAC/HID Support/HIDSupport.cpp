/*
 * $Id: HIDSupport.cpp,v 1.3 2006/10/18 23:53:10 nathanst Exp $
 *
 *  HIDSupport.c
 *  Carbon OpenGL
 *
 *  Created by Geoff Stahl on Sat May 03 2003.
 *  Copyright (c) 2003 Apple. All rights reserved.

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

#include <IOKit/hid/IOHIDLib.h>
#include <IOKit/hid/IOHIDUsageTables.h>

#include "HIDSupport.h"
#include "trackball.h"
#include "common.h"

EventLoopTimerRef gHIDTimer = NULL; // input timer
static actionRec gActionArray [kNumActions]; // array of action records for mapping and values

// ---------------------------------

// internal routines

static void GetInput (void);
pascal void HIDTimer (EventLoopTimerRef inTimer, void* userData);
EventLoopTimerUPP GetHIDTimerUPP (void);
static void InitHIDInputArray (void);
static Boolean GetInputElements (pRecDevice pDevice);
static Boolean SetupHIDInputs (void);

// ---------------------------------

// polls for input for all actions based on current mapping

static void GetInput (void)
{
    short a;
    for (a = 0; a < kNumActions; a++) {
		if (gActionArray [a].pDevice && gActionArray [a].pElement) {
			gActionArray [a].value = HIDGetElementValue (gActionArray [a].pDevice, gActionArray [a].pElement);
//			gActionArray [a].value = HIDCalibrateValue (gActionArray [a].value, gActionArray [a].pElement);
			gActionArray [a].value = HIDScaleValue (gActionArray [a].value, gActionArray [a].pElement);
		}
		else
			gActionArray [a].value = 0;
    }
}

// ---------------------------------

// get input and update window for each timer beat

pascal void HIDTimer (EventLoopTimerRef inTimer, void* )
{
    #pragma unused (inTimer)
	static AbsoluteTime time = {0, 0};
	long				deadZone = 25;
   pRecContext			pContextInfo = NULL;
//	float				rotation[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    AbsoluteTime currTime = UpTime ();
    float deltaTime = (float) AbsoluteDeltaToDuration (currTime, time);
	time = currTime;	// reset for next time interval
    if (0 > deltaTime)	// if negative microseconds
		deltaTime /= -1000000.0;
    else				// else milliseconds
		deltaTime /= 1000.0;
    if (deltaTime > 10.0) // skip pauses
        return;

    // get input values
    GetInput ();
	// apply to front window
	pContextInfo = GetCurrentContextInfo(FrontWindow()); // call back to main to get the current context info record
	if (!pContextInfo)
	{
		return; // not an application window so do not process
	}
	// apply input
	double panX=0, panY=0;
	if (gActionArray [kActionXAxis].pDevice && gActionArray [kActionXAxis].pElement && (abs (gActionArray [kActionXAxis].value) > deadZone)) {
		// pan
		panX = gActionArray [kActionXAxis].value;
		//panX = abs (gActionArray [kActionXAxis].value) * gActionArray [kActionXAxis].value * 0.002f;
		//panX = abs (gActionArray [kActionXAxis].value) * gActionArray [kActionXAxis].value * 0.002f / (1500.0f / -pContextInfo->camera.viewPos.z);
		//pContextInfo->camera.viewPos.x += panX;
	}
	if (gActionArray [kActionYAxis].pDevice && gActionArray [kActionYAxis].pElement && (abs (gActionArray [kActionYAxis].value) > deadZone)) {
		// pan
		panY = gActionArray [kActionYAxis].value;
		//panY = abs (gActionArray [kActionYAxis].value) * gActionArray [kActionYAxis].value * deltaTime * 0.002f;
		//panY = abs (gActionArray [kActionYAxis].value) * gActionArray [kActionYAxis].value * deltaTime * 0.002f / (1500.0f / -pContextInfo->camera.viewPos.z);
		//pContextInfo->camera.viewPos.y -= panY;
	}
	handleJoystickMovement(pContextInfo, panX, panY);

	//	doMouseDelta(panX, panY);
//	if (gActionArray [kActionZAxis].pDevice && gActionArray [kActionZAxis].pElement && (abs (gActionArray [kActionZAxis].value) > deadZone)) {
//		// dolly
//		GLfloat dolly = abs (gActionArray [kActionZAxis].value) * gActionArray [kActionZAxis].value * deltaTime * 0.002f * -pContextInfo->camera.viewPos.z / 500.0f;
//		pContextInfo->camera.viewPos.z += dolly;
//		if (pContextInfo->camera.viewPos.z == 0.0) // do not let z = 0.0
//			pContextInfo->camera.viewPos.z = 0.0001;
//	}
	// handle rotations about each respective axis
//	if (gActionArray [kActionXRot].pDevice && gActionArray [kActionXRot].pElement && (abs (gActionArray [kActionXRot].value) > deadZone)) {
//		rotation[0] = abs (gActionArray [kActionXRot].value) * -gActionArray [kActionXRot].value * deltaTime * 0.0003f;
//		rotation[1] = 1.0f;
//		rotation[2] = 0.0f;
//		rotation[3] = 0.0f;
//		addToRotationTrackball (rotation, pContextInfo->worldRotation);
//	}
//	if (gActionArray [kActionYRot].pDevice && gActionArray [kActionYRot].pElement && (abs (gActionArray [kActionYRot].value) > deadZone)) {
//		rotation[0] = abs (gActionArray [kActionYRot].value) * gActionArray [kActionYRot].value * deltaTime * 0.0003f;
//		rotation[1] = 0.0f;
//		rotation[2] = 1.0f;
//		rotation[3] = 0.0f;
//		addToRotationTrackball (rotation, pContextInfo->worldRotation);
//	}
//	if (gActionArray [kActionZRot].pDevice && gActionArray [kActionZRot].pElement && (abs (gActionArray [kActionZRot].value) > deadZone)) {
//		rotation[0] = abs (gActionArray [kActionZRot].value) * -gActionArray [kActionZRot].value * deltaTime * 0.0003f;
//		rotation[1] = 0.0f;
//		rotation[2] = 0.0f;
//		rotation[3] = 1.0f;
//		addToRotationTrackball (rotation, pContextInfo->worldRotation);
//	}
	// need to force draw here...
	{
		Rect rectPort;
		WindowRef window = FrontWindow ();
		GetWindowPortBounds (window, &rectPort);
		InvalWindowRect (window, &rectPort);
	}
}

// ---------------------------------

// make timer UPP

EventLoopTimerUPP GetHIDTimerUPP (void)
{
    static EventLoopTimerUPP	sTimerUPP = NULL;
    
    if (sTimerUPP == NULL)
			sTimerUPP = NewEventLoopTimerUPP (HIDTimer);
    
    return sTimerUPP;
}

// ---------------------------------

static void InitHIDInputArray (void)
{
	long i;
	for (i = 0; i < kNumActions; i++) {
		gActionArray [i].pElement = NULL;
		gActionArray [i].pDevice = NULL;
		gActionArray [i].value = 0;
	}
}

// ---------------------------------

static Boolean GetInputElements (pRecDevice pDevice)
{
	short i;
	pRecElement pElement = NULL;
   
	if (pDevice) { // if we have found a device set up elements
		// prefer correct elements, then try any
		// look for x axis	
		pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO); // get first element
		while (pElement) { // for each element
			if ((kHIDPage_GenericDesktop == pElement->usagePage) && (kHIDUsage_GD_X == pElement->usage)) { // if it is the x axis
				gActionArray [kActionXAxis].pDevice = pDevice;
				gActionArray [kActionXAxis].pElement = pElement;
				break;
			}
			pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); // get next element
		}
		// look for y axis
		pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO); // get first element
		while (pElement) { // for each element
			if ((kHIDPage_GenericDesktop == pElement->usagePage) && (kHIDUsage_GD_Y == pElement->usage)) { // if it is the y axis
				gActionArray [kActionYAxis].pDevice = pDevice;
				gActionArray [kActionYAxis].pElement = pElement;
				break;
			}
			pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); // get next element
		}
//		// look for z axis
//		pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO); // get first element
//		while (pElement) { // for each element
//			if ((kHIDPage_GenericDesktop == pElement->usagePage) && (kHIDUsage_GD_Z == pElement->usage)) { // if it is the y axis
//				gActionArray [kActionZAxis].pDevice = pDevice;
//				gActionArray [kActionZAxis].pElement = pElement;
//				break;
//			}
//			pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); // get next element
//		}
//		// look for x rotation
//		pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO); // get first element
//		while (pElement) { // for each element
//			if ((kHIDPage_GenericDesktop == pElement->usagePage) && (kHIDUsage_GD_Rx == pElement->usage)) { // if it is the x axis
//				gActionArray [kActionXRot].pDevice = pDevice;
//				gActionArray [kActionXRot].pElement = pElement;
//				break;
//			}
//			pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); // get next element
//		}
//		// look for y rotation
//		pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO); // get first element
//		while (pElement) { // for each element
//			if ((kHIDPage_GenericDesktop == pElement->usagePage) && (kHIDUsage_GD_Ry == pElement->usage)) { // if it is the y axis
//				gActionArray [kActionYRot].pDevice = pDevice;
//				gActionArray [kActionYRot].pElement = pElement;
//				break;
//			}
//			pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); // get next element
//		}
//		// look for z rotation
//		pElement = HIDGetFirstDeviceElement (pDevice, kHIDElementTypeIO); // get first element
//		while (pElement) { // for each element
//			if ((kHIDPage_GenericDesktop == pElement->usagePage) && (kHIDUsage_GD_Rz == pElement->usage)) { // if it is the y axis
//				gActionArray [kActionZRot].pDevice = pDevice;
//				gActionArray [kActionZRot].pElement = pElement;
//				break;
//			}
//			pElement = HIDGetNextDeviceElement (pElement, kHIDElementTypeIO); // get next element
//		}
	}
	// set device limits
	for (i = 0; i < kNumActions; i++)
	{
#if 0
		if (!gActionArray [i].pElement) { // check to ensure we have valid elements
			InitHIDInputArray ();
			return false;
		}
#endif
		if (gActionArray[i].pElement) {
			(gActionArray[i].pElement)->userMin = -100;
			(gActionArray[i].pElement)->userMax = 100;
		}
	}
	return true;
}

// ---------------------------------

static Boolean SetupHIDInputs (void)
{
	UInt32	aUsage = 0;
	UInt32	aUsagePage = 0;
	pRecDevice pDevice = NULL;
   
	InitHIDInputArray ();
	HIDUpdateDeviceList(&aUsagePage, &aUsage, 1); // will do the right thing the first time
    pDevice = HIDGetFirstDevice(); // get the first device
	while(pDevice) {
		printf("Device: %s, %s.\n", pDevice->manufacturer, pDevice->product);
		// if the device has 4 axis try to use it
		if ((pDevice->axis >= 3) && (GetInputElements (pDevice))) // if we can find the axis needed use else continue
			break;
		pDevice = HIDGetNextDevice(pDevice); // check next device
	}
	if (pDevice) // means we found a valid device
	{
		printf("Using Device: %s, %s.\n", pDevice->manufacturer, pDevice->product);
		return true;
	}
	else
		return false;
}

// ---------------------------------

void EndHIDInput (void)
{
	// remove timer
	if (gHIDTimer)
		RemoveEventLoopTimer (gHIDTimer);
	gHIDTimer = NULL;
	// dump array
	InitHIDInputArray ();
	// dump devices
	if (HIDHaveDeviceList())
		HIDReleaseDeviceList ();
}

// ---------------------------------

void StartHIDInput (void)
{   
	EndHIDInput (); // ensure we are overwriting an earlier set up
	// setup inputs
	if (SetupHIDInputs ())
		InstallEventLoopTimer (GetCurrentEventLoop(), 0, 0.01, GetHIDTimerUPP (), NULL, &gHIDTimer); // start timer 100 hz
	else
		printf ("No 2 DOF device found.\n");
}
