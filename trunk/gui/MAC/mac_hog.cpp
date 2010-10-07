/*
 * $Id: mac_main.cpp,v 1.28 2006/11/07 22:35:24 bulitko Exp $
 *
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#define kUseMultiSample 1

#include <Carbon/Carbon.h>
#include <Quicktime/QuickTime.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <OpenGL/glext.h>

#include "MacGlCheck.h"
#include "Trackball.h"
#include "Common.h"
#include "HIDSupport.h"
#include <vector>

void saveSimHistory(WindowRef window, pRecContext pContextInfo);
void openSimHistory();
OSStatus disposeGL(pRecContext pContextInfo);
OSStatus getSaveFSSpec(WindowRef parentWindow, CFStringRef title, CFStringRef action, CFStringRef file, CFStringRef message, FSSpec &fileRef);

// ==================================

// single set of interaction flags and states
GLint gDollyPanStartPoint[2] = {0, 0};
GLfloat gTrackBallRotation [4] = {0.0f, 0.0f, 0.0f, 0.0f};
GLboolean gDolly = GL_FALSE;
GLboolean gPan = GL_FALSE;
GLboolean gTrackball = GL_FALSE;
pRecContext gTrackingContextInfo = NULL;

EventHandlerUPP gEvtHandler;			// main event handler
EventHandlerUPP gWinEvtHandler;			// window event handler

IBNibRef nibRef = NULL;

AbsoluteTime gStartTime;
static bool recording = false;

char gErrorMessage[256] = ""; // buffer for error message output
float gErrorTime = 0.0;

#define kOneSecond            600

void exportMovie();

void startRecording()
{
	recording = true;
}

void stopRecording()
{
	recording = false;
	exportMovie();
}


pRecContext GetContext(unsigned int windowID)
{
	WindowRef f = FrontWindow();
	pRecContext pContextInfo = (pRecContext) GetWRefCon(f);
	while (1)
	{
		if (pContextInfo && (pContextInfo->windowID == windowID))
			return pContextInfo;
		f = GetNextWindow (f);
		if (f == 0)
			break;
		pContextInfo = (pRecContext) GetWRefCon(f);
	}
	return 0;
}

pRecContext getCurrentContext()
{
	WindowRef f = FrontWindow();
	pRecContext pContextInfo = (pRecContext) GetWRefCon(f);
	return pContextInfo;
}

#pragma mark ---- OpenGL Capabilities ----

// GL configuration info globals
// see glcheck.h for more info
GLCaps * gDisplayCaps = NULL; // array of GLCaps
CGDisplayCount gNumDisplays = 0;
// related DM change notification:
DMExtendedNotificationUPP gConfigEDMUPP = NULL;

static void getCurrentCaps(void)
{
 	// Check for existing opengl caps here
	// This can be called again with same display caps array when display configurations are changed and
	//   your info needs to be updated.  Note, if you are doing dynmaic allocation, the number of displays
	//   may change and thus you should always reallocate your display caps array.
	if (gDisplayCaps && HaveOpenGLCapsChanged (gDisplayCaps, gNumDisplays)) { // see if caps have changed
		free (gDisplayCaps);
		gDisplayCaps = NULL;
	}
	if (!gDisplayCaps) { // if we do not have caps
		CheckOpenGLCaps (0, NULL, &gNumDisplays); // will just update number of displays
		gDisplayCaps = (GLCaps*) malloc (sizeof (GLCaps) * gNumDisplays);
		CheckOpenGLCaps (gNumDisplays, gDisplayCaps, &gNumDisplays);
	}
}

#pragma mark ---- Utilities ----

// return float elpased time in seconds since app start
static float GetElapsedTime(void)
{	
	float deltaTime = (float) AbsoluteDeltaToDuration (UpTime(), gStartTime);
    if (0 > deltaTime)	// if negative microseconds
		deltaTime /= -1000000.0;
    else				// else milliseconds
		deltaTime /= 1000.0;
	return deltaTime;
}

#pragma mark ---- Error Reporting ----

// C string to Pascal string
static void cstr2pstr(StringPtr outString, const char *inString)
{	
	unsigned short x = 0;
	do {
	    outString [x + 1] = (unsigned char) inString [x];
		x++;
	} while ((inString [x] != 0)  && (x < 256));
	outString [0] = x;									
}

// ---------------------------------

// error reporting as both window message and debugger string
void reportError(char * strError)
{
	Str255 strErr = "\p";

	gErrorTime = GetElapsedTime ();
	sprintf (gErrorMessage, "Error: %s (at time: %0.1f secs)", strError, gErrorTime);
	 
	// out as debug string
	cstr2pstr (strErr, gErrorMessage);
	DebugStr (strErr);
}

// ---------------------------------

// if error dump agl errors to debugger string, return error
OSStatus aglReportError(void)
{
	GLenum err = aglGetError();
	if (AGL_NO_ERROR != err)
		reportError ((char *) aglErrorString(err));
	// ensure we are returning an OSStatus noErr if no error condition
	if (err == AGL_NO_ERROR)
		return noErr;
	else
		return (OSStatus) err;
}

// ---------------------------------

// if error dump gl errors to debugger string, return error
OSStatus glReportError(void)
{
	GLenum err = glGetError();
	if (GL_NO_ERROR != err)
		reportError ((char *) gluErrorString (err));
	// ensure we are returning an OSStatus noErr if no error condition
	if (err == GL_NO_ERROR)
		return noErr;
	else
		return (OSStatus) err;
}

#pragma mark ---- OpenGL Utilities ----

// C string drawing function
void drawCStringGL(char * cstrOut, GLuint fontList)
{
	GLint i = 0;
	while (cstrOut [i])
		glCallList (fontList + cstrOut[i++]);
}

// ---------------------------------

// AGL bitmpp font setup
GLuint buildFontGL(AGLContext ctx, GLint fontID, Style face, GLint size)
{
	GLuint listBase = glGenLists (256);
	if (aglUseFont(ctx, fontID , face, size, 0, 256, (long) listBase)) {
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		glReportError();
		return listBase;
	} else {
		reportError("aglUseFont failed\n" );
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		glDeleteLists(listBase, 256);
		return 0;
	}
}

// ---------------------------------

// delete font list passed in
void deleteFontGL(GLuint fontList)
{
	if (fontList)
		glDeleteLists (fontList, 256);
}

// ---------------------------------

// given a delta time in seconds and current roatation accel, velocity and position, update overall object rotation
//void updateRotation(double deltaTime, GLfloat * fRot, GLfloat * fVel, GLfloat * fAccel, GLfloat * objectRotation )
//{
//	// update rotation based on vel and accel
//	float rotation[4] = {0.0f, 0.0f, 0.0f, 0.0f};
//	GLfloat fVMax = 2.0f;
//	short i;
//	// do velocities
//	for (i = 0; i < 3; i++) {
//		fVel[i] += fAccel[i] * deltaTime * 30.0f;
//		
//		if (fVel[i] > fVMax) {
//			fAccel[i] *= -1.0f;
//			fVel[i] = fVMax;
//		} else if (fVel[i] < -fVMax) {
//			fAccel[i] *= -1.0f;
//			fVel[i] = -fVMax;
//		}
//		
//		fRot[i] += fVel[i] * deltaTime * 30.0f;
//		
//		while (fRot[i] > 360.0f)
//			fRot[i] -= 360.0f;
//		while (fRot[i] < -360.0f)
//			fRot[i] += 360.0f;
//	}
//	rotation[0] = fRot[0];
//	rotation[1] = 1.0f;
//	addToRotationTrackball (rotation, objectRotation);
//	rotation[0] = fRot[1];
//	rotation[1] = 0.0f; rotation[2] = 1.0f;
//	addToRotationTrackball (rotation, objectRotation);
//	rotation[0] = fRot[2];
//	rotation[2] = 0.0f; rotation[3] = 1.0f;
//	addToRotationTrackball (rotation, objectRotation);
//}

// ---------------------------------

// update the projection matrix based on camera and view info
// should be called when viewport size, eye z position, or camera aperture changes
// also call if far or near changes which is determined by shape size in this case
void updateProjection(pRecContext pContextInfo, int viewPort)
{
	GLdouble ratio, radians, wd2;
	int minVal, maxVal;
	if (viewPort == -1)
	{
		minVal = 0;
		maxVal = pContextInfo->numPorts-1;
	}
	else {
		minVal = maxVal = viewPort;
	}
	for (int x = minVal; x <= maxVal; x++)
	{
		pContextInfo->frust[x].near = -pContextInfo->camera[x].viewPos.z - pContextInfo->shapeSize * 0.5;
		pContextInfo->frust[x].far = -pContextInfo->camera[x].viewPos.z + pContextInfo->shapeSize * 0.5;
		if (pContextInfo->frust[x].far < 4.0)
		{
			pContextInfo->frust[x].far = sqrt(pContextInfo->camera[x].viewDir.x*pContextInfo->camera[x].viewDir.x+
																				pContextInfo->camera[x].viewDir.y*pContextInfo->camera[x].viewDir.y+
																				pContextInfo->camera[x].viewDir.z+pContextInfo->camera[x].viewDir.z);
			pContextInfo->frust[x].far *= 2;
		}
		if (pContextInfo->frust[x].near < 1.0)
			pContextInfo->frust[x].near = 0.125;
		
		radians = 0.0174532925 * pContextInfo->camera[x].aperture / 2; // half aperture degrees to radians 
		wd2 = pContextInfo->frust[x].near * tan(radians);
		ratio = pContextInfo->camera[x].viewWidth / (float) pContextInfo->camera[x].viewHeight;
		if (ratio >= 1.0) {
			pContextInfo->frust[x].left  = -ratio * wd2;
			pContextInfo->frust[x].right = ratio * wd2;
			pContextInfo->frust[x].top = wd2;
			pContextInfo->frust[x].bottom = -wd2;	
		} else {
			pContextInfo->frust[x].left  = -wd2;
			pContextInfo->frust[x].right = wd2;
			pContextInfo->frust[x].top = wd2 / ratio;
			pContextInfo->frust[x].bottom = -wd2 / ratio;
		}
	}
}

// ---------------------------------

// updates the contexts model view matrix for object and camera moves
// we will call this every draw loop for simplicity
void updateModelView(pRecContext pContextInfo, int currPort)
{
	aglSetCurrentContext (pContextInfo->aglContext);
	
	// move view
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt (pContextInfo->camera[currPort].viewPos.x, pContextInfo->camera[currPort].viewPos.y, pContextInfo->camera[currPort].viewPos.z,
			   pContextInfo->camera[currPort].viewPos.x + pContextInfo->camera[currPort].viewDir.x,
			   pContextInfo->camera[currPort].viewPos.y + pContextInfo->camera[currPort].viewDir.y,
			   pContextInfo->camera[currPort].viewPos.z + pContextInfo->camera[currPort].viewDir.z,
			   pContextInfo->camera[currPort].viewUp.x, pContextInfo->camera[currPort].viewUp.y ,pContextInfo->camera[currPort].viewUp.z);

	if ((gTrackingContextInfo == pContextInfo) && gTrackBallRotation[0] != 0.0f) // if we have trackball rotation to map (this IS the test I want as it can be explicitly 0.0f)
	{
		if (pContextInfo->currPort == currPort)
			glRotatef (gTrackBallRotation[0], gTrackBallRotation[1], gTrackBallRotation[2], gTrackBallRotation[3]);
	}
	else {
	}
	// accumlated world rotation via trackball
	glRotatef (pContextInfo->rotations[currPort].worldRotation[0],
						 pContextInfo->rotations[currPort].worldRotation[1],
						 pContextInfo->rotations[currPort].worldRotation[2],
						 pContextInfo->rotations[currPort].worldRotation[3]);
	// object itself rotating applied after camera rotation
	glRotatef (pContextInfo->rotations[currPort].objectRotation[0],
						 pContextInfo->rotations[currPort].objectRotation[1],
						 pContextInfo->rotations[currPort].objectRotation[2],
						 pContextInfo->rotations[currPort].objectRotation[3]);
	
//	pContextInfo->fRot[0] = 0.0f; // reset animation rotations (do in all cases to prevent rotating while moving with trackball)
//	pContextInfo->fRot[1] = 0.0f;
//	pContextInfo->fRot[2] = 0.0f;
}

// ---------------------------------

// handles resizing of GL need context update and if the window dimensions change, a
// a window dimension update, reseting of viewport and an update of the projection matrix

OSStatus resizeGL(pRecContext pContextInfo, CGRect viewRect)
{
    OSStatus err = noErr;

    if (!pContextInfo || !(pContextInfo->aglContext))
        return paramErr;

    if (!aglSetCurrentContext (pContextInfo->aglContext)) err = aglReportError ();
    if (!aglUpdateContext (pContextInfo->aglContext)) err = aglReportError ();

		pContextInfo->globalCamera.viewOriginX = viewRect.origin.x;
		pContextInfo->globalCamera.viewOriginY = viewRect.origin.y;
		
		pContextInfo->globalCamera.viewWidth = (GLint)viewRect.size.width;
		pContextInfo->globalCamera.viewHeight = (GLint)viewRect.size.height;

		for (int x = 0; x < pContextInfo->numPorts; x++)
		{
			setPortCamera(pContextInfo, x);
		}
//		glViewport(0, 0, pContextInfo->camera.viewWidth, pContextInfo->camera.viewHeight);

		updateProjection(pContextInfo);  // update projection matrix

    return err;
}

// ---------------------------------

// sets the camera data to initial conditions
//static void resetCamera(recCamera * pCamera)
//{
//   pCamera->aperture = 6.0;
//   pCamera->rotPoint = gOrigin;
//
//   pCamera->viewPos.x = 0.0;
//   pCamera->viewPos.y = 0.0;
//   pCamera->viewPos.z = -12.0;
//   pCamera->viewDir.x = -pCamera->viewPos.x; 
//   pCamera->viewDir.y = -pCamera->viewPos.y; 
//   pCamera->viewDir.z = -pCamera->viewPos.z;
//
//   pCamera->viewUp.x = 0;  
//   pCamera->viewUp.y = 1; 
//   pCamera->viewUp.z = 0;
//
//	// will be set in resize once the target view size and position is known
//	pCamera->viewOriginY = 0;
//	pCamera->viewOriginX = 0;
//	pCamera->viewHeight = 0;
//	pCamera->viewWidth = 0;
//}


#pragma mark ---- OpenGL Minimize Handler (thanks to Dan Herman) ----

void InvertGLImage(char *imageData, long imageSize, long rowBytes)
{
	long i, j;
	long numRows = 0;
	char *tBuffer = NULL;
	
	i = j = 0;
//	for (int x = 0; x < imageSize-4; x+=4)
//	{
//		imageData[x+2] = 0;
//	}
	
	tBuffer = (char*)malloc((size_t)imageSize);
	if (!tBuffer)
	{
		printf("Out of memory!");
		return;  // continue without flip
	}
	
	numRows = imageSize / rowBytes;
	
	// Copy rows into tmp buffer one at a time, reversing their order
	for (i = 0, j = imageSize - rowBytes; i < imageSize; i += rowBytes, j -= rowBytes)
		memcpy( &tBuffer[j], &imageData[i], (size_t)rowBytes );
		
	// Copy tmp buffer back into original buffer
	memcpy(imageData, tBuffer, (size_t)imageSize);
	free(tBuffer);
}

// ---------------------------------

void CompositeGLBufferIntoFile(AGLContext ctx, Rect *bufferRect, const FSSpec *fileSpec)
{
	GWorldPtr pGWorld;
	QDErr err;
	// blit OpenGL content into window backing store
	// allocate buffer to hold pane image
	long width  =(bufferRect->right - bufferRect->left);
	long height  =(bufferRect->bottom - bufferRect->top);
	Rect src_rect = {0, 0, height, width};
	long rowBytes = width * 4;
	long imageSize = rowBytes * height;
	char *image =(char *) NewPtr(imageSize);
	if (!image) {
		printf("Out of memory in CompositeGLBufferIntoFile()!");
		return;		// no harm in continuing
	}
	
	// pull GL content down to our image buffer
	aglSetCurrentContext( ctx );
	glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image);
	
	// GL buffers are upside-down relative to QD buffers, so we need to flip it
	InvertGLImage( image, imageSize, rowBytes );
	
	// create a GWorld containing our image
	err = NewGWorldFromPtr(&pGWorld, k32ARGBPixelFormat, &src_rect, 0, 0, 0, image, rowBytes);
	if (err != noErr) {
		printf("WARNING: error in NewGWorldFromPtr, called from CompositeGLBufferIntoFile()");
		free( image );
		return;
	}
	
//	void writeGWorldToImageFile( GWorldPtr gw, const FSSpec *fileSpec )
//	{
	GraphicsExportComponent ge = 0;
	//err = FSpCreate(fileSpec, 'xxxx', 'PNG ', smSystemScript);
	err = FSpCreate(fileSpec, 'xxxx', 'JPEG', smSystemScript);
	if (err != 0) printf("Error %d creating1\n", (int)err);
	//err = OpenADefaultComponent( GraphicsExporterComponentType, kQTFileTypePNG, &ge );
	err = OpenADefaultComponent( GraphicsExporterComponentType, kQTFileTypeJPEG, &ge );
	if (err != 0) printf("Error %d exporting1\n", (int)err);
	err = GraphicsExportSetInputGWorld( ge, pGWorld );
	if (err != 0) printf("Error %d exporting2\n", (int)err);
  err = GraphicsExportSetOutputFile( ge, fileSpec );
	if (err != 0) printf("Error %d exporting3\n", (int)err);
	err = GraphicsExportSetCompressionQuality( ge, codecLosslessQuality);//codecMaxQuality);
	if (err != 0) printf("Error %d exporting4\n", (int)err);
   err = GraphicsExportDoExport( ge, nil );
	if (err != 0) printf("Error %d exporting5\n", (int)err);
   err = CloseComponent( ge );
	if (err != 0) printf("Error %d exporting6\n", (int)err);
//	}
	//	SetPort( GetWindowPort(win));
//	CopyBits( GetPortBitMapForCopyBits(pGWorld), GetPortBitMapForCopyBits(GetWindowPort(win)), &src_rect, bufferRect, srcCopy, 0 );
	DisposeGWorld( pGWorld );
	DisposePtr( image );
}

void CompositeGLBufferIntoWindow(AGLContext ctx, Rect *bufferRect, WindowRef win)
{
	GWorldPtr pGWorld;
	QDErr err;
	// blit OpenGL content into window backing store
	// allocate buffer to hold pane image
	long width  =(bufferRect->right - bufferRect->left);
	long height  =(bufferRect->bottom - bufferRect->top);
	Rect src_rect = {0, 0, height, width};
	long rowBytes = width * 4;
	long imageSize = rowBytes * height;
	char *image =(char *) NewPtr(imageSize);
	if (!image) {
		printf("Out of memory in CompositeGLBufferIntoWindow()!");
		return;		// no harm in continuing
	}
	
	// pull GL content down to our image buffer
	aglSetCurrentContext( ctx );
	glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image);
	
	// GL buffers are upside-down relative to QD buffers, so we need to flip it
	InvertGLImage( image, imageSize, rowBytes );
	
	// create a GWorld containing our image
	err = NewGWorldFromPtr(&pGWorld, k32ARGBPixelFormat, &src_rect, 0, 0, 0, image, rowBytes);
	if (err != noErr) {
		printf("WARNING: error in NewGWorldFromPtr, called from CompositeGLBufferIntoWindow()");
		free( image );
		return;
	}
	
	SetPort( GetWindowPort(win));
	CopyBits( GetPortBitMapForCopyBits(pGWorld), GetPortBitMapForCopyBits(GetWindowPort(win)), &src_rect, bufferRect, srcCopy, 0 );
	DisposeGWorld( pGWorld );
	DisposePtr( image );
}

class movieFrame {
public:
	movieFrame(char *im, int w, int h, double dur)
		:image(im), width(w), height(h), duration(dur) { }
	~movieFrame() { DisposePtr(image); }
	char *image;
	int width, height;
	double duration;
};

std::vector<movieFrame *> frames;

// MacErrors.h
void captureNextMovieFrame(pRecContext pContextInfo, WindowRef win, double duration)
{
	// blit OpenGL content into window backing store
	// allocate buffer to hold pane image
	Rect rectPort;
	GetWindowPortBounds(win, &rectPort);
	long width  =(rectPort.right - rectPort.left);
	long height  =(rectPort.bottom - rectPort.top);
	long rowBytes = width * 4;
	long imageSize = rowBytes * height;
	char *image = 0;

	image = (char *) NewPtr(imageSize);
	if (!image) {
		printf("Out of memory in CompositeGLBufferIntoWindow()!");
		return;		// no harm in continuing
	}
	AGLContext ctx = pContextInfo->aglContext;
	
	// pull GL content down to our image buffer
	aglSetCurrentContext( ctx );
	glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image);
	//glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image);
	
	frames.push_back(new movieFrame(image, width, height, duration));
}

Movie theMovie = NULL;
Track theSourceTrack = NULL;
Media movieMedia = NULL;
short resRefNum;

void addFrameToMovie(unsigned int which)
{
	OSStatus err;
	GWorldPtr movieGWorld;
	PixMapHandle moviePixMap = NULL;
	CGrafPtr mySavedPort = NULL;
	GDHandle mySavedGDevice = NULL;
	ImageDescriptionHandle myDesc = NULL;
	Handle myData = NULL;
	long mySize;
		
	long rowBytes = frames[which]->width * 4;
	long imageSize = rowBytes * frames[which]->height;
	Rect src_rect = { 0, 0, frames[which]->height, frames[which]->width };
	InvertGLImage( frames[which]->image, imageSize, rowBytes );
	
	// get the current port and device
	GetGWorld(&mySavedPort, &mySavedGDevice);
	
	// create a GWorld containing our image
	err = NewGWorldFromPtr(&movieGWorld, k32ARGBPixelFormat, &src_rect, 0, 0, 0, frames[which]->image, rowBytes);
	
	moviePixMap = GetGWorldPixMap(movieGWorld);
	LockPixels(moviePixMap);
	
	myDesc = (ImageDescriptionHandle)NewHandle(sizeof(ImageDescription));
	err = GetMaxCompressionSize(moviePixMap, &src_rect, 0, codecNormalQuality, kAnimationCodecType, anyCodec, &mySize);
	if (err) printf("maxcompression error: %d\n", (int)err);
		
	myData = NewHandle(mySize);
	if (err) printf("newhandle error: %d\n", (int)err);
	
	HLockHi(myData);
	
	SetGWorld(movieGWorld, 0);
	
	err = CompressImage(moviePixMap, &src_rect, codecNormalQuality, kAnimationCodecType, myDesc, *myData);
	if (err) printf("compressImage error: %d\n", (int)err);
	
	err = AddMediaSample(movieMedia, myData, 0, (**myDesc).dataSize, (long)(kOneSecond*frames[which]->duration), (SampleDescriptionHandle)myDesc, 1, 0, NULL);
	if (err) printf("addMediaSample error: %d\n", (int)err);
	
	// restore the original port and device
	SetGWorld(mySavedPort, mySavedGDevice);
	
	if (myData != NULL) {
		HUnlock(myData);
		DisposeHandle(myData);
	}
	
	if (myDesc) DisposeHandle((Handle)myDesc);
	if (moviePixMap) UnlockPixels(moviePixMap);
	if (movieGWorld) DisposeGWorld( movieGWorld );
	delete frames[which];
	frames[which] = 0;
}

void exportMovie() //startRecordingMovie(pRecContext pContextInfo, WindowRef win)
{
	if (frames.size() == 0)
		return;
	long width = frames[0]->width;
	long height = frames[0]->height;
	for (unsigned int x = 1; x < frames.size(); x++)
	{
		if (frames[x]->width > width)
			width = frames[x]->width;
		if (frames[x]->height > height)
			height = frames[x]->height;
	}
	WindowRef window;
	ControlID loadID = { 'LOAD', 128 };
	ControlRef loadField;
	CreateWindowFromNib(nibRef, CFSTR("LoadingWindow"), &window); // build window
	GetControlByID(window, &loadID, &loadField);
	ShowWindow(window);
	//	FSSpec  movieFSSpec;
	OSStatus err;
	EnterMovies();
//	Rect rectPort = {0, 0, height, width};
	FSRef fref;
	FSSpec movieFSSpec;
	FSFindFolder(kOnAppropriateDisk,kTemporaryFolderType, kCreateFolder, &fref);
	err = FSGetCatalogInfo (&fref,0,0,0,&movieFSSpec,0);
	if (err) printf("FSGetCatalogInfo error: %d\n", (int)err);
	err = FSMakeFSSpec(movieFSSpec.vRefNum, movieFSSpec.parID, "\ptempHogMovie", &movieFSSpec);
	if (err) printf("FSMakeFSSpec error: %d\n", (int)err);
	err = CreateMovieFile (&movieFSSpec, 
													 'TVOD',
													 smCurrentScript, 
													 createMovieFileDeleteCurFile | createMovieFileDontCreateResFile,
													 &resRefNum, 
													 &theMovie);
	if (err) printf("CreateMovieFile error: %d\n", (int)err);

	//theMovie = NewMovie(0); // ??
	
	// create a video track in the movie
	theSourceTrack = NewMovieTrack(theMovie, FixRatio(width, 1), FixRatio(height, 1), kNoVolume);
	movieMedia = NewTrackMedia(theSourceTrack, VideoMediaType, kOneSecond, NULL, 0); //  kAnimationCodecType

	// begin editing the new track
	BeginMediaEdits(movieMedia);
	
	SetControl32BitMaximum(loadField, 1000);
	for (unsigned int x = 0; x < frames.size(); x++)
	{
		SetControl32BitValue(loadField, (1000*x)/frames.size());
		Draw1Control(loadField);
		QDFlushPortBuffer(GetWindowPort(window), 0);
		// printf("Exporting %1.2f%% Done\n", (double)x/frames.size());
		addFrameToMovie(x);
	}
	err = EndMediaEdits(movieMedia);
	if (err) printf("endMediaEdits error: %d\n", (int)err);
		
	err = InsertMediaIntoTrack(theSourceTrack, 0/* start time*/, 0, GetMediaDuration(movieMedia), fixed1);
	
	short resId = movieInDataForkResID;
	err = AddMovieResource(theMovie, resRefNum, &resId, "\pHOG Movie");
	if (err) printf("AddMovieResource error: %d\n", (int)err);
	if (resRefNum)
	{
		CloseMovieFile(resRefNum);
	}
	DisposeWindow(window);
	SetMovieProgressProc(theMovie, (MovieProgressUPP)-1L, 0);
	err = ConvertMovieToFile (theMovie,     /* identifies movie */
		0,                /* all tracks */
		0,                /* no output file */
		0,                  /* no file type */
		0,                  /* no creator */
		-1,                 /* script */
		0,                /* no resource ID */
		createMovieFileDeleteCurFile |
		showUserSettingsDialog |
		movieToFileOnlyExport, 
		0);                 /* no specific component */
	if (err) printf("ConvertMovieToFile error: %d\n", (int)err);


	DisposeMovie(theMovie);
	err = FSpDelete(&movieFSSpec);
	if (err) printf("FSpDelete error: %d\n", (int)err);
	frames.resize(0);
}


//void recordNextFrame(pRecContext pContextInfo, WindowRef win, double duration)
//{
////static OSErr QTEffects_AddVideoTrackFromGWorld (Movie *theMovie, GWorldPtr theGW, Track *theSourceTrack, long theStartTime, short theWidth, short theHeight)
////{
//		ImageDescriptionHandle    myDesc = NULL;
//		Rect            myRect;
//		long            mySize;
//		Handle            myData = NULL;
//		Ptr              myDataPtr = NULL;
//		CGrafPtr           mySavedPort = NULL;
//		GDHandle           mySavedGDevice = NULL;
////		PicHandle          myHandle = NULL;
////		PixMapHandle        mySrcPixMap = NULL;
//		PixMapHandle        myDstPixMap = NULL;
//		OSErr            myErr = noErr;
//		GWorldPtr movieGWorld = NULL;
//		
//		// get the current port and device
//		GetGWorld(&mySavedPort, &mySavedGDevice);
//		
//		// get the rectangle for the movie
//		GetMovieBox(theMovie, &myRect);
//    
//		// create a new GWorld; we draw the picture into this GWorld and then compress it
//		// (note that we are creating a picture with the maximum bit depth)
////		FAIL_OSERR(myErr);
////		movieGWorld = getImageGWorld(&movieGWorld);
//		movieGWorld = captureMovieFrame(pContextInfo, win);
//
////		mySrcPixMap = GetGWorldPixMap(theGW);
////		// LockPixels(mySrcPixMap);
////		
//		myDstPixMap = GetGWorldPixMap(movieGWorld);
//		LockPixels(myDstPixMap);
//		
//		// create a new image description; CompressImage will fill in the fields of this structure
//		// myDesc = (ImageDescriptionHandle)NewHandle(4);
//		myDesc = (ImageDescriptionHandle)NewHandle(sizeof(ImageDescription));
//		
//		myErr = GetMaxCompressionSize(myDstPixMap, &myRect, 0, codecNormalQuality, kAnimationCodecType, anyCodec, &mySize);
//    if (myErr) printf("maxcompression error: %d\n", myErr);
//
//		myData = NewHandle(mySize);
//    if (myErr) printf("newhandle error: %d\n", myErr);
//    
//		HLockHi(myData);
//		myDataPtr = *myData;//StripAddress(*myData);
//		
//		SetGWorld(movieGWorld, 0);
//
//		myErr = CompressImage(myDstPixMap, &myRect, codecNormalQuality, kAnimationCodecType, myDesc, *myData);
//    if (myErr) printf("compressImage error: %d\n", myErr);
//    
//		myErr = AddMediaSample(movieMedia, myData, 0, (**myDesc).dataSize, (long)(kOneSecond*duration), (SampleDescriptionHandle)myDesc, 1, 0, NULL);
//    if (myErr) printf("addMediaSample error: %d\n", myErr);
//
//		// restore the original port and device
//		SetGWorld(mySavedPort, mySavedGDevice);
//		
//		if (myData != NULL) {
//			HUnlock(myData);
//			DisposeHandle(myData);
//		}
//		
//		if (myDesc != NULL)
//			DisposeHandle((Handle)myDesc);
//    
//		if (myDstPixMap != NULL)
//			UnlockPixels(myDstPixMap);
//
//		if (movieGWorld != NULL)
//			DisposeGWorld( movieGWorld );
//		
//		//return(myErr);
//}
//
//void stopRecordingMovie()
//{
//	OSStatus myErr;
//	myErr = EndMediaEdits(movieMedia);
//	if (myErr) printf("endMediaEdits error: %d\n", myErr);
//		
//	myErr = InsertMediaIntoTrack(theSourceTrack, 0/* start time*/, 0, GetMediaDuration(movieMedia), fixed1);
//
//	short resId = movieInDataForkResID;
//	myErr = AddMovieResource(theMovie, resRefNum, &resId, "\ptesting");
//	if (resRefNum)
//	{
//		CloseMovieFile (resRefNum);
//	}
//
//	theSourceTrack = NULL;
//}

#pragma mark ---- Display Manager Event Handling ----

// update our GL configuration info based on display change notification
void handleConfigDMEvent(void *, short theMessage, void *)
{
	if (kDMNotifyEvent == theMessage) { // post change notifications only
		getCurrentCaps ();
	}
}

// ---------------------------------

// handle display config changes meaing we need to update the GL context via the resize function and check for windwo dimension changes
// also note we redraw the content here as it could be lost in a display config change
void handleWindowDMEvent(void *userData, short theMessage, void *)
{
	if (kDMNotifyEvent == theMessage) { // post change notifications only
		pRecContext pContextInfo = NULL;
		WindowRef window = (WindowRef) userData;
		if (window)
			pContextInfo = (pRecContext) GetWRefCon (window);
		if (pContextInfo) { // have a valid OpenGl window
			Rect rectPort;
			CGRect viewRect = {{0.0f, 0.0f}, {0.0f, 0.0f}};
			sprintf (pContextInfo->message, "Event: Display Change at %0.1f secs", GetElapsedTime ());
			pContextInfo->msgTime = GetElapsedTime ();
			GetWindowPortBounds (window, &rectPort);
			viewRect.size.width = (float) (rectPort.right - rectPort.left);
			viewRect.size.height = (float) (rectPort.bottom - rectPort.top);
			resizeGL(GetCurrentContextInfo(window), viewRect); // update context and handle possible resizes
			InvalWindowRect (window,  &rectPort); // force redrow
		}
	}
}

#pragma mark ---- OpenGL setup and teardown ----

//void SetLighting(unsigned int mode)
//{
//	GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
//	GLfloat mat_shininess[] = {90.0};
//
//	GLfloat position[4] = {7.0,-7.0,12.0,0.0};
//	GLfloat ambient[4]  = {0.2,0.2,0.2,1.0};
//	GLfloat diffuse[4]  = {1.0,1.0,1.0,1.0};
//	GLfloat specular[4] = {1.0,1.0,1.0,1.0};
//	
//	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
//	glMaterialfv (GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
//	
//	glEnable(GL_COLOR_MATERIAL);
//	glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
//
//	switch (mode) {
//		case 0:
//			break;
//		case 1:
//			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_FALSE);
//			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_FALSE);
//			break;
//		case 2:
//			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_FALSE);
//			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
//			break;
//		case 3:
//			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
//			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_FALSE);
//			break;
//		case 4:
//			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
//			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
//			break;
//	}
//	
//	glLightfv(GL_LIGHT0,GL_POSITION,position);
//	glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
//	glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
//	glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
//	glEnable(GL_LIGHT0);
//}

// ---------------------------------

OSStatus buildGL(WindowRef window)
{
	OSStatus err = noErr;
	Rect rectPort;
#ifndef kUseMultiSample
	GLint attrib[] = { AGL_RGBA, AGL_DOUBLEBUFFER, AGL_DEPTH_SIZE, 16, AGL_NONE };
#else
	GLint attrib[] = { AGL_RGBA, AGL_DOUBLEBUFFER, AGL_DEPTH_SIZE, 16, AGL_SAMPLE_BUFFERS_ARB, 1, AGL_SAMPLES_ARB, kSamples, AGL_NO_RECOVERY, AGL_NONE };
#endif
    pRecContext pContextInfo = GetCurrentContextInfo(window);
	ProcessSerialNumber psn = { 0, kCurrentProcess };
    
    if (NULL == pContextInfo)
        return paramErr;
		
	// build context
	pContextInfo->aglContext = NULL;
	pContextInfo->aglPixFmt = aglChoosePixelFormat(NULL, 0, attrib);
	aglReportError ();
	if (pContextInfo->aglPixFmt) {
		pContextInfo->aglContext = aglCreateContext(pContextInfo->aglPixFmt, NULL);
		aglReportError ();
	}
	if (pContextInfo->aglContext) {
		short fNum;
		GLint swap = 1;
		CGRect viewRect = {{0.0f, 0.0f}, {0.0f, 0.0f}};

        GrafPtr portSave = NULL;
        GetPort (&portSave);
        SetPort ((GrafPtr) GetWindowPort (window));

		if (!aglSetDrawable(pContextInfo->aglContext, GetWindowPort (window)))
			err = aglReportError ();
		if (!aglSetCurrentContext(pContextInfo->aglContext))
			err = aglReportError ();

		// VBL SYNC
		if (!aglSetInteger (pContextInfo->aglContext, AGL_SWAP_INTERVAL, &swap))
			aglReportError ();

		switch (pContextInfo->modeFSAA) {
			case kFSAAOff:
				glDisable (GL_MULTISAMPLE_ARB);
				break;
			case kFSAAFast:
				glEnable (GL_MULTISAMPLE_ARB);
				glHint (GL_MULTISAMPLE_FILTER_HINT_NV, GL_FASTEST);
				break;
			case kFSAANice:
				glEnable (GL_MULTISAMPLE_ARB);
				glHint (GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);
				break;
		}
		
		// ensure we know when display configs are changed
		pContextInfo->windowEDMUPP = NewDMExtendedNotificationUPP (handleWindowDMEvent); // for display change notification
		//DMRegisterExtendedNotifyProc (pContextInfo->windowEDMUPP, (void *) window, NULL, &psn);
		DMRegisterExtendedNotifyProc (pContextInfo->windowEDMUPP, (void *) window, 0, &psn);

		// init GL stuff here
		glEnable(GL_DEPTH_TEST);
	
		glShadeModel(GL_SMOOTH);    
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		glFrontFace(GL_CCW);
		glPolygonOffset (1.0, 1.0);
		
		glClearColor(0.0,0.0,0.0,0.0);
		pContextInfo->shapeSize = 3.0f; // max radius of of objects

		GetFNum ("\pGeneva", &fNum); // build font
		pContextInfo->boldFontList = buildFontGL (pContextInfo->aglContext, fNum, bold, 9);
		pContextInfo->regFontList = buildFontGL (pContextInfo->aglContext, fNum, normal, 9);

		// setup viewport and prespective
		GetWindowPortBounds (window, &rectPort);
		viewRect.size.width = (float) (rectPort.right - rectPort.left);
		viewRect.size.height = (float) (rectPort.bottom - rectPort.top);

		// setup viewport and prespective
		resizeGL(pContextInfo, viewRect); // forces projection matrix update
		
		SetLighting(4);
		
		SetPort(portSave);
	}
	return err;
}

// ---------------------------------

// dump window data structures and OpenGL context and related data structures
OSStatus disposeGL(pRecContext pContextInfo)
{
	// release our data
	if ( pContextInfo != NULL )
	{
		if (pContextInfo->windowEDMUPP) { // dispose UPP for DM notifications
			DisposeDMExtendedNotificationUPP (pContextInfo->windowEDMUPP);
			pContextInfo->windowEDMUPP = NULL;
		}
		aglSetCurrentContext (NULL);
		aglSetDrawable (pContextInfo->aglContext, NULL);
		if (pContextInfo->aglContext) {
			aglDestroyContext (pContextInfo->aglContext);
			pContextInfo->aglContext = NULL;
		}
		if (pContextInfo->aglPixFmt) {
			aglDestroyPixelFormat (pContextInfo->aglPixFmt);
			pContextInfo->aglPixFmt = NULL;
		}
		if (pContextInfo->timer) {
			RemoveEventLoopTimer(pContextInfo->timer);
			pContextInfo->timer = NULL;
		}
		// dump font display list
		if (pContextInfo->boldFontList) {
			deleteFontGL (pContextInfo->boldFontList);
			pContextInfo->boldFontList = 0;
		}
		if (pContextInfo->regFontList) {
			deleteFontGL (pContextInfo->regFontList);
			pContextInfo->regFontList = 0;
		}
//		if (pContextInfo->unitLayer)
//		{
//			processStats(pContextInfo->unitLayer->getStats());
//			delete pContextInfo->unitLayer;
//			pContextInfo->unitLayer = 0;
//		}
	}
    
	return noErr;
}

#pragma mark ---- OpenGL Drawing ----

#include "drawInfo.h" // source info for info output


void appendTextToBuffer(char *tempStr)
{
	WindowRef window = FrontWindow();
	pRecContext pContextInfo = (pRecContext) GetWRefCon (window);
	if (pContextInfo)
	{
		pContextInfo->msgTime = GetElapsedTime();
		int ind = strlen(pContextInfo->message);
		pContextInfo->message[ind] = ' ';
		sprintf(&pContextInfo->message[ind+1], "%s", tempStr);
	}
}

// ---------------------------------
void submitTextToBuffer(const char *val)
{
	WindowRef window = FrontWindow();
	pRecContext pContextInfo = (pRecContext) GetWRefCon (window);
	if (pContextInfo)
	{
		pContextInfo->msgTime = GetElapsedTime();
		strncpy(pContextInfo->message, val, 255);
	}
}


// draw text info
// note: this bitmap technique is not the speediest and one should use textures for font in more proformance critical code
static void drawInfo(pRecContext )
{
//	static float msgPresistance = 240.0f;
//	char cstr [256];
//	GLint matrixMode, line = 1;
//	GLboolean depthTest = glIsEnabled (GL_DEPTH_TEST);
//	GLfloat height, width;
//	
//	if (!pContextInfo)
//		return;
//	
//	height = pContextInfo->camera.viewHeight;
//	width = pContextInfo->camera.viewWidth;
//
//	glDisable (GL_DEPTH_TEST); // ensure text is not remove by deoth buffer test.
//	glEnable (GL_BLEND); // for text fading
//	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // ditto
//	
//	// set orthograhic 1:1  pixel transform in local view coords
//	glGetIntegerv (GL_MATRIX_MODE, &matrixMode);
//	glMatrixMode (GL_PROJECTION);
//	glPushMatrix();
//	glLoadIdentity ();
//	glMatrixMode (GL_MODELVIEW);
//	glPushMatrix();
//	glLoadIdentity ();
//	glScalef (2.0 / width, -2.0 /  height, 1.0);
//	glTranslatef (-width / 2.0, -height / 2.0, 0.0);
//	// output strings
//	glColor3f (1.0, 1.0, 1.0);
//	sprintf (cstr, "Camera at (%0.1f, %0.1f, %0.1f) looking at (%0.1f, %0.1f, %0.1f) with %0.1f aperture", 
//	pContextInfo->camera.viewPos.x, pContextInfo->camera.viewPos.y, pContextInfo->camera.viewPos.z,
//	pContextInfo->camera.viewDir.x, pContextInfo->camera.viewDir.y, pContextInfo->camera.viewDir.z,
//	pContextInfo->camera.aperture);
//	glRasterPos3d (10, line++ * 12, 0); 
//	drawCStringGL (cstr, pContextInfo->boldFontList);
////	sprintf (cstr, "Trackball Rotation: (%0.1f, %0.2f, %0.2f, %0.2f)", gTrackBallRotation[0], gTrackBallRotation[1], gTrackBallRotation[2], gTrackBallRotation[3]);
////	glRasterPos3d (10, line++ * 12, 0); 
////	drawCStringGL (cstr, pContextInfo->regFontList);
////	sprintf (cstr, "World Rotation: (%0.1f, %0.2f, %0.2f, %0.2f)", pContextInfo->worldRotation[0], pContextInfo->worldRotation[1], pContextInfo->worldRotation[2], pContextInfo->worldRotation[3]);
////	glRasterPos3d (10, line++ * 12, 0); 
////	drawCStringGL (cstr, pContextInfo->regFontList);
////	sprintf (cstr, "Vertices: %ld, Color Scheme: %ld", pContextInfo->subdivisions * pContextInfo->xyRatio * pContextInfo->subdivisions, pContextInfo->colorScheme);
////	glRasterPos3d (10, line++ * 12, 0); 
////	drawCStringGL (cstr, pContextInfo->regFontList);
////	{
////		GLboolean twoSidedLighting, localViewer;
////		glGetBooleanv (GL_LIGHT_MODEL_LOCAL_VIEWER, &localViewer);
////		glGetBooleanv (GL_LIGHT_MODEL_TWO_SIDE, &twoSidedLighting);
////		if (!pContextInfo->lighting) {
////			sprintf (cstr, "-- Lighting off");
////		} else {
////			if (!twoSidedLighting)
////				sprintf (cstr, "-- Single Sided Lighting");
////			else
////				sprintf (cstr, "-- Two Sided Lighting");
////			if (localViewer)
////				sprintf (cstr, "%s: Local Viewer", cstr);
////		}	
////		glRasterPos3d (10, line++ * 12, 0); 
////		drawCStringGL (cstr, pContextInfo->regFontList);
////	}
//
////	glRasterPos3d (10, line++ * 12, 0); 
////#ifndef kUseMultiSample
////	sprintf (cstr, "-- FSAA: Off");
////#else
////	switch (pContextInfo->modeFSAA) {
////		case kFSAAOff:
////			sprintf (cstr, "-- %d x FSAA: Disabled", kSamples);
////			break;
////		case kFSAAFast:
////			sprintf (cstr, "-- %d x FSAA: Fastest hint", kSamples);
////			break;
////		case kFSAANice:
////			sprintf (cstr, "-- %d x FSAA: Nicest hint", kSamples);
////			break;
////	}
////#endif
////	drawCStringGL (cstr, pContextInfo->regFontList);
//
//		// message string
//	if (pContextInfo->message[0])
//	{
//		float currDelta = GetElapsedTime () - pContextInfo->msgTime;
//		glColor4f (1.0, 1.0, 1.0, (msgPresistance - currDelta) * 0.1);
//		glRasterPos3d (10, line++ * 12, 0); 
//		drawCStringGL (pContextInfo->message, pContextInfo->boldFontList);
//		if (currDelta > msgPresistance)
//			pContextInfo->message[0] = 0;
//	}
//	// global error message
//	if (gErrorMessage[0]) {
//		float currDelta = GetElapsedTime () - gErrorTime;
//		glColor4f (1.0, 0.2, 0.2, (msgPresistance - currDelta) * 0.1);
//		glRasterPos3d (10, line++ * 12, 0); 
//		drawCStringGL (gErrorMessage, pContextInfo->boldFontList);
//		if (currDelta > msgPresistance)
//			gErrorMessage[0] = 0;
//	}
////		if (pContextInfo->showCredits) {
//////				char *strName, *strAuthor, *strX, *strY, *strZ, *strRange;
//////				GetStrings(pContextInfo->surface, &strName, &strAuthor, &strX, &strY, &strZ, &strRange);
//////				line = 10;
//////				glColor3f (1.0f, 1.0f, 0.0f);
//////				glRasterPos3d (10, line++ * 12, 0); 
//////				drawCStringGL (strName, pContextInfo->boldFontList);
//////				glRasterPos3d (10, line++ * 12, 0); 
//////				drawCStringGL (strAuthor, pContextInfo->regFontList);
//////				glColor3f (0.7f, 0.7f, 0.0f);
//////				glRasterPos3d (10, line++ * 12, 0); 
//////				drawCStringGL (strX, pContextInfo->regFontList);
//////				glRasterPos3d (10, line++ * 12, 0); 
//////				drawCStringGL (strY, pContextInfo->regFontList);
//////				glRasterPos3d (10, line++ * 12, 0); 
//////				drawCStringGL (strZ, pContextInfo->regFontList);
//////				glRasterPos3d (10, line++ * 12, 0); 
//////				drawCStringGL (strRange, pContextInfo->regFontList);
////		}
////		if (pContextInfo->drawHelp) {
////			line = 4;
////			glColor3f (0.8f, 0.8f, 0.8f);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("\\: cycle surface type", pContextInfo->regFontList);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("; & ': decrease/increase subdivisions", pContextInfo->regFontList);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("[ & ]: cycle color scheme", pContextInfo->regFontList);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("'l': cycle lighting  'm': cycle full scene anti-aliasing", pContextInfo->regFontList);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("'p': toggle points   'w': toggle wireframe   'f': toggle fill", pContextInfo->regFontList);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("'q': toggle credits  'c': toggle OpenGL caps", pContextInfo->regFontList);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("Cmd-A: animate       Cmd-I: show info", pContextInfo->regFontList);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("Wheel: zoom camera", pContextInfo->boldFontList);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("Middle Button Drag: dolly object", pContextInfo->boldFontList);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("Right Button Drag: pan object", pContextInfo->boldFontList);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("Left Button Drag: rotate object", pContextInfo->boldFontList);
////			glRasterPos3d (10, height - line++ * 12, 0); 
////			drawCStringGL ("-- Help ('h') --", pContextInfo->boldFontList);
////		}
//
////		glColor3f (1.0, 1.0, 1.0);
////		glRasterPos3d (10, height - 27, 0); 
////		sprintf (cstr, "(%0.0f x %0.0f)", width, height);
////		drawCStringGL (cstr, pContextInfo->boldFontList);
////		// render and vendor info
////		glRasterPos3d (10, height - 15, 0); 
////		drawCStringGL ((char*) glGetString (GL_RENDERER), pContextInfo->boldFontList);
////		glRasterPos3d (10, height - 3, 0); 
////		drawCStringGL ((char*) glGetString (GL_VERSION), pContextInfo->regFontList);
////		if (pContextInfo->drawCaps) {
////			drawCaps (pContextInfo);
////		}
//	// reset orginal martices
//	glPopMatrix(); // GL_MODELVIEW
//	glMatrixMode (GL_PROJECTION);
//	glPopMatrix();
//	glMatrixMode (matrixMode);
//	if (depthTest)
//		glEnable (GL_DEPTH_TEST);
//	glReportError ();
}

// ---------------------------------



// main OpenGL drawing function
void drawGL(pRecContext pContextInfo, bool swap)
{
   if (!pContextInfo || !(pContextInfo->aglContext))
        return;
	// ensure the context is current	
	if (!aglSetCurrentContext (pContextInfo->aglContext))
		aglReportError ();

	// clear our drawable
	glClear(GL_COLOR_BUFFER_BIT);

	for (int x = 0; x < pContextInfo->numPorts; x++)
	{
		glClear(GL_DEPTH_BUFFER_BIT);
	
		setViewport(pContextInfo, x);
		if (pContextInfo->drawing)
		{
			// set projection
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glFrustum(pContextInfo->frust[x].left, pContextInfo->frust[x].right,
								pContextInfo->frust[x].bottom, pContextInfo->frust[x].top,
								pContextInfo->frust[x].near, pContextInfo->frust[x].far);
			// projection matrix already set	
			updateModelView(pContextInfo, x);

			HandleFrame(pContextInfo, x);

			if (pContextInfo->currPort == x)
			{
				glMatrixMode(GL_PROJECTION); glLoadIdentity();
				glMatrixMode(GL_MODELVIEW); glLoadIdentity();
				gluOrtho2D(0, pContextInfo->camera[x].viewWidth, 0, pContextInfo->camera[x].viewHeight);
				glDisable(GL_LIGHTING);
				glColor3ub(255, 255, 255);
				glBegin(GL_LINE_LOOP);
				glVertex2i(0, 0);
				glVertex2i(pContextInfo->camera[x].viewWidth, 0);
				glVertex2i(pContextInfo->camera[x].viewWidth, pContextInfo->camera[x].viewHeight);
				glVertex2i(0, pContextInfo->camera[x].viewHeight);
				glEnd();
			}
		}
	}
	if (swap)
		aglSwapBuffers(pContextInfo->aglContext);
	else
		glFlush();
}

#pragma mark ---- Carbon Timer ----

// per-window timer function, basic time based animation preformed here
static void timerContextCB(pRecContext pContextInfo, WindowRef window)
{
	AbsoluteTime currTime = UpTime ();
	double deltaTime = (float) AbsoluteDeltaToDuration (currTime, pContextInfo->time);

	pContextInfo->time = currTime;	// reset for next time interval
	if (0 > deltaTime)	// if negative microseconds
		deltaTime /= -1000000.0;
	else				// else milliseconds
		deltaTime /= 1000.0;
	if (deltaTime > 10.0) // skip pauses
		return;
	else {
		// do idle timer stuff here
		// animation, etc.
		// in this case we will draw the window instead of 
		// of just invalidating to get updates during drags
		// if we are not rotating with trackball in this window
		if (!gTrackball || (gTrackingContextInfo != pContextInfo)) {
			//updateRotation(deltaTime, pContextInfo->fRot, pContextInfo->fVel, pContextInfo->fAccel, pContextInfo->objectRotation);
		}
//		if (recording)
//			pContextInfo->unitLayer->advanceTime(1.0/30.0);
//		else
//			pContextInfo->unitLayer->advanceTime(deltaTime);

		drawGL(pContextInfo, true); // required to do this directly to get animation during resize and drags
//		frameCallback(pContextInfo->unitLayer);
		
		if (recording)
			captureNextMovieFrame(pContextInfo, window, 1.0/30.0);
	}
}

// ---------------------------------

// timer callback bottleneck
static pascal void timerCB(EventLoopTimerRef inTimer, void* userData)
{
	#pragma unused (inTimer)
    timerContextCB (GetCurrentContextInfo((WindowRef) userData), (WindowRef) userData); // timer based update
}

// ---------------------------------

// get UPP for timer
static EventLoopTimerUPP getTimerUPP(void)
{
	static EventLoopTimerUPP	sTimerUPP = NULL;
	
	if (sTimerUPP == NULL)
		sTimerUPP = NewEventLoopTimerUPP (timerCB);
	
	return sTimerUPP;
}

#pragma mark ---- Carbon Event Handling ----

pRecContext GetCurrentContextInfo (WindowRef window)
{
	if (NULL == window)  // HID use this path
		window = FrontWindow ();
	if (window)
		return (pRecContext) GetWRefCon (window);
	else
		return NULL;
}

// ---------------------------------

// move camera in z axis
static void mouseDolly(HIPoint location, pRecContext pContextInfo)
{
	GLfloat dolly = (gDollyPanStartPoint[1] - location.y) * -pContextInfo->camera[pContextInfo->currPort].viewPos.z / 300.0f;
	pContextInfo->camera[pContextInfo->currPort].viewPos.z += dolly;
	if (pContextInfo->camera[pContextInfo->currPort].viewPos.z == 0.0) // do not let z = 0.0
		pContextInfo->camera[pContextInfo->currPort].viewPos.z = 0.0001;
	updateProjection(pContextInfo, pContextInfo->currPort);  // update projection matrix
	gDollyPanStartPoint[0] = (long)location.x;
	gDollyPanStartPoint[1] = (long)location.y;
}
	
// ---------------------------------
	
// move camera in x/y plane
static void mousePan (HIPoint location, pRecContext pContextInfo)
{
	GLfloat panX = (gDollyPanStartPoint[0] - location.x) / (900.0f / -pContextInfo->camera[pContextInfo->currPort].viewPos.z);
	GLfloat panY = (gDollyPanStartPoint[1] - location.y) / (900.0f / -pContextInfo->camera[pContextInfo->currPort].viewPos.z);
	pContextInfo->camera[pContextInfo->currPort].viewPos.x += panX;
	pContextInfo->camera[pContextInfo->currPort].viewPos.y += panY;
	gDollyPanStartPoint[0] = (long)location.x;
	gDollyPanStartPoint[1] = (long)location.y;
}

// ---------------------------------

// Handles global non-system mouse events for GL windows as follows:
// primary button = object tumble (trackball)
// secondary button (or cntl-primary) = pan
// tertiary button (or option-primary) = dolly
// wheel = aperture
	
static OSStatus handleWindowMouseEvents (EventHandlerCallRef myHandler, EventRef event)
{
    WindowRef			window = NULL;
    pRecContext 		pContextInfo = NULL;
	OSStatus			result = eventNotHandledErr;
    UInt32 				kind = GetEventKind (event);
	EventMouseButton	button = 0;
	HIPoint				location = {0.0f, 0.0f};
	UInt32				modifiers = 0;	
	long				wheelDelta = 0;		
	Rect 				rectPort;

	// Mac OS X v10.1 and later
	// should this be front window???
	GetEventParameter(event, kEventParamWindowRef, typeWindowRef, NULL, sizeof(WindowRef), NULL, &window);
	if (window)
		pContextInfo = (pRecContext) GetWRefCon (window);
	if (!pContextInfo)
		return result; // not an application GLWindow so do not process (there is an exception)
	GetWindowPortBounds (window, &rectPort);
		
	result = CallNextEventHandler(myHandler, event);	
	if (eventNotHandledErr == result) 
	{ // only handle events not already handled (prevents wierd resize interaction)
		switch (kind) {
			// start trackball, pan, or dolly
			case kEventMouseDown:
				GetEventParameter(event, kEventParamMouseButton, typeMouseButton, NULL, sizeof(EventMouseButton), NULL, &button);
				GetEventParameter(event, kEventParamWindowMouseLocation, typeHIPoint, NULL, sizeof(HIPoint), NULL, &location);	// Mac OS X v10.1 and later
				GetEventParameter(event, kEventParamKeyModifiers, typeUInt32, NULL, sizeof(UInt32), NULL, &modifiers);
				{
					Point loc;
					GetEventParameter(event, kEventParamMouseLocation, typeQDPoint,
														NULL, sizeof( Point ), NULL, &loc );
					QDGlobalToLocalPoint( GetWindowPort( window ), & loc );
					
					point3d p = GetOGLPos(pContextInfo, loc.h, loc.v);
					printf("Mouse down hit (%1.2f, %1.2f, %1.2f) in the world (%d, %d) in window\n",
								 p.x, p.y, p.z, (int)location.x, (int)location.y);
					tButtonType bType = kLeftButton;
					switch (button)
					{
						case kEventMouseButtonPrimary: bType = kLeftButton; break;
						case kEventMouseButtonSecondary: bType = kRightButton; break;
						case kEventMouseButtonTertiary: bType = kMiddleButton; break;
					}
					if (HandleMouseClick(pContextInfo, (int)location.x, (int)location.y, p, bType, kMouseDown))
						break;
				}
				
				if ((button == kEventMouseButtonSecondary) || ((button == kEventMouseButtonPrimary) && (modifiers & controlKey)))
				{ // pan
					if (gTrackball)
					{ // if we are currently tracking, end trackball
						gTrackball = GL_FALSE;
						if (gTrackBallRotation[0] != 0.0)
							addToRotationTrackball(gTrackBallRotation, pContextInfo->rotations[pContextInfo->currPort].worldRotation);
						gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
					}
					else if (gDolly)
					{ // if we are currently dollying, end dolly
						gDolly = GL_FALSE;
					}
					gDollyPanStartPoint[0] = (GLint)location.x;
					gDollyPanStartPoint[1] = (GLint)location.y;
					gPan = GL_TRUE;
					gTrackingContextInfo = pContextInfo;
				}
				else if ((button == kEventMouseButtonTertiary) || ((button == kEventMouseButtonPrimary) && (modifiers & optionKey)))
				{ // dolly
					if (gTrackball)
					{ // if we are currently tracking, end trackball
						gTrackball = GL_FALSE;
						if (gTrackBallRotation[0] != 0.0)
							addToRotationTrackball (gTrackBallRotation, pContextInfo->rotations[pContextInfo->currPort].worldRotation);
						gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
					}
					else if (gPan)
					{ // if we are currently panning, end pan
						gPan = GL_FALSE;
					}
					gDollyPanStartPoint[0] = (long)location.x;
					gDollyPanStartPoint[1] = (long)location.y;
					gDolly = GL_TRUE;
					gTrackingContextInfo = pContextInfo;
				}
				else if (button == kEventMouseButtonPrimary)
				{ // trackball
					if (gDolly)
					{ // if we are currently dollying, end dolly
						gDolly = GL_FALSE;
						gTrackingContextInfo = NULL;
					}
					else if (gPan)
					{ // if we are currently panning, end pan
						gPan = GL_FALSE;
						gTrackingContextInfo = NULL;
					}
					startTrackball((long)location.x, (long)location.y,
												 (long)pContextInfo->camera[pContextInfo->currPort].viewOriginX,
												 (long)pContextInfo->camera[pContextInfo->currPort].viewOriginY,
												 pContextInfo->globalCamera.viewWidth,
												 pContextInfo->globalCamera.viewHeight);
					gTrackball = GL_TRUE;
					gTrackingContextInfo = pContextInfo;
				} 
					break;
			// stop trackball, pan, or dolly
			case kEventMouseUp:
			GetEventParameter(event, kEventParamWindowMouseLocation, typeHIPoint, NULL, sizeof(HIPoint), NULL, &location);	// Mac OS X v10.1 and later
			GetEventParameter(event, kEventParamMouseButton, typeMouseButton, NULL, sizeof(EventMouseButton), NULL, &button);
			{
				Point loc;
				GetEventParameter(event, kEventParamMouseLocation, typeQDPoint,
													NULL, sizeof( Point ), NULL, &loc );
				QDGlobalToLocalPoint( GetWindowPort( window ), & loc );
				
				point3d p = GetOGLPos(pContextInfo, loc.h, loc.v);
//				printf("Mouse down hit (%1.2f, %1.2f, %1.2f) in the world (%d, %d) in window\n",
//							 p.x, p.y, p.z, (int)location.x, (int)location.y);
				tButtonType bType = kLeftButton;
				switch (button)
				{
					case kEventMouseButtonPrimary: bType = kLeftButton; break;
					case kEventMouseButtonSecondary: bType = kRightButton; break;
					case kEventMouseButtonTertiary: bType = kMiddleButton; break;
				}
				if (HandleMouseClick(pContextInfo, (int)location.x, (int)location.y, p, bType, kMouseUp))
					break;
			}

				if (gDolly) { // end dolly
					gDolly = GL_FALSE;
				} else if (gPan) { // end pan
					gPan = GL_FALSE;
				} else if (gTrackball) { // end trackball
					gTrackball = GL_FALSE;
					if (gTrackBallRotation[0] != 0.0)
						addToRotationTrackball (gTrackBallRotation, pContextInfo->rotations[pContextInfo->currPort].worldRotation);
					gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
				} 
				gTrackingContextInfo = NULL;
				break;
			// trackball, pan, or dolly
			case kEventMouseDragged:
				GetEventParameter(event, kEventParamMouseButton, typeMouseButton, NULL, sizeof(EventMouseButton), NULL, &button);
				GetEventParameter(event, kEventParamWindowMouseLocation, typeHIPoint, NULL, sizeof(HIPoint), NULL, &location);	// Mac OS X v10.1 and later
				{
					Point loc;
					GetEventParameter(event, kEventParamMouseLocation, typeQDPoint,
														NULL, sizeof( Point ), NULL, &loc );
					QDGlobalToLocalPoint( GetWindowPort( window ), & loc );
					
					point3d p = GetOGLPos(pContextInfo, loc.h, loc.v);
//					printf("Mouse down hit (%1.2f, %1.2f, %1.2f) in the world (%d, %d) in window\n",
//								 p.x, p.y, p.z, (int)location.x, (int)location.y);
					tButtonType bType = kLeftButton;
					switch (button)
					{
						case kEventMouseButtonPrimary: bType = kLeftButton; break;
						case kEventMouseButtonSecondary: bType = kRightButton; break;
						case kEventMouseButtonTertiary: bType = kMiddleButton; break;
					}
					if (HandleMouseClick(pContextInfo, (int)location.x, (int)location.y, p, bType, kMouseDrag))
						break;
				}

				if (gTrackball)
				{
					rollToTrackball((long)location.x, (long)location.y, gTrackBallRotation);
					InvalWindowRect (window, &rectPort);
				}
				else if (gDolly) {
					mouseDolly(location, pContextInfo);
					InvalWindowRect(window, &rectPort);
				}
				else if (gPan) {
					mousePan(location, pContextInfo);
					InvalWindowRect(window, &rectPort);
				}
				break;
			// aperture change
			case kEventMouseWheelMoved: 
				GetEventParameter(event, kEventParamMouseWheelDelta, typeLongInteger, NULL, sizeof(long), NULL, &wheelDelta);
				if (wheelDelta)
				{
					GLfloat deltaAperture = wheelDelta * -pContextInfo->camera[pContextInfo->currPort].aperture / 200.0f;
					pContextInfo->camera[pContextInfo->currPort].aperture += deltaAperture;
					if (pContextInfo->camera[pContextInfo->currPort].aperture < 0.1) // do not let aperture <= 0.1
						pContextInfo->camera[pContextInfo->currPort].aperture = 0.1;
					if (pContextInfo->camera[pContextInfo->currPort].aperture > 179.9) // do not let aperture >= 180
						pContextInfo->camera[pContextInfo->currPort].aperture = 179.9;
					updateProjection(pContextInfo, pContextInfo->currPort); // update projection matrix
					InvalWindowRect(window, &rectPort);
				}
				break;
		}
		result = noErr;
	}	
	return result;
}

// ---------------------------------

// key input handler
static OSStatus handleKeyInput (EventHandlerCallRef myHandler, EventRef event, Boolean keyDown, void* userData)
{
	WindowRef window = (WindowRef) userData;
	OSStatus result = eventNotHandledErr;
	Rect rectPort = {0,0,0,0};
	pRecContext pContextInfo = GetCurrentContextInfo(window);
	
	result = CallNextEventHandler(myHandler, event);	
	if (eventNotHandledErr == result) { 
		if (pContextInfo) {
			UInt32 keyCode;
			UInt32 modifiers = 0;	
			unsigned char rawKey;
			GetEventParameter(event, kEventParamKeyMacCharCodes, typeChar, NULL, sizeof(char), NULL, &rawKey);
			//printf("Maybe %c was hit...\n", rawKey);
			GetEventParameter (event, kEventParamKeyCode, typeUInt32, NULL, sizeof(UInt32), NULL, &keyCode);
			GetEventParameter(event, kEventParamKeyModifiers, typeUInt32, NULL, sizeof(UInt32), NULL, &modifiers);
			if ((keyDown) && (!(modifiers&cmdKey)))
			{
				if (DoKeyboardCommand(pContextInfo, rawKey, modifiers&shiftKey, modifiers&controlKey, modifiers&optionKey))
				{
//					aglSetCurrentContext(pContextInfo->aglContext);
//					if (pContextInfo->unitLayer->GetMapAbstraction())
//						pContextInfo->unitLayer->GetMapAbstraction()->rebuild();
				}
			}
		}
	}
			
	GetWindowPortBounds(window, &rectPort);
	InvalWindowRect(window, &rectPort);
	
	return result;
}
// ---------------------------------

// window update event handling
static void handleWindowUpdate(WindowRef window)
{
	// set port for QD drawing
    GrafPtr portSave;
    GetPort (&portSave);
    SetPort (GetWindowPort (window));
    drawGL (GetCurrentContextInfo (window), true); // in this case just draw content
    SetPort (portSave);
}
	
// ---------------------------------

void createNewWindow(void)
{
    EventHandlerRef ref;
    EventTypeSpec list[] = { { kEventClassWindow, kEventWindowCollapsing },
		{ kEventClassWindow, kEventWindowCollapsed },
		{ kEventClassWindow, kEventWindowShown },
		{ kEventClassWindow, kEventWindowActivated },
		{ kEventClassWindow, kEventWindowClose },
		{ kEventClassWindow, kEventWindowDrawContent },
		{ kEventClassWindow, kEventWindowBoundsChanged },
		{ kEventClassWindow, kEventWindowZoomed },
		{ kEventClassKeyboard, kEventRawKeyDown },
		{ kEventClassKeyboard, kEventRawKeyRepeat },
		{ kEventClassKeyboard, kEventRawKeyUp } };

	
  WindowRef window = NULL;
	pRecContext pContextInfo = (pRecContext) NewPtrClear (sizeof (recContext)); // memory for window record
	initialConditions(pContextInfo);
	CreateWindowFromNib(nibRef, CFSTR("MainWindow"), &window); // build window
	if (window)
	{
		SetWRefCon (window, (long) pContextInfo); // point to the window record in the ref con of the window
		InstallWindowEventHandler(window, gWinEvtHandler, GetEventTypeCount (list), list, (void*)window, &ref); // add event handler
		ShowWindow(window);
		if (!pContextInfo->timer)
		{
			pContextInfo->time = UpTime();
			InstallEventLoopTimer(GetCurrentEventLoop(), 0, 1/60.0, getTimerUPP (), (void *) window, &pContextInfo->timer);
		}
	}
}

OSStatus getSaveFSSpec(WindowRef parentWin, CFStringRef title, CFStringRef action, CFStringRef file, CFStringRef message, FSSpec &fileRef)
{
  OSStatus err;
  AEKeyword theKeyword; 
  NavReplyRecord	theReply;
  //FSSpec fileRef; 
  Size actualSize; 
  DescType actualType; 
  NavDialogRef outDialog;
	
  NavDialogCreationOptions navopts;
  navopts.version = kNavDialogCreationOptionsVersion;
  navopts.optionFlags = kNavSupportPackages;
  navopts.location.h=-1;
  navopts.location.v=-1;
  navopts.clientName = CFSTR("HOG");
  navopts.windowTitle = title;
  navopts.actionButtonLabel = action;
  navopts.cancelButtonLabel = 0;
  navopts.saveFileName = file;
  navopts.message = message;
  navopts.preferenceKey = 0;
  navopts.popupExtension = 0;
  navopts.modality = kWindowModalityAppModal;
  navopts.parentWindow = parentWin;
  
	err = NavCreatePutFileDialog(&navopts,0,0,0,0,&outDialog);
	//err = NavCreateChooseFileDialog(&navopts, 0, 0, 0, 0, 0, &outDialog);
  err = NavDialogRun(outDialog);
  
  if (NavDialogGetUserAction(outDialog) == kNavUserActionSaveAs)
  {
    int value = 1;
    err = NavDialogGetReply(outDialog, &theReply);
    while ((err == noErr) && (theReply.validRecord))
    {
      err = AEGetNthPtr(&(theReply.selection), value, typeFSS, &theKeyword, &actualType,
                        &fileRef, sizeof (FSSpec), &actualSize);
			value++;
      if (err != noErr)
        return err;
      else {
				CInfoPBRec pb;
				StrFileName fileName;
				
				if (!fileRef.name[0])
				{
					err = FSMakeFSSpec(fileRef.vRefNum, fileRef.parID, "\p", &fileRef);
					if (err)
						return err;
				}
				
				// find the vRefNum and dirID of the parent directory
				pb.dirInfo.ioCompletion = nil;
				pb.dirInfo.ioNamePtr = fileRef.name;
				pb.dirInfo.ioVRefNum = fileRef.vRefNum;
				pb.dirInfo.ioFDirIndex = 0;
				pb.dirInfo.ioDrDirID = fileRef.parID;
				err = PBGetCatInfoSync(&pb);
				if (err || pb.dirInfo.ioResult)
					return err;
				
				// get the file name
				CFStringGetPascalString(theReply.saveFileName, fileName, 64, CFStringGetSystemEncoding());
				
				// build the spec
				FSMakeFSSpec(pb.dirInfo.ioVRefNum, pb.dirInfo.ioDrDirID, fileName, &fileRef);
				return noErr;
			}
		}
	}
	return -50; // parameter error?!?
}

void savePicture(WindowRef window, pRecContext pContextInfo)
{
	FSSpec fileRef;
	OSStatus err = getSaveFSSpec(window, CFSTR("HOG Image Export"), CFSTR("Save"), CFSTR("HOG ScreenShot.jpg"),
															 CFSTR("Enter file name to save screen export:"), fileRef);
	if (err)
		return;

	Rect bufferRect;
	GetWindowPortBounds (window, &bufferRect);
	drawGL(GetCurrentContextInfo (window), false); // in this case just draw content
	CompositeGLBufferIntoFile(pContextInfo->aglContext, &bufferRect, &fileRef);
}


void savePath(WindowRef window, pRecContext /*pContextInfo*/)
{
  OSStatus err;
  AEKeyword theKeyword; 
  NavReplyRecord	theReply;
  FSRef fileRef; 
  Size actualSize; 
  DescType actualType; 
  NavDialogRef outDialog;
	
  NavDialogCreationOptions navopts;
  navopts.version = kNavDialogCreationOptionsVersion;
  navopts.optionFlags = kNavSupportPackages;
  navopts.location.h=-1;
  navopts.location.v=-1;
  navopts.clientName = CFSTR("ORTS Map Test");
  navopts.windowTitle = CFSTR("ORTS Map Saver");
  navopts.actionButtonLabel = CFSTR("Save");
  navopts.cancelButtonLabel = 0;
  navopts.saveFileName = 0;
  navopts.message = CFSTR("Enter file name to save map:");
  navopts.preferenceKey = 0;
  navopts.popupExtension = 0;
  navopts.modality = kWindowModalityAppModal;
  navopts.parentWindow = window;
  
	err = NavCreatePutFileDialog(&navopts,0,0,0,0,&outDialog);
	//err = NavCreateChooseFileDialog(&navopts, 0, 0, 0, 0, 0, &outDialog);
  err = NavDialogRun(outDialog);
  
  if (NavDialogGetUserAction(outDialog) == kNavUserActionSaveAs)
  {
    int value = 1;
    err = NavDialogGetReply(outDialog, &theReply);
    while ((err == noErr) && (theReply.validRecord))
    {
      err = AEGetNthPtr(&(theReply.selection), value, typeFSRef, &theKeyword, &actualType,
                        &fileRef, sizeof (fileRef), &actualSize);
      if (err != noErr)
        break;
      else {
        CFURLRef f = CFURLCreateFromFSRef(0, &fileRef);
        CFStringRef fstr = CFURLCopyFileSystemPath(f, kCFURLPOSIXPathStyle);
				
        char buffer[1024];
				CFStringGetCString(fstr, buffer, 1024, kCFStringEncodingASCII);
				int len = strlen(buffer);
				buffer[len] = '/';
				CFStringGetCString(theReply.saveFileName, &buffer[len+1], 128, kCFStringEncodingASCII);
        {
			printf("You selected \"%s\"\n", buffer);
					// FIXME
			//pContextInfo->unitLayer->GetMap()->Save(buffer);
					break;
        }        
      }
    }
  }
}

void loadPath(char *path)
{
  OSStatus err;
  AEKeyword theKeyword; 
  NavReplyRecord	theReply;
  FSRef fileRef; 
  Size actualSize; 
  DescType actualType; 
  NavDialogRef outDialog;
	
  NavDialogCreationOptions navopts;
  navopts.version = kNavDialogCreationOptionsVersion;
  navopts.optionFlags = kNavSupportPackages;
  navopts.location.h=-1;
  navopts.location.v=-1;
  navopts.clientName = CFSTR("ORTS Map Test");
  navopts.windowTitle = CFSTR("ORTS Map Loader");
  navopts.actionButtonLabel = CFSTR("Open");
  navopts.cancelButtonLabel = 0;
  navopts.saveFileName = 0;
  navopts.message = CFSTR("Find map to load:");
  navopts.preferenceKey = 0;
  navopts.popupExtension = 0;
  navopts.modality = kWindowModalityAppModal;
  navopts.parentWindow = 0;
  
  err = NavCreateChooseFileDialog(&navopts, 0, 0, 0, 0, 0, &outDialog);
  err = NavDialogRun(outDialog);
  if (NavDialogGetUserAction(outDialog) == kNavUserActionChoose)
  {
    int value = 1;
    err = NavDialogGetReply(outDialog, &theReply);
    while ((err == noErr) && (theReply.validRecord))
    {
      err = AEGetNthPtr(&(theReply.selection), value, typeFSRef, &theKeyword, &actualType,
                        &fileRef, sizeof (fileRef), &actualSize);
      if (err != noErr)
        break;
      else {
        CFURLRef f = CFURLCreateFromFSRef(0, &fileRef);
        CFStringRef fstr = CFURLCopyFileSystemPath(f, kCFURLPOSIXPathStyle);
        char buffer[1024];
        if (CFStringGetCString(fstr,
                               buffer,
                               1024,
                               kCFStringEncodingASCII))
        {
          sprintf(path, "%s", buffer);
					CFRelease(fstr);
					CFRelease(f);
					NavDisposeReply(&theReply);
					NavDialogDispose(outDialog);
          return;
        }        
				CFRelease(fstr);
				CFRelease(f);
      }
    }
		NavDisposeReply(&theReply);
  }
	NavDialogDispose(outDialog);
  path[0] = 0;
}

void saveSimHistory(WindowRef window, pRecContext )
{
  OSStatus err;
  AEKeyword theKeyword; 
  NavReplyRecord	theReply;
  FSRef fileRef; 
  Size actualSize; 
  DescType actualType; 
  NavDialogRef outDialog;
	
  NavDialogCreationOptions navopts;
  navopts.version = kNavDialogCreationOptionsVersion;
  navopts.optionFlags = kNavSupportPackages;
  navopts.location.h=-1;
  navopts.location.v=-1;
  navopts.clientName = CFSTR("ORTS Test");
  navopts.windowTitle = CFSTR("ORTS History Saver");
  navopts.actionButtonLabel = CFSTR("Save");
  navopts.cancelButtonLabel = 0;
  navopts.saveFileName = 0;
  navopts.message = CFSTR("Enter file name to save history:");
  navopts.preferenceKey = 0;
  navopts.popupExtension = 0;
  navopts.modality = kWindowModalityAppModal;
  navopts.parentWindow = window;
  
	err = NavCreatePutFileDialog(&navopts,0,0,0,0,&outDialog);
	//err = NavCreateChooseFileDialog(&navopts, 0, 0, 0, 0, 0, &outDialog);
  err = NavDialogRun(outDialog);
  
  if (NavDialogGetUserAction(outDialog) == kNavUserActionSaveAs)
  {
    int value = 1;
    err = NavDialogGetReply(outDialog, &theReply);
    while ((err == noErr) && (theReply.validRecord))
    {
      err = AEGetNthPtr(&(theReply.selection), value, typeFSRef, &theKeyword, &actualType,
                        &fileRef, sizeof (fileRef), &actualSize);
      if (err != noErr)
        break;
      else {
        CFURLRef f = CFURLCreateFromFSRef(0, &fileRef);
        CFStringRef fstr = CFURLCopyFileSystemPath(f, kCFURLPOSIXPathStyle);
				
        char buffer[1024];
				CFStringGetCString(fstr, buffer, 1024, kCFStringEncodingASCII);
				int len = strlen(buffer);
				buffer[len] = '/';
				CFStringGetCString(theReply.saveFileName, &buffer[len+1], 128, kCFStringEncodingASCII);
        {
          printf("You selected \"%s\"\n", buffer);
					// FIXME
//					pContextInfo->unitLayer->saveHistory(buffer);
					break;
        }        
      }
    }
  }
}

// FIXME
void openSimHistory()
{
//	WindowRef window = FrontWindow();
//	pRecContext pContextInfo;
//
//	loadPath(gDefaultMap);
//	if (gDefaultMap[0] == 0)
//		return;
//
//	if (window == 0)
//	{
//		createNewWindow();
//		window = FrontWindow();
//		pContextInfo = (pRecContext)GetWRefCon(window);
////		pContextInfo->unitLayer->loadHistory(gDefaultMap);
//	}
//	else {
//		pContextInfo = (pRecContext)GetWRefCon(window);
//		
//		printf("Loading \'%s\'\n", gDefaultMap);
//		processStats(pContextInfo->unitLayer->getStats());
//		delete pContextInfo->unitLayer;
//		createSimulation(pContextInfo->unitLayer);
//		pContextInfo->unitLayer->loadHistory(gDefaultMap);
//	}
//	char pathTitle[1024];
//	sprintf(pathTitle, "%s (%ldx%ld)", gDefaultMap, pContextInfo->unitLayer->GetMap()->GetMapWidth(), pContextInfo->unitLayer->GetMap()->GetMapHeight());
//	CFStringRef str = CFStringCreateWithCString(NULL, pathTitle, 0);
//	SetWindowTitleWithCFString(window, str);
//	CFRelease(str);
}


void openNewMap()
{
	WindowRef window = FrontWindow();
	pRecContext pContextInfo;

	loadPath(gDefaultMap);
	if (gDefaultMap[0] == 0)
		return;

	if ((1)||(window == 0))
	{
		createNewWindow();
		window = FrontWindow();
		pContextInfo = (pRecContext)GetWRefCon(window);
		HandleWindowEvent(pContextInfo, kWindowCreated);
	}
//	else {
//		pContextInfo = (pRecContext)GetWRefCon(window);
//		
//		printf("Loading \'%s\'\n", gDefaultMap);
//		processStats(pContextInfo->unitLayer->getStats());
//		delete pContextInfo->unitLayer;
//		createSimulation(pContextInfo->unitLayer);
//		//pContextInfo->unitLayer->GetMap()->Load(gDefaultMap);
//	}
	
//	char pathTitle[1024];
//	sprintf(pathTitle, "%s (%ldx%ld)", gDefaultMap, pContextInfo->unitLayer->GetMap()->GetMapWidth(), pContextInfo->unitLayer->GetMap()->GetMapHeight());
//	CFStringRef str = CFStringCreateWithCString(NULL, pathTitle, 0);
//	SetWindowTitleWithCFString(window, str);
//	CFRelease(str);
}

// ---------------------------------

// window event handler
static pascal OSStatus windowEvtHndlr (EventHandlerCallRef myHandler, EventRef event, void* userData)
{
#pragma unused (userData)
	WindowRef			window = NULL;
  pRecContext			pContextInfo = NULL;
	Rect				rectPort = {0,0,0,0};
	CGRect 				viewRect = {{0.0f, 0.0f}, {0.0f, 0.0f}};
  OSStatus			result = eventNotHandledErr;
  UInt32 				eventClass = GetEventClass (event);
  UInt32 				eventKind = GetEventKind (event);

	switch (eventClass) {
		case kEventClassKeyboard:
			switch (eventKind) {
				case kEventRawKeyDown:
				case kEventRawKeyRepeat:
					result = handleKeyInput(myHandler, event, true, userData);
					break;
				case kEventRawKeyUp:
					result = handleKeyInput(myHandler, event, false, userData);
					break;
			}
			break;
		case kEventClassWindow:
			GetEventParameter(event, kEventParamDirectObject, typeWindowRef, NULL, sizeof(WindowRef), NULL, &window);
			switch (eventKind) {
				case kEventWindowCollapsing:
					pContextInfo = (pRecContext) GetWRefCon (window);
					GetWindowPortBounds (window, &rectPort);
					drawGL (GetCurrentContextInfo (window), false); // in this case just draw content
					CompositeGLBufferIntoWindow( pContextInfo->aglContext, &rectPort, window);
					result = UpdateCollapsedWindowDockTile (window);
					break;
				case kEventWindowActivated: // called on click activation and initially
					handleWindowUpdate(window);
					break;
				case kEventWindowDrawContent:
					handleWindowUpdate(window);
					break;
				case kEventWindowClose: // called when window is being closed (close box)
					HideWindow (window);
					pContextInfo = (pRecContext) GetWRefCon (window); // get window info
					disposeGL(pContextInfo);
					DisposePtr((Ptr) pContextInfo);
          SetWRefCon(window, 0);
					break;
				case kEventWindowShown: // called on initial show (not on un-minimize)
					buildGL (window);
					if (window == FrontWindow ())
						SetUserFocusWindow (window);
					GetWindowPortBounds (window, &rectPort);
					InvalWindowRect (window, &rectPort);
					break;
				case kEventWindowBoundsChanged: // called for resize and moves (drag)
					GetWindowPortBounds (window, &rectPort); // get port rect
					pContextInfo = (pRecContext) GetWRefCon (window); // get window info	
					GetWindowPortBounds (window, &rectPort);
					viewRect.size.width = (float) (rectPort.right - rectPort.left);
					viewRect.size.height = (float) (rectPort.bottom - rectPort.top);
					//char text[128];
					//sprintf(text, "Window size (%d, %d)", rectPort.right - rectPort.left, rectPort.bottom - rectPort.top);
					//reportError(text);
					// if we are not animating and shrinking, force update
					if (pContextInfo && !pContextInfo->timer && 
					    ((pContextInfo->globalCamera.viewHeight > viewRect.size.height) ||
						 (pContextInfo->globalCamera.viewWidth > viewRect.size.width))) {
						resizeGL(pContextInfo, viewRect);
						handleWindowUpdate (window); // must force immediate update to get live resize
					} else
						resizeGL (pContextInfo, viewRect);					
					break;
				case kEventWindowZoomed: // called when user clicks on zoom button
					GetWindowPortBounds (window, &rectPort);
					viewRect.size.width = (float) (rectPort.right - rectPort.left);
					viewRect.size.height = (float) (rectPort.bottom - rectPort.top);
					resizeGL(pContextInfo, viewRect);
					break;
			}
			break;
	}
    return result;
}

// ---------------------------------

// application level event handler
static pascal OSStatus appEvtHndlr (EventHandlerCallRef myHandler, EventRef event, void* userData)
{
#pragma unused (myHandler, userData)
    OSStatus result = eventNotHandledErr;
    Rect rectPort;
    pRecContext pContextInfo = NULL;
    WindowRef window = FrontWindow ();
    UInt32 eventClass = GetEventClass (event);
    UInt32 eventKind = GetEventKind (event);
    HICommand command;

	// gss: use current context call
    if (window) { // do we have a window?
        GetWindowPortBounds(window, &rectPort); // get bounds for later inval
        pContextInfo = (pRecContext) GetWRefCon (window); // get window info
    }
	
	switch (eventClass) {
		case kEventClassMouse:
			handleWindowMouseEvents (myHandler, event);
			break;
		case kEventClassCommand:
			switch (eventKind) {
				case kEventProcessCommand:
					GetEventParameter (event, kEventParamDirectObject, kEventParamHICommand, NULL, sizeof(command), NULL, &command); // get command
					switch (command.commandID) {
						case 'neww':
							//if (FrontWindow() == 0)
							createNewWindow();
							window = FrontWindow();
							pContextInfo = (pRecContext)GetWRefCon(window);
							HandleWindowEvent(pContextInfo, kWindowCreated);
							result = noErr;
							break;
						case 'open':
							openNewMap();
							break;
						case 'ohis':
							openSimHistory();
							break;
						case 'shis':
							if (window != 0)
								saveSimHistory(window, (pRecContext)GetWRefCon(window));
							break;
						case 'save':
							if (window != 0)
								savePath(window, (pRecContext)GetWRefCon(window));
							break;
						case 'EXPT':
							pContextInfo = (pRecContext)GetWRefCon(window);
							if (pContextInfo)
							{
								savePicture(window, (pRecContext)GetWRefCon(window)); 
							}
							break;
						case 'RMOV':
							if (recording)
							{
								recording = false;
//								bool paused = pContextInfo->unitLayer->getSimulationPaused();
//								pContextInfo->unitLayer->setSimulationPaused(true);
								exportMovie();
//								pContextInfo->unitLayer->setSimulationPaused(paused);
							}
							else {
								recording = true;
							}
							break;
						case 'clsw':
							if (window) {
								HideWindow(window);
								HandleWindowEvent(GetCurrentContextInfo(window), kWindowDestroyed);
								disposeGL(GetCurrentContextInfo(window));  // dump our structures
								DisposeWindow (window); // if it exists dump it
							}
							break;
						case 'quit':
							//memory will get dumped on exit, other callbacks/timers are for this process only
							break;
						case 'gogo':
							if (pContextInfo) {  // have window info
								pContextInfo->animate = 1 - pContextInfo->animate;
								if (pContextInfo->animate && !pContextInfo->timer) {
									pContextInfo->time = UpTime ();
									InstallEventLoopTimer (GetCurrentEventLoop(), 0, 0.001, getTimerUPP (), (void *) window, &pContextInfo->timer);
								} else if (!pContextInfo->animate && pContextInfo->timer) {
									//RemoveEventLoopTimer(pContextInfo->timer);
									pContextInfo->timer = NULL;
								}
							}
							break;
						case 'info':
							if (pContextInfo) {  // have window info
								pContextInfo->info = 1 - pContextInfo->info;
								GetWindowPortBounds (window, &rectPort);
								InvalWindowRect (window, &rectPort);
							}
							break;
					}
					break;
				case kEventCommandUpdateStatus:
					GetEventParameter (event, kEventParamDirectObject, kEventParamHICommand, NULL, sizeof(command), NULL, &command); // get command
					switch (command.commandID) {
						case 'clsw':
							if (pContextInfo)
								EnableMenuItem (GetMenuHandle (kMainMenu), kCloseMenuItem);
							else 
								DisableMenuItem (GetMenuHandle (kMainMenu), kCloseMenuItem);
							break;
						case 'gogo':
							if (pContextInfo) {
								EnableMenuItem (GetMenuHandle (kMainMenu), kAnimateMenuItem);
								CheckMenuItem (GetMenuHandle (kMainMenu), kAnimateMenuItem, pContextInfo->animate);
							} else { 
								DisableMenuItem (GetMenuHandle (kMainMenu), kAnimateMenuItem);
								CheckMenuItem (GetMenuHandle (kMainMenu), kAnimateMenuItem, kAnimateState);
							}
							break;
						case 'info':
							if (pContextInfo) {
								EnableMenuItem (GetMenuHandle (kMainMenu), kInfoMenuItem);
								CheckMenuItem (GetMenuHandle (kMainMenu), kInfoMenuItem, pContextInfo->info);
							} else { 
								DisableMenuItem (GetMenuHandle (kMainMenu), kInfoMenuItem);
								CheckMenuItem (GetMenuHandle (kMainMenu), kInfoMenuItem, kInfoState);
							}
							break;
					}
					result = noErr;
					break;
			}
			break;
	}
    return result;
}

#pragma mark ==== main ====

void RunHOGGUI(int argc, char* argv[])
{
    OSStatus		err;
    EventHandlerRef	ref;
    EventTypeSpec	list[] = { { kEventClassCommand,  kEventProcessCommand },
							   { kEventClassCommand,  kEventCommandUpdateStatus },
							   { kEventClassMouse, kEventMouseDown },// handle trackball functionality globaly because there is only a single user
							   { kEventClassMouse, kEventMouseUp }, 
							   { kEventClassMouse, kEventMouseDragged },
							   { kEventClassMouse, kEventMouseWheelMoved } };
	ProcessSerialNumber psn = { 0, kCurrentProcess };

	ProcessCommandLineArgs(argc, argv);

	StartHIDInput();
	gStartTime = UpTime (); // get app start time
	
    // Create a Nib reference passing the name of the nib file (without the .nib extension)
    // CreateNibReference only searches into the application bundle.
    err = CreateNibReference(CFSTR("main"), &nibRef);
    require_noerr( err, CantGetNibRef );
    err = SetMenuBarFromNib(nibRef, CFSTR("MainMenu"));
    require_noerr( err, CantSetMenuBar );
		gEvtHandler = NewEventHandlerUPP(appEvtHndlr);
    InstallApplicationEventHandler (gEvtHandler, GetEventTypeCount (list), list, 0, &ref);
    gWinEvtHandler = NewEventHandlerUPP(windowEvtHndlr);
	
	getCurrentCaps();

	// ensure we know when display configs are changed
	gConfigEDMUPP = NewDMExtendedNotificationUPP (handleConfigDMEvent); // for display change notification
	DMRegisterExtendedNotifyProc (gConfigEDMUPP, NULL, 0, &psn);
		//createNewWindow();

	// Call the event loop
  RunApplicationEventLoop();
	
	// clean up any windows...
	{
		WindowRef window;
		window = FrontWindow();
		while (window) {
			HideWindow(window);
			pRecContext pContextInfo = (pRecContext)GetWRefCon(window); // get window info
			disposeGL(pContextInfo);
			DisposePtr((Ptr) pContextInfo);
			SetWRefCon(window, 0);
			DisposeWindow(window); // if it exists dump it
			window = GetNextWindow(window);
		}
	}
	
	// Exiting...
	if (gConfigEDMUPP) { // dispose UPP for DM notifications
		DisposeDMExtendedNotificationUPP (gConfigEDMUPP);
		gConfigEDMUPP = NULL;
	}

	if (gDisplayCaps) { // dispose diaplsy capabilities info
		free (gDisplayCaps);
		gDisplayCaps = NULL;
	}
	//getchar();
//CantCreateWindow:
CantSetMenuBar:
CantGetNibRef:

    // We don't need the nib reference anymore.
    DisposeNibReference (nibRef);
}

