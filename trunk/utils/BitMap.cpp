/*
 *  BitMap.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/4/11.
 *  Copyright 2011 NS Software. All rights reserved.
 *
 */

#include "BitMap.h"
#include <stdio.h>

BitMapPic::BitMapPic(const char *file)
{
	Load(file);
}

BitMapPic::BitMapPic(int w, int h)
:width(w), height(h)
{
	image.resize(w*h*4);
	bytesReversed = false;
}

BitMapPic::BitMapPic(int w, int h, uint8_t *data)
:width(w), height(h)
{
	bytesReversed = false;
	image.resize(w*h*4);
	// slow copy, but correct TODO: memcpy
	for (int x = 0; x < w*h*4; x++)
		image[x] = data[x];
}

void BitMapPic::Save(const char *file)
{
	FILE *f = fopen(file, "w+");
	
	if (f == 0) return;
	
//	long rowBytes = width * 4;
	char zero[4] = {0, 0, 0, 0};
	
	bmp_header h;
	h.biWidth = width;
	if (bytesReversed)
		h.biHeight = -height;
	else
		h.biHeight = height;

	int buffer = (4-width%4)%4;
	h.bfSize = sizeof(bmp_header)+2+(width+buffer)*height*4;
	h.biSizeImage = (width+buffer)*height*4;
	uint16_t bfType = 19778;
	fwrite(&bfType, sizeof(bfType), 1, f);
	fwrite(&h, sizeof(bmp_header), 1, f);
	for (int x = 0; x < height; x++)
	{
		fwrite(&image[x*width*4], sizeof(char), width*4, f);
		if (0 != width%4)
			fwrite(&zero, sizeof(char), buffer, f);
	}
	fclose(f);
}

void BitMapPic::Load(const char *file)
{
	FILE *f = fopen(file, "rb");
	
	if (f == 0)
	{
		printf("Unable to find file\n");
		width = 0;
		height = 0;
		image.resize(0);
		return;
	}
	
//	long rowBytes = width * 4;
//	long imageSize = rowBytes * height;
	char zero[4] = {0, 0, 0, 0};
	int buffer = (4-width%4)%4;
	
	bmp_header h;
	uint16_t bfType;
	fread(&bfType, sizeof(bfType), 1, f);
	if (bfType != 19778)
	{
		printf("Unable to load file properly\n");
		width = 0;
		height = 0;
		image.resize(0);
		fclose(f);
		return;
	}
	fread(&h, sizeof(bmp_header), 1, f);
	h.dump();
	height = h.biHeight;
	if ((int32_t)h.biHeight < 0) 
	{
		height = -h.biHeight;
		bytesReversed = true;
	}
	width = h.biWidth;
	image.resize(height*width*4);
//	printf("char size: %d\n", sizeof(char));
	fseek(f, h.bfOffBits, SEEK_SET);
	for (int x = 0; x < height; x++)
	{
//		printf("Reading at %d (%d)\n", x*width*4, width*4);
		//printf("Read: %d\n", );
		fread(&image[x*width*4], sizeof(char), width*4, f);
		if (0 != width%4)
		{
			fread(&zero, sizeof(char), buffer, f);
		}
	}
	fclose(f);
//	for (unsigned int x = image.size()-4096; x < image.size(); x++)
//		printf("%4d", image[x]);
}

