/*
 *  BitMapPic.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/4/11.
 *  Copyright 2011 NS Software. All rights reserved.
 *
 */

#ifndef BITMAPPIC_H
#define BITMAPPIC_H

#include <stdio.h>
#include <vector>
#ifndef WIN32
#include <stdint.h>
#endif
#ifdef WIN32
#include "pstdint.h"
#endif

class bmp_header {
public:
	bmp_header()
	://bfType(19778),
	zero(0), bfOffBits(sizeof(bmp_header)+2), biSize(40), biPlanes(1),
	biBitCount(32), biCompression(0), biSizeImage(0), biXPelsPerMeter(2835), biYPelsPerMeter(2835),
	biClrUsed(0), biClrImportant(0) {}

	void dump()
	{
		printf("bfSize: %d\n", bfSize);
		printf("zero: %d\n", zero);
		printf("bfOffBits: %d\n", bfOffBits);

		printf("biSize: %d\n", biSize);
		printf("biWidth: %d\n", biWidth);
		printf("biHeight: %d\n", biHeight);
		printf("biPlanes: %d\n", biPlanes);
		printf("biBitCount: %d\n", biBitCount);
		printf("biCompression: %d\n", biCompression);
		printf("biSizeImage: %d\n", biSizeImage);
		printf("biXPelsPerMeter: %d\n", biXPelsPerMeter);
		printf("biYPelsPerMeter: %d\n", biYPelsPerMeter);
		printf("biClrUsed: %d\n", biClrUsed);
		printf("biClrImportant: %d\n", biClrImportant);
	}
	
	//	uint16_t bfType;//	19778
	uint32_t bfSize; //	??	specifies the size of the file in bytes.
	uint32_t zero; // 0
	uint32_t bfOffBits;
	//	11	4	bfOffBits	1078	specifies the offset from the beginning of the file to the bitmap data.
	
	uint32_t biSize; // 40
	uint32_t biWidth;
	uint32_t biHeight;
	uint16_t biPlanes; // 0 (1??)
	uint16_t biBitCount; // 24
	uint32_t biCompression; // 0
	uint32_t biSizeImage; // 0
	uint32_t biXPelsPerMeter; // 0
	uint32_t biYPelsPerMeter; // 0
	uint32_t biClrUsed; // 0
	uint32_t biClrImportant; // 0
};

class BitMapPic {
public:
	BitMapPic(const char *file);
	BitMapPic(int width, int height);
	BitMapPic(int width, int height, uint8_t *data);
	void Save(const char *file);
	void Load(const char *file);
	int GetWidth() { return width; }
	int GetHeight() { return height; }
	void SetPixel(int x, int y, uint8_t redByte, uint8_t greenByte, uint8_t blueByte, uint8_t alphaByte = 0)
	{ 
		if (BytesReversed())
		{
			image[y*width*4+x*4+0] = blueByte;
			image[y*width*4+x*4+1] = greenByte;
			image[y*width*4+x*4+2] = redByte;
			image[y*width*4+x*4+3] = alphaByte;
		}
		else {
			image[y*width*4+x*4+0] = redByte;
			image[y*width*4+x*4+1] = greenByte;
			image[y*width*4+x*4+2] = blueByte;
			image[y*width*4+x*4+3] = alphaByte;
		}
	}
	void GetPixel(int x, int y, uint8_t &redByte, uint8_t &greenByte, uint8_t &blueByte, uint8_t &alphaByte)
	{
		if (BytesReversed())
		{
			blueByte = image[y*width*4+x*4+0];
			greenByte = image[y*width*4+x*4+1];
			redByte = image[y*width*4+x*4+2];
			alphaByte = image[y*width*4+x*4+3];
		}
		else {
			redByte = image[y*width*4+x*4+0];
			greenByte = image[y*width*4+x*4+1];
			blueByte = image[y*width*4+x*4+2];
			alphaByte = image[y*width*4+x*4+3];
		}
	}
	uint8_t *GetBytes() { return &image[0]; }
	bool BytesReversed() { return bytesReversed; }
private:
	uint32_t width, height;
	bool bytesReversed;
	std::vector<uint8_t> image;
};

#endif
