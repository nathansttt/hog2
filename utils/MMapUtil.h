//
//  MMapUtil.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/14/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#ifndef hog2_glut_MMapUtil_h
#define hog2_glut_MMapUtil_h

uint8_t *GetMMAP(const char *filename, uint64_t mapSizeBytes, int &fd, bool zero = false);
void CloseMMap(uint8_t *mem, uint64_t mapSizeBytes, int fd);

#endif
