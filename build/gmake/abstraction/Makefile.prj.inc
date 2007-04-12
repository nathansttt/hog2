#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.2 2006/11/01 23:34:14 nathanst Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/abstraction/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = abstraction
DBG_NAME = lib$(NAME).a
REL_NAME = lib$(NAME).a

ROOT = ../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs/$(NAME)/release
DBG_BINDIR = $(ROOT)/bin/debug
REL_BINDIR = $(ROOT)/bin/release

PROJ_CXXFLAGS = -I$(ROOT)/util -I$(ROOT)/simulation -I$(ROOT)/abstraction
PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS)

PROJ_DBG_LNFLAGS =
PROJ_REL_LNFLAGS =

PROJ_DBG_LIB =
PROJ_REL_LIB =

default : all

SRC_CPP = \
  abstraction/clusterAbstraction.cpp \
  abstraction/mapAbstraction.cpp \
  abstraction/mapProvider.cpp \
  abstraction/width.cpp \
  abstraction/graphAbstraction.cpp \
  abstraction/mapCliqueAbstraction.cpp \
  abstraction/mapQuadTreeAbstraction.cpp \
  abstraction/loadedCliqueAbstraction.cpp \
  abstraction/mapFlatAbstraction.cpp \
  abstraction/radiusAbstraction.cpp
