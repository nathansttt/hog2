#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.2 2006/10/20 20:20:15 emarkus Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/apps/sample/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = sample
DBG_NAME = $(NAME)
REL_NAME = $(NAME)

ROOT = ../../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs/$(NAME)/release
DBG_BINDIR = $(ROOT)/bin/debug
REL_BINDIR = $(ROOT)/bin/release

PROJ_CXXFLAGS = -I$(ROOT)/abstraction -I$(ROOT)/driver -I$(ROOT)/simulation -I$(ROOT)/util -I$(ROOT)/shared
PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS)

PROJ_DBG_LNFLAGS = -L$(DBG_BINDIR)
PROJ_REL_LNFLAGS = -L$(REL_BINDIR)

PROJ_DBG_LIB = -ldriver -labstraction -lshared -lsimulation -lutil
PROJ_REL_LIB = -ldriver -labstraction -lshared -lsimulation -lutil

PROJ_DBG_DEP = \
  $(DBG_BINDIR)/libgraph.a \
  $(DBG_BINDIR)/libutil.a \
  $(DBG_BINDIR)/libalgorithms.a \
  $(DBG_BINDIR)/libabstractionalgorithms.a \
  $(DBG_BINDIR)/libgraphalgorithms.a \
  $(DBG_BINDIR)/libabsmapalgorithms.a \
  $(DBG_BINDIR)/libabstraction.a \
  $(DBG_BINDIR)/libshared.a \
  $(DBG_BINDIR)/libsimulation.a \
  $(DBG_BINDIR)/libmapalgorithms.a \

PROJ_REL_DEP = \
  $(DBG_BINDIR)/libgraph.a \
  $(DBG_BINDIR)/libutil.a \
  $(DBG_BINDIR)/libalgorithms.a \
  $(DBG_BINDIR)/libabstractionalgorithms.a \
  $(DBG_BINDIR)/libgraphalgorithms.a \
  $(DBG_BINDIR)/libabsmapalgorithms.a \
  $(DBG_BINDIR)/libabstraction.a \
  $(DBG_BINDIR)/libshared.a \
  $(DBG_BINDIR)/libsimulation.a \
  $(DBG_BINDIR)/libmapalgorithms.a \

default : all

SRC_CPP = \
	apps/pathsample/Sample.cpp \
