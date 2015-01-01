#-----------------------------------------------------------------------------
# Main Makefile for project_hog.
# The sub-makefiles are invoked sequentially
# (see: Peter Miller: Recursive Make Considered Harmful
# http://aegis.sourceforge.net/auug97.pdf)
#
# $Source: /usr/cvsroot/project_hog/build/gmake/Makefile,v $
# $Id: Makefile,v 1.5 2006/11/01 23:34:14 nathanst Exp $
#-----------------------------------------------------------------------------

# All projects to build in this order.
PROJECTS = \
  gui/GL \
  absmapalgorithms \
  abstraction \
  abstractionalgorithms \
  algorithms \
  environments \
  generic \
  graph \
  graphalgorithms \
  gui \
  mapalgorithms \
  shared \
  utils \
  apps/arm \
  apps/localsensingsearch \
  apps/cpdb \
  apps/pathsample \
  apps/puzzlesample \
  apps/multiagent \
  apps/inconsistency \
  apps/measure_inconsistency \
  apps/dynamicsearch \
  apps/directional \
  apps/dragonage \
  apps/mapUtils \
  apps/prediction \
  apps/emotion \
  apps/sfbds \
  apps/3dmaps \
  apps/rubik \
  apps/fling \
  apps/topspin \
#  simulation \
#  learning \
#	apps/coprobber
#  apps/lv.pathfinding	
#  apps/inconsistency \




# Include projects with broken compilation
PROJECTS_ALL = \
  gui/GL \
  absmapalgorithms \
  abstraction \
  abstractionalgorithms \
  algorithms \
  environments \
  generic \
  graph \
  graphalgorithms \
  gui \
  mapalgorithms \
  shared \
  simulation \
  utils \
  apps/arm \
  apps/localsensingsearch \
  apps/cpdb \
  apps/pathsample \
  apps/puzzlesample \
  apps/inconsistency \
  apps/multiagent \
  apps/dynamicsearch \
  apps/directional \
  apps/mapUtils \
  apps/dragonage \
  apps/sfbds \
  apps/prediction \
  apps/rubik \
  apps/fling \
  apps/topspin \
#	apps/coprobber

# sequentially to avoid same sub-target in sub-make invoked twice
default:
	$(MAKE) debug
	$(MAKE) release

debug:
	@for dir in $(PROJECTS); do \
           if test -f $$dir/Makefile; then \
             echo -e "\n--------------- $$dir (dbg)"; \
             $(MAKE) -C $$dir debug || exit -1; \
           fi; \
        done
	@echo ""

release:
	@for dir in $(PROJECTS); do \
           if test -f $$dir/Makefile; then \
             echo -e "\n--------------- $$dir (rel)"; \
             $(MAKE) -C $$dir release || exit -1; \
           fi; \
        done
	@echo ""

clean:
	@for dir in $(PROJECTS); do \
           if test -f $$dir/Makefile; then \
             echo -e "\n--------------- Removing temporary files for $$dir"; \
             $(MAKE) -C $$dir clean || exit -1; \
           fi; \
         done
	@echo ""

# sequentially to avoid same sub-target in sub-make invoked twice
all: override PROJECTS = $(PROJECTS_ALL)
all:
	$(MAKE) all-debug
	$(MAKE) all-release

all-debug: override PROJECTS = $(PROJECTS_ALL)
all-debug: debug

all-release: override PROJECTS = $(PROJECTS_ALL)
all-release: release

tags:
	@echo -e "Generating tags for Emacs"
	@(ctags -e `find ../.. -name "*.h" -o -name "*.cpp"`)

.PHONY: clean debug default release tags
