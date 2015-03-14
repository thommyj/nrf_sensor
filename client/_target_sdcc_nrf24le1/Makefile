##################################################
### Begin library-specific portion of Makefile ###
##################################################

# Add directories here that appear under the <top level>/src folder that should not be compiled
SUBSRCDIRSTOFILTER = 
TARGETPACKAGE = _target_sdcc_nrf24le1_32

# Add include directories (outside this folder) that the sub makes need to know about (separated by a space)
# These MUST be the relative path from within this directory to the desired directory (no absolute paths)
export EXTERNINCDIRS = ../../../lib_nrf24le1_sdk/include ../../../lib_nrf24le1_sdk/$(TARGETPACKAGE)/include/ ../../shared/
export EXTERNLIBDIRS = ../../../lib_nrf24le1_sdk/$(TARGETPACKAGE)/lib 

# Functions needed by this makefile and the one that builds the source packages (MakefileSrc)
export ECHO = @echo
export RM = rm
export SED = sed
export MKDIR = mkdir
export TR = tr
export BLACKHOLE = /dev/null
export PWD = pwd
export CD = cd
export LS = ls
PACKIHX = packihx
TAIL = tail

# Program to use for the linker
LK = sdcc

# Flags for above program when calling it from the command line
LFLAGS = --model-large --code-loc 0x0000 --code-size 0x4000 --xram-loc 0x0000 --xram-size 0x400 -o $(MAINIHX) $(LIBDIRS) $(LIBFILES) $(OBJFILES)

################################################
### End library-specific portion of Makefile ###
################################################


export TARGETNAME := $(shell $(PWD) | $(SED) -e 's_/.*/__' | $(SED) -e 's_\S.*\\__')

TOPLEVELREL = ..
APPNAME := $(shell $(CD) .. && $(PWD) | $(SED) -e 's_/.*/__' | $(SED) -e 's_\S.*\\__' && $(CD) $(TARGETNAME))
TARGETTOSRCDIRREL = $(TOPLEVELREL)/src
SRCTOTARGETDIRREL = ../$(TARGETNAME)
LOCALSRCRELTOTARGET = ../../$(TARGETNAME)

export TARGETDIR = $(TARGETNAME)
export TARGETOBJDIRREL = obj
export TARGETDEPDIRREL = dep
export TARGETFLASHDIRREL = flash
export TARGETSRCMAKEFILE = $(LOCALSRCRELTOTARGET)/MakefileSrc
export TARGETRELTOSUBSRCREL = ../../$(TARGETNAME)

SUBSRCDIRSRAW := $(subst /.,,$(strip $(shell $(CD) $(TARGETTOSRCDIRREL) && $(LS) -d */. && $(CD) $(SRCTOTARGETDIRREL))))
SUBSRCDIRSRAWFILT := $(filter-out $(SUBSRCDIRSTOFILTER),$(SUBSRCDIRSRAW))
SUBSRCDIRS := $(foreach dir,$(SUBSRCDIRSRAWFILT),$(TARGETTOSRCDIRREL)/$(dir))
OBJDIRS := $(foreach dir,$(SUBSRCDIRSRAWFILT),$(TARGETOBJDIRREL)/$(dir))

FLASHDIR = flash
OBJFILES = $(foreach _dir,$(OBJDIRS),$(strip $(shell ls $(_dir)/*.rel)))
MAINHEX = $(FLASHDIR)/main.hex
MAINIHX = $(FLASHDIR)/main.ihx

LIBFILES := $(foreach _dir,$(EXTERNLIBDIRS),$(strip $(shell $(LS) $(_dir))))
LIBDIRS := $(foreach _dir,$(EXTERNLIBDIRS),-L $(_dir))


all: build link finish

build:
	$(ECHO)
	$(ECHO) "Building target '$(TARGETNAME)' for project '$(APPNAME)'"
	@$(MAKE) $(SUBSRCDIRS)
	
$(SUBSRCDIRS):
	$(ECHO)
	$(ECHO) "Building project section '$(@F)'"
	$(MAKE) -C $@ all
	$(ECHO) "Finished building project section '$(@F)'"

link:
	$(ECHO)
	$(ECHO) "Linking project"
	$(LK) $(LFLAGS)
	$(ECHO) "Finished linking project!"
	$(ECHO)
	$(ECHO) "Converting hex file"
	$(PACKIHX) $(MAINIHX) > $(MAINHEX)
	$(ECHO) "Finished converting hex file"
	$(ECHO)
	$(ECHO) "Memory statistics:"
	$(TAIL) -n 100 $(TARGETFLASHDIRREL)/main.mem

finish:
	$(ECHO)
	$(ECHO) "Finished building target '$(TARGETNAME)' for project '$(APPNAME)'"

clean:
	$(ECHO)
	$(ECHO) "Cleaning target '$(TARGETNAME)' for project '$(APPNAME)'"
	$(if $(TARGETOBJDIRREL),$(RM) -rf $(TARGETOBJDIRREL)/*)
	$(if $(TARGETDEPDIRREL),$(RM) -rf $(TARGETDEPDIRREL)/*)
	$(if $(TARGETFLASHDIRREL),$(RM) -rf $(TARGETFLASHDIRREL)/*)
	$(ECHO) "Finished cleaning target '$(TARGETNAME)' for project '$(APPNAME)'"

.PHONY: $(SUBSRCDIRS)
