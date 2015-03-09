##################################################
# PROJECT: Thor - ROBOTIS Inc.
##################################################

#---------------------------------------------------------------------
# C++ COMPILER, COMPILER FLAGS, AND TARGET PROGRAM NAME
#---------------------------------------------------------------------
DIR_DXL    = ./..
DIR_OBJS    = ../build

INSTALL_ROOT = /usr/local
 
MAJ_VERSION = 2
MIN_VERSION = 0
REV_VERSION = 0

TARGET      = libdxl.so
TARGET1     = $(TARGET).$(MAJ_VERSION)
TARGET2     = $(TARGET).$(MAJ_VERSION).$(MIN_VERSION)
TARGET3     = $(TARGET).$(MAJ_VERSION).$(MIN_VERSION).$(REV_VERSION)

CHK_DIR_EXISTS = test -d
PRINT       = echo
STRIP       = strip
AR          = ar
ARFLAGS     = cr
LD          = g++
LDFLAGS     = -shared -fPIC #-Wl,-soname,dxl
LD_CONFIG   = ldconfig
CP          = cp
CP_ALL      = cp -r
RM          = rm
RM_ALL      = rm -rf
SYMLINK     = ln -s
MKDIR       = mkdir
CC          = gcc
CX          = g++
CCFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c $(INCLUDES) -fPIC -g
CXFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c $(INCLUDES) -fPIC -g
# CXFLAGS    += -DDEBUG
INCLUDES   += -I$(DIR_DXL)/include

#---------------------------------------------------------------------
# Required external libraries
#---------------------------------------------------------------------
#LIBRARIES  += -lpthread
#LIBRARIES  += -ljpeg
#LIBRARIES  += -lpthread 
#LIBRARIES  += -ldl
#LIBRARIES  += -lncurses 
#LIBRARIES  += -lrt

#---------------------------------------------------------------------
# Framework Files
#---------------------------------------------------------------------
SOURCES  = src/serialport.cpp \
           src/dynamixel.cpp \
           src/bulkread.cpp \

OBJECTS=$(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))


#---------------------------------------------------------------------
# COMPILING RULES
#---------------------------------------------------------------------
$(TARGET): makedirs $(OBJECTS)
	#$(AR) $(ARFLAGS) $(DIR_THOR)/lib/$(TARGET) $(OBJECTS)
	$(LD) $(LDFLAGS) -o $(DIR_DXL)/lib/$(TARGET) $(OBJECTS) $(LIBS)

makedirs:
	mkdir -p ../build
	mkdir -p ../lib

clean:
	rm -f $(OBJECTS) $(DIR_DXL)/lib/$(TARGET)

install: $(TARGET)
    # copy the binaries into the lib directory
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/lib || $(MKDIR) $(INSTALL_ROOT)/lib
	-$(CP) "$(DIR_DXL)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET)"
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET1)"
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET2)"
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET3)"

    # copy the headers into the include directory
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/include || $(MKDIR) $(INSTALL_ROOT)/include
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/include/libdxl || $(MKDIR) $(INSTALL_ROOT)/include/libdxl
	$(CP_ALL) $(DIR_DXL)/include/* $(INSTALL_ROOT)/include/

	$(LD_CONFIG)

uninstall:
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET1)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET2)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET3)

	$(RM_ALL) $(INSTALL_ROOT)/include/libdxl

reinstall: uninstall install


#---------------------------------------------------------------------
# Make rules for all .c and .cpp files in each directory
#---------------------------------------------------------------------

$(DIR_OBJS)/%.o: $(DIR_DXL)/src/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: $(DIR_DXL)/src/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@
	


#---------------------------------------------------------------------
# END OF MAKEFILE
#---------------------------------------------------------------------
