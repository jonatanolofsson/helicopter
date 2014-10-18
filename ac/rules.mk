# Template rules.mk file.
include $(MAKEDIR)/header.mk
###############################################################################

### Change this middle section for your project.

### Source subdirectories
#~ BUILDDIRS += $(BUILD_PATH)/$(d)/getter

### Local flags: these control how the compiler gets called.
LOCAL_INCLUDE_DIR := $(d)/include
LIBSYRUP_PATH := $(SRCROOT)/../syrup
SYRUP_INCLUDES := -I$(LIBSYRUP_PATH)/syrup/include

CFLAGS_$(d) := $(WIRISH_INCLUDES) $(LIBMAPLE_INCLUDES) ${SYRUP_INCLUDES}
CFLAGS_$(d) += -I$(LOCAL_INCLUDE_DIR)
CXXFLAGS_$(d) :=  -I$(LOCAL_INCLUDE_DIR) -DMY_MAGIC_NUMBER=0x1eaf1ab5  \
	${SYRUP_INCLUDES} -std=c++0x -I${USER_MODULES}/../../ -DMAPLE_MINI \
	-fno-rtti
#~ LDFLAGS_$(d) := -Wl,-gc-sections

LIBS += -L${LIBSYRUP_PATH}/build -lsyrup

### Source files

cppSRCS_$(d) := ac.cpp
#~ cppSRCS_$(d) := serialtest.cpp

###############################################################################
include $(MAKEDIR)/footer.mk
