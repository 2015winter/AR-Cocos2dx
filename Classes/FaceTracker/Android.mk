LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_CPPFLAGS += -fexceptions

LOCAL_MODULE := mytrack_static

LOCAL_MODULE_FILENAME := libtracker_static

LOCAL_SRC_FILES := CLM.cc \
				   FCheck.cc \
				   FDet.cc \
				   IO.cc \
				   Patch.cc \
				   PAW.cc \
				   PDM.cc \
				   PnPProblem.cpp \
				   Tracker.cc

 LOCAL_C_INCLUDES := $(LOCAL_PATH)/FaceTracker \
 					 E:/Libraries/opencv2410/build/include
include $(BUILD_STATIC_LIBRARY)