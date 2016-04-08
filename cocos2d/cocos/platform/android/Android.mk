LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OPENCV_INSTALL_MODULES:=on
OPENCV_CAMERA_MODULES:=off
OPENCV_LIB_TYPE:=STATIC
include E:/Libraries/OpenCV-2.4.10-android-sdk/sdk/native/jni/OpenCV.mk

LOCAL_MODULE := cocos2dxandroid_static

LOCAL_MODULE_FILENAME := libcocos2dandroid

LOCAL_SRC_FILES := \
CCApplication-android.cpp \
CCCommon-android.cpp \
CCDevice-android.cpp \
CCGLViewImpl-android.cpp \
CCFileUtils-android.cpp \
javaactivity-android.cpp \
CCEnhanceAPI-android.cpp \
jni/DPIJni.cpp \
jni/IMEJni.cpp \
jni/Java_org_cocos2dx_lib_Cocos2dxAccelerometer.cpp \
jni/Java_org_cocos2dx_lib_Cocos2dxBitmap.cpp \
jni/Java_org_cocos2dx_lib_Cocos2dxHelper.cpp \
jni/Java_org_cocos2dx_lib_Cocos2dxRenderer.cpp \
jni/JniHelper.cpp \
jni/TouchesJni.cpp \
jni/CocosPlayClient.cpp


LOCAL_C_INCLUDES := $(LOCAL_PATH) \
                    $(LOCAL_PATH)/.. \
                    $(LOCAL_PATH)/../.. \
                    E:/Libraries/vuforia-sdk-android-5-5-9/build/include \
                    E:/Libraries/opencv2410/build/include

#include E:/Libraries/OpenCV-2.4.10-android-sdk/sdk/native/jni/OpenCV.mk

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH) \
						   E:/Libraries/opencv2410/build/include

LOCAL_EXPORT_LDLIBS := -lGLESv1_CM \
                       -lGLESv2 \
                       -lEGL \
                       -llog \
                       -landroid

LOCAL_CPPFLAGS := -Wno-extern-c-compat

LOCAL_EXPORT_CPPFLAGS := -Wno-extern-c-compat

include $(BUILD_STATIC_LIBRARY)
