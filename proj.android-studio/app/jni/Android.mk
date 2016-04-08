LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := Vuforia-prebuilt
LOCAL_SRC_FILES := thirdpartylibs/libVuforia.so
#LOCAL_C_INCLUDES := E:/Libraries/vuforia-sdk-android-5-5-9/build/include
LOCAL_EXPORT_C_INCLUDES := E:/Libraries/vuforia-sdk-android-5-5-9/build/include
include $(PREBUILT_SHARED_LIBRARY)

#include $(CLEAR_VARS)
#LOCAL_MODULE := OpenCV-prebuilt
#LOCAL_SRC_FILES := thirdpartylibs/libopencv_java.so
##LOCAL_EXPORT_C_INCLUDES := E:/Libraries/opencv2410/build/include
#include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
#$(call import-add-path,$(LOCAL_PATH)/../../../Classes)
$(call import-add-path,$(LOCAL_PATH)/../../../cocos2d)
$(call import-add-path,$(LOCAL_PATH)/../../../cocos2d/external)
$(call import-add-path,$(LOCAL_PATH)/../../../cocos2d/cocos)

LOCAL_MODULE := cocos2dcpp_shared

LOCAL_MODULE_FILENAME := libcocos2dcpp

LOCAL_SRC_FILES := hellocpp/main.cpp \
                   ../../../Classes/AppDelegate.cpp \
                   ../../../Classes/HelloWorldScene.cpp

LOCAL_C_INCLUDES := $(LOCAL_PATH)/../../../Classes \
					$(LOCAL_PATH)../../../Classes/FaceTracker

# _COCOS_HEADER_ANDROID_BEGIN
# _COCOS_HEADER_ANDROID_END

LOCAL_STATIC_LIBRARIES := cocos2dx_static
LOCAL_STATIC_LIBRARIES += mytrack_static
LOCAL_SHARED_LIBRARIES += Vuforia-prebuilt
#LOCAL_SHARED_LIBRARIES += OpenCV-prebuilt


# _COCOS_LIB_ANDROID_BEGIN
# _COCOS_LIB_ANDROID_END
include $(BUILD_SHARED_LIBRARY)

$(call import-module,.)
$(call import-module,../Classes/FaceTracker)

# _COCOS_LIB_IMPORT_ANDROID_BEGIN
# _COCOS_LIB_IMPORT_ANDROID_END
