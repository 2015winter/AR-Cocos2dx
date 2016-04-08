/****************************************************************************
Copyright (c) 2013-2014 Chukong Technologies Inc.

http://www.cocos2d-x.org

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
****************************************************************************/

#include "platform/CCPlatformConfig.h"
#if CC_TARGET_PLATFORM == CC_PLATFORM_ANDROID

#include "CCApplication-android.h"
#include "CCGLViewImpl-android.h"
#include "base/CCDirector.h"
#include "base/CCEventCustom.h"
#include "base/CCEventType.h"
#include "base/CCEventDispatcher.h"
#include "renderer/CCGLProgramCache.h"
#include "renderer/CCTextureCache.h"
#include "renderer/ccGLStateCache.h"
#include "2d/CCDrawingPrimitives.h"
#include "platform/android/jni/JniHelper.h"
#include <android/log.h>
#include <jni.h>

//Added by winter on 2016-3-28
#include <Vuforia/Vuforia.h>
#include <Vuforia/CameraDevice.h>
#include <Vuforia/UpdateCallback.h>
#include <Vuforia/State.h>
#include <Vuforia/Frame.h>
#include <Vuforia/Image.h>
#include <opencv2/opencv.hpp>
#include <pthread.h>
// #include <WinDef.h>
#include "time.h"
#ifndef _CLOCK_T_DEFINED
typedef long clock_t;
#define _CLOCK_T_DEFINED
#endif

#define  LOG_TAG    "myErr"
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480

void cocos_android_app_init(JNIEnv* env) __attribute__((weak));

using namespace cocos2d;


typedef unsigned char byte;
//Added by winter on 2016-3-28
// static JavaVM* javaVM = 0;
static cv::Mat channel2Mat = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC2);
static cv::Mat channel3Mat = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);
static cv::Mat grayMat = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
byte* grayImage = new byte[FRAME_HEIGHT*FRAME_WIDTH];
static jobject activityObj = 0;
pthread_mutex_t mutex;

#define RED(a)      ((((a) & 0xf800) >> 11) << 3)
#define GREEN(a)    ((((a) & 0x07e0) >> 5) << 2)
#define BLUE(a)     (((a) & 0x001f) << 3)

void toGreyscale(byte *rgbs, int widthIn, int heightIn, byte *greyscales)
{
    const int textSize = 128;
    int x,y;
    short* rgbPtr = (short*)rgbs;
    byte *greyPtr = greyscales;

    // rgbs arrives in RGB565 (16 bits) format
    for (y=0; y<textSize; y++)
    {
        for (x=0; x<textSize; x++)
        {
            short pixel = *(rgbPtr++);
            int r = RED(pixel);
            int g = GREEN(pixel);
            int b = BLUE(pixel);

            *(greyPtr++) = (byte)((r+g+b) / 3);
        }
        rgbPtr += widthIn - textSize;
    }
}

extern "C"
{
    JNIEXPORT jint JNI_OnLoad(JavaVM *vm, void *reserved)
    {
        JniHelper::setJavaVM(vm);
        //Added by winter
        // javaVM = vm;

        cocos_android_app_init(JniHelper::getEnv());

        return JNI_VERSION_1_4;
    }


//Added by winter on 2016-3-28
    class Vuforia_updateCallback : public Vuforia::UpdateCallback
    {
        virtual void Vuforia_onUpdate(Vuforia::State& state)
        {
            // LOGE("Vuforia_onUpdateNative!");
//Added by Sunny on 2016-1-20
clock_t start2, finish2;
double totaltime2;
double totaltime = 0.0;
int count = 0;
start2 = clock(); 
            Vuforia::Image *imageRGB888 = NULL;
            Vuforia::Image* imageGRAY = NULL;
            Vuforia::Frame frame = state.getFrame();

            for (int i = 0; i < frame.getNumImages(); ++i) {
                const Vuforia::Image *image = frame.getImage(i);
                if (image->getFormat() == Vuforia::RGB888) {
                    imageRGB888 = (Vuforia::Image*)image;

                    break;
                }
                else if(image->getFormat() == Vuforia::GRAYSCALE){
                    imageGRAY = (Vuforia::Image*)image;
                    break;
                }
            }

            if (imageRGB888) {
                JNIEnv* env = 0;
                // JniHelper::getJavaVM()->AttachCurrentThread((void**)&env, NULL);

                if ((JniHelper::getJavaVM() != 0) && (activityObj != 0) 
                    &&(JniHelper::getJavaVM()->GetEnv((void**)&env, JNI_VERSION_1_4) == JNI_OK)
                    ) {

                    // JNIEnv* env = JniHelper::getEnv();
                    const short* pixels = (const short*) imageRGB888->getPixels();
                    int width = imageRGB888->getWidth();
                    int height = imageRGB888->getHeight();
                    int numPixels = width * height;
                    LOGE("Vuforia Image Size: %d, %d", width, height);

                    jbyteArray pixelArray = env->NewByteArray(numPixels * 3);
                    env->SetByteArrayRegion(pixelArray, 0, numPixels * 3, (const jbyte*) pixels);

                    // cv::Mat mat = cv::Mat(imageRGB888->getHeight(), imageRGB888->getWidth(), CV_8UC1, (unsigned char *)imageRGB888->getPixels());
                    cv::Mat temp3Channel = cv::Mat(imageRGB888->getHeight(), imageRGB888->getWidth(), CV_8UC3, (unsigned char *)imageRGB888->getPixels());
                    // toGreyscale((byte*)pixels, width, height, (byte*)grayImage);
            pthread_mutex_lock(&mutex);
                    // cv::cvtColor(mat, channel3Mat, CV_BGR5652BGR);
                    // cv::cvtColor(channel3Mat, channel3Mat, CV_BGR2RGB);                
                    // channel2Mat = mat;
                    channel3Mat = temp3Channel.clone();
                    // cv::cvtColor(temp3Channel, grayMat, CV_RGB2GRAY);   
                    // imageBuffer = (char *)env->GetByteArrayElements(pixelArray, 0);
            pthread_mutex_unlock(&mutex);
                    // imageBuffer = (char*)env->NewGlobalRef(pixelArray);
                    jclass javaClass = env->GetObjectClass(activityObj);
                    // LOGE("width : %d ,height : %d", channel2Mat.rows, channel2Mat.cols);

                    // jmethodID method = env-> GetMethodID(javaClass, "processCameraImage", "([BII)V");
                    // env->CallVoidMethod(activityObj, method, pixelArray, width, height);
                    env->DeleteLocalRef(pixelArray);
                    // if (channel2Mat.data == NULL)
                        // LOGE("channel2Mat is NULL");
                    // else
                    //     LOGE("channel2Mat get it!");
                    // JniHelper::getJavaVM()->DetachCurrentThread();
                }
            }
                else if (imageGRAY) {
                JNIEnv* env = 0;
                // JniHelper::getJavaVM()->AttachCurrentThread((void**)&env, NULL);

                if ((JniHelper::getJavaVM() != 0) && (activityObj != 0) 
                    &&(JniHelper::getJavaVM()->GetEnv((void**)&env, JNI_VERSION_1_4) == JNI_OK)
                    ) {

                    // JNIEnv* env = JniHelper::getEnv();
                    const short* pixels = (const short*) imageGRAY->getPixels();
                    int width = imageGRAY->getWidth();
                    int height = imageGRAY->getHeight();
                    int numPixels = width * height;


                    jbyteArray pixelArray = env->NewByteArray(numPixels * 1);
                    env->SetByteArrayRegion(pixelArray, 0, numPixels * 1, (const jbyte*) pixels);

                    cv::Mat gray_frame = cv::Mat(480, 640, CV_8UC1, (unsigned char*)pixels, 0); 
                    // cv::Mat mat = cv::Mat(imageRGB888->getHeight(), imageRGB888->getWidth(), CV_8UC1, (unsigned char *)imageRGB888->getPixels());
                    cv::Mat tempGray = cv::Mat(imageGRAY->getHeight(), imageGRAY->getWidth(), CV_8UC1, (unsigned char *)imageGRAY->getPixels());
            pthread_mutex_lock(&mutex);
                    // cv::cvtColor(temp3Channel, grayMat, CV_RGB2GRAY);  
                    grayMat = gray_frame; 
                    // imageBuffer = (char *)env->GetByteArrayElements(pixelArray, 0);
            pthread_mutex_unlock(&mutex);
                    jclass javaClass = env->GetObjectClass(activityObj);
                    env->DeleteLocalRef(pixelArray);
                }
            }
//Added by Sunny on 2016-1-20
finish2 = clock();
count++;
totaltime2 = (double)(finish2 - start2) / CLOCKS_PER_SEC;
totaltime += totaltime2;
double frameRate = (double)count/totaltime;
#if (CC_TARGET_PLATFORM == CC_PLATFORM_ANDROID)
     // LOGE("vuforia frameRate: %lf", frameRate);
#endif
        }
    };

    Vuforia_updateCallback updateCallback;

    JNIEXPORT void JNICALL
    Java_com_vuforia_samples_SampleApplication_SampleApplicationSession_onVuforiaInitializedNative(JNIEnv * env, jobject)
    {
        LOGE("onVuforiaInitalizeNative!");

        // Register the update callback where we handle the data set swap:
        Vuforia::registerCallback(&updateCallback);

        // Comment in to enable tracking of up to 2 targets simultaneously and
        // split the work over multiple frames:
        // Vuforia::setHint(Vuforia::HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 2);
    }

    JNIEXPORT void JNICALL
    Java_org_cocos2dx_lib_Cocos2dxActivity_initApplicationNative(
        JNIEnv* env, jobject obj)
    {
        activityObj = env->NewGlobalRef(obj);
    }

    JNIEXPORT void JNICALL
    Java_com_vuforia_samples_SampleApplication_SampleApplicationSession_startNativeCamera(JNIEnv *, jobject, jint camera)
    {
        // LOGE("Java_com_qualcomm_QCARSamples_ImageTargets_ImageTargets_startCamera");
        // Start the camera:
        if (!Vuforia::CameraDevice::getInstance().start())
            return;
        Vuforia::setFrameFormat(Vuforia::RGB888, true);
        // Vuforia::setFrameFormat(Vuforia::GRAYSCALE, true);
        /*    currentCamera = static_cast<Vuforia::CameraDevice::CAMERA_DIRECTION> (camera);

            // Initialize the camera:
            if (!Vuforia::CameraDevice::getInstance().init(currentCamera))
                return;
        const Vuforia::CameraCalibration& cameraCalibration =
                Vuforia::CameraDevice::getInstance().getCameraCalibration();
        Vuforia::Vec2F size = cameraCalibration.getSize();
        Vuforia::Vec2F focalLength = cameraCalibration.getFocalLength();
        Vuforia::Vec2F principalPoint = cameraCalibration.getPrincipalPoint();

        LOGE("%lf, %lf, %lf, %lf, %lf, %lf", size.data[0], size.data[1], focalLength.data[0], focalLength.data[1], principalPoint.data[0], principalPoint.data[1]);

            // Select the default camera mode:
            if (!Vuforia::CameraDevice::getInstance().selectVideoMode(
                                        Vuforia::CameraDevice::MODE_DEFAULT))
                return;

            // Configure the rendering of the video background
            configureVideoBackground();

            // Start the camera:
            if (!Vuforia::CameraDevice::getInstance().start())
                return;

            // Uncomment to enable flash
            //if(Vuforia::CameraDevice::getInstance().setFlashTorchMode(true))
            //    LOG("IMAGE TARGETS : enabled torch");

            // Uncomment to enable infinity focus mode, or any other supported focus mode
            // See CameraDevice.h for supported focus modes
            //if(Vuforia::CameraDevice::getInstance().setFocusMode(Vuforia::CameraDevice::FOCUS_MODE_INFINITY))
            //    LOG("IMAGE TARGETS : enabled infinity focus");

            // Start the tracker:
            // Vuforia::TrackerManager& trackerManager = Vuforia::TrackerManager::getInstance();
            // Vuforia::Tracker* objectTracker = trackerManager.getTracker(Vuforia::ObjectTracker::getClassType());
            // if(objectTracker != 0)
            //     objectTracker->start();
            */
    }


    JNIEXPORT void JNICALL
    Java_com_vuforia_samples_SampleApplication_SampleApplicationSession_stopNativeCamera(JNIEnv *, jobject)
    {
        LOGE("Java_com_vuforia_samples_ImageTargets_ImageTargets_stopCamera");

        // Stop the tracker:
        // Vuforia::TrackerManager& trackerManager = Vuforia::TrackerManager::getInstance();
        // Vuforia::Tracker* objectTracker = trackerManager.getTracker(Vuforia::ObjectTracker::getClassType());
        // if(objectTracker != 0)
        //     objectTracker->stop();

        Vuforia::CameraDevice::getInstance().stop();
        Vuforia::CameraDevice::getInstance().deinit();
    }


    JNIEXPORT void Java_org_cocos2dx_lib_Cocos2dxRenderer_nativeInit(JNIEnv*  env, jobject thiz, jint w, jint h)
    {
        // javaVM->AttachCurrentThread((void**)&env, NULL);
        auto director = cocos2d::Director::getInstance();
        auto glview = director->getOpenGLView();
        if (!glview)
        {
            glview = cocos2d::GLViewImpl::create("Android app");
            glview->setFrameSize(w, h);
            director->setOpenGLView(glview);
                    
            // LOGE("width1 : %d ,height1 : %d", channel2Mat.rows, channel2Mat.cols);
            // if(channel2Mat.data != NULL)
                cocos2d::Application::getInstance()->run(&grayMat, &channel3Mat, &mutex);
            // cocos2d::Application::getInstance()->run(imageBuffer, &mutex);
        }
        else
        {
            cocos2d::GL::invalidateStateCache();
            cocos2d::GLProgramCache::getInstance()->reloadDefaultGLPrograms();
            cocos2d::DrawPrimitives::init();
            cocos2d::VolatileTextureMgr::reloadAllTextures();

            cocos2d::EventCustom recreatedEvent(EVENT_RENDERER_RECREATED);
            director->getEventDispatcher()->dispatchEvent(&recreatedEvent);
            director->setGLDefaultValues();
        }
    }

    JNIEXPORT jintArray Java_org_cocos2dx_lib_Cocos2dxActivity_getGLContextAttrs(JNIEnv*  env, jobject thiz)
    {
        cocos2d::Application::getInstance()->initGLContextAttrs();
        GLContextAttrs _glContextAttrs = GLView::getGLContextAttrs();

        int tmp[6] = {_glContextAttrs.redBits, _glContextAttrs.greenBits, _glContextAttrs.blueBits,
                      _glContextAttrs.alphaBits, _glContextAttrs.depthBits, _glContextAttrs.stencilBits
                     };


        jintArray glContextAttrsJava = env->NewIntArray(6);
        env->SetIntArrayRegion(glContextAttrsJava, 0, 6, tmp);

        return glContextAttrsJava;
    }

    JNIEXPORT void Java_org_cocos2dx_lib_Cocos2dxRenderer_nativeOnSurfaceChanged(JNIEnv*  env, jobject thiz, jint w, jint h)
    {
        cocos2d::Application::getInstance()->applicationScreenSizeChanged(w, h);
    }

}

#endif // CC_TARGET_PLATFORM == CC_PLATFORM_ANDROID

