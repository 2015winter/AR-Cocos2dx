#include "HelloWorldScene.h"

#if CC_TARGET_PLATFORM == CC_PLATFORM_ANDROID
#include <android/log.h>
#include <jni.h>
#include <pthread.h>
#define  LOG_TAG    "myErr"
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#endif

USING_NS_CC;
pthread_mutex_t* mutex1 = NULL;
// cv::Mat HelloWorld::gray_frame = cv::Mat(640, 480, CV_8UC1);
cv::Mat* HelloWorld::drawing_frame = NULL;
cv::Mat* HelloWorld::grayImage = NULL;
// static char* imageBuffer = NULL;
// Scene* HelloWorld::createScene(char* image, pthread_mutex_t* mutex)
Scene* HelloWorld::createScene(cv::Mat* gray, cv::Mat* channel3Mat, pthread_mutex_t* mutex)
{
    // LOGE("width3 : %d ,height3 : %d", mat->rows, mat->cols);

    // 'scene' is an autorelease object
    auto scene = Scene::create();

    // drawing_frame.push_back(mat);
    drawing_frame = channel3Mat;
    grayImage = gray;
    // imageBuffer = image;
    mutex1 = mutex;
    // 'layer' is an autorelease object
    auto layer = HelloWorld::create();

    // add layer as a child to scene
    scene->addChild(layer);

    // return the scene
    return scene;
}

//=============================================================================
// Draw only the 2D points
void draw2DPoints(cv::Mat image, std::vector<cv::Point2f> &list_points, cv::Scalar color)
{
    for (size_t i = 0; i < list_points.size(); i++)
    {
        cv::Point2f point_2d = list_points[i];

        // Draw Selected points
        cv::circle(image, point_2d, 4, color, -1, 8);
    }
}

//=============================================================================
void Draw(cv::Mat &image, cv::Mat &shape, const cv::Mat &con, const cv::Mat &tri, cv::Mat &visi) {
    int i, n = shape.rows / 2;
    cv::Point p1, p2;
    cv::Scalar c;

    //draw triangulation
    c = CV_RGB(0, 0, 0);
    for (i = 0; i < tri.rows; i++) {
        if (visi.at<int>(tri.at<int>(i, 0), 0) == 0 ||
                visi.at<int>(tri.at<int>(i, 1), 0) == 0 ||
                visi.at<int>(tri.at<int>(i, 2), 0) == 0)
            continue;
        p1 = cv::Point(shape.at<double>(tri.at<int>(i, 0), 0),
                       shape.at<double>(tri.at<int>(i, 0) + n, 0));
        p2 = cv::Point(shape.at<double>(tri.at<int>(i, 1), 0),
                       shape.at<double>(tri.at<int>(i, 1) + n, 0));
        cv::line(image, p1, p2, c);
        p1 = cv::Point(shape.at<double>(tri.at<int>(i, 0), 0),
                       shape.at<double>(tri.at<int>(i, 0) + n, 0));
        p2 = cv::Point(shape.at<double>(tri.at<int>(i, 2), 0),
                       shape.at<double>(tri.at<int>(i, 2) + n, 0));
        cv::line(image, p1, p2, c);
        p1 = cv::Point(shape.at<double>(tri.at<int>(i, 2), 0),
                       shape.at<double>(tri.at<int>(i, 2) + n, 0));
        p2 = cv::Point(shape.at<double>(tri.at<int>(i, 1), 0),
                       shape.at<double>(tri.at<int>(i, 1) + n, 0));
        cv::line(image, p1, p2, c);
    }
    //draw connections
    c = CV_RGB(0, 0, 255);
    for (i = 0; i < con.cols; i++) {
        if (visi.at<int>(con.at<int>(0, i), 0) == 0 ||
                visi.at<int>(con.at<int>(1, i), 0) == 0)
            continue;
        p1 = cv::Point(shape.at<double>(con.at<int>(0, i), 0),
                       shape.at<double>(con.at<int>(0, i) + n, 0));
        p2 = cv::Point(shape.at<double>(con.at<int>(1, i), 0),
                       shape.at<double>(con.at<int>(1, i) + n, 0));
        cv::line(image, p1, p2, c, 1);
    }
    //draw points
    for (i = 0; i < n; i++) {
        if (visi.at<int>(i, 0) == 0)
            continue;
        p1 = cv::Point(shape.at<double>(i, 0), shape.at<double>(i + n, 0));
        c = CV_RGB(255, 0, 0);
        cv::circle(image, p1, 2, c);
    }
    return;
}

// on "init" you need to initialize your instance
bool HelloWorld::init()
{
    //////////////////////////////
    // 1. super init first
    if ( !Layer::init() )
    {
        return false;
    }

    // auto glview = Director::getInstance()->getOpenGLView();
    // glview->setFrameSize(640, 480);
    // Director::getInstance()->setOpenGLView(glview);
    // Size frameSize = glview->getFrameSize();
    // LOGE("frameSize: %d, %d", frameSize.width, frameSize.height);

    auto designResolutionSize = Director::getInstance()->getWinSize();
    Size visibleSize = Director::getInstance()->getVisibleSize();
    Vec2 origin = Director::getInstance()->getVisibleOrigin();
    LOGE("designResolutionSize is %f, %f", designResolutionSize.width, designResolutionSize.height);
    const char* ftFile = "/sdcard/face/face2.tracker";
    const char* conFile = "/sdcard/face/face.con";
    const char* triFile = "/sdcard/face/face.tri";
    const char* xmlFile = "/sdcard/face/face.xml";

    // Intrinsic camera parameters: UVC WEBCAM         
    double width = visibleSize.width, height = visibleSize.height;        // image size
    double f = 1081.804810;                     // focal length in mm
    double sx = 1280.000000, sy = 720.000000;                   // sensor size

    double params_WEBCAM[] = { width*f / sx,   // fx
        height*f / sy,  // fy
        width / 2,      // cx
        height / 2 };    // cy
    this->pnp_detection.init(params_WEBCAM);

    this->model = new FACETRACKER::Tracker(ftFile);
    this->tri = FACETRACKER::IO::LoadTri(triFile);
    this->con = FACETRACKER::IO::LoadCon(conFile);
    FACETRACKER::IO::LoadXml(xmlFile, this->list_points3d, this->list_indexes);

    this->sprite3d_tortoise = Sprite3D::create("face/header.obj");
    this->sprite3d_tortoise->setTexture("face/header.jpg");
    this->sprite3d_tortoise->setOpacity(50.0f);
    //this->sprite3d_tortoise->setScale(1.2f);
    this->sprite3d_tortoise->setScale(0.81f);
    this->sprite3d_tortoise->setCameraMask((unsigned short)CameraFlag::USER2);
    this->sprite3d_tortoise->setPosition3D(Vec3(0, 0, -800));
    this->addChild(sprite3d_tortoise);

    this->capture = Sprite::create("HelloWorld2.png");
    // this->capture->setAnchorPoint(Vec2(0.0f, 0.0f));
    this->capture->setPosition3D(Vec3(0, 0, -1100));
    this->capture->setCameraMask((unsigned short)CameraFlag::USER2);
    this->capture->setGlobalZOrder(-1);
    this->addChild(this->capture);

    this->_spriteCamera = Camera::createPerspective(5.419f, (GLfloat)visibleSize.width / visibleSize.height, 1081.804810, 5000);

    this->_spriteCamera->setCameraFlag(CameraFlag::USER2);
    this->_spriteCamera->setPosition3D(Vec3(0, 0, 1081.804810));
    this->_spriteCamera->lookAt(Vec3(0, 0, 0), Vec3(0, 1, 0));
    this->addChild(_spriteCamera);

    /////////////////////////////
    // 2. add a menu item with "X" image, which is clicked to quit the program
    //    you may modify it.

    // add a "close" icon to exit the progress. it's an autorelease object
    auto closeItem = MenuItemImage::create(
                         "CloseNormal.png",
                         "CloseSelected.png",
                         CC_CALLBACK_1(HelloWorld::menuCloseCallback, this));

    closeItem->setPosition(Vec2(origin.x + visibleSize.width - closeItem->getContentSize().width / 2 ,
                                origin.y + closeItem->getContentSize().height / 2));

    // create menu, it's an autorelease object
    auto menu = Menu::create(closeItem, NULL);
    menu->setPosition(Vec2::ZERO);
    this->addChild(menu, 1);

    /////////////////////////////
    // 3. add your codes below...

    // add a label shows "Hello World"
    // create and initialize a label

/*    auto label = Label::createWithTTF("Hello World", "fonts/Marker Felt.ttf", 24);

    // position the label on the center of the screen
    label->setPosition(Vec2(origin.x + visibleSize.width / 2,
                            origin.y + visibleSize.height - label->getContentSize().height));

    // add the label as a child to this layer
    this->addChild(label, 1);

    // add "HelloWorld" splash screen"
    auto sprite = Sprite::create("HelloWorld.png");

    // position the sprite on the center of the screen
    sprite->setPosition(Vec2(visibleSize.width / 2 + origin.x, visibleSize.height / 2 + origin.y));

    // add the sprite as a child to this layer
    this->addChild(sprite, 0);*/

    this->scheduleUpdate();

    return true;
}

int count = 0;
void HelloWorld::update(float dt)
{
    // if(imageBuffer == NULL)
    //     LOGE("IMAGE IS NULL");
    // else
    //     LOGE("IMAGE GET IT");
    // LOGE("IMAGE SIZE: %f, %f",(*image).rows, (*image).cols);
    // if((*drawing_frame).data == NULL)
    //     LOGE("IMAGE GET NULL");

    bool fcheck = false;
    double scale = 1;
    int fpd = 30;
    bool show = true;
    std::vector<int> wSize1(1);
    std::vector<int> wSize2(3);
    wSize1[0] = 7;
    wSize2[0] = 11;
    wSize2[1] = 9;
    wSize2[2] = 7;
    int nIter = 5;
    double clamp = 3, fTol = 0.01;

    // pthread_t pid;
    // int ret = pthread_create(&pid, NULL, HelloWorld::grayProcess, NULL);
    // if(ret != 0);

    //track this image
    std::vector<int> wSize;
    if (this->failed)
        wSize = wSize2;
    else
        wSize = wSize1;
    cv::Mat tempGrayMat;
    cv::Mat tempMat;
    pthread_mutex_lock(mutex1);
    tempMat = (*drawing_frame).clone();
    pthread_mutex_unlock(mutex1);
    cv::flip(tempMat, tempMat, 1);
    cvtColor(tempMat, tempGrayMat, CV_RGB2GRAY);
    cv::Mat tempRGBMat;
    cvtColor(tempGrayMat, tempRGBMat, CV_GRAY2RGB);

//Added by Sunny on 2016-1-20
clock_t start2, finish2;
double totaltime2;
start2 = clock(); 
count++;
if(count == 1)
    LOGE("~~~~~~~~~~~~~~~~~~~start of frame~~~~~~~~~~~~~~~~");
    LOGE("~~~~~~~~~~~~~~~~~~The %d frames~~~~~~~~~~~~~~~~~~~~", count);
  if (tempGrayMat.data != NULL) 
  {
        if ((*this->model).Track(tempGrayMat, wSize, fpd, nIter, clamp, fTol, fcheck) == 0)
        {
            LOGE("Track Succeed!");
                         int idx = (*this->model)._clm.GetViewIdx();
                        this->failed = false;
                        Draw(tempMat, (*this->model)._shape, this->con, this->tri, (*this->model)._clm._visi[idx]);
                        std::vector<cv::Point2f> list_points2d(0);
                        cv::Mat shape = this->model->_shape.clone();
                        int n = shape.rows / 2;
                        for (int iter = 0; iter < this->list_indexes.size(); iter++)
                        {
                            int index = this->list_indexes[iter] - 1;
                            cv::Point2f p = cv::Point(shape.at<double>(index, 0), shape.at<double>(index +  n, 0));
                            list_points2d.push_back(p);
                        }
                        // Draw points
                        draw2DPoints(tempMat, list_points2d, cv::Scalar(0, 0, 255));
            
            // Estimate the pose using solvePNP approach
            this->pnp_detection.estimatePose(this->list_points3d, list_points2d,
            cv::ITERATIVE, this->useExtrinsicGuess);

            if (this->useExtrinsicGuess == false)
            {
                this->useExtrinsicGuess = true;
            }

            float rx = this->pnp_detection.get_R_vector().at<double>(2) * 180 / M_PI;
            float ry = 180 - this->pnp_detection.get_R_vector().at<double>(1) * 180 / M_PI;
            float rz = -this->pnp_detection.get_R_vector().at<double>(0) * 180 / M_PI;

            float tx = -this->pnp_detection.get_t_matrix().at<double>(0);
            float ty = this->pnp_detection.get_t_matrix().at<double>(1);
            float tz = this->pnp_detection.get_t_matrix().at<double>(2);

            this->sprite3d_tortoise->setRotation3D(Vec3(rx, ry, rz));
            this->sprite3d_tortoise->setPosition3D(Vec3(tx, ty, tz));
        }       
        else 
        {
            LOGE("Track Failure!");
            if (show) 
            {
                // cv::Mat R(tempMat, cvRect(0, 0, 150, 50));
                // R = cv::Scalar(0, 0, 255);
            }
        (*this->model).FrameReset();
        this->failed = true;
        }   
    }
    else
            LOGE("GrayMat gets null!");
if(count == 10)
    LOGE("~~~~~~~~~~~~~~~~end of 10 frames~~~~~~~~~~~~~~~~~~~");
if(count == 20)
    LOGE("~~~~~~~~~~~~~~~~end of 20 frames~~~~~~~~~~~~~~~~~~~");
if(count == 30)
    LOGE("~~~~~~~~~~~~~~~~end of 30 frames~~~~~~~~~~~~~~~~~~~");

    auto texture = new Texture2D;
    texture->initWithData(tempMat.data, tempMat.elemSize() * tempMat.cols * tempMat.rows, Texture2D::PixelFormat::RGB888,
                          tempMat.cols, tempMat.rows,
                          Size(tempMat.cols, tempMat.rows));
    this->capture->setTexture(texture);
//Added by Sunny on 2016-1-20
finish2 = clock();
totaltime2 = (double)(finish2 - start2) / CLOCKS_PER_SEC;
#if (CC_TARGET_PLATFORM == CC_PLATFORM_ANDROID)
     LOGE("Track time consuming in total: %lf", totaltime2);
#endif
    // if((*drawing_frame).type() == CV_8UC3)
    // LOGE("frame Size: %d, %d", (*drawing_frame).rows, (*drawing_frame).cols);
    // }
    // pthread_mutex_unlock(mutex1);

    // cv::Mat* gray_frame = NULL;
    // drawing_frame.push_back(*image);
    // cv::Mat frame = cv::Mat(640, 480, CV_8UC3);
    // pthread_mutex_lock(mutex1);
    // cv::cvtColor(*drawing_frame, gray_frame, cv::COLOR_GRAY2RGB);
    // pthread_mutex_unlock(mutex1);

    // drawing_frame.pop_back();
}

// void* HelloWorld::grayProcess(void* ptr)
// {
//     pthread_mutex_lock(mutex1);
//     if ((*drawing_frame).data != NULL)
//     {
//         cv::Mat gray_frame;
//         cv::cvtColor(*drawing_frame, gray_frame, CV_RGB2GRAY);
//         if (gray_frame.type() == CV_8UC1)
//             LOGE("gray frame Size: %d, %d", gray_frame.rows, gray_frame.cols);
//     }
//     pthread_mutex_unlock(mutex1);
// }

void HelloWorld::menuCloseCallback(Ref* pSender)
{
    Director::getInstance()->end();

#if (CC_TARGET_PLATFORM == CC_PLATFORM_IOS)
    exit(0);
#endif
}
