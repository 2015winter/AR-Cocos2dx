#ifndef __HELLOWORLD_SCENE_H__
#define __HELLOWORLD_SCENE_H__

#include "cocos2d.h"
#include "track/Tracker.h"
#include "track/PnPProblem.h"
// #include <vector>

class HelloWorld : public cocos2d::Layer
{
public:
    static cocos2d::Scene* createScene(cv::Mat*, cv::Mat*, pthread_mutex_t*);
    // static cocos2d::Scene* createScene(char* imageBuffer, pthread_mutex_t*);

    virtual bool init();
    
    // a selector callback
    void menuCloseCallback(cocos2d::Ref* pSender);
    
    // implement the "static create()" method manually
    CREATE_FUNC(HelloWorld);
public:
	void update(float dt);
	static cv::Mat* drawing_frame;
	static cv::Mat* grayImage;
	// static cv::Mat gray_frame;
	// static cv::Mat* image;
	cocos2d::Sprite3D* sprite3d_tortoise;
	cocos2d::Camera* _spriteCamera;
	cocos2d::Sprite* capture;
	static void* grayProcess(void* ptr);
protected:
	FACETRACKER::Tracker* model;
	PnPProblem pnp_detection;		
	cv::Mat tri;
	cv::Mat con;
	std::vector<cv::Point3f> list_points3d;
	std::vector<int> list_indexes;
	bool failed = true;	
	bool useExtrinsicGuess = false;
	bool selectModel = true;
};

#endif // __HELLOWORLD_SCENE_H__
