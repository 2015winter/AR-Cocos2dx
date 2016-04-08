#Description
To implement AR based on Cocos2dx
## Requestion
- Cocos2dx Environments
  + Android Studio
  + Android SDK
  + NDK
- Vuforia SDK 5.5.9
- OpenCV SDK 2.4.10
- Windows 10 platforms
##explanation
To implement AR based on Cocos2dx, I use Vuforia instead of OpenCV to get Camera frame, beca-
use OpenCV to get it with frameRate 30fps, Vuforia get it with 48fps, of course, the framerate
is influenced with different phone, the sample is tested on MeiZu, mlnote in China, which is 
performented not well, I hope to have someone to help me improve the project, when track parts
is added to the project, the framerate is performented so terrible.