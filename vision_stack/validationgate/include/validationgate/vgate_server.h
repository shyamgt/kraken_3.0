#ifndef VGATE_H
#define VGATE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ip_msgs/vgateAction.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <actionmsg/vgateAction.h>
#include <actionlib/server/simple_action_server.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <iostream>
#include <fstream>
#include <validationgate/taskheader.h>
#include <validationgate/resultheader.h>

using namespace std;
using namespace cv;

typedef actionlib::SimpleActionServer<actionmsg::vgateAction> Server;


class Vgate
{
private:
    ros::NodeHandle _n;
    Server _s;
    image_transport::ImageTransport _it;
    image_transport::Subscriber _sub;
    image_transport::Publisher _pub;
    actionmsg::vgateFeedback _feedback;
    actionmsg::vgateResult _result;
    Mat _image, _imageHSV, _imageBW;
    Scalar _lowerThresh, _upperThresh;
    Mat _kernelDilateErode;
    std::string _actionName;
    cv_bridge::CvImage _finalImage;
    sensor_msgs::ImagePtr _finalImagemsg;
    vector<Vec4i> _lines;
    Point2i _coordinate[4], _rodB;
public:
    Vgate(std::string _name);
    void executCB(const actionmsg::vgateGoalConstPtr &_goal);
    void imageCallBack(const sensor_msgs::ImageConstPtr &msg);
    float angleWrtY(const Point2f &v1, const Point2i &v2);
    Point2i rotatePoint(const Point2i &v1, const Point2i &v2, float angle);
    bool detectVgate();
    void getAlignment();
    ~Vgate();
};

#endif
