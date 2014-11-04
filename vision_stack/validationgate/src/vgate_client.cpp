#include <iostream>
#include <ros/ros.h>
#include <ip_msgs/vgateAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <validationgate/resultheader.h>
#include <validationgate/taskheader.h>

using namespace std;

typedef actionlib::SimpleActionClient<actionmsg::vgateAction> Client;

void done_cb(const actionlib::SimpleClientGoalState& state,const ip_msgs::vgateResultConstPtr& result)
{
    ROS_INFO("Action server process complete");
}



void feedback_cb(const ip_msgs::vgateFeedbackConstPtr& feedback_msg)
{
    ROS_INFO("feed back %f %d",feedback_msg->errory,feedback_msg->errorx);
}

void activeCb()
{
  ROS_INFO("Goal just went active");
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "vgate_client");
    Client _client("vgate");
    ROS_INFO("vgatenclient started : waiting for server to start.");
    _client.waitForServer();
    ROS_INFO("vgateserver started.");
    actionmsg::vgateGoal _goal;
    _goal.order = DETECT_VGATE;
    _client.sendGoal(_goal);
    bool _actionStatus = _client.waitForResult(ros::Duration(15.0));
    if(_actionStatus)
    {
        actionlib::SimpleClientGoalState _state = _client.getState();
        ROS_INFO("vgate_client : Action finished: %s",_state.toString().c_str());
    }
    else
    {
        ROS_INFO("vgate_client : Action did not finish within specified time.");
        _client.cancelGoal();
    }

    _goal.order = ALIGN_VGATE;
    _client.sendGoal(_goal);
    _actionStatus = _client.waitForResult(ros::Duration(15.0));
    if(_actionStatus == true)
    {
        actionlib::SimpleClientGoalState _state = _client.getState();
        ROS_INFO("vgate_client : Action finished: %s",_state.toString().c_str());
    }
    else
    {
        ROS_INFO("vgate_client : Action did not finish within specified time.");
        _client.cancelGoal();
    }
	return 0;
}
