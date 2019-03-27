// client as the position info publisher
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include "aubo_demo/PublishPositionAction.h"
#include "std_msgs/Float32.h"

typedef actionlib::SimpleActionClient<aubo_demo::PublishPositionAction> Client;

//action完成后调用此函数
void doneCb(const actionlib::SimpleClientGoalState& state,
            const aubo_demo::PublishPositionResultConstPtr& result)
{
    ROS_INFO("Finsh Move!");
    //任务完成就关闭节点
    ros::shutdown();
}
//action的目标任务发送给server且开始执行时，调用此函数
void activeCb()
{
   ROS_INFO("Goal is active! Begin to move.");
}

//action任务在执行过程中，server对过程有反馈则调用此函数
void feedbackCb(const aubo_demo::PublishPositionFeedbackConstPtr& feedback)
{
    //将服务器的反馈输出
    ROS_INFO("Move time: %d", feedback->move_time);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_position_client");
    if(argc != 4)
    {
        ROS_INFO("usage: move_position_client X Y Z");
        return 1;
    }
    //创建一个action的client，指定action名称为”move_position”
    Client client("move_position", true);
    ROS_INFO("Waiting for action server to start");
    client.waitForServer();
    ROS_INFO("Action server started");
    
    aubo_demo::PublishPositionGoal goal;
    std_msgs::Float32 position_x; 
    std_msgs::Float32 position_y;
    std_msgs::Float32 position_z;

    position_x.data = atof(argv[1]);
    position_y.data = atof(argv[2]);
    position_z.data = atof(argv[3]);
    std::cout << position_x.data << std::endl;
    std::cout << position_y.data << std::endl;
    std::cout << position_z.data << std::endl;
    goal.object_position.push_back(position_x.data);
    goal.object_position.push_back(position_y.data);
    goal.object_position.push_back(position_z.data);
    // goal.object_position.push_back(-0.195);
    // goal.object_position.push_back(-0.039);
    // goal.object_position.push_back(0.661);
    //把action的任务目标发送给服务器，绑定上面定义的各种回调函数
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    // client.sendGoal(goal);
    // client.waitForResult(ros::Duration(5.0)); //Blocks untill this goal finishes

    // if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    //     printf("Yay! The dishes are now clean\n");
    // }
    // printf("Current State: %s\n", client.getState().toString().c_str());
    // return 0;
}

