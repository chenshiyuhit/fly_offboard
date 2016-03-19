#include "../include/flytrack.h"


void tldposeCallBack(const geometry_msgs::Pose2DConstPtr &msg)
{
    x     = msg->x;
    y     = msg->y;
    theta = msg->theta;

    printf("x = %f , y = %f , theta = %f \n",x,y,theta);
}

void localposeCallBack(const geometry_msgs::PoseStampedConstPtr &msg)
{
    uav_x = msg->pose.position.x;
    uav_y = msg->pose.position.y;
    uav_z = msg->pose.position.z;

    printf("uav_x = %f , uav_y = %f , uav_z = %f \n",uav_x,uav_y,uav_z);
}

void takeoff(Vector3d bp)
{
    geometry_msgs::PoseStamped takeoff_sp;
    takeoff_sp.pose.position.x = bp(0);
    takeoff_sp.pose.position.y = bp(1);
    takeoff_sp.pose.position.z = bp(2) ;
    takeoff_sp.pose.orientation.w = 1.0;
    takeoff_sp.pose.orientation.x = 0.0;
    takeoff_sp.pose.orientation.y = 0.0;
    takeoff_sp.pose.orientation.z = 0.0;

    pose_pub_ = takeoff_sp;
}

void track(Vector3d t)
{
    geometry_msgs::PoseStamped track_sp;
    track_sp.pose.position.x = t(0);
    track_sp.pose.position.y = t(1);
    track_sp.pose.position.z = t(2) ;
    track_sp.pose.orientation.w = 1.0;
    track_sp.pose.orientation.x = 0.0;
    track_sp.pose.orientation.y = 0.0;
    track_sp.pose.orientation.z = 0.0;

    pose_pub_ = track_sp;
}

void addheight(Vector3d h)
{
    geometry_msgs::PoseStamped addheight_sp;

    h(2)=h(2)+0.1;

    addheight_sp.pose.position.x = h(0);
    addheight_sp.pose.position.y = h(1);
    addheight_sp.pose.position.z = h(2) ;
    addheight_sp.pose.orientation.w = 1.0;
    addheight_sp.pose.orientation.x = 0.0;
    addheight_sp.pose.orientation.y = 0.0;
    addheight_sp.pose.orientation.z = 0.0;

    pose_pub_ = addheight_sp;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "flytrack");

    Vector3d beginpoint_;
    Vector3d endpoint_;

    ros::NodeHandle l_nh_;
    ros::Subscriber tldpose_sub_;
    ros::Subscriber localpose_sub_;
    ros::Publisher  pos_sp_pub_;

    tldpose_sub_  = l_nh_.subscribe("/TLDpose",1000,tldposeCallBack);
    localpose_sub_= l_nh_.subscribe("/firefly/ground_truth/pose",1000,localposeCallBack);
    pos_sp_pub_   = l_nh_.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose",1000);

    l_nh_.param<double>("beginpoint/x",beginpoint_(0), 0.0);
    l_nh_.param<double>("beginpoint/y",beginpoint_(1), 0.0);
    l_nh_.param<double>("beginpoint/z",beginpoint_(2), 1.5);
    l_nh_.param<double>("endpoint/x",endpoint_(0), 0.0);
    l_nh_.param<double>("endpoint/y",endpoint_(1), 0.0);
    l_nh_.param<double>("endpoint/z",endpoint_(2), 1.5);

    while(ros::ok())
    {
        if(search_flag)
        {
            ROS_INFO("Taking off and hovering at the height 1.5m, waiting the target selection!!!");
            takeoff(beginpoint_);

            ros::spinOnce();
            if(x>0 && y>0)
            {
                search_flag = false;
                track_flag  = true;
            }
        }

        if(track_flag)
        {
            ROS_INFO("Tracking the ground target!!!");

            ros::spinOnce();
            if(x<150 && y<110 && x>0 && y>0)
            {
                beginpoint_(0) = uav_x - 0.05;
                beginpoint_(1) = uav_y + 0.05*(y/x);
                track(beginpoint_);
            }
            if(x<150 && y>130 && x>0)
            {
                beginpoint_(0) = uav_x - 0.05;
                beginpoint_(1) = uav_y - 0.05*(y/x);
                track(beginpoint_);
            }
            if(x>170 && y<110 && y>0)
            {
                beginpoint_(0) = uav_x + 0.05;
                beginpoint_(1) = uav_y + 0.05*(y/x);
                track(beginpoint_);
            }
            if(x>170 && y>130)
            {
                beginpoint_(0) = uav_x + 0.05;
                beginpoint_(1) = uav_y - 0.05*(y/x);
                track(beginpoint_);
            }
            if(x>150 && y>110 && x<170 && y<130)
            {
                beginpoint_(0) = uav_x;
                beginpoint_(1) = uav_y;
                track(beginpoint_);
            }


            ros::spinOnce();
            if(x==0 && y==0)
            {
                track_flag  = false;
                lost_flag   = true;
            }
        }

        if(lost_flag)
        {
            ROS_INFO("The target is lost, add height to let it come back!!!");

            ros::spinOnce();
            endpoint_(0) = uav_x;
            endpoint_(1) = uav_y;
            addheight(endpoint_);

            ros::spinOnce();
            if(x>0 && y>0)
            {
                lost_flag   = false;
                track_flag  = true;
            }
        }

        pos_sp_pub_.publish(pose_pub_);
    }

    return 0;
}

