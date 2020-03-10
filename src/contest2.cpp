#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include "LaserData.hpp"


LaserData laserData;
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
     laserData.updateState(msg);
}  

int main(int argc, char **argv)
{
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    ros::Subscriber laser_sub = n.subscribe("scan", 10, &laserCallback);
    // Initialize box coordinates and templates
    Boxes boxes;
    if (!boxes.load_coords() || !boxes.load_templates())
    {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for (int i = 0; i < boxes.coords.size(); ++i)
    {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: "
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.
    while (ros::ok())
    {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        // Transform boxes.coords to desired positions (nodes)
        // start and end node are given as starting position
        // apply heuristic to get cost of edges between nodes
        // build graph
        // solve travelling salesman problem
        // navigate between the nodes in the order from above
        //std::vector<float> coords = laserData.getClosestObjCoords();
        //std::cout << "First laser: " << coords[0] << " Second Laser: " << coords[1] << std::endl;
        int id = imagePipeline.getTemplateID(boxes, laserData);
        std::cout << id << std::endl;
        ros::Duration(0.01).sleep();
    }
    return 0;
}
