#include <cmath>
#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

#define DISTANCE_TO_BOX 0.5

std::vector<RobotPose> boxesToRobotPoses(Boxes boxes)
{
    int num_boxes = boxes.coords.size();
    std::vector<RobotPose> poses;

    for (int i = 0; i < num_boxes; i++)
    {
        std::vector<float> coords = boxes.coords[i];

        float boxX = coords[0];
        float boxY = coords[1];
        float boxPhi = coords[2];

        float poseX = boxX + DISTANCE_TO_BOX * cos(boxPhi);
        float poseY = boxY + DISTANCE_TO_BOX * sin(boxPhi);
        float posePhi = boxPhi + M_PI;

        poses.push_back(RobotPose(poseX, poseY, posePhi));
    }

    return poses;
}

int main(int argc, char **argv)
{
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // Robot pose object + subscriber.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    // Initialize box coordinates and templates
    Boxes boxes;
    if (!boxes.load_coords() || !boxes.load_templates())
    {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    // print out box coordinates
    for (int i = 0; i < boxes.coords.size(); ++i)
    {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: "
                  << boxes.coords[i][2] << std::endl;
    }
    // transform box coordinates to robot poses
    std::vector<RobotPose> poses = boxesToRobotPoses(boxes);

    // print out robot poses
    for (int i = 0; i < boxes.coords.size(); ++i)
    {
        std::cout << "Pose coordinates: " << std::endl;
        std::cout << i << " x: " << poses[i].x << " y: " << poses[i].y << " phi: "
                  << poses[i].phi << std::endl;
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

        // start and end node are given as starting position
        // apply heuristic to get cost of edges between nodes
        // build graph
        // solve travelling salesman problem
        // navigate between the nodes in the order from above

        // imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}