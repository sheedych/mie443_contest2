#include <algorithm>
#include <limits>
#include <cmath>
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

#define DISTANCE_TO_BOX 0.6

std::vector<RobotPose> boxesToRobotPoses(Boxes boxes)
{
    int numBoxes = boxes.coords.size();
    std::vector<RobotPose> poses;

    for (int i = 0; i < numBoxes; i++)
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

float distanceHeuristic(RobotPose pose1, RobotPose pose2)
{
    float dx = pose1.x - pose2.x;
    float dy = pose1.y - pose2.y;
    return sqrt(dx * dx + dy * dy);
}

std::vector<RobotPose> solveTravellingSalesman(std::vector<RobotPose> poses, RobotPose currentPose)
{
    poses.insert(poses.begin(), currentPose);

    int numPoses = poses.size();
    float adjMatrix[numPoses][numPoses];

    for (int i = 0; i < numPoses; i++)
    {
        for (int j = 0; j < numPoses; j++)
        {
            adjMatrix[i][j] = distanceHeuristic(poses[i], poses[j]);
        }
    }

    std::vector<int> poseIndices(numPoses - 1);
    for (int i = 1; i < numPoses; i++)
    {
        poseIndices[i - 1] = i;
    }

    std::vector<std::vector<int>> possiblePaths;

    do
    {
        std::vector<int> path = poseIndices;
        path.insert(path.begin(), 0);
        path.push_back(0);
        possiblePaths.push_back(path);
    } while (std::next_permutation(poseIndices.begin(), poseIndices.end()));

    std::vector<int> bestPath;
    float bestPathLength = std::numeric_limits<float>::max();

    for (int i = 0; i < possiblePaths.size(); i++)
    {
        float currentPathLength = 0;
        std::vector<int> path = possiblePaths[i];
        for (int j = 1; j < path.size(); j++)
        {
            currentPathLength += distanceHeuristic(poses[path[j]], poses[path[j - 1]]);
        }

        if (currentPathLength < bestPathLength)
        {
            bestPathLength = currentPathLength;
            bestPath = path;
        }
    }

    std::vector<RobotPose> bestPoses;
    for (int i = 1; i < bestPath.size(); i++)
    {
        bestPoses.push_back(poses[bestPath[i]]);
    }

    return bestPoses;
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

    // get robotPose
    ros::Duration(2).sleep();
    ros::spinOnce();

    // print out robot pose
    std::cout << "Robot pose: " << std::endl;
    std::cout << " x: " << robotPose.x << " y: " << robotPose.y << " phi: "
              << robotPose.phi << std::endl;

    std::vector<RobotPose> bestPath = solveTravellingSalesman(poses, robotPose);

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    for (int i = 0; i < bestPath.size(); i++)
    {
        RobotPose pose = bestPath[i];
        std::cout << "current pose: " << pose.x << " " << pose.y << " " << pose.phi << std::endl;
        Navigation::moveToGoal(pose.x, pose.y, pose.phi);
    }

    while (ros::ok())
    {
    }

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
        //std::vector<float> coords = laserData.getClosestObjCoords();
        //std::cout << "First laser: " << coords[0] << " Second Laser: " << coords[1] << std::endl;
        int id = imagePipeline.getTemplateID(boxes, laserData);
        std::cout << id << std::endl;
        ros::Duration(0.01).sleep();
    }
    return 0;
}