#include <algorithm>
#include <iostream>
#include <fstream>
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
#define MAX_IMAGE_CAPTURE_ATTEMPTS 5

enum FsmState
{
    INITIALIZING,
    CAPTURING_IMAGE,
    MOVING_TO_GOAL,
    FINISHED
};

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

RobotPose poseToBox(RobotPose currentPose)
{
    float poseX = currentPose.x;
    float poseY = currentPose.y;
    float posePhi = currentPose.phi;

    float boxPoseX = poseX + DISTANCE_TO_BOX * cos(posePhi);
    float boxPoseY = poseY + DISTANCE_TO_BOX * sin(posePhi);
    float boxPosePhi = posePhi - M_PI;

    return RobotPose(boxPoseX, boxPoseY, boxPosePhi);
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

    // Robot pose object + subscribers.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    ros::Subscriber laser_sub = n.subscribe("scan", 10, &laserCallback);

    // Initialize box coordinates and templates.
    Boxes boxes;
    if (!boxes.load_coords() || !boxes.load_templates())
    {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }

    int numBoxes = boxes.coords.size();

    // Print out box coordinates.
    for (int i = 0; i < numBoxes; ++i)
    {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: "
                  << boxes.coords[i][2] << std::endl;
    }

    // Transform box coordinates to robot poses.
    std::vector<RobotPose> poses = boxesToRobotPoses(boxes);

    // Print out robot poses.
    for (int i = 0; i < numBoxes; ++i)
    {
        std::cout << "Pose coordinates: " << std::endl;
        std::cout << i << " x: " << poses[i].x << " y: " << poses[i].y << " phi: "
                  << poses[i].phi << std::endl;
    }

    // Ensure that current robot pose is updated by subscriber.
    ros::Duration(2).sleep();
    ros::spinOnce();

    // Print out current robot pose.
    std::cout << "Robot pose: " << std::endl;
    std::cout << " x: " << robotPose.x << " y: " << robotPose.y << " phi: "
              << robotPose.phi << std::endl;

    // Determine best path.
    std::vector<RobotPose> bestPath = solveTravellingSalesman(poses, robotPose);

    // Initialize image object and subscriber.
    ImagePipeline imagePipeline(n);

    for (int i = 0; i < bestPath.size(); i++)
    {
        RobotPose pose = bestPath[i];
        std::cout << "current pose: " << pose.x << " " << pose.y << " " << pose.phi << std::endl;
    }

    // Initialize FSM variables.
    FsmState state = INITIALIZING;
    int currentPoseIndex = 0;
    int imageCaptureAttempts = 0;

    // Execute strategy.
    std::ofstream myfile;
    myfile.open("visiting_order.txt");
    int timesseen1 = 0;
    int timesseen2 = 0;
    int timesseen3 = 0;
    int timesseen4 = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        if (state == INITIALIZING)
        {
            state = MOVING_TO_GOAL;
        }
        else if (state == MOVING_TO_GOAL)
        {
            RobotPose nextPose = bestPath[currentPoseIndex];
            Navigation::moveToGoal(nextPose.x, nextPose.y, nextPose.phi);

            currentPoseIndex++;
            if (currentPoseIndex <= numBoxes)
            {
                state = CAPTURING_IMAGE;
            }
            else
            {
                state = FINISHED;
            }
        }
        else if (state == CAPTURING_IMAGE)
        {
            int id = imagePipeline.getTemplateID(boxes, laserData);

            imageCaptureAttempts++;
            if (id >= 0 || imageCaptureAttempts == MAX_IMAGE_CAPTURE_ATTEMPTS)
            {
                // TODO do something with id.
                int lastIdx = currentPoseIndex - 1;
                RobotPose nextPose = bestPath[lastIdx];
                std::cout << "image id" << id << std::endl;
                if(id == 0) {
                    RobotPose currentBoxPose = poseToBox(nextPose);
                    timesseen1++;
                    std::cout << "Visited Raisin Bran\n";
                    myfile << "Visited Raisin Bran " << timesseen1 << " time(s) at x = " << currentBoxPose.x << ", y = "<< currentBoxPose.y << ", phi = " << currentBoxPose.phi << "\n";
                }
                else if (id == 1) {
                    RobotPose currentBoxPose = poseToBox(nextPose);                   
                    timesseen2++;
                    std::cout << "Visited Cinnamon Toast Crunch\n";
                    myfile << "Visited Cinnamon Toast Crunch " << timesseen2 << " time(s) at x = " << currentBoxPose.x << ", y = "<< currentBoxPose.y << ", phi = " << currentBoxPose.phi << "\n";
                }
                else if (id == 2) {
                    RobotPose currentBoxPose = poseToBox(nextPose);
                    timesseen3++;
                    std::cout << "Visited Rice Krispies\n";
                    myfile << "Visited Rice Krispies " << timesseen3 << " time(s) at x = " << currentBoxPose.x << ", y = "<< currentBoxPose.y << ", phi = " << currentBoxPose.phi << "\n";
                }
                else {
                    RobotPose currentBoxPose = poseToBox(nextPose);
                    timesseen4++;
                    std::cout << "Visited Nothing\n";
                    myfile << "Visited Nothing " << timesseen4 << " time(s) at x = " << currentBoxPose.x << ", y = "<< currentBoxPose.y << ", phi = " << currentBoxPose.phi << "\n";
                }
                imageCaptureAttempts = 0;
                state = MOVING_TO_GOAL;
            }
        }
        else if (state == FINISHED)
        {
            myfile.close();
            // Do nothing.
        }
        ros::Duration(0.01).sleep();
    }
    return 0;
}
