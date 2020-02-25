#pragma once

#include <boxes.h>
#include <robot_pose.h>

#define NUM_POSES 6

class PoseGraph
{
public:
    PoseGraph(Boxes boxes, RobotPose initialPose);
    RobotPose *getRobotPoses();

private:
    RobotPose robotPoses[NUM_POSES] = {
        RobotPose(0, 0, 0),
        RobotPose(0, 0, 0),
        RobotPose(0, 0, 0),
        RobotPose(0, 0, 0),
        RobotPose(0, 0, 0),
        RobotPose(0, 0, 0),
    };
    float graph[NUM_POSES][NUM_POSES];

    RobotPose *posesFromBoxes(Boxes boxes);
};