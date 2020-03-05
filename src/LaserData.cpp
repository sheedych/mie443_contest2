#include <sensor_msgs/LaserScan.h>
#include <cmath>

#include "LaserData.hpp"

LaserData::LaserData()
{
    _laserArray = new float[NumLasers];
}

void LaserData::updateState(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // TODO use a memory copy method instead for performance
    for (int i = 0; i < NumLasers; i++)
    {
        _laserArray[i] = msg->ranges[i];
    }
}

std::vector<float> LaserData::getClosestObjCoords() {
    float minDist = 100;
    float threshold = 0.2;
    float rightLaser = -1;
    float leftLaser = -1;
    //first find the closes thing
    for(int i = 0; i < NUM_LASERS; i++) {
        minDist = std::min(minDist, _laserArray[i]);
    }
    std::cout << "The minimum distance is "<< minDist << std::endl;
    std::vector<float> ret;
    if(minDist > 1.5) {
        ret.push_back(-1);
        ret.push_back(-1);
        return ret;
    }
    //next, find the amount of the object that exists in that range
    //find the first instance of that
    float range = minDist + threshold;
    int i = 0;
    for(i = 0; i < NUM_LASERS; i++) {
        if((_laserArray[i] <= range)) {
            rightLaser = i;
            std::cout << "Right laser set" << std::endl;
            break;
        }
    }

    for(i; i < NUM_LASERS; i++) {
        if((_laserArray[i] > range) && (_laserArray[i] < 100)) {
            leftLaser = i - 1;
            std::cout << "Left laser set" << std::endl;
            break;
        }
    }
    ret.push_back(leftLaser);
    ret.push_back(rightLaser);
    return ret;

}

float LaserData::getMinDistance()
{
    float minDistance = std::numeric_limits<float>::infinity();
    for (int i = 200; i < 336; i++)
    {
        minDistance = std::min(minDistance, _laserArray[i]);
    }

    return minDistance;
}

float LaserData::getLeftDistance() 
{
    float leftDistance = 0;
    int leftLasers = 539;
    int leftUsed = 0;
    for (int i = leftLasers; i < NUM_LASERS; i++) {
        if(_laserArray[i] < 100)  {
            leftDistance += _laserArray[i];
            leftUsed++;
        }
    }
    leftDistance = leftDistance /((float) leftUsed);
    return leftDistance;
}

float LaserData::getRightDistance() 
{
    float rightDistance = 0;
    int rightLasers = 100;
    int rightUsed = 0;
    for (int i = 0; i < rightLasers; i++) {
        if(_laserArray[i] < 100)  {
            rightDistance += _laserArray[i];
            rightUsed++;
        }
    }
    rightDistance = rightDistance / ((float)rightUsed);
    return rightDistance;
}
