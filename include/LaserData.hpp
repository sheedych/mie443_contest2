#ifndef LASERDATA_H
#define LASERDATA_H

#include <stdint.h>
#include <sensor_msgs/LaserScan.h>

#define NUM_LASERS 639
#define MIN_ANGLE -0.521567881107
#define MAX_ANGLE 0.524276316166
#define ANGLE_INCREMENT 0.00163668883033
#define RANGE_MIN 0.449999988079
#define RANGE_MAX 10.0

class LaserData
{
private:
    float *_laserArray;

public:
    uint32_t NumLasers = NUM_LASERS;
    float MinAngle = MIN_ANGLE;
    float MaxAngle = MAX_ANGLE;
    float AngleIncrement = ANGLE_INCREMENT;
    float RangeMin = RANGE_MIN;
    float RangeMax = RANGE_MAX;

    LaserData();
    void updateState(const sensor_msgs::LaserScan::ConstPtr &msg);
    float getMinDistance();
    float getLeftDistance();
    float getRightDistance();
    std::vector<float> getClosestObjCoords();
};

#endif
