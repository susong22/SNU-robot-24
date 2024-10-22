#ifndef LIDAR_H
#define LIDAR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <limits>  // for std::numeric_limits

class Lidar {
public:
    Lidar();

    // /scan 토픽에서 데이터를 받아오는 콜백 함수
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // 최소 거리값과 각도값을 반환하는 함수들
    float getMinDistanceRight();
    float getMinDistanceMiddle();
    float getMinDistanceLeft();

    float getMinAngleRight();
    float getMinAngleMiddle();
    float getMinAngleLeft();

private:
    // 오른쪽, 중간, 왼쪽 방향에 대한 최소 거리값과 각도
    float min_distance_right;
    float min_distance_middle;
    float min_distance_left;

    float min_angle_right;
    float min_angle_middle;
    float min_angle_left;

    // 초기화 값 설정
    void resetDistances() {
        min_distance_right = std::numeric_limits<float>::infinity();
        min_distance_middle = std::numeric_limits<float>::infinity();
        min_distance_left = std::numeric_limits<float>::infinity();
        min_angle_right = 0.0;
        min_angle_middle = 0.0;
        min_angle_left = 0.0;
    }
};

#endif  // LIDAR_H
