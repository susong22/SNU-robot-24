#include "lidar.h"
#include <limits>
#include <cmath>

#define RAD2DEG(x) ((x) * 180.0 / M_PI)  // 라디안 -> 디그리 변환 함수

Lidar::Lidar() 
    : min_distance_right(std::numeric_limits<float>::infinity()), 
      min_distance_middle(std::numeric_limits<float>::infinity()), 
      min_distance_left(std::numeric_limits<float>::infinity()),
      min_angle_right(0.0), min_angle_middle(0.0), min_angle_left(0.0) {}

// /scan 데이터를 구독해서 처리하는 콜백 함수
void Lidar::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // 초기화
    min_distance_right = std::numeric_limits<float>::infinity();
    min_distance_middle = std::numeric_limits<float>::infinity();
    min_distance_left = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float angle = msg->angle_min + i * msg->angle_increment;
        float degree_angle = RAD2DEG(angle);  // 라디안에서 도(degree)로 변환

        if(msg->ranges[i] == 0.0 || msg->ranges[i] >= 1.0) {
            continue;
        }


        if (degree_angle >= 195 && degree_angle <= 270) {
            // Right view, 90?
            if (msg->ranges[i] < min_distance_right) {
                min_distance_right = msg->ranges[i];
                min_angle_right = degree_angle;
            }
        } else if (degree_angle >= 165 && degree_angle <= 195) {
            // Middle view
            if (msg->ranges[i] < min_distance_middle) {
                min_distance_middle = msg->ranges[i];
                min_angle_middle = degree_angle;
            }
        } else if (degree_angle >= 90 && degree_angle <= 165) {
            // Left view
            if (msg->ranges[i] < min_distance_left) {
                min_distance_left = msg->ranges[i];
                min_angle_left = degree_angle;
            }
        }
    }
}

// Right view에서 최소 거리와 각도를 반환하는 함수
float Lidar::getMinDistanceRight() {
    return min_distance_right;
}

float Lidar::getMinAngleRight() {
    return min_angle_right;
}

// Middle view에서 최소 거리와 각도를 반환하는 함수
float Lidar::getMinDistanceMiddle() {
    return min_distance_middle;
}

float Lidar::getMinAngleMiddle() {
    return min_angle_middle;
}

// Left view에서 최소 거리와 각도를 반환하는 함수
float Lidar::getMinDistanceLeft() {
    return min_distance_left;
}

float Lidar::getMinAngleLeft() {
    return min_angle_left;
}
