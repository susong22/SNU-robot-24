#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <nav_msgs/Odometry.h>
#include <utility>  // for std::pair
#include <tf/tf.h>  // for quaternion to euler conversion

class Navigation {
public:
    Navigation();

    // /odom 토픽에서 데이터를 받아오는 콜백 함수
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // 위치와 방향을 설정하는 함수들
    void setPosition(std::pair<float, float> position);
    void setOdometry(float roll, float pitch, float heading);

    // 위치, 속도, 각도를 반환하는 함수들
    std::pair<float, float> getPosition();
    float getHeading();
    float getLinearVel();
    float getAngularVel();

private:
    std::pair<float, float> position;  // x, y 좌표
    float roll, pitch, heading;  // 오일러 각도
    float vel_linear, vel_angular;  // 선형 및 각속도
};

#endif  // NAVIGATION_H
