#include "navigation.h"
#include <tf/tf.h>  // tf 라이브러리
#include <cmath>    // for math functions

Navigation::Navigation() : roll(0.0), pitch(0.0), heading(0.0), vel_linear(0.0), vel_angular(0.0) {}

// odom 콜백 함수 정의
void Navigation::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 쿼터니언 값 가져오기
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    // 쿼터니언을 오일러 각(Roll, Pitch, Yaw)으로 변환
    tfScalar roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);  // RPY: Roll, Pitch, Yaw

    // 변환된 값을 사용
    this->roll = roll;
    this->pitch = pitch;
    this->heading = yaw;

    // 위치 설정
    position.first = msg->pose.pose.position.x;
    position.second = msg->pose.pose.position.y;

    // 선형 속도와 각속도 설정
    vel_linear = msg->twist.twist.linear.x;
    vel_angular = msg->twist.twist.angular.z;

    // std::cout << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;
    // std::cout << "position_x: " << position.first << " position_y: " << position.second << std::endl;
    // std::cout << "vel_linear: " << vel_linear << ", vel_angular: " << vel_angular << std::endl;
}

// 위치 설정
void Navigation::setPosition(std::pair<float, float> _position) {
    this->position = _position;
}

// 오도메트리(roll, pitch, heading) 설정
void Navigation::setOdometry(float _roll, float _pitch, float _heading) {
    this->roll = _roll;
    this->pitch = _pitch;
    this->heading = _heading;
}

// 위치 반환
std::pair<float, float> Navigation::getPosition() {
    return position;
}

// 헤딩 반환
float Navigation::getHeading() {
    return heading;
}

// 선형 속도 반환
float Navigation::getLinearVel() {
    return vel_linear;
}

// 각속도 반환
float Navigation::getAngularVel() {
    return vel_angular;
}
