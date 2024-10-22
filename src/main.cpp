#include <ros/ros.h>
#include "lidar.h"
#include "navigation.h"

// 상수 정의
#define ROBOT_LINEAR_VELOCITY -0.1
#define ROBOT_ANGULAR_VELOCITY 0.3
#define ROBOT_ALIGNMENT_SENSITIVITY 0.8

// 모드 정의
enum RobotMode { DRIVE, STOP };

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_control_node");
    ros::NodeHandle nh;

    Lidar lidar;
    Navigation navigation;
    RobotMode mode = DRIVE;  // 초기 모드는 주행

    ros::Subscriber scan_sub = nh.subscribe("/scan", 1, &Lidar::scanCallback, &lidar);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, &Navigation::odomCallback, &navigation);

    ros::Publisher cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Rate loop_rate(10);  // 10Hz

    while (ros::ok()) {
        ros::spinOnce();

        // Left, Right, Middle 거리값 가져오기
        float left_distance = lidar.getMinDistanceLeft();
        float right_distance = lidar.getMinDistanceRight();
        float middle_distance = lidar.getMinDistanceMiddle();

        geometry_msgs::Twist cmd_;
        cmd_.linear.x = ROBOT_LINEAR_VELOCITY;  // 상수로 정의된 기본 전진 속도

        if (mode == DRIVE) {
            // 중앙에 장애물이 있는지 확인
            if (middle_distance < 0.2) {
                mode = STOP;  // 정지 모드로 전환
                std::cout << "middle obstacle, stop!!!! distance: " << middle_distance << std::endl;
                cmd_.linear.x = 0.0;  // 정지
            } else {
                // 좌우 거리 비교 후 중앙을 유지하며 주행
                float distance_diff = left_distance - right_distance;
                cmd_.angular.z = - ROBOT_ALIGNMENT_SENSITIVITY * distance_diff;
                std::cout << "left distance: " << left_distance << ", right distance: " << right_distance << ", angular.z: " << cmd_.angular.z << ", linear.x: " << cmd_.linear.x << std::endl;

                // 장애물에 너무 가까운 경우 속도를 줄이거나 정지
                if (left_distance < 0.2 || right_distance < 0.2) {
                    cmd_.linear.x = -0.05;  // 속도 줄이기
                    if (left_distance < 0.18 || right_distance < 0.18) {
                        cmd_.linear.x = 0.0;
                        cmd_.angular.z = 0.0; // 완전히 정지
                    }
                }
            }
        } else if (mode == STOP) {
            // 장애물이 사라지면 다시 주행 모드로 전환
            cmd_.linear.x = 0.0;
            cmd_.angular.z = 0.0;
            std::cout << "cmd_linear.x: " << cmd_.linear.x << std::endl;
            if (middle_distance >= 0.2) {
                mode = DRIVE;  // 주행 모드로 전환
                ROS_INFO("Obstacle cleared! Resuming movement.");
            }
        }

        cmd_pub_.publish(cmd_);  // 명령어 전송
        loop_rate.sleep();
    }

    return 0;
}
