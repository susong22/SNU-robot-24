#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from cmd_interface_linux import CmdInterfaceLinux  # 필요에 따라 이 부분을 설정하세요.
from lipkg import LiPkg, LD08_LiPkg  # LiPkg 라이브러리 임포트 (해당 라이브러리가 Python으로 제공되어야 함)
import time
import math

ANGLE_TO_RADIAN = lambda angle: (angle * 3141.59 / 180000)

def main():
    # LiPkg 객체 생성
    pkg = LD08_LiPkg()
    ver = 8

    # ROS 초기화
    product_ver = "LD08"
    rospy.init_node(product_ver, anonymous=True)
    nh = rospy.Publisher(f'{product_ver}/scan', LaserScan, queue_size=1)

    # CmdInterfaceLinux 설정
    cmd_port = CmdInterfaceLinux(ver)
    device_list = cmd_port.GetCmdDevices()
    port_name = None
    for n in device_list:
        print(n[0], "   ", n[1])
        if "CP2102" in n[1]:
            port_name = n[0]

    if port_name:
        print(f"FOUND LiDAR_{product_ver} @port : {port_name}")
        
        def read_callback(byte, length):
            if pkg.Parse(byte, length):
                pkg.AssemblePacket()

        cmd_port.SetReadCallback(read_callback)
        cmd_port.Open(port_name)

        scan = LaserScan()
        scan.header.frame_id = "base_scan"
        scan.range_min = 0.0
        scan.range_max = 100.0

        rate = rospy.Rate(10)  # 10Hz로 루프 실행
        while not rospy.is_shutdown():
            if pkg.IsFrameReady():
                data = pkg.GetFrameData()
                scan.angle_min = ANGLE_TO_RADIAN(data.angle_min)
                scan.angle_max = ANGLE_TO_RADIAN(data.angle_max)
                scan.angle_increment = (scan.angle_max - scan.angle_min) / data.len
                scan.ranges = [0] * data.len
                scan.intensities = [0] * data.len

                for i in range(data.len):
                    scan.ranges[i] = data.distance[i] / 1000.0  # 거리를 미터로 변환
                    scan.intensities[i] = data.intensities[i]

                scan.header.stamp = rospy.Time.now()
                nh.publish(scan)
            
            rate.sleep()

    else:
        print(f"Can't find LiDAR {argv[1]}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
