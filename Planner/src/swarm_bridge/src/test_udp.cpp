#include <ros/ros.h>
#include <traj_utils/MINCOTraj.h>
#include <ros/serialization.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#define UDP_PORT 8081
#define BUF_LEN 1048576 

enum MESSAGE_TYPE {
    ONE_TRAJ = 101
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "udp_traj_receiver");
    ros::NodeHandle nh("~");

    ros::Publisher traj_pub = nh.advertise<traj_utils::MINCOTraj>("/received_traj", 10);

    // 创建 UDP 套接字
    int server_fd;
    struct sockaddr_in address;
    int opt = 1;

    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
        perror("socket failed");
        return -1;
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        close(server_fd);
        return -1;
    }

    address.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.255", &address.sin_addr);
    address.sin_port = htons(UDP_PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        close(server_fd);
        return -1;
    }

    ROS_INFO("UDP trajectory receiver started on port %d", UDP_PORT);

    char buffer[BUF_LEN];
    struct sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);
    double time_now = ros::Time::now().toSec();
    while (ros::ok()) {
        int valread = recvfrom(server_fd, buffer, BUF_LEN, 0,
                               (struct sockaddr *)&sender_addr, &addr_len);
        if (valread < 0) {
            perror("recvfrom error");
            continue;
        }

        // 检查消息类型
        MESSAGE_TYPE type = *((MESSAGE_TYPE *)buffer);
        if (type != ONE_TRAJ) {
            ROS_WARN_THROTTLE(5, "Received unknown message type: %d", type);
            continue;
        }

        // 反序列化
        traj_utils::MINCOTraj traj_msg;
        uint8_t *ptr = (uint8_t *)(buffer + sizeof(MESSAGE_TYPE));
        uint32_t msg_size = *((uint32_t *)ptr);
        ptr += sizeof(uint32_t);

        if (valread != (int)(sizeof(MESSAGE_TYPE) + sizeof(uint32_t) + msg_size)) {
            ROS_WARN("Message size mismatch: received %d, expected %lu",
                     valread, sizeof(MESSAGE_TYPE) + sizeof(uint32_t) + msg_size);
            continue;
        }

        ros::serialization::IStream stream(ptr, msg_size);
        ros::serialization::deserialize(stream, traj_msg);

        // 输出轨迹信息
        ROS_INFO("Received trajectory: drone_id=%d, order=%d",
                 traj_msg.drone_id, traj_msg.order);
        ROS_INFO("start_time: %f,piece num: %ld",traj_msg.start_time.toSec() - time_now, traj_msg.inner_x.size());
        // ROS_INFO("Duration = %ld",traj_msg.duration);

        traj_pub.publish(traj_msg);
    }

    close(server_fd);
    return 0;
}