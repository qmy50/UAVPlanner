#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <traj_utils/MINCOTraj.h>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define UDP_PORT 8081
#define BUF_LEN 1048576

using namespace std;

int udp_server_fd_, udp_send_fd_;
ros::Subscriber one_traj_sub_;
ros::Publisher one_traj_pub_;
string udp_ip_;
int drone_id_;
char udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];
struct sockaddr_in addr_udp_send_;
traj_utils::MINCOTraj MINCOTraj_msg_;

enum MESSAGE_TYPE
{
    ONE_TRAJ = 101  
};

int init_broadcast(const char *ip, const int port)
{
    int fd;
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
    {
        ROS_ERROR("[bridge_node] Socket sender creation error!");
        exit(EXIT_FAILURE);
    }

    int so_broadcast = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
    {
        cout << "Error in setting Broadcast option";
        exit(EXIT_FAILURE);
    }

    addr_udp_send_.sin_family = AF_INET;
    addr_udp_send_.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
    return fd;
}

int udp_bind_to_port(const int port, int &server_fd)
{
    struct sockaddr_in address;
    int opt = 1;

    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    return server_fd;
}

template <typename T>
int serializeTopic(const MESSAGE_TYPE msg_type, const T &msg)
{
    auto ptr = (uint8_t *)(udp_send_buf_);
    *((MESSAGE_TYPE*)ptr) = msg_type;
    ptr += sizeof(MESSAGE_TYPE);

    namespace ser = ros::serialization;
    uint32_t msg_size = ser::serializationLength(msg);
    *((uint32_t *)ptr) = msg_size;
    ptr += sizeof(uint32_t);

    ser::OStream stream(ptr, msg_size);
    ser::serialize(stream, msg);
    return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

template <typename T>
int deserializeTopic(T &msg) //从 UDP 接收缓冲区 udp_recv_buf_ 中解析出一条 ROS 消息，并将其存入传入的 msg 引用中
{
    auto ptr = (uint8_t *)(udp_recv_buf_ + sizeof(MESSAGE_TYPE));
    uint32_t msg_size = *((uint32_t *)ptr);
    ptr += sizeof(uint32_t);

    namespace ser = ros::serialization;
    ser::IStream stream(ptr, msg_size);
    ser::deserialize(stream, msg);
    return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

void one_traj_sub_udp_cb(const traj_utils::MINCOTrajPtr &msg)
{
    ROS_WARN_THROTTLE(1.0,"Get own traj !");
    int len = serializeTopic(MESSAGE_TYPE::ONE_TRAJ, *msg);
    if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
               (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
    {
        ROS_ERROR("UDP SEND ERROR (traj)!!!");
    }else{
      ROS_WARN_THROTTLE(1.0,"Broadcast own traj !");
    }
}

void udp_recv_fun()
{
    int valread;
    struct sockaddr_in addr_client;
    socklen_t addr_len;

    if (udp_bind_to_port(UDP_PORT, udp_server_fd_) < 0)
    {
        ROS_ERROR("[bridge_node] Socket receiver creation error!");
        exit(EXIT_FAILURE);
    }

    while (true)
    {
        if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0,
                                (struct sockaddr *)&addr_client, &addr_len)) < 0)
        {
            perror("recvfrom() < 0, error:");
            exit(EXIT_FAILURE);
        }

        MESSAGE_TYPE type = *((MESSAGE_TYPE *)udp_recv_buf_);
        if (type == MESSAGE_TYPE::ONE_TRAJ)
        {
            if (valread == deserializeTopic(MINCOTraj_msg_))
            {
                one_traj_pub_.publish(MINCOTraj_msg_);
            }
            else
            {
                ROS_ERROR("Received message length mismatch for traj");
            }
        }
        else
        {
            ROS_WARN("Unknown message type received, ignoring");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_bridge_traj");
    ros::NodeHandle nh("~");

    nh.param("broadcast_ip", udp_ip_, string("127.0.0.255"));
    nh.param("drone_id", drone_id_, -1);

    if (drone_id_ == -1)
    {
        ROS_WARN("[swarm bridge] Wrong drone_id!");
        exit(EXIT_FAILURE);
    }

    one_traj_sub_ = nh.subscribe("/broadcast_traj_from_planner", 100,
                                 one_traj_sub_udp_cb, ros::TransportHints().tcpNoDelay());
    one_traj_pub_ = nh.advertise<traj_utils::MINCOTraj>("/broadcast_traj_to_planner", 100);
    boost::thread udp_recv_thd(udp_recv_fun);
    udp_recv_thd.detach();
    ros::Duration(0.1).sleep();

    udp_send_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT);

    cout << "[swarm_bridge_traj] start running (only trajectory exchange)" << endl;

    ros::spin();

    close(udp_server_fd_);
    close(udp_send_fd_);
    return 0;
}