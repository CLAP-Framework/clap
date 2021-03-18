#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <thread>
#include <iostream>

#include <jeayeson/value.hpp>
#include <zzz_perception_msgs/DetectionBoxArray.h>
#include <zzz_perception_msgs/DetectionBox.h>
#include <zzz_perception_msgs/ObjectSignals.h>

#include <ros/ros.h>

#define PORT 6001
#define QUEUE 20
#define BUFFER_SIZE 1024
int conn;

#define  GREEN_LIGHT                 1
#define  YELLOW_LIGHT                2
#define  RED_LIGHT                   3
#define  GREEN_LIGHT_REMAINING_TIME  3

int main(int argc, char** argv) {
  ros::init(argc, argv, "obu_trafficlight");
  ros::NodeHandle node;
  // ros::NodeHandle priv_nh("~");
  ros::Publisher pub_obu = 
      node.advertise<zzz_perception_msgs::DetectionBoxArray>(
          "/zzz/perception/traffic_lights", 10);

  struct sockaddr_in server_sockaddr; 
  server_sockaddr.sin_family = AF_INET;
  server_sockaddr.sin_port = htons(PORT); 
  server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

  int ss; 
  if((ss = socket(AF_INET, SOCK_STREAM, 0))<0)  
  {  
    perror("server sockfd creation failed");  
    exit(EXIT_FAILURE);  
  }  
  // 设置套接字选项避免地址使用错误  
  int on=1;  
  if((setsockopt(ss,SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on))) < 0)  
  {  
    perror("setsockopt failed");  
    exit(EXIT_FAILURE);  
  }  
  if(bind(ss, (struct sockaddr* ) &server_sockaddr, sizeof(server_sockaddr))==-1)
  {
    perror("bind");
    exit(1);
  }
  if(listen(ss, QUEUE) == -1)
  {
      perror("listen");
      exit(1);
  }

  struct sockaddr_in client_addr;
  socklen_t length = sizeof(client_addr);
  conn = accept(ss, (struct sockaddr*)&client_addr, &length);

  if( conn < 0 )
  {
      perror("connect");
      exit(1);
  }
  unsigned char buffer[BUFFER_SIZE];
  uint32_t seq = 0;
  while(ros::ok()) {
      memset(buffer, 0 ,sizeof(buffer));
      int len = recv(conn, buffer, sizeof(buffer), 0);
      zzz_perception_msgs::DetectionBoxArray lights_info;
      lights_info.header.stamp = ros::Time::now();
      lights_info.header.seq = seq++;
      lights_info.header.frame_id = "obu";

      if (len > 8) {
        if (buffer[0] == 0x7E) {
          if (buffer[1] == 0x02) {
            int seq = static_cast<int>(buffer[2]);
            uint16_t len = static_cast<uint16_t>(buffer[3]);
            len <<= 8;
            len += static_cast<uint16_t>(buffer[4]);
            int length = static_cast<int>(len);
            if (length) {
              unsigned char check = 0;
              size_t check_len = 5 + length;
              size_t idx = 0;
              while (idx < check_len) {
                check ^= buffer[idx++];
              }
              if (buffer[5+length] == check) {
                buffer[5 + length] = 0;
                char * str_ptr = reinterpret_cast<char *>(buffer + 5);
                std::string json_str(str_ptr);
                json_map obj { json_data {json_str} };

                if (obj.has("LightsState") && 
                    obj["LightsState"].is(json_value::type::integer) ) {
                  if (0 != obj["LightsState"].as<json_int>()) 
                    continue;
                }
                if (obj.has("LightsInfo") && 
                    obj["LightsInfo"].is(json_value::type::array)) {
                  auto sub_info = obj["LightsInfo"].as<json_array>();
                  zzz_perception_msgs::DetectionBox light;
                  for (size_t i = 0; i < sub_info.size(); i++) {
                    auto o = sub_info[i].as<json_map>();
                    if (o.has("LightsColor") && o.has("RemainingTime") ) {
                      int remaining_time = o["RemainingTime"].as<json_int>();
                      int lights_color = o["LightsColor"].as<json_int>();
                      if (lights_color == GREEN_LIGHT && 
                          remaining_time > GREEN_LIGHT_REMAINING_TIME) {
                        light.signal.flags = 
                            zzz_perception_msgs::ObjectSignals::TRAFFIC_LIGHT_GREEN;
                      } else {
                        light.signal.flags = 
                            zzz_perception_msgs::ObjectSignals::TRAFFIC_LIGHT_RED;
                      }
                    }
                    break;
                  }
                  lights_info.detections.push_back(light);
                }
              }
            }
          }
        }
      }
      pub_obu.publish(lights_info);
  }
  close(conn);
  close(ss);
  return 0;
}