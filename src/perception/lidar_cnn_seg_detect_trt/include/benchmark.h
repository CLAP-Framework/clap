#pragma once

#include <autoware_msgs/DetectedObjectArray.h>
#include <iostream>
#include <fstream>
#include <string>

namespace nova {

class Benchmark {
public:
enum Dataset : int {
  Waymo = 0,
  Kitti,

  DatasetMax
};

const std::string WaymoClassType[6] = {
  "TYPE_UNKNOWN", 
  "TYPE_VEHICLE", 
  "TYPE_VEHICLE", 
  "TYPE_CYCLIST",
  "TYPE_PEDESTRIAN",
  "TYPE_SIGN"
};

public:
  Benchmark(Dataset set, std::string& path)
      : set_(set), path_(path) {}
  bool record (autoware_msgs::DetectedObjectArray& objects) {
    std::stringstream s_file;
    s_file << std::setw(6) 
        << std::setfill('0') 
        << objects.header.seq ;
    std::string file = path_ + "/" + s_file.str() + ".txt";
    std::remove(file.c_str());
    
    std::stringstream s_text;
    if (set_ == Dataset::Waymo)
      generateWaymoFormat(s_text, objects);
    else 
      return false;
    std::ofstream outputfile(file, 
        std::ofstream::out | std::ofstream::app);
    outputfile << s_text.str();
    return true;
  }

private:
  std::string getTypeString(std::string& label)  {
    if (0 == strcasecmp(label.c_str(), "Unknown")) 
      return "TYPE_UNKNOWN";
    if (0 == strcasecmp(label.c_str(), "Car")) 
      return "TYPE_VEHICLE"; 
    if (0 == strcasecmp(label.c_str(), "Truck")) 
      return "TYPE_VEHICLE"; 
    if (0 == strcasecmp(label.c_str(), "Cyclist")) 
      return "TYPE_CYCLIST";
    if (0 == strcasecmp(label.c_str(), "Pedestrian")) 
      return "TYPE_PEDESTRIAN";
    // if () return "TYPE_SIGN"
  }

  bool generateWaymoFormat(std::stringstream& ss, 
      autoware_msgs::DetectedObjectArray& objects) {
    for (auto& obj : objects.objects)
      ss << getTypeString(obj.label) << " " 
        << 0 << " " 
        << obj.pose.position.x << " "
        << obj.pose.position.y << " "
        << obj.pose.position.z + 1.8 << " "
        << obj.dimensions.x << " "
        << obj.dimensions.y << " "
        << obj.dimensions.z << " "
        << obj.angle << " " 
        << 0 << " "
        << obj.score << std::endl; 
    return true;
  }

  bool generateKittiFormat(std::stringstream& ss, 
      autoware_msgs::DetectedObjectArray& objects) {
    for (auto& obj : objects.objects)
      ss << obj.label << " " 
        << obj.score << " " 
        << obj.pose.position.x << " "
        << obj.pose.position.y << " "
        << obj.pose.position.z << " "
        << obj.dimensions.x << " "
        << obj.dimensions.y << " "
        << obj.dimensions.z << " "
        << -obj.angle << std::endl; 
    return true;
  }

private:
  Dataset set_;
  std::string path_;
}; 

} // namespace nova