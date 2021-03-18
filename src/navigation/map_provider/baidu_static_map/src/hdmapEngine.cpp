#include <iostream>
#include <fstream>
#include <sstream>
#include "hdmapEngine.h"
#include "CVshow.hpp"
#include "Utm.hpp"

#define STDCOUT(express)  std::cout << express << std::endl
#define SIZECOUT(vec) STDCOUT(#vec << " size : " << vec.size() )
#define TRY_RUN(express)  if ( 0 == (express) ) { \
  std::cout << "run " << #express << " success." << std::endl; \
} else { \
  std::cout << "run " << #express << " failed." << std::endl; \
}

HDMapEngine::HDMapEngine() : routing_request_ptr(&routing_request)
{
  if (!map.LoadMapFromFile(apollo::hdmap::BaseMapFile())){
    ROS_INFO("HDMap initialize successfully");
  };
  last_lane_segment.Clear();
}

HDMapEngine::HDMapEngine(std::string waypoint_path): routing_request_ptr(&routing_request)
{
  if (!map.LoadMapFromFile(apollo::hdmap::BaseMapFile())){
    ROS_INFO("HDMap initialize successfully");
  };
  if (apollo::cyber::common::GetProtoFromFile(waypoint_path, &routing_request)){
    ROS_INFO("Navigation waypoints initialize successfully");
  };
  last_lane_segment.Clear();
}

void HDMapEngine::egoStateCallback(const zzz_driver_msgs::RigidBodyStateStamped::ConstPtr& msg)
{
    _ego_vehicle_state_lock.lock();
    _ego_vehicle_state_buffer = msg;
    _ego_vehicle_state_lock.unlock();
}

int HDMapEngine::exportCurrentPosition(float* start_x, float* start_y)
{
  if (this->_ego_vehicle_state_buffer == nullptr){
    return 1;
  }
  else{
    *start_x = this->_ego_vehicle_state_buffer->state.pose.pose.position.x + 428191;
    *start_y = this->_ego_vehicle_state_buffer->state.pose.pose.position.y + 4417667;
    return 0;
  }
}

void HDMapEngine::generate_routing_request(float s_x, float s_y, float e_x, float e_y)
{
  this->routing_request.Clear();
  apollo::common::PointENU start_point = apollo::common::PointENU();
  start_point.set_x(s_x);
  start_point.set_y(s_y);
  apollo::routing::LaneWaypoint start_waypoint = apollo::routing::LaneWaypoint();
  apollo::common::PointENU *start_pointer = start_waypoint.mutable_pose();
  *start_pointer = start_point;
  apollo::common::PointENU end_point = apollo::common::PointENU();
  end_point.set_x(e_x);
  end_point.set_y(e_y);
  apollo::routing::LaneWaypoint end_waypoint = apollo::routing::LaneWaypoint();
  apollo::common::PointENU *end_pointer = end_waypoint.mutable_pose();
  *end_pointer = end_point;

  apollo::routing::LaneWaypoint *request_s_pointer = this->routing_request.add_waypoint();
  *request_s_pointer = start_waypoint;
  apollo::routing::LaneWaypoint *request_e_pointer = this->routing_request.add_waypoint();
  *request_e_pointer = end_waypoint;
}

// nav_msgs::Path HDMapEngine::generate_global_trajactory(){
//   routing_.Init();
//   routing_.Start();
//   if (routing_.Process(routing_request_ptr, &routing_response)){
//     std::cout << "Routing successfully!"<< std::endl;
//   } else {ROS_ERROR("Routing Failed!");}

//   bool has_changed_lane = false;
//   nav_msgs::Path global_trajactory;
//   global_trajactory.header.frame_id = "map";
//   for (auto road_idx = 0; road_idx < routing_response.road().size(); road_idx++){
//     // roadsegments -> roadsegment
//     apollo::routing::RoadSegment temp_road_seg = routing_response.road().Get(road_idx);
//     std::cout << "********" << std::endl;
//     std::cout << temp_road_seg.id() << std::endl;
//     std::cout << "passage:size" <<temp_road_seg.passage().size() << std::endl;
//     for (auto road_seg_idx = 0; road_seg_idx < temp_road_seg.passage().size(); road_seg_idx++){
//       apollo::routing::Passage temp_passage = temp_road_seg.passage().Get(road_seg_idx);
//       std::cout << "passage: direction" << temp_passage.change_lane_type() << std::endl;
//         for (auto lane_seg_idx = 0; lane_seg_idx < temp_passage.segment().size(); lane_seg_idx++){
//           apollo::routing::LaneSegment temp_lane_seg = temp_passage.segment().Get(lane_seg_idx);
//           this-> get_lane_info(temp_lane_seg);

//           if (last_lane_segment.has_id()){
//             std::vector<std::string> last_name = this->split(last_lane_segment.id(), std::string("-"));
//             std::vector<std::string> this_name = this->split(temp_lane_seg.id(), std::string("-"));
//             std::cout << "leader" <<temp_lane_seg.id() << std::endl;
//             std::cout << "--------" << std::endl;
//             std::cout << last_name[0] << ":" << last_name[1] << std::endl;
//             std::cout << this_name[0]<< ":" << this_name[1] << std::endl;
//             std::cout << "--------" << std::endl;
//             if (last_name[0] == this_name[0]){
//               ROS_ERROR("lane changing");
//               std::cout << last_name[0]<< std::endl;
//               std::cout << this_name[0]<< std::endl;
//               this-> get_half_lane(last_lane_segment, &global_trajactory, 0);
//               has_changed_lane = true;
//             } 
//             // if lane number different, we save lane of last time
//             else
//             {
//               if (has_changed_lane){
//                 this-> get_half_lane(last_lane_segment, &global_trajactory, 1);
//                 has_changed_lane = false;
//               }
//               else{
//                 this->get_lane(last_lane_segment, &global_trajactory);
//               }
//             }
//             last_lane_segment.set_id(temp_lane_seg.id());
//             last_lane_segment.set_start_s(temp_lane_seg.start_s());
//             last_lane_segment.set_end_s(temp_lane_seg.end_s());
//           }
//           else {
//             // first lane we meet this
//             ROS_ERROR("first");
//             last_lane_segment.set_id(temp_lane_seg.id());
//             last_lane_segment.set_start_s(temp_lane_seg.start_s());
//             last_lane_segment.set_end_s(temp_lane_seg.end_s());
//           }
//         }
//     }
//   }
//   return global_trajactory;
// }

nav_msgs::Path HDMapEngine::generate_global_trajactory()
{
  routing_.Init();
  routing_.Start();
  if (routing_.Process(routing_request_ptr, &routing_response)){
    std::cout << "Routing successfully!"<< std::endl;
  } else {ROS_ERROR("Routing Failed!");}

  nav_msgs::Path global_trajactory;
  global_trajactory.header.frame_id = "map";
  for (auto road_idx = 0; road_idx < routing_response.road().size(); road_idx++){
    // roadsegments -> roadsegment
    apollo::routing::RoadSegment temp_road_seg = routing_response.road().Get(road_idx);
    std::cout << "********" << std::endl;
    std::cout << temp_road_seg.id() << std::endl;
    std::cout << "passage:size" <<temp_road_seg.passage().size() << std::endl;
    if (temp_road_seg.passage().size() > 1){
      // if a lane changing happens, we have more than 2 passage
      // for passage 0, we want first half
      apollo::routing::LaneSegment first_lane_seg_0 = temp_road_seg.passage().Get(0).segment().Get(0);
      this-> get_half_lane(first_lane_seg_0, &global_trajactory, 0);
      // for passage 1, we want all but first half
      apollo::routing::LaneSegment first_lane_seg_1 = temp_road_seg.passage().Get(1).segment().Get(0);
      this-> get_half_lane(first_lane_seg_1, &global_trajactory, 1);
      this->get_lane_info(first_lane_seg_1);
      // there maybe several passages , and each passage is consist of severtal segment
      apollo::routing::Passage temp_passage = temp_road_seg.passage().Get(1);
      for (auto lane_seg_idx = 1; lane_seg_idx < temp_passage.segment().size(); lane_seg_idx++){
          apollo::routing::LaneSegment temp_lane_seg = temp_passage.segment().Get(lane_seg_idx);
          this->get_lane_info(temp_lane_seg);
          this->get_lane(temp_lane_seg, &global_trajactory);
      }
    }
    else {
      // normal case, only 1 passage.
      apollo::routing::Passage temp_passage = temp_road_seg.passage().Get(0);
        for (auto lane_seg_idx = 0; lane_seg_idx < temp_passage.segment().size(); lane_seg_idx++){
          apollo::routing::LaneSegment temp_lane_seg = temp_passage.segment().Get(lane_seg_idx);
          this->get_lane_info(temp_lane_seg);
          this->get_lane(temp_lane_seg, &global_trajactory);
        }
    }
  }
  return global_trajactory;
}

visualization_msgs::MarkerArray HDMapEngine::generate_all_lanes(){
  this->lane_output_info.clear();
  std::vector<apollo::hdmap::LaneInfoConstPtr> lanes = get_all_lanes_after_junction();
  std::cout << "**********************" << std::endl;
  visualization_msgs::MarkerArray all_lanes_marker_array;
  int marker_id = 1;
  int color = 1;
  for (auto& lane_info : lanes){
    LaneBundle temp_laneBundle(lane_info);
    temp_laneBundle.speed_limit = lane_info->lane().speed_limit() * 3.6;
    /* 
    TODO how to deal with traffic light with hdmap?
    1.record all traffic lights place.
    */
    temp_laneBundle.traffic_light_pos.emplace_back(0);
    
    std::vector<std::vector<geometry_msgs::Point>> points = this -> get_parallel_lanes_until_junction(lane_info, &temp_laneBundle);
    for (auto lane_point : points){
      for (auto point : lane_point){
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = "map";
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.id = marker_id;
        if (color%2){
          bbox_marker.color.r = 0.0f;
          bbox_marker.color.g = 1.0f;
          bbox_marker.color.b = 0.0f;
        }
        else
        {
          bbox_marker.color.r = 1.0f;
          bbox_marker.color.g = 0.0f;
          bbox_marker.color.b = 0.0f;
        }
        bbox_marker.color.a = 0.3;
        bbox_marker.lifetime = ros::Duration(100);
        bbox_marker.type = visualization_msgs::Marker::CUBE;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        bbox_marker.scale.x = 2;
        bbox_marker.scale.y = 2;
        bbox_marker.scale.z = 2;
        bbox_marker.pose.position.x = point.x;
        bbox_marker.pose.position.y = point.y;
        bbox_marker.pose.position.z = point.z;
        all_lanes_marker_array.markers.push_back(bbox_marker);
        marker_id++;
      }
    }
    color++;
    lane_output_info.emplace_back(temp_laneBundle);
  }
  std::cout << "**********************" << std::endl;
  this->generate_distances_array();
  return all_lanes_marker_array;
}

void HDMapEngine::dump_lanes_info()
{
  this->lane_output_info.clear();
  std::vector<apollo::hdmap::LaneInfoConstPtr> lanes = get_all_lanes_after_junction();
  std::cout << "**********************" << std::endl;
  int lane_number = 0;
  for (auto& lane_info : lanes){
    // lane bundle is useless here, just use it for function API
    LaneBundle temp_laneBundle(lane_info);
    std::vector<std::vector<geometry_msgs::Point>> points = this -> get_parallel_lanes_until_junction(lane_info, &temp_laneBundle);
    int passage_number = 0;
    for (auto lane_point : points){
      std::ofstream lanes_info_file;
      lanes_info_file.open(std::to_string(lane_number) + "_" + std::to_string(passage_number), std::ios::out);
      for (auto point : lane_point){
        lanes_info_file << point.x << "," << point.y << std::endl;
      }
      lanes_info_file.close();
      passage_number++;
    }
    lane_number++;
  }
}

int HDMapEngine::update(const zzz_navigation_msgs::Map::Ptr& static_map)
{
  if (this->lane_output_info.size() == 0 || this->_ego_vehicle_state_buffer == nullptr){
    ROS_WARN("Map Provider: not receive ego pose");
    return 1;
  }
  int closest_index = this->generate_cloest_lane_index();
  //fill informations
  static_map->in_junction = false;
  static_map->exit_lane_index.assign(this->lane_output_info[closest_index].exit_lane_index.begin(),
                                      this->lane_output_info[closest_index].exit_lane_index.end());
  static_map->lanes.assign(this->lane_output_info[closest_index].lanes.begin(),
                          this->lane_output_info[closest_index].lanes.end());
  return 0;
}

void HDMapEngine::generate_distances_array()
{
  this->lanes_distance_info.clear();
  for (auto& lane_bundle : this->lane_output_info){
    std::vector<std::pair<float, float>> temp_lane_central_curve;
    for (int i = 1; i < lane_bundle.central_points.size(); i++){
      std::vector<std::pair<float, float>> temp_result = this->linear_interpolate(lane_bundle.central_points[i-1][0], lane_bundle.central_points[i-1][1],
                                                                                  lane_bundle.central_points[i][0], lane_bundle.central_points[i][1]);
      temp_lane_central_curve.insert(temp_lane_central_curve.end(), temp_result.begin(), temp_result.end());
    }
    temp_lane_central_curve.emplace_back(lane_bundle.central_points[lane_bundle.central_points.size()-1][0],
                                        lane_bundle.central_points[lane_bundle.central_points.size()-1][1]);
    this->lanes_distance_info.emplace_back(temp_lane_central_curve);
  }
}

int HDMapEngine::generate_cloest_lane_index()
{
  _ego_vehicle_state_lock.lock();
  float ego_x = this->_ego_vehicle_state_buffer->state.pose.pose.position.x;
  float ego_y = this->_ego_vehicle_state_buffer->state.pose.pose.position.y;
  _ego_vehicle_state_lock.unlock();

  int closest_idx = 0;
  float closest_distance = 99999;
  for (int i=0; i < this->lanes_distance_info.size(); i++){
    float lane_closest_point_distance = 100000;
    for (int j=0; j < this->lanes_distance_info[i].size(); j++){
      float point_distance = sqrt(pow(ego_x - lanes_distance_info[i][j].first,2) + pow(ego_y - lanes_distance_info[i][j].second,2));
      if (point_distance < lane_closest_point_distance){
        lane_closest_point_distance = point_distance;
      }
    }

    if (lane_closest_point_distance < closest_distance){
      closest_distance = lane_closest_point_distance;
      closest_idx = i;
    }
  }
  return closest_idx;
}

std::vector<apollo::hdmap::LaneInfoConstPtr> HDMapEngine::get_all_lanes_after_junction(){

  std::vector<apollo::hdmap::LaneInfoConstPtr> lanes_before_junction;
  lanes_before_junction.emplace_back(complete_lane[0]);
  for(int i = 1; i < complete_lane.size(); i++){
    if (map.GetRoadById(complete_lane[i]->road_id())->has_junction_id() && !map.GetRoadById(complete_lane[i+1]->road_id())->has_junction_id()){
      lanes_before_junction.emplace_back(complete_lane[i+1]);
    }
  }
  return lanes_before_junction;
}

void HDMapEngine::get_lane_info(const apollo::routing::LaneSegment &temp_lane_seg){
  apollo::hdmap::Id temp_id;
  temp_id.set_id(temp_lane_seg.id());
  this->complete_lane.emplace_back(map.GetLaneById(temp_id));
}

std::vector<std::vector<geometry_msgs::Point>> HDMapEngine::get_parallel_lanes_until_junction(const apollo::hdmap::LaneInfoConstPtr &start_lane, LaneBundle* laneBundle){
  std::vector<std::vector<geometry_msgs::Point>> parallel_lanes_path;
  std::vector<apollo::hdmap::LaneInfoConstPtr> parallel_lanes;
  print_lane_info(start_lane, std::string("father"));
  // TODO: in this way the lanes are place from most right(0) to most left?
  this->get_right_lanes(start_lane, &parallel_lanes);
  int trajactory_lane_index = parallel_lanes.size();
  laneBundle->exit_lane_index.emplace_back(trajactory_lane_index);
  parallel_lanes.emplace_back(start_lane);
  this->get_left_lanes(start_lane, &parallel_lanes);
  for (int i=0; i< parallel_lanes.size(); i++){
    print_lane_info(parallel_lanes[i], std::string("neighbor"));
    std::vector<apollo::hdmap::LaneInfoConstPtr> temp_complete_lane;
    get_next_sublane_until_sudo_junction(parallel_lanes[i], &temp_complete_lane);
    std::vector<geometry_msgs::Point> merged_lane = this->merge_lanes(temp_complete_lane);
    
    if (i == trajactory_lane_index){
      laneBundle->generate_endpoints(merged_lane);
    }
    laneBundle->add_path(merged_lane);
    parallel_lanes_path.emplace_back(merged_lane);
  }
  return parallel_lanes_path;
}

void HDMapEngine::get_next_sublane_until_sudo_junction(const apollo::hdmap::LaneInfoConstPtr &start_lane, std::vector<apollo::hdmap::LaneInfoConstPtr>* lanes){
  lanes->emplace_back(start_lane);
  std::cout << "1" << std::endl;
  if ((start_lane->lane().successor_id().size() == 1) &&
    !map.GetRoadById(map.GetLaneById(start_lane->lane().successor_id().Get(0))->road_id())->has_junction_id()){
    std::cout << "1.1" << std::endl;
    apollo::hdmap::LaneInfoConstPtr next_lane = map.GetLaneById(start_lane->lane().successor_id().Get(0));
    get_next_sublane_until_sudo_junction(next_lane, lanes);
  }
  std::cout << "2" << std::endl;
}

std::vector<geometry_msgs::Point> HDMapEngine::merge_lanes(const std::vector<apollo::hdmap::LaneInfoConstPtr>& lane_pieces){
  std::vector<geometry_msgs::Point> whole_lane_path;
  for (auto& lane : lane_pieces){
    std::vector<apollo::common::math::Vec2d> central_points = lane->points();
    for (auto point : central_points){
      geometry_msgs::Point lane_point;
      lane_point.x = point.x() - 428191;
      lane_point.y = point.y() - 4417667;
      lane_point.z = 0;
      whole_lane_path.emplace_back(lane_point);
    }
  }
  return whole_lane_path;
}

void HDMapEngine::get_lane(const apollo::routing::LaneSegment &temp_lane_seg, nav_msgs::Path* global_trajactory){
  apollo::hdmap::Id temp_id;
  temp_id.set_id(temp_lane_seg.id());
  apollo::hdmap::LaneInfoConstPtr map_lane_ptr = map.GetLaneById(temp_id);
  std::cout << ":" << map_lane_ptr->lane().turn() << std::endl;
  // std::cout << ":" << map_lane_ptr->road_id().id() << std::endl;
  // if (map.GetRoadById(map_lane_ptr->road_id())->has_junction_id()){
  //           std::cout << "shiter" << map.GetRoadById(map_lane_ptr->road_id())->junction_id().id() << std::endl;
  //         }

  for (auto lane_piece_segment_id=0; lane_piece_segment_id < map_lane_ptr->lane().central_curve().segment().size(); lane_piece_segment_id++){
    auto lane_piece_segment_size = map_lane_ptr->lane().central_curve().segment().Get(lane_piece_segment_id).line_segment().point().size();
    for (auto lane_piece_segment_p=0 ;  lane_piece_segment_p < lane_piece_segment_size; lane_piece_segment_p++){
      apollo::common::PointENU lane_piece_segment_pt = map_lane_ptr->lane().central_curve().segment().Get(lane_piece_segment_id).line_segment().point().Get(lane_piece_segment_p);
      geometry_msgs::PoseStamped temp_pose;
      temp_pose.pose.position.x = lane_piece_segment_pt.x() - 428191;
      temp_pose.pose.position.y = lane_piece_segment_pt.y() - 4417667;
      temp_pose.pose.position.z = 0;
      global_trajactory->poses.emplace_back(temp_pose);
    }
  }
}

void HDMapEngine::get_half_lane(const apollo::routing::LaneSegment &temp_lane_seg, nav_msgs::Path* global_trajactory, int number){
  apollo::hdmap::Id temp_id;
  temp_id.set_id(temp_lane_seg.id());
  double start_s = temp_lane_seg.start_s();
  double end_s = temp_lane_seg.end_s();
  double START, END;
  if (!number){
    // first half
    START = start_s;
    END = end_s/2 - 4;
  } else{
    START = end_s/2 + 4;
    END = end_s;
  }
  apollo::hdmap::LaneInfoConstPtr map_lane_ptr = map.GetLaneById(temp_id);
  std::cout << ":" << map_lane_ptr->lane().turn() << std::endl;
  std::vector<apollo::common::PointENU> lane_ref_path;

  for (auto lane_piece_segment_id=0; lane_piece_segment_id < map_lane_ptr->lane().central_curve().segment().size(); lane_piece_segment_id++){
    auto lane_piece_segment_size = map_lane_ptr->lane().central_curve().segment().Get(lane_piece_segment_id).line_segment().point().size();
    for (auto lane_piece_segment_p=0 ;  lane_piece_segment_p < lane_piece_segment_size; lane_piece_segment_p++){
      apollo::common::PointENU lane_piece_segment_pt = map_lane_ptr->lane().central_curve().segment().Get(lane_piece_segment_id).line_segment().point().Get(lane_piece_segment_p);
      lane_ref_path.emplace_back(lane_piece_segment_pt);
    }
  }
  double accumulated_s = 0;
  for (auto i=0; i<lane_ref_path.size()-1; i++){
    std::vector<apollo::common::PointENU> points = this->linear_interpolate(lane_ref_path[i], lane_ref_path[i+1]);
    for (auto j=0; j<points.size(); j++){
      //std::cout << START << END << "-" << accumulated_s << std::endl;
      accumulated_s = accumulated_s + 0.05;
      if ((accumulated_s > START) && (accumulated_s < END)){
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.pose.position.x = points[j].x() - 428191;
        temp_pose.pose.position.y = points[j].y() - 4417667;
        temp_pose.pose.position.z = 0;
        global_trajactory->poses.emplace_back(temp_pose);
        //std::cout << "ACCEPT" <<std::endl;
      }
    }
  }
}

int HDMapEngine::get_partial_lane(const apollo::routing::LaneSegment &temp_lane_seg, nav_msgs::Path* global_trajactory){
  apollo::hdmap::Id temp_id;
  temp_id.set_id(temp_lane_seg.id());
  double start_s = temp_lane_seg.start_s();
  double end_s = temp_lane_seg.end_s();
  apollo::hdmap::LaneInfoConstPtr map_lane_ptr = map.GetLaneById(temp_id);
  std::vector<apollo::common::PointENU> lane_ref_path;

  for (auto lane_piece_segment_id=0; lane_piece_segment_id < map_lane_ptr->lane().central_curve().segment().size(); lane_piece_segment_id++){
    auto lane_piece_segment_size = map_lane_ptr->lane().central_curve().segment().Get(lane_piece_segment_id).line_segment().point().size();
    for (auto lane_piece_segment_p=0 ;  lane_piece_segment_p < lane_piece_segment_size; lane_piece_segment_p++){
      apollo::common::PointENU lane_piece_segment_pt = map_lane_ptr->lane().central_curve().segment().Get(lane_piece_segment_id).line_segment().point().Get(lane_piece_segment_p);
      if (lane_piece_segment_p == lane_piece_segment_size-1){
        std::cout << lane_piece_segment_pt.x() <<std::endl;
        std::cout << std::fixed << std::setprecision(0) << lane_piece_segment_pt.y() <<std::endl;
      }
      lane_ref_path.emplace_back(lane_piece_segment_pt);
    }
  }
  double accumulated_s = 0;
  for (auto i=0; i<lane_ref_path.size()-1; i++){
    std::vector<apollo::common::PointENU> points = this->linear_interpolate(lane_ref_path[i], lane_ref_path[i+1]);
    for (auto j=0; j<points.size(); j++){
      if (((accumulated_s + (j+1) * 1 ) > start_s) && ((accumulated_s + (j+1) * 1 ) < end_s)){
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.pose.position.x = points[j].x() - 428191;
        temp_pose.pose.position.y = points[j].y() - 4417667;
        temp_pose.pose.position.z = 0;
        global_trajactory->poses.emplace_back(temp_pose);
      }
      if ((accumulated_s + (j+1) * 1 ) > end_s){
        return 0;
      }
    }
  }
  return 0;
}

void HDMapEngine::get_left_lanes(const apollo::hdmap::LaneInfoConstPtr &start_lane, std::vector<apollo::hdmap::LaneInfoConstPtr>* parallel_lanes)
{
  for (auto left_lane_idx=0; left_lane_idx < start_lane->lane().left_neighbor_forward_lane_id().size(); left_lane_idx++){
    if (map.GetLaneById(start_lane->lane().left_neighbor_forward_lane_id().Get(left_lane_idx))->lane().type() != 3){
      apollo::hdmap::LaneInfoConstPtr current_left_lane = map.GetLaneById(start_lane->lane().left_neighbor_forward_lane_id().Get(left_lane_idx));
      parallel_lanes->emplace_back(map.GetLaneById(start_lane->lane().left_neighbor_forward_lane_id().Get(left_lane_idx)));
      get_left_lanes(current_left_lane, parallel_lanes);
    }
  }
}

void HDMapEngine::get_right_lanes(const apollo::hdmap::LaneInfoConstPtr &start_lane, std::vector<apollo::hdmap::LaneInfoConstPtr>* parallel_lanes)
{
  for (auto right_lane_idx=start_lane->lane().right_neighbor_forward_lane_id().size()-1; right_lane_idx >= 0; right_lane_idx--){
    if (map.GetLaneById(start_lane->lane().right_neighbor_forward_lane_id().Get(right_lane_idx))->lane().type() != 3){
      apollo::hdmap::LaneInfoConstPtr current_right_lane = map.GetLaneById(start_lane->lane().right_neighbor_forward_lane_id().Get(right_lane_idx));
      get_right_lanes(current_right_lane, parallel_lanes);
      parallel_lanes->emplace_back(map.GetLaneById(start_lane->lane().right_neighbor_forward_lane_id().Get(right_lane_idx)));
    }
  }
}


std::vector<apollo::common::PointENU> HDMapEngine::linear_interpolate(const apollo::common::PointENU& start_point, const apollo::common::PointENU& end_point){
  std::vector<apollo::common::PointENU> interpolated_points;
  double distance = this->calculate_distance(start_point, end_point);
  for(double d=0; d<distance; d=d+0.05){
    apollo::common::PointENU temp_point;
    temp_point.set_x(start_point.x() + (end_point.x()- start_point.x()) * (d / distance));
    temp_point.set_y(start_point.y() + (end_point.y()- start_point.y()) * (d / distance));
    temp_point.set_z(0);
    interpolated_points.emplace_back(temp_point);
  }
  return interpolated_points;
}

std::vector<std::pair<float,float>> HDMapEngine::linear_interpolate(float start_x, float start_y, float end_x, float end_y, float resolution){
  std::vector<std::pair<float,float>> interpolated_points;
  float distance = sqrt(pow(start_x - end_x, 2) + pow(start_y - end_y, 2));
  for(float d=0; d<distance; d=d+resolution){
    interpolated_points.emplace_back((start_x + (end_x - start_x) * (d / distance)), (start_y + (end_y - start_y) * (d / distance)));
  }
  return interpolated_points;
}

std::vector<std::string> HDMapEngine::split(const std::string& srcstr, const std::string& delimeter)
{
    std::vector<std::string> ret(0);//use ret save the spilted reault
    if(srcstr.empty())    //judge the arguments
    {
        return ret;
    }
    std::string::size_type pos_begin = srcstr.find_first_not_of(delimeter);//find first element of srcstr

    std::string::size_type dlm_pos;//the delimeter postion
    std::string temp;              //use third-party temp to save splited element
    while(pos_begin != std::string::npos)//if not a next of end, continue spliting
    {
        dlm_pos = srcstr.find(delimeter, pos_begin);//find the delimeter symbol
        if(dlm_pos != std::string::npos)
        {
            temp = srcstr.substr(pos_begin, dlm_pos - pos_begin);
            pos_begin = dlm_pos + delimeter.length();
        }
        else
        {
            temp = srcstr.substr(pos_begin);
            pos_begin = dlm_pos;
        }
        if(!temp.empty())
            ret.push_back(temp);
    }
    return ret;
}

double HDMapEngine::calculate_distance(const apollo::common::PointENU& start_point, const apollo::common::PointENU& end_point){
  return sqrt(pow(start_point.x() - end_point.x(),2) + pow(start_point.y() - end_point.y(),2));
}

bool HDMapEngine::is_lane_successor_at_intersection(const apollo::hdmap::LaneInfoConstPtr &lane){
  bool is_lane_at_intersection = false;
  // check if left lane has turning successor
  for (auto left_lane_idx=0; left_lane_idx<lane->lane().left_neighbor_forward_lane_id().size(); left_lane_idx++){
      apollo::hdmap::LaneInfoConstPtr temp_left_lane = map.GetLaneById(lane->lane().left_neighbor_forward_lane_id().Get(left_lane_idx));
      for (auto temp_left_next_lane_idx = 0; temp_left_next_lane_idx < temp_left_lane->lane().successor_id().size(); temp_left_next_lane_idx++){
        // std::cout << "LEFT CHECK:" << temp_left_lane->lane().successor_id().size() <<std::endl;
        if (map.GetLaneById(temp_left_lane->lane().successor_id().Get(temp_left_next_lane_idx))->lane().turn() != 1){
          is_lane_at_intersection = true;
          break;
        }
      }
    }
  // check if right lane has turning successor
  for (auto right_lane_idx=0; right_lane_idx<lane->lane().right_neighbor_forward_lane_id().size(); right_lane_idx++){
    apollo::hdmap::LaneInfoConstPtr temp_right_lane = map.GetLaneById(lane->lane().right_neighbor_forward_lane_id().Get(right_lane_idx));
    for (auto temp_right_next_lane_idx = 0; temp_right_next_lane_idx < temp_right_lane->lane().successor_id().size(); temp_right_next_lane_idx++){
      // std::cout << "RIGHT CHECK:" << temp_right_lane->lane().successor_id().size() <<std::endl;
      if (map.GetLaneById(temp_right_lane->lane().successor_id().Get(temp_right_next_lane_idx))->lane().turn() != 1){
        is_lane_at_intersection = true;
        break;
      }
    }
  }
  return is_lane_at_intersection;
}

bool HDMapEngine::is_lane_divergent(const apollo::hdmap::LaneInfoConstPtr &lane){
  bool is_divergentd_lane = true;
  for (auto next_lane_idx = 0; next_lane_idx < lane->lane().successor_id().size(); next_lane_idx++){
    if (map.GetLaneById(lane->lane().successor_id().Get(next_lane_idx))->lane().turn() != 1){
      is_divergentd_lane = false;
      break;
    }
  }
  // judge if all neighbors have 1 successor or all successor(1)
  std::vector<apollo::hdmap::LaneInfoConstPtr> parallel_lanes;
  // TODO: in this way the lanes are place from most right(0) to most left?
  for (auto right_lane_idx=lane->lane().right_neighbor_forward_lane_id().size()-1; right_lane_idx >= 0; right_lane_idx--){
    if (map.GetLaneById(lane->lane().right_neighbor_forward_lane_id().Get(right_lane_idx))->lane().type() != 3){
      parallel_lanes.emplace_back(map.GetLaneById(lane->lane().right_neighbor_forward_lane_id().Get(right_lane_idx)));
    }
  }
  for (auto left_lane_idx=0; left_lane_idx < lane->lane().left_neighbor_forward_lane_id().size(); left_lane_idx++){
    if (map.GetLaneById(lane->lane().left_neighbor_forward_lane_id().Get(left_lane_idx))->lane().type() != 3){
      parallel_lanes.emplace_back(map.GetLaneById(lane->lane().left_neighbor_forward_lane_id().Get(left_lane_idx)));
    }
  }
  for (auto neighbor_lane : parallel_lanes){
    if (neighbor_lane->lane().successor_id().size() > 1){
    for (auto next_lane_idx = 0; next_lane_idx < neighbor_lane->lane().successor_id().size(); next_lane_idx++){
      if (map.GetLaneById(neighbor_lane->lane().successor_id().Get(next_lane_idx))->lane().turn() != 1){
        is_divergentd_lane = false;
        break;
      }
    }
    }
  }
  return is_divergentd_lane;
}

void HDMapEngine::print_lane_info(const apollo::hdmap::LaneInfoConstPtr & lane,std::string title){
  std::cout << "-------------"<< title << "-------------" << std::endl;
  std::cout << lane->id().id() << std::endl;
  std::cout << "-------------"<< title << "-------------" << std::endl;
}


// std::vector<apollo::hdmap::LaneInfoConstPtr> HDMapEngine::get_all_lanes_after_junction(){
//   std::vector<apollo::hdmap::LaneInfoConstPtr> lanes_before_junction;
//   lanes_before_junction.emplace_back(complete_lane[0]);
//   for(int i = 0; i < complete_lane.size(); i++){
//     // judge if this lane has only one successor.
//     if (complete_lane[i]->lane().successor_id().size() == 1){
//       // has only one successor, but successor is actually in intersection
//       if (this->is_lane_successor_at_intersection(complete_lane[i])){
//         int temp_idx = i;
//         for (int k=1; k<10; k++){
//           temp_idx = temp_idx + 1;
//           if (!map.GetRoadById(complete_lane[temp_idx]->road_id())->has_junction_id()){
//             lanes_before_junction.emplace_back(complete_lane[temp_idx]);
//             //sometimes, there are several junction lanes connect together which makes overlaps, we need to skip them
//             i = i + k;
//             break;
//           }
//         }
//       }
//     }
//     else if(complete_lane[i]->lane().successor_id().size() > 1){
//       // this lane has more than 1 successor, but we need to make sure it's not a divergent lane
//       if (!this->is_lane_divergent(complete_lane[i])){
//         //it's a real junction
//         int temp_idx = i;
//         for (int k=0; k<10; k++){
//           temp_idx = temp_idx + 1;
//           if (!map.GetRoadById(complete_lane[temp_idx]->road_id())->has_junction_id()){
//             lanes_before_junction.emplace_back(complete_lane[temp_idx]);
//             i = i + k;
//             break;
//           }
//         }
//       }
//     }
//   }
//   return lanes_before_junction;
// }

// std::vector<apollo::hdmap::LaneInfoConstPtr> HDMapEngine::get_all_lanes_after_junction(){
//   std::vector<apollo::hdmap::LaneInfoConstPtr> lanes_before_junction;
//   for(int i = 0; i < complete_lane.size(); i++){
//     if (map.GetRoadById(complete_lane[i]->road_id())->has_junction_id() && !map.GetRoadById(complete_lane[i+1]->road_id())->has_junction_id()){
//       // it's a lane in junction
//       if (complete_lane[i]->lane().successor_id().size() == 1){
//         if (map.GetLaneById(complete_lane[i]->lane().successor_id().Get(0))->lane().predecessor_id().size() == 1 ){
//           continue;
//         }
//       }
//       lanes_before_junction.emplace_back(complete_lane[i+1]);
//     }
//   }
//   return lanes_before_junction;
// }

// void HDMapEngine::get_next_sublane_until_junction(const apollo::hdmap::LaneInfoConstPtr &start_lane, std::vector<apollo::hdmap::LaneInfoConstPtr>* lanes){
//   lanes->emplace_back(start_lane); 
//   if (start_lane->lane().successor_id().size() == 1){
//     if (!this->is_lane_successor_at_intersection(start_lane)){
//       apollo::hdmap::LaneInfoConstPtr next_lane = map.GetLaneById(start_lane->lane().successor_id().Get(0));
//       get_next_sublane_until_junction(next_lane, lanes);
//     }
//   }
//   else if (start_lane->lane().successor_id().size() > 1){
//     // we need to check if it's a divergent lane on straight lane, if yes, add it.
//     if (this->is_lane_divergent(start_lane)){
//       /* 
//       known that it's a divergent road, then which to choose as complete lane?
//       1. For 2to3, 3to4, 2to5 situation, a straight lane means successor 0
//       */ 
//       // std::cout << "FOUND DIVERGENT ROAD" << std::endl;
//       // std::cout << start_lane->lane().successor_id().size() << std::endl;
//       apollo::hdmap::LaneInfoConstPtr next_lane = map.GetLaneById(start_lane->lane().successor_id().Get(0));
//       get_next_sublane_until_junction(next_lane, lanes);
//     }
//   }
// }
