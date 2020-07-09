#include "time_point.h"
#include "inference.h"
#include "inference_factory.h"
#include "pointpillars.h"
#include <fstream>
#include "benchmarch.h"
#include "time_point.h"
#include "detected_object.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
using namespace tf;

std::string label_name[4] = {"Car", "Truck", "Pedestrian", "Cyclist"};
cv::Scalar class_color[4] = {
    cv::Scalar(0, 25, 255),
    cv::Scalar(100, 0, 255),
    cv::Scalar(0, 0, 255),
    cv::Scalar(0, 255, 0)
};

Pointpillars::Pointpillars() : nh_()
{
}

bool Pointpillars::init() {
    cudaEventCreate(&start_cu_);    //创建event
    cudaEventCreate(&stop_cu_);

    std::string engine_file;
    ros::NodeHandle private_node_handle("~");//to receive args
    benchmark_path_.clear();
    duration_path_.clear();
    if (private_node_handle.getParam("benchmark_path", benchmark_path_)){
        ROS_INFO("[%s] benchmark_path: %s", __APP_NAME__, benchmark_path_.c_str());
    }
    if (private_node_handle.getParam("duration_path", duration_path_)){
        ROS_INFO("[%s] duration_path: %s", __APP_NAME__, duration_path_.c_str());
    }
    if (private_node_handle.getParam("engine_file", engine_file)){
        ROS_INFO("[%s] engine_path: %s", __APP_NAME__, engine_file.c_str());
    } else {
        ROS_ERROR("[%s] error path for engine_file.", __APP_NAME__);
        return false;
    } 

    private_node_handle.param<std::string>("publish_objects", 
        topic_pub_objects_, "/detection/lidar_detector/objects");
    private_node_handle.param<std::string>("points_src", topic_src_, "/points_raw");
    private_node_handle.param<float>("min_horizontal", min_horizontal_, -25.);
    private_node_handle.param<float>("max_horizontal", max_horizontal_, 25.);
    private_node_handle.param<float>("min_vertical", min_vertical_, -50.);
    private_node_handle.param<float>("max_vertical", max_vertical_, 50.);
    private_node_handle.param<float>("min_height", min_height_, -0.5);
    private_node_handle.param<float>("max_height", max_height_, 3.5);
    private_node_handle.param<int>("max_pillar_num", max_pillar_num_, 20000);
    private_node_handle.param<int>("max_point_num", max_point_num_, 32);
    private_node_handle.param<int>("feature_channel", feature_channel_, 3);
    private_node_handle.param<int>("width", width_, 320);
    private_node_handle.param<int>("height", height_, 640);
    private_node_handle.param<int>("channel", channel_, 32);
    private_node_handle.param<bool>("use_gpu", use_gpu_, false);
    private_node_handle.param<int>("gpu_device_id", gpu_device_id_, 0);
    
    private_node_handle.param<float>("score_car_threshold", score_car_threshold_, 0.4);
    private_node_handle.param<float>("score_ped_cyc_threshold", score_ped_cyc_threshold_, 0.3);
    private_node_handle.param<float>("nms_threshold", nms_threshold_, 0.3);
    private_node_handle.param<int>("class_num", class_num_, 1);
    private_node_handle.param<int>("dims_car_channel", dims_car_channel_, 9);
    private_node_handle.param<int>("dims_car_height", dims_car_height_, 320);
    private_node_handle.param<int>("dims_car_width", dims_car_width_, 160);
    private_node_handle.param<int>("dims_ped_cyc_channel", dims_ped_cyc_channel_, 22);
    private_node_handle.param<int>("dims_ped_cyc_height", dims_ped_cyc_height_, 640);
    private_node_handle.param<int>("dims_ped_cyc_width", dims_ped_cyc_width_, 320);
    private_node_handle.param<float>("cls0_anchor_x", cls0_anchor_x_, 10.24);
    private_node_handle.param<float>("cls0_anchor_y", cls0_anchor_y_, 24.96);
    private_node_handle.param<float>("cls0_anchor_z", cls0_anchor_z_, 0);
    private_node_handle.param<float>("cls1_anchor_x", cls1_anchor_x_, 3.84);
    private_node_handle.param<float>("cls1_anchor_y", cls1_anchor_y_, 5.12);
    private_node_handle.param<float>("cls1_anchor_z", cls1_anchor_z_, 0);
    private_node_handle.param<float>("cls2_anchor_x", cls2_anchor_x_, 3.84);
    private_node_handle.param<float>("cls2_anchor_y", cls2_anchor_y_, 11.264);
    private_node_handle.param<float>("cls2_anchor_z", cls2_anchor_z_, 0);
    private_node_handle.param<int>("NUM_THREADS", NUM_THREADS_, 64);

    ROS_INFO("[%s] max_pillar_num: %d", __APP_NAME__, max_pillar_num_);
    ROS_INFO("[%s] max_point_num_: %d", __APP_NAME__, max_point_num_);
    ROS_INFO("[%s] feature_channel_: %d", __APP_NAME__, feature_channel_);
    ROS_INFO("[%s] publish_objects: %s", __APP_NAME__, topic_pub_objects_.c_str());
    ROS_INFO("[%s] points_src: %s", __APP_NAME__, topic_src_.c_str());
    ROS_INFO("[%s] min_vertical: %.2f", __APP_NAME__, min_vertical_);
    ROS_INFO("[%s] max_vertical: %.2f", __APP_NAME__, max_vertical_);
    ROS_INFO("[%s] min_horizontal: %.2f", __APP_NAME__, min_horizontal_);
    ROS_INFO("[%s] max_horizontal: %.2f", __APP_NAME__, max_horizontal_);
    ROS_INFO("[%s] min_height: %.2f", __APP_NAME__, min_height_);
    ROS_INFO("[%s] max_height: %.2f", __APP_NAME__, max_height_);
    ROS_INFO("[%s] width: %d", __APP_NAME__, width_);
    ROS_INFO("[%s] height: %d", __APP_NAME__, height_);
    ROS_INFO("[%s] channel: %d", __APP_NAME__, channel_);
    ROS_INFO("[%s] use_gpu: %d", __APP_NAME__, use_gpu_);
    ROS_INFO("[%s] gpu_device_id: %d", __APP_NAME__, gpu_device_id_);
    //post process
    ROS_INFO("[%s] score_car_threshold: %.2f", __APP_NAME__, score_car_threshold_);
    ROS_INFO("[%s] score_ped_cyc_threshold: %.2f", __APP_NAME__, score_ped_cyc_threshold_);
    ROS_INFO("[%s] nms_threshold: %.2f", __APP_NAME__, nms_threshold_);
    ROS_INFO("[%s] class_num: %d", __APP_NAME__, class_num_);
    ROS_INFO("[%s] dims_car_channel: %d", __APP_NAME__, dims_car_channel_);
    ROS_INFO("[%s] dims_car_height: %d", __APP_NAME__, dims_car_height_);
    ROS_INFO("[%s] dims_car_width: %d", __APP_NAME__, dims_car_width_);
    ROS_INFO("[%s] dims_ped_cyc_channel: %d", __APP_NAME__, dims_ped_cyc_channel_);
    ROS_INFO("[%s] dims_ped_cyc_height: %d", __APP_NAME__, dims_ped_cyc_height_);
    ROS_INFO("[%s] dims_ped_cyc_width: %d", __APP_NAME__, dims_ped_cyc_width_);
    ROS_INFO("[%s] cls1_anchor_x: %.2f", __APP_NAME__, cls1_anchor_x_);
    ROS_INFO("[%s] cls1_anchor_y: %.2f", __APP_NAME__, cls1_anchor_y_);
    ROS_INFO("[%s] cls1_anchor_z: %.2f", __APP_NAME__, cls1_anchor_z_);

    std::vector<std::string> outputBlobName_{
        "pointpillars_part2/features_car", "pointpillars_part2/features_ped_cyc"};
    output_blob_name_.clear();
    output_blob_name_ = outputBlobName_;
    std::vector<std::string> input_blob_name_{"coors_input_", "voxel_num_", "points"};
    trt_net_.reset(novauto::tensorrt::inference::CreateInferenceByName(
        "TRTNet", engine_file, input_blob_name_, outputBlobName_)); 
    output_data_.resize(output_blob_name_.size());

    nbytes_scatter_ = max_pillar_num_ * sizeof(float) * 2;
    nbytes_p1_input_ = max_pillar_num_ * max_point_num_ * feature_channel_ * sizeof(float);
    
    #ifdef X86
    host_coors_ = (float*)malloc(nbytes_scatter_);
    host_p1_input_ = (float*)malloc(nbytes_p1_input_);
    #else
    cudaMallocHost((void **)&dev_coors_, nbytes_scatter_);
    cudaMallocHost((void**)&dev_p1_input_, nbytes_p1_input_);
    #endif

    int dev = 0;
    cudaSetDevice(dev);
    // get device information
    cudaDeviceProp deviceProp;
    cudaGetDeviceProperties(&deviceProp, dev);
    ROS_INFO(
        "%s starting at device %d: %s memory size %d nbyte %fMB .", \
        "Coors", dev, deviceProp.name, nbytes_scatter_ / sizeof(float), \
        nbytes_scatter_ / (1024.0f * 1024.0f));
    ROS_INFO(
        "%s starting at device %d: %s memory size %d nbyte %fMB .", \
        "Part1_input", dev, deviceProp.name, nbytes_p1_input_ / sizeof(float), \
        nbytes_p1_input_ / (1024.0f * 1024.0f));
    feature_generator_.reset(new FeatureGenerator());

    if ( !feature_generator_ -> init(width_, height_,\
                max_pillar_num_, max_point_num_, feature_channel_,\
                min_vertical_, max_vertical_, \
                min_horizontal_, max_horizontal_,\
                min_height_, max_height_) ) {
        ROS_ERROR("[%s] Fail to Initialize feature generator for Pointpillars", __APP_NAME__);
        return false;
    }

    #ifdef __USE_GPU__
        get_bounding_box_gpu_.reset(new PostprocessCuda(
            min_height_, max_height_, width_, 
            dims_car_width_, dims_car_height_, dims_car_channel_,
            dims_ped_cyc_width_, dims_ped_cyc_height_, dims_ped_cyc_channel_,
            cls0_anchor_x_, cls0_anchor_y_,
            cls1_anchor_x_, cls1_anchor_y_, cls2_anchor_x_, cls2_anchor_y_,
            score_car_threshold_, score_ped_cyc_threshold_, nms_threshold_, 8, 10));

    #else
        get_bounding_box_.reset(new PostProcess());
        if( !get_bounding_box_ -> init(width_, nms_threshold_, 
            score_car_threshold_, score_ped_cyc_threshold_,\
            dims_car_channel_, dims_car_height_, dims_car_width_,\
            dims_ped_cyc_channel_, dims_ped_cyc_height_, dims_ped_cyc_width_,\
            cls0_anchor_x_, cls0_anchor_y_, cls0_anchor_z_,\
            cls1_anchor_x_, cls1_anchor_y_, cls1_anchor_z_,\
            cls2_anchor_x_, cls2_anchor_y_, cls2_anchor_z_,\
            min_height_, max_height_)) {
            ROS_ERROR("[%s] Fail to Initialize postprocess for Pointpillars", __APP_NAME__);
            return false;
        }
    #endif
    return true;
}


bool Pointpillars::detect(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr,
    const pcl::PointIndices &valid_idx, autoware_msgs::DetectedObjectArray &objects) {
    int num_pts = static_cast<int>(pc_ptr->points.size());
    if (num_pts == 0) {
        ROS_INFO("[%s] Empty point cloud.", __APP_NAME__);
        return true;
    }
    _TCSV_INIT();

    #ifdef X86
    memset(host_coors_, 0, nbytes_scatter_);
    memset(host_p1_input_, 0, nbytes_p1_input_);
    #else
    cudaMemset(dev_coors_, 0, nbytes_scatter_);
    cudaMemset(dev_p1_input_, 0, nbytes_p1_input_);
    #endif

    _TCSV_START();
    int num_pillars = 0;

    #ifdef X86
    feature_generator_.get() -> generate(pc_ptr, host_coors_, num_pillars, host_p1_input_);
    trt_net_ -> SetInputTensor(0, host_p1_input_, \
                        max_pillar_num_ * max_point_num_ * feature_channel_, true);
    trt_net_ -> SetInputTensor(1, host_coors_, 1 * 1 * 40000, true);
    #else
    feature_generator_.get() -> generate(pc_ptr, dev_coors_, num_pillars, dev_p1_input_);
    trt_net_ -> SetInputTensor(0, dev_p1_input_, \
                        max_pillar_num_ * max_point_num_ * feature_channel_, false);
    trt_net_ -> SetInputTensor(1, dev_coors_, 1 * 1 * 40000, false);
    #endif
    
    std::vector<float> voxel_num_vec;
    voxel_num_vec.push_back(num_pillars * 1.0);
    trt_net_ -> SetInputTensor(2, voxel_num_vec);
    _TCSV_END();

    _TCSV_START();
    trt_net_ -> Infer();
    _TCSV_END();

    _TCSV_START();
    #ifdef __USE_GPU__
        #ifdef X86
        void *car     = trt_net_ -> GetBindingPtr(4);
        void *ped_cyc = trt_net_ -> GetBindingPtr(3);
        #else
        void *car     = trt_net_ -> GetBindingPtr(3);
        void *ped_cyc = trt_net_ -> GetBindingPtr(4);
        #endif
    #else
        #ifdef X86
        trt_net_ -> GetOutputTensor(4, output_data_[0]);
        trt_net_ -> GetOutputTensor(3, output_data_[1]);
        #else
        trt_net_ -> GetOutputTensor(3, output_data_[0]);
        trt_net_ -> GetOutputTensor(4, output_data_[1]);
        #endif
    #endif
    _TCSV_END();

    _TCSV_START();
    std::vector<BoundingBox> result;
    #ifdef __USE_GPU__
        get_bounding_box_gpu_ -> doPostprocessCuda("car", (float*)car,
                                            NUM_THREADS_,
                                            result);
        get_bounding_box_gpu_ -> doPostprocessCuda("ped_cyc", (float*)ped_cyc,
                                            NUM_THREADS_,
                                            result);
    #else
        get_bounding_box_ -> PointCloudPostTreat("car", 
                                (float *)(output_data_[0].data()), result);
        get_bounding_box_ -> PointCloudPostTreat("ped_cyc",
                                (float *)(output_data_[1].data()), result);
    #endif
    _TCSV_END();
    // _TCSV_PRINT(duration_path_, (!duration_path_.empty()) );
    for (auto &bbox: result) {
        // output box
        autoware_msgs::DetectedObject object;  
        // object.label = "unknown";   
        object.header = message_header_;
        int temp_label = 0;
        if (bbox.class_idx == 0) {
            temp_label = 0;
        } else {
            temp_label = bbox.class_idx + 1;
        }
        object.label = label_name[temp_label]; 
        object.color.r = class_color[temp_label].val[0];
        object.color.g = class_color[temp_label].val[1];
        object.color.b = class_color[temp_label].val[2];
        object.color.a = 0.4f;  
        object.score = bbox.class_prob;

        object.pose.position.x = (double)(bbox.x * 100.0 / 640.0 - 50);
        object.pose.position.y = (double)(bbox.y * 50.0 / 320.0 - 25);
        object.pose.position.z = (double)(bbox.z - 1.2);

        object.dimensions.y = (double)(bbox.w * 100.0 / 640.0);
        object.dimensions.x = (double)(bbox.l * 50.0 / 320.0);
        object.dimensions.z = (double)(bbox.h);

        ///> TODO: need a variable to store the angle
        // float angle_out = atan2(, );
        object.angle = bbox.angle;
        tf::Quaternion quaternion;
        quaternion.setRPY(0, 0, bbox.angle);
        object.pose.orientation.x = quaternion.getX();
        object.pose.orientation.y = quaternion.getY();
        object.pose.orientation.z = quaternion.getZ();
        object.pose.orientation.w = quaternion.getW();

        ///> TODO: need a variable to store the class probability.
        object.variance.x = 0;
        object.variance.y = 0;
        object.variance.z = 0;
        object.pose_reliable = true;
        object.velocity_reliable = false;
        object.acceleration_reliable = false;
        object.valid = true;
        object.space_frame = message_header_.frame_id;
        objects.objects.push_back(object);
    }
    return true;
}

void Pointpillars::run() {
    init();
    points_sub_ = nh_.subscribe(topic_src_, 1, 
        &Pointpillars::pointsCallback, this);
    objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(
        topic_pub_objects_, 2);
    ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);
}

void Pointpillars::pointsCallback(const sensor_msgs::PointCloud2 &msg)
{
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, *in_pc_ptr);
    pcl::PointIndices valid_idx;
    message_header_ = msg.header;

    autoware_msgs::DetectedObjectArray objects;
    objects.header = message_header_;
    _TP(detect(in_pc_ptr, valid_idx, objects));
        objects_pub_.publish(objects);

    if (!benchmark_path_.empty()) {
        std::cout << "record" << std::endl;
        nova::Benchmarch bench(nova::Benchmarch::Dataset::Waymo, benchmark_path_);
        bench.record(objects);
    }

}
