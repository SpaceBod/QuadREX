#include <cstdio>
#include <iostream>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TrackSpatialDetectionConverter.hpp"
#include "depthai_ros_msgs/msg/track_detection2_d_array.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/ObjectTracker.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

dai::Pipeline createPipeline(bool syncNN, std::string nnPath, int confidence, int LRchecktresh, std::string resolution, bool fullFrameTracking) {
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::MobileNetSpatialDetectionNetwork>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto tracker = pipeline.create<dai::node::ObjectTracker>();

    // create xlink connections
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutTracker = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("preview");
    xoutDepth->setStreamName("depth");
    xoutTracker->setStreamName("tracklets");

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    /// setting node configs
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setSubpixel(false);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    // object tracker settings
    tracker->setTrackerType(dai::TrackerType::ZERO_TERM_COLOR_HISTOGRAM);
    tracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::UNIQUE_ID);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(spatialDetectionNetwork->input);
    tracker->passthroughTrackerFrame.link(xoutRgb->input);

    if(fullFrameTracking) {
        colorCam->setPreviewKeepAspectRatio(false);
        colorCam->video.link(tracker->inputTrackerFrame);
    } else {
        spatialDetectionNetwork->passthrough.link(tracker->inputTrackerFrame);
    }

    spatialDetectionNetwork->passthrough.link(tracker->inputDetectionFrame);
    spatialDetectionNetwork->out.link(tracker->inputDetections);
    tracker->out.link(xoutTracker->input);
    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);

    return pipeline;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mobilenet_tracker_node");

    std::string tfPrefix, resourceBaseFolder, nnPath;
    std::string cameraParamUri = "package://depthai_examples/params/camera";
    std::string nnName(BLOB_NAME);
    bool syncNN, fullFrameTracking;
    int confidence = 200, LRchecktresh = 5;
    std::string monoResolution = "400p";

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", cameraParamUri);
    node->declare_parameter("sync_nn", true);
    node->declare_parameter("nnName", "");
    node->declare_parameter("confidence", confidence);
    node->declare_parameter("LRchecktresh", LRchecktresh);
    node->declare_parameter("monoResolution", monoResolution);
    node->declare_parameter("resourceBaseFolder", "");
    node->declare_parameter("fullFrameTracking", false);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", cameraParamUri);
    node->get_parameter("sync_nn", syncNN);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);
    node->get_parameter("fullFrameTracking", fullFrameTracking);

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("Send the path to the resource folder containing NNBlob in 'resourceBaseFolder'");
    }

    std::string nnParam;
    node->get_parameter("nnName", nnParam);
    if(nnParam != "x") {
        node->get_parameter("nnName", nnName);
    }

    nnPath = resourceBaseFolder + "/" + nnName;
    dai::Pipeline pipeline = createPipeline(syncNN, nnPath, confidence, LRchecktresh, monoResolution, fullFrameTracking);
    dai::Device device(pipeline);

    auto colorQueue = device.getOutputQueue("preview", 30, false);
    auto depthQueue = device.getOutputQueue("depth", 30, false);
    auto trackQueue = device.getOutputQueue("tracklets", 30, false);
    auto calibrationHandler = device.readCalibration();

    int width, height;
    if(monoResolution == "720p") {
        width = 1280;
        height = 720;
    } else if(monoResolution == "400p") {
        width = 640;
        height = 400;
    } else if(monoResolution == "800p") {
        width = 1280;
        height = 800;
    } else if(monoResolution == "480p") {
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", monoResolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(height > 480 && boardName == "OAK-D-LITE") {
        width = 640;
        height = 480;
    }

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, -1, -1);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(colorQueue,
                                                                                       node,
                                                                                       std::string("color/image"),
                                                                                       std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                 &rgbConverter,
                                                                                                 std::placeholders::_1,
                                                                                                 std::placeholders::_2),
                                                                                       30,
                                                                                       rgbCameraInfo,
                                                                                       "color");

    dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame", true);
    auto depthCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(depthQueue,
                                                                                         node,
                                                                                         std::string("stereo/depth"),
                                                                                         std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                   &depthConverter,
                                                                                                   std::placeholders::_1,
                                                                                                   std::placeholders::_2),
                                                                                         30,
                                                                                         depthCameraInfo,
                                                                                         "stereo");

    dai::rosBridge::TrackSpatialDetectionConverter trackConverter(tfPrefix + "_rgb_camera_optical_frame", width, height, false, 0.3);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::TrackDetection2DArray, dai::Tracklets> trackPublish(trackQueue,
                                                                                                              node,
                                                                                                              std::string("color/tracklets"),
                                                                                                              std::bind(&dai::rosBridge::TrackSpatialDetectionConverter::toRosMsg,
                                                                                                                        &trackConverter,
                                                                                                                        std::placeholders::_1,
                                                                                                                        std::placeholders::_2),
                                                                                                              30);

    rgbPublish.addPublisherCallback();
    depthPublish.addPublisherCallback();
    trackPublish.addPublisherCallback();

    rclcpp::spin(node);
    return 0;
}
