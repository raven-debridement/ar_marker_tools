/*
 *  marker_detector_from_stream_node.cpp
 *  Created on: Aug 9, 2010
 *  Author: Christian Bersch
 *  Modified by: Ziang Xie
 */

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>

#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "marker_tracker.h"
#include <marker_detect/MarkerInfos.h>
#include <marker_detect/MarkerXYs.h>

#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

/*
ostream& operator<< (ostream& out,  const tf::Transform  tf){
    Eigen::Affine3d tf_eigen;
    tf::TransformTFToEigen(tf,tf_eigen);
    out << tf_eigen.matrix();
    return out;
}
*/

geometry_msgs::Pose toPose(const Eigen::Affine3f& in) {
  geometry_msgs::Pose out;
  Eigen::Vector3f trans = in.translation();
} 

class MarkerDetector{

    public:

        ros::NodeHandle nh;

        // Publish marker poses (MarkerInfos messages)
        ros::Publisher marker_pub;
        // Publish x, y centerpoints of markers in images (MarkerXYs messages)
        ros::Publisher marker_xy_pub;

        MyTrackerROS* tracker;

        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;


        // Parameters (mostly specified in launch file, defaults below)
        std::string topic;          // Camera topic
        std::string pub_topic;      // Topic to publish poses to
        std::string xy_pub_topic;   // Topic to publish x, y coords to
        std::string camera_frame;
        std::string target_frame;   // Frame to publish poses in
        int num_markers;            // Maximum number of markers to consider
        bool with_history;
        double scale_factor;        // Set depending on scale of markers printed
        int block_size;             // Set proportional to size of markers
        double offset;              // Increase if poor contrast
        int frame_rate;
        bool debugging;             // Will display debug image windows
        bool no_x_output;           // Whether to display detected markers
        bool mean;                  // Whether to use the mean adaptive method (as opposed to gaussian)

        sensor_msgs::CvBridge bridge_;
        image_geometry::PinholeCameraModel cam_model;

        IplImage* out;
        IplImage* ada;

        MarkerDetector():it_(nh) {

            ros::NodeHandle nh_priv("~");
			/*
            topic = nh.resolveName("image");
            nh_priv.param<std::string>("pub_topic", pub_topic, "marker_detector");
            nh_priv.param<std::string>("xy_pub_topic", xy_pub_topic, "marker_xy");
            nh_priv.param("num_markers", num_markers, 20);
            nh_priv.param<std::string>("camera_frame", camera_frame, "logitech");
            nh_priv.param<std::string>("target_frame", target_frame, "logitech");
            nh_priv.param("scale_factor", scale_factor, 0.018);
            nh_priv.param("history", with_history, false);
            nh_priv.param("offset", offset, 15.0);
            nh_priv.param("block_size", block_size, 31);
            nh_priv.param("no_x_output", no_x_output, true);
            nh_priv.param("frame_rate", frame_rate, 20);
            nh_priv.param("debugging", debugging, false);
            */
			topic = nh.resolveName("image_rect");
            nh_priv.param<std::string>("pub_topic", pub_topic, "marker_detector");
            nh_priv.param<std::string>("xy_pub_topic", xy_pub_topic, "marker_xy");
            nh_priv.param("num_markers", num_markers, 20);
            nh_priv.param<std::string>("camera_frame", camera_frame, "logitech_red");
            nh_priv.param<std::string>("target_frame", target_frame, "logitech_red");
            nh_priv.param("scale_factor", scale_factor, 0.0105445269);
            nh_priv.param("history", with_history, false);
            nh_priv.param("offset", offset, 15.0);
            nh_priv.param("block_size", block_size, 31);
            nh_priv.param("no_x_output", no_x_output, false);
            nh_priv.param("frame_rate", frame_rate, 10);
            nh_priv.param("debugging", debugging, false);
            nh_priv.param("mean", mean, true);

            marker_pub = nh.advertise<marker_detect::MarkerInfos>(pub_topic, 10);
            marker_xy_pub = nh.advertise<marker_detect::MarkerXYs>(xy_pub_topic, 10);

            int size = topic.rfind('/');
            std::string camerainfotopic = topic.substr(0, size) + "/camera_info";
            ROS_INFO("Waiting for camera info on %s", camerainfotopic.c_str());
            sensor_msgs::CameraInfoConstPtr caminfo =
                ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camerainfotopic);

            cam_model.fromCameraInfo(caminfo);

            printf("Camera parameters - w: %d, h: %d. cx: %f, cy: %f, fx: %f, fy: %f\n",
                    cam_model.width(), cam_model.height(),
                    cam_model.cx(), cam_model.cy(),
                    cam_model.fx(), cam_model.fy());

            tracker = new MyTrackerROS(cam_model, num_markers, false);

            printf("Created marker tracker.\n");

            image_sub_ = it_.subscribe(topic, 1, &MarkerDetector::callback, this);

            out = cvCreateImage(cvSize(caminfo->width, caminfo->height), IPL_DEPTH_8U,3);
            ada = cvCreateImage(cvSize(caminfo->width, caminfo->height), IPL_DEPTH_8U,1);
        }

        ~MarkerDetector() {
            cvReleaseImage(&ada);
            cvReleaseImage(&out);

        }

        void callback(const sensor_msgs::ImageConstPtr& msg_ptr) {
            marker_detect::MarkerInfos marker_infos;
            marker_infos.header.frame_id = target_frame;
            marker_infos.header.stamp = msg_ptr->header.stamp;

            IplImage* cv_image = 0;
            try
            {
                cv_image = bridge_.imgMsgToCv(msg_ptr, "mono8");
                int method = mean ? CV_ADAPTIVE_THRESH_MEAN_C : CV_ADAPTIVE_THRESH_GAUSSIAN_C;
                cvAdaptiveThreshold(cv_image, ada,  255, mean, CV_THRESH_BINARY, block_size, offset);
                if (debugging)
                {
                    cvShowImage("Adaptive Threshold Results", ada);
                    cvWaitKey(50);
                    return;
                }
                
                if (!with_history) {
                    tracker->setImage(cv_image,false);
                }
                else {
                    tracker->setImage(cv_image,true);
                }

                vector<int> marker_ids;
                vector<geometry_msgs::Point> marker_xys;
                tracker->get_marker_ids(marker_ids);
                tracker->get_marker_xys(marker_xys);
                marker_detect::MarkerXYs xys;
                for (int i=0; i < marker_ids.size(); i++)
                    xys.ids.push_back(marker_ids[i]);
                for (int i=0; i < marker_xys.size(); i++)
                    xys.xys.push_back(marker_xys[i]);

                /* Work on trying to extract pose from corner positions
                int n_markers = tracker->detections_filtered.size();
                // Get pose estimate using corners of markers
                float extrinsic_poses[500][6]; //FIXME, hard-coding max markers
                for (int i=0; i < 500; i++)
                {
                    tracker->get_extrinsic_pose_estimate(i,
                            &extrinsic_poses[i][0]);
                    // Scale transform
                    for (int j=0; j<3; j++)
                        extrinsic_poses[i][j] *= 0.1; //scale_factor;
                }
                // Change this back later
                marker_infos.header.frame_id = camera_frame;
                // Transform into PoseStamped and check values
                for (int i=0; i<n_markers; i++)
                {
                    int mid = tracker->detections_filtered[i].info.id;
                    marker_infos.ids.push_back(mid);
                    geometry_msgs::Pose ps;
                    ps.position.x = extrinsic_poses[mid][0];
                    ps.position.y = extrinsic_poses[mid][1];
                    ps.position.z = extrinsic_poses[mid][2];
                    ps.orientation = tf::createQuaternionMsgFromRollPitchYaw(extrinsic_poses[mid][3],
                            extrinsic_poses[mid][4], extrinsic_poses[mid][5]);
                    marker_infos.poses.push_back(ps);
                }
                */

                for ( int i=0; i<tracker->detections_filtered.size(); i++){
                    //geometry_msgs::Pose marker_pose = toPose(tracker->detections_filtered[i].tf);
                    geometry_msgs::Pose marker_pose;
					tf::poseEigenToMsg(tracker->detections_filtered[i].tf.cast<double>(),marker_pose);
                    
                    marker_pose.position.x *= scale_factor;
                    marker_pose.position.y *= scale_factor;
                    marker_pose.position.z *= scale_factor;

                    
                    marker_infos.ids.push_back(tracker->detections_filtered[i].info.id);

                    
                    marker_infos.poses.push_back(marker_pose);
                    
                    xys.oris.push_back(marker_pose.orientation);
                }
                
                if(!no_x_output){
                    cvCvtColor(ada, out, CV_GRAY2BGR);
                    tracker->drawOnImage(out);
                    cvShowImage("Detected Markers", out);
                    cvWaitKey(50);
                }
                
                if(!marker_infos.ids.empty())
                {
                    marker_pub.publish(marker_infos);
                    marker_xy_pub.publish(xys);
                }
                
            }

            catch (sensor_msgs::CvBridgeException error){
                ROS_WARN("Error converting image to CV image.");
            }
            catch (tf::ExtrapolationException error){
                ROS_ERROR("TF exception %s", error.what());
                
            }

            ros::Rate r(frame_rate);
            r.sleep();
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "marker_detector_node");
    MarkerDetector md;
    ros::spin();
    return 0;
}
