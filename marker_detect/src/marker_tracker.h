/*
 * marker_tracker.h
 * Created on: Jun 3, 2010
 * Author: Christian Bersch
 * Modified by: Ziang Xie
 */

#ifndef MARKERTRACKER_H_
#define MARKERTRACKER_H_

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <cstdio>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ARToolKitPlus/TrackerSingleMarker.h>

#define EPS 0.000001 // For float and double comparison
#define CAP 1000000.0 // For checking tvec/rvec components in bounds

using ARToolKitPlus::TrackerSingleMarker;

using std::cout;
using std::endl;

void printCvMat(CvMat* A, int rows, int cols) {
    cout << std::setprecision(2) << std::right << std::fixed;
    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            cout << std::setw(4) << (double) cvmGet(A, row, col) << " ";
        }
        cout << endl;
    }
}

class TrackerCam: public ARToolKitPlus::Camera{

    public:

        TrackerCam( const image_geometry::PinholeCameraModel& cam_model, bool distorted) {
            xsize = cam_model.width();
            ysize = cam_model.height();
            cc[0] = (ARFloat) cam_model.cx();
            cc[1] = (ARFloat) cam_model.cy();
            fc[0] = (ARFloat) cam_model.fx();
            fc[1] = (ARFloat) cam_model.fy();

            int i,j;
            for (i = 0; i < 5; i++) {
                if (distorted)
                    kc[i] = (ARFloat) cam_model.distortionCoeffs().at<double>(0,i);
                else
                    kc[i] = 0.0f;
            }
            kc[5] = (ARFloat) 0.0f;

            for (i = 0; i < 3; i++)
                for (j = 0; j < 4; j++)
                    mat[i][j] = 0.;

            mat[0][0] = fc[0]; // fc_x
            mat[1][1] = fc[1]; // fc_y
            mat[0][2] = cc[0]; // cc_x
            mat[1][2] = cc[1]; // cc_y
            mat[2][2] = 1.0;

            undist_iterations = 1;
        }

};

struct Detection{
    ARToolKitPlus::ARMarkerInfo info;
    Eigen::Affine3f tf;
    bool operator<(const Detection& other) const { return (info.id < other.info.id); }
};

bool sortDetection (const Detection& d1, const Detection& d2) {
    return (d1.info.id < d2.info.id);
}

class MyTrackerROS:public TrackerSingleMarker {

    public:

        static CvScalar color[19];
        CvFont font;

        vector<Eigen::Affine3f> transforms;
        vector<int> markerId;
        vector<int> markerId_filtered;
        vector<Detection> detections_filtered;
        vector<ARToolKitPlus::ARMarkerInfo> markerInfos_filtered;
        int max_markers_;

        int block_size_;
        double offset_;
        bool cv_thresholding_;

        // #FIXME: Currently assuming we'll never be dealing w/ more than 500 markers
        float extrinsic_pose_estimates[500][6];

        MyTrackerROS(int width, int height, int max_markers = 6):
            max_markers_(max_markers),
            TrackerSingleMarker(width, height, max_markers, 6, 6, 6, 0) {
                setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
                init("no_distortion.cal", 1.0f, 1000.0f);
                getCamera()->printSettings();
                // setBorderWidth(0.0625);
                setBorderWidth(0.125f);
                initTracker();
            }

        // Assumes image has been pre-thresholded with OpenCV
        void setCvThresholding(bool cv) {
            if (!cv) {
                activateAutoThreshold(true);
                cv_thresholding_ =false;
            } else {
                activateAutoThreshold(false);
                setThreshold(127);
                cv_thresholding_ = true;
            }
        }

        MyTrackerROS(const image_geometry::PinholeCameraModel& cam_model, int max_markers = 6, bool distorted = false):
            TrackerSingleMarker(cam_model.width(), cam_model.height(), max_markers, 6, 6, 6, 0), max_markers_(max_markers) {
                setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
                if (!this->checkPixelFormat()) {
                    ROS_ERROR("MyTrackerROS: Invalid Pixel Format!");
                }

                // Initialize some "static" members from ARToolkit
                // (Some systems don't like such large global members so we allocate this manually)
                if (this->marker_infoTWO == NULL)
                    this->marker_infoTWO = new ARToolKitPlus::ARMarkerInfo2[MAX_IMAGE_PATTERNS];

                ARToolKitPlus::Camera* cam = new TrackerCam(cam_model, distorted);
                if (arCamera)
                    delete arCamera;
                arCamera = NULL;
                setCamera(cam, 1.0f, 1000.0f);
                getCamera()->printSettings();

                // setBorderWidth(0.0625);
                setBorderWidth(0.125f);
                initTracker();
            }

        vector<int> calcWithoutHistory(const uint8_t* nImage, ARToolKitPlus::ARMarkerInfo** nMarker_info = NULL, int* nNumMarkers = NULL);

        vector<int>  calc(const uint8_t* nImage, bool with_history = true , ARToolKitPlus::ARMarkerInfo** nMarker_info = NULL, int* nNumMarkers = NULL) {
            if (with_history)
                return TrackerSingleMarker::calc(nImage, nMarker_info,  nNumMarkers);
            else
                return this->calcWithoutHistory(nImage, nMarker_info,  nNumMarkers);
        }

        void initTracker() {
            setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);

            setCvThresholding(false);

            // Use lookup-table undistortion for high-speed
            // Note: LUT only works with images up to 1024x1024 (reason doesn't work w/ Prosilica)
            setUndistortionMode(ARToolKitPlus::UNDIST_NONE);
            setImageProcessingMode(ARToolKitPlus::IMAGE_FULL_RES);

            // Switch to simple ID-based markers
            setMarkerMode( ARToolKitPlus::MARKER_ID_BCH);

            cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 2);
        }


        void setImage(IplImage* img, bool with_history = true, bool thresholding = true) {
            assert(img->depth == IPL_DEPTH_8U  && img->width == screenWidth && img->height == screenHeight);

            markerId = calc((unsigned char*)img->imageData, with_history);
            postprocess();
        }

        void setImage(const sensor_msgs::ImageConstPtr& msg, bool with_history = true) {
            markerId = calc(msg->data.data(), with_history);
            postprocess();
        }

        void postprocess() {
            int i_info =0;

            transforms.clear();
            markerId_filtered.clear();
            markerInfos_filtered.clear();
            detections_filtered.clear();

            for (int i=0; i<markerId.size(); i++) {
                if (markerId[i] >= max_markers_) {
                    continue;
                }
                selectDetectedMarker(markerId[i]);

                Eigen::Affine3f tf;
                memcpy(tf.data(), getModelViewMatrix(), 16 * sizeof(float));
                if (tf(0,0) == 0) {
                    continue;
                }
                markerId_filtered.push_back(markerId[i]);
                transforms.push_back(tf);

                while (marker_info[i_info].id != markerId[i]  ) {
                    i_info++;
                }
                markerInfos_filtered.push_back(marker_info[i_info]);
                Detection dt = {marker_info[i_info] , tf};
                detections_filtered.push_back(dt);

            }
        }

        bool pointsAreOnTopLine(const int markeridx, const cv::Point2d p1, const cv::Point2d& p2) {
            ARToolKitPlus::ARMarkerInfo& marker =  markerInfos_filtered[markeridx];
            float a = marker.line[marker.dir][0];
            float b = marker.line[marker.dir][1];
            float c = marker.line[marker.dir][2];

            double dist1 = fabs(a*p1.x + b*p1.y + c);
            double dist2 = fabs(a*p2.x + b*p2.y + c);

            return  (dist1 < 3.0 && dist2 < 3.0) ? true : false;
        }

        void get_marker_ids(vector<int>& marker_ids)
        {
            marker_ids.clear();
            for (int i = 0; i < markerId_filtered.size(); i++)
            {
                marker_ids.push_back(markerId_filtered[i]);
            }
        }

        void get_marker_xys(vector<geometry_msgs::Point>& marker_xys)
        {
            marker_xys.clear();
            for (int i = 0; i < markerId_filtered.size(); i++)
            {
                geometry_msgs::Point p;
                p.x = markerInfos_filtered[i].pos[0];
                p.y = markerInfos_filtered[i].pos[1];
                p.z = 0.0;
                marker_xys.push_back(p);
            }
        }

        /* 
         * For verification of ARToolkitPlus pose estimates; estimates obtained
         * by using coordinates of marker corners in image and OpenCV's
         * FindExtrinsicCameraParams2 function
         */
        void get_extrinsic_pose_estimate(int id, float* ps)
        {
            for (int i = 0; i < 6; i++)
                ps[i] = extrinsic_pose_estimates[id][i];
        }

        void drawOnImage(IplImage* img) {
            char buffer [255];

            for (int i=0; i<markerId_filtered.size(); i++) {
                int m_id = markerId_filtered[i];

                // Figure out the top left corner index (in image)
                int lc_index = 0;
                float maxSum = 0.0;
                for (int k = 0; k < 4; k++) {
                    // This should work assuming marker isn't rotated in very specific angles
                    // Finds bottom right corner (in image) index, + 2 to get top left
                    // Vertices are indexed counterclockwise
                    float sum = markerInfos_filtered[i].vertex[k][0] + markerInfos_filtered[i].vertex[k][1];
                    if (sum > maxSum) {
                        maxSum = sum;
                        lc_index = (k+2)%4;
                    }
                }

                // Vertices -> cvPoints
                float corner_xys[8] = {
                    markerInfos_filtered[i].vertex[(lc_index)%4][0], markerInfos_filtered[i].vertex[(lc_index)%4][1],
                    markerInfos_filtered[i].vertex[(lc_index+1)%4][0], markerInfos_filtered[i].vertex[(lc_index+1)%4][1],
                    markerInfos_filtered[i].vertex[(lc_index+2)%4][0], markerInfos_filtered[i].vertex[(lc_index+2)%4][1],
                    markerInfos_filtered[i].vertex[(lc_index+3)%4][0], markerInfos_filtered[i].vertex[(lc_index+3)%4][1]
                };
                CvPoint2D32f vertices[5] = { // Includes center point as well
                    cvPoint2D32f(corner_xys[0], corner_xys[1]),
                    cvPoint2D32f(corner_xys[2], corner_xys[3]),
                    cvPoint2D32f(corner_xys[4], corner_xys[5]),
                    cvPoint2D32f(corner_xys[6], corner_xys[7]),
                    cvPoint2D32f(markerInfos_filtered[i].pos[0], markerInfos_filtered[i].pos[1])
                };

                /* Work on trying to extract pose from corner positions
                // Unit square in xy-plane, top left first, counterclockwise
                double unwarped_xyzs[15] = {
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0,
                    1.0, 0.0, 0.0,
                    1.0, 1.0, 0.0,
                    0.5, 0.5, 0.0
                };

                const CvMat *object_pts = &cvMat(3, 4, CV_32FC1, unwarped_xyzs);
                const CvMat *image_pts = &cvMat(2, 4, CV_32FC1, corner_xys);
                float cam_mat[3][3];
                for (int row=0; row<3; row++)
                    for (int col=0; col<3; col++)
                        cam_mat[row][col] = getCamera()->mat[row][col];
                const CvMat *camera_mat = &cvMat(3, 3, CV_32FC1, cam_mat);
                const CvMat *distortion_mat = &cvMat(5, 1, CV_32FC1, getCamera()->kc); // Should be all 0.0
                CvMat* rvec = cvCreateMat(1, 3, CV_32FC1);
                CvMat* tvec = cvCreateMat(1, 3, CV_32FC1);
                cvFindExtrinsicCameraParams2(
                        object_pts,
                        image_pts,
                        camera_mat,
                        distortion_mat,
                        rvec,
                        tvec,
                        1
                );

                if (fabs(cvmGet(tvec, 0, 0)) < CAP && fabs(cvmGet(tvec, 0, 1)) < CAP && fabs(cvmGet(tvec, 0, 2)) < CAP &&
                        fabs(cvmGet(rvec, 0, 0)-0.0)> EPS && fabs(cvmGet(rvec, 0, 1)-0.0)>EPS && fabs(cvmGet(rvec, 0, 2)-0.0) > 0.0 &&
                        fabs(cvmGet(rvec, 0, 0)) <= 4.0 && fabs(cvmGet(rvec, 0, 1)) <= 4.0 && fabs(cvmGet(rvec, 0, 2)) <= 4.0)
                    // Make sure estimate didn't blow up
                {
                    extrinsic_pose_estimates[m_id][0] = cvmGet(tvec, 0, 0);
                    extrinsic_pose_estimates[m_id][1] = cvmGet(tvec, 0, 1);
                    extrinsic_pose_estimates[m_id][2] = cvmGet(tvec, 0, 2);
                    extrinsic_pose_estimates[m_id][3] = cvmGet(rvec, 0, 0);
                    extrinsic_pose_estimates[m_id][4] = cvmGet(rvec, 0, 1);
                    extrinsic_pose_estimates[m_id][5] = cvmGet(rvec, 0, 2);
                }
                */

                for (int j=0; j<4; j++) {

                    // Vertices -> cvPoints
                    CvPoint vertex1 = cvPointFrom32f(vertices[j]);
                    CvPoint vertex2 = cvPointFrom32f(vertices[(j+1)%4]);

                    // Colors
                    CvScalar color1 = color[markerInfos_filtered[i].id%19];
                    CvScalar color2 = color[(markerInfos_filtered[i].id+7)%19];
                    
                    if (!pointsAreOnTopLine(i, vertex1, vertex2)) {
                        cvLine(img, vertex1, vertex2, color1, 2.5);
                    } else {
                        cvLine(img, vertex1, vertex2, color2, 2.5);
                    }

                    sprintf(buffer, "%3.3d", markerInfos_filtered[i].id);
                    cvPutText(img, buffer, cvPoint(markerInfos_filtered[i].pos[0], markerInfos_filtered[i].pos[1]),
                            &font, color1);

                    // Draw point in marker center to verify ARMarkerInfos pos what we want in stereo_marker_detect
                    // cvCircle(img, cvPoint(markerInfos_filtered[i].pos[0], markerInfos_filtered[i].pos[1]), 1, CvScalar(cvScalar(255, 0, 0, 0)));

                    // Draw point at top left corner
                    // cvCircle(img, cvPoint(markerInfos_filtered[i].vertex[lc_index][0], markerInfos_filtered[i].vertex[lc_index][1]), 1, CvScalar(cvScalar(0, 0, 255, 0)), 2);
                }
            }

        }

    private:

        IplImage* current;

};


vector<int> MyTrackerROS::calcWithoutHistory(const uint8_t* nImage, ARToolKitPlus::ARMarkerInfo** nMarker_info, int* nNumMarkers) {
    vector<int> detected;
    if (nImage == NULL)
        return detected;

    confidence = 0.0f;

    // Detect the markers in the video frame
    if (arDetectMarkerLite(const_cast<unsigned char*> (nImage), this->thresh, &marker_info, &marker_num) < 0) {
        return detected;
    }

    // Copy all valid ids
    for (int j = 0; j < marker_num; j++) {
        if (marker_info[j].id != -1) {
            detected.push_back(marker_info[j].id);
        }
    }

    if (nMarker_info)
        *nMarker_info = marker_info;

    if (nNumMarkers)
        *nNumMarkers = marker_num;

    return detected;
}

CvScalar MyTrackerROS::color[] = {
    CvScalar(cvScalar(255,0,0,0)),
    CvScalar(cvScalar(0,255,0,0)),
    CvScalar(cvScalar(0,0,255,0)),
    CvScalar(cvScalar(255,255,0,0)),
    CvScalar(cvScalar(255,0,255,0)),
    CvScalar(cvScalar(0,255,255,0)),
    CvScalar(cvScalar(127,0,0,0)),
    CvScalar(cvScalar(0,127,0,0)),
    CvScalar(cvScalar(0,0,127,0)),
    CvScalar(cvScalar(127,127,0,0)),
    CvScalar(cvScalar(127,0,127,0)),
    CvScalar(cvScalar(0,127,127,0)),
    CvScalar(cvScalar(127,255,0,0)),
    CvScalar(cvScalar(127,0,255,0)),
    CvScalar(cvScalar(0,127,255,0)),
    CvScalar(cvScalar(255,127,0,0)),
    CvScalar(cvScalar(255,0,127,0)),
    CvScalar(cvScalar(0,255,127,0)),
    CvScalar(cvScalar(127,127,127,0))
};

#endif
