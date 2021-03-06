
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <deque>

#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";

    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1; 
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)



    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time

    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */


    std::ofstream logger_ttc_camera = std::move(std::ofstream("camera.csv"));
    std::ofstream logger_ttc_lidar = std::move(std::ofstream("lidar.csv"));

    vector<string>  detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    vector<string>  descriptorTypes = {"BRIEF", "FREAK", "ORB", "AKAZE", "SIFT"};
    string detectorType = "ORB";
    string descriptorType = "ORB"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

    // for(auto &detectorType : detectorTypes)  for(auto &descriptorType : descriptorTypes) 
    { 
        std::deque<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time

        std::cout << "#############################################################\n";
        std::cout << detectorType << "-" << descriptorType << "\n";
        std::cout << "#############################################################\n";
        


        std::vector<float> ttcs_lidar;
        std::vector<float> ttcs_camera;

        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
        {
            /* LOAD IMAGE INTO BUFFER */

            // assemble filenames for current index
            ostringstream imgNumber;
            imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
            string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

            // load image from file 
            cv::Mat img = cv::imread(imgFullFilename);

            // push image into data frame buffer
            DataFrame frame;
            frame.cameraImg = img;
            frame.image_id = imgIndex;
            frame.maskImg = cv::Mat(img.rows, img.cols, CV_8UC1, 255).clone();

            dataBuffer.push_back(frame);
            // Limite the ring buffer size
            if(dataBuffer.size() > dataBufferSize) dataBuffer.pop_front();

            /* DETECT & CLASSIFY OBJECTS */
            // bVis = true;
            float confThreshold = 0.2;
            float nmsThreshold = 0.4;        
            detectObjects((dataBuffer.end() - 1)->cameraImg, 
                        (dataBuffer.end() - 1)->boundingBoxes, 
                        confThreshold, nmsThreshold,
                        yoloBasePath, yoloClassesFile, 
                        yoloModelConfiguration, yoloModelWeights, bVis);
            bVis = false;
            cout << "DETECT & CLASSIFY OBJECTS done. Number of detected objects: " 
                << (dataBuffer.end() - 1)->boundingBoxes.size() << endl;

            // Produce a mask image based on the detected bounding boxes
            // Each boundbing box region will be colored by the boxID
            generateDetectedImageMask((dataBuffer.end()-1)->cameraImg, (dataBuffer.end()-1)->maskImg, (dataBuffer.end()-1)->boundingBoxes, imgIndex);

            /* CROP LIDAR POINTS */

            // load 3D Lidar points from file
            string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
            std::vector<LidarPoint> lidarPoints;
            loadLidarFromFile(lidarPoints, lidarFullFilename);

            // remove Lidar points based on distance properties
            float minZ = -1.5, maxZ = -0.5, minX = 1.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
            cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
        
            (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

            cout << "#3 : CROP LIDAR POINTS done" << endl;


            /* CLUSTER LIDAR POINT CLOUD */

            // associate Lidar points with camera-based ROI
            float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
            clusterLidarWithROI((dataBuffer.end()-1)->boundingBoxes, 
                (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);



            // Visualize 3D objects
            bVis = false;
            if(bVis)
            {
                show3DObjects((dataBuffer.end()-1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), true);
            }
            bVis = false;

            
            // REMOVE THIS LINE BEFORE PROCEEDING WITH THE FINAL PROJECT
            // continue; // skips directly to the next image without processing what comes beneath

            /* DETECT IMAGE KEYPOINTS */

            // convert current image to grayscale
            cv::Mat imgGray;
            cv::cvtColor((dataBuffer.end()-1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

            // extract 2D keypoints from current image
            vector<cv::KeyPoint> keypoints; // create empty feature list for current image
            bVis = false;
            detKeypointsModern(keypoints, imgGray, detectorType, bVis);
            bVis = false;


            // optional : limit number of keypoints (helpful for debugging and learning)
            bool bLimitKpts = false;
            if (bLimitKpts)
            {
                int maxKeypoints = 50;

                if (detectorType.compare("SHITOMASI") == 0)
                { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                }
                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                cout << " NOTE: Keypoints have been limited!" << endl;
            }


            

            // push keypoints and descriptor for current frame to end of data buffer
            (dataBuffer.end() - 1)->keypoints = keypoints;

            cout << "#5 : DETECT KEYPOINTS done" << endl;
            
            // Only keep the points are within the detected boundbox
            vector<cv::KeyPoint> keypoints_roi;
            for(auto &kp : keypoints) 
            {
                int x = kp.pt.x, y = kp.pt.y;
                // This location has a valid box id, then keep this keypoint
                int box_id = (dataBuffer.end()-1)->maskImg.at<uint8_t>(y, x);
                if(box_id != 255 && (dataBuffer.end()-1)->boundingBoxes[box_id].lidarPoints.size() > 0) 
                {
                    keypoints_roi.push_back(kp);
                    (dataBuffer.end()-1)->boundingBoxes[box_id].keypoints.push_back(kp);
                }
            }
            (dataBuffer.end() - 1)->keypoints = std::move(keypoints_roi);

            /* EXTRACT KEYPOINT DESCRIPTORS */

            cv::Mat descriptors;

            descKeypoints((dataBuffer.end() - 1)->keypoints, 
                        (dataBuffer.end() - 1)->cameraImg, 
                        descriptors, descriptorType);

            // push descriptors for current frame to end of data buffer
            (dataBuffer.end() - 1)->descriptors = descriptors;

            cout << "#6 : EXTRACT DESCRIPTORS done" << endl;


            cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
            for(auto &bbox : (dataBuffer.end()-1)->boundingBoxes)
            {
                showLidarImgOverlay(visImg, bbox.lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                if(bbox.lidarPoints.size() != 0)
                    cv::drawKeypoints(visImg, bbox.keypoints, visImg);
                cv::rectangle(visImg, cv::Point(bbox.roi.x, bbox.roi.y), 
                                    cv::Point(bbox.roi.x + bbox.roi.width, bbox.roi.y + bbox.roi.height), 
                                    cv::Scalar(0, 255, 0), 2);
                if(bbox.lidarPoints.size() != 0)
                    std::cout << "bbox with lidar points " << bbox.boxID << " lidar points size " << bbox.lidarPoints.size() << "\n"; 
            }
            cv::imwrite("lidar_bbox_" + std::to_string(imgIndex) + ".jpg", visImg);

            double ttcLidar = 0.0, ttcCamera = 0.0;
            
            if (dataBuffer.size() > 1) // wait until at least two images have been processed
            {
                /* MATCH KEYPOINT DESCRIPTORS */
                vector<cv::DMatch> matches;
                string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
                
                matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                matches, descriptorType, matcherType, selectorType);

                // store matches in current data frame
                (dataBuffer.end() - 1)->kptMatches = matches;

                cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                
                /* TRACK 3D OBJECT BOUNDING BOXES */

                //// STUDENT ASSIGNMENT
                //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between 
                // current and previous frame (implement ->matchBoundingBoxes)
                // associate bounding boxes between current and previous frame using keypoint matches
                map<int, int> bbBestMatches;
                matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1) ); 
                
                //// EOF STUDENT ASSIGNMENT

                // store matches in current data frame
                (dataBuffer.end()-1)->bbMatches = bbBestMatches;

                cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done ----> " << imgIndex << " " << bbBestMatches.size() << "\n";



                /* COMPUTE TTC ON OBJECT IN FRONT */

                // loop over all BB match pairs
                for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
                {
                    // find bounding boxes associates with current match
                    BoundingBox *prevBB, *currBB;
                    for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
                    {
                        if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                        {
                            currBB = &(*it2);
                        }
                    }

                    for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
                    {
                        if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                        {
                            prevBB = &(*it2);
                        }
                    }

                    // compute TTC for current match
                    if( currBB->lidarPoints.size()> 10 && prevBB->lidarPoints.size()> 10) // only compute TTC if we have Lidar points
                    {
                        //// STUDENT ASSIGNMENT
                        //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                        computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                        cout << "#9 : LIDAR TTC is done\n";

                        //// EOF STUDENT ASSIGNMENT

                        //// STUDENT ASSIGNMENT
                        //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                        


                        //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)

                        // This has been done inside matchBoundingBoxes
                        // clusterKptMatchesWithROI(*currBB, 
                        //                         (dataBuffer.end() - 2)->keypoints, 
                        //                         (dataBuffer.end() - 1)->keypoints, 
                        //                         (dataBuffer.end() - 1)->kptMatches);             

                        computeTTCCamera((dataBuffer.end() - 2)->keypoints, 
                                        (dataBuffer.end() - 1)->keypoints, 
                                        currBB->kptMatches, 
                                        sensorFrameRate, 
                                        ttcCamera);
                        //// EOF STUDENT ASSIGNMENT
                        cout << "#11 : Camera TTC is done\n";

                        bVis = false;
                        if (bVis)
                        {
                            cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                            showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                            cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);
                            
                            char str[200];
                            sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                            putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));

                            cv::imwrite("visImg" + std::to_string(imgIndex) + ".jpg", visImg);
                            string windowName = "Final Results : TTC";
                            cv::namedWindow(windowName, 4);
                            cv::imshow(windowName, visImg);
                            cout << "Press key to continue to next frame" << endl;
                            cv::waitKey(0);
                        }
                        bVis = false;

                    } // eof TTC computation

                } // eof loop over all BB matches            
                ttcs_lidar.push_back(ttcLidar);
                ttcs_camera.push_back(ttcCamera);
            }

        } // eof loop over all images
        
    // Save the result into csv 
    string key = detectorType + "-" + descriptorType;
    logger_ttc_lidar << key;
    logger_ttc_camera << key;
    for(int i = 0; i < ttcs_lidar.size(); i++)
    {
        logger_ttc_lidar << ", " << ttcs_lidar[i];
        logger_ttc_camera << ", " << ttcs_camera[i];
    }
    logger_ttc_lidar << "\n";
    logger_ttc_camera << "\n";
    logger_ttc_lidar.flush();
    logger_ttc_camera.flush();
    }

    logger_ttc_lidar.close();
    logger_ttc_camera.close();
    return 0;
}
