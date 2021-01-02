/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <unordered_map>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    vector<string>  detectorTypes = {"HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    vector<string>  descriptorTypes = {"BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};


    std::ofstream logger_mp7 = std::move(std::ofstream("mp7_num_keypoints.csv"));
    std::ofstream logger_mp8 = std::move(std::ofstream("mp8_num_matched_keyoints.csv"));
    std::ofstream logger_mp9 = std::move(std::ofstream("mp9_time.csv"));

    logger_mp7 << "Detector, Frame0, Frame1, Frame2, Frame3, Frame4, Frame5, Frame6, Frame7, Frame8, Frame9, Total, Average\n";
    logger_mp8 << "Descriptor, Frame0, Frame1, Frame2, Frame3, Frame4, Frame5, Frame6, Frame7, Frame8, Frame9, Total, Average\n";
    logger_mp9 << "Detector-Descriptor, Frame0, Frame1, Frame2, Frame3, Frame4, Frame5, Frame6, Frame7, Frame8, Frame9, Total, Average\n";
    
                
    vector<int> num_keypoints(imgEndIndex + 1, 0);
    vector<int> num_matched_keypoints(imgEndIndex + 1, 0);
    vector<pair<float, string>> perf_time; 

    for(auto &detectorType : detectorTypes) 
    {
        for(auto &descriptorType : descriptorTypes) 
        {
            // misc
            int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
            vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
            bool bVis = false;            // visualize results

            dataBuffer.resize(dataBufferSize);

            vector<float> time_keypoints_det;
            vector<float> time_keypoints_descriptor;

            /* MAIN LOOP OVER ALL IMAGES */
            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
                /* LOAD IMAGE INTO BUFFER */

                // assemble filenames for current index
                ostringstream imgNumber;
                imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                // load image from file and convert to grayscale
                cv::Mat img, imgGray;
                img = cv::imread(imgFullFilename);
                cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                //// STUDENT ASSIGNMENT
                //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

                // push image into data frame buffer
                DataFrame frame;
                frame.cameraImg = imgGray;
                dataBuffer[imgIndex % dataBufferSize] = frame;

                //// EOF STUDENT ASSIGNMENT
                cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

                /* DETECT IMAGE KEYPOINTS */

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image


                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
                float t1 = detKeypointsModern(keypoints, imgGray, detectorType, false);
                

                time_keypoints_det.push_back(t1);


                //// EOF STUDENT ASSIGNMENT

                //// STUDENT ASSIGNMENT
                //// TASK MP.3 -> only keep keypoints on the preceding vehicle

                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;
                cv::Rect vehicleRect(535, 180, 180, 150);
                if (bFocusOnVehicle)
                {
                    vector<cv::KeyPoint> tmp;
                    for(auto &kp : keypoints) 
                    {
                        float x = kp.pt.x, y = kp.pt.y;
                        if(x >= vehicleRect.x && x <= vehicleRect.x + vehicleRect.width && 
                        y >=  vehicleRect.y && y <= vehicleRect.y + vehicleRect.height) 
                        {
                            tmp.push_back(kp);
                        }
                    }
                    keypoints = tmp;
                }
                num_keypoints[imgIndex] = keypoints.size();

                //// EOF STUDENT ASSIGNMENT

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
                dataBuffer[imgIndex % dataBufferSize].keypoints = keypoints;
                cout << "#2 : DETECT KEYPOINTS done" << endl;

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// STUDENT ASSIGNMENT
                //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

                cv::Mat descriptors;

                float t2 = descKeypoints(dataBuffer[imgIndex % dataBufferSize].keypoints, 
                            dataBuffer[imgIndex % dataBufferSize].cameraImg, 
                            descriptors, 
                            descriptorType);
                //// EOF STUDENT ASSIGNMENT
                time_keypoints_descriptor.push_back(t2);
                // push descriptors for current frame to end of data buffer
                dataBuffer[imgIndex % dataBufferSize].descriptors = descriptors;

                cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

                int num_matches = 0;
                if (imgIndex > 1) // wait until at least two images have been processed
                {

                    /* MATCH KEYPOINT DESCRIPTORS */

                    vector<cv::DMatch> matches;
                    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                    string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
                    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                    matchDescriptors(dataBuffer[(imgIndex  - 1)% dataBufferSize].keypoints, 
                                    dataBuffer[imgIndex % dataBufferSize].keypoints,
                                    dataBuffer[(imgIndex  - 1)% dataBufferSize].descriptors, 
                                    dataBuffer[imgIndex % dataBufferSize].descriptors,
                                    matches, descriptorType, matcherType, selectorType);

                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    dataBuffer[imgIndex % dataBufferSize].kptMatches = matches;

                    cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
                    num_matches = matches.size();
                    // visualize matches between current and previous image
                    bVis = false;
                    if (bVis)
                    {
                        cv::Mat matchImg = (dataBuffer[imgIndex % dataBufferSize].cameraImg).clone();
                        cv::drawMatches(dataBuffer[(imgIndex  - 1)% dataBufferSize].cameraImg, dataBuffer[(imgIndex  - 1)% dataBufferSize].keypoints,
                                        dataBuffer[imgIndex % dataBufferSize].cameraImg, dataBuffer[imgIndex % dataBufferSize].keypoints,
                                        matches, matchImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, matchImg);
                        cout << "Press key to continue to next image" << endl;
                        cv::waitKey(0); // wait for key to be pressed
                    }
                    bVis = false;
                }
                num_matched_keypoints[imgIndex] = num_matches;
            } // eof loop over all images


            string key = detectorType + "-" + descriptorType;
            float total_keypoints = 0, total_matched = 0, total_det_time = 0, total_match_time = 0;
            logger_mp7 << detectorType << ", ";
            logger_mp8 << key << ", ";
            logger_mp9 << key << " detection, ";
            
            for(int i = 0; i < 10; i++) 
            {
                logger_mp7 << num_keypoints[i] << ", ";
                logger_mp8 << num_matched_keypoints[i] << ", ";
                logger_mp9 << time_keypoints_det[i] << ", ";

                total_keypoints += num_keypoints[i];
                total_matched += num_matched_keypoints[i];
                total_det_time += time_keypoints_det[i];
            }
            logger_mp7 << total_keypoints << ", " << total_keypoints / 10 << "\n";
            logger_mp8 << total_matched << ", " << total_matched / 10 << "\n";


            logger_mp9 << total_det_time << ", " << total_det_time / 10 << "\n";

            logger_mp9 << key << " matching, ";
            for(int i = 0; i < 10; i++) 
            {
                logger_mp9 << time_keypoints_descriptor[i] << ", ";
                total_match_time += time_keypoints_descriptor[i];
            }
            logger_mp9 << total_match_time << ", " << total_match_time / 10 << "\n";


            logger_mp9 << key << " combined, ";
            for(int i = 0; i < 10; i++) 
            {
                logger_mp9 << time_keypoints_det[i] + time_keypoints_descriptor[i] << ", ";
            }
            logger_mp9 << total_det_time + total_match_time << ", " << ( total_det_time + total_match_time) / 10 << "\n";

            perf_time.push_back({(total_det_time + total_match_time) / 10, key});
        } 
    }
    logger_mp7.flush();
    logger_mp8.flush();
    logger_mp7.close();
    logger_mp8.close();


    logger_mp9.flush();
    logger_mp9.close();  

    sort(perf_time.begin(), perf_time.end());
    std::cout << "Top 1 " << perf_time[0].second << " avg_time " << perf_time[0].first << "\n";
    std::cout << "Top 2 " << perf_time[1].second << " avg_time " << perf_time[1].first << "\n";
    std::cout << "Top 3 " << perf_time[2].second << " avg_time " << perf_time[2].first << "\n";

    return 0;
}
