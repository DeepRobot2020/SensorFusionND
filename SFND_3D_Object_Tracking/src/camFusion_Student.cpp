
#include <iostream>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <chrono>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>


#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


void generateDetectedImageMask(cv::Mat &camImage, cv::Mat &mask, std::vector<BoundingBox> &boundingBoxes, int imgIndex)
{
    // create
    for(int i = 0; i < boundingBoxes.size(); i++)
    {
        cv::rectangle(mask, boundingBoxes[i].roi, boundingBoxes[i].boxID, cv::FILLED);
    }

    cv::Mat visImg = mask.clone();
    cv::Mat visImg2 = camImage.clone();
    
    for(int i = 0; i < boundingBoxes.size(); i++)
    {
        int box_id = boundingBoxes[i].boxID;
        int xc = boundingBoxes[i].roi.x + boundingBoxes[i].roi.width / 2;
        int yc = boundingBoxes[i].roi.y + boundingBoxes[i].roi.height / 2;
        char str[200];
        sprintf(str, "%d ", box_id);
        cv::putText(visImg, str, cv::Point2f(xc, yc), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));


        int x0 = boundingBoxes[i].roi.x;
        int y0 = boundingBoxes[i].roi.y;
        int x1 = boundingBoxes[i].roi.x + boundingBoxes[i].roi.width;
        int y1 = boundingBoxes[i].roi.y + boundingBoxes[i].roi.height;
        cv::rectangle(visImg2, cv::Point(x0, y0), cv::Point(x1, y1),cv::Scalar(0, 255, 0), 2);
        char str2[200];
        sprintf(str2, "box_id = %d ", box_id);
        cv::putText(visImg2, str2, cv::Point2f(xc, yc), cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(0, 255, 0));
    }
    cv::imwrite("mask_" + std::to_string(imgIndex) + ".jpg", visImg);
    cv::imwrite("det_" + std::to_string(imgIndex) + ".jpg", visImg2);   
}

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, 
                         std::vector<LidarPoint> &lidarPoints, 
                         float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, 
    std::vector<cv::KeyPoint> &kptsPrev, 
    std::vector<cv::KeyPoint> &kptsCurr, 
    std::vector<cv::DMatch> &kptMatches)
{
    // This has been solved inside matchBoundingBoxes
}




// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 75.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}

// Fit the lidar points on the cars with a plane and return the medium X of all the inliers
double ransacPlaneFitting(std::vector<LidarPoint> &points, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand() % (points.size()));

		auto itr = inliers.begin();
		float x1 = points[*itr].x;
		float y1 = points[*itr].y; 
        float z1 = points[*itr].z; 
        
        itr++;
		float x2 = points[*itr].x;
		float y2 = points[*itr].y; 
        float z2 = points[*itr].z; 

        itr++;
		float x3 = points[*itr].x;
		float y3 = points[*itr].y; 
        float z3 = points[*itr].z; 

        float a, b, c, d;
        a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        d = -(a * x1 + b * y1 + c * z1);
        float norm = sqrt(a * a + b * b + c * c);

		for(int index = 0; index < points.size(); index++) {
			if(inliers.count(index) > 0) continue;
			auto pt = points[index];
			float x4 = pt.x, y4 = pt.y, z4 = pt.z;
            float dist = fabs(a * x4 + b * y4 + c * z4 + d) / norm;

			if(dist<= distanceThreshold) inliers.insert(index);
		}	
		if(inliers.size() > inliersResult.size()) 
		{
			inliersResult = inliers;
		}
	}
    // find the medium X
    std::vector<double> values;
    for(auto i : inliersResult)
    {
        values.push_back(points[i].x);
    }
    std::sort(values.begin(), values.end());
    if(values.size() % 2) return values[values.size() / 2];
    return 0.5 * (values[values.size() / 2] + values[values.size() / 2 - 1]);
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1.0 / frameRate; // time between two measurements in seconds
    // 15: max iterations, 0.25 distance tolerance 
    double xPrev = ransacPlaneFitting(lidarPointsPrev, 15, 0.25);
    double xCurr = ransacPlaneFitting(lidarPointsCurr, 15, 0.25);
    // compute TTC from both measurements
    TTC = xCurr * dT / (xPrev - xCurr);
}

void computeTTCLidarFromLecture(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1.0 / frameRate; // time between two measurements in seconds

    // find closest distance to Lidar points 
    double minXPrev = 1e9, minXCurr = 1e9;
    for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) {
        minXPrev = minXPrev>it->x ? it->x : minXPrev;
    }

    for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) {
        minXCurr = minXCurr>it->x ? it->x : minXCurr;
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev-minXCurr);
}



void matchBoundingBoxes(std::vector<cv::DMatch> &matches, 
                        std::map<int, int> &bbBestMatches, 
                        DataFrame &prevFrame, 
                        DataFrame &currFrame)
{
    cv::Mat outImg;
    // draw matched keypoints
    cv::drawMatches(prevFrame.cameraImg, prevFrame.keypoints, 
                    currFrame.cameraImg, currFrame.keypoints,
                    matches, outImg);
    cv::Mat outImg1;
    cv::drawKeypoints(prevFrame.cameraImg, prevFrame.keypoints, outImg1);
    cv::Mat outImg2;
    cv::drawKeypoints(currFrame.cameraImg, currFrame.keypoints, outImg2);

    cv::imwrite("test_" + std::to_string(currFrame.image_id) + ".jpg", outImg);
    cv::imwrite("test1_" + std::to_string(currFrame.image_id) + ".jpg", outImg1);
    cv::imwrite("test2_" + std::to_string(currFrame.image_id) + ".jpg", outImg2);
    
    std::unordered_map<int, std::unordered_map<int, int>> freq;
    for(auto &match : matches) 
    {
        cv::KeyPoint kp0 = prevFrame.keypoints[match.queryIdx];
        cv::KeyPoint kp1 = currFrame.keypoints[match.trainIdx];

        int prev_box_id = prevFrame.maskImg.at<uint8_t>((int)kp0.pt.y, (int)kp0.pt.x);
        int curr_box_id = currFrame.maskImg.at<uint8_t>((int)kp1.pt.y, (int)kp1.pt.x);

        if(prev_box_id != 255 && curr_box_id != 255 && currFrame.boundingBoxes[curr_box_id].lidarPoints.size() > 0)
        {
            freq[prev_box_id][curr_box_id]++;
            // store DMatch
            currFrame.boundingBoxes[curr_box_id].kptMatches.push_back(match);
        }
    }
    
    for(auto it : freq)
    {
        int prev_box_id = it.first;
        // for each previous box_id, find the best match of current box_id
        int max_freq = 0;
        int best_id = -1;
        for(auto it2 : it.second)
        {
            if(it2.second > max_freq)
            {
                max_freq = it2.second;
                best_id = it2.first;
            }
        }
        if(best_id !=-1)
        {
            bbBestMatches[prev_box_id] = best_id;
        }
    }
}


