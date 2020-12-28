#include <numeric>
#include "matching2D.hpp"

using namespace std;
using namespace cv;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, 
                      std::vector<cv::KeyPoint> &kPtsRef, 
                      cv::Mat &descSource, 
                      cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, 
                      std::string descriptorType, 
                      std::string matcherType, 
                      std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    int normType = cv::NORM_L2;

    if(descriptorType.compare("ORB") == 0 ||
       descriptorType.compare("BRIEF") == 0 ||
        descriptorType.compare("BRISK") == 0)
    {
        normType = cv::NORM_HAMMING;
    }

    if (matcherType.compare("MAT_BF") == 0)
    {
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
       matcher = cv::DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    {   // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    {   // k nearest neighbors (k=2)
        std::vector< std::vector<DMatch> > knn_matches;
        matcher->knnMatch( descSource, descRef, knn_matches, 2 );
        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.8f;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
            }
        }
    }
}

// BRIEF, ORB, FREAK, AKAZE, SIFT
// Use one of several types of state-of-art descriptors to uniquely identify keypoints
float descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    double t = (double)cv::getTickCount();

    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        int nfeatures = 180;
        float scaleFactor = 2.0f;
        int nlevels = 4, edgeThreshold = 31, firstLevel = 0, WTA_K = 2;
        extractor = ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold);
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        bool 	orientationNormalized = true;
        bool 	scaleNormalized = true;
        float 	patternScale = 22.0f;
        int 	nOctaves = 4;
        extractor = cv::xfeatures2d::FREAK::create(orientationNormalized, 
                                                   scaleNormalized, 
                                                   patternScale,
                                                   nOctaves);
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        int descriptor_type = AKAZE::DESCRIPTOR_MLDB;
        int descriptor_size = 0;
        int descriptor_channels = 3;
        float threshold = 0.001f;
        int nOctaves = 4;

        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        int nfeatures=0, nOctaveLayers=3;
        double contrastThreshold=0.04, edgeThreshold=10, sigma=1.6;
        extractor = cv::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
         extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(32);
    }
    else
    {
        std::cout<< "unknown descriptor type: " << descriptorType << "\n";
        return 0;
    }
    // perform feature description


    extractor->compute(img, keypoints, descriptors);
    double delta = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    double delta_ms = 1000 * delta / 1.0;
    cout << descriptorType << " descriptor extraction in " << delta_ms << " ms" << endl;
    return delta_ms;
}



// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
}

void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img)
{
    // compute detector parameters based on image size
    int blockSize = 2;     
    double k = 0.04;
    int apertureSize = 3;

    cv::Mat dst = cv::Mat::zeros( img.size(), CV_32FC1 );
    cv::cornerHarris(img, dst, blockSize, apertureSize, k );

    int thresh = 150;
    Mat dst_norm, dst_norm_scaled;
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    
    // add corners to result vector
    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > thresh )
            {
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(j, i);
                newKeyPoint.size = blockSize;
                keypoints.push_back(newKeyPoint);
            }
        }
    }
}

float detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, 
                        cv::Mat &img, const std::string &detectorType, bool bVis)
{
    keypoints.clear();
    double t = (double)cv::getTickCount();
    if (detectorType.compare("SHITOMASI") == 0) {
        detKeypointsShiTomasi(keypoints, img);
    }
    else if (detectorType.compare("HARRIS") == 0) {
        detKeypointsHarris(keypoints, img);
    }
    else if (detectorType.compare("FAST") == 0) {
        cv:FAST(img, keypoints, 10);
    }
    else if (detectorType.compare("BRISK") == 0) {
        cv::Ptr<cv::FeatureDetector> detector = BRISK::create();
        detector->detect(img, keypoints);
    } 
    else if (detectorType.compare("ORB") == 0) {
        cv::Ptr<cv::FeatureDetector> detector = ORB::create();
        detector->detect(img, keypoints);
    } 
    else if (detectorType.compare("AKAZE") == 0) {
        cv::Ptr<cv::FeatureDetector> detector = AKAZE::create();
        detector->detect(img, keypoints);
    } 
    else if (detectorType.compare("SIFT") == 0) {
       cv::Ptr<cv::FeatureDetector> detector = SIFT::create();
        detector->detect(img, keypoints);
        for(auto &p : keypoints) 
        {
            p.octave = 0;     
        }
    } 
    else {
        cout << "unrecognized detector type\n";
        return -1;
    }

    for(auto &p : keypoints) 
    {
        p.class_id = 0;
    }

    float delta = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    float delta_ms = 1000 * delta / 1.0;

    cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " <<  delta_ms << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return delta_ms;
}


