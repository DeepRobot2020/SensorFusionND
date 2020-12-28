# SFND 2D Feature Tracking Report

## MP.1 Data Buffer
This has been implemented

## MP.2 Keypoint Detection
All the required detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT have been implement

## MP.3 Keypoint Removal
All the keypoints not within the range of (535, 180, 180, 150) have been removed

## MP.4 Keypoint Descriptors
All the required descriptors BRIEF, ORB, FREAK, AKAZE and SIFT have been implement and 
can be select by a string. In the final version, the code will loop all the combinnations of 
required feature detector and descriptors to compare the performance

## MP.5 Descriptor Matching
Both FLANN matching as well as k-nearest neighbor selection have been implement. Both methods must be selectable using the respective strings in the main function.

## MP.6 Descriptor Distance Ratio
In the KNN matching, a ratio threshold 0.8 has been used to decide whether to keep an associated pair of keypoints.


## MP.7 Performance Evaluation 1
The number of keypoints on the preceding vehicle for all 10 images have been logged in mp7_num_keypoints.csv


## MP.8 Performance Evaluation 2
The number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors have been logged in mp8_num_matched_keyoints.csv

## MP.9 Performance Evaluation 3
The time it takes for keypoint detection and descriptor extraction for each frame have been logged inside mp9_time.csv. I also log the total time and average time and if use average time as the sorting key, we can see the top 3 detector and descriptor are as below. 
Top 1 FAST-BRIEF avg_time 2.62793
Top 2 FAST-ORB avg_time 4.59356
Top 3 ORB-ORB avg_time 9.42388










