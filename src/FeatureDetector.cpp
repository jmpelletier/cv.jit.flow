#include "FeatureDetector.h"

char FeatureDetector::adjustTempImagesSize(CvMat* image){
	if(!tempImage){
		tempImage = cvCreateMat(image->rows, image->cols, CV_32FC1);
		if(!tempImage){strcpy(error,"OpticalFlowTracker::adjustTempImagesSize failed: tempImage"); return 0;}
	}
	if(!eigImage){
		eigImage = cvCreateMat(image->rows, image->cols, CV_32FC1);
		if(!eigImage){strcpy(error,"OpticalFlowTracker::adjustTempImagesSize failed: eigImage"); return 0;}
	}
	if(!CV_ARE_SIZES_EQ(tempImage, image)){
		cvReleaseMat(&tempImage);
		tempImage = cvCreateMat(image->rows, image->cols, CV_32FC1);
		if(!tempImage){strcpy(error,"OpticalFlowTracker::adjustTempImagesSize failed: tempImage 02"); return 0;}
	}
	if(!CV_ARE_SIZES_EQ(eigImage, image)){
		cvReleaseMat(&eigImage);
		eigImage = cvCreateMat(image->rows, image->cols, CV_32FC1);
		if(!eigImage){strcpy(error,"OpticalFlowTracker::adjustTempImagesSize failed: eigImage 02"); return 0;}
	}
	
	return 1;
}

char FeatureDetector::findFeatures(CvMat* image){
	//Save previous features
	CvPoint2D32f *temp;
	temp = features;
	features = previousFeatures;
	previousFeatures = temp;
	previousCount = count;
	switch(algorithm){
		case(FEATURE_ALGO_FAST):
			return findFeaturesFAST(image);
		default:
			return findFeaturesEigVals(image);
	}
}

char FeatureDetector::findFeaturesEigVals(CvMat* image){
	if(!adjustTempImagesSize(image))return 0;
	features = (CvPoint2D32f*)realloc(features, 2048*sizeof(CvPoint2D32f));
	if(!features){strcpy(error,"OpticalFlowTracker::findFeaturesEigVals failed"); return 0;}
	int fcount = 2048;
	cvGoodFeaturesToTrack(image, eigImage, tempImage, features, &fcount,
					threshold, minDistance*(float)image->cols, 0, 3, 0, 0.04);
	count = (unsigned int)fcount;
	return 1;
}
		
char FeatureDetector::findFeaturesFAST(CvMat* image){
	return 1;
}