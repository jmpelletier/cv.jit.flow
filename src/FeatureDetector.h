#ifndef _FEATUREDETECTOR_H
#define _FEATUREDETECTOR_H

#include "cv.h"

#define FEATURE_ALGO_EIGENVALS 0
#define FEATURE_ALGO_FAST 1

#define MAX_EIG_FEATURE_COUNT 2048 

class FeatureDetector{
	private:
		CvPoint2D32f *features;
		CvPoint2D32f *previousFeatures;
		CvMat *tempImage;
		CvMat *eigImage;
		unsigned int count;
		unsigned int previousCount;
		int algorithm;
		float threshold;
		float minDistance;
		char error[256];
		
		char adjustTempImagesSize(CvMat* image);
		
		char findFeaturesEigVals(CvMat* image);
		char findFeaturesFAST(CvMat* image);
		
	public:
		FeatureDetector(){
			count = 0;
			tempImage = 0;
			eigImage = 0;
			algorithm = FEATURE_ALGO_EIGENVALS;
			minDistance = 0.01;
			threshold = 0.1;
			features = NULL;
			previousFeatures = NULL;
		}
		~FeatureDetector(){
			if(features)free(features);
			if(tempImage)cvReleaseMat(&tempImage);
			if(eigImage)cvReleaseMat(&eigImage);
		}
		
		void setMinDistance(float d){
			if(d < 0.f)minDistance = 0.f;
			else if(d > 1.f)minDistance = 1.f;
			else minDistance = d;		
		}
		float getMinDistance(){return minDistance;}
		
		void setThreshold(float t){
			if(t < 0.001f)threshold = 0.001f;
			else if(t > 1.f)threshold = 1.f;
			else threshold = t;		
		}
		float getThreshold(){return threshold;}
		
		void setAlgorithm(int a){algorithm = a;}
		int getAlgorithm(){return algorithm;}
		
		unsigned int getCount(){return count;}
		unsigned int getPreviousCount(){return previousCount;}
		CvPoint2D32f* getFeaturePtr(){return features;}
		CvPoint2D32f* getPreviousFeaturePtr(){return previousFeatures;}
		
		char findFeatures(CvMat* image);
		
		const char* getErrorMess(){return error;}
};

#endif