#ifndef _OPTICALFLOWTRACKER_H_
#define _OPTICALFLOWTRACKER_H_

#include "FeatureDetector.h"

#include "cv.h"
//#include <deque>
#include <vector>

using namespace std;


/*Errors*/

typedef struct _vector
{
	float x;
	float y;
	float x2;
	float y2;
	float alpha;
	float theta;
	unsigned int friends;
	unsigned int age;
	unsigned int index;
}Vector;

class IndexManager{
	private:
		vector<unsigned int> indexStack;
	public:
		IndexManager();		
		~IndexManager(){;}
		
		unsigned int getIndex();		
		void removeIndex(unsigned int ndx);		
		void reset();
};

class OpticalFlowTracker{
	private:
		CvMat *currentImage;
		CvMat *previousImage;
		CvMat *currentPyramid;
		CvMat *previousPyramid;
		CvPoint2D32f *features;
		CvPoint2D32f *newPositions;
		CvPoint2D32f dummyPoint;
		Vector *vectors;
		Vector dummyVector;
		char *status;
		unsigned int *indices;
		unsigned int *ages;
		unsigned int maxAge;
		unsigned int maxFriends;
		IndexManager indexManager;
		FeatureDetector featureDetector;
		char dummyChar;
		unsigned int featureCount;
		unsigned int vectorCount;
		unsigned int goodVectorCount;
		CvSize windowSize;
		unsigned int pyramidLevels;
		int flags;
		float minDistance;
		char error[256];
		
		char rebuildImages();
		char checkImages();
		char updateFeatureList();
		char calculateVectors();
		char findFriends();
		
	protected:
	
	public:
		OpticalFlowTracker();
		
		~OpticalFlowTracker();
		
		void setPyramidLevels(unsigned int l){pyramidLevels = l;}
		unsigned int getPyramidLevels(){return pyramidLevels;}
		
		void setWindowSize(unsigned int s){windowSize = s > 0 ? cvSize(s,s) : cvSize(1,1);}
		unsigned int getWindowSize(){return windowSize.width;}
		
		void setFeatureDetector(int d){featureDetector.setAlgorithm(d);}
		int getFeatureDetector(){return featureDetector.getAlgorithm();}
		
		void setMinDistance(float d){minDistance = d < 0.f ? 0.f : (d > 1.f ? 1. : d); featureDetector.setMinDistance(d);}
		float getMinDistance(){return minDistance;}
		
		void setDetectorThreshold(float t){featureDetector.setThreshold(t);}
		float getDetectorThreshold(){return featureDetector.getThreshold();}
		
		void setMaxAge(unsigned int a){maxAge = a;}
		unsigned int getMaxAge(){return maxAge;}
		
		unsigned int getFeatureCount(){return featureCount;}
		
		unsigned int getVectorCount(){return vectorCount;}
		unsigned int getGoodVectorCount(){return goodVectorCount;}
		Vector* getVectorPtr(){return vectors;}
		Vector* vectorAt(unsigned int ndx){if(ndx<vectorCount)return vectors+ndx; else return &dummyVector;}
		bool isGoodVector(unsigned int ndx){return (ndx < vectorCount)&&(vectors[ndx].age == maxAge)&&(vectors[ndx].friends > maxFriends);}
		
		unsigned int getMaxFriends(){return maxFriends;}
		
		CvPoint2D32f* getFeature(unsigned int ndx){if(ndx < featureCount)return features+ndx; else return &dummyPoint;}
		CvPoint2D32f* getNewPosition(unsigned int ndx){if(ndx < featureCount)return newPositions+ndx; else return &dummyPoint;}
		char getStatus(unsigned int ndx){if(ndx < featureCount)return *(status+ndx); else return dummyChar;}
		
		CvPoint2D32f* getFeaturePtr(){return features;}
		CvPoint2D32f* getNewPositionPtr(){return newPositions;}
		
		CvMat* getCurrentImage(){return currentImage;}
		CvMat* getPreviousImage(){return previousImage;}
		
		const char* getErrorMess(){return error;}
		
		char storePreviousImage();
		char setImage(CvMat *image);
		char trackFeatures();
		char processFrame(CvMat *image);
		void reset();
		
};

#endif