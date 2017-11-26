#include "OpticalFlowTracker.h"


/*******************************Constructor/Destructor*********************************/
OpticalFlowTracker::OpticalFlowTracker(){
	currentImage = 0;
	previousImage = 0;
	currentPyramid = 0;
	previousPyramid = 0;
	vectors = 0;
	features = 0;
	newPositions = 0;
	dummyPoint = cvPoint2D32f(0.f,0.f);
	status = 0;
	indices = 0;
	dummyChar = 0;
	featureCount = 0;
	windowSize = cvSize(10,10);
	pyramidLevels = 3;
	flags = 0;
	minDistance = 0.01;
	vectorCount = 0;
	goodVectorCount = 0;
	maxAge = 3;
	maxFriends = 0;
	dummyVector.x = -1000.f;
	dummyVector.y = -1000.f;
	dummyVector.x2 = -1000.f;
	dummyVector.y2 = -1000.f;
	dummyVector.alpha = -1000.f;
	dummyVector.theta = -1000.f;
	dummyVector.friends = 0;
	dummyVector.age = 0;
	
	featureDetector.setMinDistance(minDistance);
	featureDetector.setThreshold(0.01);
}

OpticalFlowTracker::~OpticalFlowTracker(){
	if(previousImage)cvReleaseMat(&previousImage);
	if(currentPyramid)cvReleaseMat(&currentPyramid);
	if(previousPyramid)cvReleaseMat(&previousPyramid);
	
	free(status);
	free(newPositions);
	free(indices);
}


/*******************************Private methods*********************************/

char OpticalFlowTracker::rebuildImages(){
	if(previousImage)cvReleaseMat(&previousImage);
		if(currentPyramid)cvReleaseMat(&currentPyramid);
		if(previousPyramid)cvReleaseMat(&previousPyramid);
		flags = 0;
		previousImage = cvCreateMat(currentImage->rows, currentImage->cols, currentImage->type);
		currentPyramid = cvCreateMat(currentImage->rows, currentImage->cols, currentImage->type);
		previousPyramid = cvCreateMat(currentImage->rows, currentImage->cols, currentImage->type);
		if((!previousImage)||(!currentPyramid)||(!previousPyramid)){
			if(previousImage)cvReleaseMat(&previousImage);
			if(currentPyramid)cvReleaseMat(&currentPyramid);
			if(previousPyramid)cvReleaseMat(&previousPyramid);
			strcpy(error,"OpticalFlowTracker::rebuildImages failed");
			return 0;
		}
		return 1;
}

char OpticalFlowTracker::checkImages(){
	if(!currentImage){strcpy(error,"OpticalFlowTracker::checkImages failed");return 0;}
	if((!previousImage)||(!currentPyramid)||(!previousPyramid))return rebuildImages();
	if(!CV_ARE_SIZES_EQ(previousImage, currentImage))return rebuildImages();
	return 1;
}

/*******************************Public methods*********************************/
		
char OpticalFlowTracker::storePreviousImage(){
	if(!checkImages())return 0;
	CvMat* tmp;
	cvCopy(currentImage, previousImage, 0);
	CV_SWAP(currentPyramid, previousPyramid, tmp);
	return 1;
}

char OpticalFlowTracker::setImage(CvMat *image){
	if(!image){strcpy(error,"OpticalFlowTracker::setImage failed");return 0;}
	currentImage = image;
	return checkImages();
}

char OpticalFlowTracker::trackFeatures(){
	if((!currentImage)||(!previousImage)||(!currentPyramid)||(!previousPyramid)){
		strcpy(error,"OpticalFlowTracker::trackFeatures failed");
		return 0;
	}
	if(featureCount < 1)return 1;
	if((!features)||(!newPositions)||(!status)){
		strcpy(error,"OpticalFlowTracker::trackFeatures failed");
		return 0;
	}
	
	cvCalcOpticalFlowPyrLK(previousImage, currentImage, previousPyramid, currentPyramid,
					 features, newPositions, featureCount, windowSize, pyramidLevels, status,
					 0 , cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags);
	flags |= CV_LKFLOW_PYR_A_READY;
	
	return 1;
}

char OpticalFlowTracker::processFrame(CvMat *image){
	if(!setImage(image))return 0;
	if(!featureDetector.findFeatures(previousImage)){strcpy(error, featureDetector.getErrorMess()); return 0;}
	if(!updateFeatureList())return 0;
	if(!trackFeatures())return 0;
	if(!calculateVectors())return 0;
	if(!findFriends())return 0;
	return storePreviousImage();
}

char OpticalFlowTracker::updateFeatureList(){
	unsigned int totalCount = featureCount+featureDetector.getCount();
	if(totalCount < 1)return 1;
	CvPoint2D32f* tempFeatures = (CvPoint2D32f*)malloc(sizeof(CvPoint2D32f)*totalCount);
	if(!tempFeatures){strcpy(error,"OpticalFlowTracker::updateFeatureList failed: tempFeatures"); return 0;}
	unsigned int* tempIndices = (unsigned int*)malloc(sizeof(unsigned int)*totalCount);
	if(!tempFeatures){strcpy(error,"OpticalFlowTracker::updateFeatureList failed: tempIndices"); return 0;}
	unsigned int* tempAges = (unsigned int*)malloc(sizeof(unsigned int)*totalCount);
	if(!tempAges){strcpy(error,"OpticalFlowTracker::updateFeatureList failed: tempAges"); return 0;}
	
	CvPoint2D32f* f = featureDetector.getFeaturePtr();
	unsigned int c = featureDetector.getCount();
	
	//Merge into new list features that were tracked successfully
	unsigned int i,j, index=0;
	float dx, dy;
	float d_thresh = minDistance*(float)currentImage->cols; d_thresh*=(d_thresh*1.5);
	bool isolated;
	for(i=0;i<featureCount;i++){
		if(status[i]){
			//check against other features, implement minimal distance
			isolated = true;
			for(j=0;j<featureCount;j++){
				if(i==j)continue;
				dx=newPositions[j].x - newPositions[i].x; dx*=dx;
				dy=newPositions[j].y - newPositions[i].y; dy*=dy;
				if((dx+dy)<d_thresh){
					isolated = false;
					break;
				}
			}
			if(isolated){
				tempFeatures[index] = newPositions[i];
				tempIndices[index] = indices[i];
				tempAges[index] = tempAges[index] < maxAge ? ages[i]+1 : maxAge;
				index++;
			}
			else{
				indexManager.removeIndex(indices[i]);
			}
		}
		else{
			indexManager.removeIndex(indices[i]);
		}
	}
		
	unsigned int newcount = index;
	for(i=0;i<c;i++){
		isolated = true;
		for(j=0;j<newcount;j++){
			dx=tempFeatures[j].x - f[i].x; dx*=dx;
			dy=tempFeatures[j].y - f[i].y; dy*=dy;
			if((dx+dy)<d_thresh){
				isolated = false;
				break;
			}
		}
		if(isolated){
			tempFeatures[index] = f[i];
			tempIndices[index] = indexManager.getIndex();
			tempAges[index] = 0;
			index++;
		}
	}
	
	free(features);
	free(indices);
	free(ages);
	
	features = tempFeatures;
	indices = tempIndices;
	ages = tempAges;
	
	featureCount = index;
	
	status = (char*)realloc(status, sizeof(char)*featureCount);
	if(!status){strcpy(error,"OpticalFlowTracker::updateFeatureList failed: status"); return 0;};
	
	newPositions = (CvPoint2D32f*)realloc(newPositions, sizeof(CvPoint2D32f)*featureCount);
	if(!newPositions){strcpy(error,"OpticalFlowTracker::updateFeatureList failed: newPositions"); return 0;}
	
	return 1;
}

char OpticalFlowTracker::calculateVectors(){
	vectorCount = 0;
	if(featureCount < 1)return 1;
	if((!features)||(!newPositions)||(!status)||(!ages)||(!indices))
		{strcpy(error,"OpticalFlowTracker::calculateVectors failed"); return 0;}
	if(!currentImage){strcpy(error,"OpticalFlowTracker::calculateVectors failed: currentImage"); return 0;}
	if(featureCount < 1)return 1;
	vectors = (Vector*)realloc(vectors,featureCount*sizeof(Vector));
	if(!vectors){strcpy(error,"OpticalFlowTracker::calculateVectors failed: vectors"); return 0;}
	
	unsigned int i,j;
	float dx, dy;
	float scale_x = 1.f / currentImage->cols;
	float scale_y = 1.f / currentImage->rows;
	
	for(i=0, j=0;i<featureCount;i++){
		if(!status[i])continue;
		vectors[j].x = features[i].x * scale_x;
		vectors[j].y = features[i].y * scale_y;
		vectors[j].x2 = newPositions[i].x * scale_x;
		vectors[j].y2 = newPositions[i].y * scale_y;
		dx = vectors[j].x - vectors[j].x2;
		dy = vectors[j].y - vectors[j].y2;
		vectors[j].alpha = cvSqrt(dx*dx+dy*dy);
		vectors[j].theta = cvFastArctan(dy,dx);
		vectors[j].friends = 0;
		vectors[j].age = ages[i];
		vectors[j].index = indices[i];
		j++;
	}
	
	vectorCount = j;
	
	return 1;
}


char OpticalFlowTracker::findFriends(){
	if(featureCount<1)return 1;
	if(!vectors){strcpy(error,"OpticalFlowTracker::findFriends failed: vectors"); return 0;}
	if(!currentImage){strcpy(error,"OpticalFlowTracker::findFriends failed: currentImage"); return 0;}
	
	const float d_thresh = 0.03f; //Arbitrary distance threshold. = (1/8)^2 + (1/8)^2
	//float small_movement = (float)(currentImage.cols + currentImage.rows) / 280.f;
	const float small_movement = 0.007; //Again arbitrary value (corresponds to ~2 pixels for 320x240 image)
	maxFriends = cvFloor((float)vectorCount * 0.015625f);
	
	goodVectorCount = 0;
	
	unsigned int i,j;
	float dx,dy;
	for(i=0;i<vectorCount;i++){
		if(vectors[i].friends > maxFriends) continue;
		
		if(vectors[i].alpha < small_movement){
			for(j=0;j<vectorCount;j++){
				if(i==j)continue;
				dx = vectors[i].x - vectors[j].x; dx*=dx;
				dy = vectors[i].y - vectors[j].y; dy*=dy;
				if((dx+dy)>d_thresh)continue;
				if(vectors[j].alpha < small_movement){
					vectors[i].friends++;
					vectors[j].friends++;
					if(vectors[i].friends > maxFriends){
						//if(vectors[i].age == maxAge)goodVectorCount++;
						break;
					}
				}
			}
		}
		else{
			for(j=0;j<vectorCount;j++){
				if(i==j)continue;
				dx = vectors[i].x - vectors[j].x; dx*=dx;
				dy = vectors[i].y - vectors[j].y; dy*=dy;
				if((dx+dy)>d_thresh)continue;
				dx = vectors[i].theta - vectors[j].theta; 
				if((dx < -337.5f)||((dx < 22.5f)&&(dx > -22.5f))||(dx > 337.5f)){
					dy = vectors[i].alpha / vectors[j].alpha;
					if((dy>0.75)&&(dy<1.25)){
						vectors[i].friends++;
						vectors[j].friends++;
						if(vectors[i].friends > maxFriends){
							//if(vectors[i].age == maxAge)goodVectorCount++;
							break;
						}
					}
				}
			}
		}
	}
	
	for(i=0;i<vectorCount;i++)if((vectors[i].age == maxAge)&&(vectors[i].friends > maxFriends))goodVectorCount++;
	
	return 1;
}


void OpticalFlowTracker::reset(){
	cvReleaseMat(&previousImage);
	cvReleaseMat(&currentPyramid);
	cvReleaseMat(&previousPyramid);
	free(newPositions); newPositions = 0;
	free(vectors); vectors = 0;
	free(status); status = 0;
	free(indices); indices = 0;
	free(ages); ages = 0;
	indexManager.reset();
	featureCount = 0;
	vectorCount = 0;
	goodVectorCount = 0;
	flags = 0;
}


IndexManager::IndexManager(){
	indexStack.reserve(1024);
	indexStack.push_back(1);
	
}

unsigned int IndexManager::getIndex()
{
	if(indexStack.capacity()<1024){
		indexStack.reserve(1024);
		indexStack.push_back(1);
	}
	unsigned int ndx = indexStack.back();
	indexStack.pop_back();
	if(indexStack.empty())indexStack.push_back(ndx+1);
	return ndx;
}

void IndexManager::removeIndex(unsigned int ndx){
	indexStack.push_back(ndx);
}

void IndexManager::reset(){
	indexStack.clear();
	indexStack.push_back(1);
}







