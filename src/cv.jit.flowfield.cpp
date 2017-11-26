/* 
	cv.jit.flowfield.c
	
	Finds and outputs the position of strong flowfield (corners) in the input image.
	Useful for initializing tracking algorithms.
	
	Copyright (c) 2008-2017, Jean-Marc Pelletier
	jmp@jmpelletier.com

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "jit.common.h"
#include "cv.h"

#define MAXPOINTS 256

void cvJitter2CvMat(void *jit, CvMat *cv)
{
	t_jit_matrix_info info;
	unsigned char *bp;
	int type;

	if(jit && cv)
	{
		jit_object_method(jit,_jit_sym_getinfo,&info);	
		jit_object_method(jit,_jit_sym_getdata,&bp);

		if(info.type == _jit_sym_char)
		{
			type = CV_MAKETYPE(CV_8U,info.planecount);
		}
		else if(info.type == _jit_sym_long)
		{
			type = CV_MAKETYPE(CV_32S,info.planecount);
		}
		else if(info.type == _jit_sym_float32)
		{
			type = CV_MAKETYPE(CV_32F,info.planecount);
		}
		else if(info.type == _jit_sym_float64)
		{
			type = CV_MAKETYPE(CV_64F,info.planecount);
		}

		cvInitMatHeader( cv, info.dim[1], info.dim[0], type, bp, info.dimstride[1] );
	}
}

typedef struct _cv_jit_flowfield 
{
	t_object				ob;

	//Parameters
	float			threshold;
	float			distance;

	long			npoints;
	long			radius;
	long			motionthresh;
	long			mode;

	//Images for processing	
	CvMat		*previous;
	CvMat		*movement;
	CvMat		*mask;
	CvMat		*eigImage;
	CvMat		*tmpImage;
	CvMat		*pyr;
	CvMat		*prevPyr;
	CvMat		*dummy;

	//Arrays for tracking
	CvPoint2D32f	points[MAXPOINTS]; 
	CvPoint2D32f	newPoints[MAXPOINTS];
	char			status[MAXPOINTS];

	int			pointCount;

	int			flags;

} t_cv_jit_flowfield;

void *_cv_jit_flowfield_class;

t_jit_err 				cv_jit_flowfield_init(void); 
t_cv_jit_flowfield *	cv_jit_flowfield_new(void);
void 					cv_jit_flowfield_free(t_cv_jit_flowfield *x);
t_jit_err 				cv_jit_flowfield_matrix_calc(t_cv_jit_flowfield *x, void *inputs, void *outputs);

t_jit_err cv_jit_flowfield_init(void) 
{
	long attrflags=0;
	t_jit_object *attr,*mop,*output;
	t_symbol *atsym;
	t_jit_matrix_info out_minfo; 
	
	atsym = gensym("jit_attr_offset");
	
	_cv_jit_flowfield_class = jit_class_new("cv_jit_flowfield",(method)cv_jit_flowfield_new,(method)cv_jit_flowfield_free,
		sizeof(t_cv_jit_flowfield),A_CANT,0L); //A_CANT = untyped

	//add mop
	mop = jit_object_new(_jit_sym_jit_mop,1,1);  //Object has one input and one output
	output = jit_object_method(mop,_jit_sym_getoutput,1); //Get a pointer to the output matrix

   	jit_mop_single_type(mop,_jit_sym_char);   //Set input type and planecount
   	jit_mop_single_planecount(mop,1);  
   	
   	jit_mop_output_nolink(mop,1); //Turn off output linking so that output matrix does not adapt to input

   	jit_attr_setlong(output,_jit_sym_minplanecount,4);  //Four planes defining one motion vector 
  	jit_attr_setlong(output,_jit_sym_maxplanecount,4);
  	jit_attr_setlong(output,_jit_sym_mindim,1); //Only one dimension
  	jit_attr_setlong(output,_jit_sym_maxdim,1);
  	jit_attr_setsym(output,_jit_sym_types,_jit_sym_float32); //Coordinates are returned with sub-pixel accuracy
   	   	
	jit_class_addadornment(_cv_jit_flowfield_class,mop);
	
	
	//add methods
	jit_class_addmethod(_cv_jit_flowfield_class, (method)cv_jit_flowfield_matrix_calc, 		"matrix_calc", 		A_CANT, 0L);	

	//add attributes	
	attrflags = JIT_ATTR_GET_DEFER_LOW | JIT_ATTR_SET_USURP_LOW;
	
	//threshold for feature detection
	attr = jit_object_new(	_jit_sym_jit_attr_offset,"threshold",_jit_sym_float32,attrflags,(method)0L,(method)0L,calcoffset(t_cv_jit_flowfield,threshold));			
	jit_attr_addfilterset_clip(attr,0.001,1,TRUE,TRUE);	//clip to 0.001-1
	jit_class_addattr(_cv_jit_flowfield_class, attr);
	
	//minimum distance between two features
	attr = jit_object_new(	_jit_sym_jit_attr_offset,"distance",_jit_sym_float32,attrflags,(method)0L,(method)0L,calcoffset(t_cv_jit_flowfield,distance));			
	jit_class_addattr(_cv_jit_flowfield_class, attr);

	//Maximum number of features
	attr = jit_object_new(	_jit_sym_jit_attr_offset,"npoints",_jit_sym_long,attrflags,(method)0L,(method)0L,calcoffset(t_cv_jit_flowfield,npoints));			
	jit_attr_addfilterset_clip(attr,1,MAXPOINTS,TRUE,TRUE);	//clip to 1 - MAXPOINTS
	jit_class_addattr(_cv_jit_flowfield_class, attr);

	//Radius of optical flow window
	attr = jit_object_new(	_jit_sym_jit_attr_offset,"radius",_jit_sym_long,attrflags,(method)0L,(method)0L,calcoffset(t_cv_jit_flowfield,radius));			
	jit_attr_addfilterset_clip(attr,1,10,TRUE,TRUE);
	jit_class_addattr(_cv_jit_flowfield_class, attr);

	//Threshold for motion detection
	attr = jit_object_new(	_jit_sym_jit_attr_offset,"motionthresh",_jit_sym_long,attrflags,(method)0L,(method)0L,calcoffset(t_cv_jit_flowfield,motionthresh));			
	jit_attr_addfilterset_clip(attr,0,255,TRUE,TRUE); //clip to 0 - 255
	jit_class_addattr(_cv_jit_flowfield_class, attr);

	//Threshold for motion detection
	attr = jit_object_new(	_jit_sym_jit_attr_offset,"mode",_jit_sym_long,attrflags,(method)0L,(method)0L,calcoffset(t_cv_jit_flowfield,mode));			
	jit_attr_addfilterset_clip(attr,0,1,TRUE,TRUE); //clip to 0 - 1
	jit_class_addattr(_cv_jit_flowfield_class, attr);
	
	
	jit_class_register(_cv_jit_flowfield_class);

	return JIT_ERR_NONE;
}

t_jit_err cv_jit_flowfield_matrix_calc(t_cv_jit_flowfield *x, void *inputs, void *outputs)
{
	t_jit_err				err=JIT_ERR_NONE;
	long					in_savelock,out_savelock;
	t_jit_matrix_info		in_minfo,out_minfo;
	char					*out_bp, *in_bp;
	void					*in_matrix,*out_matrix;
	int						i,j,k;
	float					dx,dy;
	float					t,td,tr;
	float					*out_data;
	CvMat					source;
	int						featureCount;
	CvSize					window;

	int		nClusters = 0;
	
	//Get pointers to matrices
	in_matrix 	= jit_object_method(inputs,_jit_sym_getindex,0);
	out_matrix  = jit_object_method(outputs,_jit_sym_getindex,0);

	if (x&&in_matrix&&out_matrix) 
	{
		//Lock the matrices
		in_savelock = (long) jit_object_method(in_matrix,_jit_sym_lock,1);
		out_savelock = (long) jit_object_method(out_matrix,_jit_sym_lock,1);
		
		//Make sure input is of proper format
		jit_object_method(in_matrix,_jit_sym_getinfo,&in_minfo);
		jit_object_method(out_matrix,_jit_sym_getinfo,&out_minfo);

		if(in_minfo.dimcount != 2)
		{
			err = JIT_ERR_MISMATCH_DIM;
			goto out;
		}
		if(in_minfo.planecount != 1)
		{
			err = JIT_ERR_MISMATCH_PLANE;
			goto out;
		}
		if(in_minfo.type != _jit_sym_char)
		{
			err = JIT_ERR_MISMATCH_TYPE;
			goto out;
		}
		
		//Don't process if one dimension is < 2
		if((in_minfo.dim[0] < 2)||(in_minfo.dim[1] < 2))
			goto out;
		
		//Convert Jitter matrix to OpenCV matrix
		cvJitter2CvMat(in_matrix, &source);
		
		//Adjust the size of eigImage and tempImage if need be
		if((source.cols != x->eigImage->cols)||(source.rows != x->eigImage->rows))
		{
			cvReleaseMat(&(x->previous));
			x->previous = cvCreateMat( source.rows, source.cols, CV_8UC1 );
			cvSet(x->previous,cvScalarAll(0),NULL);

			cvReleaseMat(&(x->movement));
			x->movement = cvCreateMat( source.rows, source.cols, CV_8UC1 );

			cvReleaseMat(&(x->mask));
			x->mask = cvCreateMat( source.rows, source.cols, CV_8UC1 );

			cvReleaseMat(&(x->eigImage));
			x->eigImage = cvCreateMat( source.rows, source.cols, CV_32FC1 );

			cvReleaseMat(&(x->tmpImage));
			x->tmpImage = cvCreateMat( source.rows, source.cols, CV_32FC1 );

			x->pyr = cvCreateMat( source.rows, source.cols, CV_8UC1 );

			cvReleaseMat(&(x->prevPyr));
			x->prevPyr = cvCreateMat( source.rows, source.cols, CV_8UC1 );

			x->flags = 0;
		}
		
		//Adjust parameters
		x->threshold = MAX(0.001,x->threshold);
		x->distance = MAX(1,x->distance);
		featureCount = x->npoints;
		window.height = window.width = x->radius * 2 + 1;
		
		//Calculate
		//
		//Frame Differencing
		cvAbsDiff(&source,x->previous,x->movement);
		//
		//Threshold to obtain binary mask
		cvThreshold(x->movement,x->mask,x->motionthresh,255,CV_THRESH_BINARY);
		//
		
		//

		if(x->mode == 1)  //Use features from previous pass
		{
			CvPoint2D32f tempPoints[MAXPOINTS];

			//Find strong features only in areas where movement was detected
			cvGoodFeaturesToTrack( &source, x->eigImage, x->tmpImage, tempPoints, &featureCount, x->threshold, x->distance, x->mask, 3, 0, 0.04 );

			for(i=0,j=0;i<x->pointCount;i++)
			{
				if(x->status[i] == 1)
				{
					x->points[i] = x->newPoints[i];
				}
				else
				{
					if(j<featureCount)
					{
						x->points[i] = tempPoints[j];
						j++;
					}
				}
			}
			for(;(j<featureCount)&&(i<x->npoints);j++,i++)
			{
				x->points[i] = tempPoints[j];
			}

			featureCount = i;
		}
		else
		{
			//Find strong features only in areas where movement was detected
			cvGoodFeaturesToTrack( &source, x->eigImage, x->tmpImage, x->points, &featureCount, x->threshold, x->distance, x->mask, 3, 0, 0.04 );
		}

		//Find optical flow for detected features
		if(featureCount > 0) //Don't process if there are no features
		{
			cvCalcOpticalFlowPyrLK( x->previous, &source, x->prevPyr, x->pyr,
                x->points, x->newPoints, featureCount, window, 3, x->status, 0,
                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), x->flags );
                
			x->flags |= CV_LKFLOW_PYR_A_READY;
			CV_SWAP(x->prevPyr,x->pyr,x->dummy);
		}
		
		//Copy current frame for next pass
		cvCopy(&source, x->previous, 0);


		//Prepare output
		//Change dimensions of output matrix to match number of features
		out_minfo.dim[0] = featureCount;
		jit_object_method(out_matrix,_jit_sym_setinfo,&out_minfo);
		jit_object_method(out_matrix,_jit_sym_getinfo,&out_minfo);
		jit_object_method(out_matrix,_jit_sym_getdata,&out_bp);
		if (!out_bp) { err=JIT_ERR_INVALID_OUTPUT; goto out;}
		
		out_data = (float *)out_bp;
		
		for(i=0; i < featureCount; i++)
		{
			out_data[0] = x->points[i].x;
			out_data[1] = x->points[i].y;
			out_data[2] = x->newPoints[i].x;
			out_data[3] = x->newPoints[i].y;
			
			out_data += 4;
		}
		
	}

	
out:
	jit_object_method(out_matrix,gensym("lock"),out_savelock);
	jit_object_method(in_matrix,gensym("lock"),in_savelock);
	return err;
}



t_cv_jit_flowfield *cv_jit_flowfield_new(void)
{
	t_cv_jit_flowfield *x;
		
	if (x=(t_cv_jit_flowfield *)jit_object_alloc(_cv_jit_flowfield_class)) {
	
		x->threshold = 0.1;
		x->distance = 5;

		x->npoints = 128;
		x->radius = 5;

		x->motionthresh = 3;

		x->mode = 0;

		x->pointCount = 0;
		
		//Initialize matrices
		x->previous = cvCreateMat( 1, 1, CV_8UC1 );
		x->movement = cvCreateMat( 1, 1, CV_8UC1 );
		x->mask = cvCreateMat( 1, 1, CV_8UC1 );
		x->eigImage = cvCreateMat( 1, 1, CV_32FC1 );
		x->tmpImage = cvCreateMat( 1, 1, CV_32FC1 );
		x->pyr = cvCreateMat( 1, 1, CV_8UC1 );
		x->prevPyr = cvCreateMat( 1, 1, CV_8UC1 );

		x->flags = 0;

	} else {
		x = NULL;
	}	
	return x;
}

void cv_jit_flowfield_free(t_cv_jit_flowfield *x)
{
	if(x->previous)
		cvReleaseMat(&(x->previous));
	if(x->movement)
		cvReleaseMat(&(x->movement));
	if(x->mask)
		cvReleaseMat(&(x->mask));
	if(x->eigImage)
		cvReleaseMat(&(x->eigImage));
	if(x->tmpImage)
		cvReleaseMat(&(x->tmpImage));
	if(x->pyr)
		cvReleaseMat(&(x->pyr));
	if(x->prevPyr)
		cvReleaseMat(&(x->prevPyr));
}

