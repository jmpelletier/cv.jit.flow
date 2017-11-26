/*
Copyright (c) 2008, Jean-Marc Pelletier
jmp@iamas.ac.jp

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

	+Redistributions of source code must retain the above copyright notice, 
	 this list of conditions and the following disclaimer.
	+Redistributions in binary form must reproduce the above copyright notice, 
	 this list of conditions and the following disclaimer in the documentation 
	 and/or other materials provided with the distribution.
	+Neither the name of Jean-Marc Pelletier nor the name of cv.jit may be 
	 used to endorse or promote products derived from this software without 
	 specific prior written permission.
	 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef __cplusplus
extern "C" {
#endif
#include "jit.common.h"
#ifdef __cplusplus 
} //extern "C"
#endif
#include "cv.h"
#include "jitOpenCV.h"
#include "OpticalFlowTracker.h"

typedef struct _cv_jit_flow 
{
	t_object			ob;
	double				threshold;
	float				min_distance;
	long				radius;
	
	OpticalFlowTracker		tracker;
} t_cv_jit_flow;

void *_cv_jit_flow_class;

t_jit_err 			cv_jit_flow_init(void); 
t_cv_jit_flow*		cv_jit_flow_new(void);
void 				cv_jit_flow_free(t_cv_jit_flow *x);
t_jit_err 			cv_jit_flow_matrix_calc(t_cv_jit_flow *x, void *inputs, void *outputs);
void				cv_jit_flow_calculate(t_cv_jit_flow *x, long dimcount, long *dim, long planecount, t_jit_matrix_info *in_minfo, uchar *bip);
void				cv_jit_flow_reset(t_cv_jit_flow *x);

t_jit_err cv_jit_flow_init(void) 
{
	long attrflags=0;
	t_jit_object *attr,*mop,*output;
	t_symbol *atsym;
	t_jit_err err=JIT_ERR_NONE;
		
	atsym = gensym("jit_attr_offset");
	
	_cv_jit_flow_class = jit_class_new((char *)"cv_jit_flow",(method)cv_jit_flow_new,(method)cv_jit_flow_free, sizeof(t_cv_jit_flow),0L); 

	//add mop
	mop = (t_jit_object *)jit_object_new(_jit_sym_jit_mop,1,1);  //Object has one input and one output
	output = (t_jit_object *)jit_object_method(mop,_jit_sym_getoutput,1); //Get a pointer to the output matrix

   	jit_mop_single_type(mop,_jit_sym_char);   //Set input type and planecount
   	jit_mop_single_planecount(mop,1);  
   	
   	jit_mop_output_nolink(mop,1); //Turn off output linking so that output matrix does not adapt to input
   	
   	jit_attr_setlong(output,_jit_sym_minplanecount,7);  //Seven planes, holding a motion vector
  	jit_attr_setlong(output,_jit_sym_maxplanecount,7);
  	jit_attr_setlong(output,_jit_sym_mindim,1); //Only one dimension
  	jit_attr_setlong(output,_jit_sym_maxdim,1);
  	jit_attr_setsym(output,_jit_sym_types,_jit_sym_float32); //Coordinates are returned with sub-pixel accuracy
   	   	
	jit_class_addadornment(_cv_jit_flow_class,mop);
	
	
	//add methods
	jit_class_addmethod(_cv_jit_flow_class, (method)cv_jit_flow_matrix_calc,(char *)"matrix_calc",A_CANT, 0L);
	jit_class_addmethod(_cv_jit_flow_class, (method)cv_jit_flow_reset,(char *)"reset",0L);	

	//add attributes	
	attrflags = JIT_ATTR_GET_DEFER_LOW | JIT_ATTR_SET_USURP_LOW;
	//threshold
	attr = (t_jit_object *)jit_object_new(	_jit_sym_jit_attr_offset,"threshold",_jit_sym_float64,attrflags,(method)0L,(method)0L,calcoffset(t_cv_jit_flow,threshold));			
	jit_attr_addfilterset_clip(attr,0,1,TRUE,TRUE);	//clip to 0-1
	jit_class_addattr(_cv_jit_flow_class, attr);
	//distance
	attr = (t_jit_object *)jit_object_new(	_jit_sym_jit_attr_offset,"distance",_jit_sym_float32,attrflags,(method)0L,(method)0L,calcoffset(t_cv_jit_flow,min_distance));			
	jit_attr_addfilterset_clip(attr,0,0,TRUE,FALSE);	//clip to 0
	jit_class_addattr(_cv_jit_flow_class, attr);
	//radius
	attr = (t_jit_object *)jit_object_new(	_jit_sym_jit_attr_offset,"radius",_jit_sym_long,attrflags,(method)0L,(method)0L,calcoffset(t_cv_jit_flow,radius));			
	jit_attr_addfilterset_clip(attr,1,0,TRUE,FALSE);	//Must be at least 1
	jit_class_addattr(_cv_jit_flow_class, attr);
			
	err=jit_class_register(_cv_jit_flow_class);

	return err;
}

void cv_jit_flow_reset(t_cv_jit_flow *x)
{
	x->tracker.reset();
}

t_jit_err cv_jit_flow_matrix_calc(t_cv_jit_flow *x, void *inputs, void *outputs)
{
	t_jit_err err=JIT_ERR_NONE;
	long in_savelock=0,out_savelock=0;
	t_jit_matrix_info in_minfo,out_minfo;
	uchar *out_bp, *in_bp;
	void *in_matrix,*out_matrix;
	unsigned int i;
	float *out_data;
	int result;
	//CvPoint2D32f* f;
	Vector* v;
	CvMat image;
			
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
			
		//get matrix data pointers
		jit_object_method(in_matrix,_jit_sym_getdata,&in_bp);
		
		if (!in_bp) { err=JIT_ERR_INVALID_INPUT; goto out;}
		
		image = jitMatrix2CvMat(in_matrix);
		
		x->tracker.setDetectorThreshold((float)x->threshold);
		x->tracker.setMinDistance(x->min_distance);
		x->tracker.setWindowSize(x->radius);
		x->tracker.setMaxAge(3);
		
		result = x->tracker.processFrame(&image);
		if(!result){
			error("Could not process frame: %s", x->tracker.getErrorMess());
			err=JIT_ERR_GENERIC;
			goto out;
		}
		
		out_minfo.dim[0] = x->tracker.getGoodVectorCount();
		jit_object_method(out_matrix,_jit_sym_setinfo,&out_minfo);
		jit_object_method(out_matrix,_jit_sym_getinfo,&out_minfo);
		jit_object_method(out_matrix,_jit_sym_getdata,&out_bp);
		if (!out_bp) { err=JIT_ERR_INVALID_OUTPUT; goto out;}
		
		out_data = (float *)out_bp;
		for(i=0;i<x->tracker.getVectorCount();i++){
			if(x->tracker.isGoodVector(i)){
				v=x->tracker.vectorAt(i);
				out_data[0] = v->x;
				out_data[1] = v->y;
				out_data[2] = v->x2;
				out_data[3] = v->y2;
				out_data[4] = v->alpha;
				out_data[5] = v->theta;
				out_data[6] = (float)v->index;
				
				out_data += 7;
			}
		}
				
		/*result = x->tracker.updateFeatureList();
		if (!result)post("Error: %s", x->tracker.getError());
		 result = x->tracker.trackFeatures(&image);
		if (!result)post("Error: %s", x->tracker.getError());
					
		out_minfo.dim[0] = x->tracker.getGoodVectorCount();
		//out_minfo.dim[0] = x->tracker.getTrackedFeatureCount(); //debug
		jit_object_method(out_matrix,_jit_sym_setinfo,&out_minfo);
		jit_object_method(out_matrix,_jit_sym_getinfo,&out_minfo);
		
		jit_object_method(out_matrix,_jit_sym_getdata,&out_bp);
		if (!out_bp) { err=JIT_ERR_INVALID_OUTPUT; goto out;}
		
		out_data = (float *)out_bp;
		*/
		
		/*for(i=0;i<x->tracker.getTrackedFeatureCount();i++){
			f=x->tracker.getFeature(i);
			out_data[0] = f->x;
			out_data[1] = f->y;
			f=x->tracker.getTrackedFeature(i);
			out_data[2] = f->x;
			out_data[3] = f->y;
			out_data[4] = 3;
			out_data[5] = 0.5;
			
			out_data += 6;
		}*/
				
		/*for(i=0,j=0;i<x->tracker.getTrackedFeatureCount()&&j<out_minfo.dim[0];i++){
			if(x->tracker.isVectorInlier(i)){
				v = x->tracker.getVector(i);
				out_data[0] = v->x;
				out_data[1] = v->y;
				out_data[2] = v->x2;
				out_data[3] = v->y2;
				out_data[4] = v->alpha;
				out_data[5] = v->theta;
				
				out_data += 6;
				j++;
			}
		}
		
		result = x->tracker.findFeatures(&image);
		if (!result)post("Error: %s", x->tracker.getError());
		*/
	}

	
out:
	jit_object_method(out_matrix,gensym("lock"),out_savelock);
	jit_object_method(in_matrix,gensym("lock"),in_savelock);
	return err;
}

void cv_jit_flow_calculate(t_cv_jit_flow *x, long dimcount, long *dim, long planecount, t_jit_matrix_info *in_minfo, uchar *bip)
{
	CvMat image;
	
	/*image.type = CV_8UC1;
	image.step = in_minfo->dimstride[1];
	image.data.ptr = bip;
	image.rows = dim[1];
	image.cols = dim[0];*/
	
	cvInitMatHeader(&image, dim[1], dim[0], CV_MAKETYPE(CV_8U,planecount), bip, in_minfo->dimstride[1]);
		
	
}

t_cv_jit_flow *cv_jit_flow_new(void)
{
	t_cv_jit_flow *x;
			
	if ((x=(t_cv_jit_flow *)jit_object_alloc(_cv_jit_flow_class))) {
	
		x->threshold = 0.01;
		x->radius = 7;
		x->min_distance = 0.01;
		//x->dims[0] = x->dims[1] = 0;
		//x->count = 0;
	} else {
		x = NULL;
	}	
	return x;
}

void cv_jit_flow_free(t_cv_jit_flow *x)
{
	
}