/*

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

#ifdef __cplusplus
extern "C" {
#endif
#undef error
#include "jit.common.h"
#ifdef __cplusplus 
} //extern "C"
#endif

#undef error
#include "opencv.hpp"
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
		
		in_savelock = reinterpret_cast<long>(jit_object_method(in_matrix,_jit_sym_lock,1));
		out_savelock = reinterpret_cast<long>(jit_object_method(out_matrix,_jit_sym_lock,1));
		
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
	}

	
out:
	jit_object_method(out_matrix,gensym("lock"),out_savelock);
	jit_object_method(in_matrix,gensym("lock"),in_savelock);
	return err;
}

void cv_jit_flow_calculate(t_cv_jit_flow *x, long dimcount, long *dim, long planecount, t_jit_matrix_info *in_minfo, uchar *bip)
{
	CvMat image;
	
	cvInitMatHeader(&image, dim[1], dim[0], CV_MAKETYPE(CV_8U,planecount), bip, in_minfo->dimstride[1]);
}

t_cv_jit_flow *cv_jit_flow_new(void)
{
	t_cv_jit_flow *x;
			
	if ((x=(t_cv_jit_flow *)jit_object_alloc(_cv_jit_flow_class))) {
	
		x->threshold = 0.01f;
		x->radius = 7;
		x->min_distance = 0.01f;
	} else {
		x = NULL;
	}	
	return x;
}

void cv_jit_flow_free(t_cv_jit_flow *x)
{
	
}