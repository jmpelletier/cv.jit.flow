/*OpenCV support for Jitter
Copyright 2006
Jean-Marc Pelletier 
jmp@iamas.ac.jp*/

#include <cv.h>
#include <jit.common.h>

CvMat jitMatrix2CvMat(void *jitMatrix);
void* cvMat2jitMatrix(CvMat *cvMatrix);



CvMat jitMatrix2CvMat(void *jitMatrix)
{
	CvMat cvMatrix;
	t_jit_matrix_info info;
	unsigned char *bp;
	int type;

	if(jitMatrix)
	{
		jit_object_method(jitMatrix,_jit_sym_getinfo,&info);	
		jit_object_method(jitMatrix,_jit_sym_getdata,&bp);

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

		cvInitMatHeader( &cvMatrix, info.dim[1], info.dim[0], type, bp, info.dimstride[1] );
	}
	return cvMatrix;
}
