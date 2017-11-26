/*
	OpenCV support for Jitter

	Copyright (c) 2006-2017, Jean-Marc Pelletier
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

#undef error
#include "opencv.hpp"

#undef error
#include <jit.common.h>

CvMat jitMatrix2CvMat(void *jitMatrix);

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
