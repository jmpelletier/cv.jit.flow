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
#include "max.jit.mop.h"

#ifdef __cplusplus 
} //extern "C"
#endif

typedef struct _max_cv_jit_flow 
{
	t_object		ob;
	void			*obex;
} t_max_cv_jit_flow;

t_jit_err cv_jit_flow_init(void); 

void *max_cv_jit_flow_new(t_symbol *s, long argc, t_atom *argv);
void max_cv_jit_flow_free(t_max_cv_jit_flow *x);

void *max_cv_jit_flow_class;
		 	
#ifdef __cplusplus
extern "C"
#endif
int main(void)
{	
	void *p,*q;
	
	cv_jit_flow_init();
	setup(
			(t_messlist **)&max_cv_jit_flow_class,	//A pointer to the Max class pointer
			(method)max_cv_jit_flow_new,			//The constructor function
			(method)max_cv_jit_flow_free,			//The destructor function
			(short)sizeof(t_max_cv_jit_flow),		//The size of the Max class
			0L,											//Use only for GUI objects, null for standard Jitter objects
			A_GIMME,									//Pass arguments as a list of t_atoms
			0);											//End of type list

	p = max_jit_classex_setup(calcoffset(t_max_cv_jit_flow,obex));	//Setup Max class
	q = jit_class_findbyname(gensym("cv_jit_flow"));				//Get a pointer to the Jitter object class
    max_jit_classex_mop_wrap(p,q,0); 		
    max_jit_classex_standard_wrap(p,q,0); 	

    addmess((method)max_jit_mop_assist, "assist", A_CANT,0);	//Add outlet assistance to object
	
	return 0;
}

void max_cv_jit_flow_free(t_max_cv_jit_flow *x)
{
	max_jit_mop_free(x);		//Free the matrix operator
	jit_object_free(max_jit_obex_jitob_get(x));	//Free the Jitter object
	max_jit_obex_free(x);		//Free the Max wrapper object
}

void *max_cv_jit_flow_new(t_symbol *s, long argc, t_atom *argv)
{
	t_max_cv_jit_flow *x;
	void *o;

	if (x=(t_max_cv_jit_flow *)max_jit_obex_new(max_cv_jit_flow_class,gensym("cv_jit_flow"))) {
		if (o=jit_object_new(gensym("cv_jit_flow"))) {
			max_jit_mop_setup_simple(x,o,argc,argv);			
			max_jit_attr_args(x,argc,argv);
		} else {
			error("cv.jit.flow: could not allocate object");
			freeobject((t_object *)x);
		}
	}
	return (x);
}