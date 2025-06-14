#ifndef _DLL_H_
#define _DLL_H_

#if BUILDING_DLL
#define DLLIMPORT __declspec(dllexport)
#else
#define DLLIMPORT __declspec(dllimport)
#endif

DLLIMPORT void free(void* ptr);
DLLIMPORT void set_I(float x, float y, float z);
DLLIMPORT void norm(float* q);
DLLIMPORT float* quaternion(float q0, float q1, float q2, float q3,
	float q0_dot, float q1_dot, float q2_dot, float q3_dot,
	float Mx, float My, float Mz);
DLLIMPORT float* jacobian(float q0, float q1, float q2, float q3,
	float q0_dot, float q1_dot, float q2_dot, float q3_dot,
	float Mx, float My, float Mz);
DLLIMPORT float* f(float* x, float* u);
DLLIMPORT float* gradient(float* x, float *u);

#endif
