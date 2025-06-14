#ifndef _DLL_H_
#define _DLL_H_

#if BUILDING_DLL
#define DLLIMPORT __declspec(dllexport)
#else
#define DLLIMPORT __declspec(dllimport)
#endif

DLLIMPORT void free(void* ptr);
DLLIMPORT void set_I(double x, double y, double z);
DLLIMPORT void norm(double* q);
DLLIMPORT double* quaternion(double q0, double q1, double q2, double q3,
	double q0_dot, double q1_dot, double q2_dot, double q3_dot,
	double Mx, double My, double Mz);
DLLIMPORT double* jacobian(double q0, double q1, double q2, double q3,
	double q0_dot, double q1_dot, double q2_dot, double q3_dot,
	double Mx, double My, double Mz);
DLLIMPORT double* f(double* x, double* u);
DLLIMPORT double* gradient(double* x, double *u);

#endif
