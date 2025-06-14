/* Replace "dll.h" with the name of your header */
#include "dll.h"
#include <windows.h>
#include <stdlib.h>
#include <math.h>

BOOL WINAPI DllMain(HINSTANCE hinstDLL,DWORD fdwReason,LPVOID lpvReserved)
{
	switch(fdwReason)
	{
	case DLL_PROCESS_ATTACH:
		{
			break;
		}
	case DLL_PROCESS_DETACH:
		{
			break;
		}
	case DLL_THREAD_ATTACH:
		{
			break;
		}
	case DLL_THREAD_DETACH:
		{
			break;
		}
	}
	
	/* Return TRUE on success, FALSE on failure */
	return TRUE;
}

static double Ix = 1., Iy = 1., Iz = 1.;

void set_I(double x, double y, double z) {
	Ix = x;
	Iy = y;
	Iz = z;
}

void norm(double* q) {
	double r;
	r = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
	r = sqrt(r);
	for (int i = 0; i < 4; i++) {
		q[i] /= r;
	}
}

// 四元数变化率与力矩关系
double* quaternion(double q0, double q1, double q2, double q3,
	double q0_dot, double q1_dot, double q2_dot, double q3_dot,
	double Mx, double My, double Mz) {
		double* q_ddot = (double*)calloc(4, sizeof(double));
		
		// 计算角速度分量
		double wx = 2*(-q1_dot*q0 + q0_dot*q1 - q3_dot*q2 + q2_dot*q3);
		double wy = 2*(-q2_dot*q0 + q3_dot*q1 + q0_dot*q2 - q1_dot*q3);
		double wz = 2*(-q3_dot*q0 - q2_dot*q1 + q1_dot*q2 + q0_dot*q3);
		
		// 计算角加速度分量
		double wx_dot = (Mx - (Iz - Iy)*wy*wz)/Ix;
		double wy_dot = (My - (Ix - Iz)*wz*wx)/Iy;
		double wz_dot = (Mz - (Iy - Ix)*wx*wy)/Iz;
		
		// 计算四元数二阶导数
		q_ddot[0] = 0.5*(-q1_dot*wx - q2_dot*wy - q3_dot*wz - q1*wx_dot - q2*wy_dot - q3*wz_dot);
		q_ddot[1] = 0.5*(q0_dot*wx - q3_dot*wy + q2_dot*wz + q0*wx_dot - q3*wy_dot + q2*wz_dot);
		q_ddot[2] = 0.5*(q3_dot*wx + q0_dot*wy - q1_dot*wz + q3*wx_dot + q0*wy_dot - q1*wz_dot);
		q_ddot[3] = 0.5*(-q2_dot*wx + q1_dot*wy + q0_dot*wz - q2*wx_dot + q1*wy_dot + q0*wz_dot);
		
		return q_ddot;
	}

double* jacobian(double q0, double q1, double q2, double q3,
	double q0_dot, double q1_dot, double q2_dot, double q3_dot,
	double Mx, double My, double Mz) {
		double* J = (double*)calloc(44, sizeof(double)); // 4行11列展平后的数组
		
		// 计算角速度分量
		double wx = 2*(-q1_dot*q0 + q0_dot*q1 - q3_dot*q2 + q2_dot*q3);
		double wy = 2*(-q2_dot*q0 + q3_dot*q1 + q0_dot*q2 - q1_dot*q3);
		double wz = 2*(-q3_dot*q0 - q2_dot*q1 + q1_dot*q2 + q0_dot*q3);
		
		// 计算角加速度分量
		double wx_dot = (Mx - (Iz - Iy)*wy*wz)/Ix;
		double wy_dot = (My - (Ix - Iz)*wz*wx)/Iy;
		double wz_dot = (Mz - (Iy - Ix)*wx*wy)/Iz;
		
		// 计算雅可比矩阵的各个元素
		// 第0行（q_ddot[0]的偏导数）
		J[0*11 + 0] = 0.5 * (-q1_dot * (-2*q1_dot) + q0_dot * (2*q1) - q3_dot * (2*q2_dot) + q2_dot * (2*q3)
			- q1 * (Iz - Iy)*wy_dot - q2 * (Ix - Iz)*wz_dot - q3 * (Iy - Ix)*wx_dot); // 对q0的偏导
		J[0*11 + 1] = 0.5 * (-2*q1_dot*q0 + q0_dot*(2*q1) - q3_dot*(2*q2_dot) + q2_dot*(2*q3)
			- 1 * (Iz - Iy)*wy_dot - q2 * (Ix - Iz)*wz_dot - q3 * (Iy - Ix)*wx_dot); // 对q1的偏导
		J[0*11 + 2] = 0.5 * (q0_dot*(2*q2) - q3_dot*(2*q3_dot) - 2*q2_dot*q0
			- q1 * (Iz - Iy)*wy_dot - 1 * (Ix - Iz)*wz_dot - q3 * (Iy - Ix)*wx_dot); // 对q2的偏导
		J[0*11 + 3] = 0.5 * (q0_dot*(2*q3) - q3_dot*(2*q2_dot) + q2_dot*(2*q1)
			- q1 * (Iz - Iy)*wy_dot - q2 * (Ix - Iz)*wz_dot - 1 * (Iy - Ix)*wx_dot); // 对q3的偏导
		J[0*11 + 4] = 0.5 * (-q1*wx - q2*wy - q3*wz); // 对q0_dot的偏导
		J[0*11 + 5] = 0.5 * (-wx); // 对q1_dot的偏导
		J[0*11 + 6] = 0.5 * (-wy); // 对q2_dot的偏导
		J[0*11 + 7] = 0.5 * (-wz); // 对q3_dot的偏导
		J[0*11 + 8] = 0.5 * (- (Iz - Iy)*wy*wz / Ix); // 对Mx的偏导
		J[0*11 + 9] = 0.5 * (- (Ix - Iz)*wz*wx / Iy); // 对My的偏导
		J[0*11 + 10] = 0.5 * (- (Iy - Ix)*wx*wy / Iz); // 对Mz的偏导
		
		// 第1行（q_ddot[1]的偏导数）
		J[1*11 + 0] = 0.5 * (q0_dot*wx - q3_dot*wy + q2_dot*wz + q0*wx_dot - q3*wy_dot + q2*wz_dot); // 对q0的偏导
		J[1*11 + 1] = 0.5 * (q0*wx + q0_dot*wx_dot - q3*wy_dot + q2*wz_dot); // 对q1的偏导
		J[1*11 + 2] = 0.5 * (q0*wx + q0_dot*wx_dot - q3*wy_dot + q2*wz_dot); // 对q2的偏导
		J[1*11 + 3] = 0.5 * (q0*wx + q0_dot*wx_dot - q3*wy_dot + q2*wz_dot); // 对q3的偏导
		J[1*11 + 4] = 0.5 * (wx); // 对q0_dot的偏导
		J[1*11 + 5] = 0.5 * (q0*wx + q0_dot*wx_dot - q3*wy_dot + q2*wz_dot); // 对q1_dot的偏导
		J[1*11 + 6] = 0.5 * (q0*wx + q0_dot*wx_dot - q3*wy_dot + q2*wz_dot); // 对q2_dot的偏导
		J[1*11 + 7] = 0.5 * (q0*wx + q0_dot*wx_dot - q3*wy_dot + q2*wz_dot); // 对q3_dot的偏导
		J[1*11 + 8] = 0.5 * (1/Ix); // 对Mx的偏导
		J[1*11 + 9] = 0.5 * ( - (Iz - Iy)*wy / Ix ); // 对My的偏导
		J[1*11 + 10] = 0.5 * ( - (Iy - Ix)*wx / Iz ); // 对Mz的偏导
		
		// 第2行（q_ddot[2]的偏导数）
		J[2*11 + 0] = 0.5 * (q3_dot*wx + q0_dot*wy - q1_dot*wz + q3*wx_dot + q0*wy_dot - q1*wz_dot); // 对q0的偏导
		J[2*11 + 1] = 0.5 * (q3*wx + q3_dot*wx_dot + q0*wy_dot - q1*wz_dot); // 对q1的偏导
		J[2*11 + 2] = 0.5 * (q3*wx + q3_dot*wx_dot + q0*wy_dot - q1*wz_dot); // 对q2的偏导
		J[2*11 + 3] = 0.5 * (q3*wx + q3_dot*wx_dot + q0*wy_dot - q1*wz_dot); // 对q3的偏导
		J[2*11 + 4] = 0.5 * (wy); // 对q0_dot的偏导
		J[2*11 + 5] = 0.5 * (q3*wx + q3_dot*wx_dot + q0*wy_dot - q1*wz_dot); // 对q1_dot的偏导
		J[2*11 + 6] = 0.5 * (q3*wx + q3_dot*wx_dot + q0*wy_dot - q1*wz_dot); // 对q2_dot的偏导
		J[2*11 + 7] = 0.5 * (q3*wx + q3_dot*wx_dot + q0*wy_dot - q1*wz_dot); // 对q3_dot的偏导
		J[2*11 + 8] = 0.5 * ( - (Iz - Iy)*wy / Ix ); // 对Mx的偏导
		J[2*11 + 9] = 0.5 * (1/Iy); // 对My的偏导
		J[2*11 + 10] = 0.5 * ( - (Ix - Iz)*wz / Iy ); // 对Mz的偏导
		
		// 第3行（q_ddot[3]的偏导数）
		J[3*11 + 0] = 0.5 * (-q2_dot*wx + q1_dot*wy + q0_dot*wz - q2*wx_dot + q1*wy_dot + q0*wz_dot); // 对q0的偏导
		J[3*11 + 1] = 0.5 * (-q2*wx + q1*wy + q0_dot*wz_dot - q2*wx_dot + q1*wy_dot + q0*wz_dot); // 对q1的偏导
		J[3*11 + 2] = 0.5 * (-q2*wx + q1*wy + q0_dot*wz_dot - q2*wx_dot + q1*wy_dot + q0*wz_dot); // 对q2的偏导
		J[3*11 + 3] = 0.5 * (-q2*wx + q1*wy + q0_dot*wz_dot - q2*wx_dot + q1*wy_dot + q0*wz_dot); // 对q3的偏导
		J[3*11 + 4] = 0.5 * (wz); // 对q0_dot的偏导
		J[3*11 + 5] = 0.5 * (-q2*wx + q1*wy + q0_dot*wz_dot - q2*wx_dot + q1*wy_dot + q0*wz_dot); // 对q1_dot的偏导
		J[3*11 + 6] = 0.5 * (-q2*wx + q1*wy + q0_dot*wz_dot - q2*wx_dot + q1*wy_dot + q0*wz_dot); // 对q2_dot的偏导
		J[3*11 + 7] = 0.5 * (-q2*wx + q1*wy + q0_dot*wz_dot - q2*wx_dot + q1*wy_dot + q0*wz_dot); // 对q3_dot的偏导
		J[3*11 + 8] = 0.5 * ( - (Iy - Ix)*wx / Iz ); // 对Mx的偏导
		J[3*11 + 9] = 0.5 * ( - (Ix - Iz)*wz / Iy ); // 对My的偏导
		J[3*11 + 10] = 0.5 * (1/Iz); // 对Mz的偏导
		
		return J;
	}

double* f(double* x, double* u) {
	double* result = (double *)calloc(4, sizeof(double));
	double *op = quaternion(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7],u[0], u[1], u[2]);
	
	for (int i = 0; i < 4; i++){
		result[i] = op[i];
	}
	
	return result;
}

double* gradient(double* x, double* u) {
	double* result = (double *)calloc(44, sizeof(double));
	double *op = jacobian(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7],u[0], u[1], u[2]);
	
	for (int i = 0; i < 44; i++){
		result[i] = op[i];
	}
	
	return result;
}

