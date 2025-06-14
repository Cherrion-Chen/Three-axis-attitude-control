#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include "quaternion_euler.h"

double* euler_to_quaternion(double roll, double pitch, double yaw) {
	double *op = (double *)calloc(4, sizeof(double));

	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
		
	op[0] = cr * cp * cy + sr * sp * sy;
	op[1] = sr * cp * cy - cr * sp * sy;
	op[2] = cr * sp * cy + sr * cp * sy;
	op[3] = cr * cp * sy - sr * sp * cy;
	
	return op;
}

double* quaternion_to_euler(double q0, double q1, double q2, double q3) {
	double *op = (double *)calloc(3, sizeof(double));

	// 计算俯仰角（pitch）
	double sinp = 2 * (q0 * q2 - q1 * q3);
	if (fabs(sinp) >= 1) {
		op[1] = copysign(M_PI / 2, sinp); // 处理奇异值
	}
	else {
		op[1] = asin(sinp);
	}

	// 计算横滚角（roll）和偏航角（yaw）
	op[0] = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
	op[2] = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

	return op;
}

int main() {
	double roll, pitch, yaw;
	double *op;
	while (1) {
		printf("依次输入横滚角，俯仰角，偏航角(deg)，空格衔接，直接回车退出：\n");
		if (scanf("%lf %lf %lf", &roll, &pitch, &yaw) != 3) {
			// 检测是否输入为空（直接回车）
			if (getchar() == '\n') {
				break; // 退出循环
			}
			printf("输入无效，请重新输入。\n");
			// 清空输入缓冲区
			while (getchar() != '\n');
			continue;
		}
		
		// 将角度从度转换为弧度
		roll *= M_PI / 180.0;
		pitch *= M_PI / 180.0;
		yaw *= M_PI / 180.0;
		
		op = euler_to_quaternion(roll, pitch, yaw);
		printf("四元数: %f, %f, %f, %f\n\n", op[0], op[1], op[2], op[3]);
		free(op);
		
		// 清空输入缓冲区
		while (getchar() != '\n');
	}
	return 0;
}

