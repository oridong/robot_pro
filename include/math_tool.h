#ifndef MATHTOOL_H
#define MATHTOOL_H

extern double max(double *x, int size);
extern void cross(const double b_a[3], const double b_b[3], double b_c[3]);
extern double dot(const double b_a[3], const double b_b[3]);
extern double b_norm(const double x[3]);
extern void b_sign(double *x);
extern void b_sqrt(double *x);
extern double sum(const double x_data[]);
extern void mldivide(const double A[16], double B[16]);
extern void mldivide_rot(const double A[9], const double B[9], double Y[9]);
extern void matrixMultiply(double *left, int xl, int yl, double *right, int xr, int yr, double *result);
extern void slerp(float starting[4], float ending[4], float result[4], float t );
extern void quatU2Axis(const double qin[4], double qout[4]);

extern void printf_d(double * data, int size);
#endif 