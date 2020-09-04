#ifndef STRAJECTORY_H
#define STRAJECTORY_H

/* Include Files */


/* Function Declarations */
extern void STrajectoryPara(double q0, double q1, double limit[4], double para[16]);

extern double S_position(double t, const double para[16]);

#endif