#ifndef MOTIONPLAN_H
#define MOTIONPLAN_H
#include "rt_type/rtwUtil.h"

/* Function Declarations */

extern void quat2rot(const double quaternionMatrix_data[4], double R[9]);
extern void rot2quat(const double rotationMatrix[9], double rEquivalent[4]);
extern void VectorRot(double n[3], double angle, double R[9]);

extern void CoordinateTrans(double alpha0, double a0, double theta1, double d1, double coordinateTrans[16]);
extern void ForwardKinematics(const double angle[7], double T07[16]);
extern void InverseKinematics(const double angleInit[7], double expectPose[16], double betaInit, double betaScanInterval, double betaEnd,
                   double angleByPlanning_data[], int angleByPlanning_size[2]);
extern double FindBeta(const double angle[7]);

extern void posfromT(double T[16], double pos[3]);
extern void rotfromT(double T[16], double rot[9]);
extern void TfromRotPos(const double rot[9], const double pos[3], double trans[16]);
extern void TfromPose(double pose[6], double T[16]);    // ZYX 顺规
extern void PosefromT(double T[16], double pose[6]);    // ZYX 顺规
extern void Jacobian(double jointangle[7], double J[42]);
extern int InvJacobian(double jointangle[7], double invJ[42]);

#endif