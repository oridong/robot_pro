#ifndef MOVEX_H
#define MOVEX_H

#include "control.h"

extern void moveJ(const double angleInit[7], const double poseOrjointFinal[16], double speedRate, bodypart arm);
extern void moveLPoseUnchanged(double angleInit[7], const double locationFinal[3], double speedRate, bodypart arm);
extern void moveLPoseChanged(double angleInit[7], const double poseFinal[16], double speedRate, bodypart arm);
extern void moveCPoseUnchanged(double angleInit[7], const double pointMiddle[3], const double pointFinal[3], double speedRate, bodypart arm);
extern void moveCPoseChanged(double angleInit[7], const double pointMiddle[3], const double pointFinal[3], double speedRate, bodypart arm);



#endif // !MOVEX_H