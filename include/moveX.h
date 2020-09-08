#ifndef MOVEX_H
#define MOVEX_H

#include "control.h"

extern void moveJ(bodypart &arm, double jointFinal[7], double speedRate);
extern void moveLPoseUnchanged(double angleInit[7], const double locationFinal[3], double speedRate, bodypart arm);
extern void moveLPoseChanged(bodypart &arm, double poseFinal[6], double speedRate);
extern void moveCPoseUnchanged(double angleInit[7], const double pointMiddle[3], const double pointFinal[3], double speedRate, bodypart arm);
extern void moveCPoseChanged(double angleInit[7], const double pointMiddle[3], const double pointFinal[3], double speedRate, bodypart arm);

#endif // !MOVEX_H