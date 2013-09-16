#ifndef MOGES_NURBS_FACTORIES_H
#define MOGES_NURBS_FACTORIES_H

#include "NURBS/Curve.h"
#include "Types.h"

#include <memory>

namespace MoGES
{
  namespace NURBS
  {
    CurvePtr clampedBSpline
    (
      Size kA,
      const std::vector<Point>& pA
    );

    CurvePtr periodicBSpline
    (
      Size kA,
      const std::vector<Point>& pA,
      bool sortControlPointsA = true
    );

    CurvePtr randomPeriodicCurve
    (
      Size degreeA = 2,
      Size numberOfControlPointsA = 8,
      Real minimumXA = -1,
      Real maximumXA = 1,
      Real minimumYA = -1,
      Real maximumYA = 1,
      Real minimumWeightA = 0.01,
      Real maximumWeightA = 1,
      Real minimumKnotSpanRatioA = 0.01,
      bool sortControlPointsA = true
    );
  }
}

#endif //ndef MOGES_NURBS_FACTORIES_H
