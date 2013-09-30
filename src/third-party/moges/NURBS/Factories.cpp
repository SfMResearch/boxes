#include "NURBS/Factories.h"
#include "NURBS/Curve.h"
#include "missing.h"
#include "Types.h"

#include <memory>
#include <random>


MoGES::NURBS::CurvePtr
MoGES::NURBS
::
clampedBSpline
(
  Size kA,
  const std::vector<Point>& pA
)
{
  // Create weights.
  std::vector<Real> wL(pA.size(), 1.0);

  // Create a proper knot vector.
  std::vector<Real> sL;
  sL.reserve(pA.size() + kA + 1);

  // Start with multiplicity of k.
  int countL = kA;
  while (countL--) sL.push_back(0.0);

  // Have equidistant knots in the center.
  Real stepL = 1.0 / static_cast<Real>(pA.size() - kA + 2);
  for (Size binL = 1; binL < pA.size() + 2 - kA; ++binL)
  {
    sL.push_back(stepL * binL);
  }

  // End with multiplicity of k.
  countL = kA;
  while (countL--) sL.push_back(1.0);

  return new_Curve(kA, pA, wL, sL, false);
}


MoGES::NURBS::CurvePtr
MoGES::NURBS
::
periodicBSpline
(
  Size kA,
  const std::vector<Point>& pA,
  bool sortControlPointsA
)
{
  // Create weights.
  std::vector<Real> wL(pA.size(), 1.0);

  // Create a proper knot vector.
  std::vector<Real> sL;
  sL.reserve(pA.size() + 1);
  for (Size nL = 0; nL <= pA.size(); ++nL)
    sL.push_back(static_cast<Real>(nL) / static_cast<Real>(pA.size()));

  if (sortControlPointsA)
  {
    std::vector<Point> pL = pA;
    missing::sortCircular(pL);

    return new_Curve(kA, pL, wL, sL, true);
  }

  return new_Curve(kA, pA, wL, sL, true);
}


MoGES::NURBS::CurvePtr
MoGES::NURBS
::
randomPeriodicCurve
(
  Size orderA,
  Size numberOfControlPointsA,
  Real minimumXA,
  Real maximumXA,
  Real minimumYA,
  Real maximumYA,
  Real minimumWeightA,
  Real maximumWeightA,
  Real minimumKnotSpanRatioA,
  bool sortControlPointsA
)
{
  typedef std::uniform_real_distribution<Real> DistributionL;

  //todo?: #ifndef PRAGMA_DISABLE_SANITY
  if (minimumKnotSpanRatioA < EPSILON)  minimumKnotSpanRatioA = EPSILON;

  // Create a random number generator and a uniform distribution to draw from.
  std::random_device deviceL;
  std::mt19937 generatorL(deviceL());
  DistributionL distributionL(0, 1);

  // Create a knot vector. --------------------------------
  std::vector<Real> knotsL;
  knotsL.reserve(numberOfControlPointsA+1);
  // Always start the valid range at 0.0.
  knotsL.push_back(0.0);
  // Define the knots to have spans of random length.
  for (Size iL = 0; iL < numberOfControlPointsA; ++iL)
  {
    knotsL.push_back(knotsL.back() + distributionL(generatorL));
  }
  // Normalize:
  Real normL = knotsL.back();
  std::for_each
  ( knotsL.begin()+1, knotsL.end(),
    [&normL] (Real& knotA)
    {
      knotA /= normL;
    }
  );

  // Create control points. -------------------------------
  DistributionL xDistributionL(minimumXA, maximumXA);
  DistributionL yDistributionL(minimumYA, maximumYA);
  DistributionL weightDistributionL(minimumWeightA, maximumWeightA);
  // Create vector of control points.
  std::vector<Point> controlPointsL;
  std::vector<Real> weightsL;
  controlPointsL.reserve(numberOfControlPointsA);
  weightsL.reserve(numberOfControlPointsA);
  // Fill the vector with random points.
  for (Size iL = 0; iL < numberOfControlPointsA; ++iL)
  {
    // Get random coordintes and weights from the respective ranges.
    Real xL = xDistributionL(generatorL);
    Real yL = yDistributionL(generatorL);
    Real wL = weightDistributionL(generatorL);

    // Shove that random control point into the vector.
    controlPointsL.push_back({xL, yL});
    weightsL.push_back(wL);
  }
  if (sortControlPointsA)  missing::sortCircular(controlPointsL);

  // Create the random NURBS.
  return new_Curve(orderA, controlPointsL, weightsL, knotsL, true);
}
