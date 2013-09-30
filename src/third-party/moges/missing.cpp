#include "missing.h"

#include <cmath>
#include <list>


MoGES::IntPoint
MoGES
::
round
(
  const Point& pointA
)
{
  IntPoint roundedL(pointA.size());

  for (Size iL = 0; iL < pointA.size(); ++iL)
  {
    roundedL[iL] = std::round(pointA[iL]);
  }

  return roundedL;
}


// todo: replace with lookup table
// This childish implementation of a factorial is used for small n only.
MoGES::Size
MoGES::missing
::
factorial
(
  MoGES::Size nA
)
{
  return (nA < 2)  ?  1  :  factorial(nA-1) * nA;
}

MoGES::Size
MoGES::missing
::
fallingFactorial
(
  MoGES::Size xA,
  MoGES::Size nA
)
{
  if (xA < nA)  return 0;

  MoGES::Size factorialL = xA;
  while (--nA)  factorialL *= (xA - nA);

  return factorialL;
}

MoGES::Size
MoGES::missing
::
binomialCoefficient
(
  MoGES::Size nA,
  MoGES::Size kA
)
{
  return fallingFactorial(nA,kA) / factorial(kA);
}


MoGES::Real
MoGES::missing::positiveAngle
(
  Real angleA
)
{
  return angleA < 0.0 ? std::fmod(angleA, PI * 2.0) + PI * 2.0
                      : std::fmod(angleA, PI * 2.0);
}


MoGES::Integer
MoGES::missing::roundDown
(
  Real numberA
)
{
  return  numberA < 0.0  ?  static_cast<Integer>(numberA) - 1
                         :  static_cast<Integer>(numberA);
}


void
MoGES::missing::sortCircular
(
  std::vector<Point>& pointsA
)
{
  // Compute the center of gravity.
  Point centroidL = centroid(pointsA);

  // Copy points to the more suitable list container and make them mean free.
  std::list<Point> pointsL;
  for
  ( std::vector<Point>::const_iterator pointItL = pointsA.begin();
    pointItL < pointsA.end();
    ++pointItL )
  {
    pointsL.push_back(*pointItL - centroidL);
  }

  // Sort points counter-clockwise.
  pointsL.sort(smallerAngleToXAxis<Point>);

  // Copy points to vector and restore their original position in space.
  std::list<Point>::const_iterator pointItL = pointsL.begin();
  std::vector<Point>::iterator resultItL = pointsA.begin();
  while (pointItL != pointsL.end())
  {
    *resultItL = *pointItL + centroidL;

    ++resultItL;  ++pointItL;
  }
}
