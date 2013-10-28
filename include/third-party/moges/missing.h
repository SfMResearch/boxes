#ifndef MISSING_H
#define MISSING_H

#include "Types.h"

#include <cassert>
#include <cmath>
#include <ctime>
#include <functional>
#include <iostream>
#include <iterator>
#include <random>
#include <string>
#include <utility>
#include <valarray>

namespace MoGES
{
  //! Finds the lowest index for which condition holds
  template <class Size>
  Size binarySearch (Size minimumA, Size maximumA, std::function<bool (Size)> conditionA)
  {
    while (minimumA < maximumA)
    {
      Size centerL = (minimumA + maximumA) / 2;
      if (conditionA(centerL))  maximumA = centerL;
      else  minimumA = centerL + 1;
    }
    return minimumA;
  }

  template <class Container>
  Container&
  split
  (
    InStream& sourceA,
    Container& segmentsA,
    Character delimiterA = '\n'
  )
  {
    String segmentL;
    while (std::getline<Character>(sourceA, segmentL, delimiterA))
    {
      segmentsA.push_back(segmentL);
    }
    return segmentsA;
  }

  template <class Container>
  Container
  split
  (
    InStream& sourceA,
    Character delimiterA = '\n'
  )
  {
    Container segmentsL;
    return split<Container>(sourceA, segmentsL, delimiterA);
  }


  template <class T>
  T square(const T& valueA)
  {
    return valueA * valueA;
  }

  template <class T>
  T cube(const T& valueA)
  {
    return valueA * valueA * valueA;
  }

  //! Creates an IntPoint by rounding a Point (i.e.: each coordinate).
  IntPoint round (const Point&);

  //! Computes the squared norm of a point.
  template <class T>
  Real squaredNorm (const std::valarray<T>& pointA)
  {
    return square(pointA).sum();
    //return std::pow<T>(pointA, 2).sum() / static_cast<Real>(pointA.size());
  }

  template <class T>
  std::valarray<T>& normalize (std::valarray<T>& pointA)
  {
    Real normL = std::sqrt(squaredNorm<T>(pointA));

    // Prevent division by zero, but still try to normalize.
    while (normL < EPSILON)
    {
      pointA /= EPSILON;
      normL = std::sqrt(squaredNorm<T>(pointA));
    }

    pointA /= normL;
    return pointA;
  }

  namespace missing
  {
    //! Compute the area of a simple polygon - vertices must be ordered according
    //! to positive orientation. (see https://en.wikipedia.org/wiki/Polygon#Area_and_centroid)
    template <class BidirectionalIterator>
    Real
    area
    (
      //std::iterator<std::bidirectional_iterator_tag, Point2D> beginA,
      //std::iterator<std::bidirectional_iterator_tag, Point2D> endA
      BidirectionalIterator beginA,
      BidirectionalIterator endA
    )
    {
      // Start with edge from last to first vertex.
      Real areaL = (endA-1)->x() * beginA->y() - beginA->x() * (endA-1)->y();
      while (++beginA != endA)
      {
        areaL += (beginA-1)->x() * beginA->y() - beginA->x() * (beginA-1)->y();
      }

      return areaL / 2.0;
    }

    template <class Point2D>
    Point2D&
    rotate
    (
      Point2D& vectorA,
      Real angleA
    )
    {
      //assert(vectorA.size() == 2);

      const Real sinL = std::sin(angleA);
      const Real cosL = std::cos(angleA);

      Real xL = cosL * vectorA[0] - sinL * vectorA[1];

      vectorA[1] = sinL * vectorA[0] + cosL * vectorA[1];
      vectorA[0] = xL;

      return vectorA;
    }


    Integer
    roundDown
    (
      Real numberA
    );

    template <class Point2D>
    Point2D
    centroid
    (
      const std::vector<Point2D>& pointsA
    )
    {
      return pointsA.size() > 0
        ? std::accumulate(pointsA.begin(), pointsA.end(), Point2D({0, 0}))
          / static_cast<Real>(pointsA.size())
        : Point2D({0,0});
    }

    void
    sortCircular
    (
      std::vector<Point>& pointsA
    );

    Real
    positiveAngle
    (
      Real angleA
    );

    Size factorial (Size nA);

    Size fallingFactorial (Size nA, Size kA);

    Size binomialCoefficient(Size nA, Size kA);

    template <class Point2D>
    void
    coutPoint
    (
      const Point2D& pointA
    )
    {
      std::cout << "\t(" << pointA[0] << ", " << pointA[1] << ")" << std::flush;
    }

    template <class Point2D>
    Real
    positiveAngleToXAxis
    (
      const Point2D& vectorA
    )
    {
      return positiveAngle(std::atan2(vectorA[1], vectorA[0]));
    }

    template <class Point2D>
    Real
    angle
    (
      const Point2D& vectorFromA,
      const Point2D& vectorToA
    )
    {
      Real ang1 = positiveAngleToXAxis(vectorToA);
      Real ang2 = positiveAngleToXAxis(vectorFromA);
      return ang1 - ang2;
      //return positiveAngleToXAxis(vectorToA) - positiveAngleToXAxis(vectorFromA);
    }

    template <class Point2D>
    Real
    positiveAngle
    (
      const Point2D& vectorFromA,
      const Point2D& vectorToA
    )
    {
      Real angleL = angle<Point2D>(vectorFromA, vectorToA); //todo (for curiuousity): If the template argument <Point2D> is omitted this produces a floating point exception; why?
      Real posAngL = positiveAngle(angleL);
      return posAngL;
      //return positiveAngle(angle(vectorFromA, vectorToA));
    }

    template <class Point2D>
    bool
    smallerAngleToXAxis
    (
      const Point2D& point1A,
      const Point2D& point2A
    )
    {
      Real angle1L = positiveAngleToXAxis<Point2D>(point1A);
      Real angle2L = positiveAngleToXAxis<Point2D>(point2A);

      if (angle1L < angle2L)
      {
        return true;
      }
      /*todo? I think it only makes the convex hull faster
      else if (angle1L == angle2L)
      {
        return (point1A.norm2() < point2A.norm2());
      }*/

      return false;
    }

    template <class RandomAccessIterator>
    void
    shuffle
    (
      RandomAccessIterator beginL,
      RandomAccessIterator endL
    )
    {
      //std::random_device randomDeviceL;
      //std::mt19937 mersenneTwisterL(randomDeviceL);
      std::mt19937 mersenneTwisterL;
      mersenneTwisterL.seed(static_cast<unsigned int>(std::time(0)));
      std::shuffle(beginL, endL, mersenneTwisterL);
    }
  }
}

#endif //ndef MISSING_H
