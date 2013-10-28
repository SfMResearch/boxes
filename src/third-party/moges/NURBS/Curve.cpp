#include "NURBS/Curve.h"
#include "missing.h"

#include <algorithm>
#include <cassert>
#include <cmath>
//#include <fstream>
#include <sstream>
#include <stdexcept>

const MoGES::String MoGES::NURBS::Curve::NAME_TAG = "MoGES::NURBS::CURVE";

// -- Method -- construct, destruct, assign, validate ---------------------- //

//! The default constructor is useless.
MoGES::NURBS::Curve
::
Curve
()
: degreeE(0),
  controlPointsE(),
  knotsE(),
  periodicE(false)
{
}

//! Copy constructor resembling the passed NURBS curve.
MoGES::NURBS::Curve::Curve
(
  const Curve& otherA
)
: degreeE(otherA.degreeE),
  controlPointsE(otherA.controlPointsE),
  knotsE(otherA.knotsE),
  periodicE(otherA.periodicE)
{
}

//! Move constructor taking over the passed NURBS curve.
MoGES::NURBS::Curve::Curve
(
  Curve&& otherA
)
: degreeE(otherA.degreeE),
  controlPointsE(std::move(otherA.controlPointsE)),
  knotsE(std::move(otherA.knotsE)),
  periodicE(otherA.periodicE)
{
}

//! Explicit constructor completely defining the curve.
MoGES::NURBS::Curve::Curve
(
  Size degreeA, // degree of the curve
  const std::vector<Point>& controlPointsA, // initial control points
  const std::vector<Real>& weightsA,
  const std::vector<Real>& knotsA,
  bool periodicA
)
: degreeE(degreeA),
  controlPointsE(),
  knotsE(knotsA),
  periodicE(periodicA)
{
  if (weightsA.size() != controlPointsA.size())
  {
    throw std::invalid_argument("Number of weights and control points does not match.");
  }
  else
  {
    // Convert control points and weights to homogenuous coordinates.
    controlPointsE.reserve(controlPointsA.size());
    for (Size iL = 0; iL < controlPointsA.size(); ++iL)
    {
      controlPointsE.emplace_back(map(controlPointsA[iL], weightsA[iL]));
    }
  }
}

//! Very explicit constructor completely defining the NURBS curve.
MoGES::NURBS::Curve::Curve
(
  Size degreeA,
  const std::vector<Point>& controlPointsA,
  const std::vector<Real>& knotsA,
  bool periodicA
)
: degreeE(degreeA),
  controlPointsE(controlPointsA),
  knotsE(knotsA),
  periodicE(periodicA)
{
}

//! Very explicit constructor completely defining the NURBS curve
//! moving control points and knots (instead of copying).
MoGES::NURBS::Curve::Curve
(
  Size degreeA,
  std::vector<Point>&& controlPointsA,
  std::vector<Real>&& knotsA,
  bool periodicA
)
: degreeE(degreeA),
  controlPointsE(std::move(controlPointsA)),
  knotsE(std::move(knotsA)),
  periodicE(periodicA)
{
}

//! Destroy the NURBS curve.
MoGES::NURBS::Curve::~Curve()
{}


//! Returns true, iff this is a valid NURBS curve.
bool
MoGES::NURBS::Curve::isValid 
(
  bool controlPointsMustBeSortedA
) const
{
  try
  {
    throwIfInvalid(controlPointsMustBeSortedA);
  }
  catch (const InvalidCurve& exceptionA)
  {
    return false;
  }

  return true;
}


//! Throws an exception, iff this is not a valid NURBS curve.
void
MoGES::NURBS::Curve::throwIfInvalid
(
  bool controlPointsMustBeSortedA
) const
{
  // Are there enough control points for the set degree?
  if (numberOfControlPoints() <= degreeE)  throw InvalidCurve("The number of control points of a NURBS must exceed its degree");

  // Does the NURBS have the correct number of knots?
  if (periodicE) //todo: don't make this distinction
  {
    if (knotsE.size() != (numberOfExplicitControlPoints() + 1))
    {
      throw InvalidCurve("For periodic NURBS the number of knots must equal n - k +1, with total number of control points n and degree k. So the number of knot spans equals the number of explicitly stored control points.");
    }
  }
  else
  {
    if (knotsE.size() != (numberOfControlPoints() + degreeE + 1))
    {
      throw InvalidCurve("The number of knots should be n + k +1 (for non-periodic NURBS), with total number of control points n and degree k.");
    }
  }

  // Is the vector of knots sorted in ascending order?
  if (not std::is_sorted(knotsE.begin(), knotsE.end()))
  {
    throw InvalidCurve("The elements of the knot vector should be in ascending order.");
  }

  // Do all knots have multiplicities not greater than degree k?
  Real previousKnotL = knotsE.front();
  Size repetitionsL = 1;
  std::for_each
  ( knotsE.begin()+1, knotsE.end(),
    [&previousKnotL, &repetitionsL, this] (Real knotA)
    {
      if ((knotA - previousKnotL) < EPSILON) //todo: reconsider this epsilon (in contrast to equality comparison)
      {
        ++repetitionsL;

        if (repetitionsL > (degreeE + 1))
        {
          throw InvalidCurve("No knot may have a multiplicity greater than the curve's degree + 1.");
        }
      }
      else repetitionsL = 1;

      previousKnotL = knotA;
    }
  );

  // Are all weights larger than zero?
  auto nonPositiveWeightItL
    = std::find_if_not (controlPointsE.begin(), controlPointsE.end(),
        [](const Point& pL)
        {
          return pL[2] > 0.0;
        });
  if (nonPositiveWeightItL != controlPointsE.end())
  {
    throw InvalidCurve("All weights must be positive.");
  }

  //todo: think of a way to ensure that the curve doesn't cross itself - the center of gravity thing is not the real deal
  if (controlPointsMustBeSortedA)
  {
    //todo: Are the control points sorted circularly around their center of gravity?
  }
}


// -- Method -- evaluate, derive, discretize, traverse, sample ------------- //

//! Evaluate this curve at sA (map curve parameter to point on the curve
//! in cartesian coordinates).
MoGES::Point
MoGES::NURBS::Curve::operator()
(
  Real sA
) const
{
#ifndef MOGES_DISABLE_CHECKS
  throwIfNotInRange(sA);
#endif //ndef MOGES_DISABLE_CHECKS
  //todo: compare: return map(value(sA));
  return map(homogeneous(sA));
}

//! Get derivative of this curve at sA.
MoGES::Point
MoGES::NURBS::Curve::operator()
(
  Size degreeA,
  Real sA
) const
{
#ifndef MOGES_DISABLE_CHECKS
  // Sanity checks.
  throwIfNotInRange(sA);
  // Derivatives beyond the NURBS' degree are zero.
  if (degreeA > degreeE)  return {0, 0};
#endif //ndef PRAGMA_DISABLE_CHECKS

  if (degreeA == 0)
  {
    return (*this)(sA);
  }
  else
  {
    // Sum up the mixed terms.
    Point mixL = {0, 0};
    for (Size dL = 1; dL < degreeA; ++dL)
    {
      mixL += (*this)(degreeA-dL, sA) * derivative(sA, dL)[2] * static_cast<Real>(missing::binomialCoefficient(degreeA, dL));
    }

    Point valueL = homogeneous(sA);
    Point derivativeL = derivative(sA, degreeA);
    mixL += map(valueL) * derivativeL[2];

    return (Point({derivativeL[0], derivativeL[1]}) - mixL) * (static_cast<Real>(1) / valueL[2]);
  }
}


//! Calculates the unit normal vector at s.
MoGES::Point
MoGES::NURBS::Curve
::
unitNormalVector
(
  Real sA
) const
{
  Point unitNormalVectorL = (*this)(1, sA);
  normalize(unitNormalVectorL);
  return missing::rotate(unitNormalVectorL, PI / Real(-2.0));//todo: rotation by pi/2 is coordinate switching (and fixing signs)
}


//! Get a discretized version of the curve.
MoGES::NURBS::DiscreteCurvePtr
MoGES::NURBS::Curve
::
discretize() const
{
  return discretize(beginOfValidRange(), endOfValidRange());
}


//! Get a discrete version of a segment of the curve.
MoGES::NURBS::DiscreteCurvePtr
MoGES::NURBS::Curve::discretize
(
  Real sMinA,
  Real sMaxA
) const
{
#ifndef PRAGMA_DISABLE_CHECKS
  throwIfNotInRange(sMinA);
  throwIfNotInRange(sMaxA);
#endif //ndef PRAGMA_DISABLE_CHECKS

  DiscreteCurvePtr curveL = new_DiscreteCurve();

  // Start recursion to find discretization.
  // For periodic NURBS the finishing criterion is imediately fulfilled (when
  // discretizing the entire curve). To work around this problem the interval
  // is halfed in advance. See Willfried Horn's dissertation.
  Real sHalfL = (sMinA + sMaxA) / static_cast<Real>(2.0);
  IntPoint posHalfL = round((*this)(sHalfL));
  discretizeRecursively(curveL, sMinA, round((*this)(sMinA)), sHalfL, posHalfL);
  IntPoint posMaxL = round((*this)(sMaxA));
  discretizeRecursively(curveL, sHalfL, posHalfL, sMaxA, posMaxL);

  return curveL;
}

//! Call doA with each point on a discrete version of this curve.
MoGES::Size
MoGES::NURBS::Curve::traverse
(
  std::function<void (const IntPoint&)> doA
) const
{
  return traverse(doA, beginOfValidRange(), endOfValidRange());
}

 
MoGES::Size
MoGES::NURBS::Curve::traverse
(
  std::function<void (const IntPoint&)> doA,
  Real sMinA,
  Real sMaxA
) const
{
  #ifndef PRAGMA_DISABLE_CHECKS
    throwIfNotInRange(sMinA);
    throwIfNotInRange(sMaxA);
  #endif //ndef PRAGMA_DISABLE_CHECKS

  DiscreteCurvePtr curveL = discretize(sMinA, sMaxA);

  std::for_each
  (
    curveL->begin(),
    curveL->end(),
    [&doA] (const DiscreteCurve::value_type& pointA)
    {
      doA(pointA.second);
    }
  );
  
  return curveL->size();
}


void
MoGES::NURBS::Curve::sample
(
  std::function<void (const Point&)> doA,
  Size samplesA
) const
{
  Real stepL = (endOfValidRange() - beginOfValidRange()) / Real(samplesA);
  Real startL = beginOfValidRange() + stepL/2.0;
  for (Size sL = 0; sL < samplesA; ++sL)
  {
    doA( (*this)( startL + stepL * Real(sL) ) );
  }
}


// -- Method -- get, set --------------------------------------------------- //

//! Reading access to degree of the curve.
MoGES::Size
MoGES::NURBS::Curve::degree
() const
{
  return degreeE;
}

//! Reading access to number of control points.
//! For periodic curves first and last degreeE control points are identical.
MoGES::Size
MoGES::NURBS::Curve
::
numberOfControlPoints
() const
{
  return  periodicE  ?  controlPointsE.size() + degreeE  :  controlPointsE.size();
}

//! Reading access to number of explicitly stored control points.
MoGES::Size
MoGES::NURBS::Curve
::
numberOfExplicitControlPoints
() const
{
  return controlPointsE.size();
}

//! Reading access to the number of knots.
MoGES::Size
MoGES::NURBS::Curve
::
numberOfKnots
() const
{
  return numberOfControlPoints() + degreeE + 1;
}

//! Returns true, iff the NURBS curve is periodic (closed).
bool
MoGES::NURBS::Curve
::
periodic
() const
{
  return periodicE;
}

//! Mapped reading access to the curve's control points. Handles periodic property.
MoGES::Point
MoGES::NURBS::Curve
::
controlPoint
(
  Size iA
) const
{
  return map(controlPointsE.at(index(iA)));
}

//! Mapped writing access to the curve's control points. Handles periodic property.
void
MoGES::NURBS::Curve
::
controlPoint
(
  Size iA,
  const Point& p_iA
)
{
  Size iL = index(iA);
  controlPointsE.at(iL) = map(p_iA, controlPointsE.at(iL)[2]);
}

//! Mapped reading access to the control points' weights. Handles periodic property.
MoGES::Real
MoGES::NURBS::Curve
::
weight
(
  Size iA
) const
{
  return controlPointsE.at(index(iA))[2];
}

//! Mapped writing access to the control points' weights. Handles periodic property.
void
MoGES::NURBS::Curve
::
weight
(
  Size iA,
  Real w_iA
)
{
  Point& pL = controlPointsE.at(index(iA));
  pL = map( map(pL), w_iA);
}

//! Reading access to this NURBS' knots.
MoGES::Real
MoGES::NURBS::Curve
::
knot
(
  Size iA
) const
{
  if (periodicE)
  {
    const Size nL = numberOfControlPoints();
    // These knots are computed to have the proper relation to the last valid ranges.
    if (iA < degreeE)
    {
      return knotsE.at(0) + knotsE.at(nL + iA - 2*degreeE) - knotsE.at(nL - degreeE);
    }
    // These knots are computed to have the proper relation to the first valid ranges.
    else if ((iA > nL) && /*is valid knot index*/(iA < nL + degreeE + 1))
    {
      return knotsE.at(nL-degreeE) + knotsE.at(iA - nL);
    }
    // These knots are free parameters.
    else
    {
      Real knotL = knotsE.at(iA - degreeE);
      return knotL;
    }
  }
  else  return knotsE.at(iA);
}

//! Writing access to this NURBS' knots.
void
MoGES::NURBS::Curve
::
knot
(
  Size iA,
  Real s_iA
)
{
  if (periodicE)
  {
    if ((iA < degreeE) || (iA > numberOfControlPoints()))
    {
      throw std::invalid_argument("The first and last degree knots of a periodic NURBS can't be modified.");
    }

    else  knotsE.at(iA - degreeE) = s_iA;
  }
  else  knotsE.at(iA) = s_iA;
}

//! Minimum of the valid range of curve parameter s (inclusive).
MoGES::Real
MoGES::NURBS::Curve
::
beginOfValidRange
() const
{
  return knotsE.front();
}

//! Maximum of the valid range of curve parameter s (exclusive).
MoGES::Real
MoGES::NURBS::Curve
::
endOfValidRange
() const
{
  return knotsE.back();
}

MoGES::Point
MoGES::NURBS::Curve
::
reachOfControlPoint
(
  Size indexA
) const
{
  Point rangeL = {knot(indexA), knot(indexA+degreeE+1)};
  for (Real& elementL : rangeL)
  {
    if (elementL < beginOfValidRange())
    {
      elementL += endOfValidRange() - beginOfValidRange();
    }
    else if (elementL > endOfValidRange())
    {
      elementL += beginOfValidRange() - endOfValidRange();
    }
  }

  return rangeL;
}


//! Insert a new knot without changing the curves shape.
void
MoGES::NURBS::Curve
::
insertKnot
(
  Real knotA
)
{
  // Find the correct index for the new knot.
  Size indexL = upperLimitIndex(knotA);

  // Create the new vector of control points.
  std::vector<Point> pL;
  pL.reserve(controlPointsE.size()+1);

  // todo: don't use an additional vector to do this
  // These control points before the new knot don't change.
  for (Size iL = 0; iL < indexL - degreeE; ++iL)
  {
    pL.push_back(controlPointsE.at(index(iL)));
  }
  // These contol points need to be computed.
  for (Size iL = indexL - degreeE; iL < indexL; ++iL)
  {
    Point p1L = {0,0,0};
    try
    {
     p1L = controlPointsE.at(index(iL));
    }catch(...){}

    Point p2L = {0,0,0};
    try{
      p2L = controlPointsE.at(index(iL-1));}catch(...){}
    pL.push_back
    (
      p1L * ((knotA - knot(iL)) / (knot(iL+degreeE) - knot(iL))) +
      p2L * ((knot(iL+degreeE) - knotA) / (knot(iL+degreeE) - knot(iL)))
      //controlPointsE.at(index(iL)) * ((knotA - knot(iL)) / (knot(iL+degreeE) - knot(iL))) +
      //controlPointsE.at(index(iL-1)) * ((knot(iL+degreeE) - knotA) / (knot(iL+degreeE) - knot(iL)))
    );
  }
  // These control points after the new knot don't change.
  for (Size iL = indexL-1; iL < controlPointsE.size(); ++iL)
  {
    pL.push_back(controlPointsE.at(index(iL)));
  }

  // Insert the new knot.
  Size newKnotIndexL = periodicE ? indexL - degreeE : indexL;
  knotsE.insert(knotsE.begin() + newKnotIndexL, knotA);

  // Switch to the new control points.
  controlPointsE = pL;
}


//! Translates the curve.
void
MoGES::NURBS::Curve
::
translate
(
  const Point& offsetA
)
{
  // Ensure the offset's dimensionality is correct.
  assert(offsetA.size() == 2);
  // Displace all control points.
  for (Size nL = 0; nL < numberOfExplicitControlPoints(); ++nL)
  {
    controlPoint(nL, controlPoint(nL) + offsetA);
  }
}

//! Scales the curve.
void
MoGES::NURBS::Curve
::
scale
(
  Real factorA,
  const Point& centerA
)
{
  // Ensure the center's dimensionality is correct.
  assert(centerA.size() == 2);
  // Scale all control points.
  for (Size nL = 0; nL < numberOfExplicitControlPoints(); ++nL)
  {
    controlPoint(nL, (controlPoint(nL) - centerA) * factorA);
  }
}

//! Rotates the curve.
void
MoGES::NURBS::Curve
::
rotate
(
  Real angleA,
  const Point& centerA
)
{
  // Ensure the center's dimensionality is correct.
  assert(centerA.size() == 2);
  // Rotate all control points.
  for (Size nL = 0; nL < numberOfExplicitControlPoints(); ++nL)
  {
    Point controlPointL = controlPoint(nL) - centerA;
    controlPoint(nL, missing::rotate(controlPointL, angleA));
  }
}

// -- Method -- output ----------------------------------------------------- //

//#define NURBS_IO_NAME_TAG "MoGES::NURBS::CURVE"
#define NURBS_IO_DEGREE_TAG "degree = "
#define NURBS_IO_POLYGON_TAG  " control points and weights:"
#define NURBS_IO_CP_START "  ("
#define NURBS_IO_CP_XY_DELIM ", "
#define NURBS_IO_CP_WEIGHT_DELIM ") * "
#define NURBS_IO_CP_END ";"
#define NURBS_IO_KNOTS_TAG " knots: "
#define NURBS_IO_KNOTS_DELIM " - "
#define ENDLINE '\n'

//! Writes this NURBS curve to an output stream.
void
MoGES::NURBS::Curve
::
write
(
  OutStream& outStreamA
) const
{
  // Set the stream's precision to ensure consistency of the NURBS.
  Integer minimalPrecisionL = -std::log10(EPSILON) + 1;
  if (minimalPrecisionL > outStreamA.precision())
  {
    outStreamA.precision(minimalPrecisionL);
  }

  outStreamA << NAME_TAG << std::endl;
  //outStreamA.write(NURBS_IO_NAME_TAG, 19);

  outStreamA << NURBS_IO_DEGREE_TAG << degreeE << ENDLINE;

  Size nL = controlPointsE.size();
  outStreamA << nL << NURBS_IO_POLYGON_TAG;
  for (Size iL = 0; iL < nL; ++iL)
  {
    Point pointL = controlPoint(iL);
    outStreamA << NURBS_IO_CP_START << pointL[0] << NURBS_IO_CP_XY_DELIM << pointL[1] << NURBS_IO_CP_WEIGHT_DELIM << weight(iL) << NURBS_IO_CP_END;
  }
  outStreamA << ENDLINE;

  nL = knotsE.size();
  outStreamA << nL << NURBS_IO_KNOTS_TAG << knotsE.at(0);
  for (Size iL = 1; iL < nL; ++iL)
  {
    outStreamA << NURBS_IO_KNOTS_DELIM << knotsE[iL];
  }
  outStreamA << ENDLINE;
}


//! Reads a NURBS curve from an input stream.
void
MoGES::NURBS::Curve
::
read
(
  InStream& inStreamA
)
{
  // Read the name tag.
  String tagL;
  tagL.resize(NAME_TAG.size());
  inStreamA.getline(&tagL[0], tagL.size()+1);

  // -- Read degree. ----------------------------------------------------------
  // Read the degree tag.
  tagL = NURBS_IO_DEGREE_TAG;
  inStreamA.get(&tagL[0], tagL.size()+1);
  // Read the degree.
  Size degreeL; inStreamA >> degreeL;
  if (not inStreamA.good())
  {
    throw std::invalid_argument("Can't read degree from stream.");
  }
  inStreamA.ignore(); // skip endl 

  // -- Read control points and weights. --------------------------------------
  // Read the number of control points.
  Size nL; inStreamA >> nL;
  if (not inStreamA.good())
  {
    throw std::invalid_argument("Can't read number of control points from stream.");
  }
  // Create the control polygon.
  std::vector<Point> controlPointsL(nL, {0, 0, 0});
  // Read the control point tag.
  tagL = NURBS_IO_POLYGON_TAG;
  inStreamA.get(&tagL[0], tagL.size()+1);
  // Read all control points and weights.
  for (Point& pointL : controlPointsL)
  {
    // Read the initial charactres of the control point string.
    tagL = NURBS_IO_CP_START;
    inStreamA.get(&tagL[0], tagL.size()+1);
    // Read the point's x-coordinate.
    inStreamA >> pointL[0];
    if (not inStreamA.good())
    {
      throw std::invalid_argument("Can't read x-coordinate from stream.");
    }

    // Read the delimiting charactres of the control point string.
    tagL = NURBS_IO_CP_XY_DELIM;
    inStreamA.get(&tagL[0], tagL.size()+1);
    // Read the point's y-coordinate.
    inStreamA >> pointL[1];
    if (not inStreamA.good())
    {
      throw std::invalid_argument("Can't read y-coordinate from stream.");
    }

    // Read the delimiting charactres of the control point string.
    tagL = NURBS_IO_CP_WEIGHT_DELIM;
    inStreamA.get(&tagL[0], tagL.size()+1);
    // Read the point's weight.
    inStreamA >> pointL[2];
    if (not inStreamA.good())
    {
      throw std::invalid_argument("Can't read weight from stream.");
    }

    // Read the charactres concluding the control point string.
    tagL = NURBS_IO_CP_END;
    inStreamA.get(&tagL[0], tagL.size()+1);

    // Do the mapping to homogeneous coordinates manually.
    pointL[0] *= pointL[2];
    pointL[1] *= pointL[2];

  }
  inStreamA.ignore(); // skip endl 

  // -- Read knots. -----------------------------------------------------------
  // Read the number of knots.
  inStreamA >> nL;
  if (not inStreamA.good())
  {
    throw std::invalid_argument("Can't read number of knots from stream.");
  }
  // Check whether the stored NURBS curve is periodic.
  bool periodicL;
  if (nL == (controlPointsL.size() + 1))
  {
    periodicL = true;
  }
  else if (nL == (controlPointsL.size() + degreeL + 1))
  {
    periodicL = false;
  }
  else
  {
    throw std::invalid_argument("Sizes of control polygon and knot vector are incompatible.");
  }

  // Resize the knot vector.
  std::vector<Real> knotsL(nL);
  // Read the knots tag.
  tagL = NURBS_IO_KNOTS_TAG;
  inStreamA.get(&tagL[0], tagL.size()+1);
  // Read all knots
  for (Size iL = 0; iL < nL; ++iL)
  {
    // Read the current knot
    inStreamA >> knotsL[iL];
    if (not inStreamA.good())
    {
      throw std::invalid_argument("Can't read enough knots from stream.");
    }
    // Read the knot delimiter. Note that trying to read the non-existing
    // delimiter beyond the last knot does not hurt.
    tagL = NURBS_IO_KNOTS_DELIM; //todo: do this before the loop
    inStreamA.get(&tagL[0], tagL.size()+1);
  }
  inStreamA.ignore(); // skip endl 

  // Update this NURBS curve with the read parameters.
  degreeE = degreeL;
  controlPointsE = std::move(controlPointsL);
  knotsE = std::move(knotsL);
  periodicE = periodicL;
}

//#undef NURBS_IO_NAME_TAG
#undef NURBS_IO_DEGREE_TAG
#undef NURBS_IO_POLYGON_TAG
#undef NURBS_IO_CP_START
#undef NURBS_IO_CP_XY_DELIM
#undef NURBS_IO_CP_WEIGHT_DELIM
#undef NURBS_IO_CP_END
#undef NURBS_IO_KNOTS_TAG
#undef NURBS_IO_KNOTS_DELIM

//! Writes this NURBS curve to file.
void
MoGES::NURBS::Curve
::
write
(
  const std::string& filenameA
) const
{
  OutFileStream fileStreamL(filenameA);

  if (fileStreamL.is_open())
  {
    write(fileStreamL);
    if (not fileStreamL.good())
    {
      std::cout << "Warning: Some problem occured while writing NURBS::Curve "
                << "to file " << filenameA << ".\n";
    }
    fileStreamL.close();
  }
  else
  {
    throw std::invalid_argument("Specified string does not refer to a readable file.");
  }
}


//! Reads this NURBS curve from file.
void
MoGES::NURBS::Curve
::
read
(
  const std::string& filenameA
)
{
  InFileStream fileStreamL(filenameA);

  if (fileStreamL.is_open())
  {
    read(fileStreamL);
    fileStreamL.close();
  }
  else
  {
    throw std::invalid_argument("Specified string does not refer to a writable file.");
  }
}



// -- Method -- math base -------------------------------------------------- //

//! Maps curve parameter s to homogeneous coordinates (point on the curve).
/*
MoGES::Point
MoGES::NURBS::Curve
::
value
(
  Real sA
) const
{
  // Skip basis functions that must be zero.
  // Find the knot holding the upper limit corresponding to sA.
  Size maxL = upperLimitIndex(sA);
  //todo: These border cases don't occur for valid sA, I think.
  Size minL = (maxL > degreeE) ? maxL - degreeE - 1 : 0;
  if (maxL > numberOfControlPoints())
  {
    std::cout << "\n\nOH, it can happen!\n\n";
    maxL = numberOfControlPoints(); // N(i,k,s) = 0, for k = degreeE and i > n.
  }

  Point valueL = {0, 0, 0};
  for (Size iL = minL; iL < maxL; ++iL)
  {
    valueL += controlPointsE.at(index(iL)) * basisFunction(iL, degreeE, sA);
  }

  return valueL;
}
*/

//! Maps curve parameter s to homogeneous coordinates (point on the curve).
MoGES::Point
MoGES::NURBS::Curve
::
homogeneous
(
  Real sA
) const
{
  std::vector<Real> basisFunctionsL;
  Size indexL = basisFunctions(sA, basisFunctionsL);

  Point coordinatesL = {0, 0, 0};
  for (Real basisFunL : basisFunctionsL)
  {
    coordinatesL += controlPointsE[index(indexL)] * basisFunL;
    ++indexL;
  }

  return coordinatesL;
}



//! Calculate dAth derivative of this curve at s = sA in homogeneous coordinates.
MoGES::Point
MoGES::NURBS::Curve
::
derivative
(
  Real sA,
  Size dA
) const
{
  Point derivativeL = {0, 0, 0};

  for (Size iL = 0; iL < numberOfControlPoints(); ++iL)
  {
    derivativeL += controlPointsE.at(index(iL)) * derivativeOfBasisFunction(iL, degreeE, sA, dA);
  }

  return derivativeL;
}


MoGES::Real
MoGES::NURBS::Curve
::
basisFunction
(
  Size iA,
  Size degreeA,
  Real sA
) const
{
  if (degreeA == 0) // end recursion
  {
    if ((sA >= knot(iA)) && (sA < knot(iA+1))) return 1.0;
    else return 0.0;
  }
  else
  {
    Real pole0L = basisFunction(iA, degreeA-1, sA)  * (sA - knot(iA));
    // This prevents dividing by zero implicitly.
    if (std::abs(pole0L) > EPSILON)  pole0L /= (knot(iA + degreeA) - knot(iA));

    Real pole1L = basisFunction(iA+1, degreeA-1, sA) * (knot(iA + degreeA + 1) - sA);
    if (std::abs(pole1L) > EPSILON)  pole1L /= (knot(iA + degreeA + 1) - knot(iA + 1));

    return pole0L + pole1L;
  }
}

MoGES::Size
MoGES::NURBS::Curve
::
basisFunctions
(
  Real sA,
  std::vector<Real>& basisFunctionsA
) const
{
  // Skip basis functions that must be zero.
  // Find the knot holding the upper limit corresponding to sA.
  Size maxL = upperLimitIndex(sA);
  //todo: This border case (maxL <= degreeE) doesn't occur for valid sA, I think.
  Size minL = (maxL > degreeE) ? maxL - degreeE - 1 : 0;
  //todo?: if (sA == endOfValidRange()) ++minL
  if (maxL > numberOfControlPoints()) //todo: this should not occur anymore
  {
    std::cout << "OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOh\n";
    std::cout << "OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOh\n";
    maxL = numberOfControlPoints();
  }

  // Initialize with the basis functions of degree 0.
  basisFunctionsA.resize(maxL - minL + 1); // with an extra 0 at the end
  for (Real& basisFunctionL : basisFunctionsA)
  {
    basisFunctionL = 0.0;
  }
  *(basisFunctionsA.end()-2) = 1.0;

  for (Size degreeL = 1; degreeL <= degreeE; ++degreeL)
  {
    for (Size iL = 0; iL < basisFunctionsA.size()-1; ++iL)
    {
      Size indexL = minL + iL;
      Real denominatorL = knot(indexL + degreeL) - knot(indexL);
      if (denominatorL < EPSILON)
      {
        //todo: Setting to zero should not be necessary, because it was zero already.
        basisFunctionsA[iL] = 0.0;
      }
      else
      {
        basisFunctionsA[iL] = basisFunctionsA[iL] * (sA - knot(indexL)) / denominatorL;
      }
      denominatorL = knot(indexL + degreeL + 1) - knot(indexL + 1);
      if (denominatorL >= EPSILON)
      {
        basisFunctionsA[iL] += basisFunctionsA[iL+1] * (knot(indexL + degreeL + 1) - sA) / denominatorL;
      }
    }
  }

  // Remove the extra element from the vector.
  basisFunctionsA.pop_back();

  // Return the index of the first control point with non-zero basis function.
  return minL;
}


MoGES::Real
MoGES::NURBS::Curve
::
derivativeOfBasisFunction
(
  Size iA,
  Size degreeA,
  Real sA,
  Size dA
) const
{
  if (dA == 1) // End recursion.
  {
    Real basis_0L = basisFunction(iA,degreeA-1,sA);
    if (basis_0L > EPSILON) basis_0L /= (knot(iA+degreeA) - knot(iA));

    Real basis_1L = basisFunction(iA+1,degreeA-1,sA);
    if (basis_1L > EPSILON) basis_1L /= (knot(iA+degreeA+1) - knot(iA+1));

    return (basis_0L - basis_1L) * static_cast<Real>(degreeA);
  }
  else //if (dA > 1)
  {
    Real basis_0L = derivativeOfBasisFunction(iA,degreeA-1,sA,dA-1);
    if (basis_0L > EPSILON) basis_0L /= (knot(iA+degreeA) - knot(iA));

    Real basis_1L = derivativeOfBasisFunction(iA+1,degreeA-1,sA,dA-1);
    if (basis_1L > EPSILON) basis_1L /= (knot(iA+degreeA+1) - knot(iA+1));

    return (basis_0L - basis_1L) * static_cast<Real>(degreeA);
  }
}


// -- Method -- help ------------------------------------------------------- //

//! Handles the periodic property when accessing control points and weights.
MoGES::Size
MoGES::NURBS::Curve
::
index
(
  Size iA
) const
{
  if (periodicE && (iA < numberOfControlPoints()))
  {
    return iA % controlPointsE.size();
  }
  else
  {
    return iA;
  }
}


//! Maps a 2D point and its weight to homogeneous coordinates in 3D.
MoGES::Point
MoGES::NURBS::Curve
::
map
(
  const Point& p2dA,
  Real weightA
)
{
  return {p2dA[0] * weightA, p2dA[1] * weightA, weightA};
}

//! Maps homogeneous coordinates in 3D to 2D.
MoGES::Point
MoGES::NURBS::Curve
::
map
(
  const Point& p3dA
)
{
  //todo?: If the weight is zero, return the direction of (x,y) (Wilfried Horn's Dissertation).
  if (p3dA[2] < EPSILON)
  {
    return {p3dA[0] / EPSILON, p3dA[1] / EPSILON};
  }
  else
  {
    // Project the point from homogeneous to cartesian coordinates.
    return {p3dA[0] / p3dA[2], p3dA[1] / p3dA[2] };
  }
}


//! Thorws an exception if sA is not in the valid range for curve parameter s.
void
MoGES::NURBS::Curve
::
throwIfNotInRange
(
  Real sA
) const
{
  // Check if sA is on the curve.
  if ((sA < beginOfValidRange()) || (sA > endOfValidRange()))
  {
    // Throw an exception.
    std::stringstream whatL;
    whatL << "Value of curve parameter s = " << sA
          << " is outside of the valid range ["
          << beginOfValidRange() << ", " << endOfValidRange() << "[.";

    throw std::out_of_range(whatL.str());
  }
}


//! Find that bin of the knot vector belonging to the smallest upper bound for sA.
MoGES::Size
MoGES::NURBS::Curve
::
upperLimitIndex
(
  Real sA
) const
{
#ifndef MOGES_DISABLE_CHECKS
  throwIfNotInRange(sA);
#endif //ndef MOGES_DISABLE_CHECKS

  // The definition of a NURBS curve is somewhat paradox. Though the upper
  // limit of a knot span is exclusive when evaluating the basis functions,
  // the curve is defined over the entire span including the upper limit.
  if (sA == endOfValidRange())  return numberOfControlPoints();

//todo: all non-periodic NURBS should be clamped -> always start at degreeE
  Size iCheckL = periodicE  ?  degreeE  :  0;
  while (sA >= knot(++iCheckL)); // Yes, it's supposed to be like that. todo: speed this up by exploiting the fact that knotsE is sorted

  Size iL = binarySearch<Size>
            ( degreeE,//todo: +1?
              numberOfControlPoints(),
              [&] (Size iA) { return sA < knot(iA); } //todo: search directly on knotE and add degreeE afterwards
            );

  if (iL != iCheckL) std::cout << "\nNooooo... binarySearch does not work!!!\n" << iL << " != " << iCheckL << "\n";

  return iL;
}


//! The recursive function for discretizing splits a segment of the
//! curve in smaller segments until begin and end of the range evaluate
//! to the same (integral) point.
void
MoGES::NURBS::Curve
::
discretizeRecursively
(
  DiscreteCurvePtr curveA,
  Real sMinA,
  const IntPoint& posMinA,
  Real sMaxA,
  const IntPoint& posMaxA
) const
{
  if ((std::abs(posMaxA[0] - posMinA[0]) < 2) && (std::abs(posMaxA[1] - posMinA[1]) < 2))
  {
    if (curveA->empty() || (squaredNorm<Integer>(posMinA - curveA->rbegin()->second) > 0))
    {
      curveA->insert
      (
        curveA->end(), // hint
        std::make_pair(sMinA, posMinA)
      );
    }
    if (curveA->empty() || (squaredNorm<Integer>(posMaxA - curveA->rbegin()->second) > 0))
    {
      curveA->insert
      (
        curveA->end(), // hint
        std::make_pair(sMaxA, posMaxA)
      );
    }
  }
  else if (std::abs(sMaxA - sMinA) < EPSILON)
  {
    // todo: check if the segment is a straight line (see legth calculation in hornDiss - use his condition here - it should be a straight line)
    Real distanceL = std::sqrt(squaredNorm<Integer>(posMaxA - posMinA));
    std::cout << "Between (" << posMinA[0] << ", " << posMinA[1] << ") and (" 
      << posMaxA[0] << ", " << posMaxA[1] << ") is a gap of about " << distanceL
      << " pixels in the curve!\n";
    // Okay, since we can't get a pixel precise discretization, just force end of recursion (and store both points).
    discretizeRecursively(curveA, sMinA, posMinA, sMinA, posMinA);
    discretizeRecursively(curveA, sMaxA, posMaxA, sMaxA, posMaxA);

  }
  else // The segment needs to be split in smaller pieces.
  {
    Real sHalfL = (sMinA + sMaxA) / static_cast<Real>(2.0);
    IntPoint posHalfL = round((*this)(sHalfL));

    discretizeRecursively(curveA, sMinA, posMinA, sHalfL, posHalfL);
    discretizeRecursively(curveA, sHalfL, posHalfL, sMaxA, posMaxA);
  }
}
