#ifndef MOGES_NURBS_CURVE_H
#define MOGES_NURBS_CURVE_H

#include "Types.h"

#include <exception>
#include <functional>
#include <map>
#include <vector>


namespace MoGES
{
  namespace NURBS
  {
    //! Discrete version of a curve, its pointers, and a factory to make them.
    typedef std::map<Real, IntPoint> DiscreteCurve;
    SHARED_PTR_TO(DiscreteCurve)

    class InvalidCurve : public std::exception
    {
      public:
        explicit InvalidCurve (const String& whatA) : whatE(whatA) {} 
        virtual ~InvalidCurve () noexcept {}
        virtual const char* what () const noexcept { return whatE.c_str(); }
      private:
        const String whatE;
    };

    class Curve
    {
      public: // ---------------------------------------------------------------

      // -- Method -- construct, destruct, assign, validate ----------------- //

        //! The default constructor is useless.
        Curve();

        //! Copy constructor resembling the passed NURBS curve.
        Curve (const Curve& otherA);

        //! Move constructor taking over the passed NURBS curve.
        Curve (Curve&& otherA);

        //! Explicit constructor completely defining the curve.
        Curve
        (
          Size degreeA, // degree of the curve
          const std::vector<Point>& controlPointsA, // initial control points
          const std::vector<Real>& weightsA,
          const std::vector<Real>& knotsA,
          bool periodicA = false
        );

        //! Very explicit constructor completely defining the NURBS curve.
        Curve
        (
          Size degreeA,
          const std::vector<Point>& controlPointsA,
          const std::vector<Real>& knotsA,
          bool periodicA
        );

        //! Very explicit constructor completely defining the NURBS curve;
        //! moving control points and knots (instead of copying).
        Curve
        (
          Size degreeA,
          std::vector<Point>&& controlPointsA,
          std::vector<Real>&& knotsA,
          bool periodicA
        );

        //! Destroy the NURBS curve.
        virtual ~Curve();

        //! Assigns (copies) another curve to this curve.
        Curve& operator= (const Curve& otherA) = default;
        //! Assigns (moves) another curve to this curve.
        Curve& operator= (Curve&& otherA) = default;

        //! Returns true, iff this is a valid NURBS curve.
        bool isValid (bool controlPointsMustBeSorted = false) const;
        //! Throws an exception, iff this is not a valid NURBS curve.
        void throwIfInvalid (bool controlPointsMustBeSortedA = false) const;


      // -- Method -- evaluate, derive, discretize, traverse, sample ------- //

        //! Evaluate this curve at s = sA (map curve parameter s to point on
        //! the curve in cartesian coordinates).
        Point operator() (Real sA) const;

        //! Get derivative of this curve at s = sA.
        Point operator() (Size derivativeL, Real sA) const;

        //! Calculates the unit normal vector at s.
        Point unitNormalVector (Real sA) const;

        //! Get a discrete version of the curve.
        DiscreteCurvePtr discretize () const;

        //! Get a discrete version of a segment of the curve.
        DiscreteCurvePtr discretize (Real sMinA, Real sMaxA) const;

        //! Call doA with each point on a discrete version of this curve.
        Size traverse
        (
          std::function<void (const IntPoint&)> doA
        ) const;

        //! Call doA with each point on a discretized segment of this curve.
        Size traverse
        (
          std::function<void (const IntPoint&)> doA,
          Real sMinA,
          Real sMaxA
        ) const;

        //! Take samplesA samples (evenly distributed over curve parameter s) and
        //! execute doA for each sampled cartesian point on the curve.
        void sample
        (
          std::function<void (const Point&)> doA,
          Size samplesA
        ) const;


      // -- Method -- get, set, modify ------------------------------------- //

        //! Reading access to degree of the curve.
        Size degree () const;

        //! Reading access to number of control points n.
        //! For periodic curves first and last degreeE control points are identical.
        Size numberOfControlPoints () const;
        //! Reading access to number of explicitly stored control points.
        Size numberOfExplicitControlPoints () const;

        //! Reading access to the number of knots.
        Size numberOfKnots () const;

        //! Returns true, iff the NURBS curve is periodic (closed).
        bool periodic() const;

        //! Mapped reading access to the curve's control points. Handles periodic property.
        Point controlPoint (Size iA) const;
        //! Mapped writing access to the curve's control points. Handles periodic property.
        void controlPoint (Size iA, const Point& p_iA);

        //! Mapped reading access to the control points' weights. Handles periodic property.
        Real weight (Size iA) const;
        //! Mapped writing access to the control points' weights. Handles periodic property.
        void weight (Size iA, Real w_iA);

        //! Reading access to this NURBS' knots.
        Real knot (Size iA) const;
        //! Writing access to this NURBS' knots.
        void knot (Size iA, Real s_iA);

        //! Minimum of the valid range of curve parameter s (inclusive).
        Real beginOfValidRange () const;
        //! Maximum of the valid range of curve parameter s (exclusive).
        Real endOfValidRange () const;

        //! Get the value range of curve parameter s referring to the segment of
        //! the curve which is influenced by control point iA.
        Point reachOfControlPoint (Size iA) const;

        //! Insert a new knot without changing the curves shape.
        void insertKnot (Real knotA);

        //! Translates the curve.
        void translate (const Point& offsetA);
        //! Scales the curve.
        void scale (Real factorA, const Point& centerA = {0, 0});
        //! Rotates the curve.
        void rotate (Real angleA, const Point& centerA = {0, 0});


      // -- Method -- output ----------------------------------------------- //

        //! Writes this NURBS curve to an output stream.
        void write (OutStream& outStreamA) const;
        //! Reads a NURBS curve from an input stream.
        void read (InStream& inStreamA);

        //! Writes this NURBS curve to file.
        void write (const String& filenameA) const;
        //! Reads this NURBS curve from file.
        void read (const String& filenameA);


      // -- Method -- math base -------------------------------------------- //

        //! Maps curve parameter s to homogeneous coordinates (point on the curve).
        //Point value (Real sA) const;

        Point homogeneous (Real sA) const;

        //! Calculate dAth derivative of this curve at s = sA in homogeneous coordinates.
        Point derivative (Real sA, Size dA) const;

        //! Computes the basis functions of this spline.
        Real basisFunction (Size iA, Size degreeA, Real sA) const;
        Size basisFunctions (Real sA, std::vector<Real>& basisFunctionsA) const;
        //! Computes dAth derivatives of the basis functions of this spline.
        Real derivativeOfBasisFunction (Size iA, Size degreeA, Real sA, Size dA) const;



      private: // -------------------------------------------------------------

      // -- Method -- help ------------------------------------------------- //

        //! Handles the periodic property when accessing control points and weights.
        Size index (Size iA) const;

        //! Maps a 2D point and its weight to homogeneous coordinates in 3D.
        static
        Point map (const Point& p2dA, Real weightA);

        //! Maps homogeneous coordinates in 3D to 2D.
        static
        Point map (const Point& p3dA);

        //! Thorws an exception if sA is not in the valid range for curve parameter s.
        void throwIfNotInRange(Real sA) const;

        //! Find that bin of the knot vector belonging to the smallest upper bound for sA.
        Size upperLimitIndex (Real sA) const;
        
        //! The recursive function for discretizing splits a segment of the
        //! curve in smaller segments until begin and end of the range evaluate
        //! to the same (integral) point.
        void
        discretizeRecursively
        (
          DiscreteCurvePtr curveA,
          Real rangeBeginA,
          const IntPoint& positionBeginA,
          Real rangeEndA,
          const IntPoint& positionEndA
        ) const;


      // -- Property --------------------------------------------------------- //

        //! Degree of this curve. Degree + 1 (= order) control points influence
        //! a single location s on the curve.
        Size degreeE;
        //! The B-spline's n control points in homogeneous coordinates.
        std::vector<Point> controlPointsE;//todo: this is bad: it's a vector of vectors. Either use a mapping or PointPointers
        //! Knots defining n+k spans, which associate curve parameter s with
        //! the degreeE + 1 responsible control points.
        std::vector<Real> knotsE;
        //! Forces periodicity of this NURBS (wrapping of control points and knots).
        bool periodicE;

        //! Tags for reading and writing curves.
        static const String NAME_TAG;
    };

    SHARED_PTR_TO(Curve)
  }
}

#endif //ndef MOGES_NURBS_CURVE_H
