#ifndef MOGES_TYPES_H
#define MOGES_TYPES_H

#include <cstddef>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <valarray>

//! This macro defines a smart pointers as std::shared_ptr and supplies two
//! convenient factories.
#define SHARED_PTR_TO(_CLASS_)                                      \
  typedef std::shared_ptr<_CLASS_> _CLASS_##Ptr;                    \
  typedef std::shared_ptr<const _CLASS_> const_##_CLASS_##Ptr;      \
                                                                    \
  template <class... Args>                                          \
  _CLASS_##Ptr new_##_CLASS_(Args && ... argsA)                     \
  {                                                                 \
    return std::make_shared<_CLASS_>(std::forward<Args>(argsA)...); \
  }                                                                 \
                                                                    \
  inline                                                            \
  _CLASS_##Ptr clone (const_##_CLASS_##Ptr otherA)                  \
  {                                                                 \
    return new_##_CLASS_(*otherA);                                  \
  }


namespace MoGES
{
  // Types

  typedef std::size_t Size;
  typedef int Integer;
  typedef double Real;
  typedef char Character;

  typedef std::basic_string<Character> String;
  typedef std::basic_istream<Character> InStream;
  typedef std::basic_ostream<Character> OutStream;
  typedef std::basic_ifstream<Character> InFileStream;
  typedef std::basic_ofstream<Character> OutFileStream;

  typedef std::valarray<Real> Point;
  typedef std::valarray<Integer> IntPoint;


  // Constants

  const Real EPSILON = 1.0e-7;
  const Real REAL_HIGH = std::numeric_limits<Real>::max();
  const Real REAL_LOW = std::numeric_limits<Real>::lowest();
  const Real SQRT2 = std::sqrt(static_cast<Real>(2));
  const Real PI = M_PI;
}

#endif //ndef MOGES_TYPES_H
