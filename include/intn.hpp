#ifndef INT_VARY_H
#define INT_VARY_H
#include <iostream>

// class for integer with N bit size
class IntN
{
public:
    IntN();
    IntN(double value);
    ~IntN();

    long long raw_value() const;

    IntN operator+ (IntN value) const;
    IntN operator* (int value);
    IntN operator/ (int value);
    IntN operator* (IntN value) const;
    IntN operator/ (IntN value);
    IntN operator+= (IntN value);
    IntN operator-= (IntN value);
    IntN operator+= (int value);
    IntN operator-= (int value);
    IntN operator*= (int value);
    IntN operator/= (int value);
    IntN operator*= (IntN value);
    IntN operator/= (IntN value);

    void operator= (unsigned long value);
    void operator= (IntN value);
    operator int();

    // Declare prefix and postfix increment operators.
    IntN &operator++();       // Prefix increment operator.
    IntN operator++(int);     // Postfix increment operator.

    // Declare prefix and postfix decrement operators.
    IntN &operator--();       // Prefix decrement operator.
    IntN operator--(int);     // Postfix decrement operator.

    // Bit shift operators.
    IntN operator<<(int value);
    IntN operator>>(int value);

    unsigned long value();

    friend std::ostream& operator<<(std::ostream& os, const IntN& value);

private:
    void mask_value();
    long long* value_;
    unsigned long mask;
    unsigned long bit_number_;
};


#include <Eigen/Core>
 
namespace Eigen {
 
template<> struct NumTraits<IntN>
 : NumTraits<float> // permits to get the epsilon, dummy_precision, lowest, highest functions
{
  typedef IntN Real;
  typedef IntN NonInteger;

  static inline int digits10() { return 0; }
 
  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 1,
    ReadCost = 1,
    AddCost = 3,
    MulCost = 3
  };
};
 
}


#endif