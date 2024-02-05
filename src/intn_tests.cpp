#include <iostream>
#include <iomanip>

//#include <fpm/fixed.hpp>
//#include <fpm/ios.hpp>

#include <Eigen/Dense>

#include <bitset>

#include <FixedPoint/fixed_point.hpp>

#include <Eigen/Core>

template<int INT_BITS, int FRAC_BITS> struct Eigen::NumTraits<FixedPoint<INT_BITS, FRAC_BITS>>
 : NumTraits<int> // permits to get the epsilon, dummy_precision, lowest, highest functions
{
  typedef FixedPoint<INT_BITS, FRAC_BITS> Integer;

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
 



typedef FixedPoint<5, 4> IntN;

int main() {
    IntN a = 1.5;
    IntN b = 2.5;
    IntN c = a + b;
    std::cout << c << std::endl;

    IntN d = a * b;
    std::cout << d << std::endl;

    Eigen::Matrix<IntN, 2, 2> m;
    m << 1, 2, 3, 4;
    std::cout << m << std::endl;

    auto mm = m * m;
    std::cout << mm << std::endl;

    return 0;

}





/*
//using fixed_4_4 = fpm::fixed<int8_t, int32_t, 4>;
using fixed_4_4 = fpm::fixed<IntN, int32_t, 4>;

int main(){
    fixed_4_4 a {1.5};
    fixed_4_4 b {2.5};
    fixed_4_4 c = a + b;
    std::cout << c << std::endl;
    
    
    Eigen::Matrix<IntN, 2, 2> m;
    std::cout << "allocating" << std::endl;
    m << 1, 2, 3, 4;
    std::cout << "allocated" << std::endl;
    std::cout << m << std::endl;

    auto mm = m * m;
    std::cout << mm << std::endl;
    
    
    Eigen::Matrix<fixed_4_4, 2, 2> m;
    m << (fixed_4_4)1, (fixed_4_4)2, (fixed_4_4)3, (fixed_4_4)4;
    std::cout << m << std::endl;

    Eigen::Matrix<fixed_4_4, 2, 2> n;
    n << (fixed_4_4)1, (fixed_4_4)2, (fixed_4_4)3, (fixed_4_4)4;
    std::cout << n << std::endl;

    std::cout << m + n << std::endl;
    

    return 0;
}


*/