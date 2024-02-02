#include <iostream>
#include <iomanip>

#include <fpm/fixed.hpp>
#include <fpm/ios.hpp>

#include <Eigen/Dense>

#include <bitset>

#include <int_vary.hpp>

#include <math/wide_integer/uintwide_t.h>

using int24_t = math::wide_integer::uintwide_t<static_cast<size_t>(UINT32_C(24)), std::uint8_t, void, true>;

//const int INT24_MAX = 8388607;

// the first type is the actual storage for our fixed point number
// the second type is the type used for intermediate calculations, so that we can avoid overflows
// the third type is the number of fractional bits
using fixed_8_8 = fpm::fixed<std::int16_t, std::int32_t, 8>;
using fixed_24_8 = fpm::fixed<std::int32_t, std::int64_t, 8>;
using fixed_16_8 = fpm::fixed<int24_t, std::uint32_t, 8>;

typedef int24_t curr_type;

void simple_add_test(){
    // implicit conversion from double to fixed_8_8 is not allowed to reduce bugs
    //fixed_8_8 a = 1.5; ERROR
    curr_type a(1.5);
    curr_type b(2.5);
    curr_type c = a + b;
    std::cout << c << " " << a << " " << b << std::endl;
    return;
}

int main() {
    simple_add_test();
    Eigen::Matrix<curr_type, 2, 2> A;    
    // ugly :(
    A << (curr_type)1.5, (curr_type)2.5, (curr_type)3.5, (curr_type)4.5;
    auto A2 = A * A;
    auto A3 = A2 * A;
    auto A4 = A3 * A;
    auto A5 = A4 * A;
    auto A6 = A5 * A;
    
    Eigen::Matrix<double, 2, 2> B;
    B << 1.5, 2.5, 3.5, 4.5;
    auto B2 = B * B;
    auto B3 = B2 * B;
    auto B4 = B3 * B;
    auto B5 = B4 * B;
    auto B6 = B5 * B;

    std::cout << "A: " << std::endl << A << std::endl;
    std::cout << "A2: " << std::endl << A2 << std::endl;
    std::cout << "A3: " << std::endl << A3 << std::endl;
    std::cout << "A4: " << std::endl << A4 << std::endl;
    std::cout << "A5: " << std::endl << A5 << std::endl;
    std::cout << "A6: " << std::endl << A6 << std::endl;

    std::cout << "B: " << std::endl << B << std::endl;
    std::cout << "B2: " << std::endl << B2 << std::endl;
    std::cout << "B3: " << std::endl << B3 << std::endl;
    std::cout << "B4: " << std::endl << B4 << std::endl;
    std::cout << "B5: " << std::endl << B5 << std::endl;
    std::cout << "B6: " << std::endl << B6 << std::endl;
    



    return 0;
}




