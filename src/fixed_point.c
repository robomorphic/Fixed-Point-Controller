// try adding two fixed-point numbers using clang extension _Fract

// _Fract has a sign bit and the rest is the fractional part
// meaning it is meant to be used for numbers between -1 and 1

// _Accum has an integral part, too

// these types have the variations short and long, try it!

// 

#include <stdio.h>


int main() {
    _Fract a = 0.5;
    _Fract b = 0.25;
    _Fract c = a + b;
    printf("a: %f\n", (float)a);
    printf("b: %f\n", (float)b);
    printf("c: %f\n", (float)c);
    // print the size of fract
    printf("size of _Fract: %lu\n", sizeof(_Fract));
    printf("size of _Fract short: %lu\n", sizeof(_Fract short));
    printf("size of _Fract long: %lu\n", sizeof(_Fract long));
    printf("size of _Accum: %lu\n", sizeof(_Accum));
    printf("size of _Accum short: %lu\n", sizeof(_Accum short));
    printf("size of _Accum long: %lu\n", sizeof(_Accum long));

    return 0;
}



