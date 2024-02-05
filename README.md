I needed to change the line 
/opt/homebrew/include/eigen3/Eigen/Core/MathFunctions.h line 1372 from
template<typename T> EIGEN_DEVICE_FUNC bool (isfinite)(const T &x) { return internal::isfinite_impl(x); }
to
template<typename T> EIGEN_DEVICE_FUNC bool (isfinite)(const T &x) { return true; }

