#include <intn.hpp>
#include <stdlib.h>
#include <iostream>

const int N = 4;

IntN::IntN(){
    bit_number_ = N;
    value_ = (long long*)malloc(sizeof(long long));
    mask = 0;
    // mask will be 0000...0001111 for N = 4
    for (int i = 0; i < bit_number_; i++){
        mask = mask << 1;
        mask = mask | 1;
    }
    mask_value();
}


IntN::IntN(double value){
    bit_number_ = N;
    value_ = (long long*)malloc(sizeof(long long));
    mask = 0;
    for (int i = 0; i < bit_number_; i++){
        mask = mask << 1;
        mask = mask | 1;
    }
    *value_ = value;
    mask_value();
}


void IntN::mask_value(){
    *(long long*)value_ = raw_value() & mask;
}


long long IntN::raw_value() const {
    return *value_;
}


IntN IntN::operator+ (IntN value) const {
    //mask_value();
    value.mask_value();
    IntN result(bit_number_);
    result = raw_value() + value.raw_value();
    return result;
}


IntN IntN::operator* (int value) {
    mask_value();
    IntN result(bit_number_);
    result = raw_value() * value;
    result.mask_value();
    return result;
}

IntN IntN::operator* (IntN value) const {
    //mask_value();
    IntN result(bit_number_);
    result = raw_value() * value;
    result.mask_value();
    return result;
}


IntN IntN::operator/ (int value) {
    mask_value();
    IntN result(bit_number_);
    result = raw_value() / value;
    mask_value();
    return result;
}


void IntN::operator= (unsigned long value) {
    *value_ = value;
    mask_value();
}


void IntN::operator= (IntN value) {
    *value_ = value.raw_value();
    mask_value();
}


IntN::operator int() {
    mask_value();
    return (int)raw_value();
}


IntN &IntN::operator++() {
    mask_value();
    *value_ = raw_value() + 1;
    mask_value();
    return *this;
}


IntN IntN::operator++(int) {
    mask_value();
    IntN temp = *this;
    *value_ = raw_value() + 1;
    temp.mask_value();
    return temp;
}


IntN &IntN::operator--() {
    mask_value();
    *value_ = raw_value() - 1;
    mask_value();
    return *this;
}


IntN IntN::operator--(int) {
    mask_value();
    IntN temp = *this;
    *value_ = raw_value() - 1;
    temp.mask_value();
    return temp;
}


IntN IntN::operator<<(int value) {
    mask_value();
    IntN result;
    result = raw_value() << value;
    result.mask_value();
    return result;
}


IntN IntN::operator>>(int value) {
    mask_value();
    IntN result(bit_number_);
    result = raw_value() >> value;
    result.mask_value();
    return result;
}


unsigned long IntN::value() {
    mask_value();
    return (unsigned long)raw_value();
}


std::ostream& operator<<(std::ostream& os, const IntN& value) {
    os << value.raw_value();
    return os;
}


IntN::~IntN() {
    //free(value_);
}


IntN IntN::operator+= (IntN value) {
    mask_value();
    value.mask_value();
    *value_ = raw_value() + value.raw_value();
    mask_value();
    return *this;
}


IntN IntN::operator-= (IntN value) {
    mask_value();
    value.mask_value();
    *value_ = raw_value() - value.raw_value();
    mask_value();
    return *this;
}


IntN IntN::operator+= (int value) {
    mask_value();
    *value_ = raw_value() + value;
    mask_value();
    return *this;
}


IntN IntN::operator-= (int value) {
    mask_value();
    *value_ = raw_value() - value;
    mask_value();
    return *this;
}


IntN IntN::operator*= (int value) {
    mask_value();
    *value_ = raw_value() * value;
    mask_value();
    return *this;
}


IntN IntN::operator/= (int value) {
    mask_value();
    *value_ = raw_value() / value;
    mask_value();
    return *this;
}


IntN IntN::operator*= (IntN value) {
    mask_value();
    value.mask_value();
    *value_ = raw_value() * value.raw_value();
    mask_value();
    return *this;
}


IntN IntN::operator/= (IntN value) {
    mask_value();
    value.mask_value();
    *value_ = raw_value() / value.raw_value();
    mask_value();
    return *this;
}



