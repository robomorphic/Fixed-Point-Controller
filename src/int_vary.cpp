#include <int_vary.hpp>


Int24::Int24()
{
    mBytes[0] = 0;
    mBytes[1] = 0;
    mBytes[2] = 0;
}

Int24::Int24(unsigned long value)
{
    mBytes[0] = ( value         & 0xff);
    mBytes[1] = ((value >>  8)  & 0xff);
    mBytes[2] = ((value >> 16 ) & 0xff);
}

Int24 Int24::operator+(Int24 value)
{
    Int24 retVal;
    unsigned long myValue;
    unsigned long addValue;

     myValue = this->mBytes[2];
     myValue <<= 8;
     myValue |= this->mBytes[1];
     myValue <<= 8;
     myValue |= this->mBytes[0];

     addValue = value.mBytes[2];
     addValue <<= 8;
     addValue |= value.mBytes[1];
     addValue <<= 8;
     addValue |= value.mBytes[0];

     myValue += addValue;

     retVal = myValue;

     return retVal;
}

Int24 Int24::operator*(int value)
{
    (*this) = (*this).value() * value;
    return (*this);
}

Int24 Int24::operator/(int value)
{
    (*this) = (*this).value() / value;
    return (*this);
}

void Int24::operator=(unsigned long value)
{
    mBytes[0] = ( value         & 0xff);
    mBytes[1] = ((value >>  8)  & 0xff);
    mBytes[2] = ((value >> 16 ) & 0xff);
}

void Int24::operator=(Int24 value)
{
    mBytes[0] = value.mBytes[0];
    mBytes[1] = value.mBytes[1];
    mBytes[2] = value.mBytes[2];
}

Int24 &Int24::operator++()
{
    (*this) = (*this).value() + 1;
    return *this;
}

Int24 Int24::operator++(int)
{
    Int24 temp = (*this);
    ++(*this);
    return temp;
}

Int24 &Int24::operator--()
{
    (*this) = (*this).value() - 1;
    return *this;
}

Int24 Int24::operator--(int)
{
    Int24 temp = (*this);
    --(*this);
    return temp;
}

Int24::operator int() const
{
    return value();
}

unsigned long Int24::value() const
{
    unsigned long retVal;

    retVal = this->mBytes[2];
    retVal <<= 8;
    retVal |= this->mBytes[1];
    retVal <<= 8;
    retVal |= this->mBytes[0];

    return retVal;
}

long long Int24::raw_value() const
{
    long long retVal;

    retVal = this->mBytes[2];
    retVal <<= 8;
    retVal |= this->mBytes[1];
    retVal <<= 8;
    retVal |= this->mBytes[0];

    return retVal;
}