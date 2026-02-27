#pragma once

struct Radian {
  double val; 

  Radian& operator+=(const Radian& other){
    val += other.val;
    return *this;
  }

  Radian& operator-=(const Radian& other){
    val -= other.val;
    return *this;
  }
  
  Radian& operator*=(const double scalar){
    val *= scalar;
    return *this;
  }

  Radian& operator/=(const double scalar){
    val /= scalar;
    return *this;
  }

};

// Arithmetic Operators

constexpr Radian operator+(Radian left, const Radian& right) noexcept {
  return left += right;
}

constexpr Radian operator-(Radian left, const Radian& right) noexcept {
  return left -= right;
}

constexpr Radian operator*(Radian radian, double scalar) noexcept {
  return radian *= scalar;
}

constexpr Radian operator*(double scalar, const Radian& radian) noexcept{
  return radian*scalar;
}

constexpr Radian operator/(Radian radian, double scalar) noexcept{
  return radian /= scalar;
}

// Comparison Operators

constexpr Radian operator<(const Radian& left, const Radian& right) noexcept{
  return left.val < right.val;
}

constexpr Radian operator<=(const Radian& left, const Radian& right) noexcept{
  return left.val <= right.val;
}

constexpr Radian operator>(const Radian& left, const Radian& right) noexcept{
  return left.val > right.val;
}

constexpr Radian operator>=(const Radian& left, const Radian& right) noexcept{
  return left.val >= right.val;
}
