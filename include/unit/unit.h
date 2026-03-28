#pragma once

namespace unit {


template <class Tag, class Rep>
struct Unit {
  using TagT = Tag;
  using RepT = Rep;

  Rep val; 

  // Constructors
  constexpr explicit Unit()         : val( Rep{} ){  } 
  constexpr explicit Unit(Rep val_) : val(val_){  } 
  constexpr Unit(const Unit& other) : val(other.val){  } 

  // In-place Arithmetic Operations
  constexpr Unit& operator+=(const Unit& other){
    val += other.val;
    return *this;
  }

  constexpr Unit& operator-=(const Unit& other){
    val -= other.val;
    return *this;
  }
  
  constexpr Unit& operator*=(const double scalar){
    val *= scalar;
    return *this;
  }

  constexpr Unit& operator/=(const double scalar){
    val /= scalar;
    return *this;
  }

};

// Arithmetic Operations

template <class Tag, class Rep>
constexpr Unit<Tag, Rep> operator+(
  Unit<Tag, Rep> left,
  const Unit<Tag, Rep>& right
){
  return left += right;
}


template <class Tag, class Rep>
constexpr Unit<Tag, Rep> operator-(
  Unit<Tag, Rep> left, 
  const Unit<Tag, Rep>& right
){
  return left -= right;
}


template <class Tag, class Rep>
constexpr Unit<Tag, Rep> operator*(
  Unit<Tag, Rep> unit,
  double scalar
){
  return unit *= scalar;
}

template <class Tag, class Rep>
constexpr Unit<Tag, Rep> operator*(
  double scalar,
  Unit<Tag, Rep> unit
){
  return unit *= scalar;
}

template <class Tag, class Rep>
constexpr Unit<Tag, Rep> operator/(
  Unit<Tag, Rep> unit, 
  double scalar
){
  return unit /= scalar;
}

// Comparison Operators

template <class Tag, class Rep>
constexpr bool operator<(
  const Unit<Tag, Rep>& left, 
  const Unit<Tag, Rep>& right
) noexcept{
  return left.val < right.val;
}

template <class Tag, class Rep>
constexpr bool operator<=(
  const Unit<Tag, Rep>& left,
  const Unit<Tag, Rep>& right
) noexcept{
  return left.val <= right.val;
}

template <class Tag, class Rep>
constexpr bool operator>(
  const Unit<Tag, Rep>& left,
  const Unit<Tag, Rep>& right
) noexcept{
  return left.val > right.val;
}

template <class Tag, class Rep>
constexpr bool operator>=(
  const Unit<Tag, Rep>& left,
  const Unit<Tag, Rep>& right
) noexcept{
  return left.val >= right.val;
}

template <class Tag, class Rep>
constexpr bool operator==(
  const Unit<Tag, Rep>& left,
  const Unit<Tag, Rep>& right
) noexcept{
  return left.val == right.val;
}

template <class Tag, class Rep>
constexpr bool operator!=(
  const Unit<Tag, Rep>& left,
  const Unit<Tag, Rep>& right
) noexcept{
  return left.val != right.val;
}

// Concepts

template <class T>
constexpr bool is_unit_impl = false;

template <class Tag, class Rep>
constexpr bool is_unit_impl<Unit<Tag, Rep>> = true;

template <class T>
concept IsUnit = is_unit_impl<std::remove_cvref_t<T>>;

template <class T>
concept IsFPUnit = IsUnit<T> && std::is_floating_point_v<T::RepT>;


} // namespace unit
