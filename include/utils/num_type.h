#pragma once

namespace utils {

template <class T>
DblScalable = requires (T a, double b){
  { a * b } -> { std::same_as_v<T> }
  { b * a } -> { std::same_as_v<T> }
  { a / b } -> { std::same_as_v<T> }
};

template <class T>
AdditiveUnit = requires (T a, T b){
  { a + b } -> { std::same_as_v<T> }
  { a - b } -> { std::same_as_v<T> }
};


} // namespace utils
