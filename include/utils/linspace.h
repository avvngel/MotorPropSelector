#pragma once

namespace utils {

template <class T>
LinspaceCompatible = DblScalable && AdditiveUnit

template <LinspaceCompatible NumType, class OutputIt>
void linspace_half_open(
    NumType start, 
    NumType stop, 
    size_t n_samples,
    OutputIt& output_it
  ){

  if (!n_samples){ return; }

  NumType step = (stop - start)/static_cast<double>(n_samples);

  for (size_t i{}; i < n_samples; ++i){
    *output_it = start + i*step;
    ++output_it;
  }

}

} // namespace utils
