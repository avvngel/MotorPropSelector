# pragma once

struct ScanRes{ 
  bool found;
  cv::Point pt;
};

using ScanResults = std::vector<ScanRes>;

