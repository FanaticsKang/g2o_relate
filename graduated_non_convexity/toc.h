//
// Created by ubuntu on 18-10-23.
//

#ifndef ORB_SLAM2_TOC_H
#define ORB_SLAM2_TOC_H

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>

class TicToc {
 public:
  TicToc() { 
    tic(); 
  }

  void tic() { 
    start = std::chrono::system_clock::now(); 
  }

  double toc() const {
    std::chrono::time_point<std::chrono::system_clock> end =
        std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

  friend std::ostream& operator<<(std::ostream& os, const TicToc& tic) {
    return os << "Time Costs: " << tic.toc() << " ms";
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start;
};

#endif  // ORB_SLAM2_TOC_H
