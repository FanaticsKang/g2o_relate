#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
int main() {
    plt::plot({1,3,2,4});
    plt::pause(2); //最好加上该句，否则有时候显示不了图像，或者图像显示很慢
    plt::show();  
}