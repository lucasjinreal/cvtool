#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int main() {

    // contains all to calib image path
    ifstream fin("calibdata.txt");
    ofstream fout("calib_result.txt");

    cout << "------------------ Start extract contours -------------------\n";
    int image_count = 0;
    Size image_size;
    Size board_size = Size(4 ,6);
}
