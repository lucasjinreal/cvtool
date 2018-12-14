#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "librealsense2/rs.hpp"

#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int main() {

    rs2::pipeline pipe;
    rs2::config cfg;
    pipe.start();
    cout << "realsense initialized.\n";

    // contains all to calib image path
    int numBoards = 0;
    int numCornersHor;
    int numCornersVer;

    printf("Enter number of corners along width: ");
    scanf("%d", &numCornersHor);

    printf("Enter number of corners along height: ");
    scanf("%d", &numCornersVer);

    printf("Enter number of boards (20): ");
    scanf("%d", &numBoards);

    int numSquares = numCornersHor * numCornersVer;
    Size board_sz = Size(numCornersHor, numCornersVer);


    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> image_points;

    vector<Point2f> corners;
    int successes=0;

    Mat image;
    Mat gray_image;
    rs2::frameset data = pipe.wait_for_frames();
    rs2::frame color = data.get_color_frame();
    const int w = color.as<rs2::video_frame>().get_width();
    const int h = color.as<rs2::video_frame>().get_height();

    cv::Mat color_image(Size(w, h), CV_8UC3, (void*) color.get_data(), Mat::AUTO_STEP);
    cv::cvtColor(color_image, image, cv::COLOR_BGR2RGB);
//    cv::cvtColor(image, gray_image, cv::COLOR_RGB2GRAY);


    vector<Point3f> obj;
    for(int j=0;j<numSquares;j++) {
        obj.push_back(Point3f(j / numCornersHor, j%numCornersHor, 0.0f));
    }

    while (successes < numBoards) {
        cvtColor(image, gray_image, CV_BGR2GRAY);
        bool found = findChessboardCorners(image, board_sz, corners,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        if(found)
        {
            // the 11,11 comes from 5,5
            cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1),
                    TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray_image, board_sz, corners, found);
        }


        imshow("win1", image);
        imshow("win2", gray_image);

//        capture >> image;
        color = data.get_color_frame();
        Mat color_image_(Size(w, h), CV_8UC3, (void*) color.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(color_image_, image, cv::COLOR_BGR2RGB);

        int key = waitKey(1);

        if(key==27)
            return 0;

        if(key==' ' && found!=0)
        {
            image_points.push_back(corners);
            object_points.push_back(obj);

            printf("Snap stored!");
            successes++;

            if(successes>=numBoards)
                break;
        }
    }

    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;

    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;

    calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

    cout << "calibration result: " << intrinsic << endl;

//    Mat imageUndistorted;
//    while(1)
//    {
//        capture >> image;
//        undistort(image, imageUndistorted, intrinsic, distCoeffs);
//
//        imshow("win1", image);
//        imshow("win2", imageUndistorted);
//        waitKey(1);
//    }

    pipe.stop();

    return 0;

}
