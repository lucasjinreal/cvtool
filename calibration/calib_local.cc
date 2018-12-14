#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "thor/os.h"
#include "thor/colors.h"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;
using namespace thor;

void show_usage() {
    cout << "calibration local tool, version 0.1, by Lucas Jin.\n"
        << "\nUsage:\n"
        << "1: chessboard size\n2: every square size\n3: path to images.\n"
        << "calib_local 8x4 0.05 /dir/to/images\n" << colors::reset;
}

int main(int argc, char* argv[]) {
    
    // parse board size from argument
    if (argc < 4) {
        show_usage();
    } else {
        string size_s = string(argv[1]);
        // cout << size_s.substr(0, size_s.find("x")) << endl;
        // cout << size_s.substr(size_s.find("x")+1) << endl;
        double square_w = stod(argv[2]);
        int board_w = stoi(size_s.substr(0, size_s.find("x")));
        int board_h = stoi(size_s.substr(size_s.find("x")+1));
        Size board_size = Size(board_w, board_h);

        string image_dir = string(argv[3]);
        // contains all to calib image path
        ofstream fout("calib_result.txt");

        cout << "board size: " << board_w << "x" << board_h 
            << "\nresult will be saved into: calib_result.txt\n";

        cout << "------------------ Start extract contours -------------------\n";
        int image_count = 0;
        Size image_size;
        
        vector<string> all_image_files;
        all_image_files = thor::os::list_files(image_dir, true);
        
        cout << "find all " << all_image_files.size() << " images.\n";


        // buffer every contours detect on every image
        vector<Point2f> img_points_buffer;
        // buffer all contours detect in all image
        vector<vector<Point2f>> img_points_all;
        for (auto& img_f: all_image_files) {
                
            image_count++;
            cout << img_f << endl;
            Mat img_in = cv::imread(img_f);
            if (image_count == 1) {
                image_size.width = img_in.cols;
                image_size.height = img_in.rows;
                cout << "image width: " << image_size.width << " image height: " << image_size.height << endl;
            }

            if(0 == cv::findChessboardCorners(img_in, board_size, img_points_buffer)) {
                cout << "can not find chessboard corners! image file: " << img_f << endl;
            } else {
                Mat view_grey;
                cv::cvtColor(img_in, view_grey, cv::COLOR_RGB2GRAY);

                // more prescise
                cv::find4QuadCornerSubpix(view_grey, img_points_buffer, Size(5, 5));
                img_points_all.emplace_back(img_points_buffer);
                cv::drawChessboardCorners(view_grey, board_size, img_points_buffer, false);
                cv::imshow("camera calibration", view_grey);
                // waiting for 0.5 s
                cv::waitKey(500);
            }
        }

        int total = img_points_all.size();
        cout << "total corners size: " << total << endl;
        int corner_num = board_size.width*board_size.height;
        for (int j=0; j<total; j++) {
            if (0 == j%corner_num) {
                int i = -1;
                i = j/corner_num;
                int k = i+1;
                cout << "--> corners of " << k << " image: " << endl;
            }
            cout << "       " << img_points_all[j][0].x << " " << img_points_all[j][0].y << endl;
        }

        cout << "\ncorners extraction finished.\n";

        cout << "---------------------- start to calibration -------------------------\n";
        // TODO: this should change to dynamic, now it hard code into fix size
        Size square_size = Size(square_w, square_w);
        // board 3d coordinate
        vector<vector<Point3f>> object_points;

        Mat camera_intrinsic_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
        // corners counts inside every image
        vector<int> point_counts;
        // camera 5 distortion params:k1, k2, p1, p2, k3
        Mat dis_coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
        // tranformation and rotation vectors in all images
        vector<Mat> t_vec;
        vector<Mat> r_vec;

        int i, j, t;
        for(t=0; t<image_count; t++) {
        
            // iterate all images
            vector<Point3f> tmp_points;
            for(i=0; i<board_h;i++) {
                for(j=0; j<board_w; j++) {
                    
                    Point3f real_point;
                    real_point.x = i*square_size.width;
                    real_point.y = i*square_size.height;
                    real_point.z = 0;
                    tmp_points.emplace_back(real_point);

                }
            }
            object_points.emplace_back(tmp_points);
        }

        // init corners num in every image, suppose very image contains full chessboard
        for (i=0; i<image_count;i++) {
            point_counts.push_back(board_size.width*board_size.height);
        }

        cv::calibrateCamera(object_points, img_points_all, image_size, camera_intrinsic_matrix, dis_coeffs, 
                r_vec, t_vec, 0);
        cout << "\ncalibration finished!!\n";
        cout << "----------------------- start judge the calibration result -----------------------\n";

        // all average error
        double total_err = .0;
        // error in every image
        double err = .0;
        vector<Point2f> image_points2d;
        
        for (i=0; i<image_count; i++) {
            vector<Point3f> tmp_points = object_points[i];
            cv::projectPoints(tmp_points, r_vec[i], t_vec[i], camera_intrinsic_matrix, dis_coeffs, image_points2d);

            // cal error of new project points with previous
            vector<Point2f> tmp_img_point = img_points_all[i];
            Mat tmp_img_point_mat = Mat(1, tmp_img_point.size(), CV_32FC2);
            Mat img_points2d_mat = Mat(1, image_points2d.size(), CV_32FC2);
            for(int j=0; j<tmp_img_point.size(); j++) {
                img_points2d_mat.at<Vec2f>(0, j) = Vec2f(image_points2d[j].x, image_points2d[j].y);
                tmp_img_point_mat.at<Vec2f>(0, j) = Vec2f(tmp_img_point[j].x, tmp_img_point[j].y);
            }

            err = norm(img_points2d_mat, tmp_img_point_mat, NORM_L2);
            total_err += err /= point_counts[i];

            cout << i+1 << "th image average error: " << err << " pixel\n";
        }

        cout << "over all average error: " << total_err / image_count << " pixel\n";
        cout << "judge done!\n";
        
        cout << "------------------- start save calibration result --------------------------\n";
        Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
        cout << "camera intrinsic: " << camera_intrinsic_matrix << endl;
        cout << "camera distortion: " << dis_coeffs << endl;

        cout << "calibration done!\n";

    }
    
}
