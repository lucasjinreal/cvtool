#include <iostream>  
#include <fstream>  
#include <io.h>  
#include "opencv2/opencv.hpp"
using namespace cv;  
using namespace std;  

#define BOARD_SCALE 20
#define BOARD_HEIGHT 6
#define BOARD_WIDTH 8

//获取某一文件夹的所有文件名
void getFiles(string path, vector<string>& files)
{
    //文件句柄  
    long   hFile = 0;
    //文件信息，声明一个存储文件信息的结构体  
    struct _finddata_t fileinfo;
    string p;//字符串，存放路径
    if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)//若查找成功，则进入
    {
        do
        {
            //如果是目录,迭代之（即文件夹内还有文件夹）  
            if ((fileinfo.attrib &  _A_SUBDIR))
            {
                //文件名不等于"."&&文件名不等于".."
                //.表示当前目录
                //..表示当前目录的父目录
                //判断时，两者都要忽略，不然就无限递归跳不出去了！
                if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
                    getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
            }
            //如果不是,加入列表  ,这里进行了判断，只有是.jpg的文件才加入列表
            else
            {
                string a=".bmp";
                string b=".jpg";
                string::size_type idx1,idx2;
                char* temp = fileinfo.name;
                string temp1 = temp;
                idx1=temp1.find(a);
                idx2=temp1.find(b);
                if(idx1 == string::npos && idx2 != string::npos )
                    files.push_back(p.assign(path).append("\\").append(fileinfo.name));
            }
        } while (_findnext(hFile, &fileinfo) == 0);
        //_findclose函数结束查找
        _findclose(hFile);
    }
}

void main()   
{  
    vector<string> files;
    files.clear();
    string filePath = "image/";
    getFiles(filePath, files );

    cout << "找到的文件有"<< endl;
    for(int i = 0;i<files.size();i++)
    {
        cout << files[i] <<endl;
    }


    //读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化   
    cout<<"开始提取角点………………"<<endl;  
    int image_count=0;  /* 图像数量 */  
    Size image_size;  /* 图像的尺寸 */  
    Size board_size = Size(BOARD_HEIGHT,BOARD_WIDTH);    /* 标定板上每行、列的角点数 */  
    vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */  
    vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */ 

    for(int i = 0;i<files.size();i++)
    {  
        cout<<files[i]<<endl;

        Mat imageInput=imread(files[i]);

        /* 提取角点 */  
        if (0 == findChessboardCorners(imageInput,board_size,image_points_buf))  
        {             
            cout<<"can not find chessboard corners!\n"; //找不到角点  
            continue;
        }   
        else   
        {  
            //找到一幅有效的图片
            image_count++;
            if (image_count == 1)  //读入第一张图片时获取图像宽高信息  
            {  
                image_size.width = imageInput.cols;  
                image_size.height =imageInput.rows;           
                cout<<"image_size.width = "<<image_size.width<<endl;  
                cout<<"image_size.height = "<<image_size.height<<endl;  
            }  

            Mat view_gray;  
            cvtColor(imageInput,view_gray,CV_RGB2GRAY);

            /* 亚像素精确化 */  
            //find4QuadCornerSubpix(view_gray,image_points_buf,Size(5,5)); //对粗提取的角点进行精确化  
            cornerSubPix(view_gray, image_points_buf, 
                Size(5,5), 
                Size(-1,-1), 
                TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 
                30,        // max number of iterations 
                0.1));     // min accuracy

            image_points_seq.push_back(image_points_buf);  //保存亚像素角点

            /* 在图像上显示角点位置 */  
            drawChessboardCorners(view_gray,board_size,image_points_buf,true); //用于在图片中标记角点 

            //写入文件
            string filePath = files[i]; 
            filePath+=".bmp"; 
            imwrite(filePath,view_gray);   
        }  
    }  

    int total = image_points_seq.size();  
    cout<< "共使用了"<<total << "幅图片"<<endl;
    cout<<"角点提取完成！\n"; 


    cout<<"开始标定………………\n";  
    /*棋盘三维信息*/  
    Size square_size = Size(BOARD_SCALE,BOARD_SCALE);  /* 实际测量得到的标定板上每个棋盘格的大小 */  
    vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */  
    /*内外参数*/  
    Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */  
    vector<int> point_counts;  // 每幅图像中角点的数量  
    Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */  
    vector<Mat> tvecsMat;  /* 每幅图像的旋转向量 */  
    vector<Mat> rvecsMat; /* 每幅图像的平移向量 */  
    /* 初始化标定板上角点的三维坐标 */  
    int i,j,t;  
    for (t=0;t<image_count;t++)   
    {  
        vector<Point3f> tempPointSet;  
        for (i=0;i<board_size.height;i++)   
        {  
            for (j=0;j<board_size.width;j++)   
            {  
                Point3f realPoint;  
                /* 假设标定板放在世界坐标系中z=0的平面上 */  
                realPoint.x = i*square_size.width;  
                realPoint.y = j*square_size.height;  
                realPoint.z = 0;  
                tempPointSet.push_back(realPoint);  
            }  
        }  
        object_points.push_back(tempPointSet);  
    }  

    /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */  
    for (i=0;i<image_count;i++)  
    {  
        point_counts.push_back(board_size.width*board_size.height);  
    }


    /* 开始标定 */  
    calibrateCamera(object_points,image_points_seq,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,CV_CALIB_RATIONAL_MODEL);  
    cout<<"标定完成！\n";  

    //对标定结果进行评价  
    ofstream fout("caliberation_result.txt");  /* 保存标定结果的文件 */   

    double total_err = 0.0; /* 所有图像的平均误差的总和 */  
    double err = 0.0; /* 每幅图像的平均误差 */  
    vector<Point2f> image_points2; /* 保存重新计算得到的投影点 */  
    cout<<"\t每幅图像的标定误差：\n";  
    fout<<"每幅图像的标定误差：\n";  
    for (i=0;i<image_count;i++)  
    {  
        vector<Point3f> tempPointSet=object_points[i];  
        /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */  
        projectPoints(tempPointSet,rvecsMat[i],tvecsMat[i],cameraMatrix,distCoeffs,image_points2);  
        /* 计算新的投影点和旧的投影点之间的误差*/  
        vector<Point2f> tempImagePoint = image_points_seq[i];  
        Mat tempImagePointMat = Mat(1,tempImagePoint.size(),CV_32FC2);  
        Mat image_points2Mat = Mat(1,image_points2.size(), CV_32FC2);  
        for (int j = 0 ; j < tempImagePoint.size(); j++)  
        {  
            image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);  
            tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);  
        }  
        err = norm(image_points2Mat, tempImagePointMat, NORM_L2);  
        total_err += err/=  point_counts[i];     
        cout<<"第"<<i+1<<"幅图像的平均误差："<<err<<"像素"<<endl;     
        fout<<"第"<<i+1<<"幅图像的平均误差："<<err<<"像素"<<endl;     
    }  

    cout<<"总体平均误差："<<total_err/image_count<<"像素"<<endl;     
    fout<<"总体平均误差："<<total_err/image_count<<"像素"<<endl<<endl;     

    //保存定标结果      
    cout<<"开始保存定标结果………………"<<endl;         
    Mat rotation_matrix = Mat(3,3,CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */  
    fout<<"相机内参数矩阵："<<endl;     
    fout<<cameraMatrix<<endl<<endl;     
    fout<<"畸变系数：\n";     
    fout<<distCoeffs<<endl<<endl<<endl;     
    for (int i=0; i<image_count; i++)   
    {   
        fout<<"第"<<i+1<<"幅图像的旋转向量："<<endl;     
        fout<<tvecsMat[i]<<endl;      
        /* 将旋转向量转换为相对应的旋转矩阵 */     
        Rodrigues(tvecsMat[i],rotation_matrix);     
        fout<<"第"<<i+1<<"幅图像的旋转矩阵："<<endl;     
        fout<<rotation_matrix<<endl;     
        fout<<"第"<<i+1<<"幅图像的平移向量："<<endl;     
        fout<<rvecsMat[i]<<endl<<endl;     
    }     
    cout<<"完成保存"<<endl;   
    fout<<endl;

    while(1);
    return ;  
}

