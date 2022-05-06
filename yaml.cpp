#include <opencv2/opencv.hpp>
#include <time.h>
#include <iostream>
using namespace std;
int main(int argc, char** argv)
{
  //write a yaml file
  cv::FileStorage fs_read("config.yaml", cv::FileStorage::READ);
  string path;
  fs_read["mid_cut_pcd"] >> path;
  fs_read.release();
  return 0;
}