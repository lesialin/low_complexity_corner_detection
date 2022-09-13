#pragma once
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <vector> 
#include "opencv2/opencv.hpp"
#include "nlohmann/json.hpp"

using namespace std;
using namespace cv;


void load_config(string filename);
void cal_image_gradient(uint8_t *image, uint16_t image_width, uint16_t image_height, uint32_t *gxx, uint32_t *gyy, int32_t *gxy, uint32_t &max_gxy);
void detect_corner_lcp(uint8_t *image, uint16_t image_width, uint16_t image_height, vector<Point> &corners);
void draw_corner(vector<Point> corners, Mat &corner_image);


