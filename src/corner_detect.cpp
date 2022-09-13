#include "corner_detect.h"
#include <cmath>
using json = nlohmann::json;

// reference: https://github.com/coder-mano/Shi-Tomasi-Corner-Detector

uint8_t g_block_size = 3;
uint8_t g_guassian_3x3[9] = {1, 2, 1, 2, 4, 2, 1, 2, 1};
int8_t g_sobel_x[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
int8_t g_sobel_y[9] = {1, 2, 1, 0, 0, 0, -1, -2, -1};
uint16_t g_thresh = 100;
uint16_t g_max_corner = 500;
uint8_t g_dist = 10;
float g_t1 = 0.01;
float g_t2 = 0.01;

void load_config(string filename)
{

    std::ifstream file(filename);
    json jfile = json::parse(file);

    g_thresh = jfile["corner_thresh"];
    g_max_corner = jfile["max_corner"];
    g_dist = jfile["min_dist"];
    g_t1 = jfile["t1"];
    g_t2 = jfile["t2"];

    printf("------------------------\n");
    printf("corner detection config:\n");
    printf("------------------------\n");
    printf("g_thresh = %d\n", g_thresh);
    printf("g_max_corner = %d\n", g_max_corner);
    printf("g_dist = %d\n", g_dist);
    printf("g_t1 = %f\n", g_t1);
    printf("g_t2 = %f\n", g_t2);
    printf("------------------------\n\n");


}

void cal_image_gradient(uint8_t *image, uint16_t image_width, uint16_t image_height, uint32_t *gxx, uint32_t *gyy, int32_t *gxy, uint32_t &max_gxy)
{
    max_gxy = 0;
    uint32_t abs_gxy;
    int16_t gx, gy;
    uint64_t g_idx,idx;
    

    for (int i = 1; i < image_height - 1; i++)
    {
        for (int j = 1; j < image_width - 1; j++)
        {
            idx = image_width * i + j;
            gx = 0;
            gy = 0;
            for (int m = -1; m <= 1; m++)
            {
                for (int n = -1; n <= 1; n++)
                {
                    idx = image_width * (i + m) + (j + n);
                    g_idx = (m + 1) * 3 + (n + 1);
                    gx += image[idx] * g_sobel_x[g_idx];
                    gy += image[idx] * g_sobel_y[g_idx];

                }
            }

            gxx[idx] = (gx * gx) >> 6;
            gyy[idx] = (gy * gy) >> 6;
            gxy[idx] = (gx * gy) >> 6;

            abs_gxy = gxy[idx] > 0 ? gxy[idx] : -gxy[idx];

            if (abs_gxy > max_gxy)
            {
                max_gxy = abs_gxy;
            }
        }
    }
}

void detect_corner_lcp(uint8_t *image, uint16_t image_width, uint16_t image_height, vector<Point> &corners)
{
    uint32_t *Ia, *Ic, a, c;
    int32_t *Ib, b;
    uint64_t g_idx, idx;
    Point pts;
    uint8_t half_block_size;
    uint32_t corner_thresh;
    uint32_t max_ac;
    uint32_t abs_ac;
    double r, sqrt_value, max_r;
    int16_t x0, y0, x1, y1;
    uint32_t dist_dist;
    bool bigger;
    uint32_t min_dist_dist = g_dist * g_dist;
    float t1, t2;
    t1 = g_t1;
    t2 = g_t2;

    vector<vector<double>> candidate_corners_0;
    vector<vector<double>> candidate_corners_1;
    vector<vector<double>> candidate_corners_2;

    // Ia is gradient image gx*gx
    // Ib is gradient image gx*gy
    // Ic is gradient image gy*gy
    Ia = (uint32_t *)malloc(sizeof(uint32_t) * image_width * image_height);
    Ic = (uint32_t *)malloc(sizeof(uint32_t) * image_width * image_height);
    Ib = (int32_t *)malloc(sizeof(int32_t) * image_width * image_height);

    cal_image_gradient(image, image_width, image_height, Ia, Ic, Ib, max_ac);


    corner_thresh = t1 * max_ac;

    half_block_size = g_block_size >> 1;
    max_r = 0.0;

    for (int i = half_block_size; i < image_height - half_block_size; i++)
    {
        for (int j = half_block_size; j < image_width - half_block_size; j++)
        {

            idx = image_width * i + j;

            a = 0;
            b = 0;
            c = 0;

            // guassian filter to sum up gradient
            for (int m = -half_block_size; m <= half_block_size; m++)
            {
                for (int n = -half_block_size; n <= half_block_size; n++)
                {
                    idx = image_width * (i + m) + (j + n);
                    g_idx = (m + half_block_size) * 3 + (n + half_block_size);
                    a += Ia[idx] * g_guassian_3x3[g_idx];
                    b += Ib[idx] * g_guassian_3x3[g_idx];
                    c += Ic[idx] * g_guassian_3x3[g_idx];
                }
            }

            a = a >> 4;
            b = b >> 4;
            c = c >> 4;

            abs_ac = b > 0 ? b : -b;

            if (abs_ac > corner_thresh)
            {

                sqrt_value = (a - c) * (a - c) + 4 * (b * b);

                if (sqrt_value > 0.0)
                {
                    r = 0.5 * ((double)(a + c) - sqrt(sqrt_value));

                    if (r > max_r)
                    {
                        max_r = r;
                    }
                   
                    candidate_corners_0.push_back({r, (double)j, (double)i});
                }
            }
        }
    }
    
    // filter corner by r >  t*max_r
    corner_thresh = t2 * max_r;
    for (int i = 0; i < static_cast<int>(candidate_corners_0.size()); i++)
    {

        if (candidate_corners_0[i][0] > corner_thresh)
        {
            candidate_corners_1.push_back({candidate_corners_0[i][0], candidate_corners_0[i][1], candidate_corners_0[i][2]});
        }
    }

  
    // sorting by R
    sort(candidate_corners_1.begin(), candidate_corners_1.end(), greater<>());
   
    if (candidate_corners_1.size() > 0)
    {

        candidate_corners_2.push_back({candidate_corners_1[0][0], candidate_corners_1[0][1], candidate_corners_1[0][2]});

        for (int i = 0; i < static_cast<int>(candidate_corners_1.size()); i++)
        {
            bigger = true;
            x0 = candidate_corners_1[i][1];
            y0 = candidate_corners_1[i][2];
            for (int j = 0; j < static_cast<int>(candidate_corners_2.size()); j++)
            {
                x1 = candidate_corners_2[j][1];
                y1 = candidate_corners_2[j][2];

                dist_dist = (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0);
                if (dist_dist < min_dist_dist)
                {
                    bigger = false;
                    break;
                }
            }
            if (bigger)
            {

                candidate_corners_2.push_back({candidate_corners_1[i][0], candidate_corners_1[i][1], candidate_corners_1[i][2]});
            }
        }
    }

    
    
    // // sorting by R
    // sort(candidate_corners_2.begin(), candidate_corners_2.end(), greater<>());

    for (int i = 0; i < static_cast<int>(candidate_corners_2.size()); i++)
    {
        pts.x = candidate_corners_2[i][1];
        pts.y = candidate_corners_2[i][2];
        corners.push_back(pts);
        if (corners.size() >= g_max_corner)
        {
            break;
        }
    }
        
    printf("There are  %ld corner candidate, %.2f %% corners be punned \n",candidate_corners_0.size(),(float)100.0*(image_width*image_height -candidate_corners_0.size())/ (image_width*image_height));
    printf("There are %ld corners to be detected!\n",candidate_corners_1.size());
    printf("There are %ld corners after NMS\n", corners.size());

    free(Ia);
    free(Ib);
    free(Ic);
}


void draw_corner(vector<Point> corners, Mat &corner_image)
{
    
    for (int i = 0; i < static_cast<int>(corners.size()); ++i)
    {
        circle(corner_image, corners[i], 3, Scalar(255, 0, 0), -1);
    }
}
