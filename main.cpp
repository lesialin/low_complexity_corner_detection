#include "corner_detect.h"

string config_path = "../config.json";
/*
 * Get File Name from a Path with or without extension
 */
string getFileName(string filePath, bool withExtension = false, char seperator = '/')
{
    // Get last dot position
    size_t dotPos = filePath.rfind('.');
    size_t sepPos = filePath.rfind(seperator);
    if(sepPos != std::string::npos)
    {
        return filePath.substr(sepPos + 1, filePath.size() - (withExtension || dotPos != std::string::npos ? 1 : dotPos) );
    }
    return "";
}




int main_image(char image_path[100])
{
    Mat image, result_image;
    char out_image_path[100];
    string image_filename;
    uint16_t image_width, image_height;
    vector<Point> corners;

    load_config(config_path);

    image_filename = getFileName(image_path);
    

    image = imread(image_path, 0);
    result_image = imread(image_path, 1);

    if (!image.data)
    {
        printf("No image data \n");
        return -1;
    }

    image_width = image.cols;
    image_height = image.rows;

    // detect_corner(image.data, image_width, image_height, corners);
    detect_corner_lcp(image.data, image_width, image_height, corners);
    draw_corner(corners, result_image);
    imshow("corner detection", result_image);

    imwrite(image_filename, result_image);
    waitKey(0);

    return 0;
}

int main(int argc, char **argv)
{

    if (argc != 2)
    {
        printf("Please enter: ./corner_detect xxx.png\n");
        return -1;
    }

    main_image(argv[1]);

    return 0;
}