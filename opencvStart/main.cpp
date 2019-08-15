
#include <cstdlib>
#include <opencv.hpp>


/*
 * 
 */
int main(int argc, char** argv) {

    // Name of image
    std::string file("/home/angel/Dropbox/Master/src/graphModels/Figure8.30b.jpg"); 
    
    // Reading image from file in its original color
    cv::Mat imgOrg=cv::imread(file.c_str());
 
    // Showing image in a window of imgOrg as window name
    cv::imshow("imgOrg", imgOrg);
    
    // Iteration over each pixel image and obtainig the negative of the image
    // Acces as vector of three compones (one for each channel) RGB
    // In this case the elements are unsigned char
    for(unsigned int i=0; i<imgOrg.rows; i++)
    {
        for(unsigned int j=0; j<imgOrg.cols; j++)
        {
            imgOrg.at<cv::Vec3b>(i,j)[0] = (uchar) 255-imgOrg.at<cv::Vec3b>(i,j)[0]; // Blue
            imgOrg.at<cv::Vec3b>(i,j)[1] = (uchar) 255-imgOrg.at<cv::Vec3b>(i,j)[1]; // Green
            imgOrg.at<cv::Vec3b>(i,j)[2] = (uchar) 255-imgOrg.at<cv::Vec3b>(i,j)[2]; // Red
        }
    }
    
    // Showing image in a window of Negative as window name
    cv::imshow("Negative", imgOrg);
    
    // Waiting for the user to press some key to close window display
    // Its default parameter is 0 ms (the window stay steady)
    cv::waitKey();

    /*
     * 
     */
    
    // Creating a 3X3 matrix of doubles
    cv::Mat tmp = cv::Mat(3,3, CV_64FC1);
    
    // Iteration over matrix elements and filling them
    for(unsigned int i=0; i<tmp.rows; i++)
        for(unsigned int j=0; j<tmp.cols; j++)
            tmp.at<double>(i,j) = i*j;
    
    // Showing filled matrix
    std::cout<<tmp;
    
    return 0;
}

