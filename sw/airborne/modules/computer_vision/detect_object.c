#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>

//#include <opencv/highgui.h>

using namespace cv;
using namespace std;

//void get_integral_image(uint8_t, uint32_t, uint32_t, uint32_t);


///*
int main(){

/*--------------------------  threshold image --------------------------*/
/*
   vector<string> filename;
   string folder = "/image/";
   glob(folder, filename);
   
   for(size_t i=0; i < filename.size(); ++i)
   {
      Mat image = imread(filename[i]);
      if(! image.data )                              // Check for invalid input
      {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
      }
      */
   String path("image/*.jpg");
   //String path("image/*.png");      
   vector<String> filename;
   vector<Mat> data;
   glob(path,filename,true);
   
   for (size_t i =0; i<filename.size(); ++i)
   //for (size_t i =0; i<2; ++i)
   {
      Mat image = imread(filename[i]);
      if(! image.data )                              // Check for invalid input
      {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
      }
      //data.push_back(image);
   
    
    int image_width = 272;
    int image_height = 272;
   //} //for loop
    /*
    Mat image;
    // test iamges
    //image = imread("image/circle/circle_image5.jpg", CV_LOAD_IMAGE_UNCHANGED);   // Read the file
    //image = imread("image/Contour/contour2.png", 1);  
    // real images

    image = imread("image/img_00013.jpg", CV_LOAD_IMAGE_UNCHANGED);
    //image = imread("image/img_00172.jpg", 1); 
    image = imread("image/cyberzoo/im_18.png", CV_LOAD_IMAGE_UNCHANGED);
    //resize(image,image,Size(640,480));
    */

    imshow( "Original image", image );
    
    // convert to YUV
    Mat YUV_image, thresh_image;
    cvtColor( image, YUV_image, CV_BGR2YUV );
    //imshow("YUV image", YUV_image);

    //Variables for thresholds
    int y1, y2, u1, u2, v1, v2;
   // for mavlab:   Y:16-95, U:135-179, V:80-165, turn white.  Y:16-255, U: 77-135, V:71-137, turn black. img_00172 
   y1=16;  u1=135; v1=80; // in the mavlab, bright
   y2=100; u2=175; v2=165;
   // for cyberzoo: Y:12-95, U:129-161, V:80-165, turn white.     
   //int y1=16;  int u1=129; int v1=80; % cyberzoo, dark
   //int y2=100; int u2=161; int v2=165;
   
   //y1=16; u1=129; v1=95; // in the cyberzoo, dark
   //y2=90; u2=161; v2=150;
   // a few options:
   // Y:16-95, U:135-179, V:80-165, turn white


   /// Show some stuff
   inRange(YUV_image, Scalar(y1,u1,v1), Scalar(y2,u2,v2), thresh_image);
   //Showing the images
   /*
   for(int i =50; i<100; i++)
      for (int j = 50; j<100; j++)
      {
         cout<<"pixel value:" << int(YUV_image.at<Vec3f>(i, j)[0]) << ","
         << int(YUV_image.at<Vec3f>(i, j)[1])  << "," 
         << int(YUV_image.at<Vec3f>(i, j)[2]) <<std::endl;
      }
      */
   imshow("Thresholded Image", thresh_image);
   

   //cout<<"pixel value:" << int(thresh_image.at<Vec3f>(20, 70)[0]) <<std::endl;
   //cout<<"pixel value:" << int(thresh_image.at<Vec3f>(20, 70)[1]) <<std::endl;
   //cout<<"pixel value:" << int(thresh_image.at<Vec3f>(20, 70)[2]) <<std::endl;

/*
   for(int i =0; i<272; i++)
      for (int j = 0; j<272; j++)
      {
        if ( YUV_image.at<Vec3f>(i, j)[0] <1)
        {YUV_image.at<Vec3f>(i, j)[0]=0;}
        if ( YUV_image.at<Vec3f>(i, j)[1] <1)
        {YUV_image.at<Vec3f>(i, j)[1]=0;}
        if ( YUV_image.at<Vec3f>(i, j)[2] <1)
        {YUV_image.at<Vec3f>(i, j)[2]=0;}
        
        if ( YUV_image.at<Vec3f>(i, j)[0] >1)
        {YUV_image.at<Vec3f>(i, j)[0]=255;}
        if ( YUV_image.at<Vec3f>(i, j)[1] >1)
        {YUV_image.at<Vec3f>(i, j)[1]=255;}
        if ( YUV_image.at<Vec3f>(i, j)[2] >1)
        {YUV_image.at<Vec3f>(i, j)[2]=255;}
      } 
 */   
 /*--------------------------  Object Detection --------------------------*/      
    
    ///*
    Mat edge_image; //Mat blur_image, edge_image;
    // canny
    //Canny( thresh_image, edge_image, 200,3 );// no blur
    // blur
    /*
    GaussianBlur(thresh_image, blur_image, Size(9,9), 5,5);
    imshow("blur image", blur_image);
    // threshold
    threshold(blur_image, blur_image, 127, 255, THRESH_BINARY);

    Canny( thresh_image, edge_image, 200, 3 );// with blur
    
    imshow("blur image threshold", blur_image);
    //imshow("edge image", edge_image);
    */
    
    
    /* get integral images */
    /*
    uint32_t integral_image;
    get_integral_image(image, image_width, image_height, integral_image);
    */
    //Mat integral_image;
    //integral(thresh_image, integral_image, CV_64F);
    //
    /*
    for (int y=0; y<image_width; y++)
      for (int x=0; x<image_height; x++)
      {
         //integral_image.at<uchar>(y, x) = integral_image.at<uchar>(y, x) /12;
         //cout<<"pixel value:" <<integral_image.at<uchar>(y, x) <<std::endl;
      }
      */
    
    // print pixel values  
    //cout<< "heehee" << endl;
    //cout<<"pixel value:" << int(integral_image.at<Vec3f>(270, 270).val[0]) <<std::endl;
    //cout<<"pixel value:" << int(integral_image.at<Vec3f>(270, 270).val[1]) <<std::endl;
    //cout<<"pixel value:" << int(integral_image.at<Vec3f>(270, 270).val[2]) <<std::endl;
    //cout<<"pixel value:" << int(integral_image.at<Vec3f>(270, 270)[0]) <<std::endl;
    //cout<<"pixel value:" << int(integral_image.at<Vec3f>(270, 270)[1]) <<std::endl;
    //cout<<"pixel value:" << int(integral_image.at<Vec3f>(270, 270)[2]) <<std::endl;
    
    //cout<< "   " <<std::endl;
    //Multiply(integral_image/128);
    //imshow("integral image", integral_image);
    
    /// Find contours   
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    edge_image = thresh_image;
    //findContours( edge_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    findContours( edge_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    // Get the moments
    vector<Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
      { mu[i] = moments( contours[i], false ); }

    //  Get the mass centers:
    vector<Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
     
    /// Draw contours
    Mat drawing = Mat::zeros( edge_image.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
        circle( drawing, mc[i], 4, color, -1, 8, 0 );
        //cout<< "mc: " << mc[1] <<std::endl;
    }     
    
    imshow( "Result window", drawing );

/*-------------------------- Find Largest Contour------------------------ */
    int largest_contour_index = 0;
    int largest_area = 0;
    Rect bounding_rect;
    //vector<vector<Point>> contours; // Vector for storing contour
    //vector<Vec4i> hierarchy;
    
    
    // iterate through each contour.
    for( int i = 0; i< contours.size(); i++ )
    {
        //  Find the area of contour
        double a = contourArea( contours[i],false); 
        if(a>largest_area)
        {
            largest_area = a; 
            //cout<< i <<" area  "<< a <<endl;
            // Store the index of largest contour
            largest_contour_index = i;               
            // Find the bounding rectangle for biggest contour
            bounding_rect = boundingRect(contours[i]);
        }
    }
    Scalar color( 255,255,255);  // color of the contour in the
    /*Draw the contour and rectangle */
    drawContours( image, contours, largest_contour_index, color, CV_FILLED,8,hierarchy);
    
    rectangle(image, bounding_rect,  Scalar(0,255,0), 2, 8, 0);
    
    // some figure can cause there are no largest circles, in this case, do not draw circle
    circle( image, mc[largest_contour_index], 4, Scalar(0,255,0), -1, 8, 0 );
    
    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
    imshow( "Display window", image ); 
    //*/
    
    
    waitKey(1);     
    
    }
    
    
    waitKey(0);                                         
    return 0;
    
    
}




/*
void get_integral_image(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *integral_image)
{
  uint16_t x, y;
  for (x = 0; x < image_width; x++) {
    for (y = 0; y < image_height; y++) {
      if (x >= 1 && y >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x - 1 + y * image_width] +
                                              integral_image[x + (y - 1) * image_width] - integral_image[x - 1 + (y - 1) * image_width];
      } else if (x >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x - 1 + y * image_width];
      } else if (y >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x + (y - 1) * image_width];
      } else {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width];
      }
    }
  }
}
*/






