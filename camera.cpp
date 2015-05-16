#include <stdio.h>

// Includes:

#include "MarkerDetector.hpp"
#include "CameraCalibration.hpp"
#include "BGRAVideoFrame.h"
#include <iostream>
#include "marker.hpp"
#include "GeometryTypes.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <queue>          // std::queue

#include <math.h>

using namespace cv;
using namespace std;

int marker_detection_wrapper(cv::Mat, std::vector<Marker> & ,cv::Mat &, std::vector<Marker> &, vector<cv::Point2f> &, int, int);
cv::Mat warping_wrapper(cv::Mat , cv::Mat , std::vector<Marker> & , vector<cv::Point2f> & , cv::Mat &);
cv::Mat update_frame(cv::Mat ,cv::Mat ,std::vector<Marker> & , cv::Mat, cv::Mat);


cv::Point2f translate(cv::Point2f , cv::Point2f, float , float );

cv::Point2f warpPoints(cv::Point2f, cv::Mat );

pair<float, float> find_velocity(queue<cv::Point>);
float newRows, newCols;


int main( int argc, char** argv )
{
	//string frameName = "/Users/Leech/Dropbox/Project/Cheng's Folder/Marker_Detection/photo2.jpg";
    //cv::Mat camera_frame = cv::imread(frameName, 1);

	cv::Mat mountain_frame = cv::imread("Mountain.jpg",1);
    VideoCapture cap(0); // open the default camera
    VideoCapture video("ball.MOV"); //open video

    if(!cap.isOpened() || !video.isOpened())  // check if we succeeded
        return -1;
    
    namedWindow("MyWindow",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
    
    //get size of the embedded video
    int w = video.get(3);
    int h = video.get(4);
    
    cout << "width is " << w << endl;
    cout << "height is "<< h << endl;

    std::vector<Marker> markersFound;

    std::vector<Marker> temp;

    cv::Mat dstImg;

    int count=0;
    int marker_size;
    int not_found_count=0; 

    cv::Mat final_frame;
    std::vector<Marker>  prev_markersFound;
    vector<cv::Point2f>  prev_rPoints;

    //variables to keep track of points
    queue<cv::Point> c1;
    queue<cv::Point> c2;
    queue<cv::Point> c3;
    queue<cv::Point> c4;

    cv::Point P1,P2,P3,P4;

    while(1)
    {
        count = count + 1;
        cout << count << endl;
        Mat frame_camera;
        Mat frame_vid;


        bool bSuccess = cap.read(frame_camera); // read a new frame from webcam

        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }
    
        if(count > 20){
            bool vidSuccess = video.read(frame_vid); // read a new from from video 
            if (!frame_vid.empty()){
                std::vector<Marker> markersFound;
                vector<cv::Point2f>  rPoints;

                //resize(frame_vid, frame_vid, Size(300,300),0,0, INTER_CUBIC);
  
                marker_size = marker_detection_wrapper(frame_camera, markersFound, dstImg, markersFound, rPoints, w,h);
                
                // if a marker is found, apply homography
                if(marker_size>0){
                    not_found_count = 0;
                    prev_markersFound = markersFound;
                    prev_rPoints = rPoints;
                    

                    cv::Mat homography;
                    cv::Mat dstImg = warping_wrapper(frame_camera, frame_vid, markersFound, rPoints, homography );

                    P1 = cv::Point(int(markersFound[0].points[0].x),int(markersFound[0].points[0].y));
                    P2 = cv::Point(int(markersFound[0].points[1].x),int(markersFound[0].points[1].y));
                    P3 = cv::Point(int(markersFound[0].points[2].x),int(markersFound[0].points[2].y));
                    P4 = cv::Point(int(markersFound[0].points[3].x),int(markersFound[0].points[3].y));
                
                    if(c1.size()<5){
                        c1.push(P1);
                        c2.push(P2);
                        c3.push(P3);
                        c4.push(P4);
                    }
                    else{
                        c1.pop();
                        c2.pop();
                        c3.pop();
                        c4.pop();

                        c1.push(P1);
                        c2.push(P2);
                        c3.push(P3);
                        c4.push(P4);
                    }

                    final_frame = update_frame(frame_camera, dstImg, markersFound, frame_vid, homography);
                }
                // if no marker found, just display the camera feed
                else{
                    // if there is no marker found in n frames
                    if(not_found_count<5){
                        cout << "no marker detected" << endl;
                        not_found_count++;
                        
                        // if there are 5 points in the queue, calculate the velocity
                        if(c1.size()>=5){
                            cout << "apply velocity" << endl;

                            pair<float, float> c1_v;
                            pair<float, float> c2_v;
                            pair<float, float> c3_v;
                            pair<float, float> c4_v;

                            c1_v = find_velocity(c1);
                            c2_v = find_velocity(c2);
                            c3_v = find_velocity(c3);
                            c4_v = find_velocity(c4);

                            //update marker location
                            prev_markersFound[0].points[0].x += c1_v.first;
                            prev_markersFound[0].points[0].y += c1_v.second;
                            
                            prev_markersFound[0].points[1].x += c2_v.first;
                            prev_markersFound[0].points[1].y += c2_v.second;

                            prev_markersFound[0].points[2].x += c3_v.first;
                            prev_markersFound[0].points[2].y += c3_v.second;

                            prev_markersFound[0].points[3].x += c4_v.first;
                            prev_markersFound[0].points[3].y += c4_v.second;

                            cout << "dx " << c1_v.first << endl;
                        }
                        if(prev_markersFound.size() > 0){

                            cv::Mat homography;
                            cv::Mat dstImg = warping_wrapper(frame_camera, frame_vid, prev_markersFound, prev_rPoints, homography);
                            final_frame = update_frame(frame_camera, dstImg, prev_markersFound, frame_vid, homography);
                        }
                        else{
                            final_frame = frame_camera;
                        }
                    }
                    else{
                        final_frame = frame_camera;
                    }
                }
            }
            else{
                cout << "no more frames" << endl;
                bool set = video.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
                count = 21;
            }
        }
        else{
            final_frame = frame_camera;
        }
        imshow("MyWindow", final_frame); //show the frame in "MyVideo" window

        if(waitKey(30) == 27)
        {
            cout << "esc key is pressed by user" << endl; 
            break; 
        }

    }
    return 0;
}


int marker_detection_wrapper(cv::Mat camera_frame, std::vector<Marker> & markers, cv::Mat & dstImg_save, std::vector<Marker> & markersFound, vector<cv::Point2f> & rPoints, int w, int h)
{
	CameraCalibration calib1(981.722,991.469,592.84,374.235);
	MarkerDetector marker1(calib1);
	string frameName = "/Users/Leech/Dropbox/Project/Cheng's Folder/Marker_Detection/photo2.jpg";
	int frameHeight = camera_frame.rows;
	int frameWidth = camera_frame.cols;
	//cout<<"things worked"<<endl;

    marker1.processFrame(camera_frame);
    markersFound = marker1.returnDetectedMarkers();
    markers = markersFound;

    rPoints.push_back(cv::Point2f(0,0));
    rPoints.push_back(cv::Point2f(299,0));
    rPoints.push_back(cv::Point2f(299,299));
    rPoints.push_back(cv::Point2f(0,299));

    return markers.size();
}


cv::Mat warping_wrapper(cv::Mat camera_frame, cv::Mat video_frame, std::vector<Marker> & markersFound, vector<cv::Point2f> & rPoints, cv::Mat & homography){
	float scaleFactor = 1.5;
    
    //cout << "main called" << endl;
    cv::Mat dstImg;
    int frameHeight = camera_frame.rows;
	int frameWidth = camera_frame.cols;

    int h = video_frame.rows;
    int w = video_frame.cols;
/*
    if(h<w){
        w = w * 300/h;
        h = 300;

        resize(video_frame, video_frame, Size(w,h),0,0, INTER_CUBIC);
    }
    else{
        h = h * 300/w;
        w = 300;
        resize(video_frame, video_frame, Size(w,h),0,0, INTER_CUBIC);
    }
*/
    homography = cv::getPerspectiveTransform(rPoints,markersFound[0].points);
    cv::Mat srcImg = video_frame;//cv::imread("Mountain.jpg",1);

	int tempw, temph;
	tempw = srcImg.cols;
	temph = srcImg.rows;
	if (tempw<temph){
		temph = scaleFactor * temph * 300/tempw;
		tempw = scaleFactor * 300;
		cv::resize(srcImg, srcImg, cv::Size(tempw,temph),0,0, cv::INTER_CUBIC);
	}
	else{
		tempw = scaleFactor * tempw * 300/temph;
		temph = scaleFactor * 300;
		cv::resize(srcImg, srcImg, cv::Size(tempw,temph),0,0, cv::INTER_CUBIC);
	}


    cv::Size dSize(frameWidth,frameHeight);
    cv::warpPerspective(srcImg,dstImg,homography,dSize);
	
	newRows = srcImg.rows;
	newCols = srcImg.cols;

    return dstImg;
}


cv::Mat update_frame(cv::Mat camera_frame, cv::Mat dstImg, std::vector<Marker> & markersFound, cv::Mat video_frame, cv::Mat homography){
	cv::Mat origImg = camera_frame; //cv::imread(frameName,1);

	int new_h, new_w;
    
    float vid_h = video_frame.rows;
    float vid_w = video_frame.cols;

    int frameHeight = camera_frame.rows;
	int frameWidth = camera_frame.cols;
    cv::Size dSize(frameWidth,frameHeight);



	if(dstImg.rows>origImg.rows)
	    new_h=origImg.rows;
	else
	    new_h=dstImg.rows;

	if(dstImg.cols>origImg.cols)
        new_w=origImg.cols;
	else
	    new_w=dstImg.cols;

	cv::Rect rectROI(0,0,new_w,new_h);
	// ADDED
	cv::Mat tempMask(cv::Size(newCols,newRows),CV_8UC3,CV_RGB(255,255,255));
	cv::Mat tempMask2,mask;
	cv::warpPerspective(tempMask,tempMask2,homography,dSize);
	// convert mask to 8UC1
	cv::cvtColor(tempMask2,mask,CV_RGB2GRAY);
	// ADDED END
/*

	cv::Mat mask(new_h, new_w, CV_8UC1, cv::Scalar(0));

    Mat white_mask = Mat(vid_h, vid_w, CV_8UC1, cv::Scalar(1));
    Mat out_mask = Mat(vid_h, vid_w, CV_8UC1, cv::Scalar(0));

    cv::Size dSize(new_w,new_h);
    cv::warpPerspective(white_mask,out_mask,homography,dSize,   INTER_LINEAR, BORDER_CONSTANT, Scalar(0));

	cv::Point P1,P2,P3,P4;

	P1 = cv::Point(int(markersFound[0].points[0].x),int(markersFound[0].points[0].y));
	P2 = cv::Point(int(markersFound[0].points[1].x),int(markersFound[0].points[1].y));
	P3 = cv::Point(int(markersFound[0].points[2].x),int(markersFound[0].points[2].y));
	P4 = cv::Point(int(markersFound[0].points[3].x),int(markersFound[0].points[3].y));
  
    P2 = translate(P1, P2, 300, vid_w);
    P3 = translate(P4, P3, 300, vid_w);

    P1 = warpPoints(cv::Point2f(0,0), homography);
    P2 = warpPoints(cv::Point2f(vid_h,0), homography);
    P3 = warpPoints(cv::Point2f(vid_h,vid_w), homography);
    P4 = warpPoints(cv::Point2f(0,vid_w), homography);
    cout << "POINTS" << endl;
    cout << P1.x << " " << P1.y << endl;
    cout << P2.x << " " << P2.y << endl;
    cout << P3.x << " " << P3.y << endl;
    cout << P4.x << " " << P4.y << endl;

	vector<vector<cv::Point> >  co_ordinates;
	co_ordinates.push_back(vector<cv::Point>());

	co_ordinates[0].push_back(P1);
	co_ordinates[0].push_back(P2);
	co_ordinates[0].push_back(P3);
	co_ordinates[0].push_back(P4);
	cv::drawContours( mask,co_ordinates,0, cv::Scalar(255),CV_FILLED, 8 );
*/
	cv::Mat srcROI = dstImg(rectROI);
	cv::Mat dstROI = origImg(rectROI);
	cv::Mat dst1;
	cv::Mat dst2;

	srcROI.copyTo(dst1,mask);

	bitwise_not(mask,mask);
	dstROI.copyTo(dst2,mask);

	dstROI.setTo(0);
	dstROI=dst1+dst2;

	return dstROI;
}

cv::Point2f warpPoints(cv::Point2f p, cv::Mat m)
{
    cv::Point2f dst;

    dst.x = (m.at<float>(1,1)*p.x + m.at<float>(1,2)*p.y + m.at<float>(1,3)) / (m.at<float>(3,1)*p.x + m.at<float>(3,2)*p.y + m.at<float>(3,3));

    dst.y = (m.at<float>(2,1)*p.x + m.at<float>(2,2)*p.y + m.at<float>(2,3)) / (m.at<float>(3,1)*p.x + m.at<float>(3,2)*p.y + m.at<float>(3,3));
    
    cout << "M" << endl;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            cout << m.at<float>(i,j) << " ";
        } 
        cout << endl;
    }
     

    return dst;
}


cv::Point2f translate(cv::Point2f p1, cv::Point2f p2, float l1, float l2)
{
    float p1x = p1.x;
    float p1y = p1.y;

    float p2x = p2.x;
    float p2y = p2.y;

    float d1 = pow((pow(p1x - p2x,2) + pow(p1y - p2y,2)),0.5);
    float d2 = (d1*l2)/l1;

    cv::Point2f ret;

    ret.x = p1x + (d2 * (p2x-p1x))/d1;

    ret.y = p1y + (d2 * (p2y-p1y))/d1;

    return ret;
}


pair<float, float> find_velocity(queue<cv::Point> corner)
{
    int n = corner.size();
    float vx=0;
    float vy=0;

    cv::Point prev = corner.front();
    corner.pop();

    for(int i=0; i<(n-1); i++){
        cv::Point temp = corner.front();
        corner.pop();

        vx = vx + (temp.x-prev.x);
        vy = vy + (temp.y-prev.y);

        prev = temp;
    }

    vx = vx/n;
    vy = vy/n;

    pair<float, float> ret;
    ret.first = vx;
    ret.second = vy;

    return ret;
}
