#include <stdio.h>
#include <stdlib.h>

#include "MarkerDetector.hpp"
#include "CameraCalibration.hpp"
#include "BGRAVideoFrame.h"
#include <iostream>
#include "marker.hpp"
#include "GeometryTypes.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/video/background_segm.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <opencv2/core/core.hpp>

#include <queue>          // std::queue


using namespace cv;
using namespace std;

int marker_detection_wrapper(cv::Mat, std::vector<Marker> & ,cv::Mat &, vector<cv::Point2f> &);
cv::Mat warping_wrapper(cv::Mat , cv::Mat , std::vector<Marker> & , vector<cv::Point2f> & );
cv::Mat update_frame(cv::Mat ,cv::Mat ,std::vector<Marker> &, cv::Mat );

pair<float, float> find_velocity(queue<cv::Point>);

bool is_black(cv::Vec3b);
bool is_white(cv::Vec3b);

// new for occlusion
cv::Point2f pullPointFromMarker(int markerId, int pointId, std::vector<Marker> & markersFound);
bool outerMarkersFound(std::vector<Marker> & markersFound);
cv::Point2f locateMissingPoint(std::vector<Marker> & markersFound,int pointId);
bool isMissing(std::vector<Marker> & markersFound,int missingId);
cv::Point2f findPtUsingPts(cv::Point2f P1,cv::Point2f P2,cv::Point2f P3,cv::Point2f P4);
int numOuterMarkersMissing(std::vector<Marker> & markersFound);
cv::Point2f approxPoint(cv::Point2f Ptemp1,cv::Point2f Ptemp2,cv::Point2f Ptemp3,cv::Point2f Ptemp4);


int main( int argc, char** argv )
{
	float scaleFactor = 2.0;
    

    //for occlusion
    cv::Mat fgMaskMOG;
    cv::Ptr<cv::BackgroundSubtractor> pMOG;
    //create Background Subtractor objects
    pMOG = new cv::BackgroundSubtractorMOG();
    // end for occlusion

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

        //cout << "color " << frame_camera.at<cv::Vec3b>(frame_camera.rows/2,frame_camera.cols/2) << endl;
        //for occlusion
        pMOG->operator()(frame_camera, fgMaskMOG, 0.5);    //0.1 is learning rate
    
        if(count > 20){
            bool vidSuccess = video.read(frame_vid); // read a new from from video 
            if (!frame_vid.empty()){
                std::vector<Marker> markersFound;
                vector<cv::Point2f>  rPoints;

                resize(frame_vid, frame_vid, Size(300,300),0,0, INTER_CUBIC);
  
                marker_size = marker_detection_wrapper(frame_camera, markersFound, dstImg, rPoints);
                
                // if a marker is found, apply homography
                if(marker_size>0){
                    not_found_count = 0;
                    prev_markersFound = markersFound;
                    prev_rPoints = rPoints;

                    cv::Mat dstImg = warping_wrapper(frame_camera, frame_vid, markersFound, rPoints);

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

                    final_frame = update_frame(frame_camera, dstImg, markersFound, fgMaskMOG);
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
                            cv::Mat dstImg = warping_wrapper(frame_camera, frame_vid, prev_markersFound, prev_rPoints);
                            final_frame = update_frame(frame_camera, dstImg, prev_markersFound, fgMaskMOG);
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


int marker_detection_wrapper(cv::Mat camera_frame, std::vector<Marker> & newMarkersFound, cv::Mat & dstImg_save, vector<cv::Point2f> & rPoints)
{
	CameraCalibration calib1(981.722,991.469,592.84,374.235);
	MarkerDetector marker1(calib1);
	string frameName = "/Users/Leech/Dropbox/Project/Cheng's Folder/Marker_Detection/photo2.jpg";
	int frameHeight = camera_frame.rows;
	int frameWidth = camera_frame.cols;
	//cout<<"things worked"<<endl;

    marker1.processFrame(camera_frame);
    std::vector<Marker> markersFound = marker1.returnDetectedMarkers();
    std::vector<Marker> markers = markersFound;

    rPoints.push_back(cv::Point2f(0,0));
    rPoints.push_back(cv::Point2f(299,0));
    rPoints.push_back(cv::Point2f(299,299));
    rPoints.push_back(cv::Point2f(0,299));

	if(markersFound.size() == 6){
		Marker M1;
		M1.id = -1;
		M1.points.push_back(pullPointFromMarker(621,3,markersFound));
		M1.points.push_back(pullPointFromMarker(623,0,markersFound));
		M1.points.push_back(pullPointFromMarker(624,1,markersFound));
		M1.points.push_back(pullPointFromMarker(626,2,markersFound));
		newMarkersFound.push_back(M1);
	}
	else if(markersFound.size() == 5){
		if(outerMarkersFound(markersFound) == true){
			Marker M1;
			M1.id = -1;
			M1.points.push_back(pullPointFromMarker(621,3,markersFound));
			M1.points.push_back(pullPointFromMarker(623,0,markersFound));
			M1.points.push_back(pullPointFromMarker(624,1,markersFound));
			M1.points.push_back(pullPointFromMarker(626,2,markersFound));
			newMarkersFound.push_back(M1);
		}
		else{
			//cout<<"came here"<<endl;
			cv::Point2f missingP;
			Marker M1;
			M1.id = -1;
			if (isMissing(markersFound,621)){
				missingP = locateMissingPoint(markersFound,621);

				M1.points.push_back(missingP);
				M1.points.push_back(pullPointFromMarker(623,0,markersFound));
				M1.points.push_back(pullPointFromMarker(624,1,markersFound));
				M1.points.push_back(pullPointFromMarker(626,2,markersFound));
			}
			else if (isMissing(markersFound,623)){
				missingP = locateMissingPoint(markersFound,623);

				M1.points.push_back(pullPointFromMarker(621,3,markersFound));
				M1.points.push_back(missingP);
				M1.points.push_back(pullPointFromMarker(624,1,markersFound));
				M1.points.push_back(pullPointFromMarker(626,2,markersFound));
			}
			else if (isMissing(markersFound,624)){
				missingP = locateMissingPoint(markersFound,624);

				M1.points.push_back(pullPointFromMarker(621,3,markersFound));
				M1.points.push_back(pullPointFromMarker(623,0,markersFound));
				M1.points.push_back(missingP);
				M1.points.push_back(pullPointFromMarker(626,2,markersFound));
			}
			else{
				missingP = locateMissingPoint(markersFound,626);

				M1.points.push_back(pullPointFromMarker(621,3,markersFound));
				M1.points.push_back(pullPointFromMarker(623,0,markersFound));
				M1.points.push_back(pullPointFromMarker(624,1,markersFound));
				M1.points.push_back(missingP);
			}
			newMarkersFound.push_back(M1);
		}
	}
	else if(markersFound.size() == 4){
		if(outerMarkersFound(markersFound) == true){
			Marker M1;
			M1.id = -1;
			M1.points.push_back(pullPointFromMarker(621,3,markersFound));
			M1.points.push_back(pullPointFromMarker(623,0,markersFound));
			M1.points.push_back(pullPointFromMarker(624,1,markersFound));
			M1.points.push_back(pullPointFromMarker(626,2,markersFound));
			newMarkersFound.push_back(M1);
		}
		else if(numOuterMarkersMissing(markersFound) == 1){
			cv::Point2f missingP;
			Marker M1;
			M1.id = -1;
			if (isMissing(markersFound,621)){
				missingP = locateMissingPoint(markersFound,621);
				M1.points.push_back(missingP);
				M1.points.push_back(pullPointFromMarker(623,0,markersFound));
				M1.points.push_back(pullPointFromMarker(624,1,markersFound));
				M1.points.push_back(pullPointFromMarker(626,2,markersFound));
			}
			else if (isMissing(markersFound,623)){
				missingP = locateMissingPoint(markersFound,623);
				M1.points.push_back(pullPointFromMarker(621,3,markersFound));
				M1.points.push_back(missingP);
				M1.points.push_back(pullPointFromMarker(624,1,markersFound));
				M1.points.push_back(pullPointFromMarker(626,2,markersFound));
			}
			else if (isMissing(markersFound,624)){
				missingP = locateMissingPoint(markersFound,624);
				M1.points.push_back(pullPointFromMarker(621,3,markersFound));
				M1.points.push_back(pullPointFromMarker(623,0,markersFound));
				M1.points.push_back(missingP);
				M1.points.push_back(pullPointFromMarker(626,2,markersFound));
			}
			else{
				missingP = locateMissingPoint(markersFound,626);
				M1.points.push_back(pullPointFromMarker(621,3,markersFound));
				M1.points.push_back(pullPointFromMarker(623,0,markersFound));
				M1.points.push_back(pullPointFromMarker(624,1,markersFound));
				M1.points.push_back(missingP);
			}
			newMarkersFound.push_back(M1);
		}
		else if(numOuterMarkersMissing(markersFound) == 2){
			cv::Point2f missingP;
			Marker M1;
			M1.id = -1;

			if (isMissing(markersFound,621)){
				missingP = locateMissingPoint(markersFound,621);
				M1.points.push_back(missingP);
			}
			else{
				M1.points.push_back(pullPointFromMarker(621,3,markersFound));
			}

			if (isMissing(markersFound,623)){
				missingP = locateMissingPoint(markersFound,623);
				M1.points.push_back(missingP);
			}
			else {
				M1.points.push_back(pullPointFromMarker(623,0,markersFound));
			}
			if (isMissing(markersFound,624)){
				missingP = locateMissingPoint(markersFound,624);
				M1.points.push_back(missingP);
			}
			else{
				M1.points.push_back(pullPointFromMarker(624,1,markersFound));
			}
			if (isMissing(markersFound,626)){
				missingP = locateMissingPoint(markersFound,626);
				M1.points.push_back(missingP);
			}
			else{
				M1.points.push_back(pullPointFromMarker(626,2,markersFound));
			}
			newMarkersFound.push_back(M1);
		}
		else{
			cout<<"no help here"<<endl;
			return 0;
		}
	}
	else{
		cout<<"Not enough markers found"<<endl;
		return 0;
	}




    return markers.size();
}


cv::Mat warping_wrapper(cv::Mat camera_frame, cv::Mat video_frame, std::vector<Marker> & markersFound, vector<cv::Point2f> & rPoints){
    //cout << "main called" << endl;
    cv::Mat dstImg;
    int frameHeight = camera_frame.rows;
	int frameWidth = camera_frame.cols;

    cv::Mat homography = cv::getPerspectiveTransform(rPoints,markersFound[0].points);
    cv::Mat srcImg = video_frame;//cv::imread("Mountain.jpg",1);
/*
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
*/

    cv::Size dSize(frameWidth,frameHeight);
    cv::warpPerspective(srcImg,dstImg,homography,dSize);

    return dstImg;
}


cv::Mat update_frame(cv::Mat camera_frame, cv::Mat dstImg, std::vector<Marker> & markersFound, cv::Mat fgMaskMOG){
	cv::Mat origImg = camera_frame; //cv::imread(frameName,1);

	int new_h, new_w;

	if(dstImg.rows>origImg.rows)
	 new_h=origImg.rows;
	else
	 new_h=dstImg.rows;

	if(dstImg.cols>origImg.cols)
	 new_w=origImg.cols;
	else
	 new_w=dstImg.cols;

	cv::Rect rectROI(0,0,new_w,new_h);
    /*
	// ADDED
	cv::Mat tempMask(cv::Size(srcImg.cols,srcImg.rows),CV_8UC3,CV_RGB(255,255,255));
	cv::Mat tempMask2,mask;
	cv::warpPerspective(tempMask,tempMask2,homography,dSize);
	// convert mask to 8UC1
	cv::cvtColor(tempMask2,mask,CV_RGB2GRAY);
	// ADDED END
    */
	cv::Mat mask(new_h, new_w, CV_8UC1, cv::Scalar(0));
    

	cv::Point P1,P2,P3,P4;

	P1 = cv::Point(int(markersFound[0].points[0].x),int(markersFound[0].points[0].y));
	P2 = cv::Point(int(markersFound[0].points[1].x),int(markersFound[0].points[1].y));
	P3 = cv::Point(int(markersFound[0].points[2].x),int(markersFound[0].points[2].y));
	P4 = cv::Point(int(markersFound[0].points[3].x),int(markersFound[0].points[3].y));
    
	vector<vector<cv::Point> >  co_ordinates;
	co_ordinates.push_back(vector<cv::Point>());

	co_ordinates[0].push_back(P1);
	co_ordinates[0].push_back(P2);
	co_ordinates[0].push_back(P3);
	co_ordinates[0].push_back(P4);
	cv::drawContours( mask,co_ordinates,0, cv::Scalar(255),CV_FILLED, 8 );
	cv::Mat srcROI = dstImg(rectROI);
	cv::Mat dstROI = origImg(rectROI);
	cv::Mat dst1;
	cv::Mat dst2;

    for(int i=0; i<camera_frame.rows; i++){
        for(int j=0; j<camera_frame.cols; j++){
            //cout << "outside " << mask.at<double>(i,j) << endl;
            if ((int)mask.at<uchar>(i,j) == 255){
                //cout << "inside " << camera_frame.at<cv::Vec3b>(i,j) << endl;
                if( !is_black(camera_frame.at<cv::Vec3b>(i,j)) &&  !is_white(camera_frame.at<cv::Vec3b>(i,j)) ){
                    mask.at<uchar>(i,j) = 0;
                }
            }
        }
    }
    cout << "after loop" << endl;
    //bitwise_not(fgMaskMOG,fgMaskMOG);
    //cv::bitwise_and(fgMaskMOG, mask, final_mask);
	srcROI.copyTo(dst1,mask);

	bitwise_not(mask,mask);

	dstROI.copyTo(dst2,mask);

	dstROI.setTo(0);

	dstROI=dst1+dst2;

	return dstROI; 
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

// NEW FOR OCCLUSION
bool is_black(cv::Vec3b p){
    int thresh = 110;
    if(p[0]>thresh && p[1]>thresh && p[2]>thresh){
        return true;
    }
    else{
        return false;
    }
}
bool is_white(cv::Vec3b p){
    int thresh = 90;
    if(p[0]<thresh && p[1]<thresh && p[2]<thresh){
        return true;
    }
    else{
        return false;
    }
}

cv::Point2f pullPointFromMarker(int markerId, int pointId,  std::vector<Marker> & markersFound){
	//locate the correct marker and place it into a var
	Marker temp;
	for (size_t i=0;i<markersFound.size();i++){
		if (markersFound[i].id == markerId)
			temp = markersFound[i];
	}
	return temp.points[pointId];
}

bool outerMarkersFound(std::vector<Marker> & markersFound){
	int counter = 0;
	for (size_t i=0;i<markersFound.size();i++){
		if (markersFound[i].id == 621 || markersFound[i].id == 623 || markersFound[i].id == 624 || markersFound[i].id == 626)
			counter++;
	}
	if (counter==4)
		return true;
	else
		return false;
}

cv::Point2f locateMissingPoint(std::vector<Marker> & markersFound,int pointId){
	cv::Point2f missingPt;
	cv::Point2f P1, P2, P3, P4;
	if (pointId == 621){
		if(!isMissing(markersFound,622))
			P1 = pullPointFromMarker(622, 3, markersFound);
		else
			P1 = pullPointFromMarker(623, 3, markersFound);
		if(!isMissing(markersFound,623))
			P2 = pullPointFromMarker(623, 0, markersFound);
		else
			P2 = pullPointFromMarker(622, 0, markersFound);
		if(!isMissing(markersFound,626)){
			P3 = pullPointFromMarker(626, 2, markersFound);
			P4 = pullPointFromMarker(626, 3, markersFound);
		}
		else{
			cv::Point2f Ptemp1,Ptemp2,Ptemp3,Ptemp4;
			Ptemp1 = pullPointFromMarker(624, 1, markersFound);
			Ptemp2 = pullPointFromMarker(624, 2, markersFound);
			Ptemp3 = pullPointFromMarker(625, 1, markersFound);
			Ptemp4 = pullPointFromMarker(625, 2, markersFound);
			P3 = approxPoint(Ptemp1,Ptemp2,Ptemp3,Ptemp4);
			Ptemp1 = pullPointFromMarker(624, 0, markersFound);
			Ptemp2 = pullPointFromMarker(624, 3, markersFound);
			Ptemp3 = pullPointFromMarker(625, 0, markersFound);
			Ptemp4 = pullPointFromMarker(625, 3, markersFound);
			P4 = approxPoint(Ptemp1,Ptemp2,Ptemp3,Ptemp4);
		}
	}
	else if (pointId == 623){
		if(!isMissing(markersFound,621))
			P1 = pullPointFromMarker(621, 3, markersFound);
		else
			P1 = pullPointFromMarker(622, 3, markersFound);
		if(!isMissing(markersFound,622))
			P2 = pullPointFromMarker(622, 0, markersFound);
		else
			P2 = pullPointFromMarker(621, 0, markersFound);
		if(!isMissing(markersFound,624)){
			P3 = pullPointFromMarker(624, 0, markersFound);
			P4 = pullPointFromMarker(624, 1, markersFound);
		}
		else{
			cv::Point2f Ptemp1,Ptemp2,Ptemp3,Ptemp4;
			Ptemp1 = pullPointFromMarker(626, 2, markersFound);
			Ptemp2 = pullPointFromMarker(626, 1, markersFound);
			Ptemp3 = pullPointFromMarker(625, 2, markersFound);
			Ptemp4 = pullPointFromMarker(625, 1, markersFound);
			P3 = approxPoint(Ptemp1,Ptemp2,Ptemp3,Ptemp4);
			Ptemp1 = pullPointFromMarker(626, 3, markersFound);
			Ptemp2 = pullPointFromMarker(626, 0, markersFound);
			Ptemp3 = pullPointFromMarker(625, 3, markersFound);
			Ptemp4 = pullPointFromMarker(625, 0, markersFound);
			P4 = approxPoint(Ptemp1,Ptemp2,Ptemp3,Ptemp4);
		}
	}
	else if (pointId == 624){
		if(!isMissing(markersFound,625))
			P1 = pullPointFromMarker(625, 1, markersFound);
		else
			P1 = pullPointFromMarker(626, 1, markersFound);
		if(!isMissing(markersFound,626))
			P2 = pullPointFromMarker(626, 2, markersFound);
		else
			P2 = pullPointFromMarker(625, 2, markersFound);
		if(!isMissing(markersFound,623)){
			P3 = pullPointFromMarker(623, 0, markersFound);
			P4 = pullPointFromMarker(623, 1, markersFound);
		}
		else{
			cv::Point2f Ptemp1,Ptemp2,Ptemp3,Ptemp4;
			Ptemp1 = pullPointFromMarker(621, 3, markersFound);
			Ptemp2 = pullPointFromMarker(621, 0, markersFound);
			Ptemp3 = pullPointFromMarker(622, 3, markersFound);
			Ptemp4 = pullPointFromMarker(622, 0, markersFound);
			P3 = approxPoint(Ptemp1,Ptemp2,Ptemp3,Ptemp4);
			Ptemp1 = pullPointFromMarker(621, 2, markersFound);
			Ptemp2 = pullPointFromMarker(621, 1, markersFound);
			Ptemp3 = pullPointFromMarker(622, 2, markersFound);
			Ptemp4 = pullPointFromMarker(622, 1, markersFound);
			P4 = approxPoint(Ptemp1,Ptemp2,Ptemp3,Ptemp4);
		}
	}
	//if 626 is missing
	else{
		if(!isMissing(markersFound,624))
			P1 = pullPointFromMarker(624, 1, markersFound);
		else
			P1 = pullPointFromMarker(625, 1, markersFound);
		if(!isMissing(markersFound,625))
			P2 = pullPointFromMarker(625, 2, markersFound);
		else
			P2 = pullPointFromMarker(624, 2, markersFound);
		if(!isMissing(markersFound,621)){
			P3 = pullPointFromMarker(621, 2, markersFound);
			P4 = pullPointFromMarker(621, 3, markersFound);
		}
		else{
			cv::Point2f Ptemp1,Ptemp2,Ptemp3,Ptemp4;
			Ptemp1 = pullPointFromMarker(623, 0, markersFound);
			Ptemp2 = pullPointFromMarker(623, 3, markersFound);
			Ptemp3 = pullPointFromMarker(622, 0, markersFound);
			Ptemp4 = pullPointFromMarker(622, 3, markersFound);
			P3 = approxPoint(Ptemp1,Ptemp2,Ptemp3,Ptemp4);
			Ptemp1 = pullPointFromMarker(623, 1, markersFound);
			Ptemp2 = pullPointFromMarker(623, 2, markersFound);
			Ptemp3 = pullPointFromMarker(622, 1, markersFound);
			Ptemp4 = pullPointFromMarker(622, 2, markersFound);
			P4 = approxPoint(Ptemp1,Ptemp2,Ptemp3,Ptemp4);
		}
	}
	missingPt = findPtUsingPts(P1,P2,P3,P4);

	return missingPt;
}

bool isMissing(std::vector<Marker> & markersFound,int missingId){
	bool missing = true;
	for(size_t i=0;i<markersFound.size();i++){
		if(markersFound[i].id == missingId)
			missing = false;
	}
	return missing;
}

cv::Point2f findPtUsingPts(cv::Point2f P1,cv::Point2f P2,cv::Point2f P3,cv::Point2f P4){
	float y1 = P1.y;
	float y3 = P3.y;
	float x1 = P1.x;
	float x3 = P3.x;
	float c1, c2;

	cv::Point2f pointToFind;
	// if slopes near vertical:
	if(abs(P2.x-P1.x)<0.1){
		//cout<<P2.x<<P1.x<<endl;
		c2 = (P4.y-P3.y)/(P4.x-P3.x);
		pointToFind.x = x1;
		pointToFind.y = y3+c2*(x1-x3);
	}
	else if (abs(P4.x-P3.x)<0.1){
		//cout<<"wrong 2"<<endl;
		c1 = (P2.y-P1.y)/(P2.x-P1.x);
		pointToFind.x = x3;
		pointToFind.y = y1+c1*(x3-x1);
	}
	// if slopes are non vertical
	else{
		//cout<<"right sub case"<<endl;
		c1 = (P2.y-P1.y)/(P2.x-P1.x);
		c2 = (P4.y-P3.y)/(P4.x-P3.x);
		pointToFind.x = (y3-y1+c1*x1-c2*x3)/(c1-c2);
		pointToFind.y = y1+c1*(pointToFind.x-x1);
	}
	return pointToFind;
}

int numOuterMarkersMissing(std::vector<Marker> & markersFound){
	int missing = 4;
	for(size_t i=0;i<markersFound.size();i++){
		if(markersFound[i].id == 621 || markersFound[i].id == 623 || markersFound[i].id == 624 || markersFound[i].id == 626)
			missing--;
	}
	return missing;
}

cv::Point2f approxPoint(cv::Point2f P1,cv::Point2f P2,cv::Point2f P3,cv::Point2f P4){
	cv::Point2f ptToReturn;
	// midpt of P3 and P4 + length of line from midpt of P1-P2 to P3-P4 + length of P3-P4/2 + (length of P3-P4 - P1-P2)/2
	ptToReturn.x = (P3.x + P4.x)/2.0 + ((P3.x + P4.x)/2.0 - (P1.x + P2.x)/2.0) + ((P4.x - P3.x)/2.0) + ((P4.x - P3.x)/2.0) - ((P2.x - P1.x)/2.0);
	ptToReturn.y = (P3.y + P4.y)/2.0 + ((P3.y + P4.y)/2.0 - (P1.y + P2.y)/2.0) + ((P4.y - P3.y)/2.0) + ((P4.y - P3.y)/2.0) - ((P2.y - P1.y)/2.0);
	return ptToReturn;
}
