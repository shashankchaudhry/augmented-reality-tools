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

// OPENGL

#include <GLUT/glut.h>
#include <pthread.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <cstdio>
#define pi 3.1415926


// OPENGL


using namespace cv;
using namespace std;

int marker_detection_wrapper(cv::Mat, std::vector<Marker> & ,cv::Mat &, std::vector<Marker> &, vector<cv::Point2f> &);
cv::Mat warping_wrapper(cv::Mat , cv::Mat , std::vector<Marker> & , vector<cv::Point2f> & );
cv::Mat update_frame(cv::Mat ,cv::Mat ,std::vector<Marker> & );

pair<float, float> find_velocity(queue<cv::Point>);


//OPENGL

VideoCapture *cap = NULL; // open the default camera
VideoCapture *video = NULL; //open video
int marker_size = 0;
int marker_x1 = 0;
int marker_y1 = 0;

Vector3 trans;
Matrix33 rot;
cv::Mat final_frame_2;



int width = 640;
int height = 480;
cv::Mat image;
void idle();
void* startOCV(void* arg) ;

// a useful function for displaying your coordinate system
void drawAxes(float length)
{
  glPushAttrib(GL_POLYGON_BIT | GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT) ;

  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) ;
  glDisable(GL_LIGHTING) ;

  glBegin(GL_LINES) ;
  glColor3f(1,0,0) ;
  glVertex3f(0,0,0) ;
  glVertex3f(length,0,0);

  glColor3f(0,1,0) ;
  glVertex3f(0,0,0) ;
  glVertex3f(0,length,0);

  glColor3f(0,0,1) ;
  glVertex3f(0,0,0) ;
  glVertex3f(0,0,length);
  glEnd() ;


  glPopAttrib() ;
}

void display()
{
  // clear the window
  glClear( GL_COLOR_BUFFER_BIT );

  // show the current camera frame

  //based on the way cv::Mat stores data, you need to flip it before displaying it
  cv::Mat tempimage;
  cv::flip(image, tempimage, 0);
  glDrawPixels( tempimage.size().width, tempimage.size().height, GL_BGR, GL_UNSIGNED_BYTE, tempimage.ptr() );

  //////////////////////////////////////////////////////////////////////////////////
  // Here, set up new parameters to render a scene viewed from the camera.

  //set viewport
  glViewport(0, 0, tempimage.size().width, tempimage.size().height);

  //set projection matrix using intrinsic camera params
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  //gluPerspective is arbitrarily set, you will have to determine these values based
  //on the intrinsic camera parameters
  gluPerspective(41.35, tempimage.size().width*1.0/tempimage.size().height, 1, 2000); 

  //you will have to set modelview matrix using extrinsic camera params
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  //gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);  
  

  /////////////////////////////////////////////////////////////////////////////////
  // Drawing routine

  //now that the camera params have been set, draw your 3D shapes
  //first, save the current matrix
  glPushMatrix();
    //move to the position where you want the 3D object to go
    //glRotatef(90.0, 1.0, 0.0, 0.0);

    glTranslatef(-trans.data[0], trans.data[1], trans.data[2]); //this is an arbitrary position for demonstration
    //glRotatef(90.0, 0.0, 1.0, 0.0);
    //glRotatef(0.0, 0.0, 0.0, 1.0);
    GLfloat rotMatrix[16] = {rot.mat[0][0],-rot.mat[0][1],-rot.mat[0][2],0,
                             rot.mat[1][0],-rot.mat[1][1],-rot.mat[1][2],0,
                             rot.mat[2][0],-rot.mat[2][1],-rot.mat[2][2],0,
                             0,0,0,1,};
    glMultMatrixf(rotMatrix);
    glRotatef(-90.0, 1.0, 0.0, 0.0);

    //you will need to adjust your transformations to match the positions where
    //you want to draw your objects(i.e. chessboard center, chessboard corners)
    //glutSolidTeapot(0.5);
    if (marker_size > 0){
      glutWireTeapot(0.3);
    }
    //glutWireSphere(.3, 100, 100);
    //drawAxes(1.0);
    //glRotatef(90.0, 0.0, 1.0, 0.0);
  glPopMatrix();
  

  // show the rendering on the screen
  glutSwapBuffers();

  // post the next redisplay
  glutPostRedisplay();
}

void reshape( int w, int h )
{
  // set OpenGL viewport (drawable area)
  glViewport( 0, 0, w, h );
}

void mouse( int button, int state, int x, int y )
{
  if ( button == GLUT_LEFT_BUTTON && state == GLUT_UP )
    {

    }
}

void keyboard( unsigned char key, int x, int y )
{
  switch ( key )
    {
    case 'q':
      // quit when q is pressed
      exit(0);
      break;

    default:
      break;
    }
}

void idle()
{
  // grab a frame from the camera
  final_frame_2.copyTo(image);
}

int main( int argc, char **argv )
{
  int w,h;

  cap = new cv::VideoCapture(0);
  //video = new cv::VideoCapture("ball.MOV");

  // check that video is opened
  if ( cap == NULL || !cap->isOpened() ) {
    fprintf( stderr, "could not start video capture\n" );
    return 1;
  }

  // get width and height
  w = (int) cap->get( CV_CAP_PROP_FRAME_WIDTH );
  h = (int) cap->get( CV_CAP_PROP_FRAME_HEIGHT );
  // On Linux, there is currently a bug in OpenCV that returns 
  // zero for both width and height here (at least for video from file)
  // hence the following override to global variable defaults: 
  width = w ? w : width;
  height = h ? h : height;


pthread_t tId;
pthread_attr_t tAttr;
pthread_attr_init(&tAttr);
pthread_create(&tId, &tAttr, startOCV, NULL);


  // initialize GLUT
  glutInit( &argc, argv );
  glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE );
  glutInitWindowPosition( 20, 20 );
  glutInitWindowSize( width, height );
  glutCreateWindow( "OpenGL / OpenCV Example" );

  // set up GUI callback functions
  glutDisplayFunc( display );
  glutReshapeFunc( reshape );
  glutMouseFunc( mouse );
  glutKeyboardFunc( keyboard );
  glutIdleFunc( idle );

  // start GUI loop
  glutMainLoop();

  return 0;
}


//OPENGL


void* startOCV(void* arg) 
{

	cv::Mat mountain_frame = cv::imread("Mountain.jpg",1);
    //VideoCapture cap(0); // open the default camera
    VideoCapture video("ball.MOV"); //open video

    if(!video.isOpened())  // check if we succeeded
        exit(0);
    
    //get size of the embedded video
    int w = video.get(3);
    int h = video.get(4);
    
    cout << "width is " << w << endl;
    cout << "height is "<< h << endl;

    std::vector<Marker> markersFound;

    std::vector<Marker> temp;

    cv::Mat dstImg;

    int count=0;
    //int marker_size;
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
        bool bSuccess = cap->read(frame_camera); // read a new frame from webcam

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

                //

                //

                resize(frame_vid, frame_vid, Size(300,300),0,0, INTER_CUBIC);
  
                marker_size = marker_detection_wrapper(frame_camera, markersFound, dstImg, markersFound, rPoints);
                
                // if a marker is found, apply homography
                if(marker_size>0){
                    not_found_count = 0;
                    prev_markersFound = markersFound;
                    prev_rPoints = rPoints;

                    //cv::Mat dstImg = warping_wrapper(frame_camera, frame_vid, markersFound, rPoints);

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

                    final_frame = frame_camera;
                    //final_frame = update_frame(frame_camera, dstImg, markersFound);
                
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
                            //cv::Mat dstImg = warping_wrapper(frame_camera, frame_vid, prev_markersFound, prev_rPoints);
                            //final_frame = update_frame(frame_camera, dstImg, prev_markersFound);
                            final_frame = frame_camera;
                        
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
        //imshow("MyWindow", final_frame); //show the frame in "MyVideo" window
        
        // image = final_frame;
        final_frame.copyTo(final_frame_2);

        if(waitKey(30) == 27)
        {
            cout << "esc key is pressed by user" << endl; 
            break; 
        }

    }
}


int marker_detection_wrapper(cv::Mat camera_frame, std::vector<Marker> & markers, cv::Mat & dstImg_save, std::vector<Marker> & markersFound, vector<cv::Point2f> & rPoints)
{
	CameraCalibration calib1(981.722,991.469,592.84,374.235);
	MarkerDetector marker1(calib1);
	//string frameName = "/Users/Leech/Dropbox/Project/Cheng's Folder/Marker_Detection/photo2.jpg";
	int frameHeight = camera_frame.rows;
	int frameWidth = camera_frame.cols;
	//cout<<"things worked"<<endl;

    marker1.processFrame(camera_frame);
    markersFound = marker1.returnDetectedMarkers();
    markers = markersFound;
    //
    marker_size = markersFound.size();
    if (marker_size > 0){
        vector<Transformation> newTransformation = marker1.getTransformations();
        trans = newTransformation[0].t();
        rot = newTransformation[0].r();
            
    }
    //float[][] rotation = newTransformation[0].r().mat;
    //float[] translation = newTransformation[0].t().data;

    rPoints.push_back(cv::Point2f(0,0));
    rPoints.push_back(cv::Point2f(299,0));
    rPoints.push_back(cv::Point2f(299,299));
    rPoints.push_back(cv::Point2f(0,299));

    return markers.size();
}


cv::Mat warping_wrapper(cv::Mat camera_frame, cv::Mat video_frame, std::vector<Marker> & markersFound, vector<cv::Point2f> & rPoints){
    //cout << "main called" << endl;
    cv::Mat dstImg;
    int frameHeight = camera_frame.rows;
	int frameWidth = camera_frame.cols;

    cv::Mat homography = cv::getPerspectiveTransform(rPoints,markersFound[0].points);
    cv::Mat srcImg = video_frame;//cv::imread("Mountain.jpg",1);
    cv::Size dSize(frameWidth,frameHeight);
    cv::warpPerspective(srcImg,dstImg,homography,dSize);

    return dstImg;
}


cv::Mat update_frame(cv::Mat camera_frame, cv::Mat dstImg, std::vector<Marker> & markersFound){
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
