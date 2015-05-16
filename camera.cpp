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

// FOR HAND GESTURE //
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include "myImage.hpp"
#include "roi.hpp"
#include "handGesture.hpp"
#include <vector>
#include <cmath>
#include "main.hpp"
#include <unistd.h>

// END HAND GESTURE //


// OPENGL
// OPENGL
// OPENGL
// OPENGL
// OPENGL

#include <GLUT/glut.h>
#include <pthread.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <cstdio>
#define pi 3.1415926


// OPENGL
// OPENGL
// OPENGL
// OPENGL
// OPENGL



using namespace cv;
using namespace std;

// ------------------------HAND GESTURE --------------------//
/* Global Variables  */

CameraCalibration calib1(981.722,991.469,592.84,374.235);

VideoCapture *cap = NULL; // open the default camera
VideoCapture *video = NULL; //open video
int same_location = 0;
int marker_x1 = 0;
int marker_y1 = 0;

Vector3 trans;
Matrix33 rot;
cv::Mat final_frame_2;
int finger_number;
int finger_num_array[10];
int finger_num_array_cnt=0;

int width = 640;
int height = 480;
cv::Mat image;
void idle();
void* startOCV(void* arg);
void estimatePosition(std::vector<Marker>& detectedMarkers);


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
    // GLfloat rotMatrix[16] = {rot.mat[0][0],-rot.mat[0][1],-rot.mat[0][2],0,
    //                          rot.mat[1][0],-rot.mat[1][1],-rot.mat[1][2],0,
    //                          rot.mat[2][0],-rot.mat[2][1],-rot.mat[2][2],0,
    //                          0,0,0,1,};
    //glMultMatrixf(rotMatrix);
    //glRotatef(-90.0, 1.0, 0.0, 0.0);

    //you will need to adjust your transformations to match the positions where
    //you want to draw your objects(i.e. chessboard center, chessboard corners)
    //glutSolidTeapot(0.5);
    
    //glutWireTeapot(0.3);
    if(same_location < 2){
    	if (finger_number > 1){
        glutWireSphere(.3, 100, 100);

      }
      else {
        glutWireIcosahedron();
      }  
      //glutWireCube(.3);
      //glutWireIcosahedron();
    }
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

  // if ( argc == 1 ) {
  //   // start video capture from camera
  //   cap = new cv::VideoCapture(0);
  // } else if ( argc == 2 ) {
  //   // start video capture from file
  //   cap = new cv::VideoCapture(argv[1]);
  // } else {
  //   fprintf( stderr, "usage: %s [<filename>]\n", argv[0] );
  //   return 1;
  // }

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
//OPENGL
//OPENGL
//OPENGL


int fontFace = FONT_HERSHEY_PLAIN;
int square_len;
int avgColor[NSAMPLES][3] ;
int c_lower[NSAMPLES][3];
int c_upper[NSAMPLES][3];
int avgBGR[3];
int nrOfDefects;
int iSinceKFInit;
struct dim{int w; int h;}boundingDim;
VideoWriter out;
Mat edges;
My_ROI roi1, roi2,roi3,roi4,roi5,roi6;
vector <My_ROI> roi;
vector <KalmanFilter> kf;
vector <Mat_<float> > measurement;

/* end global variables */

void init(MyImage *m){
	square_len=20;
	iSinceKFInit=0;
}

// change a color from one space to another
void col2origCol(int hsv[3], int bgr[3], Mat src){
	Mat avgBGRMat=src.clone();	
	for(int i=0;i<3;i++){
		avgBGRMat.data[i]=hsv[i];	
	}
	cvtColor(avgBGRMat,avgBGRMat,COL2ORIGCOL);
	for(int i=0;i<3;i++){
		bgr[i]=avgBGRMat.data[i];	
	}
}

void printText(Mat src, string text){
	int fontFace = FONT_HERSHEY_PLAIN;
	putText(src,text,Point(src.cols/2, src.rows/10),fontFace, 1.2f,Scalar(200,0,0),2);
}

void waitForPalmCover(MyImage* m){
    m->cap >> m->src;
	flip(m->src,m->src,1);
	roi.push_back(My_ROI(Point(m->src.cols/3, m->src.rows/6),Point(m->src.cols/3+square_len,m->src.rows/6+square_len),m->src));
	roi.push_back(My_ROI(Point(m->src.cols/4, m->src.rows/2),Point(m->src.cols/4+square_len,m->src.rows/2+square_len),m->src));
	roi.push_back(My_ROI(Point(m->src.cols/3, m->src.rows/1.5),Point(m->src.cols/3+square_len,m->src.rows/1.5+square_len),m->src));
	roi.push_back(My_ROI(Point(m->src.cols/2, m->src.rows/2),Point(m->src.cols/2+square_len,m->src.rows/2+square_len),m->src));
	roi.push_back(My_ROI(Point(m->src.cols/2.5, m->src.rows/2.5),Point(m->src.cols/2.5+square_len,m->src.rows/2.5+square_len),m->src));
	roi.push_back(My_ROI(Point(m->src.cols/2, m->src.rows/1.5),Point(m->src.cols/2+square_len,m->src.rows/1.5+square_len),m->src));
	roi.push_back(My_ROI(Point(m->src.cols/2.5, m->src.rows/1.8),Point(m->src.cols/2.5+square_len,m->src.rows/1.8+square_len),m->src));
	for(int i =0;i<50;i++){
    	m->cap >> m->src;
		flip(m->src,m->src,1);
		for(int j=0;j<NSAMPLES;j++){
			roi[j].draw_rectangle(m->src);
		}
		string imgText=string("Cover rectangles with palm");
		printText(m->src,imgText);	
		
		if(i==30){
		//	imwrite("./images/waitforpalm1.jpg",m->src);
		}

		imshow("img1", m->src);
		out << m->src;
        if(cv::waitKey(30) >= 0) break;
	}
}

int getMedian(vector<int> val){
  int median;
  size_t size = val.size();
  sort(val.begin(), val.end());
  if (size  % 2 == 0)  {
      median = val[size / 2 - 1] ;
  } else{
      median = val[size / 2];
  }
  return median;
}


void getAvgColor(MyImage *m,My_ROI roi,int avg[3]){
	Mat r;
	roi.roi_ptr.copyTo(r);
	vector<int>hm;
	vector<int>sm;
	vector<int>lm;
	// generate vectors
	for(int i=2; i<r.rows-2; i++){
    	for(int j=2; j<r.cols-2; j++){
    		hm.push_back(r.data[r.channels()*(r.cols*i + j) + 0]) ;
        	sm.push_back(r.data[r.channels()*(r.cols*i + j) + 1]) ;
        	lm.push_back(r.data[r.channels()*(r.cols*i + j) + 2]) ;
   		}
	}
	avg[0]=getMedian(hm);
	avg[1]=getMedian(sm);
	avg[2]=getMedian(lm);
}

void average(MyImage *m){
	m->cap >> m->src;
	flip(m->src,m->src,1);
	for(int i=0;i<30;i++){
		m->cap >> m->src;
		flip(m->src,m->src,1);
		cvtColor(m->src,m->src,ORIGCOL2COL);
		for(int j=0;j<NSAMPLES;j++){
			getAvgColor(m,roi[j],avgColor[j]);
			roi[j].draw_rectangle(m->src);
		}	
		cvtColor(m->src,m->src,COL2ORIGCOL);
		string imgText=string("Finding average color of hand");
		printText(m->src,imgText);	
		imshow("img1", m->src);
        if(cv::waitKey(30) >= 0) break;
	}
}

void initTrackbars(){
	for(int i=0;i<NSAMPLES;i++){
		c_lower[i][0]=15;
		c_upper[i][0]=25;
		c_lower[i][1]=40;
		c_upper[i][1]=50;
		c_lower[i][2]=90;
		c_upper[i][2]=90;
	}
}


void normalizeColors(MyImage * myImage){
	// copy all boundries read from trackbar
	// to all of the different boundries
	for(int i=1;i<NSAMPLES;i++){
		for(int j=0;j<3;j++){
			c_lower[i][j]=c_lower[0][j];	
			c_upper[i][j]=c_upper[0][j];	
		}	
	}
	// normalize all boundries so that 
	// threshold is whithin 0-255
	for(int i=0;i<NSAMPLES;i++){
		if((avgColor[i][0]-c_lower[i][0]) <0){
			c_lower[i][0] = avgColor[i][0] ;
		}if((avgColor[i][1]-c_lower[i][1]) <0){
			c_lower[i][1] = avgColor[i][1] ;
		}if((avgColor[i][2]-c_lower[i][2]) <0){
			c_lower[i][2] = avgColor[i][2] ;
		}if((avgColor[i][0]+c_upper[i][0]) >255){ 
			c_upper[i][0] = 255-avgColor[i][0] ;
		}if((avgColor[i][1]+c_upper[i][1]) >255){
			c_upper[i][1] = 255-avgColor[i][1] ;
		}if((avgColor[i][2]+c_upper[i][2]) >255){
			c_upper[i][2] = 255-avgColor[i][2] ;
		}
	}
}

void produceBinaries(MyImage *m){	
	Scalar lowerBound;
	Scalar upperBound;
	Mat foo;
	for(int i=0;i<NSAMPLES;i++){
		normalizeColors(m);
		lowerBound=Scalar( avgColor[i][0] - c_lower[i][0] , avgColor[i][1] - c_lower[i][1], avgColor[i][2] - c_lower[i][2] );
		upperBound=Scalar( avgColor[i][0] + c_upper[i][0] , avgColor[i][1] + c_upper[i][1], avgColor[i][2] + c_upper[i][2] );
		m->bwList.push_back(Mat(m->srcLR.rows,m->srcLR.cols,CV_8U));	
		inRange(m->srcLR,lowerBound,upperBound,m->bwList[i]);	
	}
	m->bwList[0].copyTo(m->bw);
	for(int i=1;i<NSAMPLES;i++){
		m->bw+=m->bwList[i];	
	}
	medianBlur(m->bw, m->bw,7);
}


void showWindows(MyImage m){
	pyrDown(m.bw,m.bw);
	pyrDown(m.bw,m.bw);
	Rect roi( Point( 3*m.src.cols/4,0 ), m.bw.size());
	vector<Mat> channels;
	Mat result;
	for(int i=0;i<3;i++)
		channels.push_back(m.bw);
	merge(channels,result);
	result.copyTo( m.src(roi));
	imshow("final",m.src);	
}

int findBiggestContour(vector<vector<Point> > contours){
    int indexOfBiggestContour = -1;
    int sizeOfBiggestContour = 0;
    for (int i = 0; i < contours.size(); i++){
        if(contours[i].size() > sizeOfBiggestContour){
            sizeOfBiggestContour = contours[i].size();
            indexOfBiggestContour = i;
        }
    }
    return indexOfBiggestContour;
}

void myDrawContours(MyImage *m,HandGesture *hg){
	//drawContours(m->src,hg->hullP,hg->cIdx,cv::Scalar(200,0,0),2, 8, vector<Vec4i>(), 0, Point());
	

	rectangle(m->src,hg->bRect.tl(),hg->bRect.br(),Scalar(0,0,200));
	vector<Vec4i>::iterator d=hg->defects[hg->cIdx].begin();
	int fontFace = FONT_HERSHEY_PLAIN;
		
	
	vector<Mat> channels;
		Mat result;
		for(int i=0;i<3;i++)
			channels.push_back(m->bw);
		merge(channels,result);
	//	drawContours(result,hg->contours,hg->cIdx,cv::Scalar(0,200,0),6, 8, vector<Vec4i>(), 0, Point());
		drawContours(result,hg->hullP,hg->cIdx,cv::Scalar(0,0,250),10, 8, vector<Vec4i>(), 0, Point());

		
	while( d!=hg->defects[hg->cIdx].end() ) {
   	    Vec4i& v=(*d);
	    int startidx=v[0]; Point ptStart(hg->contours[hg->cIdx][startidx] );
   		int endidx=v[1]; Point ptEnd(hg->contours[hg->cIdx][endidx] );
  	    int faridx=v[2]; Point ptFar(hg->contours[hg->cIdx][faridx] );
	    float depth = v[3] / 256;
   /*	
		line( m->src, ptStart, ptFar, Scalar(0,255,0), 1 );
	    line( m->src, ptEnd, ptFar, Scalar(0,255,0), 1 );
   		circle( m->src, ptFar,   4, Scalar(0,255,0), 2 );
   		circle( m->src, ptEnd,   4, Scalar(0,0,255), 2 );
   		circle( m->src, ptStart,   4, Scalar(255,0,0), 2 );
*/
   		circle( result, ptFar,   9, Scalar(0,205,0), 5 );
		
		
	    d++;

   	 }
//	imwrite("./images/contour_defects_before_eliminate.jpg",result);

}

void makeContours(MyImage *m, HandGesture* hg){
	Mat aBw;
	pyrUp(m->bw,m->bw);
	m->bw.copyTo(aBw);
	findContours(aBw,hg->contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	hg->initVectors(); 
	hg->cIdx=findBiggestContour(hg->contours);
	if(hg->cIdx!=-1){
//		approxPolyDP( Mat(hg->contours[hg->cIdx]), hg->contours[hg->cIdx], 11, true );
		hg->bRect=boundingRect(Mat(hg->contours[hg->cIdx]));		
		convexHull(Mat(hg->contours[hg->cIdx]),hg->hullP[hg->cIdx],false,true);
		convexHull(Mat(hg->contours[hg->cIdx]),hg->hullI[hg->cIdx],false,false);
		approxPolyDP( Mat(hg->hullP[hg->cIdx]), hg->hullP[hg->cIdx], 18, true );
		if(hg->contours[hg->cIdx].size()>3 ){
			convexityDefects(hg->contours[hg->cIdx],hg->hullI[hg->cIdx],hg->defects[hg->cIdx]);
			hg->eleminateDefects(m);
		}
		bool isHand=hg->detectIfHand();
		hg->printGestureInfo(m->src);
		if(isHand){	
			hg->getFingerTips(m);
			hg->drawFingerTips(m);
			myDrawContours(m,hg);
		}
	}
}

// ------------------------ END HAND GESTURE --------------------//

int marker_detection_wrapper(cv::Mat, std::vector<Marker> & ,cv::Mat &, std::vector<Marker> &, vector<cv::Point2f> &);
cv::Mat warping_wrapper(cv::Mat , cv::Mat , std::vector<Marker> & , vector<cv::Point2f> & );
cv::Mat update_frame(cv::Mat ,cv::Mat ,std::vector<Marker> & );

pair<float, float> find_velocity(queue<cv::Point>);

void* startOCV(void* arg) 
//int main( int argc, char** argv )
{
// ------------------------HAND GESTURE --------------------//
	MyImage m(0);		
	HandGesture hg;
	init(&m);		
	m.cap >>m.src;
    namedWindow("img1",CV_WINDOW_KEEPRATIO);
	//out.open("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 15, m.src.size(), true);
	waitForPalmCover(&m);
	average(&m);
	destroyWindow("img1");

    //namedWindow("trackbars",CV_WINDOW_AUTOSIZE); 
    initTrackbars();
// ------------------------HAND GESTURE END --------------------//


	//cv::Mat mountain_frame = cv::imread("Mountain.jpg",1);
    //VideoCapture cap(0); // open the default camera
    //cap = new cv::VideoCapture(0);
    //VideoCapture video("ball.MOV"); //open video

    // if(!cap.isOpened() || !video.isOpened())  // check if we succeeded
    //     return -1;
    
    //get size of the embedded video
    //int w = video.get(3);
    //int h = video.get(4);
    
    //cout << "width is " << w << endl;
    //cout << "height is "<< h << endl;

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

    float prevX = 0;
    float prevY = 0;
    
    //int same_location = 0;

    while(1)
    {
		hg.frameNumber++;
		m.cap >> m.src; 
		flip(m.src,m.src,1);
		pyrDown(m.src,m.srcLR);
		blur(m.srcLR,m.srcLR,Size(3,3));
		cvtColor(m.srcLR,m.srcLR,ORIGCOL2COL);
		produceBinaries(&m);
		cvtColor(m.srcLR,m.srcLR,COL2ORIGCOL);
		makeContours(&m, &hg);
		hg.getFingerNumber(&m);
    finger_num_array[finger_num_array_cnt] = hg.fingerTips.size();
    int finger_number_2 = 0;
    for (int i=1; i<10; ++i) finger_number_2 += finger_num_array[i];
    finger_num_array_cnt++;
    if (finger_num_array_cnt >= 10) finger_num_array_cnt = 0;
    finger_number = finger_number_2/10;

    cout << "finger num" << finger_number << "\n";

        count = count + 1;
        cout << count << endl;
        Mat frame_camera;
        //Mat frame_vid;
        frame_camera = m.src;
        
        // start looking for marker only after 20 frames to allow time for webcam to initialize
        if(count > 20){
            //bool vidSuccess = video.read(frame_vid); // read a new from from video 

                std::vector<Marker> markersFound;
                vector<cv::Point2f>  rPoints;
  
                rPoints.push_back(cv::Point2f(0,0));
                rPoints.push_back(cv::Point2f(299,0));
                rPoints.push_back(cv::Point2f(299,299));
                rPoints.push_back(cv::Point2f(0,299));
                //code for hand box goes here
                Marker m;
                std::vector<cv::Point2f> pts;
                cv::Point2f p;

                p.x = hg.bRect.x;
                p.y = hg.bRect.y;
                pts.push_back(p);

                p.x = hg.bRect.x + hg.bRect.width;
                p.y = hg.bRect.y; 
                pts.push_back(p);

                p.x = hg.bRect.x + hg.bRect.width;
                p.y = hg.bRect.y + hg.bRect.height;
                pts.push_back(p);

                p.x = hg.bRect.x; 
                p.y = hg.bRect.y + hg.bRect.height;
                pts.push_back(p);
              
                m.points = pts;

                markersFound.push_back(m);

                if(hg.bRect.x == prevX && hg.bRect.y == prevY){
                    cout << "same location" << endl;
                    same_location++;
                }
                else{
                    cout << "found again" << endl;
                    same_location = 0;
                }

                prevX = hg.bRect.x;
                prevY = hg.bRect.y;

                // if a marker is found, apply homography
                if(same_location < 2){
                    not_found_count = 0;
                    prev_markersFound = markersFound;
                    prev_rPoints = rPoints;

                    //cv::Mat dstImg = warping_wrapper(frame_camera, frame_vid, markersFound, rPoints);

                    P1 = cv::Point(int(markersFound[0].points[0].x),int(markersFound[0].points[0].y));
                    P2 = cv::Point(int(markersFound[0].points[1].x),int(markersFound[0].points[1].y));
                    P3 = cv::Point(int(markersFound[0].points[2].x),int(markersFound[0].points[2].y));
                    P4 = cv::Point(int(markersFound[0].points[3].x),int(markersFound[0].points[3].y));
                	estimatePosition(markersFound);

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
                    //final_frame = update_frame(frame_camera, dstImg, markersFound);
                	final_frame = frame_camera;
                }
                // if no marker found, just display the camera feed
                else{
                    // if there was a marker in the last n frames, show the marker 

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
                			estimatePosition(prev_markersFound);


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
                    // if no marker found for n continous frames, display the camera camera feed
                    else{
                        final_frame = frame_camera;
                    }
                }
            
        }
        // if still within the first 20 frames, display camera feed
        else{
            final_frame = frame_camera;
        }


        if(waitKey(30) == 27)
        {
            cout << "esc key is pressed by user" << endl; 
            break; 
        }

        final_frame.copyTo(final_frame_2);
        //final_frame = final_frame;
        //imshow("final", frame_camera);
        //usleep(10);
		//showWindows(m);
    }
    //return 0;
}

int marker_detection_wrapper(cv::Mat camera_frame, std::vector<Marker> & markers, cv::Mat & dstImg_save, std::vector<Marker> & markersFound, vector<cv::Point2f> & rPoints)
{
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


void estimatePosition(std::vector<Marker>& detectedMarkers)
{
    	cv::Mat camMatrix;
    	cv::Mat distCoeff;
    	cv::Mat(3,3, CV_32F, const_cast<float*>(&calib1.getIntrinsic().data[0])).copyTo(camMatrix);
    	cv::Mat(4,1, CV_32F, const_cast<float*>(&calib1.getDistorsion().data[0])).copyTo(distCoeff);

        Marker& m = detectedMarkers[0];
  		std::vector<cv::Point3f> m_markerCorners3d;

        m_markerCorners3d.push_back(cv::Point3f(-0.5f,-0.5f,0));
        m_markerCorners3d.push_back(cv::Point3f(+0.5f,-0.5f,0));
        m_markerCorners3d.push_back(cv::Point3f(+0.5f,+0.5f,0));
        m_markerCorners3d.push_back(cv::Point3f(-0.5f,+0.5f,0));
        cv::Mat Rvec;
        cv::Mat_<float> Tvec;
        cv::Mat raux,taux;
        cv::solvePnP(m_markerCorners3d, m.points, camMatrix, distCoeff,raux,taux);
        raux.convertTo(Rvec,CV_32F);
        taux.convertTo(Tvec ,CV_32F);

        cv::Mat_<float> rotMat(3,3);
        cv::Rodrigues(Rvec, rotMat);

        // Copy to transformation matrix
        for (int col=0; col<3; col++)
        {
            for (int row=0; row<3; row++)
            {
                m.transformation.r().mat[row][col] = rotMat(row,col); // Copy rotation component
            }
            m.transformation.t().data[col] = Tvec(col); // Copy translation component
        }

        // Since solvePnP finds camera location, w.r.t to marker pose, to get marker pose w.r.t to the camera we invert it.
        m.transformation = m.transformation.getInverted();
        //rot = m.transformation.r();
        trans = m.transformation.t();

    
}
