#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

#define PORT 12345
#define HOST "localhost"
#define PORT2 12346

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        void exit();
    
        void drawPointCloud();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        int   appFPS;
        float sequenceFPS;
        bool  bFrameIndependent;
        vector <ofImage> images;

    
        ofxOscReceiver receive;
        ofxOscSender sender;
        float alpha1 = 0.0;
        float alpha2 = 0.0;
        float attention = 0.0;
        float beta1 = 0.0;
        float beta2 = 0.0;
        float blink = 0.0;
        float contact = 0.0;
        float delta = 0.0;
        float gamma1 = 0.0;
        float gamma2 = 0.0;
        float meditation = 0.0;
        float raw = 0.0;
        float theta = 0.0;
        float total = 0.0;
    
        ofxKinect kinect;
    
        #ifdef USE_TWO_KINECTS
            ofxKinect kinect2;
        #endif
    
        ofxCvColorImage colorImg;
    
        ofxCvGrayscaleImage grayImage; // grayscale depth image
        ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
        ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
        ofxCvContourFinder contourFinder;
    
        int wScreen = ofGetWidth();
        int hScreen = ofGetHeight();
    
        bool bThreshWithOpenCV;
        bool bDrawPointCloud;
    
        int nearThreshold;
        int farThreshold;
    
        int angle;
    
        // used for viewing the point cloud
        ofEasyCam easyCam;

		
};
