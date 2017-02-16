#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    receive.setup(PORT);
    sender.setup(HOST, PORT2);
    ofSetFrameRate(60);
    
    ofDirectory dir;
    int nFiles = dir.listDir("images");
    if(nFiles) {
        
        for(int i=0; i<dir.size(); i++) {
            
            // add the image to the vector
            string filePath = dir.getPath(i);
            images.push_back(ofImage());
            images.back().load(filePath);
            
        }
        
    }
    else ofLog(OF_LOG_WARNING) << "Could not find folder";
    // this toggle will tell the sequence
    // be be indepent of the app fps
    bFrameIndependent = true;
    
    // this will set the speed to play
    // the animation back we set the
    // default to 24fps
    sequenceFPS = 7;
    
    // set the app fps
    appFPS = 60;
    ofSetFrameRate(appFPS);

    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();		// opens first available kinect
    //kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        //ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        //ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        //ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        //ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
#ifdef USE_TWO_KINECTS
    kinect2.init();
    kinect2.open();
#endif
    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = 230;
    farThreshold = 110;
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 4;
    kinect.setCameraTiltAngle(angle);
    
    // start from the front
    bDrawPointCloud = false;

}

//--------------------------------------------------------------
void ofApp::update(){
    while(receive.hasWaitingMessages()) {
        ofxOscMessage m;
        receive.getNextMessage(m);
        
        if(m.getAddress() == "/BrainWave/Raw") {
            raw = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Raw: " + ofToString(raw));
        }
        if (m.getAddress() == "/BrainWave/Theta") {
            theta = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Theta: " + ofToString(theta));
        }
        if (m.getAddress() == "/BrainWave/Alpha1") {
            alpha1 = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Alpha1: " + ofToString(alpha1));
        }
        if (m.getAddress() == "/BrainWave/Alpha2") {
            alpha2 = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Alpha2: " + ofToString(alpha2));
        }
        if (m.getAddress() == "/BrainWave/Attention") {
            //attention = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Attention: " + ofToString(attention));
        }
        if (m.getAddress() == "/BrainWave/Beta1") {
            beta1 = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Beta1: " + ofToString(beta1));
        }
        if (m.getAddress() == "/BrainWave/Beta2") {
            beta2 = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Beta2: " + ofToString(beta2));
        }
        if (m.getAddress() == "/BrainWave/Blink") {
            //blink = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Blink: " + ofToString(blink));
        }
        if (m.getAddress() == "/BrainWave/Contact") {
            //contact = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Contact: " + ofToString(contact));
        }
        if (m.getAddress() == "/BrainWave/Delta") {
            delta = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Delta: " + ofToString(delta));
        }
        if (m.getAddress() == "/BrainWave/Gamma1") {
            gamma1 = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Gamma1: " + ofToString(gamma1));
        }
        if (m.getAddress() == "/BrainWave/Gamma2") {
            gamma2 = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Gamma2: " + ofToString(gamma2));
        }
        if (m.getAddress() == "/BrainWave/Meditation") {
            meditation = m.getArgAsFloat(0);
            //ofLog(OF_LOG_NOTICE, "Meditation: " + ofToString(meditation));
        }
        if (m.getAddress() == "/BrainWave/TotalActivity") {
            total = m.getArgAsFloat(0)/400;
            //ofLog(OF_LOG_NOTICE, "Total: " + ofToString(total));
        }
    }
    
    //ofBackground(0, 0, 0);
    ofBackground(100, 100, 100);
    
    kinect.update();
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels());
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            ofPixels & pix = grayImage.getPixels();
            int numPixels = pix.size();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        // update the cv images
        grayImage.flagImageChanged();
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
        
    }
    
#ifdef USE_TWO_KINECTS
    kinect2.update();
#endif
    
    

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255, 255, 255);
    int wScreen = ofGetWidth();
    int hScreen = ofGetHeight();
    // we need some images if not return
    if((int)images.size() <= 0) {
        ofSetColor(255);
        ofDrawBitmapString("No Images...", 150, ofGetHeight()/2);
        return;
    }
    
    // this is the total time of the animation based on fps
    //float totalTime = images.size() / sequenceFPS;
    
    uint64_t frameIndex = 0;
    
    if(bFrameIndependent) {
        // calculate the frame index based on the app time
        // and the desired sequence fps. then mod to wrap
        frameIndex = (int)(ofGetElapsedTimef() * sequenceFPS) % images.size();
    }
    else {
        // set the frame index based on the app frame
        // count. then mod to wrap.
        frameIndex = ofGetFrameNum() % images.size();
    }
    
    // draw the image sequence at the new frame count
    ofSetColor(255);
    images[frameIndex].draw(0, 0, wScreen, hScreen);
    
    
    // draw where we are in the sequence
    float x = 0;
    for(int offset = 0; offset < 5; offset++) {
        int i = (frameIndex + offset) % images.size();
        ofSetColor(255);
        //images[i].draw(200+x, ofGetHeight()-40, wScreen, hScreen);
        x += 40;
    }

    
    if(bDrawPointCloud) {
        easyCam.begin();
        //ofPushMatrix();
        //ofRotate(ofGetFrameNum()*theta);
        
        drawPointCloud();
        //ofPopMatrix();
        easyCam.end();
    } else {
        // draw from the live kinect
        kinect.drawDepth(10, 10, 400, 300);
        kinect.draw(420, 10, 400, 300);
        
        grayImage.draw(10, 320, 400, 300);
        contourFinder.draw(10, 320, 400, 300);
        
#ifdef USE_TWO_KINECTS
        kinect2.draw(420, 320, 400, 300);
#endif
    }
    
    // draw instructions
    ofSetColor(255, 255, 255);
    stringstream reportStream;
    
    if(kinect.hasAccelControl()) {
//        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
//        << ofToString(kinect.getMksAccel().y, 2) << " / "
//        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
//        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
//        << "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
//    reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
//    << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
//    << "set near threshold " << nearThreshold << " (press: + -)" << endl
//    << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
//    << ", fps: " << ofGetFrameRate() << endl
//    << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
    
    if(kinect.hasCamTiltControl()) {
//        reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
//        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
    ofDrawBitmapString(reportStream.str(), 20, 652);

}

void ofApp::drawPointCloud() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    int step=2, ptSize= 1;
    int oldStep;
    float colorM=1;
    float oldColor;
    bool shadowPlay = false;
    int xD, yD;
    mesh.setMode(OF_PRIMITIVE_LINE_LOOP);
    for (int i = 0; i < contourFinder.nBlobs; i++){
        //ofLog(OF_LOG_NOTICE, "X: " + ofToString(contourFinder.blobs[i].boundingRect.getCenter().x));
        //ofLog(OF_LOG_NOTICE, "Y: " + ofToString(contourFinder.blobs[i].boundingRect.getCenter().y));
        
        
        // draw over the centroid if the blob is a hole
        
        if(contourFinder.blobs[i].boundingRect.getCenter().x <= 80){
            shadowPlay = true;
            ofLog(OF_LOG_NOTICE, "YES ");
        }
        else {
            shadowPlay = false;
        }
    }
    
    if(gamma1*100 < 0.8 && gamma1*100 > 0.6){
        sequenceFPS = 6;
        
    }
    else if(gamma1*100 < 0.6){
        sequenceFPS = 11;
        
    }
    else if(gamma1*100 > 0.8 && gamma1*100 < 1){
        sequenceFPS = 4;
        
    }
    else{
        sequenceFPS = 7;
    }
    
    
    if((beta1+beta2) > (alpha1+alpha2) && (beta1+beta2) > theta){
        step = 3;
        colorM = 30*beta1;
        ptSize = 4;
        mesh.setMode(OF_PRIMITIVE_TRIANGLE_FAN);
        ofLog(OF_LOG_NOTICE, "BETA");
    }
    if((alpha1+alpha2) > (beta1+beta2) && (alpha1+alpha2) > theta){
        step = 6;
        colorM = 50*alpha1;
        ptSize = 7;
        mesh.setMode(OF_PRIMITIVE_POINTS);
        ofLog(OF_LOG_NOTICE, "ALPHA");
    }
    if(theta > (alpha1+alpha2) && theta > (beta1+beta2)){
        step = 10;
        colorM = 64*theta;
        ptSize = 10;
        mesh.setMode(OF_PRIMITIVE_LINE_LOOP);
        ofLog(OF_LOG_NOTICE, "THETA");
    }
    
    oldStep =step;
    oldColor = colorM;
    
    if(shadowPlay){
        xD= 100;
        yD= 100;
        step = 1;
        colorM = 1;
    }
    else {
        xD = 0;
        yD = 0;
        step = oldStep;
        colorM = oldColor;
    }
    
   
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0 && kinect.getDistanceAt(x, y) < 2800) {
                //here we can check distance of performer
//                if(kinect.getDistanceAt(x, y) == 1000){
//                    step = 2;
//                    ofLog(OF_LOG_NOTICE, "Yo");
//                    ofxOscMessage m2;
//                    m2.setAddress("/test");
//                    m2.addFloatArg(1.5);
//                    m2.addFloatArg(ofGetElapsedTimef());
//                    sender.sendMessage(m2, false);
//                }
                mesh.addColor(kinect.getColorAt(x,y)*colorM);
                mesh.addVertex(kinect.getWorldCoordinateAt(x+xD, y+yD));
            }
        }
    }
    
    
    //here I am getting mesh x,y,zverticies which we can check for boundaries
//    vector<ofVec3f> test = mesh.getVertices();
//    for(int i=0;i<test.size(); i++){
//        ofLog(OF_LOG_NOTICE, "V: " + ofToString(test[i]));
//    }
    glPointSize(ptSize);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(-1, -1, -1);
    ofTranslate(0, 0, -1500); // center the points a bit
    //ofTranslate(100, 100, 0);
    ofEnableDepthTest();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
}

void ofApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
#ifdef USE_TWO_KINECTS
    kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key) {
        case ' ':
            bThreshWithOpenCV = !bThreshWithOpenCV;
            break;
            
        case'p':
            bDrawPointCloud = !bDrawPointCloud;
            break;
            
        case '>':
        case '.':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
        case '=':
            nearThreshold ++;
            if (nearThreshold > 255) nearThreshold = 255;
            break;
            
        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;
            
        case 'w':
            kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            break;
            
        case 'o':
            kinect.setCameraTiltAngle(angle); // go back to prev tilt
            kinect.open();
            break;
            
        case 'c':
            kinect.setCameraTiltAngle(0); // zero the tilt
            kinect.close();
            break;
            
        case '1':
            kinect.setLed(ofxKinect::LED_GREEN);
            break;
            
        case '2':
            kinect.setLed(ofxKinect::LED_YELLOW);
            break;
            
        case '3':
            kinect.setLed(ofxKinect::LED_RED);
            break;
            
        case '4':
            kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            break;
            
        case '5':
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;
            
        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
            break;
            
        case OF_KEY_UP:
            angle++;
            if(angle>30) angle=30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case OF_KEY_DOWN:
            angle--;
            if(angle<-30) angle=-30;
            kinect.setCameraTiltAngle(angle);
            break;
    }


}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
