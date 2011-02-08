#include "testApp.h"
#define HOST "localhost"
#define PORT 12345

//--------------------------------------------------------------
void testApp::setup()
{
	//kinect.init(true);  //shows infrared image
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();
	
	colorImg.allocate(kinect.width, kinect.height);
	filteredColorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	bodyImage.allocate(kinect.width, kinect.height);
	handImage.allocate(kinect.width, kinect.height);
	grayThresh.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	nearThreshold = 50;
	farThreshold  = 180;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);

	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
	
	closepix = 0;
	
	sender.setup( HOST, PORT );
}

//--------------------------------------------------------------
void testApp::update()
{
	ofBackground(100, 100, 100);
	kinect.update();
	
	grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
	bodyImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
	handImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
	filteredColorImg.setFromPixels(kinect.getCalibratedRGBPixels(), kinect.width, kinect.height);
		
	//we do two thresholds - one for the far plane and one for the near plane
	//we then do a cvAnd to get the pixels which are a union of the two thresholds.	

	unsigned char * pix = grayImage.getPixels();
	unsigned char * bodyPix = bodyImage.getPixels();
	unsigned char * handPix = handImage.getPixels();
	int numPixels = grayImage.getWidth() * grayImage.getHeight();
	
	int bodyPixCount = 0;
	int bodyPixTotal = 0;
	
	for(int i = 0; i < numPixels; i++){
		if( pix[i] > nearThreshold && pix[i] < farThreshold ){
			bodyPix[i] = 255;
			++bodyPixCount;
			bodyPixTotal += pix[i];
		}
		else{
			bodyPix[i] = 0;
		}
	}
	
	if (bodyPixTotal > 0) {
		meanBodyPix = bodyPixTotal/bodyPixCount;
	}
	else {
		meanBodyPix = 0;
	}
	
	//let's find stuff within a range above the mean
	for(int i = 0; i < numPixels; i++){
		if( pix[i] > meanBodyPix+10 && pix[i] < meanBodyPix+60 ){
			handPix[i] = 255;
		}else{
			handPix[i] = 0;
		}
	}
	
	grayImage.mirror(false, true);
	bodyImage.mirror(false, true);
	handImage.mirror(false, true);
	filteredColorImg.mirror(false, true);

	//update the cv image
	grayImage.flagImageChanged();
	bodyImage.flagImageChanged();
	handImage.flagImageChanged();
	filteredColorImg.flagImageChanged();

    // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
    // also, find holes is set to true so we will get interior contours as well....
    contourFinder.findContours(handImage, 300, 20000, 100, false);
	
	if (contourFinder.nBlobs == 2) {
		ofPoint hands[2];
		hands[0] = contourFinder.blobs[0].centroid;
		hands[1] = contourFinder.blobs[1].centroid;
		sendHandPositions(hands);
	}
}

//--------------------------------------------------------------
void testApp::draw()
{
	ofSetColor(255, 255, 255);

	kinect.drawDepth(10, 10, 400, 300);
	kinect.draw(420, 10, 400, 300);
	
	bodyImage.draw(420, 320, 400, 300);
	filteredColorImg.draw(10, 320, 400, 300);
	contourFinder.draw(10, 320, 400, 300);
	
	ofPushMatrix();
	ofTranslate(420, 320);
	
	//drawPointCloud();
	ofPopMatrix();

	ofSetColor(255, 255, 255);
	/*ofDrawBitmapString("accel is: " + ofToString(kinect.getMksAccel().x, 2) + " / " 
									+ ofToString(kinect.getMksAccel().y, 2) + " / "
									+ ofToString(kinect.getMksAccel().z, 2), 20, 658 );*/
	ofDrawBitmapString("Body mean pix is: " + ofToString(meanBodyPix), 20, 650);

	char reportStr[1024];
	sprintf(reportStr, "using opencv threshold = %i (press spacebar)\nset near threshold %i (press: + -)\nset far threshold %i (press: < >) num blobs found %i, fps: %f",bThreshWithOpenCV, nearThreshold, farThreshold, contourFinder.nBlobs, ofGetFrameRate());
	ofDrawBitmapString(reportStr, 20, 690);
	ofDrawBitmapString("tilt angle: " + ofToString(angle),20,670);
}

void testApp::drawPointCloud() {
	ofScale(400, 400, 400);
	int w = 640;
	int h = 480;
	ofRotateY(mouseX);
	float* distancePixels = kinect.getDistancePixels();
	glBegin(GL_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			ofPoint cur = kinect.getWorldCoordinateFor(x, y);
			glVertex3f(cur.x, cur.y, cur.z);
		}
	}
	glEnd();
}

void testApp::sendHandPositions(ofPoint h[]) {
	ofxOscMessage m;
	m.setAddress( "/hands" );
	
	m.addFloatArg( h[0].x/kinect.width );
	m.addFloatArg( h[0].y/kinect.height );
	m.addFloatArg( h[1].x/kinect.width );
	m.addFloatArg( h[1].y/kinect.height );
	sender.sendMessage( m );
	cerr << "Sending message: " << h[0].x/kinect.width << ", " << h[0].y/kinect.height << ", " << h[1].x/kinect.width << ", " << h[1].y/kinect.height << endl;
	m.clear();
}

//--------------------------------------------------------------
void testApp::exit(){
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key)
{
	switch (key)
	{
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
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
void testApp::mouseMoved(int x, int y)
{}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}

