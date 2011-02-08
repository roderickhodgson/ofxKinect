#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

class testApp : public ofBaseApp
{

	public:

		void setup();
		void update();
		void draw();
		void exit();
	
		void drawPointCloud();

		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void sendHandPositions(ofPoint h[]);

		ofxKinect kinect;

		ofxCvColorImage		colorImg;
		ofxCvColorImage		filteredColorImg;
	
	ofxCvGrayscaleImage 	grayImage;
	ofxCvGrayscaleImage 	bodyImage;
	ofxCvGrayscaleImage 	handImage;
		ofxCvGrayscaleImage 	grayThresh;
		ofxCvGrayscaleImage 	grayThreshFar;

		ofxCvContourFinder 	contourFinder;
	ofxCvBlob			blob;
	
	ofxOscSender sender;
		
		bool				bThreshWithOpenCV;
	
	float meanBodyPix;

		int 				nearThreshold;
		int					farThreshold;
		int					closepix;

		int					angle;
};

#endif
