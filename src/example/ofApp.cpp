#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	zed.init(true, true, 0, sl::zed::PERFORMANCE, sl::zed::VGA, 60);
	//zed.init();
}

//--------------------------------------------------------------
void ofApp::update()
{
	zed.update();
	//ofLog() << "FPS: " << zed.getCurrentFPS() << endl;
}

//--------------------------------------------------------------
void ofApp::draw()
{
	zed.getColorTexture()->draw(0, 0);
	zed.getDepthTexture()->draw(zed.zedWidth, 0);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
	ofLog() << "Depth at (" << x << ", " << y << ") is " << zed.getDepthAtPoint(x, y) << " mm" << endl;
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
