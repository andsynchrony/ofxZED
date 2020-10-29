#include "ofApp.h"

#define STRINGIFY(x) #x

static string depthFragmentShader =
STRINGIFY(
	uniform sampler2DRect tex;
void main()
{
	vec4 col = texture2DRect(tex, gl_TexCoord[0].xy);
	float value = col.r;
	float low1 = 0.5;
	float high1 = 5.0;
	float low2 = 1.0;
	float high2 = 0.0;
	float d = clamp(low2 + (value - low1) * (high2 - low2) / (high1 - low1), 0.0, 1.0);
	if (d == 1.0) {
		d = 0.0;
	}
	gl_FragColor = vec4(vec3(d), 1.0);
}
);

static string colorFragmentShader =
STRINGIFY(
	uniform sampler2DRect tex;
void main()
{
	vec4 col = texture2DRect(tex, gl_TexCoord[0].xy);
	gl_FragColor = vec4(col.b, col.g, col.r, col.a);
}
);

//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetFrameRate(60);

	zed.init(true, true, true, 0, sl::DEPTH_MODE::PERFORMANCE);

	depthShader.setupShaderFromSource(GL_FRAGMENT_SHADER, depthFragmentShader);
	depthShader.linkProgram();

	colorShader.setupShaderFromSource(GL_FRAGMENT_SHADER, colorFragmentShader);
	colorShader.linkProgram();
}

//--------------------------------------------------------------
void ofApp::update()
{
	zed.update();
}

//--------------------------------------------------------------
void ofApp::draw()
{
	colorShader.begin();
	zed.getColorLeftTexture().draw(0, 0);
	zed.getColorRightTexture().draw(zed.zedWidth, 0);
	colorShader.end();

	depthShader.begin();
	zed.getDepthLeftTexture().draw(0, zed.zedHeight);
	zed.getDepthRightTexture().draw(zed.zedWidth, zed.zedHeight);
	depthShader.end();

	cam.begin();
	ofPushMatrix();
	ofScale(100, 100, 100);
	ofMultMatrix(zed.getTrackedPose());
	ofDrawAxis(0.3);
	ofDrawBox(0.1);
	ofPopMatrix();

	ofDrawAxis(100);
	cam.end();

	ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate()), 10, 20);
	{
		stringstream ss;
		ss << "cam fps : " << zed.getCurrentFPS() << endl;
		auto pose = zed.getTrackedPose();
		ss << "pos:" << pose.getTranslation() << endl;
		ss << "rot:" << pose.getRotate().getEuler();
		ofDrawBitmapStringHighlight(ss.str(), 10, 40);
	}
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
	//ofLog() << "Depth at (" << x << ", " << y << ") is " << zed.getDepthAtPoint(x, y) << " mm" << endl;
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
