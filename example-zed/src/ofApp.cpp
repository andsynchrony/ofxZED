#include "ofApp.h"

#define STRINGIFY(x) #x

static string depthFragmentShader =
STRINGIFY(
	uniform sampler2DRect tex;
void main()
{
	vec4 col = texture2DRect(tex, gl_TexCoord[0].xy);
	float value = col.r;
	float low1 = 0.1;
	float high1 = 20.0;
	float low2 = 1.0;
	float high2 = 0.0;
	float d = clamp(low2 + (value - low1) * (high2 - low2) / (high1 - low1), 0.0, 1.0);
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
	ofSetFrameRate(0);

	//zed.init(true, true, false, false, 0, sl::DEPTH_MODE::PERFORMANCE, sl::RESOLUTION::HD720, 0.0);
	//zed.init(true, true, false, false, 0, sl::DEPTH_MODE::NONE, sl::RESOLUTION::VGA, 100.0);
	sl::InitParameters init_params;
	init_params.camera_resolution = sl::RESOLUTION::HD720;
	init_params.camera_fps = 60;
	init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
	init_params.coordinate_units = sl::UNIT::METER;
	init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	init_params.input.setFromCameraID(0);
	init_params.enable_right_side_measure = true;
	init_params.sdk_verbose = 10;

	sl::RuntimeParameters runtime_params;
	runtime_params.enable_depth = true;
	runtime_params.sensing_mode = sl::SENSING_MODE::FILL;
	if (zed.open(init_params, runtime_params)) {
		sl::PositionalTrackingParameters pt_params;
		pt_params.set_as_static = true;
		zed.enablePositionalTracking(pt_params);

		sl::ObjectDetectionParameters obj_det_params;
		obj_det_params.detection_model = sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE;
		obj_det_params.body_format = sl::BODY_FORMAT::POSE_18;
		sl::ObjectDetectionRuntimeParameters obj_det_params_rt;
		zed.enableObjectDetection(obj_det_params, obj_det_params_rt);
	}

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

	zed.debugDrawObjectDetectionResult2D();

	cam.begin();
	ofPushMatrix();
	ofScale(100, 100, 100);

	ofPushMatrix();
	ofMultMatrix(zed.getTrackedPose());
	ofDrawAxis(0.3);
	ofDrawBox(0.1);
	ofPopMatrix();

	zed.debugDrawObjectDetectionResult3D();
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
	if (key == 'd') {
		zed.disableObjectDetection();
	}
	if (key == 'e') {
		sl::ObjectDetectionParameters obj_det_params;
		obj_det_params.detection_model = sl::DETECTION_MODEL::HUMAN_BODY_FAST;
		sl::ObjectDetectionRuntimeParameters obj_det_params_rt;
		zed.enableObjectDetection(obj_det_params, obj_det_params_rt);
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
