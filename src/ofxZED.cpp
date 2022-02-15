#include "ofxZED.h"
namespace ofxZED
{
	Camera::Camera()
	{
	}

	Camera::~Camera()
	{
		close();
	}

	void Camera::init(bool useColorImage, bool useDepthImage, bool useTracking, bool useSensor, 
		int cameraID, sl::DEPTH_MODE mode, sl::RESOLUTION resolution, float fps,
		bool enableRightSideMeasure, float minDepth, float maxDepth, int sdkGpuId)
	{
		try
		{
			// Set configuration parameters
			sl::InitParameters init_params;
			init_params.camera_resolution = resolution;
			init_params.camera_fps = fps;
			init_params.depth_mode = mode;
			init_params.enable_right_side_measure = enableRightSideMeasure;
			init_params.input.setFromCameraID(cameraID);
			init_params.coordinate_units = sl::UNIT::METER;
			init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
			init_params.sensors_required = bUseSensor;
			init_params.depth_minimum_distance = minDepth;
			init_params.depth_maximum_distance = maxDepth;
			init_params.sdk_gpu_id = sdkGpuId;

			bUseColorImage = useColorImage;
			bUseDepthImage = useDepthImage;
			bEnableRightSideMeasure = enableRightSideMeasure;

			if (bUseDepthImage)
			{
				rt.enable_depth = true;
				rt.sensing_mode = sl::SENSING_MODE::FILL;
			}
			else if (bUseColorImage)
			{
				rt.enable_depth = false;
			}

			auto opened = this->open(init_params, rt);
			if (!opened) {
				return;
			}

			if (useTracking) {
				this->enablePositionalTracking(sl::PositionalTrackingParameters());
			}

			if (useSensor) {
				this->enableSensorThread();
			}
		}
		catch (exception& err) {
			cerr << err.what() << endl;
		}
	}

	bool Camera::open(sl::InitParameters init_params, sl::RuntimeParameters runtime_params, bool bStartThread)
	{
		try
		{
			this->rt = runtime_params;
			bEnableRightSideMeasure = init_params.enable_right_side_measure;
			bUseDepthImage = rt.enable_depth;

			ofLog() << "Initializing ZED camera." << std::endl;

			zed = new sl::Camera();
			ofLog() << "Resolution Mode:" << init_params.camera_resolution << std::endl;

			auto zedsuccess = handleZedResponse(zed->open(init_params));

			if (!zedsuccess) {
				close();
				return false;
			} else {
				bZedReady = true;
				ofLog() << "ZED initialized." << endl;
			}

			zedWidth = zed->getCameraInformation().camera_resolution.width;
			zedHeight = zed->getCameraInformation().camera_resolution.height;

			ofLog() << "Resolution: " << zedWidth << ", " << zedHeight << endl;
			ofLog() << "FPS: " << getCurrentFPS() << endl;

			colorLeftTexture.allocate(zedWidth, zedHeight, GL_RGBA);
			colorRightTexture.allocate(zedWidth, zedHeight, GL_RGBA);

			bRequestNewFrame = true;
			if (bStartThread) {
				startThread();
			}
		}
		catch (exception& err) {
			cerr << err.what() << endl;
			return false;
		}
		return true;
	}

	void Camera::enablePositionalTracking(sl::PositionalTrackingParameters tracking_params)
	{
		bool thread = isThreadRunning();
		if (thread) {
			waitForThread(true);
		}
		bUseTracking = handleZedResponse(zed->enablePositionalTracking(tracking_params));
		if (bUseTracking) {
			ofLog() << "positional tracking enable success";
		}
		if (thread) {
			startThread();
		}
	}

	void Camera::enableSensorThread()
	{
		sensorThread = make_shared<SensorThread>(this);
		sensorThread->startThread();
		bUseSensor = true;
	}

	void Camera::enableObjectDetection(sl::ObjectDetectionParameters obj_det_params, sl::ObjectDetectionRuntimeParameters obj_det_params_rt)
	{
		bool thread = isThreadRunning();
		if (thread) {
			waitForThread(true);
		}
		bUseObjectDetection = handleZedResponse(zed->enableObjectDetection(obj_det_params));
		if (bUseObjectDetection) {
			this->objDetRtParams = obj_det_params_rt;
			ofLog() << "object detection enable success";
		}
		if (thread) {
			startThread();
		}
	}

	void Camera::disablePositionalTracking()
	{
		if (bUseTracking) {
			bool thread = isThreadRunning();
			if (thread) {
				waitForThread(true);
			}
			zed->disablePositionalTracking();
			ofLog() << "positional tracking disable success";
			if (thread) {
				startThread();
			}
		}
		bUseTracking = false;
	}

	void Camera::disableSensorThread()
	{
		if (bUseSensor) {
			if (sensorThread && sensorThread->isThreadRunning()) {
				sensorThread->waitForThread();
				sensorThread.reset();
			}
		}
	}

	void Camera::disableObjectDetection()
	{
		if (bUseObjectDetection) {
			bool thread = isThreadRunning();
			if (thread) {
				waitForThread(true);
			}
			zed->disableObjectDetection();
			ofLog() << "object detection disable success";
			if (thread) {
				startThread();
			}
		}
		bUseObjectDetection = false;
	}

	void Camera::close()
	{
		if (isThreadRunning()) {
			waitForThread(true);

		}
		if (zed) {
			disableSensorThread();
			disableObjectDetection();
			disablePositionalTracking();
			zed->close();
			delete zed;
			zed = nullptr;
			bZedReady = false;
		}
	}

	ofVec2f Camera::getImageDimensions()
	{
		return ofVec2f(zedWidth, zedHeight);
	}

	float Camera::getCurrentFPS() 
	{
		return zed->getCurrentFPS();
	}

	void Camera::setSensingMode(sl::SENSING_MODE mode)
	{
		rt.sensing_mode = mode;
	}

	void Camera::debugDrawObjectDetectionResult2D() const
	{
		for (const auto& object : this->objDetResult.object_list) {
			stringstream ss;
			ss << "ID : " << object.id << endl;
			ss << "Label : " << object.label << endl;
			ss << "SubLabel : " << object.sublabel << endl;
			ss << "State : " << object.tracking_state << endl;
			ss << "Conf : " << object.confidence;
			if (object.bounding_box_2d.size() >= 4) {
				auto lt = object.bounding_box_2d[0];
				ofDrawBitmapStringHighlight(ss.str(), glm::vec2(lt.x, lt.y));
				auto rb = object.bounding_box_2d[2];

				ofPushStyle();
				ofNoFill();
				ofSetColor(0, 255, 0);
				ofSetLineWidth(3);
				ofDrawRectangle(lt.x, lt.y, (rb.x - lt.x), (rb.y - lt.y));
				ofPopStyle();
			}

			if (object.keypoint_2d.size() == 18) {
				ofPushStyle();
				ofSetColor(255, 0, 0);
				ofSetLineWidth(3);
				for (int i = 0; i < BODY18_CONNECTIONS.size(); i += 2) {
					auto idx1 = BODY18_CONNECTIONS[i];
					auto idx2 = BODY18_CONNECTIONS[i+1];
					auto conf1 = object.keypoint_confidence[idx1];
					auto conf2 = object.keypoint_confidence[idx2];
					auto pt1 = toOf(object.keypoint_2d[idx1]);
					auto pt2 = toOf(object.keypoint_2d[idx2]);
					if (conf1 > 0.2 && conf2 > 0.2 && pt1.lengthSquared() > 0.0 && pt2.lengthSquared() > 0.0) {
						ofDrawLine(pt1, pt2);
					}
				}
				ofPopStyle();
			}
			ofPushStyle();
			ofSetColor(128, 255, 0);
			for (const auto& kp2d : object.keypoint_2d) {
				ofDrawSphere(kp2d.x, kp2d.y, 3);
			}
			ofPopStyle();

		}
	}

	void Camera::debugDrawObjectDetectionResult3D() const
	{
		ofPushStyle();
		ofSetColor(255, 0, 255);
		for (const auto& object : this->objDetResult.object_list) {
			if (object.keypoint.size() == 18) {
				ofPushStyle();
				ofSetColor(255, 0, 255);
				ofSetLineWidth(3);
				for (int i = 0; i < BODY18_CONNECTIONS.size(); i += 2) {
					auto idx1 = BODY18_CONNECTIONS[i];
					auto idx2 = BODY18_CONNECTIONS[i + 1];
					auto conf1 = object.keypoint_confidence[idx1];
					auto conf2 = object.keypoint_confidence[idx2];
					auto pt1 = toOf(object.keypoint[idx1]);
					auto pt2 = toOf(object.keypoint[idx2]);
					if (conf1 > 0.2 && conf2 > 0.2 && pt1.lengthSquared() > 0.0 && pt2.lengthSquared() > 0.0) {
						ofDrawLine(pt1, pt2);
					}
				}
				ofPopStyle();
			}
			ofPushStyle();
			ofSetColor(0, 255, 255);
			for (const auto& kp : object.keypoint) {
				ofDrawSphere(toOf(kp), 0.01);
			}
			ofPopStyle();
		}
	}

	void Camera::threadedFunction()
	{
		while (isThreadRunning()) {
			sl::ERROR_CODE zederr = sl::ERROR_CODE::SUCCESS;
			if (bRequestNewFrame && bZedReady) {
				zederr = zed->grab(rt);

				if (lock()) {
					if (zederr == sl::ERROR_CODE::CAMERA_NOT_DETECTED) {
						bDisconnected = true;
						bRequestNewFrame = false;
						ofSleepMillis(10);
					}
					else if (zederr == sl::ERROR_CODE::SUCCESS) {
						cameraTimestampBack = zed->getTimestamp(sl::TIME_REFERENCE::IMAGE);
						trackingState = bUseTracking ? zed->getPosition(poseBack) : sl::POSITIONAL_TRACKING_STATE::OFF;
						
						if (bUseObjectDetection) {
							zed->retrieveObjects(objDetResultBack, objDetRtParams);
						}

						bNewBuffer = true;
						bRequestNewFrame = false;
					}
					unlock();
				}
			}

			ofSleepMillis(1);
		}
	}


	void Camera::update()
	{
		bNewFrame = false;

		if (bZedReady && bDisconnected) {
			close();
			bDisconnected = false;
			return;
		}

		if (!bZedReady || !zed) {
			return;
		}

		if (bNewBuffer) {
			if (lock()) {
				if (bUseColorImage) {
					{
						auto ret = zed->retrieveImage(this->cl, sl::VIEW::LEFT);
						if (ret == sl::ERROR_CODE::SUCCESS) {
							colorLeftPixels.setFromExternalPixels(this->cl.getPtr<uint8_t>(), zedWidth, zedHeight, 4);
							colorLeftTexture.loadData(colorLeftPixels);
						}
					}
					if (bEnableRightSideMeasure) {
						auto ret = zed->retrieveImage(this->cr, sl::VIEW::RIGHT);
						if (ret == sl::ERROR_CODE::SUCCESS) {
							colorRightPixels.setFromExternalPixels(this->cr.getPtr<uint8_t>(), zedWidth, zedHeight, 4);
							colorRightTexture.loadData(colorRightPixels);
						}
					}
				}

				if (bUseDepthImage) {
					{
						auto ret = zed->retrieveMeasure(this->dl, sl::MEASURE::DEPTH);
						if (ret == sl::ERROR_CODE::SUCCESS) {
							depthLeftPixels.setFromExternalPixels(this->dl.getPtr<float>(), zedWidth, zedHeight, 1);
							depthLeftTexture.loadData(depthLeftPixels);
						}
						else {
							ofLogError() << sl::errorCode2str(ret).c_str() << endl;
						}
					}
					if (bEnableRightSideMeasure) {
						auto ret = zed->retrieveMeasure(this->dr, sl::MEASURE::DEPTH_RIGHT);
						if (ret == sl::ERROR_CODE::SUCCESS) {
							depthRightPixels.setFromExternalPixels(this->dr.getPtr<float>(), zedWidth, zedHeight, 1);
							depthRightTexture.loadData(depthRightPixels);
						}
						else {
							ofLogError() << sl::errorCode2str(ret).c_str() << endl;
						}
					}
				}

				if (bUseTracking) {
					std::swap(pose, poseBack);
				} else {
					pose = sl::Pose();
				}

				if (bUseObjectDetection) {
					std::swap(objDetResult, objDetResultBack);
				} else {
					objDetResult = sl::Objects();
				}

				cameraTimestamp = cameraTimestampBack;
				unlock();
				bNewBuffer = false;
				bRequestNewFrame = true;
				bNewFrame = true;
				lastNewFrame = ofGetFrameNum();
			}
		}

		if (bUseSensor) {
			sensorThread->update(imu);
		}
	}

	Camera::SensorThread::SensorThread(Camera * parent) {
		this->parent_ = parent;
	}
	
	void Camera::SensorThread::threadedFunction() {
		uint64_t sensorCount = 0;
		while (isThreadRunning()) {
			sl::ERROR_CODE zederr = sl::ERROR_CODE::SUCCESS;
			sl::SensorsData sensors_data;

			if (lock()) {
				parent_->zed->getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT);
				if (imuBack.size() == 0 || sensors_data.imu.timestamp.data_ns > imuBack.back().timestamp.data_ns)
				{
					imuBack.push_back(sensors_data.imu);
					sensorCount++;
				}
				unlock();
			}
			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}
	}
	void Camera::SensorThread::update(vector<sl::SensorsData::IMUData>& imuMainThread)
	{
		lock();
		imuBack.swap(imuMainThread);
		imuBack.clear();
		unlock();
	}
}
