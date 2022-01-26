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
			bUseTracking = useTracking;
			bUseSensor = useSensor;
			bEnableRightSideMeasure = enableRightSideMeasure;

			ofLog() << "Initializing ZED camera." << std::endl;

			zed = new sl::Camera();
			ofLog() << "Resolution Mode:" << resolution << std::endl;

			auto zederr = zed->open(init_params);

			if (zederr != sl::ERROR_CODE::SUCCESS)
			{
				close();
				ofLog() << "ERROR: " << sl::errorCode2str(zederr).c_str() << endl;
				return;
			}
			else
			{
				bZedReady = true;
				ofLog() << "ZED initialized." << endl;
			}

			if (bUseTracking) {
				// Enable positional tracking with default parameters
				sl::PositionalTrackingParameters tracking_parameters;
				zederr = zed->enablePositionalTracking(tracking_parameters);

				if (zederr != sl::ERROR_CODE::SUCCESS)
				{
					close();
					ofLog() << "ERROR: " << sl::errorCode2str(zederr).c_str() << endl;
					return;
				}
			}

			zedWidth = zed->getCameraInformation().camera_resolution.width;
			zedHeight = zed->getCameraInformation().camera_resolution.height;

			ofLog() << "Resolution: " << zedWidth << ", " << zedHeight << endl;
			ofLog() << "FPS: " << getCurrentFPS() << endl;

			colorLeftTexture.allocate(zedWidth, zedHeight, GL_RGBA);
			colorRightTexture.allocate(zedWidth, zedHeight, GL_RGBA);

			bRequestNewFrame = true;
			if (bUseDepthImage)
			{
				rt.enable_depth = true;
				rt.sensing_mode = sl::SENSING_MODE::FILL;
			}
			else if (bUseColorImage)
			{
				rt.enable_depth = false;
			}

			startThread();

			if (bUseSensor) {
				sensorThread = make_shared<SensorThread>(this);
				sensorThread->startThread();
			}
		}
		catch (exception& err) {
			cerr << err.what() << endl;
		}
	}

	void Camera::close()
	{
		if (isThreadRunning()) {
			waitForThread(true);

		}
		if (bUseSensor) {
			if (sensorThread && sensorThread->isThreadRunning()) {
				sensorThread->waitForThread();
				sensorThread.reset();
			}
		}

		if (zed) {
			if (bUseTracking) {
				zed->disablePositionalTracking();
			}
			zed->close();
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

						if (bUseTracking) {
							trackingState = zed->getPosition(pose);
						}
						else {
							trackingState = sl::POSITIONAL_TRACKING_STATE::OFF;
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
			//if (ofGetFrameNum() % 60 == 0 && imu.size()) {
			//	cerr << "[GLThread] " << imu.size() << "imu samples ";
			//	cerr << ", ts: " << imu.back().timestamp.data_ns;
			//	cerr << ", hz: " << imu.back().effective_rate;
			//	cerr << ", gyro: " << imu.back().angular_velocity << endl;
			//	auto pos = toOf(imu.back().pose).getTranslation();
			//	cerr << pos << endl;
			//}
		}

	}
	Camera::SensorThread::SensorThread(Camera * parent)
	{
		this->parent_ = parent;
	}
	void Camera::SensorThread::threadedFunction()
	{
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
