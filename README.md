# ofxZED
Implementation of StereoLab's ZED camera for openFrameworks

## Requirements:
- <a href="http://openframeworks.cc/download/">openFrameworks 0.11.2 for x64 builds</a>
- <a href="https://www.visualstudio.com/">Visual Studio 2017</a>
- <a href="https://www.stereolabs.com/developers/release/3.2/">ZED SDK 3.6.4</a>
- <a href="https://developer.nvidia.com/">CUDA 11.X (11.1~11.5. tested on 11.4)</a>


## Installation:
1. If you have installed CUDA version other than v11.4, you will need to replace `CUDA 11.4.props` file on `ofxZED` directory. `CUDA 11.X.props` where X corresponds to installed CUDA version is locaced under `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.X\extras\visual_studio_integration\MSBuildExtensions`. Please copy it to `ofxZED` directory and delete `CUDA 11.4.props`.
2. Generate Project using Project Generator. 

## Credits
Put together for openFrameworks by <a href="http://designandsystems.de">Design & Systems</a> / <a href="http://www.stefanwagner.io">Stefan Wagner (andsynchrony)</a> 

Some functionalities extended by <a href="http://mithru.com/"> Mithru Vigneshwara</a>.

ZED SDK 3.6.4 support by Yuya Hanai (@hanasaan)
