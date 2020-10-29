# ofxZED
Implementation of StereoLab's ZED camera for openFrameworks

## Requirements:
- <a href="http://openframeworks.cc/download/">openFrameworks 0.11.0 for x64 builds</a>
- <a href="https://www.visualstudio.com/">Visual Studio 2017</a>
- <a href="https://www.stereolabs.com/developers/release/3.2/">ZED SDK 3.2.2</a>
- <a href="https://developer.nvidia.com/">CUDA 11.0</a>


## Installation:
1. Generate Project using Project Generator
2. From Property Manager, Add "CUDA 11.0.props"
3. Change compiler setting to enable C++14 by following
    - Project --> Property --> C/C++ --> Language --> C++ Language Standard --> `/std:c++14`

## Credits
Put together for openFrameworks by <a href="http://designandsystems.de">Design & Systems</a> / <a href="http://www.stefanwagner.io">Stefan Wagner (andsynchrony)</a> 

Some functionalities extended by <a href="http://mithru.com/"> Mithru Vigneshwara</a>.

ZED SDK 3.2.2 support by Yuya Hanai (@hanasaan)
