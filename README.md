# ofxZED
Implementation of StereoLab's ZED camera for openFrameworks

#Requirements:
- <a href="http://openframeworks.cc/download/">openFrameworks 0.9 for x64 builds</a>
- <a href="https://www.visualstudio.com/">Visual Studio 2015</a>
- <a href="https://www.stereolabs.com/developers/#start_anchor">ZED driver's and SDK</a>
- <a href="https://developer.nvidia.com/cuda-toolkit-70">NVIDIA Cuda</a> (tested with 7.0)


#Installation:
- Put into apps/myApps
- Open ZED-Viewer.vcxproj and alter these to lines to link to your ZED & Cuda directories:
  <br />
  &lt;ZED_SDK_DIR&gt;<i>D:\Program Files (x86)\ZED SDK</i>&lt;/ZED_SDK_DIR&gt;
<br />
  &lt;CUDA_DIR&gt;<i>D:\Program Files\NVidia\Cuda 7.0</i>&lt;/CUDA_DIR&gt;

#Credits
Put together for openFrameworks by <a href="http://designandsystems.de">Desgin & Systems</a> / <a href="http://www.stefanwagner.io">Stefan Wagner (andsynchrony)</a> 
