# Camera Calibration Tool on Windows/MacOS
## support Mono/Stereo/HandEye calibration and Disparity estimation
![screenshot_mono](elements/screenshot1.png)  
![screenshot_stereo](elements/screenshot2.png) 
![screenshot_handeye](elements/screenshot3.png) 
![screenshot_disparity](elements/screenshot4.png)
## usage
* mono/stereo calibration just see the UI, they are easy to use.  
* handeye should prepare quaternion of tcp in a csv file, the format is below:  

|qw        |qx        |qy        |qz        |tx        |ty        |tz        |
|:---------|:---------|:---------|:---------|:---------|:---------|:---------|
|0.05105444|-0.049133111|0.997192233|0.024228728|-126.2206137|-161.8086994|-1045.676351|

## Calibration Pattern
Use [Calib.io](https://calib.io/pages/camera-calibration-pattern-generator) to generate the pattern.
## TODO
- [x]   Support FOV < 80
- [ ]   Support WideAngle (80~110)
- [ ]   Support FishEye and 360
- [x]   Support SGBM/SGM stereo disparity estimation
- [ ]   Support 3D calibration pattern
- [ ]   Support Multi-Cam calibration
- [ ]   Support SFM
- [ ]   Provide easy to use c++ api