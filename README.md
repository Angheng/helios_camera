# helios_camera
ROS package for Helios 2+ ToF camera & MV-SUF 1200GC RGB Camera

## Test Env.
- ToF : ArenaSDK_v0.1.59_Linux_x64
- RGB : linuxSDK_V2.1.0.33
- Ubuntu 18.04 amd64, ROS Melodic

## Installation
### Install Helios2+ ToF Camera
1. Download Arena SDK: [Download](https://thinklucid.com/downloads-hub/, "arena sdk")
2. Extract & Setup
    <pre><code>
      tar -xvf {download path}/ArenaSDK_{SDK_version}_{OS_version}.tar.gz -C ~
      cd ~/ArenaSDK_{OS_version}
      sudo sh Arena_SDK_{OS_version}.conf
      echo "export ARENA_ROOT=~/ArenaSDK_{OS_version}" >> ~/.bashrc
      source ~/.bashrc
    </pre></code>
3. Install OpenCV : [Link](https://support.thinklucid.com/using-opencv-with-arena-sdk-on-linux/, "install opencv")
### Install MV-SUF 1200GC RGB Camera
1. Install MVSDK
    <pre><code>
      mkdir ~/MVSDK
      tar -xvzf linuxSDK_V2.1.0.33.tar.gz -C ~/MVSDK/
      cd ~/MVSDK
      sudo ./install.sh
    </pre></code>

## Nodes
### 1. helios_camera : starting helios 2+ ToF camera. Send PCL & intensity Image Topic
  > * **Subscribe** : None
  > * **Publish** : 
  >   + PointCloud2 : "/rdv_helios_0001/depth/point"
  >   + Image : "/rdv_helios_0001/depth/intensity_raw"
  >   + Image : "/rdv_helios_0001/depth/xyz_mm_raw"
  >   + Image : "/rdv_helios_0001/depth/xyz_color_raw"
  >   
  > * **parameters**:
  >   + **mode** : String, Set Scan 3d Operating Mode (default : "Distance3000mmSingleFreq")
  >     - ~~Distance1500mm~~
  >     - ~~Distance6000mm~~
  >     - ~~Freq100MHz~~
  >     - ~~Freq90MHz~~
  >     - ~~Freq75MHz~~
  >     - ~~Freq50MHz~~
  >     - ~~Freq25MHz~~
  >     - ~~Freq120MHz~~
  >     - Distance1250mmSingleFreq
  >     - Distance3000mmSingleFreq
  >     - Distance4000mmSingleFreq
  >     - Distance5000mmMultiFreq
  >     - Distance6000mmSingleFreq
  >     - Distance8300mmMultiFreq
  >     - HighSpeedDistance625mmSingleFreq
  >     - HighSpeedDistance1250mmSingleFreq
  >     - HighSpeedDistance2500mmSingleFreq
  >   + **exp_time** : String, Set Exposure time (default : "Exp1000Us")
  >     - Exp62_5Us
  >     - ~~Exp187_5Us~~
  >     - Exp250Us
  >     - ~~Exp750Us~~
  >     - Exp1000Us
  >     - ~~Exp3000Us~~
  >   + **gain** : String, Set Conversion gain (default : "Low")
  >     - Low
  >     - High
  >   + **accumulation** : Integer, Set Image accumulation (default : 2)
  >     - 0 ~ n
  >
  > <pre><code>rosrun helios_camera helios_camera</code></pre>
  > <pre><code>rosrun helios_camera helios_camera _mode:=Distance3000mmSingleFreq</code></pre>
  > <pre><code>rosrun helios_camera helios_camera _exp_time:=Exp250Us _gain:= High _accumulation:=4</code></pre>

### 2. pcl_to_image : Make PointCloud2 Topic to 32FC_ Image
  > * **Subscribe** :
  >   + PointCloud2 : "/rdv_helios_0001/depth/point"
  > * **Publish** : None
  >
  > <pre><code>rosrun helios_camera pcl_to_image</code></pre>

### 3. mv_rgb_camera : starting MV-SUF 1200GC camera. Send RGB Image & CameraInfo Topic
  > * **Subscribe** : None
  > * **Publish** : 
  >   + Image : "/rdv_mv_rgb_0001/color/image_rect_raw"
  >   + CameraInfo : "/rdv_mv_rgb_0001/color/camera_info"
  >
  > <pre><code>rosrun mv_rgb_camera mv_rgb_camera</code></pre>


## Launch
### 1. helios_camera.launch : Starting helios 2+ camera & tf
  > <pre><code>roslaunch helios_camera helios_camera.launch</code></pre>
  
### 2. mv_rgb_camera.launch : MV-SUF 1200GC camera with ".yaml" calibration file
  > <pre><code>roslaunch mv_rgb_camera mv_rgb_camera.launch</code></pre>
  
### 3. mv_rgb_camera.launch : Helios 2+ ToF & MV-SUF 1200GC camera
  > <pre><code>roslaunch helios_camera rgb_depth_camera.launch</code></pre>
