# helios_camera
ROS package for Helios 2+ ToF camera & MV-SUF 1200GC RGB Camera

## nodes
### 1. helios_camera : starting helios 2+ ToF camera. Send PCL & intensity Image Topic
  > * **Subscribe** : None
  > * **Publish** : 
  >  + PointCloud2 : "/rdv_helios_0001/depth/point"
  >  + Image : "/rdv_helios_0001/depth/intensity_raw"
  >
  > <pre><code>rosrun helios_camre helios_camera</code></pre>

### 2. mv_rgb_camera : starting MV-SUF 1200GC camera. Send RGB Image & CameraInfo Topic
  > * **Subscribe** : None
  > * **Publish** : 
  >   + Image : "/rdv_helios_0001/color/image_rect_raw"
  >   + CameraInfo : "/rdv_helios_0001/color/camera_info"
  >
  > <pre><code>rosrun helios_camre mv_rgb_camera</code></pre>
  
### 3. pcl_to_image : Not finished yet. Make PointCloud2 Topic to 32FC_ Image
  > * **Subscribe** :
  >   + PointCloud2 : "/rdv_helios_0001/depth/point"
  > * **Publish** : None
  >
  > <pre><code>rosrun helios_camre pcl_to_image</code></pre>


## launch
### 1. helios_camera.launch : Starting helios 2+ camera & tf
  > <pre><code>roslaunch helios_camera helios_camera.launch</code></pre>
  
### 2. mv_rgb_camera.launch : MV-SUF 1200GC camera with ".yaml" calibration file
  > <pre><code>roslaunch helios_camera mv_rgb_camera.launch</code></pre>
  
### 3. mv_rgb_camera.launch : Helios 2+ ToF & MV-SUF 1200GC camera
  > <pre><code>roslaunch helios_camera rgb_depth.launch</code></pre>
