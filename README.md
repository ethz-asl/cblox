# c-blox
![c_blox_new_small](https://user-images.githubusercontent.com/671701/55418812-b94de280-5573-11e9-9574-c83660fe3354.png)

A TSDF-based mapping library based on [Voxblox](https://github.com/ethz-asl/voxblox). C-blox adds sub-mapping to voxblox with the aim of allowing large(r)-scale mapping in the presence of imperfect pose estimates.

# Paper

If using c-blox for scientific publications, please cite the following paper, available [here](https://arxiv.org/abs/1710.07242):

Alexander Millane, Zachary Taylor, Helen Oleynikova, Juan Nieto, Roland Siegwart and Cesar Cadena. "**C-blox: A Scalable and Consistent TSDF-based Dense Mapping Approach.**" 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018.

```latex
@inproceedings{millane2018cblox,
  author={Millane, Alexander and Taylor, Zachary and Oleynikova, Helen and Nieto, Juan and Siegwart, Roland and Cadena, C{\'e}sar},
  booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={C-blox: A Scalable and Consistent TSDF-based Dense Mapping Approach},
  year={2018}
}
```

# Installation

C-blox extends the voxblox library and runs on ROS. The first step of installation is to setup a catkin workspace and install voxblox as per the instructions [here](https://voxblox.readthedocs.io/en/latest/pages/Installation.html).

Then Navigate to the catkin src space and clone c-blox; if you've installed in the default location and are using [ssh keys for github](https://help.github.com/en/articles/connecting-to-github-with-ssh):
```
cd ~/catkin_ws/src/
git clone git@github.com:ethz-asl/c-blox.git
```
Compile:
```
catkin build cblox_ros
```
You're done!

# Example Usage

We include (at the moment) two examples with the c-blox package:
* Kitti Dataset (ground truth localization)
* ~~Machine Hall (orb-slam localization)~~ (in production)

## Kitti Dataset

In this example we create a cblox map using the lidar data and "ground-truth" pose estimates from the [kitti dataset](http://www.cvlibs.net/datasets/kitti/). This simple example demonstrates the *creation* and *display* of submaps using c-blox - because we use drift-free pose estimates (rather than a SLAM system), no submap correction is required/used.

The produced map, viewed in [rviz](http://wiki.ros.org/rviz):

![c_blox_new_small](https://user-images.githubusercontent.com/671701/55502193-50866900-564c-11e9-999e-b78bf71dd3d4.png)

And under construction

![c_blox_new_small](https://user-images.githubusercontent.com/671701/55502987-04d4bf00-564e-11e9-964c-11e7ba7ea754.gif)

To run the example download a [kitti raw dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php). To produce the map above, we ran the "2011_09_30_drive_0018" dataset under the catagory "residential". Convert the data to a rosbag using [kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag).

Then, with cblox_ros built and your workspace sourced, run cblox:
```
roslaunch cblox_ros run_kitti.launch dataset_path:=PATH_TO_YOUR_BAG
```
Rviz should start up and you should see the submaps start to appear, as in the animation above.
