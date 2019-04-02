# c-blox
![c_blox_new_small](https://user-images.githubusercontent.com/671701/55418812-b94de280-5573-11e9-9574-c83660fe3354.png)

A TSDF-based mapping library based on [Voxblox](https://github.com/ethz-asl/voxblox). Adds features for large-scale mapping through the use of sub-mapping.

Contact: Alexander Millane alexander.millane@mavt.ethz.ch

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
