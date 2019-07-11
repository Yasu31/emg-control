# EMG-puppet system
2019年Sセメスター　認識行動システム論第二　最終課題

[レポート](./report.pdf)

# requirements
* [MuJoCo](http://www.mujoco.org/)(license required): physics engine
* [mujoco-py](https://github.com/openai/mujoco-py): Python interface for mujoco
* jsk_tendon_robot: this is a private repository, and the program for connecting to EMG is contained here. If you don't have EMG logger, use the rosbag files in /sample_emg directory instead of running nxwms_reader(e.g.`$ rosbag play a.bag`).

# demo
1. start ROS master `$ roscore`
1. if you want to visualize, start plotjuggler
  * `$ rosrun plotjuggler PlotJuggler`
  * load the layout from plotjuggler
1. start EMG data flow
  * connect data receiver via USB
  * `$ rosrun emg_sensor_lp nxwms_reader`
1. run program. `$ python3 script.py`
