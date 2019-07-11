import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import numpy as np
from mujoco_py import load_model_from_path, MjSim, MjViewer
from sys import argv

recent_history = np.zeros((30))
u = 0
max_emg = 0.8

FREE_MODE = 0
TEST_MODE = 1 # move with reference arm, try to track
KEMARI_MODE = 2
try:
    mode = int(argv[1])
except IndexError:
    print("provide argument to designate experiment type. (see source for ID). Setting to default FREE_MODE....")
    mode = FREE_MODE

def callback(msg):
    recent_history[:-1] = recent_history[1:]
    recent_history[-1]= np.abs(msg.vector.x)
    smoothed = np.average(recent_history)
    pub.publish(smoothed)
    global u
    u = smoothed/max_emg

rospy.init_node("emg_control", anonymous=True)
pub = rospy.Publisher("smoothed", Float32, queue_size=10)
jsPub = rospy.Publisher("joint_states", JointState, queue_size=10)
rospy.Subscriber("nxwms_reader/emg", Vector3Stamped, callback)

if mode == FREE_MODE:
    rospy.loginfo("free interaction mode!")
    xml_path = "model.xml"
elif mode == TEST_MODE:
    rospy.loginfo("test mode!")
    xml_path = "reference_arm.xml"
    ref_angle = 0
elif mode == KEMARI_MODE:
    rospy.loginfo("kemari mode!")
    xml_path = "kemari.xml"

model = load_model_from_path(xml_path)
sim = MjSim(model)
viewer = MjViewer(sim)

while not rospy.is_shutdown():
    sim.data.ctrl[0] = u
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = []
    js.position = []
    if mode == TEST_MODE:
        ref_angle = 60 * np.sin(sim.data.time*3.14/3)
        sim.data.qpos[1] = ref_angle/180*3.14
        js.name.append("reference")
        js.position.append(-ref_angle)
        sim.forward()
    if mode == KEMARI_MODE:
        if sim.data.qpos[3] < -1:
            rospy.loginfo("ball fell to floor, so returning to original position...")
            sim.data.qpos[1] = 0.3
            sim.data.qpos[2] = 0
            sim.data.qpos[3] = 1
            for i in range(6):
                sim.data.qvel[i+1] = 0
            sim.forward()
    js.name.append("puppet")
    js.position.append(-sim.data.qpos[0]*180/3.14)

    jsPub.publish(js)
    sim.step()
    viewer.render()
