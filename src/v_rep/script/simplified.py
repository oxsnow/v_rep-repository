#!/usr/bin/env python
import numpy as np, skfuzzy as fuzz
from skfuzzy import control as ctrl
import rospy
from std_msgs.msg import Float32

angle_value = 0
distance_value = 0

def cllbck_angle(data) :
    global angle_value
    angle_value = data.data
    # print(angle)
    

def cllbck_distance(data) :
    global distance_value
    distance_value = data.data
    # print(distance)


distance = ctrl.Antecedent(np.arange(0,101,1),'distance')
distance['N'] = fuzz.trapmf(distance.universe,[0,0,35,50])
distance['M'] = fuzz.trimf(distance.universe,[35,50,65])
distance['F'] = fuzz.trapmf(distance.universe,[50,65,100,100])

# distance['M'].view()

angle = ctrl.Antecedent(np.arange(-90,91,1),'angle')
angle['NB'] = fuzz.trapmf(angle.universe,[-90, -90, -75, -20])
angle['NS'] = fuzz.trimf(angle.universe,[-75, -20, 0])
angle['Z'] = fuzz.trimf(angle.universe,[-20, 0, 20])
angle['PS'] = fuzz.trimf(angle.universe,[0, 20 ,75])
angle['PB'] = fuzz.trapmf(angle.universe,[20, 75, 90, 90])

# angle = ctrl.Antecedent(np.arange(-90,91,1),'angle')
# angle['NB'] = fuzz.trapmf(angle.universe,[-90, -90, -75, -50])
# angle['NS'] = fuzz.trimf(angle.universe,[-75, -50, 0])
# angle['Z'] = fuzz.trimf(angle.universe,[-50, 0, 50])
# angle['PS'] = fuzz.trimf(angle.universe,[0, 50 ,75])
# angle['PB'] = fuzz.trapmf(angle.universe,[50, 75, 90, 90])

# angle['Z'].view()

v_left = ctrl.Consequent(np.arange(-4,5,1),'v_left')
v_left['NF'] = fuzz.trimf(v_left.universe,[-4, -4, -2])
v_left['NS'] = fuzz.trimf(v_left.universe,[-4, -2, 0])
v_left['Z'] = fuzz.trimf(v_left.universe,[-2 , 0 , 2])
v_left['PS'] = fuzz.trimf(v_left.universe,[0 , 2 , 4])
v_left['PF'] = fuzz.trimf(v_left.universe,[2 ,4 ,4])

# v_left['Z'].view()

v_right = ctrl.Consequent(np.arange(-4,5,1),'v_right')
v_right['NF'] = fuzz.trimf(v_right.universe,[-4, -4, -2])
v_right['NS'] = fuzz.trimf(v_right.universe,[-4, -2, 0])
v_right['Z'] = fuzz.trimf(v_right.universe,[-2 , 0 , 2])
v_right['PS'] = fuzz.trimf(v_right.universe,[0 , 2 , 4])
v_right['PF'] = fuzz.trimf(v_right.universe,[2 ,4 ,4])

r_left1 = ctrl.Rule(distance['N'] & angle['NB'], v_left['PF'])
r_left2 = ctrl.Rule(distance['N'] & angle['NS'], v_left['PS'])
r_left3 = ctrl.Rule(distance['N'] & angle['Z'], v_left['NS']) #
r_left4 = ctrl.Rule(distance['N'] & angle['PS'], v_left['NS'])
r_left5 = ctrl.Rule(distance['N'] & angle['PB'], v_left['Z'])
r_left6 = ctrl.Rule(distance['M'] & angle['NB'], v_left['PF'])
r_left7 = ctrl.Rule(distance['M'] & angle['NS'], v_left['PS'])
r_left8 = ctrl.Rule(distance['M'] & angle['Z'], v_left['PS'])
r_left9 = ctrl.Rule(distance['M'] & angle['PS'], v_left['NS'])
r_left10 = ctrl.Rule(distance['M'] & angle['PB'], v_left['PS'])
r_left11 = ctrl.Rule(distance['F'] & angle['NB'], v_left['PF'])
r_left12 = ctrl.Rule(distance['F'] & angle['NS'], v_left['PF'])
r_left13 = ctrl.Rule(distance['F'] & angle['Z'], v_left['PF'])
r_left14 = ctrl.Rule(distance['F'] & angle['PS'], v_left['PF'])
r_left15 = ctrl.Rule(distance['F'] & angle['PB'], v_left['PF'])

left_ctrl = ctrl.ControlSystem([r_left1, r_left2, r_left3, r_left4, r_left5,
                                 r_left6, r_left7, r_left8, r_left9, r_left10,
                                 r_left11, r_left12, r_left13, r_left14, r_left15])

left_ctrl_sim = ctrl.ControlSystemSimulation(left_ctrl)

r_right1 = ctrl.Rule(distance['N'] & angle['NB'], v_right['Z'])
r_right2 = ctrl.Rule(distance['N'] & angle['NS'], v_right['NS'])
r_right3 = ctrl.Rule(distance['N'] & angle['Z'], v_right['PS']) #
r_right4 = ctrl.Rule(distance['N'] & angle['PS'], v_right['PS'])
r_right5 = ctrl.Rule(distance['N'] & angle['PB'], v_right['PF'])
r_right6 = ctrl.Rule(distance['M'] & angle['NB'], v_right['PS'])
r_right7 = ctrl.Rule(distance['M'] & angle['NS'], v_right['NS'])
r_right8 = ctrl.Rule(distance['M'] & angle['Z'], v_right['PS'])
r_right9 = ctrl.Rule(distance['M'] & angle['PS'], v_right['PS'])
r_right10 = ctrl.Rule(distance['M'] & angle['PB'], v_right['PF'])
r_right11 = ctrl.Rule(distance['F'] & angle['NB'], v_right['PF'])
r_right12 = ctrl.Rule(distance['F'] & angle['NS'], v_right['PF'])
r_right13 = ctrl.Rule(distance['F'] & angle['Z'], v_right['PF'])
r_right14 = ctrl.Rule(distance['F'] & angle['PS'], v_right['PF'])
r_right15 = ctrl.Rule(distance['F'] & angle['PB'], v_right['PF'])

right_ctrl = ctrl.ControlSystem([r_right1, r_right2, r_right3, r_right4, r_right5,
                                 r_right6, r_right7, r_right8, r_right9, r_right10,
                                 r_right11, r_right12, r_right13, r_right14, r_right15])

right_ctrl_sim = ctrl.ControlSystemSimulation(right_ctrl)

rospy.init_node('simplified', anonymous=True)

rospy.Subscriber("/angle", Float32, cllbck_angle)
rospy.Subscriber("/distance", Float32, cllbck_distance)

left_pub = rospy.Publisher('/left_motor', Float32, queue_size=10)
right_pub = rospy.Publisher('/right_motor', Float32, queue_size=10)

while not rospy.is_shutdown():
    # # global distance
    # # global angle
    right_ctrl_sim.input['distance'] = distance_value
    right_ctrl_sim.input['angle'] = angle_value
    right_ctrl_sim.compute()
    r_result = right_ctrl_sim.output['v_right']

    left_ctrl_sim.input['distance'] = distance_value
    left_ctrl_sim.input['angle'] = angle_value
    left_ctrl_sim.compute()
    l_result = left_ctrl_sim.output['v_left']

    right_pub.publish(r_result)
    left_pub.publish(l_result)
    print("left %.3f  right %.3f  distance %d  angle %.3f" %(l_result, r_result, distance_value, angle_value))



