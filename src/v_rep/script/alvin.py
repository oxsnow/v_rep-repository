#!/usr/bin/env python
import numpy as np, skfuzzy as fuzz
from skfuzzy import control as ctrl
import rospy
from std_msgs.msg import Float32

angle = 0
distance = 0

def cllbck_angle(data) :
    global angle
    angle = data.data
    # print(angle)
    

def cllbck_distance(data) :
    global distance
    distance = data.data
    # print(distance)


d = ctrl.Antecedent(np.arange(0,101,1),'distance')
d['VS'] = fuzz.trapmf(d.universe,[0,0,10,30])
d['S'] = fuzz.trimf(d.universe,[10,30,50])
d['M'] = fuzz.trimf(d.universe,[30,50,70])
d['B'] = fuzz.trimf(d.universe,[50,70,90])
d['VB'] = fuzz.trapmf(d.universe,[70,90,100,100])

a = ctrl.Antecedent(np.arange(-90,91,1),'angle')
a['NB'] = fuzz.trapmf(a.universe,[-90,-90,-75,-50])
a['NM'] = fuzz.trimf(a.universe,[-75,-50,-25])
a['NS'] = fuzz.trimf(a.universe,[-50,-25,0])
a['Z'] = fuzz.trimf(a.universe,[-25,0,25])
a['PS'] = fuzz.trimf(a.universe,[0,25,50])
a['PM'] = fuzz.trimf(a.universe,[25,50,75])
a['PB'] = fuzz.trapmf(a.universe,[50,75,90,90])

wl = ctrl.Consequent(np.arange(-2,3,1),'langv')
wl['Z'] = fuzz.trimf(wl.universe,[-2,-2,-1])
wl['S'] = fuzz.trimf(wl.universe,[-2,-1,0])
wl['M'] = fuzz.trimf(wl.universe,[-1,0,1])
wl['B'] = fuzz.trimf(wl.universe,[0,1,2])
wl['VB'] = fuzz.trimf(wl.universe,[1,2,2])

l1 = ctrl.Rule(d['VS']&a['NB'],wl['B'])
l2 = ctrl.Rule(d['VS']&a['NM'],wl['M'])
l3 = ctrl.Rule(d['VS']&a['NS'],wl['S'])
l4 = ctrl.Rule(d['VS']&a['Z'],wl['Z'])
l5 = ctrl.Rule(d['VS']&a['PS'],wl['Z'])
l6 = ctrl.Rule(d['VS']&a['PM'],wl['S'])
l7 = ctrl.Rule(d['VS']&a['PB'],wl['S'])
l8 = ctrl.Rule(d['S']&a['NB'],wl['VB'])
l9 = ctrl.Rule(d['S']&a['NM'],wl['B'])
l10 = ctrl.Rule(d['S']&a['NS'],wl['M'])
l11 = ctrl.Rule(d['S']&a['Z'],wl['S'])
l12 = ctrl.Rule(d['S']&a['PS'],wl['S'])
l13 = ctrl.Rule(d['S']&a['PM'],wl['S'])
l14 = ctrl.Rule(d['S']&a['PB'],wl['M'])
l15 = ctrl.Rule(d['M']&a['NB'],wl['VB'])
l16 = ctrl.Rule(d['M']&a['NM'],wl['VB'])
l17 = ctrl.Rule(d['M']&a['NS'],wl['B'])
l18 = ctrl.Rule(d['M']&a['Z'],wl['M'])
l19 = ctrl.Rule(d['M']&a['PS'],wl['S'])
l20 = ctrl.Rule(d['M']&a['PM'],wl['M'])
l21 = ctrl.Rule(d['M']&a['PB'],wl['B'])
l22 = ctrl.Rule(d['B']&a['NB'],wl['VB'])
l23 = ctrl.Rule(d['B']&a['NM'],wl['VB'])
l24 = ctrl.Rule(d['B']&a['NS'],wl['VB'])
l25 = ctrl.Rule(d['B']&a['Z'],wl['B'])
l26 = ctrl.Rule(d['B']&a['PS'],wl['M'])
l27 = ctrl.Rule(d['B']&a['PM'],wl['B'])
l28 = ctrl.Rule(d['B']&a['PB'],wl['B'])
l29 = ctrl.Rule(d['VB']&a['NB'],wl['VB'])
l30 = ctrl.Rule(d['VB']&a['NM'],wl['VB'])
l31 = ctrl.Rule(d['VB']&a['NS'],wl['VB'])
l32 = ctrl.Rule(d['VB']&a['Z'],wl['VB'])
l33 = ctrl.Rule(d['VB']&a['PS'],wl['VB'])
l34 = ctrl.Rule(d['VB']&a['PM'],wl['VB'])
l35 = ctrl.Rule(d['VB']&a['PB'],wl['VB'])

wlctrl = ctrl.ControlSystem([l1,l2,l3,l4,l5,l6,l7,
                             l8,l9,l10,l11,l12,l13,l14,
                             l15,l16,l17,l18,l19,l20,l21,
                             l22,l23,l24,l25,l26,l27,l28,
                             l29,l30,l31,l32,l33,l34,l35])

langv = ctrl.ControlSystemSimulation(wlctrl)

wr = ctrl.Consequent(np.arange(-2,3,1),'rangv')
wr['Z'] = fuzz.trimf(wr.universe,[-2,-2,-1])
wr['S'] = fuzz.trimf(wr.universe,[-2,-1,0])
wr['M'] = fuzz.trimf(wr.universe,[-1,0,1])
wr['B'] = fuzz.trimf(wr.universe,[0,1,2])
wr['VB'] = fuzz.trimf(wr.universe,[1,2,2])

r1 = ctrl.Rule(d['VS']&a['NB'],wr['S'])
r2 = ctrl.Rule(d['VS']&a['NM'],wr['S'])
r3 = ctrl.Rule(d['VS']&a['NS'],wr['Z'])
r4 = ctrl.Rule(d['VS']&a['Z'],wr['Z'])
r5 = ctrl.Rule(d['VS']&a['PS'],wr['S'])
r6 = ctrl.Rule(d['VS']&a['PM'],wr['M'])
r7 = ctrl.Rule(d['VS']&a['PB'],wr['B'])
r8 = ctrl.Rule(d['S']&a['NB'],wr['M'])
r9 = ctrl.Rule(d['S']&a['NM'],wr['S'])
r10 = ctrl.Rule(d['S']&a['NS'],wr['S'])
r11 = ctrl.Rule(d['S']&a['Z'],wr['S'])
r12 = ctrl.Rule(d['S']&a['PS'],wr['M'])
r13 = ctrl.Rule(d['S']&a['PM'],wr['B'])
r14 = ctrl.Rule(d['S']&a['PB'],wr['VB'])
r15 = ctrl.Rule(d['M']&a['NB'],wr['B'])
r16 = ctrl.Rule(d['M']&a['NM'],wr['M'])
r17 = ctrl.Rule(d['M']&a['NS'],wr['S'])
r18 = ctrl.Rule(d['M']&a['Z'],wr['M'])
r19 = ctrl.Rule(d['M']&a['PS'],wr['B'])
r20 = ctrl.Rule(d['M']&a['PM'],wr['VB'])
r21 = ctrl.Rule(d['M']&a['PB'],wr['VB'])
r22 = ctrl.Rule(d['B']&a['NB'],wr['B'])
r23 = ctrl.Rule(d['B']&a['NM'],wr['B'])
r24 = ctrl.Rule(d['B']&a['NS'],wr['M'])
r25 = ctrl.Rule(d['B']&a['Z'],wr['B'])
r26 = ctrl.Rule(d['B']&a['PS'],wr['VB'])
r27 = ctrl.Rule(d['B']&a['PM'],wr['VB'])
r28 = ctrl.Rule(d['B']&a['PB'],wr['VB'])
r29 = ctrl.Rule(d['VB']&a['NB'],wr['VB'])
r30 = ctrl.Rule(d['VB']&a['NM'],wr['VB'])
r31 = ctrl.Rule(d['VB']&a['NS'],wr['VB'])
r32 = ctrl.Rule(d['VB']&a['Z'],wr['VB'])
r33 = ctrl.Rule(d['VB']&a['PS'],wr['VB'])
r34 = ctrl.Rule(d['VB']&a['PM'],wr['VB'])
r35 = ctrl.Rule(d['VB']&a['PB'],wr['VB'])

wrctrl = ctrl.ControlSystem([r1,r2,r3,r4,r5,r6,r7,
                             r8,r9,r10,r11,r12,r13,r14,
                             r15,r16,r17,r18,r19,r20,r21,
                             r22,r23,r24,r25,r26,r27,r28,
                             r29,r30,r31,r32,r33,r34,r35])

rangv = ctrl.ControlSystemSimulation(wrctrl)

rospy.init_node('alvin', anonymous=True)

rospy.Subscriber("/angle", Float32, cllbck_angle)
rospy.Subscriber("/distance", Float32, cllbck_distance)

left_pub = rospy.Publisher('/left_motor', Float32, queue_size=10)
right_pub = rospy.Publisher('/right_motor', Float32, queue_size=10)

while not rospy.is_shutdown():
    # global distance
    # global angle
    rangv.input['distance'] = distance
    rangv.input['angle'] = angle
    rangv.compute()
    r_result = rangv.output['rangv']
    right_pub.publish(r_result)
    print(r_result)

    langv.input['distance'] = distance
    langv.input['angle'] = angle
    langv.compute()
    l_result = langv.output['langv']
    left_pub.publish(l_result)
    print(l_result)



