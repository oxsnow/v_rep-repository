#!/usr/bin/env python

import numpy as np
import rospy
# import skfuzzy
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# def cllbck_sensor(data)
    

# New Antecedent/Consequent objects hold universe variables and membership
# functions

angle = ctrl.Antecedent(np.arange(-90, 91, 1), 'angle')
distance = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'distance')

left = ctrl.Consequent(np.arange(0, 11, 1), 'left')
right = ctrl.Consequent(np.arange(0, 11, 1), 'right')

# Auto-membership function population is possible with .automf(3, 5, or 7)
angle['negative'] = fuzz.trimf(angle.universe, [-90, -90, 0])
angle['zero'] = fuzz.trimf(angle.universe, [-90, 0, 90])
angle['positive']  = fuzz.trimf(angle.universe, [0, 90, 90])

distance['far'] = fuzz.trimf(distance.universe, [0, 0, 0.5])
distance['average'] = fuzz.trimf(distance.universe, [0, 0.5, 1])
distance['near']  = fuzz.trimf(distance.universe, [0.5, 1, 1])

# Custom membership functions can be built interactively with a familiar,
# Pythonic API
left['low'] = fuzz.trimf(left.universe, [0, 0, 5])
left['medium'] = fuzz.trimf(left.universe, [0, 5, 10])
left['high'] = fuzz.trimf(left.universe, [5, 10, 10])

right['low'] = fuzz.trimf(right.universe, [0, 0, 5])
right['medium'] = fuzz.trimf(right.universe, [0, 5, 10])
right['high'] = fuzz.trimf(right.universe, [5, 10, 10])

rule1 = ctrl.Rule(angle['negative'] & distance['far'], left['high'])
rule2 = ctrl.Rule(angle['negative'] & distance['average'], left['medium'])
rule3 = ctrl.Rule(angle['negative'] & distance['near'], left['high'])
rule4 = ctrl.Rule(angle['zero'] & distance['far'], left['high'])
rule5 = ctrl.Rule(angle['zero'] & distance['average'], left['medium'])
rule6 = ctrl.Rule(angle['zero'] & distance['near'], left['low'])
rule7 = ctrl.Rule(angle['positive'] & distance['far'], left['high'])
rule8 = ctrl.Rule(angle['positive'] & distance['average'], left['low'])
rule9 = ctrl.Rule(angle['positive'] & distance['near'], left['low'])

rule11 = ctrl.Rule(angle['negative'] & distance['far'], right['high'])
rule12 = ctrl.Rule(angle['negative'] & distance['average'], right['low'])
rule13 = ctrl.Rule(angle['negative'] & distance['near'], right['low'])
rule14 = ctrl.Rule(angle['zero'] & distance['far'], right['high'])
rule15 = ctrl.Rule(angle['zero'] & distance['average'], right['medium'])
rule16 = ctrl.Rule(angle['zero'] & distance['near'], right['low'])
rule17 = ctrl.Rule(angle['positive'] & distance['far'], right['high'])
rule18 = ctrl.Rule(angle['positive'] & distance['average'], right['medium'])
rule19 = ctrl.Rule(angle['positive'] & distance['near'], right['high'])

velocity_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6,
                                    rule7, rule8, rule9, rule11, rule12, rule13,
                                    rule14, rule15, rule16, rule17, rule18, rule19])

velocity = ctrl.ControlSystemSimulation(velocity_ctrl)

rospy.init_node('fuzzy', anonymous=True)
rate = rospy.Rate(10) # 10hz

angle['zero'].view()

while not rospy.is_shutdown():
    # velocity.input['quality'] = 6.5
    # velocity.input['service'] = 9.8

    # velocity.compute()
    print("print")