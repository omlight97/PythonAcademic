from Class.ControlTheory import ControlToolbox as ctrl
import control
import matplotlib.pyplot as plt
import numpy as np
from math import atan2
import scipy
from Tools.Graph import PlotTool as plot
call = ctrl(); call_plot = plot()

#Q1
poles1 = np.complex_(-8.15+16j); deno1 = [[1,10],[1,3],[1,6]]; nume1 = [[1,8]]
[P1,C1,CP1_CL,tout1,yout1,stepinfo1] = call.Classic_Controller_with_Step_Response(nume1,deno1,poles=poles1,C_type="PID")
call_plot.plot_step_response(tout1,yout1) 
print(C1)

#Q2
poles2 = call.get_second_order_imaginary_poles(17,0.93); dom_pole2 = poles2[0]
deno2 = [[1,2,3.25],[1,4]]; nume2 = [[4,8],[1,8]]
[P2,C2,CP2_CL,tout2,yout2,stepinfo2] = call.Classic_Controller_with_Step_Response(nume2,deno2,poles=dom_pole2,C_type="PI",SettlingTime=0.01)
call_plot.plot_step_response(tout2,yout2) 

#Q3
deno3 = [[1,1],[1,4],[1,8]]; nume3 = [[1,6]]
Kp = 32.9; Ki=17.8; C_P3 = control.tf([Kp,Ki],[1,0])
[P3,C3,CP3_CL,tout3,yout3,stepinfo3] = call.Classic_Controller_with_Step_Response(nume3,deno3,None,"costume",SettlingTime=0.01, Costume=C_P3)
print(stepinfo3); print(C3*P3)
call_plot.plot_step_response(tout3,yout3) 

#Q4
tf4 = call.tf_creator([[0,1]],[[1,1],[1,3],[1,5]])
#roots,gains = control.rlocus(tf4); plt.show()
Kp = 72.7; C_PID4 = call.improper_tf([Kp*0.1637,Kp*1])+control.tf([Kp*0.916],[1,0])
[P4,C4,CP4_CL,tout4,yout4,stepinfo4] = call.Classic_Controller_with_Step_Response([[0,1]],[[1,1],[1,3],[1,5]],None,"costume", Costume=C_PID4)
print(C_PID4)
print(stepinfo4)
call_plot.plot_step_response(tout4,yout4)