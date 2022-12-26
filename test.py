from Class.ControlTheory import ControlToolbox as ctrl
import control
import matplotlib.pyplot as plt
import numpy as np
from math import atan2
import scipy
from Tools.Graph import PlotTool as plot


tf4 = ctrl.tf_creator(zeros_array=[[0,1]],poles_array=[[1,1],[1,3],[1,5]])
tout, yout = control.step_response(tf4)

plot.plot_step_response(tout, yout)