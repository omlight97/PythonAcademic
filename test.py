from Class.ControlTheory import ControlToolbox as ctrl
import control
import matplotlib.pyplot as plt
import numpy as np
from math import atan2
import scipy
from Tools.Graph import PlotTool as plot
call = ctrl(); step_plot = plot()


poles = call.get_second_order_imaginary_poles(10,0.5);
print(poles[0])