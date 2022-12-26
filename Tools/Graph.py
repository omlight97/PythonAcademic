import control
from matplotlib import pyplot as plt
import matplotlib
import numpy as np

matplotlib.use('TkAgg')


# for root locus: roots,gains = control.rlocus(tf); plt.show()

class PlotTool:


    @staticmethod
    def plot_values(x_values: list, y_values: list, suptitle: str = None, x_label: str = None, y_label: str = None,
                    figure: bool = True, grid: bool = True):
        """Creates and show a plot. The plot will always have a grid & have the subtitle "Ori Einy - 208643866" 
            user's specification:
                Args: 
                    x_values(list): values on the X-axis
                    y_values(list): corresponding values on the Y-axis
                    suptitle(str-optional): upper title for the plot. 
                    x_label(str-optiona;): label for X-axis
                    y_label(str-optiona;): label for Y-axis
                Returns:
                    Plot
                    """
        if len(x_values) != len(y_values):
            raise Exception('X and Y array values are not the same size')
        if figure:
            plt.figure()
        plt.plot(x_values, y_values)
        plt.title("Ori Einy - 208643866")
        if suptitle:
            plt.suptitle(suptitle)
        if x_label:
            plt.xlabel(x_label)
        if y_label:
            plt.ylabel(y_label)
        if grid:
            plt.grid()
        plt.show()

    @staticmethod
    def plot_step_response(time: list, y_values: list):
        """Creates and show a step plot. The plot will always have a grid, a title "Step Respone" & subtitle "Ori Einy - 208643866".
            it will also have defult labels for the x and y axis.
            user's specification:
                Args: 
                    time(list): values on the X-axis [sec]
                    y_values(list): values of the respons to the step input
                Returns:
                    Step plot
                    """
        PlotTool.plot_values(time, y_values, suptitle="Step Response", x_label='Time [s]', y_label='Amplitude')

