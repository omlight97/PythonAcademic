from matplotlib import pyplot as plt
import matplotlib
import numpy as np
import control

matplotlib.use('TkAgg')


# for root locus: roots,gains = control.rlocus(tf); plt.show()

class PlotTool:

    @staticmethod
    def plot_values(x_values, y_values, suptitle: str = None, x_label: str = None, y_label: str = None,
                    figure: bool = True, grid: bool = True):
        if len(x_values) != len(y_values):
            print('X and Y array values are not the same size')
            return
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

    def plot_step_response(self, time, y_values):
        self.plot_values(time, y_values, suptitle="Step Response", x_label='Time [s]', y_label='Amplitude')

