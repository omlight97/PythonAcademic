import control
import math
import numpy as np


class ControlToolbox:

    def __init__(self):
        pass
    
    def Classic_Controller_with_Step_Response(self,numerator: list,denomerator: list,poles: complex,C_type: str= None, H=True, SettlingTime: float=0.02, Costume: control.TransferFunction=None) -> list:
        #calculate the step Response of a closed loop system with a negetive entrance. Controller is chosen based on
        # user's specification
        call = ControlToolbox()
        Plant = call.tf_creator(numerator,denomerator)
        if C_type == "PD":
            Controller = call.PD_creator(Plant,poles)
        if C_type == "PID":
            Controller = call.PID_creator(Plant,poles)
        if C_type == "PI":
            Controller = call.PI_creator(Plant,poles)
        if C_type == "PIv2":
            Controller = call.PI_creator_v2(Plant,poles)
        if C_type == "costume":
            Controller = Costume
        CP = Controller*Plant; CP_cl = control.feedback(CP,H,-1)
        [tout,yout] = control.step_response(CP_cl)
        info = control.step_info(CP_cl,SettlingTimeThreshold=SettlingTime)
        return [Plant,Controller,CP_cl,tout,yout,info]

    def tf_creator(self,zeros_array: list, poles_array:list) -> control.TransferFunction:
        #creates a tf based on convolution of (s+a) where a is and int. recives an array of coeff of zeros
        #[[z+a1],[z+a2]...,[z+an]] > [[1,a1],[1,a2]...,[1,an]] etc. same goes for array of poles
        nume = 1; deno=1
        if type(zeros_array[0]) != list or type(poles_array[0]) != list:
            return "error! Arrays must be list of lists!"
        if len(zeros_array) == len(poles_array):
            for index in range(len(zeros_array)):
                nume= np.convolve(zeros_array[index],nume)
                deno= np.convolve(poles_array[index],deno)
            return control.tf(nume,deno)
        elif len(zeros_array) < len(poles_array):
            for index in range(len(zeros_array)):
                nume= np.convolve(zeros_array[index],nume)
            for jndex in range(len(poles_array)):
                deno= np.convolve(poles_array[jndex],deno)
            return control.tf(nume,deno)
        else:
            call = ControlToolbox()
            for index in range(len(zeros_array)):
                nume= np.convolve(zeros_array[index],nume)
            for index in range(len(poles_array)):
                deno= np.convolve(poles_array[index],deno)
            tfnume = call.improper_tf(nume); tfdeno = control.tf(1,deno)
            return tfnume*tfdeno

    def improper_tf(self,coeff: list)  -> control.TransferFunction:
        #creates and improper function (order if num > order of den). recives coeff. of the tf
         s = control.tf("s"); im_tf = 0
         for index in range(len(coeff)):
            im_tf += s**(len(coeff)-index-1)*coeff[index]
         return im_tf

    def PD_creator(self,transfer_func: control.TransferFunction, dom_poles: complex) -> list: #given K>0
        # This function finds a the coeff K and a for a PD controller ,C(s) = K(s+a) , using the Gain and Phase law
        # learnd in class.
        # please enter one complex pole only - choice is arbitrary.
        call = ControlToolbox()
        N_tmp,D_tmp = control.tfdata(transfer_func); tf_numerator = call.improper_tf(((N_tmp[0])[0]))
        tf_denomirator = call.improper_tf(((D_tmp[0])[0]))
        tf_numerator_with_pole =  tf_numerator(dom_poles)
        tf_denomirator_with_pole = tf_denomirator(dom_poles)
        #useful number for the rest of the calculations
        arg_tmp_out = -math.pi - math.atan2(np.imag(tf_numerator_with_pole),
            np.real(tf_numerator_with_pole)) + math.atan2(np.imag(tf_denomirator_with_pole),np.real(tf_denomirator_with_pole)); #see hw6 file
        a = (np.imag(dom_poles)/math.tan(arg_tmp_out))-np.real(dom_poles)
        K=1/(np.linalg.norm((tf_numerator_with_pole/tf_denomirator_with_pole)))*(1/(np.linalg.norm(dom_poles+a)))
        C_PD = call.improper_tf([K,K*a])
        return C_PD

    def PI_creator(self,transfer_func: control.TransferFunction, dom_poles: complex) -> list: #given K>0
        # This function finds a the coeff K and a for a PI controller ,C(s) = K(s+a)/s. K & a are foud 
        # using the Gain and Phase law learnd in class.
        # please enter one complex pole only - choice is arbitrary.
        call = ControlToolbox()
        N_tmp,D_tmp = control.tfdata(transfer_func) 
        tf_numerator = call.improper_tf(((N_tmp[0])[0])); tf_denomirator = call.improper_tf(((D_tmp[0])[0]))
        #print(tf_denomirator); print(tf_numerator)
        tf_numerator_with_pole =  tf_numerator(dom_poles); tf_denomirator_with_pole = tf_denomirator(dom_poles)
        #print(tf_denomirator_with_pole); print(tf_numerator_with_pole)
        #calculating coeff
        arg_tmp_out = -math.pi - math.atan2(np.imag(tf_numerator_with_pole),
            np.real(tf_numerator_with_pole)) + math.atan2(np.imag(tf_denomirator_with_pole),
            np.real(tf_denomirator_with_pole)) + math.atan2(np.imag(dom_poles),np.real(dom_poles)) #see hw6 file
        a = (np.imag(dom_poles)/math.tan(arg_tmp_out))-np.real(dom_poles)
        K=(np.linalg.norm(dom_poles))/(np.linalg.norm((tf_numerator_with_pole/tf_denomirator_with_pole)))*(1/(np.linalg.norm(dom_poles+a)))
        C_PI = control.tf([K,K*a],[1,0])
        return C_PI

    def PI_creator_v2(self,transfer_func: control.TransferFunction, dom_poles: complex) -> control.TransferFunction: #given K>0
        # This function finds a the coeff K and a for a PI controller ,C(s) = K(s+a)/s. This function finds K using the Gain law
        # assuming |s+a/s|=1, and a=|real(dom_pole)|/10 
        # please enter one complex pole only - choice is arbitrary.
        call = ControlToolbox()
        N_tmp,D_tmp = control.tfdata(transfer_func) 
        tf_numerator = call.improper_tf(((N_tmp[0])[0])); tf_denomirator = call.improper_tf(((D_tmp[0])[0]))
        #print(tf_denomirator); print(tf_numerator)
        tf_numerator_with_pole =  tf_numerator(dom_poles); tf_denomirator_with_pole = tf_denomirator(dom_poles)
        #print(tf_denomirator_with_pole); print(tf_numerator_with_pole)
        #calculating coeff
        a = (np.abs(np.real(dom_poles))/10); print(a)
        K=1/(np.linalg.norm((tf_numerator_with_pole/tf_denomirator_with_pole)))
        C_PI = control.tf([K,K*a],[1,0])
        return C_PI


    def get_second_order_imaginary_poles(self, wn: float, zeta: float) -> list:
        #calculates the poles of s^2+2wn*zeta*s+wn^2
        return [-wn * zeta + (wn * np.sqrt(1 - zeta**2)) * 1j, -wn * zeta - (wn * np.sqrt(1 - zeta**2)) * 1j]