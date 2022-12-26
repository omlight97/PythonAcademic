import control
import math
import numpy as np

class ControlToolbox:

    
    @staticmethod
    def classic_controller_with_step_response(numerator: list, denomerator: list, poles: complex, C_type: str=None, H=True, SettlingTime: float=0.02, Costume: control.TransferFunction=None) -> list:
        """calculate the step Response of a closed loop system with a negetive entrance. Controller is chosen based on
        user's specification
            Args:
                numerator(list): list of coeficents for transfer function
            Returns:
                list: list of balbabla"""
        call_self = ControlToolbox()
        plant = call_self.tf_creator(numerator,denomerator)
        if C_type == "PD":
            Controller = call_self.PD_creator(plant,poles)
        if C_type == "PID":
            Controller = call_self.PID_creator(plant,poles)
        if C_type == "PI":
            Controller = call_self.PI_creator(plant,poles)
        if C_type == "PIv2":
            Controller = call_self.PI_creator_v2(plant,poles)
        if C_type == "costume":
            Controller = Costume
        CP = Controller*plant
        CP_cl = control.feedback(CP,H,-1)
        tout, yout = control.step_response(CP_cl)
        info = control.step_info(CP_cl,SettlingTimeThreshold=SettlingTime)
        return [plant,Controller,CP_cl,tout,yout,info]

    @staticmethod
    def tf_creator(zeros_array: list, poles_array: list) -> control.TransferFunction:
        """creates a tf based on convolution of (s+a) where a is and int. recives an array of coeff of zeros
        [[z+a1],[z+a2]...,[z+an]] > [[1,a1],[1,a2]...,[1,an]] etc. same goes for array of poles"""
        nume, deno = 1, 1
        if not isinstance(zeros_array[0], list | tuple) or not isinstance(poles_array[0], list | tuple):
            raise Exception('error! Arrays must be list of lists!')
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
            call_self = ControlToolbox()
            for index in range(len(zeros_array)):
                nume= np.convolve(zeros_array[index],nume)
            for index in range(len(poles_array)):
                deno= np.convolve(poles_array[index],deno)
            tfnume = call_self.improper_tf(nume); tfdeno = control.tf(1,deno)
            return tfnume*tfdeno

    @staticmethod
    def improper_tf(coeff: list)  -> control.TransferFunction:
        """creates and improper function (order if num > order of den). recives coeff. of the tf"""
        s = control.tf("s")
        im_tf = 0
        for index in range(len(coeff)):
            im_tf += s**(len(coeff)-index-1)*coeff[index]
        return im_tf

    @staticmethod
    def PD_creator(transfer_func: control.TransferFunction, dom_poles: complex) -> list: #given K>0
        """This function finds a the coeff K and a for a PD controller ,C(s) = K(s+a) , using the Gain and Phase law
        learnd in class.
        please enter one complex pole only - choice is arbitrary."""
        call_self = ControlToolbox()
        N_tmp,D_tmp = control.tfdata(transfer_func)
        tf_numerator = call_self.improper_tf(((N_tmp[0])[0]))
        tf_denomirator = call_self.improper_tf(((D_tmp[0])[0]))
        tf_numerator_with_pole =  tf_numerator(dom_poles)
        tf_denomirator_with_pole = tf_denomirator(dom_poles)
        #useful number for the rest of the calculations
        arg_tmp_out = -math.pi - math.atan2(np.imag(tf_numerator_with_pole),
            np.real(tf_numerator_with_pole)) + math.atan2(np.imag(tf_denomirator_with_pole),np.real(tf_denomirator_with_pole)); #see hw6 file
        a = (np.imag(dom_poles)/math.tan(arg_tmp_out))-np.real(dom_poles)
        K=1/(np.linalg.norm((tf_numerator_with_pole/tf_denomirator_with_pole)))*(1/(np.linalg.norm(dom_poles+a)))
        C_PD = call_self.improper_tf([K,K*a])
        return C_PD

    @staticmethod
    def PI_creator(transfer_func: control.TransferFunction, dom_poles: complex) -> list: #given K>0
        """This function finds a the coeff K and a for a PI controller ,C(s) = K(s+a)/s. K & a are found 
        using the Gain and Phase law learnd in class.
        please enter one complex pole only - choice is arbitrary."""
        call_self = ControlToolbox()
        N_tmp,D_tmp = control.tfdata(transfer_func) 
        tf_numerator = call_self.improper_tf(((N_tmp[0])[0]))
        tf_denomirator = call_self.improper_tf(((D_tmp[0])[0]))
        tf_numerator_with_pole =  tf_numerator(dom_poles)
        tf_denomirator_with_pole = tf_denomirator(dom_poles)
        #calculating coeff
        arg_tmp_out = -math.pi - math.atan2(np.imag(tf_numerator_with_pole),
            np.real(tf_numerator_with_pole)) + math.atan2(np.imag(tf_denomirator_with_pole),
            np.real(tf_denomirator_with_pole)) + math.atan2(np.imag(dom_poles),np.real(dom_poles)) #see hw6 file
        a = (np.imag(dom_poles)/math.tan(arg_tmp_out))-np.real(dom_poles)
        K=(np.linalg.norm(dom_poles))/(np.linalg.norm((tf_numerator_with_pole/tf_denomirator_with_pole)))*(1/(np.linalg.norm(dom_poles+a)))
        C_PI = control.tf([K,K*a],[1,0])
        return C_PI

    @staticmethod
    def PI_creator_v2(transfer_func: control.TransferFunction, dom_poles: complex) -> control.TransferFunction: #given K>0
        """This function finds a the coeff K and a for a PI controller ,C(s) = K(s+a)/s. This function finds K using the Gain law
        assuming |s+a/s|=1, and a=|real(dom_pole)|/10 
        please enter one complex pole only - choice is arbitrary."""
        call_self = ControlToolbox()
        N_tmp,D_tmp = control.tfdata(transfer_func) 
        tf_numerator = call_self.improper_tf(((N_tmp[0])[0]))
        tf_denomirator = call_self.improper_tf(((D_tmp[0])[0]))
        tf_numerator_with_pole =  tf_numerator(dom_poles)
        tf_denomirator_with_pole = tf_denomirator(dom_poles)
        #calculating coeff
        a = (np.abs(np.real(dom_poles))/10)
        K=1/(np.linalg.norm((tf_numerator_with_pole/tf_denomirator_with_pole)))
        C_PI = control.tf([K,K*a],[1,0])
        return C_PI

    @staticmethod
    def PID_creator(transfer_func: control.TransferFunction, dom_poles: complex) -> list: #given K>0
        """This function finds a the coeff K and a for a PD controller ,C(s) = K(s+a_PD)*(s+a_PI)/s. it calculates 
        please enter one complex pole only - choice is arbitrary."""
        call_self = ControlToolbox()
        C_PD = call_self.PD_creator(transfer_func, dom_poles)
        a = (np.abs(np.real(dom_poles))/10)
        C_PI = control.tf([1,a],[1,0])
        return C_PD*C_PI

    @staticmethod   
    def get_second_order_imaginary_poles(wn: float, zeta: float) -> list:
        """calculates the poles of s^2+2wn*zeta*s+wn^2"""
        return [-wn * zeta + (wn * np.sqrt(1 - zeta**2)) * 1j, -wn * zeta - (wn * np.sqrt(1 - zeta**2)) * 1j]