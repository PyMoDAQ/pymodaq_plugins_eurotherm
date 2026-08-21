
from typing import Union, List, Dict
from pymodaq_utils.config import GlobalConfig


from pymodaq.control_modules.move_utility_classes import (DAQ_Move_base, comon_parameters_fun,
                                                          main, DataActuatorType, DataActuator)

try:
    from pymodaq.control_modules.daq_move_ui.utils import UiType
    no_ui_type = False
except ImportError:
    no_ui_type= True

from pymodaq_utils.utils import ThreadCommand  # object used to send info back to the main thread
from pymodaq_gui.parameter import Parameter
from pymodaq_plugins_utils.hardware.serial_ports import list_resources
from nupylab.drivers.eurotherm3216 import Eurotherm3216

config = GlobalConfig()

KP_ADDRESS = 6
KI_ADDRESS = 8
KD_ADDRESS = 9

class DAQ_Move_Eurotherm3200Series(DAQ_Move_base):
    """ Instrument plugin class for an actuator.
    
    This object inherits all functionalities to communicate with PyMoDAQ’s DAQ_Move module through inheritance via
    DAQ_Move_base. It makes a bridge between the DAQ_Move module and the Python wrapper of a particular instrument.


    Attributes:
    -----------
    controller: object
        The particular object that allow the communication with the hardware, in general a python wrapper around the
         hardware library.

    """
    is_multiaxes = True
    _axis_names: Union[List[str], Dict[str, int]] = dict(zip([f'Segment_{ind:03.0f}' for ind in range(8)],
                                                               [ind for ind in range(8)]))
    _controller_units: Union[str, List[str]] = '°C'
    _epsilon: Union[float, List[float]] = 0.1

    data_actuator_type = DataActuatorType.DataActuator
    if not no_ui_type:
        ui_type = UiType.NONE
    has_encoder = True


    params = [
        {'title': 'ComPort:', 'name': 'port', 'type': 'list', 'limits': list_resources(), 'value': config('eurotherm', 'port')},
        {'title': 'Client Address', 'name': 'clientaddress', 'type': 'int', 'value': config('eurotherm', 'clientaddress')},
        {'title': 'BaudRate:', 'name': 'baudrate', 'type': 'int', 'value': config('eurotherm', 'baudrate'),},
         {'title': 'PID:', 'name': 'pid', 'type': 'group', 'children': [
             {'title': 'Kp', 'name': 'kp', 'type': 'float', 'value': 1.},
            {'title': 'Ki', 'name': 'ki', 'type': 'float', 'value': 1.},
            {'title': 'Kd', 'name': 'kd', 'type': 'float', 'value': 1.},
         ],},


             ] + comon_parameters_fun(is_multiaxes, axis_names=_axis_names, epsilon=_epsilon)

    def ini_attributes(self):
        self.controller: Eurotherm3216 = None

    def get_actuator_value(self) -> DataActuator:
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        temp_value = self.controller.process_value
        pos = DataActuator(data=temp_value,
                           units=self.axis_unit)
        pos = self.get_position_with_scaling(pos)

        return pos

    def user_condition_to_reach_target(self) -> bool:
        """ Implement a condition for exiting the polling mechanism and specifying that the
        target value has been reached

       Returns
        -------
        bool: if True, PyMoDAQ considers the target value has been reached
        """
        # TODO either delete this method if the usual polling is fine with you, but if need you can
        #  add here some other condition to be fullfilled either a completely new one or
        #  using or/and operations between the epsilon_bool and some other custom booleans
        #  for a usage example see DAQ_Move_brushlessMotor from the Thorlabs plugin
        return True

    def close(self):
        """Terminate the communication protocol"""
        if self.is_master:
            self.move_abs(DataActuator(self._title, data=30))

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        if param.name() == 'kp' :
            self.controller.write_float(registeraddress=KP_ADDRESS, val= param.value())
        elif param.name() == 'ki' :
            self.controller.write_float(registeraddress=KI_ADDRESS, val= param.value())
        elif param.name() == 'kd' :
            self.controller.write_float(registeraddress=KD_ADDRESS, val= param.value())

    def update_pid(self):
        self.settings.child('pid', 'kp').setValue(self.controller.read_float(registeraddress=KP_ADDRESS))
        self.settings.child('pid', 'ki').setValue(self.controller.read_float(registeraddress=KI_ADDRESS))
        self.settings.child('pid', 'kd').setValue(self.controller.read_float(registeraddress=KD_ADDRESS))

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        if self.is_master:
            self.controller = Eurotherm3216(port=self.settings['port'],
                                             clientaddress=self.settings['clientaddress'],)

            initialized = True

        else:
            self.controller = controller
            initialized = True

        self.settings.child('scaling','use_scaling').setValue(True)
        self.settings.child('scaling','offset').setValue(273)
        self.update_pid()
        info = "Whatever info you want to log"
        return info, initialized

    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """
        value = self.check_bound(value)  #if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one
        self.controller.target_setpoint=value.value(self.axis_unit)
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_value + value) - self.current_value
        self.target_value = value + self.current_value
        value = self.set_position_relative_with_scaling(value)
        self.controller.target_setpoint = self.target_value.value(self.axis_unit)
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))

    def move_home(self):
        """Call the reference method of the controller"""
        self.controller.target_setpoint = 27
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))

    def stop_motion(self):
        """Stop the actuator and emits move_done signal"""
        temp_value = self.controller.process_value
        self.controller.target_setpoint  = temp_value
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))

if __name__ == '__main__':
    main(__file__, init=False)
