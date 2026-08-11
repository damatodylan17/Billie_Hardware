from typing import Callable, Optional
from can import CanOperationError
import can
import os
import sys
from dataclasses import dataclass
import cybergear_driver as cgd
import motor_config
import time
import signal


@dataclass
class MotorData:
    """ Dataclass for storing motor state

    Contains commanded and most recent read state from the motor,
    as well as the current motor parameters and limits.    
    """
    motor_id: int
    motor_type: str
    current_position: float = None
    current_velocity: float = None
    current_torque: float = None
    current_temperature: float = None 
    commanded_position: float = 0
    commanded_velocity: float = 0
    commanded_torque: float = 0
    kp: float = 0
    kd: float = 0
    torque_limits: tuple = (0,0)
    velocity_limits: tuple = (0,0)
    position_limits: tuple = (0,0)
    kp_limits: tuple = (0,0)
    kd_limits: tuple = (0,0)
    timestamp: float = -1
    initialized: bool = False
    unique_hardware_id: int = None
    running_torque: list = None
    rms_torque: float = 0
    motor_mode: float = 0
    internal_params = {}

    def __post_init__(self):
        """Initializes the motor limits based on the motor type

        Raises:
            ValueError: Raised if the motor type is not specified
        """
        if self.motor_type is None:
            raise ValueError('motor_type must be specified')
        self.position_limits, self.velocity_limits, self.torque_limits, self.kp_limits, self.kd_limits = motor_config.get_motor_limits(self.motor_type)

class CyberGearListener(can.Listener):
    def __init__(self, motors: dict[int,str], allow_strangers: bool=False) -> None:
        super().__init__()
        self._allow_strangers = allow_strangers
        self._motors: dict[int, MotorData] = {}
        for motor_id, motor_type in motors.items():
            self._motors[motor_id] = MotorData( motor_id=motor_id, motor_type=motor_type)
        
    def on_message_received(self, msg: can.Message):
        if (not msg.is_extended_id) or msg.is_error_frame or (not msg.is_rx): return -1

        communication_type = (msg.arbitration_id & 0x3F000000) >> 24
        target_id = msg.arbitration_id & 0xFF

        if (target_id != cgd.MASTER_CAN_ID) and (target_id != cgd.RESPONSE_IDENTITY_CHECK_FLAG): return -1
        
        if communication_type == cgd.RESPONSE_IDENTITY:
            unique_id, can_id = cgd.parse_identity_response(msg)
            if self._allow_strangers and (can_id not in self._motors.keys()):
                self._motors[can_id] = MotorData(motor_id=can_id, unique_hardware_id=unique_id)
            elif can_id in self._motors.keys():
                self._motors[can_id].unique_hardware_id=unique_id

        if communication_type == cgd.RESPONSE_PARAM:
            param_index, data_value, can_id = cgd.parse_param_response(msg)
            self._motors[can_id].internal_params[param_index] = data_value

        if communication_type == cgd.RESPONSE_FAULT: return

        if communication_type == cgd.RESPONSE_MOTION:
            pos, vel, trq, temp, can_id, motor_mode, cal_fault, hall_enc_fault, mag_enc_fault, overtemp_fault, overcurr_fault, undervolt_fault = cgd.parse_motion_response(msg)
            self._motors[can_id].current_position = pos
            self._motors[can_id].current_velocity = vel
            self._motors[can_id].current_torque = trq
            self._motors[can_id].motor_mode = motor_mode
            self._motors[can_id].current_temperature = temp
            

class CyberGear():
    def __init__(self, motors: dict[int,str], channel:str ='can0', bustype: str='socketcan', invert:Optional[list[int]]=None, verbose: bool=False) -> None:
        cgd.load_can_drivers()

        self._bus = can.Bus(channel=channel, bustype=bustype, receive_own_messages=False)
        self._cybergear_listener = CyberGearListener(motors)
        self._notifier = can.Notifier(self._bus, [self._cybergear_listener])
        self._enabled_for_motion = False
        self._connection_established = True
        self.verbose = verbose
        self._priming_connection = False

        self.prev_command_time = time.perf_counter()
        
        if invert is None:
            invert = []

        self.invert_dict = {motor_id:-1 if motor_id in invert else 1 for motor_id in motors.keys()}

        signal.signal(signal.SIGINT, self._exit_handler)
        signal.signal(signal.SIGTERM, self._exit_handler)

        self.enable_for_motion()

        time.sleep(0.5)
        if verbose: print('CyberGear Motors Ready')


    def _guard_motor_connection(func: Callable):
        def wrapper(self, *args, **kwargs):
            if not self._connection_established:
                return -1 # Connection Lost
            try:
                res = func(self, *args, **kwargs)
            except CanOperationError as e:
                self._connection_established = False
                print(f'Connection Lost, Check Device Connections')
                return -1
            return res
        return wrapper
    
    def _exit_handler(self, sig, frame):
        if self.verbose: os.write(sys.stdout.fileno(), b'------ Shutting Down ------\n')
        if self._connection_established:
            try: self.cleanup()
            except:
                os.write(sys.stdout.fileno(), b'Error during cleanup\n')
        if self.verbose: os.write(sys.stdout.fileno(), b'------ Shutdown Complete ------\n')
        sys.exit(0)

    def _raw_set_zero_torque(self, motor_id: int) -> None:
        self._send_motion_command(motor_id, 0, 0, 0, 0, 0)

    def set_zero_position(self, motor_id: int) -> None:
        self._bus.send(cgd.create_zero_position_message(target_motor_id=motor_id)) # Set the Zero Position
        self._raw_set_zero_torque(motor_id) # Set the Zero Torque

    def ping_actuator(self, act_id: int) -> None:
        self._bus.send(cgd.create_read_device_id_message(act_id))

    @_guard_motor_connection
    def set_position(self, motor_id: int, position: float, kp: float, kd: float) -> None:
        return self._send_motion_command(motor_id, position, 0, 0, kp, kd)

    @_guard_motor_connection
    def set_velocity(self, motor_id: int, velocity: float, kd: float) -> None:
        return self._send_motion_command(motor_id, 0, velocity, 0, 0, kd)

    @_guard_motor_connection
    def set_torque(self, motor_id: int, torque: float) -> int:
        return self._send_motion_command(motor_id, 0, 0, torque, 0, 0)

    def _send_motion_command(self, motor_id: int, position: float, velocity: float, torque: float, kp: float, kd: float) -> int:
        if motor_id not in self._cybergear_listener._motors.keys(): return 0

        self._cybergear_listener._motors[motor_id].commanded_position = position
        self._cybergear_listener._motors[motor_id].commanded_velocity = velocity
        self._cybergear_listener._motors[motor_id].commanded_torque = torque
        self._cybergear_listener._motors[motor_id].kp = kp
        self._cybergear_listener._motors[motor_id].kd = kd

        motion_message = cgd.create_motion_message(target_motor_id=motor_id, position=position, velocity=velocity, kp=kp, kd=kd, torque=torque)
        self._bus.send(motion_message)

        return 1

    def get_data(self, motor_id: int) -> MotorData:
        if motor_id in self._cybergear_listener._motors.keys():
            return self._cybergear_listener._motors[motor_id]
        return None

    def get_position(self, motor_id: int) -> float:
        return self.get_data(motor_id).current_position

    def get_velocity(self, motor_id: int) -> float:
        return self.get_data(motor_id).current_velocity
    
    def get_torque(self, motor_id: int) -> float:
        return self.get_data(motor_id).current_torque

    def cleanup(self):
        for act_id in self._cybergear_listener._motors.keys():
            disable_message = cgd.create_disable_motion_message(act_id)
            self._bus.send(disable_message)

    def enable_for_motion(self):
        for act_id in self._cybergear_listener._motors.keys():
            self.ping_actuator(act_id)
            enable_motion_message = cgd.create_enable_motion_message(target_motor_id=act_id)
            self._bus.send( enable_motion_message, timeout=0.1 )
            print(f'Enabling motion for Actuator {act_id}')
            time.sleep(0.1)
            self._raw_set_zero_torque(act_id)

        self._enabled_for_motion = True


if __name__ == "__main__":
    MOTOR_ID = 0x5
    motors = CyberGear({MOTOR_ID:"Cybergear"})
    print(motors.get_data(MOTOR_ID))
