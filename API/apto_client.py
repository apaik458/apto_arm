#!/usr/bin/env python
from STservo_sdk import *
import numpy as np
import atexit


# Data Byte Length
LEN_PRESENT_POSITION = 2
LEN_GOAL_POSITION    = 2


def cleanup_handler():
    """Cleanup function to ensure motors are disconnected properly."""
    open_clients = list(AptoClient.OPEN_CLIENTS)
    for open_client in open_clients:
        if open_client.port_handler.is_using:
            print('Forcing client to close.')
        open_client.port_handler.is_using = False
        open_client.disconnect()

def signed_to_unsigned(value: int, size: int) -> int:
    """Converts the given value to its unsigned representation"""
    if value < 0:
        bit_size = 8 * size
        max_value = (1 << bit_size) - 1
        value = max_value + value
    return value

def unsigned_to_signed(value: int, size: int) -> int:
    """Converts the given value from its unsigned representation"""
    bit_size = 8 * size
    if (value & (1 << (bit_size - 1))) != 0:
        value = -((1 << bit_size) - value)
    return value

## Conversions (0 -> 2048 -> 4096 = pi -> 0 -> -pi), (positive motor rotation = clockwise)
def pos_scale_atv(angle):
    """Converts from joint angles to motor values"""
    return (-angle / np.pi + 1) * 2048

def pos_scale_vta(value):
    """Converts from motor values to joint angles"""
    return -value * (2 * np.pi / 4096) + np.pi


class AptoClient:
    """
    Client for communicating with Waveshare motors
    """

    # The currently open clients.
    OPEN_CLIENTS = set()
    
    def __init__(self,
                 motor_ids,
                 port: str = 'COM4',
                 baudrate = 1000000):
        """Initialises a new client"""
        
        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate

        self.port_handler = PortHandler(port)
        self.packet_handler = sts(self.port_handler)

        self._pos_reader = AptoPosReader(
            self,
            self.motor_ids
        )

        self._sync_writers = {}
        self.OPEN_CLIENTS.add(self)

    def is_connected(self):
        return self.port_handler.is_open

    def connect(self):
        """Connects to the Waveshare motors"""
        # Typo in port_handler.py, clearPort()?
        # assert not self.is_connected, 'Client is already connected.'

        if self.port_handler.openPort():
            print('Succeeded to open port: %s', self.port_name)
        else:
            raise OSError(
                ('Failed to open port at {} (Check that the device is powered '
                 'on and connected to your computer).').format(self.port_name))

        if self.port_handler.setBaudRate(self.baudrate):
            print('Succeeded to set baudrate to %d', self.baudrate)
        else:
            raise OSError(
                ('Failed to set the baudrate to {} (Ensure that the device was '
                 'configured for this baudrate).').format(self.baudrate))
    
    def disconnect(self):
        """Disconnects from the Waveshare device"""
        if not self.is_connected:
            return
        
        if self.port_handler.is_using:
            print('Port handler in use; cannot disconnect.')
            return
        
        # Ensure motors are disabled at the end.
        self.set_torque_enabled(self.motor_ids, False, retries=0)
        self.port_handler.closePort()
        if self in self.OPEN_CLIENTS:
            self.OPEN_CLIENTS.remove(self)

    def set_torque_enabled(self,
                           motor_ids,
                           enabled,
                           retries = -1,
                           retry_interval = 0.25):
        """Sets whether torque is enabled for the motors"""
        remaining_ids = list(motor_ids)
        while remaining_ids:
            remaining_ids = self.write_byte(remaining_ids, int(enabled), STS_TORQUE_ENABLE)
            if remaining_ids:
                print('Could not set torque %s for IDs: %s',
                    'enabled' if enabled else 'disabled',
                    str(remaining_ids))
            if retries == 0:
                break
            time.sleep(retry_interval)
            retries -= 1

    def read_pos(self):
        """Returns the current positions and velocities"""
        return self._pos_reader.read().round(3)
    
    def write_desired_pos(self, motor_ids, positions):
        """Writes the given desired positions"""
        assert len(motor_ids) == len(positions)
        
        positions = pos_scale_atv(positions)
        self.sync_write(motor_ids, positions, STS_GOAL_POSITION_L, LEN_GOAL_POSITION)

    def write_byte(self, motor_ids, value, address):
        """Writes a value to the motors"""
        self.check_connected()
        errored_ids = []
        for motor_id in motor_ids:
            sts_comm_result, sts_error = self.packet_handler.write1ByteTxRx(motor_id, address, value)
            success = self.handle_packet_result(
                sts_comm_result, sts_error, motor_id, context='write_byte')
            if not success:
                errored_ids.append(motor_id)
        return errored_ids

    def sync_write(self, motor_ids, values, address, size):
        """Writes values to a group of motors"""
        self.check_connected()
        key = (address, size)
        if key not in self._sync_writers:
            self._sync_writers[key] = GroupSyncWrite(self.packet_handler, address, size)
        sync_writer = self._sync_writers[key]

        errored_ids = []
        for motor_id, desired_pos in zip(motor_ids, values):
            value = signed_to_unsigned(int(desired_pos), size=size)
            value = value.to_bytes(size, byteorder='little')
            success = sync_writer.addParam(motor_id, value)
            if not success:
                errored_ids.append(motor_id)

        if errored_ids:
            print('Sync write failed for: %s', str(errored_ids))

        comm_result = sync_writer.txPacket()
        self.handle_packet_result(comm_result, context='sync_write')

        sync_writer.clearParam()

    def check_connected(self):
        """Ensures the robot is connected"""
        if not self.is_connected:
            self.connect()
            raise OSError('Must call connect() first.')

    def handle_packet_result(self,
                             comm_result,
                             sts_error = None,
                             sts_id = None,
                             context = None):
        """Handles the result from a communication request"""
        error_message = None
        if comm_result != COMM_SUCCESS:
            error_message = self.packet_handler.getTxRxResult(comm_result)
        elif sts_error is not None:
            error_message = self.packet_handler.getRxPacketError(sts_error)
        if error_message:
            if sts_id is not None:
                error_message = '[Motor ID: {}] {}'.format(sts_id, error_message)
            if context is not None:
                error_message = '> {}: {}'.format(context, error_message)
            print(error_message)
            return False
        return True
    
    def convert_to_unsigned(self, value: int, size: int) -> int:
        """Converts the given value to its unsigned representation"""
        if value < 0:
            max_value = (1 << (8 * size)) - 1
            value = max_value + value
        return value

class AptoReader:
    """
    Reads data from Waveshare motors
    """

    def __init__(self, client, motor_ids, address, size):
        """Initialises a new reader"""
        self.client = client
        self.motor_ids = motor_ids
        self.address = address
        self.size = size
        self._initialise_data()

        self.operation = GroupSyncRead(client.packet_handler, address, size)

        for motor_id in motor_ids:
            success = self.operation.addParam(motor_id)
            if not success:
                raise OSError('[Motor ID: {}] Could not add parameter to bulk read.'.format(motor_id))

    def read(self, retries = 1):
        """Reads data from the motors"""
        self.client.check_connected()
        success = False
        while not success and retries >= 0:
            comm_result = self.operation.txRxPacket()
            success = self.client.handle_packet_result(comm_result, context='read')
            retries -= 1

        # If we failed, send a copy of the previous data.
        if not success:
            return self._get_data()

        errored_ids = []
        for i, motor_id in enumerate(self.motor_ids):
            # Check if the data is available.
            available = self.operation.isAvailable(motor_id, self.address, self.size)
            if not available:
                errored_ids.append(motor_id)
                continue

            self._update_data(i, motor_id)

        if errored_ids:
            print('Bulk read data is unavailable for: %s', str(errored_ids))

        return self._get_data()

    def _initialise_data(self):
        """Initialises the cached data"""
        self._data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index, motor_id):
        """Updates the data index for the given motor ID"""
        self._data[index] = self.operation.getData(motor_id, self.address, self.size)

    def _get_data(self):
        """Returns a copy of the data"""
        return self._data.copy()

class AptoPosReader(AptoReader):
    """Reads positions"""

    def __init__(self,
                 client,
                 motor_ids):
        super().__init__(
            client,
            motor_ids,
            address=STS_PRESENT_POSITION_L,
            size=LEN_PRESENT_POSITION,
        )

    def _initialise_data(self):
        """Initialises the cached data"""
        self._pos_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index, motor_id):
        """Updates the data index for the given motor ID"""
        pos = self.operation.getData(motor_id, STS_PRESENT_POSITION_L, LEN_PRESENT_POSITION)
        pos = unsigned_to_signed(pos, size=4)
        self._pos_data[index] = pos_scale_vta(pos)
    
    def _get_data(self):
        """Returns a copy of the data"""
        return self._pos_data.copy()

# Register global cleanup function.
atexit.register(cleanup_handler)
