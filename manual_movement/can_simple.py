"""
Minimal example for controlling an ODrive via the CANSimple protocol.

Puts the ODrive into closed loop control mode, sends a velocity setpoint of 1.0
and then prints the encoder feedback.

Assumes that the ODrive is already configured for velocity control.

See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for protocol
documentation.
"""

import can
import struct
import time

"""
0x02: 041
0x03: 061
0x04: 081
0x05: 0a1
0x06: 0c1
0x07: 0e1

Order:
0x04
0x02
0x06
0x05
0x07
0x03
"""

# True Max is 32
MAX_SPEED = 24
MAX_VALUE = 32768
ID_ORDER = [0x04, 0x02, 0x05]
node_id = 0x02 # must match `<odrv>.axis0.config.can.node_id`. The default is 0.

def close_loop():
    for id in ID_ORDER:
        bus.send(can.Message(
            arbitration_id=(id << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
            is_extended_id=False
        ))

    print("done")

def move(velocity):
    counter = 0
    for msg in bus:
        #print(msg)
        if msg.arbitration_id == (ID_ORDER[counter] << 5 | 0x01): # 0x01: Heartbeat
            _, state, __, ___ = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state != 8: # 8: AxisState.CLOSED_LOOP_CONTROL
                continue
        else:
            continue

        bus.send(can.Message(
            arbitration_id=(ID_ORDER[counter] << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', float(velocity), 0.0), # 1.0: velocity, 0.0: torque feedforward
            is_extended_id=False
        ))

        counter += 1

        if counter >= len(ID_ORDER):
            break

bus = can.interface.Bus("can0", bustype="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

"""
close_loop()

print("moving")
move(5)

time.sleep(5)

print("stopping")
move(0)

"""
# Put axis into closed loop control state
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
    data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))

# Wait for axis to enter closed loop control by scanning heartbeat messages
for msg in bus:
    print(msg)
    if msg.arbitration_id == (node_id << 5 | 0x01): # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == 8: # 8: AxisState.CLOSED_LOOP_CONTROL
            break


# Set velocity to 1.0 turns/s
print("moving")
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
    data=struct.pack('<ff', 25.0, 0.0), # 1.0: velocity, 0.0: torque feedforward
    is_extended_id=False
))

time.sleep(5)

bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
    data=struct.pack('<ff', 0.0, 0.0), # 1.0: velocity, 0.0: torque feedforward
    is_extended_id=False
))






print("stopped")
"""
# Print encoder feedback
for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
        pos, vel = struct.unpack('<ff', bytes(msg.data))
        print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")
"""

bus.shutdown()