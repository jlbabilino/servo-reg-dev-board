import time

sequence {
    0 -> disable_button
    0 -> position_control_button
    
    time.wait{30ms}
    (-0.007) -> motor_speed_setpoint
    1 -> speed_control_button
    time.wait{1000ms}
    0 -> speed_control_button
    0 -> motor_speed_setpoint

    time.wait{30ms}
    (-800) -> motor_position_setpoint
    1 -> position_control_button
    time.wait{500ms}
    0 -> position_control_button
    
    time.wait{30ms}
    1 -> disable_button
    time.wait{30ms}
    0 -> disable_button
}