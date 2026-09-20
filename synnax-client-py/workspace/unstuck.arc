import time

sequence {
    stage back_out {
        (-1) -> motor_speed_setpoint
        time.wait{300ms} => next
    }

    0 -> motor_speed_setpoint
}