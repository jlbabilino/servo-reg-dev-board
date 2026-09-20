import time

sequence {
    stage on {
        1 -> motor_speed_setpoint
        time.wait{50ms} => off
    }

    stage off {
        0 -> motor_speed_setpoint
        time.wait{150ms} => on
    }
}