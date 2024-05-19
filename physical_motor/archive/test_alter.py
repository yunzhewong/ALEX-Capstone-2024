import time
import physical_motor.aiosv2.aios as aios
import aiosv2

connected_addresses = aiosv2.get_addresses()


if connected_addresses is not None:
    print("INITIALIZATION")
    connected_addresses.enable()
    motor_ips = connected_addresses.getConnectedMotors()

    time.sleep(2)

    print("MOVEMENT")
    if motor_ips is not None:
        for motor_ip in motor_ips:

            p, v, c = aios.getCVP(motor_ip, 1)
            print("motor IP: ", motor_ip)
            print("Position", p)
            print("Velocity", v)
            print("Current", c)

            aios.controlMode(aios.ControlMode.POSITION_CONTROL.value, motor_ip, 1)
            aios.setPosition(0.5, 0, 0, True, motor_ip, 1)
            # aios.setVelocity(1, 0, True, motor_ip, 1)
            # aios.setCurrent(0.0500, True, motor_ip, 1)

            time.sleep(3)

            p, v, c = aios.getCVP(motor_ip, 1)
            print("motor IP: ", motor_ip)
            print("Position", p)
            print("Velocity", v)
            print("Current", c)

            time.sleep(3)
    else:
        print("No motor IPs found.")
else:
    print("No connected addresses found.")

time.sleep(3)

if connected_addresses is not None:
    connected_addresses.disable()
