import json
import os


def readConfig(name: str):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "config", name)
    config_file = open(config_path)
    configuration = json.loads(config_file.read())

    control_box_ip = configuration["control_box_ip"]
    motors = configuration["motors"]

    expected_ips = [control_box_ip] + [motor["ip"] for motor in motors]

    return expected_ips, motors
