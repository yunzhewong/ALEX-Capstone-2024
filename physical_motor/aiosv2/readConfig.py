import json
import math
import os
from typing import List


def readConfig(path: List[str]):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, *path)
    config_file = open(config_path)
    configuration = json.loads(config_file.read())

    control_box_ip = configuration["control_box_ip"]
    motors = configuration["motors"]

    expected_ips = [control_box_ip] + [motor["ip"] for motor in motors]

    return expected_ips, motors

NO_POSITION_LIMIT = 10 * math.pi

def removeIntegralVelocity(motorConfigurations: List[dict]):
    for config in motorConfigurations:
        config["velocityI"] = 0

def removePositionLimits(motorConfigurations: List[dict]):
    for config in motorConfigurations:
        config["safetyConfiguration"]["position"]["soft"]["low"] = -1 * NO_POSITION_LIMIT
        config["safetyConfiguration"]["position"]["soft"]["high"] = NO_POSITION_LIMIT
        config["safetyConfiguration"]["position"]["hard"]["low"] = -1 * NO_POSITION_LIMIT
        config["safetyConfiguration"]["position"]["hard"]["high"] = NO_POSITION_LIMIT

def prepareConfigurationForCalibration(motorConfigurations: List[dict]):
    removeIntegralVelocity(motorConfigurations)
    removePositionLimits(motorConfigurations)