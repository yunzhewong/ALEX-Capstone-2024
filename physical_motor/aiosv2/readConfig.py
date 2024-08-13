import json
import math
import os
from typing import List


def destructureMotorCombinationConfig(motorConfig: dict):
    control_box_ip = motorConfig["control_box_ip"]
    motors = motorConfig["motors"]

    expected_ips = [control_box_ip] + [motor["ip"] for motor in motors]

    return expected_ips, motors

def readConfigurationJSON(path: List[str]):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, *path)
    config_file = open(config_path)
    return json.loads(config_file.read())

NO_POSITION_LIMIT = 10 * math.pi

def removePositionLimits(motorConfigurations: List[dict]):
    for config in motorConfigurations:
        config["safetyConfiguration"]["position"]["soft"]["low"] = -1 * NO_POSITION_LIMIT
        config["safetyConfiguration"]["position"]["soft"]["high"] = NO_POSITION_LIMIT
        config["safetyConfiguration"]["position"]["hard"]["low"] = -1 * NO_POSITION_LIMIT
        config["safetyConfiguration"]["position"]["hard"]["high"] = NO_POSITION_LIMIT

def prepareConfigurationForCalibration(motorConfigurations: List[dict]):
    removePositionLimits(motorConfigurations)