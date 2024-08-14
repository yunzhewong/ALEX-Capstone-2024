import json
import math
import os
from typing import List




def readConfigurationJSON(path: List[str]):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, *path)
    config_file = open(config_path)
    return json.loads(config_file.read())

def writeConfigurationJSON(dict: dict, path: List[str]):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, *path)
    config_file = open(config_path, 'w')
    json.dump(dict, config_file, ensure_ascii=False, indent=4)

def destructureMotorCombinationConfig(motorConfig: dict):
    control_box_ip = motorConfig.get("control_box_ip", None)
    motors = motorConfig["motors"]

    expected_ips = [motor["ip"] for motor in motors]
    if control_box_ip is not None:
        expected_ips.append(control_box_ip)

    return expected_ips, motors

NO_POSITION_LIMIT = 10 * math.pi
# Not sure how to make it clear that it modifies motorConfigurations in place
def removePositionLimits(motorConfig: dict):
    _, motors = destructureMotorCombinationConfig(motorConfig)
    for config in motors:
        config["safetyConfiguration"]["position"]["soft"]["low"] = -1 * NO_POSITION_LIMIT
        config["safetyConfiguration"]["position"]["soft"]["high"] = NO_POSITION_LIMIT
        config["safetyConfiguration"]["position"]["hard"]["low"] = -1 * NO_POSITION_LIMIT
        config["safetyConfiguration"]["position"]["hard"]["high"] = NO_POSITION_LIMIT
