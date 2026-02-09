from joint import Joint
import json
def load_joints(path, driver1, driver2):
    with open(path, "r") as f:
        data = json.load(f)
    joints = {}
    for channelString, config in data.items():
        channel = int(channelString)
        if channel < 16:
            driver = driver1
            driverChannel = channel
        else:
            driver = driver2
            driverChannel = channel - 16
        joints[channel] = Joint(driver, driverChannel, min(config["plus_90"], config["minus_90"]), max(config["plus_90"], config["minus_90"]), config["neutral"], config.get("direction",1))
