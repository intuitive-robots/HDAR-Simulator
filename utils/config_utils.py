import yaml


def __get_config(config_name):
    qr_config_path = "config.yaml" 
    with open(qr_config_path, "r") as f:
        task_setting = yaml.load(f, Loader=yaml.FullLoader)
    return task_setting[config_name]

def get_real_robot_config():
    return __get_config("RealRobotConfig")

def get_virtual_robot_config():
    return __get_config("VirtualRobotConfig")

def get_hdar_config():
    return __get_config("HDARConfig")

def get_qr_config():
    return __get_config("QRConfig")

def get_simulator_config():
    return __get_config("SimulatorConfig")