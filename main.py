from src.sofasurgsim.interfaces.ros_interface import ROSClient
from src.sofasurgsim.interfaces.sofa_interface import SOFASceneController

def main():
    ros_ip = 'localhost'
    GUI = False
    ros_client = ROSClient(ros_ip)
    sofa_controller = SOFASceneController(ros_ip, GUI)
    sofa_controller.run_simulation()