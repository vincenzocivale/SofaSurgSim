import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sofasurgsim.interfaces.ros_interface import ROSClient
from sofasurgsim.interfaces.sofa_interface import SOFASceneController
from config import base_config
from organ_publisher import publish_organ

def main():
    cfg = base_config.BaseConfig()
   
    ros_client = ROSClient(cfg.ROS_HOST)
    ros_client.connect()
    sofa_controller = SOFASceneController(ros_client, cfg.GUI)
    sofa_controller.run_simulation()
    
    ros_client.disconnect()

if __name__ == "__main__":
    main()
