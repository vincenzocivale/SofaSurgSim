import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sofasurgsim.interfaces.ros_interface import ROSClient
from sofasurgsim.interfaces.sofa_interface import SOFASceneController
from config.base_config import config as cfg

def callback(message):
    """Funzione di callback per la ricezione di messaggi."""
    print("Received:", message)

def main():
   
    ros_client = ROSClient(cfg.ROS_HOST)
    ros_client.connect()
    
    # Avvio della simulazione
    sofa_controller = SOFASceneController(ros_client)
    sofa_controller.run_simulation()
    
    # Il codice qui sotto non verrà eseguito finché la simulazione è attiva.
    ros_client.disconnect()

if __name__ == "__main__":
    main()
