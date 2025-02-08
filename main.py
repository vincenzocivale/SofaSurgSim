from ros_interface import ROSInterface
from sofa_scene_controller import SOFASceneController

def main():
    ros_interface = ROSInterface()
    sofa_scene_controller = SOFASceneController(ros_interface, True)
    sofa_scene_controller.run_simulation()
    
if __name__ == "__main__":
    main()