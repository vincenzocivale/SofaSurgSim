class BaseConfig:
    """Configurazioni base convalide"""
    
    # Parametri ROS
    ROS_HOST = 'localhost'
    ROS_PORT = 9090
    ROS_TOPICS = {
       
    }

    ROS_CLIENT = None

    # Parametri SOFA
    GUI = True
    SOFA_SCENE_FILE = 'scenes/main_scene.py'
    SIMULATION_STEP = 0.01  # Secondi

    # Validazione parametri
    def validate(self):
        assert isinstance(self.ROS_PORT, int), "ROS_PORT deve essere intero"
        assert self.SIMULATION_STEP > 0, "SIMULATION_STEP deve essere > 0"