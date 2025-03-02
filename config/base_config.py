import logging

class BaseConfig:
    """Configurazioni base convalide"""
    
    # Configurazione del logger
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    
    # Parametri ROS
    ROS_HOST = 'localhost'
    ROS_PORT = 9090

    # Parametri SOFA
    GUI = True
    SIMULATION_STEP = 0.01  

    ORGAN_TOPIC = '/organs'
    ORGAN_TOPIC_TYPE = 'sofa_surgical_msgs/Organ'

    ORGANS_SERVICE = '/get_organ'
    ORGANS_SERVICE_TYPE = 'sofa_surgical_msgs/GetOrgan'

config = BaseConfig()

