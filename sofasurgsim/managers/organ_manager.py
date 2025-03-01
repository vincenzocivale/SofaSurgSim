import logging
import Sofa
import Sofa.Core
import Sofa.Simulation
import numpy as np
import queue
import os

from sofasurgsim.msg.Organ import Organ
from sofasurgsim.interfaces.ros_interface import ROSClient
from config import base_config

# Configuriamo il logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class OrganManager(Sofa.Core.Controller):
    def __init__(self, *args, root_node, ros_client: ROSClient, **kwargs):
        super().__init__(*args, **kwargs)
        self.root_node = root_node
        self.ros_client = ros_client
        self.bs = base_config.BaseConfig()
        self.created_organs = {}
        # Coda per gestire i messaggi ROS in modo thread-safe
        self.organ_queue = queue.Queue()
        
        # Il callback aggiunge i messaggi alla coda
        self.ros_client.create_subscriber(
            self.bs.ORGAN_TOPIC, 
            self.bs.ORGAN_TOPIC_TYPE, 
            self._queue_new_organ
        )

    def _queue_new_organ(self, msg):
        if msg['id'] in self.created_organs:
            logger.warning(f"Organ {msg['id']} already exists!")
        else:
            # Questo callback viene eseguito in un thread separato: mettiamo il messaggio in coda
            self.organ_queue.put(msg)
    
    def onAnimateBeginEvent(self, event):
        # Questo metodo viene chiamato all'inizio di ogni ciclo di animazione
        if not self.organ_queue.empty():
            msg = self.organ_queue.get()
            self.create_new_organ(msg)
            logger.info(f"Organ queue size: {self.organ_queue.qsize()}")
    
   