import logging
import Sofa
import Sofa.Core
import Sofa.Simulation
import numpy as np
import queue

from sofasurgsim.msg.Organ import Organ
from sofasurgsim.interfaces.ros_interface import ROSClient
from config import base_config

# Configuriamo il logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class OrganManager(Sofa.Core.Controller):
    def __init__(self, *args, ros_client: ROSClient, **kwargs):
        super().__init__(*args, **kwargs)
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
        # Questo callback viene eseguito in un thread separato: mettiamo il messaggio in coda
        self.organ_queue.put(msg)
    
    def updateGraph(self, node, dt):
        """
        Metodo chiamato durante il ciclo principale della simulazione.
        Verifica se ci sono nuovi messaggi nella coda e aggiunge i relativi nodi.
        """
        print(f"updateGraph chiamato con dt={dt}", flush=True)
        while not self.organ_queue.empty():
            msg = self.organ_queue.get()
            self.create_new_organ(msg)
    
    def create_new_organ(self, msg):
        """Crea un nuovo organo a partire da un messaggio ROS."""
        try:
            organ = Organ.from_dict(msg)
            
            if organ.id in self.created_organs:
                logger.warning(f"Organ {organ.id} already exists!")
                return

            # Otteniamo il nodo radice (contesto) e aggiungiamo il nuovo nodo
            root = self.getContext()
            new_organ = root.addChild(organ.id)

            # Estraiamo i vertici e i triangoli dalla mesh
            vertices = np.array([[v.x, v.y, v.z] for v in organ.mesh.vertices], dtype=np.float32)
            triangles = np.array([t.vertex_indices for t in organ.mesh.triangles], dtype=np.uint32)
            
            # Aggiungiamo i componenti di solver e simulazione
            new_organ.addObject('EulerImplicitSolver', name="odesolver")
            new_organ.addObject('CGLinearSolver', name="linear_solver", iterations=100)
            
            new_organ.addObject('MechanicalObject', name="dofs", position=vertices.tolist())
            new_organ.addObject('TriangleSetTopologyContainer', name="topology", triangles=triangles.tolist(), position=vertices.tolist())
            new_organ.addObject('UniformMass', totalMass=1.0)
            new_organ.addObject('TriangularFEMForceField', youngModulus=3000, poissonRatio=0.3)
            
            self._add_visual_components(new_organ, vertices, triangles)
            self._add_collision_components(new_organ)
            
            self.created_organs[organ.id] = new_organ
            logger.info(f"Organ {organ.id} successfully created!")
        
        except Exception as e:
            logger.error(f"Error creating organ: {str(e)}", exc_info=True)

    def _add_visual_components(self, node, vertices, triangles):
        """Aggiunge componenti per la visualizzazione 3D."""
        visu = node.addChild("Visual")
        visu.addObject('OglModel', name="VisualModel", position=vertices.tolist(), triangles=triangles.tolist(), color=[1.0, 0.5, 0.5, 1.0])
        visu.addObject('BarycentricMapping', input="@../dofs", output="@VisualModel")

    def _add_collision_components(self, node):
        """Aggiunge componenti per la collisione."""
        collision = node.addChild("Collision")
        collision.addObject('MechanicalObject', name="collision_dofs")
        collision.addObject('PointCollisionModel', name="CollisionModel")
        collision.addObject('BarycentricMapping', input="@../dofs", output="@collision_dofs")
