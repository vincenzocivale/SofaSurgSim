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
    
    def onAnimateBeginEvent(self, dt):
        # Questo metodo viene chiamato all'inizio di ogni ciclo di animazione
        if not self.organ_queue.empty():
            msg = self.organ_queue.get()
            self.create_new_organ(msg)
    
    def create_new_organ(self, msg):
        """Crea un nuovo organo a partire da un messaggio ROS."""
        try:
            logger.info(f"Creating new organ from message")
            organ = Organ.from_dict(msg)
            
            if organ.id in self.created_organs:
                logger.warning(f"Organ {organ.id} already exists!")
                return

            # Otteniamo il nodo radice (contesto) e aggiungiamo il nuovo nodo
            root = self.getContext()
            new_organ = root.addChild(str(organ.id))

            vertices_struct = [[v.x, v.y, v.z] for v in organ.tetrahedral_mesh.vertices]
            tetrahedra = organ.tetrahedral_mesh.tetrahedra
            
            new_organ.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
            new_organ.addObject('CGLinearSolver', name="linear_solver", iterations="25", tolerance="1e-09", threshold="1e-09")
            
            # Topologia tetraedrica definita manualmente
            new_organ.addObject('TetrahedronSetTopologyContainer', 
                        name="topo",
                        position=vertices_struct,
                        tetrahedra=tetrahedra)
            
            new_organ.addObject('MechanicalObject', name="dofs")
            new_organ.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
            new_organ.addObject('DiagonalMass', name="Mass", massDensity="1.0")
            new_organ.addObject('TetrahedralCorotationalFEMForceField', 
                        template="Vec3d", 
                        name="FEM", 
                        method="large", 
                        poissonRatio="0.3", 
                        youngModulus="3000")
            new_organ.addObject('FixedProjectiveConstraint', name="FixedConstraint", indices="3 39 64")

            # Estraiamo i vertici e i triangoli dalla mesh
            surface_vertices = [[v.x, v.y, v.z] for v in organ.surface.vertices]
            surface_triangles = [t.vertex_indices for t in organ.surface.triangles]

            liver_surface = root.addChild(f"{organ.id}_surface")
            liver_surface.addObject('MeshTopology', 
                                name="mesh",
                                position= surface_vertices,
                                triangles= surface_triangles)
            
            # self._add_visual_components(new_organ, surface_vertices, surface_triangles)
            # self._add_collision_components(new_organ)
            
            self.created_organs[organ.id] = new_organ
            logger.info(f"Organ {organ.id} successfully created!")
        
        except Exception as e:
            logger.error(f"Error creating organ: {str(e)}", exc_info=True)

    # def _add_visual_components(self, organ: Organ):
    #     """Aggiunge componenti per la visualizzazione 3D."""
    #     visu = organ.addChild('Visu')
    #     visu.addObject('OglModel', name="VisualModel", src="@../../LiverSurface/mesh")
    #     visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")

    # def _add_collision_components(self, organ: Organ):
    #     surf = organ.addChild('Surf')
    #     surf.addObject('MechanicalObject',
    #                 name="spheres",
    #                 position=organ.surface.vertices)
    #     surf.addObject('SphereCollisionModel',
    #                 name="CollisionModel",
    #                 listRadius=[
    #                     r0, r1, r2  # Sostituisci con i raggi delle sfere
    #                 ])
    #     surf.addObject('BarycentricMapping', name="CollisionMapping", input="@../dofs", output="@spheres")



