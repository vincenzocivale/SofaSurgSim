import logging
import Sofa
import Sofa.Core
import Sofa.Simulation
import numpy as np

from sofasurgsim.msg.Organ import Organ
from sofasurgsim.interfaces.ros_interface import ROSClient
from config import base_config

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class OrganManager(Sofa.Core.Controller):
    def __init__(self, *args, ros_client: ROSClient, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.ros_client = ros_client
        self.bs = base_config.BaseConfig()
        self.created_organs = {}
        
        self.ros_client.create_subscriber(
            self.bs.ORGAN_TOPIC, 
            self.bs.ORGAN_TOPIC_TYPE, 
            self.create_new_organ
        )

            
    def create_new_organ(self, msg):
        """Creates a new organ from a ROS message."""
        try:
            organ = Organ.from_dict(msg)
            
            if organ.id in self.created_organs:
                logger.warning(f"Organ {organ.id} already exists!")
                return

            root = self.getContext()
            new_organ = root.addChild(organ.id)

            vertices = np.array([[v.x, v.y, v.z] for v in organ.mesh.vertices], dtype=np.float32)
            triangles = np.array([t.vertex_indices for t in organ.mesh.triangles], dtype=np.uint32)
            
            new_organ.addObject('EulerImplicitSolver', name="odesolver")
            new_organ.addObject('CGLinearSolver', name="linear_solver", iterations=100)
            
            new_organ.addObject('MechanicalObject', name="dofs", position=vertices.tolist())
            new_organ.addObject('TriangleSetTopologyContainer', name="topology", triangles=triangles.tolist(), position=vertices.tolist())
            new_organ.addObject('UniformMass', totalMass=1.0)  # Aggiunto
            new_organ.addObject('TriangularFEMForceField', youngModulus=3000, poissonRatio=0.3)
            
            self._add_visual_components(new_organ, vertices, triangles)
            self._add_collision_components(new_organ)
            
            self.created_organs[organ.id] = new_organ
            logger.info(f"Organ {organ.id} successfully created!")
        
        except Exception as e:
            logger.error(f"Error creating organ: {str(e)}", exc_info=True)

    def _add_visual_components(self, node, vertices, triangles):
        """Adds components for 3D visualization."""
        visu = node.addChild("Visual")
        visu.addObject('OglModel', name="VisualModel", position=vertices.tolist(), triangles=triangles, color=[1.0, 0.5, 0.5, 1.0])
        visu.addObject('BarycentricMapping', input="@../dofs", output="@VisualModel")

    def _add_collision_components(self, node):
        """Adds components for collision detection."""
        collision = node.addChild("Collision")
        collision.addObject('MechanicalObject', name="collision_dofs")
        collision.addObject('PointCollisionModel', name="CollisionModel")
        collision.addObject('BarycentricMapping', input="@../dofs", output="@collision_dofs")
