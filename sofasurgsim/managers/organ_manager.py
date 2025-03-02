import logging
import Sofa
import Sofa.Core
import Sofa.Simulation
import time

from sofasurgsim.msg.Organ import Organ, Displacement, DeformationUpdate
from sofasurgsim.interfaces.ros_interface import ROSClient
from config import base_config

# Configuriamo il logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class OrganManager(Sofa.Core.Controller):
    def __init__(self, *args, root_node, created_organs_node,ros_client: ROSClient, **kwargs):
        super().__init__(*args, **kwargs)
        self.root_node = root_node
        self.ros_client = ros_client
        self.bs = base_config.BaseConfig()
        self.sofa_nodes = created_organs_node
        self.record_surface_positions = self.extract_surface_positions() # Estraiamo le posizioni delle mesh superficiale attuali

    
    def extract_surface_positions(self):
        """
        Extract the surface mesh positions from a list of SOFA nodes.

        :param sofa_nodes: List of SOFA nodes.
        :return: Dictionary with node names as keys and lists of surface positions as values.
        """
        surface_positions = {}

        for node in self.sofa_nodes:
            visu_node = node.getChild('Visu')  # Verifica se esiste un nodo figlio 'Visu'
            
            if visu_node:
                visual_model = visu_node.getObject('VisualModel')  # Verifica se il nodo 'Visu' ha un 'VisualModel'

                if visual_model:
                    positions = visual_model.findData('position').value  # Estrai le posizioni dei vertici

                    node_name = str(node.name.value)  
                    surface_positions[node_name] = positions
                else:
                    print(f"Node {str(node.name)} has no 'VisualModel' in 'Visu' node")
            else:
                print(f"Node {str(node.name)} has no 'Visu' child node")

        return surface_positions

    
    def compute_deformation_updates(self, prev_positions: dict, current_positions: dict):
        """
        Compute the deformation updates between two sets of surface positions.

        :param prev_positions: Dictionary of surface positions at the previous time step.
                            Keys are node names, values are lists of vertex positions.
        :param current_positions: Dictionary of surface positions at the current time step.
                                Keys are node names, values are lists of vertex positions.
        :return: List of DeformationUpdate objects, one for each deformed mesh.
        """
        deformation_updates = []

        for node_name, current_verts in current_positions.items():
            if node_name in prev_positions:
                prev_verts = prev_positions[node_name]

                # Check if the number of vertices matches
                if len(prev_verts) != len(current_verts):
                    print(f"Warning: Number of vertices for node {node_name} has changed. Skipping this node.")
                    continue

                # Compute displacements
                vertex_ids = []
                displacements = []
                for idx, (prev_pos, curr_pos) in enumerate(zip(prev_verts, current_verts)):
                    dx = curr_pos[0] - prev_pos[0]
                    dy = curr_pos[1] - prev_pos[1]
                    dz = curr_pos[2] - prev_pos[2]

                    # Only include vertices with non-zero displacement
                    if dx != 0 or dy != 0 or dz != 0:
                        vertex_ids.append(idx)
                        displacements.append(Displacement(dx, dy, dz))

                # Create a DeformationUpdate object if there are any displacements
                if vertex_ids:
                    timestamp = time.time()
                    deformation_update = DeformationUpdate(timestamp=timestamp, node_name=node_name, vertex_ids=vertex_ids, displacements=displacements)
                    deformation_updates.append(deformation_update)
                else:
                    print(f"No displacements for node {node_name}")

        return deformation_updates
    
    def process_deformation_updates(self, deformation_updates):
        """
        Process the deformation updates by publishing them to a ROS topic.

        :param deformation_updates: List of DeformationUpdate objects.
        """
        for update in deformation_updates:
            # Convert the DeformationUpdate object to a dictionary
            update_dict = update.to_dict()

            topic_name = f"/deformation_updates_{update.node_name}"
            self.ros_client.create_publisher(topic_name, 'sofa_surgical_msgs/DeformationUpdate', update_dict)
    
    def onAnimateBeginEvent(self, event):
        """
        Called at the beginning of each simulation step.
        """
        # Extract current surface positions
        current_positions = self.extract_surface_positions()

        # Compute deformation updates if previous positions are available
        if self.record_surface_positions:
            deformation_updates = self.compute_deformation_updates(self.record_surface_positions, current_positions)
            self.process_deformation_updates(deformation_updates)
            self.record_surface_positions = current_positions

        

        
        
    