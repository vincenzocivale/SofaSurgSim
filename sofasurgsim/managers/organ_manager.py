import Sofa.Core
import numpy as np
import time

from sofasurgsim.msg.Organ import DeformationUpdate, Displacement
from sofasurgsim.interfaces.ros_interface import ROSClient
from config.base_config import config as cfg

class OrganManager(Sofa.Core.Controller):
    def __init__(self, *args, root_node, created_organs_node,ros_client: ROSClient, **kwargs):
        super().__init__(*args, **kwargs)
        self.root_node = root_node
        self.ros_client = ros_client
        self.sofa_nodes = created_organs_node
        self.reference_positions = self._get_initial_positions()
        self.deformation_threshold = cfg.DEFORMATION_THRESHOLD  # Suggested config value

    def _get_mechanical_object(self, node):
        """Retrieve the mechanical object from a SOFA node"""
        visu_node = node.getChild('Visual')
        mech_obj = visu_node.getObject('visual_dofs')
        if not mech_obj:
            cfg.logger.error(f"Missing 'dofs' MechanicalObject in node {node.name.value}")
        return mech_obj
    
    def _get_initial_positions(self):
        """Capture initial positions of all organs"""
        return {
            node.name.value: self._get_mechanical_object(node).position.array().copy()
            for node in self.sofa_nodes if self._get_mechanical_object(node)
        }

    def _compute_displacements(self, current_positions):
        """Calculate displacements using vectorized operations"""
        displacements = {}
        for name, current in current_positions.items():
            reference = self.reference_positions.get(name)
            if reference is None or len(reference) != len(current):
                cfg.logger.warning(f"Skipping invalid position data for {name}")
                continue
                
            disp_vectors = current - reference
            indices = np.arange(len(disp_vectors)).tolist()
            
            displacements[name] = (indices, disp_vectors)
        
        return displacements

    def _create_deformation_updates(self, displacements):
        """Generate ROS-compatible deformation updates"""
        updates = []
        for name, (indices, vectors) in displacements.items():
            displacements = [
                Displacement(dx=vec[0], dy=vec[1], dz=vec[2]) 
                for vec in vectors
            ]
            updates.append(DeformationUpdate(
                timestamp=time.time(),
                node_name=name,
                vertex_ids=indices,
                displacements=displacements
            ))
        return updates

    def _publish_updates(self, updates):
        """Batch publish deformation updates"""
        for update in updates:
            self.ros_client.create_publisher(
                f"/deformation_updates_{update.node_name}",
                'sofa_surgical_msgs/DeformationUpdate',
                update.to_dict()
            )

    def onAnimateEndEvent(self, event):
        """Main processing at end of simulation step"""
        current_positions = {
            node.name.value: self._get_mechanical_object(node).position.array().copy()
            for node in self.sofa_nodes if self._get_mechanical_object(node)
        }
        
        displacements = self._compute_displacements(current_positions)
        if displacements:
            updates = self._create_deformation_updates(displacements)
            self._publish_updates(updates)
            self.reference_positions = current_positions  # Update reference

        else:
            cfg.logger.info("No significant deformations detected")
