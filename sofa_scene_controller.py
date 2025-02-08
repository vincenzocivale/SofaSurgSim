#!/usr/bin/env python
import Sofa
import SofaRuntime
import numpy as np
from ros_interface import ROSInterface
from organ_manager import OrganManager

class SOFASceneController:
    def __init__(self, ros_interface: ROSInterface):
        """
        :param ros_interface: istanza della classe ROSInterface che fornisce i dati ROS
        """
        self.root_node = Sofa.Core.Node("root")
        self.ros_interface = ros_interface
        self.organ_manager = OrganManager()

    def _create_scene(self):
        """
        Crea gli oggetti nella scena di SOFA utilizzando i dati iniziali ricevuti dai topic ROS.
        """
        self.root_node.addObject(self.organ_manager)
        self.ros_interface.subscribe_to_topic("/mesh_topic", "topic_custom_da_creare_ancora", self.organ_manager.createNewOrgan)

    def run_simulation(self):
        """
        Esegue la simulazione di SOFA.
        """
        self._create_scene()
        Sofa.Simulation.initRoot(self.root_node)
        while True:
            Sofa.Simulation.animate(self.root_node, self.root_node.dt.value)
           