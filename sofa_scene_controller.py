#!/usr/bin/env python
import Sofa
import SofaRuntime
import Sofa.Gui
import numpy as np
import roslibpy
from organ_manager import OrganManager
    

class SOFASceneController:
    def __init__(self, ros_ip, GUI: bool):
        """
        :param ros_interface: istanza della classe ROSInterface che fornisce i dati ROS
        """
        self.root_node = Sofa.Core.Node("root")
       
        self.ros_client = roslibpy.Ros(host=ros_ip, port=9090)
        self.ros_client.run()

        self.GUI = GUI
        self.organ_manager = OrganManager(ros_client=self.ros_client)

    def _create_scene(self):
        """
        Crea gli oggetti nella scena di SOFA utilizzando i dati iniziali ricevuti dai topic ROS.
        """
        self.root_node.addObject(self.organ_manager)

    def run_simulation(self):
        
        """
        Esegue la simulazione di SOFA.
        """
        self._create_scene()
        Sofa.Simulation.init(self.root_node)
        if not self.GUI:
            while True:
                Sofa.Simulation.animate(self.root_node, self.root_node.dt.value) 
        
        else:
            Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
            Sofa.Gui.GUIManager.createGUI(self.root_node, __file__)
            Sofa.Gui.GUIManager.SetDimension(1080, 1080)
            Sofa.Gui.GUIManager.MainLoop(self.root_node)
            Sofa.Gui.GUIManager.closeGUI()