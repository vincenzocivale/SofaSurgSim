import Sofa
import Sofa.Core
import Sofa.Simulation
import numpy as np

from sofasurgsim.msg.Organ import Organ
from sofasurgsim.interfaces.ros_interface import ROSClient


class OrganManager(Sofa.Core.Controller):
    def __init__(self, *args, ros_client: ROSClient, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        
        # Inizializzazione connessione ROS
        self.ros_client = ros_client
        self.ros_client.connect()
        
        # Sottoscrizione al topic ROS
        self.ros_client.create_subscriber(
            '/organs', 
            'ros_sofa_bridge_msgs/Organ', 
            self.createNewOrgan
        )
        
        # Dizionario per tenere traccia degli organi creati
        self.created_organs = {}

    def createNewOrgan(self, msg):
        """Crea un nuovo organo dalla ROS message"""
        try:
            # Converti il messaggio ROS in un oggetto Organ
            organ = Organ.from_dict(msg)
            
            # Verifica che l'organo non esista già
            if organ.id in self.created_organs:
                print(f"Organo {organ.id} già esistente!")
                return

            root = self.getContext()
            
            # Crea il nodo per il nuovo organo
            new_organ = root.addChild(organ.id)
            
            # Converti i vertici in formato numpy array
            vertices = np.array(organ.mesh.vertices).reshape(-1, 3)
            
            # Aggiungi componenti SOFA
            new_organ.addObject('EulerImplicitSolver', name="odesolver")
            new_organ.addObject('CGLinearSolver', name="linear_solver", iterations=100)
            
            # Topologia e geometria
            new_organ.addObject('MechanicalObject', 
                              name="dofs", 
                              position=vertices.tolist())
            
            new_organ.addObject('TriangleSetTopologyContainer',
                              name="topology", 
                              triangles=organ.mesh.triangles,
                              position=vertices.tolist())
            
            new_organ.addObject('TetrahedralCorotationalFEMForceField',
                              youngModulus=3000,
                              poissonRatio=0.3)
            
            # Componenti di visualizzazione
            self._add_visual_components(new_organ, vertices, organ.mesh.triangles)
            
            # Componenti di collisione
            self._add_collision_components(new_organ)
            
            # Registra l'organo creato
            self.created_organs[organ.id] = new_organ
            print(f"Organo {organ.id} creato con successo!")

        except Exception as e:
            print(f"Errore nella creazione dell'organo: {str(e)}")

    def _add_visual_components(self, node, vertices, triangles):
        """Aggiungi componenti per la visualizzazione 3D"""
        visu = node.addChild("Visual")
        visu.addObject('OglModel', 
                      name="VisualModel",
                      position=vertices.tolist(),
                      triangles=triangles,
                      color=[1.0, 0.5, 0.5, 1.0])
        visu.addObject('BarycentricMapping', 
                      input="@../dofs",
                      output="@VisualModel")

    def _add_collision_components(self, node):
        """Aggiungi componenti per la rilevazione delle collisioni"""
        collision = node.addChild("Collision")
        collision.addObject('MechanicalObject', name="collision_dofs")
        collision.addObject('SphereCollisionModel', 
                           radii=0.5,  # Regola in base alle necessità
                           name="CollisionModel")
        collision.addObject('BarycentricMapping', 
                           input="@../dofs",
                           output="@collision_dofs")
