#!/usr/bin/env python
import Sofa
import Sofa.Gui
import math
from config.base_config import config as cfg
from sofasurgsim.managers.organ_manager import OrganManager
from sofasurgsim.managers.robot_manager import RobotManager
from sofasurgsim.interfaces.ros_interface import ROSClient
from sofasurgsim.msg.Organ import Organ
from sofasurgsim.msg.Robot import Robot
    
class SOFASceneController:
    def __init__(self, ros_client: ROSClient):
        
        self.root_node = Sofa.Core.Node("root")
        self.root_node.addObject('DefaultAnimationLoop')
        self.root_node.addObject('DefaultVisualManagerLoop')
        
        self.GUI = cfg.GUI
        self.ros_client = ros_client
        self.root_node.dt.value = cfg.SIMULATION_STEP 

    def _create_scene(self):
        """
        Crea gli oggetti nella scena di SOFA utilizzando i dati iniziali ricevuti dai topic ROS.
        """
        self.root_node.addChild('Root', gravity="0 -9.81 0", dt="0.02")

        self.root_node.addObject("RequiredPlugin", pluginName=[    'Sofa.Component.Collision.Detection.Algorithm',
        'Sofa.Component.Collision.Detection.Intersection',
        'Sofa.Component.Collision.Geometry',
        'Sofa.Component.Collision.Response.Contact',
        'Sofa.Component.Constraint.Projective',
        'Sofa.Component.IO.Mesh',
        'Sofa.Component.LinearSolver.Iterative',
        'Sofa.Component.Mapping.Linear',
        'Sofa.Component.Mass',
        'Sofa.Component.ODESolver.Backward',
        'Sofa.Component.SolidMechanics.FEM.Elastic',
        'Sofa.Component.StateContainer',
        'Sofa.Component.Topology.Container.Dynamic',
        'Sofa.Component.Visual',
        'Sofa.GL.Component.Rendering3D'
        ])

        self.root_node.addObject('DefaultAnimationLoop')

        self.root_node.addObject('VisualStyle', displayFlags="showCollisionModels hideVisualModels showForceFields")
        self.root_node.addObject('CollisionPipeline', name="CollisionPipeline")
        self.root_node.addObject('BruteForceBroadPhase', name="BroadPhase")
        self.root_node.addObject('BVHNarrowPhase', name="NarrowPhase")
        self.root_node.addObject('CollisionResponse', name="CollisionResponse", response="PenalityContactForceField")
        self.root_node.addObject('DiscreteIntersection')
        self.root_node.addObject('VisualStyle', displayFlags="showVisualModels showBehaviorModels showForceFields showCollisionModels")



        organ_msg = self.ros_client.use_service(cfg.ORGANS_SERVICE, cfg.ORGANS_SERVICE_TYPE, 'organ')
        organ = Organ.from_dict(organ_msg)
        organ_node = self.create_sofa_nodes_from_meshes(organ.id, organ.surface, organ.tetrahedral_mesh)

        robot_msg = self.ros_client.use_service(cfg.ROBOT_SERVICE, cfg.ROBOT_SERVICE_TYPE, 'robot') 
        
        robot = Robot.from_dict(robot_msg)
        robot_node = self.create_robot_node(robot)
        
        # Enable collision between robot and organ
        robot_node.addObject('CollisionPipeline', name="robot_collision_group")
        organ_node.addObject('CollisionPipeline', name="organ_collision_group")
        
        self.root_node.addObject(OrganManager(root_node=self.root_node, created_organs_node=[organ_node], ros_client=self.ros_client))
        self.root_node.addObject(RobotManager(root_node=self.root_node, robot_node=robot_node, ros_client=self.ros_client))

        return self.root_node

    def run_simulation(self):
        cfg.logger.info("Starting SOFA simulation.")
        self._create_scene()
        Sofa.Simulation.init(self.root_node)
        
        if not self.GUI:
            while True:
                Sofa.Simulation.animate(self.root_node, self.root_node.dt.value)
        else:
            Sofa.Gui.GUIManager.Init("main", "qglviewer")
            Sofa.Gui.GUIManager.createGUI(self.root_node)
            Sofa.Gui.GUIManager.MainLoop(self.root_node)
            Sofa.Gui.GUIManager.closeGUI()

    def create_sofa_nodes_from_meshes(self, id, surface_mesh, tetrahedral_mesh):
        """
        Create SOFA nodes from surface and tetrahedral mesh data.

        :param surface_mesh: The surface mesh (Mesh) to use for the surface.
        :param tetrahedral_mesh: The tetrahedral mesh (TetrahedralMesh) to use for the simulation.
        :return: The created node with all related SOFA objects.
        """
        organ_node = self.root_node.addChild(id)

        # Add solver and linear solver for the simulation
        organ_node.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
        organ_node.addObject('CGLinearSolver', name="linear_solver", iterations="25", tolerance="1e-09", threshold="1e-09")

        # Create tetrahedral topology from data directly (without file)
        organ_node.addObject('TetrahedronSetTopologyContainer', name="topo", 
                tetrahedra=" ".join(" ".join(map(str, tetra.vertices_indices)) for tetra in tetrahedral_mesh.tetrahedra))

        organ_node.addObject('MechanicalObject', name="dofs", 
                position=" ".join(f"{v.x} {v.y} {v.z}" for v in tetrahedral_mesh.vertices))

        organ_node.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
        organ_node.addObject('DiagonalMass', name="Mass", massDensity="1.0")
        organ_node.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio="0.3", youngModulus="3000", computeGlobalMatrix="0")
        organ_node.addObject('FixedConstraint', name="FixedConstraint", indices="3 39 64")

        # organ_node.addObject('MouseInteractor', name="MouseInteractor", template="Vec3d", button=0)

        visu = organ_node.addChild('Visual')

        visu.addObject('TriangleSetTopologyContainer', name="surface_topo",
                    triangles=" ".join(" ".join(map(str, tri.vertex_indices)) for tri in surface_mesh.triangles))

        visu.addObject('MechanicalObject', name="visual_dofs", 
                    position=" ".join(f"{v.x} {v.y} {v.z}" for v in surface_mesh.vertices))

        visu.addObject('OglModel', name="VisualModel", src="@surface_topo", color="1 0 0 1")
        visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@visual_dofs")  

        return organ_node
    
    def create_robot_node(self, robot_msg):
        """Create a SOFA node for a rigid robot from its custom message."""
        robot_node = self.root_node.addChild(robot_msg.name)
        
        # Dizionario per tenere traccia dei nodi link per nome
        link_nodes = {}
        
        # Dizionario per memorizzare la mappa parent-child dei link
        link_parent_map = {}
        
        # Costruisci la mappa parent-child dai joint
        for joint in robot_msg.joints:
            link_parent_map[joint.child_link] = joint.parent_link
        
        # Trova il link radice (quello senza parent o con parent "world")
        root_link_name = None
        for link in robot_msg.links:
            if link.name not in link_parent_map or link_parent_map[link.name] == "world":
                root_link_name = link.name
                break
        
        if not root_link_name:
            print("Impossibile trovare il link radice!")
            return robot_node
        
        # Funzione ricorsiva per costruire l'albero dei link
        def build_link_tree(parent_node, link_name):
            # Crea un nodo per questo link
            link_data = next((link for link in robot_msg.links if link.name == link_name), None)
            if not link_data:
                return None
            
            link_node = parent_node.addChild(link_name)
            link_nodes[link_name] = link_node
            
            # Aggiungi MechanicalObject per questo link
            link_node.addObject('MechanicalObject',
                            template="Rigid3d",
                            position="0 0 0 0 0 0 1",  # Posizione iniziale
                            name="dof")
            
            # Se questo non è il link radice, aggiungi un RigidMapping dal parent
            if parent_node != robot_node:
                link_node.addObject('RigidMapping',
                                input="@../dof",
                                output="@dof")
            
            # Aggiungi nodo collision
            if hasattr(link_data, 'collision_mesh') and link_data.collision_mesh:
                collision_node = link_node.addChild("Collision")
                collision_node.addObject('TriangleSetTopologyContainer',
                                    name="collision_topo",
                                    triangles=" ".join(" ".join(map(str, tri.vertex_indices))
                                                        for tri in link_data.collision_mesh.triangles))
                
                collision_node.addObject('MechanicalObject',
                                    name="collision_dofs",
                                    position=" ".join(f"{v.x} {v.y} {v.z}"
                                                    for v in link_data.collision_mesh.vertices))
                
                collision_node.addObject('TriangleCollisionModel',
                                    name="CollisionModel",
                                    src="@collision_topo")
                
                collision_node.addObject('RigidMapping',
                                    input="@../dof",
                                    output="@collision_dofs")
            
            # Trova tutti i child link di questo link
            child_links = [child for child, parent in link_parent_map.items() if parent == link_name]
            
            # Per ogni child link, trova il suo joint e costruisci l'albero
            for child_link_name in child_links:
                # Trova il joint che collega questo link al child
                joint = next((j for j in robot_msg.joints if j.parent_link == link_name and j.child_link == child_link_name), None)
                
                if joint:
                    # Crea un nodo per il joint
                    joint_node = link_node.addChild(joint.name)
                    
                    # Estrai posizione e orientamento dal joint
                    position = [
                        joint.origin.position.x,
                        joint.origin.position.y,
                        joint.origin.position.z
                    ]
                    
                    # Se l'orientamento è in RPY, convertilo in quaternione
                    if hasattr(joint.origin, 'rpy'):
                        roll, pitch, yaw = joint.origin.rpy
                        orientation = self.rpy_to_quaternion(roll, pitch, yaw)
                    else:
                        # Altrimenti, usa l'orientamento quaternione già presente
                        quat = joint.origin.orientation
                        orientation = [quat.x, quat.y, quat.z, quat.w]
                    
                    print(f"{child_link_name} joint: {position + orientation}")
                    
                    # Aggiungi MechanicalObject per il joint
                    joint_node.addObject('MechanicalObject',
                                    template="Rigid3d",
                                    position=" ".join(map(str, position + orientation)),
                                    name="dof")
                    
                    # Aggiungi RigidMapping dal link parent
                    joint_node.addObject('RigidMapping',
                                    input="@../dof",
                                    output="@dof")
                    
                    # Costruisci l'albero per il child link, a partire da questo joint
                    build_link_tree(joint_node, child_link_name)
        
        # Inizia a costruire l'albero dal link radice
        build_link_tree(robot_node, root_link_name)
        
        return robot_node

    def rpy_to_quaternion(self, roll, pitch, yaw):
        """Convert roll, pitch, yaw (in radians) to quaternion [x, y, z, w]."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]