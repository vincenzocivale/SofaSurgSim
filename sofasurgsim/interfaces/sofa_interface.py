#!/usr/bin/env python
import Sofa
import Sofa.Gui
from config import base_config
from sofasurgsim.managers.organ_manager import OrganManager
from sofasurgsim.interfaces.ros_interface import ROSClient
from sofasurgsim.msg.Organ import Organ
    
class SOFASceneController:
    def __init__(self, ros_client: ROSClient, GUI: bool):
        
        self.root_node = Sofa.Core.Node("root")
        self.root_node.addObject('DefaultAnimationLoop')
        self.root_node.addObject('DefaultVisualManagerLoop')
        
        self.cfg = base_config.BaseConfig()
        self.GUI = GUI
        self.ros_client = ros_client
        self.root_node.dt.value = self.cfg.SIMULATION_STEP 

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


        organ_msg = self.ros_client.use_service('/get_organ', 'sofa_surgical_msgs/GetOrgan')
        organ = Organ.from_dict(organ_msg)
        self.create_sofa_nodes_from_data(organ.surface, organ.tetrahedral_mesh)

        return self.root_node

    def run_simulation(self):
        print("Avvio simulazione")
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

    def create_sofa_nodes_from_meshes(self, surface_mesh, tetrahedral_mesh):
        """
        Create SOFA nodes from surface and tetrahedral mesh data.

        :param surface_mesh: The surface mesh (Mesh) to use for the surface.
        :param tetrahedral_mesh: The tetrahedral mesh (TetrahedralMesh) to use for the simulation.
        :return: The created node with all related SOFA objects.
        """
        # Create the 'Organ' node in the SOFA tree
        organ_node = self.root_node.addChild('Organ')

        # Add solver and linear solver for the simulation
        organ_node.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
        organ_node.addObject('CGLinearSolver', name="linear_solver", iterations="25", tolerance="1e-09", threshold="1e-09")

        # Create tetrahedral topology from data directly (without file)
        organ_node.addObject('TetrahedronSetTopologyContainer', name="topo", 
                tetrahedra=" ".join(" ".join(map(str, tetra.vertices_indices)) for tetra in tetrahedral_mesh.tetrahedra))

        organ_node.addObject('MechanicalObject', name="dofs", 
                        position=" ".join([f"{v.x} {v.y} {v.z}" for v in surface_mesh.vertices]))

        organ_node.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
        organ_node.addObject('DiagonalMass', name="Mass", massDensity="1.0")
        organ_node.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio="0.3", youngModulus="3000", computeGlobalMatrix="0")
        organ_node.addObject('FixedConstraint', name="FixedConstraint", indices="3 39 64")

        surface_node = organ_node.addChild("Surface")

        # Add surface topology
        surface_node.addObject('TriangleSetTopologyContainer', name="surface_topo", 
                            triangles=" ".join(" ".join(map(str, tri.vertex_indices)) for tri in surface_mesh.triangles))

        # Add surface mesh vertices
        surface_node.addObject('MechanicalObject', name="surface_dofs", 
                            position=" ".join(f"{v.x} {v.y} {v.z}" for v in surface_mesh.vertices))

        # Add mesh visualization with OglModel
        surface_node.addObject('OglModel', name="VisualModel", color="0.8 0.8 0.8 1.0")
        
        # Map visualization to surface degrees of freedom
        surface_node.addObject('BarycentricMapping', name="VisualMapping", input="@surface_dofs", output="@VisualModel")