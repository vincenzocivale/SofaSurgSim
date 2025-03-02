#!/usr/bin/env python
import Sofa
import Sofa.Gui
from config.base_config import config as cfg
from sofasurgsim.managers.organ_manager import OrganManager
from sofasurgsim.interfaces.ros_interface import ROSClient
from sofasurgsim.msg.Organ import Organ
    
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



        organ_msg = self.ros_client.use_service(cfg.ORGANS_SERVICE, cfg.ORGANS_SERVICE_TYPE)
        organ = Organ.from_dict(organ_msg)
        organ_node = self.create_sofa_nodes_from_meshes(organ.id, organ.surface, organ.tetrahedral_mesh)

        self.root_node.addObject(OrganManager(root_node=self.root_node, created_organs_node=[organ_node], ros_client=self.ros_client))

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

        visu = organ_node.addChild('Visu')

        visu.addObject('TriangleSetTopologyContainer', name="surface_topo",
                    triangles=" ".join(" ".join(map(str, tri.vertex_indices)) for tri in surface_mesh.triangles))

        visu.addObject('MechanicalObject', name="visual_dofs", 
                    position=" ".join(f"{v.x} {v.y} {v.z}" for v in surface_mesh.vertices))

        visu.addObject('OglModel', name="VisualModel", src="@surface_topo", color="1 0 0 1")
        visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@visual_dofs")    

        return organ_node