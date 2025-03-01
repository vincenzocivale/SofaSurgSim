#!/usr/bin/env python
import Sofa
import Sofa.Gui
from config import base_config
from sofasurgsim.managers.organ_manager import OrganManager
from sofasurgsim.interfaces.ros_interface import ROSClient
from sofasurgsim.msg.Organ import Organ, VTKAttributes, Point, Quaternion, Pose
    
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


        new_organ = self.root_node.addChild("Prova")

        mesh_path = r"C:\Users\cical\Documents\GitHub\Repositories\SofaSurgical\mesh_files\1.vtk"
        vtk_attributes = VTKAttributes()
        vtk_attributes.load_from_vtk(mesh_path)

        position = Point(x=1.0, y=2.0, z=3.0)

        # Creazione di un'istanza di Quaternion per l'orientamento
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Creazione di un'istanza di Pose con posizione e orientamento specificati
        pose = Pose(position=position, orientation=orientation)
        organ = Organ(
            id="Prova",
            pose=pose,
            surface=None,
            tetrahedral_mesh=vtk_attributes
        )
        

        new_organ.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness="0.1", rayleighMass="0.1")
        new_organ.addObject('CGLinearSolver', name="linear_solver", iterations="25", tolerance="1e-09", threshold="1e-09")
        new_organ.addObject('MeshVTKLoader', name="meshLoader", filename=mesh_path)  # Modifica qui per caricare il file .vtk
        new_organ.addObject('TetrahedronSetTopologyContainer', name="topo", src="@meshLoader")
        new_organ.addObject('MechanicalObject', name="dofs", src="@meshLoader")
        new_organ.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
        new_organ.addObject('DiagonalMass', name="Mass", massDensity="1.0")
        new_organ.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio="0.3", youngModulus="3000", computeGlobalMatrix="0")
        new_organ.addObject('FixedProjectiveConstraint', name="FixedConstraint", indices="3 39 64")

        self.root_node.addObject('MeshOBJLoader', name="LiverSurface", filename="mesh/liver-smooth.obj")

        visu = new_organ.addChild('Visu')
        visu.addObject('OglModel', name="VisualModel", src="@../../LiverSurface")
        visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")

        surf = new_organ.addChild('Surf')
        surf.addObject('SphereLoader', name="sphereLoader", filename="mesh/liver.sph")
        surf.addObject('MechanicalObject', name="spheres", position="@sphereLoader.position")
        surf.addObject('SphereCollisionModel', name="CollisionModel", listRadius="@sphereLoader.listRadius")
        surf.addObject('BarycentricMapping', name="CollisionMapping", input="@../dofs", output="@spheres")

        
        self.root_node.addObject(OrganManager(
            root_node=self.root_node,
            ros_client=self.ros_client,
            name="OrganManager"
        ))

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