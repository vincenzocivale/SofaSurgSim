import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import numpy as np
import roslibpy

from sofasurgsim.msg.Organ import Organ
from sofasurgsim.interfaces.ros_interface import ROSClient


class OrganManager(Sofa.Core.Controller):
    def __init__(self, *args, ros_client: ROSClient,**kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.ros_client = ros_client
        self.ros_client.create_subscriber('/organs', 'ros_sofa_bridge_msgs/Organ', self.createNewOrgan)


    def createNewOrgan(self, msg):
        """
        il msg ha gli attributi:
        - id: identificativo dell'organo che deve essere una stringa
        - mesh: mesh dell'organo mesh_msgs/MeshGeometry 
        - pose: posizione e orientazione dell'organo geometry_msgs/Pose
        """
        root = self.getContext()
        newOrgan = root.addChild(msg.id)

        organObject = Organ.from_dict(msg)
        
        newOrgan.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness=0.1, rayleighMass=0.1)
        newOrgan.addObject('CGLinearSolver', name="linear_solver", iterations=25, tolerance=1e-09, threshold=1e-09)
        newOrgan.addObject('MechanicalObject', name="dofs")
        newOrgan.addObject('TriangleSetTopologyContainer', name="topo", triangles=organObject.mesh.triangles)
        newOrgan.addObject('TriangleSetTopologyModifier')
        newOrgan.addObject('TriangleSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
        newOrgan.addObject('DiagonalMass', name="Mass", massDensity=1.0)
        newOrgan.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio=0.3, youngModulus=3000, computeGlobalMatrix=False)
        
        visu = newOrgan.addChild('Visu')
        visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")
        
        surf = newOrgan.addChild('Surf')
        surf.addObject('MechanicalObject', name="spheres", position=organObject.mesh.vertices)
        surf.addObject('SphereCollisionModel', name="CollisionModel")
        surf.addObject('BarycentricMapping', name="CollisionMapping", input="@../dofs", output="@spheres")
        
        # Converti i vertici in formato SOFA
        vertices = np.array(organObject.mesh.vertices).reshape(-1, 3)
        
        # Modifica la creazione della topologia
        newOrgan.addObject('TriangleSetTopologyContainer', 
                        name="topo", 
                        triangles=organObject.mesh.triangles,
                        position=vertices)
        
        

    


