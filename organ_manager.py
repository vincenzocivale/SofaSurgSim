import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import numpy as np
import roslibpy


class OrganManager(Sofa.Core.Controller):
    def __init__(self, *args, ros_client,**kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.ros_client = ros_client
        self.topic = roslibpy.Topic(self.ros_client, '/organ_data_topic', 'std_msgs/String')  # Sostituisci con il tipo giusto
        self.topic.subscribe(self.createNewOrgan)


    def createNewOrgan(self, msg):
        """
        il msg ha gli attributi:
        - id: identificativo dell'organo che deve essere una stringa
        - mesh: mesh dell'organo mesh_msgs/MeshGeometry 
        - pose: posizione e orientazione dell'organo geometry_msgs/Pose
        """
        root = self.getContext()
        newOrgan = root.addChild(msg.id)

        mesh_data = msg["mesh"]
        pose_data = msg["pose"]

        position_data = pose_data["position"]
        orientation_data = pose_data["orientation"]
        
        vertices = [[v.x, v.y, v.z] for v in mesh_data["vertices"]]
        faces = [[f.a, f.b, f.c] for f in mesh_data["faces"]]
        positions = [position_data["x"], position_data["y"], position_data["z"]]
        
        newOrgan.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness=0.1, rayleighMass=0.1)
        newOrgan.addObject('CGLinearSolver', name="linear_solver", iterations=25, tolerance=1e-09, threshold=1e-09)
        newOrgan.addObject('MechanicalObject', name="dofs")
        newOrgan.addObject('TriangleSetTopologyContainer', name="topo", triangles=faces)
        newOrgan.addObject('TriangleSetTopologyModifier')
        newOrgan.addObject('TriangleSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
        newOrgan.addObject('DiagonalMass', name="Mass", massDensity=1.0)
        newOrgan.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio=0.3, youngModulus=3000, computeGlobalMatrix=False)
        
        visu = newOrgan.addChild('Visu')
        visu.addObject('OglModel', name="VisualModel", position=vertices, color=[1, 0, 0, 1])
        visu.addObject('BarycentricMapping', name="VisualMapping", input="@../dofs", output="@VisualModel")
        
        surf = newOrgan.addChild('Surf')
        surf.addObject('MechanicalObject', name="spheres", position=vertices)
        surf.addObject('SphereCollisionModel', name="CollisionModel")
        surf.addObject('BarycentricMapping', name="CollisionMapping", input="@../dofs", output="@spheres")
        
        print(f"Added organ {msg.id}")

    


