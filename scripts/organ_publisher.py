import sys
import os
sys.path.append(r"C:\Users\cical\Documents\GitHub\Repositories\SofaSurgical\sofasurgsim\msg")

from Organ import Organ, Mesh, TetrahedralMesh, Point, MeshTriangle, Pose, Quaternion
import roslibpy
import meshio


def load_obj_file(file_path):
    """Carica un file OBJ utilizzando meshio per ottenere i vertici e i triangoli."""
    # Usa meshio per leggere il file OBJ
    mesh = meshio.read(file_path)
    
    # Estrai i vertici e i triangoli
    vertices = [Point(x=v[0], y=v[1], z=v[2]) for v in mesh.points]
    triangles = [MeshTriangle(vertex_indices=t.tolist()) for t in mesh.cells_dict["triangle"]]

    return Mesh(vertices, triangles)


def load_vtk_file(file_path):
    """Carica un file VTK utilizzando meshio per ottenere i vertici e i tetraedri."""
    # Usa meshio per leggere il file VTK
    mesh = meshio.read(file_path)
    
    # Estrai i vertici
    vertices = [Point(x=v[0], y=v[1], z=v[2]) for v in mesh.points]
    
    # Estrai i tetraedri (controllando se esistono nel file)
    if "tetra" in mesh.cells_dict:
        tetrahedra = [t.tolist() for t in mesh.cells_dict["tetra"]]
    else:
        raise ValueError("Il file VTK non contiene elementi tetraedrici.")

    return TetrahedralMesh(vertices, tetrahedra)

def create_organ_from_files(obj_path, msh_path, organ_id=1, pose=None):
    """Crea un organo a partire dai file OBJ e MSH."""
    surface_mesh = load_obj_file(obj_path)
    tetrahedral_mesh = load_vtk_file(msh_path)
    
    custom_position = Point(x=1.0, y=2.0, z=3.0)
    custom_orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)

    custom_pose = Pose(position=custom_position, orientation=custom_orientation)

    return Organ(id="liver",  pose=custom_pose, surface=surface_mesh, tetrahedral_mesh=tetrahedral_mesh)





# Esempio di utilizzo
obj_file_path = r'C:\Users\cical\Documents\GitHub\Repositories\SofaSurgical\liver-smooth.obj'
msh_file_path = r'C:\Users\cical\Documents\GitHub\Repositories\SofaSurgical\liver.vtk'

client = roslibpy.Ros(host="localhost", port=9090)
client.run()

print("Connected:", client.is_connected)
topic = roslibpy.Topic(client, "/organs", 'sofa_surgical_msgs/Organ')
organ = create_organ_from_files(obj_file_path, msh_file_path)

message = roslibpy.Message(organ.to_dict())
topic.publish(message)






