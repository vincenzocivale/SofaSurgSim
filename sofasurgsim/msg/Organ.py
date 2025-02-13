from geometry_msgs.msg import Pose
from shape_msgs.msg import Mesh, MeshTriangle
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion

import os
import trimesh

class Organ:
    """Classe che rappresenta un organo con ID, posizione e mesh."""
    def __init__(self, organ_id: String, pose: Pose, mesh: Mesh = None):
        self.id = organ_id
        self.pose = pose
        self.mesh = mesh

    def to_dict(self):
        """Converte l'oggetto Organ in un dizionario"""
        return {
            'id': self.id.data,
            'pose': {
            'position': {
                'x': self.pose.position.x,
                'y': self.pose.position.y,
                'z': self.pose.position.z
            },
            'orientation': {
                'x': self.pose.orientation.x,
                'y': self.pose.orientation.y,
                'z': self.pose.orientation.z,
                'w': self.pose.orientation.w
            }
            },
            'mesh': {
            'triangles': [{'vertex_indices': [triangle.vertex_indices[0], triangle.vertex_indices[1], triangle.vertex_indices[2]]} for triangle in self.mesh.triangles],
            'vertices': [{'x': vertex.x, 'y': vertex.y, 'z': vertex.z} for vertex in self.mesh.vertices]
            }
        }
    

    def set_mesh(self, file_path: str):
        """Crea un oggetto Mesh a partire da un file 3D supportato."""
        if not os.path.isfile(file_path):
            raise FileNotFoundError(f"File {file_path} not found")

        vertices = []
        triangles = []

        # Carica il file di mesh utilizzando trimesh
        mesh = trimesh.load(file_path)

        if not mesh.is_mesh:
            raise ValueError(f"File {file_path} non è un file di mesh valido")

        # Ottieni i vertici della mesh
        for vertex in mesh.vertices:
            vertices.append(Point(x=vertex[0], y=vertex[1], z=vertex[2]))

        # Ottieni i triangoli (indici dei vertici)
        for face in mesh.faces:
            triangles.append(MeshTriangle(vertex_indices=face.tolist()))

        # Crea l'oggetto Mesh
        mesh_obj = Mesh(vertices=vertices, triangles=triangles)
        
        self.mesh = mesh_obj

    @staticmethod
    def from_dict(data):
        """Ricostruisce un oggetto Organ a partire da un dizionario"""
        # Estrazione dei dati dal dizionario
        organ_id = data['id']
        pose_data = data['pose']
        mesh_data = data['mesh']

        # Ricostruzione dell'oggetto Pose
        position_data = pose_data['position']
        orientation_data = pose_data['orientation']
        position = Point(position_data['x'], position_data['y'], position_data['z'])
        orientation = Quaternion(orientation_data['x'], orientation_data['y'], orientation_data['z'], orientation_data['w'])
        pose = Pose(position, orientation)

        # Ricostruzione dell'oggetto Mesh
        triangles = [MeshTriangle(triangle['vertex_indices']) for triangle in mesh_data['triangles']]
        vertices = [Point(vertex['x'], vertex['y'], vertex['z']) for vertex in mesh_data['vertices']]
        mesh = Mesh(vertices, triangles)

        # Creazione dell'oggetto Organ
        return Organ(organ_id, pose, mesh)

        
    def __str__(self):
        return f"Organ(id={self.id}, pose={self.pose}, mesh={self.mesh})"