import roslibpy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import Mesh, MeshTriangle
import time
import sys
import os
import trimesh


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from config import base_config

class Organ:
    """Classe che rappresenta un organo con ID, posizione e mesh."""
    def __init__(self, organ_id: String, pose: Pose, mesh: Mesh):
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
                'triangles': [{'vertex_indices': [int(triangle.vertex_indices[0]), int(triangle.vertex_indices[1]), int(triangle.vertex_indices[2])]} for triangle in self.mesh.triangles],
                'vertices': [{'x': vertex.x, 'y': vertex.y, 'z': vertex.z} for vertex in self.mesh.vertices]
            }
        }
    
    def __str__(self):
        return f"Organ(id={self.id}, pose={self.pose}, mesh={self.mesh})"


def load_mesh_from_file(file_path):
    """Carica una mesh da un file STL o OBJ e la converte in una Mesh ROS."""
    mesh = trimesh.load(file_path, force='mesh')
    
    ros_mesh = Mesh()
    print("Mesh faces:", len(mesh.faces))
    print("Mesh vertices:", len(mesh.vertices))
    
    for face in mesh.faces:
        triangle = MeshTriangle()
        # Converti gli indici della faccia in una lista di interi
        triangle.vertex_indices = [int(index) for index in face.tolist()]
        print("Triangle indices:", [int(index) for index in face.tolist()])
        ros_mesh.triangles.append(triangle)  # Usa append invece di extend
    for vertex in mesh.vertices:
        point = Point()
        point.x, point.y, point.z = vertex[0], vertex[1], vertex[2]
        ros_mesh.vertices.append(point)
    
    return ros_mesh


def publish_organ(mesh_path):
    """Pubblica un oggetto Organ su un topic ROS usando roslibpy."""
    config = base_config.BaseConfig()
    client = roslibpy.Ros(host=config.ROS_HOST, port=9090)
    client.run()

    print("Connected:", client.is_connected)
    topic = roslibpy.Topic(client, config.ORGAN_TOPIC, config.ORGAN_TOPIC_TYPE)

    # Creazione dati di test
    organ_id = String(data="heart")
    
    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    
    mesh = load_mesh_from_file(mesh_path)
    organ = Organ(organ_id, pose, mesh)
    message = roslibpy.Message(organ.to_dict())

    start_time = time.time()
    while time.time() - start_time < 1000000000000:
        print("Publishinged")
        topic.publish(message)
        time.sleep(1)  # Pubblica ogni secondo
    
    client.terminate()

def publish_organ2():
    config = base_config.BaseConfig()

    """Pubblica un oggetto Organ su un topic ROS usando roslibpy."""
    client = roslibpy.Ros(host=config.ROS_HOST, port=9090)
    client.run()

    topic = roslibpy.Topic(client, config.ORGAN_TOPIC, config.ORGAN_TOPIC_TYPE)

    # Creazione dati di test
    organ_id = String(data="heart")
    
    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    mesh = Mesh()
    triangle = MeshTriangle()
    triangle.vertex_indices = [0, 1, 2]
    mesh.triangles.append(triangle)
    vertex1 = Point()
    vertex1.x = 0.0
    vertex1.y = 0.0
    vertex1.z = 0.0
    vertex2 = Point()
    vertex2.x = 1.0
    vertex2.y = 0.0
    vertex2.z = 0.0
    vertex3 = Point()
    vertex3.x = 0.0
    vertex3.y = 1.0
    vertex3.z = 0.0
    mesh.vertices.extend([vertex1, vertex2, vertex3])

    organ = Organ(organ_id, pose, mesh)

    message = roslibpy.Message(organ.to_dict())

    start_time = time.time()
    while time.time() - start_time < 200000000000:
        print("Publishing:", message)
        topic.publish(message)
        time.sleep(1)  # Pubblica ogni secondo

    client.terminate()
    


if __name__ == "__main__":
    mesh_file_path = r"/home/vincenzo/Downloads/cube.obj"  
    publish_organ(mesh_file_path)
