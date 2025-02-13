import roslibpy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import Mesh, MeshTriangle
import time


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
    
def publish_organ():
    """Pubblica un oggetto Organ su un topic ROS usando roslibpy."""
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    topic = roslibpy.Topic(client, '/organ_topic', 'ros_sofa_bridge_msgs/Organ')

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
    while time.time() - start_time < 20:
        print("Publishing:", message)
        topic.publish(message)
        time.sleep(1)  # Pubblica ogni secondo

    client.terminate()


if __name__ == "__main__":
    publish_organ()
