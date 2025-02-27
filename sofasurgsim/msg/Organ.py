from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import Mesh, MeshTriangle
from std_msgs.msg import UInt32

class TetrahedralMesh:
    """Classe che rappresenta una mesh tetraedrica."""
    def __init__(self, vertices=None, tetrahedra=None):
        self.vertices = vertices if vertices else []
        self.tetrahedra = tetrahedra if tetrahedra else []

    def to_dict(self):
        """Converte l'oggetto TetrahedralMesh in un dizionario."""
        return {
            'vertices': [{'x': v.x, 'y': v.y, 'z': v.z} for v in self.vertices],
            'tetrahedra': self.tetrahedra
        }

    @staticmethod
    def from_dict(data):
        """Crea un oggetto TetrahedralMesh da un dizionario."""
        vertices = [Point(x=v['x'], y=v['y'], z=v['z']) for v in data['vertices']]
        tetrahedra = data['tetrahedra']
        return TetrahedralMesh(vertices, tetrahedra)
    
class Organ:
    """Classe che rappresenta un organo con ID, posizione, mesh superficiale e mesh tetraedrica."""
    def __init__(self, organ_id: UInt32, pose: Pose, surface: Mesh = None, tetrahedral_mesh: TetrahedralMesh = None):
        self.id = organ_id
        self.pose = pose
        self.surface = surface
        self.tetrahedral_mesh = tetrahedral_mesh

    def to_dict(self):
        """Converte l'oggetto Organ in un dizionario."""
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
            'surface': {
                'triangles': [{'vertex_indices': [t.vertex_indices[0], t.vertex_indices[1], t.vertex_indices[2]]} for t in self.surface.triangles],
                'vertices': [{'x': v.x, 'y': v.y, 'z': v.z} for v in self.surface.vertices]
            } if self.surface else None,
            'tetrahedral_mesh': self.tetrahedral_mesh.to_dict() if self.tetrahedral_mesh else None
        }

    @staticmethod
    def from_dict(data):
        """Ricostruisce un oggetto Organ da un dizionario."""
        organ_id = UInt32(data['id'])
        pose_data = data['pose']
        pose = Pose(
            position=Point(**pose_data['position']),
            orientation=Quaternion(**pose_data['orientation'])
        )

        # Ricostruzione della mesh superficiale
        if data['surface']:
            surface = Mesh(
                vertices=[Point(**v) for v in data['surface']['vertices']],
                triangles=[MeshTriangle(vertex_indices=t['vertex_indices']) for t in data['surface']['triangles']]
            )
        else:
            surface = None

        # Ricostruzione della mesh tetraedrica
        tetrahedral_mesh = TetrahedralMesh.from_dict(data['tetrahedral_mesh']) if data['tetrahedral_mesh'] else None

        return Organ(organ_id, pose, surface, tetrahedral_mesh)

    def __str__(self):
        return f"Organ(id={self.id}, pose={self.pose}, surface={self.surface}, tetrahedral_mesh={self.tetrahedral_mesh})"