from typing import List, Optional
import vtk

class Point:
    """Class representing a 3D point."""
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def to_dict(self):
        """Converts the Point object to a dictionary."""
        return {'x': self.x, 'y': self.y, 'z': self.z}

    @staticmethod
    def from_dict(data):
        """Creates a Point object from a dictionary."""
        return Point(x=data['x'], y=data['y'], z=data['z'])

class Quaternion:
    """Class representing a quaternion."""
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def to_dict(self):
        """Converts the Quaternion object to a dictionary."""
        return {'x': self.x, 'y': self.y, 'z': self.z, 'w': self.w}

    @staticmethod
    def from_dict(data):
        """Creates a Quaternion object from a dictionary."""
        return Quaternion(x=data['x'], y=data['y'], z=data['z'], w=data['w'])

class Pose:
    """Class representing a pose with position and orientation."""
    def __init__(self, position: Point = None, orientation: Quaternion = None):
        self.position = position if position else Point()
        self.orientation = orientation if orientation else Quaternion()

    def to_dict(self):
        """Converts the Pose object to a dictionary."""
        return {
            'position': self.position.to_dict(),
            'orientation': self.orientation.to_dict()
        }

    @staticmethod
    def from_dict(data):
        """Creates a Pose object from a dictionary."""
        position = Point.from_dict(data['position'])
        orientation = Quaternion.from_dict(data['orientation'])
        return Pose(position=position, orientation=orientation)

class MeshTriangle:
    """Class representing a mesh triangle."""
    def __init__(self, vertex_indices: List[int] = None):
        self.vertex_indices = vertex_indices if vertex_indices else []

    def to_dict(self):
        """Converts the MeshTriangle object to a dictionary."""
        return {'vertex_indices': self.vertex_indices}

    @staticmethod
    def from_dict(data):
        """Creates a MeshTriangle object from a dictionary."""
        return MeshTriangle(vertex_indices=data['vertex_indices'])

class Mesh:
    """Class representing a mesh."""
    def __init__(self, vertices: List[Point] = None, triangles: List[MeshTriangle] = None):
        self.vertices = vertices if vertices else []
        self.triangles = triangles if triangles else []

    def to_dict(self):
        """Converts the Mesh object to a dictionary."""
        return {
            'vertices': [v.to_dict() for v in self.vertices],
            'triangles': [t.to_dict() for t in self.triangles]
        }

    @staticmethod
    def from_dict(data):
        """Creates a Mesh object from a dictionary."""
        vertices = [Point.from_dict(v) for v in data['vertices']]
        triangles = [MeshTriangle.from_dict(t) for t in data['triangles']]
        return Mesh(vertices=vertices, triangles=triangles)
    
class Tetrahedron:
    """Class representing a tetrahedron with 4 vertex indices."""
    def __init__(self, vertices_indices: List[int] = None):
        # Assicurarsi che 'vertices_indices' sia una lista di 4 interi
        if len(vertices_indices) != 4:
            raise ValueError("vertices_indices must contain exactly 4 integers.")
        
        self.vertices_indices = vertices_indices

    def to_dict(self):
        """Converts the Tetrahedron object to a dictionary."""
        return {'vertices_indices': self.vertices_indices}

    @staticmethod
    def from_dict(data):
        """Creates a Tetrahedron object from a dictionary."""
        vertices_indices = data['vertices_indices']
        if len(vertices_indices) != 4:
            raise ValueError("vertices_indices must contain exactly 4 integers.")
        return Tetrahedron(vertices_indices=vertices_indices)

class TetrahedralMesh:
    """Class representing a tetrahedral mesh."""
    def __init__(self, vertices: List[Point] = None, tetrahedra: List[Tetrahedron] = None):
        self.vertices = vertices if vertices else []
        self.tetrahedra = tetrahedra if tetrahedra else []

    def to_dict(self):
        """Converts the TetrahedralMesh object to a dictionary."""
        return {
            'vertices': [v.to_dict() for v in self.vertices],  # Conviene che Point32 abbia un metodo to_dict
            'tetrahedra': [t.to_dict() for t in self.tetrahedra]  # Conviene che Tetrahedron abbia un metodo to_dict
        }

    @staticmethod
    def from_dict(data):
        """Creates a TetrahedralMesh object from a dictionary."""
        # Converti i vertici da Point32
        vertices = [Point(x=v['x'], y=v['y'], z=v['z']) for v in data['vertices']]
        
        # Converti i tetraedri da Tetrahedron
        tetrahedra = [Tetrahedron.from_dict(t) for t in data['tetrahedrons']]
        
        return TetrahedralMesh(vertices=vertices, tetrahedra=tetrahedra)
    

class Organ:
    """Class representing an organ with ID, pose, surface mesh, and tetrahedral mesh."""
    def __init__(self, id: str, pose: Pose, surface: Mesh = None, tetrahedral_mesh: TetrahedralMesh = None):
        self.id = id
        self.pose = pose
        self.surface = surface
        self.tetrahedral_mesh = tetrahedral_mesh

    def to_dict(self):
        """Converts the Organ object to a dictionary."""
        return {
            'id': self.id,
            'pose': self.pose.to_dict(),
            'surface': self.surface.to_dict() if self.surface else None,
            'tetrahedral_mesh': self.tetrahedral_mesh.to_dict() if self.tetrahedral_mesh else None
        }

    @staticmethod
    def from_dict(data):
        """Creates an Organ object from a dictionary."""
        id = data['id']
        pose = Pose.from_dict(data['pose'])
        surface = Mesh.from_dict(data['surface']) if data['surface'] else None
        tetrahedral_mesh = TetrahedralMesh.from_dict(data['tetrahedral_mesh']) if data['tetrahedral_mesh'] else None
        return Organ(id=id, pose=pose, surface=surface, tetrahedral_mesh=tetrahedral_mesh)

    def __str__(self):
        return f"Organ(id={self.id}, pose={self.pose}, surface={self.surface}, tetrahedral_mesh={self.tetrahedral_mesh})"