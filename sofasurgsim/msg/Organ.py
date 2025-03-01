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

class TetrahedralMesh:
    """Class representing a tetrahedral mesh."""
    def __init__(self, vertices: List[Point] = None, tetrahedra: List[List[int]] = None):
        self.vertices = vertices if vertices else []
        self.tetrahedra = tetrahedra if tetrahedra else []

    def to_dict(self):
        """Converts the TetrahedralMesh object to a dictionary."""
        return {
            'vertices': [v.to_dict() for v in self.vertices],
            'tetrahedra': self.tetrahedra
        }

    @staticmethod
    def from_dict(data):
        """Creates a TetrahedralMesh object from a dictionary."""
        vertices = [Point.from_dict(v) for v in data['vertices']]
        flat_tetra = data['tetrahedra']
        # Raggruppa la lista flat in sotto-liste di 4 elementi ciascuna
        tetrahedra = [flat_tetra[i:i+4] for i in range(0, len(flat_tetra), 4)]
        return TetrahedralMesh(vertices=vertices, tetrahedra=tetrahedra)
    
class VTKAttributes:
    """Class representing the VTK attributes, including version, title, format, points, and cells."""
    def __init__(self, version: str = "vtk DataFile Version 2.0", title: str = "VTK Mesh Data", file_format: str = "ASCII",
                 dataset_type: str = "UNSTRUCTURED_GRID", points: List[Point] = None, cell_indices: List[int] = None,
                 cell_offsets: List[int] = None, cell_types: List[int] = None):
        self.version = version
        self.title = title
        self.file_format = file_format
        self.dataset_type = dataset_type
        self.points = points if points else []
        self.cell_indices = cell_indices if cell_indices else []
        self.cell_offsets = cell_offsets if cell_offsets else []
        self.cell_types = cell_types if cell_types else []

    def to_dict(self):
        """Converts the VTKAttributes object to a dictionary."""
        return {
            'version': self.version,
            'title': self.title,
            'file_format': self.file_format,
            'dataset_type': self.dataset_type,
            'points': [point.to_dict() for point in self.points],
            'cell_indices': self.cell_indices,
            'cell_offsets': self.cell_offsets,
            'cell_types': self.cell_types
        }

    @staticmethod
    def from_dict(data):
        """Creates a VTKAttributes object from a dictionary."""
        version = data['version']
        title = data['title']
        file_format = data['file_format']
        dataset_type = data['dataset_type']
        points = [Point.from_dict(p) for p in data['points']]
        cell_indices = data['cell_indices']
        cell_offsets = data['cell_offsets']
        cell_types = data['cell_types']
        return VTKAttributes(version=version, title=title, file_format=file_format, dataset_type=dataset_type,
                             points=points, cell_indices=cell_indices, cell_offsets=cell_offsets, cell_types=cell_types)
    
    def save_to_vtk_file(self, file_path: str):
        """Saves the VTKAttributes data to a .vtk file in ASCII or BINARY format."""
        # Create the vtk unstructured grid
        grid = vtk.vtkUnstructuredGrid()
        
        # Add points to the grid
        vtk_points = vtk.vtkPoints()
        for point in self.points:
            vtk_points.InsertNextPoint(point.x, point.y, point.z)
        grid.SetPoints(vtk_points)
        
        # Add cells to the grid
        cell_array = vtk.vtkCellArray()
        num_cells = len(self.cell_indices) // 4  # Assuming tetrahedral cells (4 points per cell)
        for i in range(num_cells):
            cell_array.InsertNextCell(4, self.cell_indices[i*4:(i+1)*4])
        grid.SetCells(vtk.VTK_TETRA, cell_array)
        
        # Write the grid to the file
        writer = vtk.vtkUnstructuredGridWriter()
        writer.SetFileName(file_path)
        
        if self.file_format == "ASCII":
            writer.SetFileTypeToASCII()
        else:
            writer.SetFileTypeToBinary()

        writer.SetInputData(grid)
        writer.Write()
    
    def load_from_vtk(self, filename: str):
        """Method to load VTK data from a file using the vtk library."""
        # Read the VTK file
        reader = vtk.vtkPolyDataReader()
        reader.SetFileName(filename)
        reader.Update()
        
        # Get the VTK polydata
        polydata = reader.GetOutput()
        
        # Extract points (coordinates)
        vtk_points = polydata.GetPoints()
        self.points = []
        for i in range(vtk_points.GetNumberOfPoints()):
            x, y, z = vtk_points.GetPoint(i)
            self.points.append(Point(x, y, z))
        
        # Extract cells (indices and offsets)
        cells = polydata.GetCells()
        self.cell_indices = []
        self.cell_offsets = []
        self.cell_types = []
        
        cell_offset = 0
        for i in range(cells.GetNumberOfCells()):
            cell = cells.GetCell(i)
            cell_type = cell.GetCellType()
            num_points = cell.GetNumberOfPoints()
            self.cell_offsets.append(cell_offset)
            self.cell_types.append(cell_type)
            for j in range(num_points):
                self.cell_indices.append(cell.GetPointId(j))
            cell_offset += num_points

class Organ:
    """Class representing an organ with ID, pose, surface mesh, and tetrahedral mesh."""
    def __init__(self, id: str, pose: Pose, surface: Mesh = None, tetrahedral_mesh: VTKAttributes = None):
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
        tetrahedral_mesh = VTKAttributes.from_dict(data['tetrahedral_mesh']) if data['tetrahedral_mesh'] else None
        return Organ(id=id, pose=pose, surface=surface, tetrahedral_mesh=tetrahedral_mesh)

    def __str__(self):
        return f"Organ(id={self.id}, pose={self.pose}, surface={self.surface}, tetrahedral_mesh={self.tetrahedral_mesh})"