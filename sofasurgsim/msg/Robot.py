from typing import List
from .Organ import Point,  Pose,  Mesh, TetrahedralMesh

class RobotLink:
    """Class representing a robot link with visual and collision meshes."""
    def __init__(self, name: str, visual_mesh: Mesh, collision_mesh: Mesh = None):
        self.name = name
        self.visual_mesh = visual_mesh
        self.collision_mesh = collision_mesh

    def to_dict(self):
        """Converts the RobotLink object to a dictionary."""
        return {
            'name': self.name,
            'visual_mesh': self.visual_mesh.to_dict(),
            'collision_mesh': self.collision_mesh.to_dict() if self.collision_mesh else None
        }

    @staticmethod
    def from_dict(data):
        """Creates a RobotLink object from a dictionary."""
        visual_mesh = Mesh.from_dict(data['visual_mesh'])
        collision_mesh = Mesh.from_dict(data['collision_mesh']) if data.get('collision_mesh') else None
        return RobotLink(
            name=data['name'],
            visual_mesh=visual_mesh,
            collision_mesh=collision_mesh
        )
    
class RobotJoint:
    """Class representing a robot joint connecting two links."""
    def __init__(self, name: str, joint_type: str, parent_link: str, child_link: str, origin: Pose, axis: Point):
        self.name = name
        self.joint_type = joint_type
        self.parent_link = parent_link
        self.child_link = child_link
        self.origin = origin
        self.axis = axis

    def to_dict(self):
        """Converts the RobotJoint object to a dictionary."""
        return {
            'name': self.name,
            'type': self.joint_type,
            'parent_link': self.parent_link,
            'child_link': self.child_link,
            'origin': self.origin.to_dict(),
            'axis': self.axis.to_dict()
        }

    @staticmethod
    def from_dict(data):
        """Creates a RobotJoint object from a dictionary."""
        origin = Pose.from_dict(data['origin'])
        axis = Point.from_dict(data['axis'])
        return RobotJoint(
            name=data['name'],
            joint_type=data['type'],
            parent_link=data['parent_link'],
            child_link=data['child_link'],
            origin=origin,
            axis=axis
        )
    
class Robot:
    """Class representing a robot with links and joints."""
    def __init__(self, name: str, links: List[RobotLink], joints: List[RobotJoint]):
        self.name = name
        self.links = links
        self.joints = joints

    def to_dict(self):
        """Converts the Robot object to a dictionary."""
        return {
            'name': self.name,
            'links': [link.to_dict() for link in self.links],
            'joints': [joint.to_dict() for joint in self.joints]
        }

    @staticmethod
    def from_dict(data):
        """Creates a Robot object from a dictionary."""
        links = [RobotLink.from_dict(link) for link in data['links']]
        joints = [RobotJoint.from_dict(joint) for joint in data['joints']]
        return Robot(name=data['name'], links=links, joints=joints)