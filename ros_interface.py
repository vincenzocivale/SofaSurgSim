#!/usr/bin/env python
import roslibpy

class ROSInterface:
    def __init__(self, host='localhost', port=9090):
        # Initialize the connection to ROS
        self.ros_client = roslibpy.Ros(host=host, port=port)
        self.ros_client.run()
        
        # Create topics
        self.mesh_topic = roslibpy.Topic(self.ros_client, '/mesh', 'custom_msgs/MeshMsg')
        self.pose_topic = roslibpy.Topic(self.ros_client, '/object_pose', 'custom_msgs/ObjectPoseMsg')
        self.deformation_topic = roslibpy.Topic(self.ros_client, '/deformation', 'custom_msgs/DeformationMsg')
        
        # Structures to save received data
        self.mesh_data = {}  # { object_id: mesh_data (dict) }
        self.pose_data = {}  # { object_id: pose (dict) }
        
        # Subscribe to topics
        self.mesh_topic.subscribe(self.mesh_callback)
        self.pose_topic.subscribe(self.pose_callback)
    
    def mesh_callback(self, message):
        # Assume the message contains the fields "id" and "mesh"
        obj_id = message['id']
        self.mesh_data[obj_id] = message
        print("Received mesh for object", obj_id)
    
    def pose_callback(self, message):
        # Assume the message contains the fields "id" and "pose"
        obj_id = message['id']
        self.pose_data[obj_id] = message['pose']
        print("Received pose for object", obj_id)
    
    def get_mesh_data(self):
        return self.mesh_data

    def get_pose_data(self):
        return self.pose_data

    def publish_deformation(self, deformation_msg):
        # The message must be a dictionary; roslibpy.Message handles the wrapping
        self.deformation_topic.publish(roslibpy.Message(deformation_msg))
        print("Published deformation for object", deformation_msg.get('object_id'))
