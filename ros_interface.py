#!/usr/bin/env python
import roslibpy

class ROSInterface:
    def __init__(self, host='localhost', port=9090):
        # Initialize the connection to ROS
        self.ros_client = roslibpy.Ros(host=host, port=port)
        self.ros_client.run()
        
    def subscribe_to_topic(self, topic_name, message_type, callback):
        """
        Sottoscrive un nuovo topic ROS dinamicamente.
        
        :param topic_name: Nome del topic ROS da sottoscrivere (es. '/mesh_topic')
        :param message_type: Tipo di messaggio ROS (es. 'mesh_msgs/MeshGeometry')
        :param callback: Funzione di callback da eseguire quando arriva un messaggio
        """
        if topic_name in self.subscribers:
            print(f"⚠️ Warning: Il topic {topic_name} è già sottoscritto!")
            return
        
        listener = roslibpy.Topic(self.ros, topic_name, message_type)
        listener.subscribe(callback)
        
