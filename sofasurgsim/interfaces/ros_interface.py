import roslibpy
from config.base_config import config as cfg


class ROSClient:
    def __init__(self, host='localhost', port=9090):
        self.client = roslibpy.Ros(host=host, port=port)
        self.publishers = {}
        self.subscribers = {}
        self.publishing_threads = {}
        self.running = False

        # create a subscriber to visualize rosbridge log messages
        #self.create_subscriber('/rosout', 'rosgraph_msgs/Log', self._log_callback)

    def connect(self):
        """Connect to the ROS server."""
        self.client.run()
        self.running = True
        cfg.logger.info(f'Connected: {self.client.is_connected}')

    def disconnect(self):
        """Disconnect from the ROS client and stop threads."""
        self.running = False
        for thread in self.publishing_threads.values():
            thread.join()  # Wait for threads to finish
        for pub in self.publishers.values():
            pub.unadvertise()
        self.client.close()
        cfg.logger.info("Disconnected")

    def create_subscriber(self, topic_name, msg_type, callback):
        """
        Create a subscriber for a topic.
        Args:
            topic_name (str): Topic name
            msg_type (str): ROS message type
            callback (function): Function to call when a message is received
        """
        def wrapped_callback(message):
            try:
                callback(message)
            except Exception as e:
                cfg.logger.error(f"Error in callback for topic {topic_name}: {str(e)}")

        subscriber = roslibpy.Topic(
            self.client,
            topic_name,
            msg_type,
            queue_size=10
        )
        subscriber.subscribe(wrapped_callback)
        self.subscribers[topic_name] = subscriber
        cfg.logger.info(f"Subscribed to {topic_name}")

    def create_publisher(self, topic_name, msg_type, message_data):
        """
        Create a publisher for a topic with a dedicated thread.
        Args:
            topic_name (str): Topic name
            msg_type (str): ROS message type
            message_data (dict): Initial message data
        """
        talker = roslibpy.Topic(self.client, topic_name, msg_type)

        talker.publish(roslibpy.Message(message_data))
        talker.unadvertise()

    def use_service(self, service_name, service_type):
        """
        Use a service.
        Args:
            service_name (str): Service name
            service_type (str): ROS service type
        """
        service = roslibpy.Service(self.client, service_name, service_type)
        request = roslibpy.ServiceRequest()
        result = service.call(request)
        cfg.logger.info(f"Service {type(result)} called")
        
        return result['organ']
