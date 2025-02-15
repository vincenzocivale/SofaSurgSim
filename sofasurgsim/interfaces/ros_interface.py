import roslibpy
import threading
from queue import Queue
from time import sleep
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ROSClient:
    def __init__(self, host='localhost', port=9090):
        self.client = roslibpy.Ros(host=host, port=port)
        self.publishers = {}
        self.subscribers = {}
        self.publishing_threads = {}
        self.running = False

        # create a subscriber to visualize rosbridge log messages
        self.create_subscriber('/rosout', 'rosgraph_msgs/Log', self._log_callback)

    def connect(self):
        """Connect to the ROS server."""
        self.client.run()
        self.running = True
        logger.info(f'Connected: {self.client.is_connected}')

    def disconnect(self):
        """Disconnect from the ROS client and stop threads."""
        self.running = False
        for thread in self.publishing_threads.values():
            thread.join()  # Wait for threads to finish
        for pub in self.publishers.values():
            pub.unadvertise()
        self.client.close()
        logger.info("Disconnected")

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
                logger.error(f"Error in callback for topic {topic_name}: {str(e)}")

        subscriber = roslibpy.Topic(
            self.client,
            topic_name,
            msg_type,
            queue_size=10
        )
        subscriber.subscribe(wrapped_callback)
        self.subscribers[topic_name] = subscriber
        logger.info(f"Subscribed to {topic_name}")

    def create_publisher(self, topic_name, msg_type, data_generator, rate=10):
        """
        Create a publisher for a topic with a dedicated thread.
        Args:
            topic_name (str): Topic name
            msg_type (str): ROS message type
            data_generator (function): Function to generate data to publish
            rate (int): Publishing rate (Hz)
        """
        publisher = roslibpy.Topic(
            self.client,
            topic_name,
            msg_type,
            queue_size=10
        )
        publisher.advertise()
        self.publishers[topic_name] = publisher

        def publishing_loop():
            while self.running:
                try:
                    data = data_generator()
                    if data is not None:
                        publisher.publish(roslibpy.Message(data))
                except Exception as e:
                    logger.error(f"Error in data generation for {topic_name}: {str(e)}")
                sleep(1.0 / rate)

        thread = threading.Thread(target=publishing_loop, daemon=True)
        thread.start()
        self.publishing_threads[topic_name] = thread
        logger.info(f"Publisher active on {topic_name} at {rate} Hz")
    
    def _log_callback(message):
        log_levels = {
            20: logging.INFO,
            40: logging.ERROR
        }

        # Check if the message is from rosbridge_server, for example by checking the 'name' field
        if 'rosbridge_websocket' in message.get('name', ''):
            # The 'level' field indicates the log level (INFO, WARN, ERROR, etc.)
            log_level = log_levels.get(message['level'], logging.DEBUG)
            logger.log(log_level, message['msg'])
        else:
            # You can also handle other logs
            pass
