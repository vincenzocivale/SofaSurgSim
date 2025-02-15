import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import roslibpy
import config.base_config as bc



def callback(message):
    """Funzione di callback per la ricezione di messaggi."""
    print("Received:", message)

    

def create_subscriber(client, topic_name, msg_type, callback):
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
                raise Exception(f"Error in callback for topic {topic_name}: {str(e)}")

        subscriber = roslibpy.Topic(
            client,
            topic_name,
            msg_type,
            queue_size=10
        )
        subscriber.subscribe(wrapped_callback)

if __name__ == "__main__":
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()
    config = bc.BaseConfig()
    create_subscriber(client, config.ORGAN_TOPIC, config.ORGAN_TOPIC_TYPE, callback)

    try:
        while True:
            pass
    except KeyboardInterrupt:
        client.terminate()