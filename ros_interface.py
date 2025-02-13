#!/usr/bin/env python
import roslibpy

class ROSClient:
    def __init__(self, host='localhost', port=9090):
        self.client = roslibpy.Ros(host=host, port=port)
        self.publishers = {}
        self.subscribers = {}

    def connect(self):
        self.client.run()
        print(f'Connected: {self.client.is_connected}')

    def create_publisher(self, topic_name, msg_type):
        publisher = roslibpy.Topic(
            self.client,
            topic_name,
            msg_type,
            queue_size=10
        )
        publisher.advertise()
        self.publishers[topic_name] = publisher
        return publisher

    def create_subscriber(self, topic_name, msg_type, callback):
        subscriber = roslibpy.Topic(
            self.client,
            topic_name,
            msg_type,
            queue_size=10
        )
        subscriber.subscribe(callback)
        self.subscribers[topic_name] = subscriber
        return subscriber

    def disconnect(self):
        for pub in self.publishers.values():
            pub.unadvertise()
        self.client.close()
        
