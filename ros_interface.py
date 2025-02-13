#!/usr/bin/env python
import roslibpy
from time import time
import threading

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
    
    def create_service_client(self, service_name, service_type):
        """Crea un client per un servizio ROS"""
        service = roslibpy.Service(
            self.client, 
            service_name, 
            service_type
        )
        self.service_clients[service_name] = service
        return service
    
    def call_service(self, service_name, request, callback=None, timeout=None):
        """
        Chiama un servizio ROS
        Args:
            service_name (str): Nome del servizio
            request (dict): Richiesta nel formato corretto
            callback (function): Funzione per gestire la risposta
            timeout (float): Timeout in secondi
        """
        if service_name not in self.service_clients:
            raise ValueError(f"Client per servizio {service_name} non trovato")

        service = self.service_clients[service_name]
        service_request = roslibpy.ServiceRequest(request)

        # Gestione timeout
        timeout = timeout or self.service_timeout
        start_time = time()
        
        # Callback wrapper
        def response_handler(response):
            if callback:
                callback(response)
            event.set()

        # Chiamata asincrona
        event = threading.Event()
        service.call(service_request, response_handler)
        
        # Attesa risposta con timeout
        if not event.wait(timeout):
            raise TimeoutError(f"Timeout servizio {service_name} dopo {timeout} secondi")

    def disconnect(self):
        for pub in self.publishers.values():
            pub.unadvertise()
        self.client.close()
        
