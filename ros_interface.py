import roslibpy
import threading
from queue import Queue
from time import sleep

class ROSClient:
    def __init__(self, host='localhost', port=9090):
        self.client = roslibpy.Ros(host=host, port=port)
        self.publishers = {}
        self.subscribers = {}
        self.publishing_threads = {}
        self.running = False

    def connect(self):
        """Connette al server ROS"""
        self.client.run()
        self.running = True
        print(f'Connesso: {self.client.is_connected}')

    def disconnect(self):
        """Disconnette il client ROS e ferma i thread"""
        self.running = False
        for thread in self.publishing_threads.values():
            thread.join()  # Attendi che i thread terminino
        for pub in self.publishers.values():
            pub.unadvertise()
        self.client.close()
        print("Disconnesso")

    def create_subscriber(self, topic_name, msg_type, callback):
        """
        Crea un subscriber per un topic.
        Args:
            topic_name (str): Nome del topic
            msg_type (str): Tipo del messaggio ROS
            callback (function): Funzione da chiamare quando arriva un messaggio
        """
        def wrapped_callback(message):
            try:
                callback(message)
            except Exception as e:
                print(f"Errore nella callback del topic {topic_name}: {str(e)}")

        subscriber = roslibpy.Topic(
            self.client,
            topic_name,
            msg_type,
            queue_size=10
        )
        subscriber.subscribe(wrapped_callback)
        self.subscribers[topic_name] = subscriber
        print(f"Sottoscritto a {topic_name}")

    def create_publisher(self, topic_name, msg_type, data_generator, rate=10):
        """
        Crea un publisher per un topic con un thread dedicato.
        Args:
            topic_name (str): Nome del topic
            msg_type (str): Tipo del messaggio ROS
            data_generator (function): Funzione che genera i dati da pubblicare
            rate (int): Frequenza di pubblicazione (Hz)
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
                    print(f"Errore nella generazione dati per {topic_name}: {str(e)}")
                sleep(1.0 / rate)

        thread = threading.Thread(target=publishing_loop, daemon=True)
        thread.start()
        self.publishing_threads[topic_name] = thread
        print(f"Publisher attivo su {topic_name} a {rate} Hz")