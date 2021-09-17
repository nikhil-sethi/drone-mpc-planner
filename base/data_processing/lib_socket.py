import socket
import time
import threading
import os
import logging
from datetime import datetime


class socket_communication:

    send_trigger = threading.Condition()
    send_data = None
    connection_ok = False
    target_name = ''
    last_msg_time = datetime.now()
    conn = socket.socket()
    exit_now = False

    def __init__(self, name, socket_path, server) -> None:
        self.logger = logging.getLogger('baseboard')
        self.target_name = name
        self.socket_path = socket_path
        self.server = server
        self.connecter_thread = threading.Thread(target=self.__connecter)
        self.connecter_thread.start()

    def __connecter(self):
        self.logger.info('Starting ' + self.target_name + ' communication thread!')
        while not self.exit_now:
            self.logger.info('Waiting for connection ' + self.target_name)
            if self.server:
                if os.path.exists(self.socket_path):
                    os.remove(self.socket_path)
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.bind(self.socket_path)
                sock.listen()
                self.conn, _ = sock.accept()
            else:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                try:
                    sock.connect(self.socket_path)
                    self.conn = sock
                except Exception as e:  # pylint: disable=broad-except
                    self.logger.warning('Cannot connect ' + self.target_name + ': ' + str(e))
                    time.sleep(10)
                    continue
            self.logger.info(self.target_name + ' connected.')
            self.connection_ok = True
            sender_thread = threading.Thread(target=self.__sender)
            sender_thread.start()

            receiver_thread = threading.Thread(target=self.__receiver)
            receiver_thread.start()

            receiver_thread.join()
            sender_thread.join()

    def send(self, data):
        with self.send_trigger:
            self.send_data = data
            self.send_trigger.notify()

    def close(self):
        self.exit_now = True
        self.connection_ok = False
        self.conn.shutdown(socket.SHUT_WR)
        with self.send_trigger:
            self.send_trigger.notify()
        self.connecter_thread.join()

    def __sender(self):
        try:
            while self.connection_ok and not self.exit_now:
                with self.send_trigger:
                    self.send_trigger.wait()
                    if self.connection_ok and self.send_data:
                        self.conn.send(self.send_data)
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error('Sending to ' + self.target_name + '  failed: ' + str(e))
            self.connection_ok = False
        self.logger.info('Sender ' + self.target_name + ' out')
        self.conn.close()

    def __receiver(self):
        try:
            while self.connection_ok and not self.exit_now:
                data = self.conn.recv(1024)
                if data:
                    # print('Received', repr(data))
                    self.last_msg_time = datetime.now()
                else:
                    self.logger.info('Connection closed?? Bye bye ' + self.target_name)
                    self.connection_ok = False
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error('Receiving from ' + self.target_name + ' failed: ' + str(e))
            self.connection_ok = False
        self.logger.info('Stopping ' + self.target_name + ' receiver')
        with self.send_trigger:
            self.send_trigger.notify()
        self.logger.info(self.target_name + ' receiver out')
