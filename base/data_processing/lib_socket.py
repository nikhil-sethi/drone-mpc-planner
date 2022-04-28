import socket
import time
import threading
import os
import logging
import struct
from datetime import datetime


class socket_communication:

    send_trigger = threading.Condition()
    send_data = None
    connection_ok = False
    connection_lost_datetime = datetime.now()
    target_name = ''
    last_msg_time = datetime.now()
    conn = socket.socket()
    exit_now = False
    receiver_callback = None

    def __init__(self, name, socket_path, server, receiver=None) -> None:
        logging.basicConfig()
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)
        self.target_name = name
        self.socket_path = socket_path
        self.server = server
        self.receiver_callback = receiver
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
                    self.logger.warning('Cannot connect ' + self.target_name + ': ' + str(e) + ' --- Retry in 10s')
                    time.sleep(10)
                    self.logger.warning('Retry connect ' + self.target_name)
                    continue
            self.logger.info(self.target_name + ' connected.')
            self.connection_ok = True
            sender_thread = threading.Thread(target=self.__sender)
            sender_thread.start()

            receiver_thread = threading.Thread(target=self.__receiver)
            receiver_thread.start()

            receiver_thread.join()
            sender_thread.join()
            time.sleep(1)
            self.logger.info('Retrying connecting ' + self.target_name)
        self.logger.info('Connector thread exit for ' + self.target_name)

    def send(self, data):
        with self.send_trigger:
            self.send_data = data
            self.send_trigger.notify()

    def close(self):
        self.logger.info('Closing socket ' + self.target_name)
        self.exit_now = True
        if self.connection_ok:
            res = self.conn.shutdown(socket.SHUT_WR)
            self.logger.info('Socket shutdown result: ' + str(res))
            self.conn.close()
            self.logger.info('Socket closed:' + self.target_name)
            self.connection_lost_datetime = datetime.now()
            self.connection_ok = False
        with self.send_trigger:
            self.send_trigger.notify()
        self.connecter_thread.join()
        self.logger.info('Closed socket ' + self.target_name)

    def __sender(self):
        self.logger.info('Sender ' + self.target_name + ' starting')
        try:
            while self.connection_ok and not self.exit_now:
                with self.send_trigger:
                    self.send_trigger.wait()
                    if self.connection_ok and self.send_data:
                        self.conn.send(self.send_data)
                    elif not self.connection_ok:
                        self.logger.info('No connection_ok in socket sender thread of: ' + self.target_name)
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error('Sending to ' + self.target_name + '  failed: ' + str(e))
            if self.connection_ok:
                self.connection_lost_datetime = datetime.now()
                self.connection_ok = False
        self.logger.info('Sender ' + self.target_name + ' out')
        self.conn.close()

    def __receiver(self):
        self.logger.info('Receiver ' + self.target_name + ' starting')
        try:
            while self.connection_ok and not self.exit_now:
                data = self.conn.recv(1024)
                if data:
                    if self.receiver_callback:
                        try:
                            self.receiver_callback(data)
                        except struct.error as e:
                            self.logger.error("Serialization config problem. Has the package changed in the executor? " + str(e))
                            raise e  # handle log here, but pass the error for local handling
                    # print('Received', repr(data))
                    self.last_msg_time = datetime.now()
                else:
                    self.logger.info('Connection closed?? Bye bye ' + self.target_name)
                    if self.connection_ok:
                        self.connection_lost_datetime = datetime.now()
                        self.connection_ok = False
        except Exception as e:  # pylint: disable=broad-except
            self.logger.error('Receiving from ' + self.target_name + ' failed: ' + str(e))
            if self.connection_ok:
                self.connection_lost_datetime = datetime.now()
                self.connection_ok = False
        self.logger.info('Stopping ' + self.target_name + ' receiver')
        with self.send_trigger:
            self.send_trigger.notify()
        self.logger.info(self.target_name + ' receiver out')
