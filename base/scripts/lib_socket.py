import socket
import time
import threading
import os
import logging
import struct
from datetime import datetime, timedelta


class socket_communication:

    send_trigger = threading.Condition()
    send_data = None
    disable_flag_fn = ''
    connection_ok = False
    connection_lost_datetime = datetime.now()
    target_name = ''
    last_msg_time = datetime.now()
    conn = socket.socket()
    exit_now = False
    trying_to_join_send_thread = True
    trying_to_join_send_thread_since = datetime.now()
    receiver_callback = None

    def __init__(self, name, logger_name, socket_path, server, disable_flag_fn, receiver=None) -> None:
        logging.basicConfig()
        self.logger = logging.getLogger(logger_name)
        self.logger.setLevel(logging.DEBUG)
        self.target_name = name
        self.socket_path = socket_path
        self.disable_flag_fn = disable_flag_fn
        self.server = server
        self.receiver_callback = receiver
        self.connecter_thread = threading.Thread(target=self.__connecter)
        self.connecter_thread.start()

    def __connecter(self):
        self.logger.info('Starting ' + self.target_name + ' communication thread!')
        while not self.exit_now:
            while (os.path.exists(self.disable_flag_fn)):
                time.sleep(1)
            self.logger.info('Waiting for connection ' + self.target_name)
            if self.server:
                if os.path.exists(self.socket_path):
                    os.remove(self.socket_path)
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.settimeout(3)
                try:
                    sock.bind(self.socket_path)
                    sock.listen()
                    self.conn, _ = sock.accept()
                except socket.timeout:
                    self.logger.debug('Time out for connection ' + self.target_name)
                    continue
            else:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                try:
                    to = sock.gettimeout()
                    sock.settimeout(3)
                    sock.connect(self.socket_path)
                    sock.settimeout(to)
                    self.conn = sock
                except Exception as e:  # pylint: disable=broad-except
                    self.logger.warning('Cannot connect ' + self.target_name + ': ' + str(e) + ' --- Retry in 3s')
                    time.sleep(3)
                    self.logger.debug('Retry connect ' + self.target_name)
                    continue
            self.logger.info(self.target_name + ' connected.')
            self.connection_ok = True
            sender_thread = threading.Thread(target=self.__sender)
            sender_thread.start()

            receiver_thread = threading.Thread(target=self.__receiver)
            receiver_thread.start()

            self.logger.debug('Waiting for receiver thread join: ' + self.target_name)
            receiver_thread.join()

            self.logger.debug('Waiting for send thread join: ' + self.target_name)
            self.trying_to_join_send_thread = True
            self.trying_to_join_send_thread_since = datetime.now()
            while sender_thread.is_alive():
                if (datetime.now() - self.trying_to_join_send_thread_since) > timedelta(seconds=5):
                    self.logger.warning('Sender thread is still alive for ' + self.target_name + ' after 5')
                    self.connection_ok = False
                    break
                if (datetime.now() - self.trying_to_join_send_thread_since) > timedelta(seconds=10):
                    self.logger.warning('Sender thread is still alive for ' + self.target_name + ' after 10s')
                    try:
                        self.conn.shutdown(socket.SHUT_WR)
                    except Exception as e:  # pylint: disable=broad-except
                        self.logger.warning('Cannot shutdown ' + self.target_name + ': ' + str(e))

                    try:
                        self.conn.close()
                    except Exception as e:  # pylint: disable=broad-except
                        self.logger.warning('Cannot close ' + self.target_name + ': ' + str(e))
                    break

                with self.send_trigger:
                    self.send_trigger.notify()
                sender_thread.join(1)

            self.trying_to_join_send_thread = False
            time.sleep(1)
            self.logger.debug('Retrying connecting ' + self.target_name)
        self.logger.info('Connector ' + self.target_name + ' out')

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
                        self.logger.debug('No connection_ok in socket sender thread of: ' + self.target_name)
        except Exception as e:  # pylint: disable=broad-except
            self.logger.debug('Sending to ' + self.target_name + '  failed: ' + str(e))
            if self.connection_ok:
                self.connection_lost_datetime = datetime.now()
                self.connection_ok = False
        try:
            self.conn.shutdown(socket.SHUT_WR)
        except Exception as e:  # pylint: disable=broad-except
            self.logger.debug('Shutdown ' + self.target_name + '  failed: ' + str(e))
        try:
            self.conn.close()
        except Exception as e:  # pylint: disable=broad-except
            self.logger.debug('Close ' + self.target_name + '  failed: ' + str(e))
        self.logger.info('Sender ' + self.target_name + ' out')

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
                    self.logger.debug('Connection closed?? Bye bye ' + self.target_name)
                    if self.connection_ok:
                        self.connection_lost_datetime = datetime.now()
                        self.connection_ok = False
        except Exception as e:  # pylint: disable=broad-except
            self.logger.debug('Receiving from ' + self.target_name + ' failed: ' + str(e))
            if self.connection_ok:
                self.connection_lost_datetime = datetime.now()
                self.connection_ok = False
        self.logger.info('Stopping ' + self.target_name + ' receiver')
        with self.send_trigger:
            self.send_trigger.notify()
        self.logger.info('Receiver ' + self.target_name + ' out')
