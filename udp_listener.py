#!/usr/bin/env python3
"""
UDP Listener for ports 9997 and 9998.
Prints received messages with timestamps.
"""

import socket
import time
import datetime
import threading
import argparse
import signal
import sys


class UDPListener:
    def __init__(self, ports=[9997, 9998], buffer_size=1024):
        """
        Initialize UDP listener for multiple ports
        
        Args:
            ports: List of ports to listen on
            buffer_size: Size of receive buffer
        """
        self.ports = ports
        self.buffer_size = buffer_size
        self.running = True
        self.sockets = []
        
    def setup_socket(self, port):
        """Set up UDP socket for a specific port"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', port))
        sock.settimeout(0.5)  # Use timeout to allow clean shutdown
        print(f"Listening on UDP port {port}...")
        return sock
        
    def listen_on_port(self, port):
        """Listen for messages on a specific port"""
        sock = self.setup_socket(port)
        self.sockets.append(sock)
        
        while self.running:
            try:
                data, addr = sock.recvfrom(self.buffer_size)
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                
                # Try to decode as string, fall back to hex if binary
                try:
                    message = data.decode('utf-8')
                    print(f"[{timestamp}] Port {port} from {addr[0]}:{addr[1]}: {message}")
                except UnicodeDecodeError:
                    # If can't decode as UTF-8, print as hex
                    hex_data = data.hex()
                    print(f"[{timestamp}] Port {port} from {addr[0]}:{addr[1]} (hex): {hex_data}")
                    
            except socket.timeout:
                # This is expected due to the timeout we set
                continue
            except Exception as e:
                if self.running:  # Only print errors if we're still supposed to be running
                    print(f"Error on port {port}: {e}")
                
        # Clean up socket when done
        sock.close()
        
    def start(self):
        """Start listening on all ports"""
        threads = []
        
        # Create a thread for each port
        for port in self.ports:
            thread = threading.Thread(target=self.listen_on_port, args=(port,))
            thread.daemon = True
            threads.append(thread)
            thread.start()
            
        # Wait for all threads to complete (they won't unless interrupted)
        try:
            while any(thread.is_alive() for thread in threads):
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()
            
    def stop(self):
        """Stop all listening threads"""
        print("\nStopping UDP listeners...")
        self.running = False
        time.sleep(1)  # Give threads time to clean up
        for sock in self.sockets:
            try:
                sock.close()
            except:
                pass


def signal_handler(sig, frame):
    """Handle Ctrl+C to clean up"""
    print("\nReceived interrupt signal, shutting down...")
    sys.exit(0)


def main():
    """Main function to run the UDP listener"""
    parser = argparse.ArgumentParser(description='UDP Listener')
    parser.add_argument('--ports', type=int, nargs='+', default=[9997, 9998],
                        help='UDP ports to listen on (default: 9997 9998)')
    parser.add_argument('--buffer', type=int, default=1024,
                        help='Size of receive buffer (default: 1024 bytes)')
    args = parser.parse_args()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    print(f"Starting UDP listener for ports: {args.ports}")
    print("Press Ctrl+C to exit")
    
    listener = UDPListener(ports=args.ports, buffer_size=args.buffer)
    
    try:
        listener.start()
    except KeyboardInterrupt:
        listener.stop()
    finally:
        print("UDP listener stopped")


if __name__ == "__main__":
    main()
