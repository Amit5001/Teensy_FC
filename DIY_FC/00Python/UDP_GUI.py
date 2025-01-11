import socket
import struct
import threading
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from collections import deque
from datetime import datetime

class DataPoint:
    def __init__(self, name, value):
        self.name = name
        self.value = value
        self.timestamp = datetime.now()

class Graph:
    def __init__(self, figure, ax, canvas, data_name):
        self.figure = figure
        self.ax = ax
        self.canvas = canvas
        self.data_name = data_name
        self.times = deque(maxlen=100)
        self.values = deque(maxlen=100)

    def update(self, timestamp, value):
        self.times.append(timestamp)
        self.values.append(value)
        self.ax.clear()
        self.ax.plot(self.times, self.values)
        self.ax.set_title(self.data_name)
        self.ax.tick_params(axis='x', rotation=45)
        self.canvas.draw()

class UDPSocketClientGUI:
    def __init__(self, address):
        self.addr = address
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.define_constants()
        
        # Data storage
        self.data_points = {}
        self.graphs = []
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("UDP Data Visualizer")
        self.root.geometry("1200x800")
        
        # Create main paned window container
        self.paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned.pack(fill=tk.BOTH, expand=True)
        
        # Create main containers
        self.left_frame = ttk.Frame(self.paned, padding="5")
        self.right_frame = ttk.Frame(self.paned, padding="5")
        
        # Add frames to paned window
        self.paned.add(self.left_frame, weight=1)
        self.paned.add(self.right_frame, weight=3)
        
        # Create data panel
        self.create_data_panel()
        
        # Create graphs panel
        # Create scrollable container for graphs
        self.canvas = tk.Canvas(self.right_frame)
        self.scrollbar = ttk.Scrollbar(self.right_frame, orient="vertical", command=self.canvas.yview)
        self.graphs_container = ttk.Frame(self.canvas)
        
        # Configure scrolling
        self.graphs_container.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )
        
        self.canvas.create_window((0, 0), window=self.graphs_container, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        
        # Pack scrollable container
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        
        # Start UDP receiver thread
        self.receiver_thread = threading.Thread(target=self.receiver, daemon=True)
        self.receiver_thread.start()

    def define_constants(self):
        self.CONNECTION = 0x01
        self.RAW = 0x02
        self.TYPED = 0x03

        self.SYN = bytes([0x01, 0x01])
        self.SYNACK = bytes([0x01, 0x02])
        self.ACK = bytes([0x01, 0x03])
        self.PING = bytes([0x01, 0x10])
        self.PONG = bytes([0x01, 0x11])
        self.TERMINATE = bytes([0x01, 0xFF])

    def create_data_panel(self):
        # Create a frame for the tree and its scrollbar
        tree_frame = ttk.Frame(self.left_frame)
        tree_frame.pack(fill=tk.BOTH, expand=True)

        # Create data display with scrollbar
        self.tree = ttk.Treeview(tree_frame, columns=('Value',), show='headings', height=20)
        self.tree.heading('Value', text='Value')
        
        # Add scrollbar
        scrollbar = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        
        # Pack tree and scrollbar
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Create button frame
        button_frame = ttk.Frame(self.left_frame)
        button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Create smaller add graph button
        self.add_graph_btn = ttk.Button(button_frame, text="Add Graph", command=self.add_graph_dialog, width=15)
        self.add_graph_btn.pack(side=tk.RIGHT)

    def add_graph_dialog(self):
        dialog = tk.Toplevel(self.root)
        dialog.title("Add Graph")
        dialog.geometry("300x200")
        
        # Create dropdown with available data points
        selected_data = tk.StringVar()
        dropdown = ttk.Combobox(dialog, textvariable=selected_data)
        dropdown['values'] = list(self.data_points.keys())
        dropdown.pack(pady=20)
        
        def confirm():
            if selected_data.get():
                self.add_graph(selected_data.get())
                dialog.destroy()
        
        ttk.Button(dialog, text="Add", command=confirm).pack()

    def add_graph(self, data_name):
        # Create new figure and canvas
        fig = Figure(figsize=(6, 4))
        ax = fig.add_subplot(111)
        
        # Create frame for this graph
        graph_frame = ttk.Frame(self.graphs_container)
        graph_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        canvas = FigureCanvasTkAgg(fig, master=graph_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Create close button for this graph
        def close_graph():
            self.graphs.remove(graph)
            graph_frame.destroy()
        
        close_btn = ttk.Button(graph_frame, text="Close", command=close_graph)
        close_btn.pack()
        
        # Create and store graph object
        graph = Graph(fig, ax, canvas, data_name)
        self.graphs.append(graph)

    def update_data_display(self, name, value):
        # Update tree view
        if name not in self.data_points:
            self.tree.insert('', 'end', iid=name, values=(value,))
        else:
            self.tree.set(name, 'Value', value)
        
        # Store data point
        self.data_points[name] = DataPoint(name, value)
        
        # Update graphs
        timestamp = datetime.now()
        for graph in self.graphs:
            if graph.data_name == name:
                graph.update(timestamp, value)

    def receiver(self):
        while True:
            try:
                message, address = self.sock.recvfrom(1024)
                if message[:2] == self.PING:
                    pong_response = bytearray(self.PONG)
                    pong_response.append(message[2])
                    self.sock.sendto(bytes(pong_response), address)
                else:
                    self.casting(message=message)
            except Exception as e:
                print(f"Error in receiver: {e}")

    def casting(self, message):
        try:
            type = chr(message[1])
            message = message[2:]
            messages_struct_float = struct.unpack("f" * (len(message) // struct.calcsize('f')), message)
            message_struct_int = struct.unpack("i" * (len(message) // struct.calcsize('i')), message)

            # Update GUI based on message type
            match type:
                case 'm':  # Magnetometer data
                    self.root.after(0, self.update_data_display, "mag_x", messages_struct_float[0])
                    self.root.after(0, self.update_data_display, "mag_y", messages_struct_float[1])
                    self.root.after(0, self.update_data_display, "mag_z", messages_struct_float[2])

                case 'q':  # Quaternion data
                    self.root.after(0, self.update_data_display, "quat_x", messages_struct_float[0])
                    self.root.after(0, self.update_data_display, "quat_y", messages_struct_float[1])
                    self.root.after(0, self.update_data_display, "quat_z", messages_struct_float[2])
                    self.root.after(0, self.update_data_display, "quat_w", messages_struct_float[3])

                case 'e':  # Euler angles data
                    self.root.after(0, self.update_data_display, "euler_x", messages_struct_float[0])
                    self.root.after(0, self.update_data_display, "euler_y", messages_struct_float[1])
                    self.root.after(0, self.update_data_display, "euler_z", messages_struct_float[2])

                # Add other cases as needed...

        except Exception as e:
            print(f"Error in casting: {e}")

    def create_connection(self):
        self.sock.sendto(self.SYN, self.addr)
        message, address = self.sock.recvfrom(1024)
        if message != self.SYNACK:
            raise Exception("FAILED SYN-ACK")
        self.sock.sendto(self.ACK, self.addr)
        print("Created a connection")

    def run(self):
        try:
            self.create_connection()
            self.root.mainloop()
        except Exception as e:
            print(f"Error in run: {e}")
        finally:
            self.sock.close()

if __name__ == "__main__":
    client = UDPSocketClientGUI(('192.169.1.199', 8888))
    client.run()