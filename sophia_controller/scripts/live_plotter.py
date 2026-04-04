#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.widgets import RadioButtons
import matplotlib.animation as animation
import threading
import numpy as np

class LivePlotterNode(Node):
    def __init__(self):
        super().__init__('live_plotter')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/leg_target_positions',
            self.listener_callback,
            10
        )
        
        self.buffer_size = 400
        self.t_data = [] 
        self.x_data = [[] for _ in range(6)]
        self.y_data = [[] for _ in range(6)]
        self.z_data = [[] for _ in range(6)]
        self.counter = 0

    def listener_callback(self, msg):
        self.counter += 1
        pos_array = np.array(msg.data).reshape(6, 3)
        self.t_data.append(self.counter)
        for i in range(6):
            self.x_data[i].append(pos_array[i][0])
            self.y_data[i].append(pos_array[i][1])
            self.z_data[i].append(pos_array[i][2])
            
        if len(self.t_data) > self.buffer_size:
            self.t_data.pop(0)
            for i in range(6):
                self.x_data[i].pop(0)
                self.y_data[i].pop(0)
                self.z_data[i].pop(0)

def main(args=None):
    rclpy.init(args=args)
    node = LivePlotterNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    plt.subplots_adjust(left=0.25, hspace=0.3)
    
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    leg_names = ['RF', 'RM', 'RB', 'LF', 'LM', 'LB']
    
    x_lines = []
    y_lines = []
    z_lines = []
    
    for i in range(6):
        vis = (i == 0)
        lx, = ax1.plot([], [], color=colors[i], label=f'Leg {leg_names[i]} X', visible=vis)
        ly, = ax2.plot([], [], color=colors[i], label=f'Leg {leg_names[i]} Y', visible=vis)
        lz, = ax3.plot([], [], color=colors[i], label=f'Leg {leg_names[i]} Z', visible=vis)
        x_lines.append(lx)
        y_lines.append(ly)
        z_lines.append(lz)

    ax1.set_ylabel('X Position')
    ax1.set_title('Live Leg Trajectories (X, Y, Z)')
    ax1.grid(True)
    ax2.set_ylabel('Y Position')
    ax2.grid(True)
    ax3.set_ylabel('Z Position')
    ax3.set_xlabel('Time Step')
    ax3.grid(True)
    
    rax = plt.axes([0.05, 0.4, 0.15, 0.3])
    radio = RadioButtons(rax, leg_names)

    def change_leg(label):
        idx = leg_names.index(label)
        for i in range(6):
            vis = (i == idx)
            x_lines[i].set_visible(vis)
            y_lines[i].set_visible(vis)
            z_lines[i].set_visible(vis)
        fig.canvas.draw_idle()

    radio.on_clicked(change_leg)

    def update(frame):
        if not node.t_data:
            return x_lines + y_lines + z_lines
            
        for i in range(6):
            x_lines[i].set_data(node.t_data, node.x_data[i])
            y_lines[i].set_data(node.t_data, node.y_data[i])
            z_lines[i].set_data(node.t_data, node.z_data[i])
            
        ax3.set_xlim(min(node.t_data), max(node.t_data) + 10)
        
        # Rescale axes based on visible data
        for ax, data_list in [(ax1, node.x_data), (ax2, node.y_data), (ax3, node.z_data)]:
            visible_idx = -1
            for i in range(6):
                if x_lines[i].get_visible():
                    visible_idx = i
                    break
            
            if visible_idx != -1 and data_list[visible_idx]:
                d = data_list[visible_idx]
                d_min, d_max = min(d), max(d)
                if d_max - d_min < 0.001:
                    ax.set_ylim(d_min - 0.05, d_max + 0.05)
                else:
                    padding = (d_max - d_min) * 0.1
                    ax.set_ylim(d_min - padding, d_max + padding)
            
        return x_lines + y_lines + z_lines

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)


    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
