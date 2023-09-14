import numpy as np
import matplotlib.pyplot as plt
import datetime
from matplotlib.animation import FuncAnimation

"""
Simple app which plots the function h(t) = 3*pi*exp(-5*sin(2*pi*t)) in real time
Pressing space pauses the program and allows the user to click on the plot to select points
Successive clicks will draw a line between the points, where the distance between the points is displayed or the area of the polygon is calculated
This is inspired by the type of tools of analysis used in medical imaging software where the plot would be replaced by an ultrasound of a heart and the tooling would
allow the doctor to measure cross-sectional the heart valve. It would also be used to measure the pulse of a patient by measuring the distance between the peaks of an EKG

I don't have that much time before the end of the week so I figured I would create an app that I know has actual use in real life, and that I may create in less than an hour.
I created the GUI for the BMS, and I devised a remote programming protocol that allows us to program the car over CAN so I have already showcased some of my capabilities @ KTHFS
// Oscar Eriksson
"""

class FunctionVisualizer:
    def __init__(self, num_points=1000):
        self.num_points = num_points
        self.paused = False  # Initialize animation state
        self.clicks = []  # List to store click coordinates

        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        self.line_h, = self.ax.plot([], [], label=r'$h(t) = 3\pi e^{-\lambda(t)}$', color='red')
        self.line_clicks, = self.ax.plot([], [], linestyle='-', markersize=5, color='blue')  # Line for clicks
        self.ax.set_xlabel('Time (s)')
        self.ax.set_title('Real-time Plot of h(t)')
        self.ax.legend(loc='upper left')
        self.ax.grid(True)

        # Initialize x-axis limits and ticks
        self.t_current = datetime.datetime.now().timestamp()
        self.t_window = 2 * np.pi  # Set the initial time window
        self.ax.set_xlim(self.t_current - self.t_window, self.t_current)
        self.ax.set_xticks(np.linspace(self.t_current - self.t_window, self.t_current, 5))

        # Connect the key press event to the toggle_pause function
        self.fig.canvas.mpl_connect('key_press_event', self.toggle_pause)
        # Connect the mouse button click event to the record_click function
        self.fig.canvas.mpl_connect('button_press_event', self.record_click)

    def toggle_pause(self, event):
        if event.key == ' ':
            self.paused = not self.paused
            self.clicks = []  # Clear the list of click coordinates

    def shoelace(self):
        clicks = self.clicks + [self.clicks[0]]
        area = 0
        for (x1, y1), (x2, y2) in zip(clicks, clicks[1:]):
            area += 0.5 * (y1 + y2) * (x2 - x1)
        
        return area

    def record_click(self, event):
        if self.paused and event.button == 1:  # Left mouse button click
            if event.xdata is not None and event.ydata is not None:
                self.clicks.append((event.xdata, event.ydata))
                self.update_clicks_line()

    def update_clicks_line(self):
        if self.clicks:
            clicks = self.clicks + [self.clicks[0]]
            x_clicks, y_clicks = zip(*clicks)
            self.line_clicks.set_data(x_clicks, y_clicks)
        else:
            self.line_clicks.set_data([], [])

    def update_plot(self, frame):
        if self.paused:
            L = len(self.clicks)
            if L == 0:
                self.ax.set_title('Real-time Plot of h(t) (click to select an initial point)')
            elif L == 1:
                self.ax.set_title(f'{self.clicks[-1]} (click again to draw a line)')
            elif L == 2:
                dist = np.sqrt((self.clicks[0][0] - self.clicks[1][0])**2 + (self.clicks[0][1] - self.clicks[1][1])**2)
                self.ax.set_title(f'distance = {dist:.3f} (click again to measure area of polygon)')
            else:
                area = self.shoelace()
                self.ax.set_title('Area = {:.3f}'.format(area))

            return self.line_h, self.line_clicks
        else:
            self.ax.set_title('Real-time Plot of h(t) (press space to pause)')
            t_current = datetime.datetime.now().timestamp()
            t_values = np.linspace(t_current - self.t_window, t_current, self.num_points)
            h_values = 3 * np.pi * np.exp(-5 * np.sin(2 * np.pi * 1 * t_values))

            self.line_h.set_data(t_values, h_values)

            # Update x-axis limits and ticks for real-time display
            self.ax.set_xlim(t_values[0], t_values[-1])
            xticks = np.linspace(t_values[0], t_values[-1], 5)
            xtick_labels = [datetime.datetime.fromtimestamp(ts).strftime('%M:%S') for ts in xticks]
            self.ax.set_xticks(xticks)
            self.ax.set_xticklabels(xtick_labels)
            self.ax.set_ylim(h_values.min(), h_values.max())
            self.update_clicks_line()  # Update clicks line
            return self.line_h, self.line_clicks

    def animate(self):
        anim = FuncAnimation(self.fig, self.update_plot, frames=self.num_points, repeat=False, interval=50)
        plt.show()

# Create an instance of the FunctionVisualizer class
visualizer = FunctionVisualizer()

# Start the real-time animation
visualizer.animate()