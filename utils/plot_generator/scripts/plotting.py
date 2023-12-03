import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

class Plotter:
    def __init__(self):
        self.left_distance_errors = []  # Store distance errors
        self.left_bearing_errors = []   # Store bearing errors
        self.right_distance_errors = [] # Store distance errors
        self.right_bearing_errors = []  # Store bearing errors

        self.fig, self.ax = plt.subplots()
        self.line_left_distance, = self.ax.plot([], [], label='Left Distance Error')
        self.line_right_distance, = self.ax.plot([], [], label='Right Distance Error')
        self.line_left_bearing, = self.ax.plot([], [], label='Left Bearing Error')
        self.line_right_bearing, = self.ax.plot([], [], label='Right Bearing Error')

        self.ax.set_xlabel('Callback Iteration')
        self.ax.set_ylabel('Error')
        self.ax.legend()

        # Create the animation function
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, frames=None, interval=200, blit=True)
        # plt.show()

    def update_plot(self, frame):
        # Update the plot with new data for each frame
        x_left = np.arange(len(self.left_distance_errors))
        x_right = np.arange(len(self.right_distance_errors))
        print("left distance errors:",self.left_distance_errors)
        print("right distance errors:",self.right_distance_errors)
        print("left bearing errors:",self.left_bearing_errors)
        print("right bearing errors:",self.right_bearing_errors)
        self.line_left_distance.set_data(x_left, self.left_distance_errors)
        self.line_right_distance.set_data(x_right, self.right_distance_errors)
        self.line_left_bearing.set_data(x_left, self.left_bearing_errors)
        self.line_right_bearing.set_data(x_right, self.right_bearing_errors)

        # Update the x-axis limits dynamically
        max_x = max(len(self.left_distance_errors), len(self.right_distance_errors))
        self.ax.set_xlim(0, max_x)

        return self.line_left_distance, self.line_right_distance, self.line_left_bearing, self.line_right_bearing

    def update_data(self, left_distance_errors, right_distance_errors, left_bearing_errors, right_bearing_errors):
        self.left_distance_errors = left_distance_errors
        self.right_distance_errors = right_distance_errors
        self.left_bearing_errors = left_bearing_errors
        self.right_bearing_errors = right_bearing_errors
        plt.show()