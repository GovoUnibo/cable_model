import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

class CablePlotter(object):

    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Point Cloud Plot')

        self.scatter = None

    def draw_point_cloud(self, x, y, z, point_diameter=0.1):
        if self.scatter is not None:
            self.scatter.remove()

        self.scatter = self.ax.scatter(x, y, z, c='r', marker='o', s=point_diameter)



if __name__ == '__main__':
    def update(frame):
        # Generiamo nuove coordinate casuali per la point cloud
        x = np.random.uniform(-10, 10, n_points)
        y = np.random.uniform(-10, 10, n_points)
        z = np.random.uniform(-10, 10, n_points)

        # Disegniamo la point cloud aggiornata
        plotter.draw_point_cloud(x, y, z)

    # Creiamo una point cloud casuale iniziale
    n_points = 5
    x = np.random.uniform(-10, 10, n_points)
    y = np.random.uniform(-10, 10, n_points)
    z = np.random.uniform(-10, 10, n_points)

    # Creiamo l'oggetto plot3dClass
    plotter = CablePlotter()

    # Creiamo l'animazione
    ani = FuncAnimation(plotter.fig, update, interval=100)

    plt.show()
