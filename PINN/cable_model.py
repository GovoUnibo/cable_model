import numpy as np
from scipy.integrate import solve_ivp, odeint
from plotter import CablePlotter
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

def triple_cross(u1, u2, u3):
    return np.cross(u1, np.cross(u2, u3))

class CableMass():
    def __init__(self, mass, initial_pos, intial_velocity):
        self.mass = mass

        self.position = initial_pos
        self.velocity = intial_velocity
        
        self.damping_force = np.zeros(3)
        self.linear_elastic_force= np.zeros(3)
        self.bending_elastic_force = np.zeros(3)
        self.twisting_elastic_force = np.zeros(3)
        self.gravity_force = np.array([0.0, 0.0, -mass*9.81])

        self.external_forces = np.zeros(3)
    
    def set_damping_force(self, force):
        self.damping_force = force
    
    def set_linear_elastic_force(self, force):
        self.linear_elastic_force = force
    
    def set_bending_elastic_force(self, force):
        self.bending_elastic_force = force
    
    def set_twisting_elastic_force(self, force):
        self.twisting_elastic_force = force

    def set_external_forces(self, forces):
        self.external_forces = forces
    
    def get_acceleration(self):
        return -(self.gravity_force + self.linear_elastic_force + self.bending_elastic_force + self.twisting_elastic_force + self.external_forces + self.damping_force) / self.mass

    # def compute_velocity(self, dt):
    #     self.velocity += self.get_acceleration() * dt
    #     return self
    
    # def get_velocity(self):
    #     return self.velocity

    # def compute_position(self, dt): 
    #     self.position += self.velocity * dt    
    #     return self.position

    # def get_position(self):
    #     return self.position


class Cable():
    def __init__(self, num_of_masses, cable_length, total_mass, cable_diameter, first_mass_initial_pos):
        
        self.num_of_masses = num_of_masses
        self.l0 = cable_length
        self.total_mass = total_mass
        self.diameter = cable_diameter
        self.num_of_links = self.num_of_masses - 1
        self.discrete_mass = self.total_mass / self.num_of_masses
        self.resolution = float(self.l0) / float(self.num_of_links)

        # Cable properties
        self.A = np.pi * (cable_diameter**2) / 4
        self.I = np.pi * (cable_diameter**4) / 64
        self.I_p = np.pi * (cable_diameter**4) / 32
        self.E = self.G = 0.0
        # Gravity

        # cable discretizzation parameters
        

        # forces and other variables
        self._fixed_axis = np.array([1.0, 0.0, 0.0])
        self.mass_velocities = [np.zeros(3) for _ in range(self.num_of_masses)]
        self.mass_positions = [first_mass_initial_pos + i*self._fixed_axis*self.resolution for i in range(self.num_of_masses)]
        self.relative_velocities = [np.zeros(3) for _ in range(self.num_of_links)]
        self.damping_forces = [np.zeros(3) for _ in range(self.num_of_masses)]
        self.linear_forces = [np.zeros(3) for _ in range(self.num_of_masses)]
        self.bending_forces = [np.zeros(3) for _ in range(self.num_of_masses)]
        self.twisting_forces = [np.zeros(3) for _ in range(self.num_of_masses)]
        self.link_displacement = [0.0 for _ in range(self.num_of_masses)]
        self.beta = [0.0 for _ in range(self.num_of_masses)]
        self.psi = [0.0 for _ in range(self.num_of_masses)]

        self.plotter = CablePlotter()

    def __update_animation(self, frame):
        x = [self.mass_positions[i][0] for i in range(self.num_of_masses)]
        y = [self.mass_positions[i][1] for i in range(self.num_of_masses)]
        z = [self.mass_positions[i][2] for i in range(self.num_of_masses)]
        self.plotter.draw_point_cloud(x, y, z, self.diameter)

    def show(self):
        ani = FuncAnimation(self.plotter.fig, self.__update_animation, interval=100)
        plt.show()

    #coef initialization
    def setYoungModulus(self, young_modulus):
        self.E = young_modulus
        print(f"Young modulus: {self.E}")
        self._setLinearElasticCoef(self.l0 / self.num_of_links)
        self._setBendingElasticCoef(self.l0 / self.num_of_links)
        # In generale, E_linear = E_bending; inoltre, la lunghezza iniziale del segmento di piegatura è l_i = (|l_(i)| + |l_(i+1)|)/2 (lunghezza iniziale di Veronoi)
        self._setMaxTensileStress(self.l0)

    def _setMaxTensileStress(self, length):
        maximum_permissible_deformation = 1E-3  # mm
        strain = maximum_permissible_deformation / self.l0
        tensile_stress = self.E * strain
        self.F_max = tensile_stress * self.A
        print(f"Maximum tensile stress: {self.F_max}")

    def setPoissonRatio(self, poisson_ratio):
        self.G = self.E / (2 * (1 + poisson_ratio))
        print(f"Shear Modulus: {self.G}")
        self._setTwistingElasticCoef(self.l0 / self.num_of_links)  # take Veronoi initial length

    def _setLinearElasticCoef(self, initial_segm_length):
        self.linear_spring = self.E * self.A / initial_segm_length
        print(f"Linear spring: {self.linear_spring}")

    def _setBendingElasticCoef(self, initial_segm_length):
        self.bending_spring = (self.E * self.I) / initial_segm_length
        self.constrain_spring = 3 * self.bending_spring
        print(f"Bending spring: {self.bending_spring}")

    def _setTwistingElasticCoef(self, initial_segm_length):
        self.twisting_spring = (self.G * self.I_p) / initial_segm_length
        print(f"Twisting spring: {self.twisting_spring}")

    def setDamperCoef(self, K_d):
        self.damping_factor = K_d
        # self.damping_factor = 2 * torch.sqrt(self.linear_spring) 
        print(f"Damping factor: {self.damping_factor}")

    # def update_mass_velocity(self, velocity, i):
    #         self.mass_velocities[i] = velocity

    # def set_mass_initial_position(self, position, i):
    #     self.mass_initial_positions[i] = position

    # def update_mass_position(self, position, i):
    #     self.mass_positions[i] = position
    
    # def update_relative_velocities(self):
    #     for i in range(self.num_of_links):
    #         self.relative_velocities[i] = self.mass_velocities[i + 1] - self.mass_velocities[i]

    # def compute_damping_forces(self):
    #     # m2*x_dotdot + d(x2_dot - x1_dot) + d(x2_dot - x3_dot) = 0 ==> mx_dotdot +  d(x2_dot - x1_dot) + d(x2_dot - x3_dot)
    #     # f_1 = d(x1_dot - x2_dot) --> 0 elemento
    #     # f_2_d = - x1_dot + 2 x2_dot - x3_dot
    #     # f_3 = d(x3_dot - x2_dot) --> n-1 elemento dell'array
    #     self.update_relative_velocities()
    #     self.damping_forces[0] = self.damping_factor * self.relative_velocities[0] - self.damping_factor * self.relative_velocities[1]
    #     for i in range(1, self.num_of_masses - 1):
    #         self.damping_forces[i] = -self.damping_factor * self.relative_velocities[i - 1] \
    #                                 + 2 * self.damping_factor * self.relative_velocities[i] \
    #                                 - self.damping_factor * self.relative_velocities[i + 1]
    #     self.damping_forces[-1] = self.damping_factor * self.relative_velocities[-1] - self.damping_factor * self.relative_velocities[self.num_of_masses - 2]


    # def linear_spring_forces(self):
    #     # m1*xdd + k(x1-x2) +k(telaio-> la mass prima non esiste) ==> -k (x2- x1)
    #     # m2*xdd + k(x2 - x1) + k(x2 - x3) = 0 ==> k(x2 - x1) - k (k3 - x2) ==> consisntency with 1 function getLinkLength(i - (i-1))
    #     # f_3 = k(x3 - x2) + k(la massa dopo non esiste) 
    #     # questa formula è presa pari pari dal paper di cable dynamics

    #     self.linear_forces[0] = -self.linear_spring * round((np.linalg.norm(self.get_link_length(1)) - self.l0 / self.num_of_links) / 1e-5) * self.get_unit_versor(1)
    #     for i in range(1, self.num_of_masses):
    #         self.linear_forces[i] = -self.linear_spring * round((np.linalg.norm(self.get_link_length(i)) - self.l0 / self.num_of_links) / 1e-5) * self.get_unit_versor(i) \
    #                                 + self.linear_spring * round((np.linalg.norm(self.get_link_length(i + 1)) - self.l0 / self.num_of_links) / 1e-5) * self.get_unit_versor(i + 1)    
        
    #     self.linear_forces[-1] = + self.linear_spring * round((np.linalg.norm(self.get_link_length(self.num_of_masses - 1)) - self.l0 / self.num_of_links) / 1e-5) * self.get_unit_versor(self.num_of_masses - 1)

    #     # cable_tension_right = ( abs(cable_tension_right) < this->F_max) ? cable_tension_right : this->F_max

    #     # PRINT
    #     # for i in range(self.num_of_masses):
    #     #     print("Linear force {}: {}".format(i, self.linear_forces[i]))



if __name__ == "__main__":

    lenght = 2
    width = 0.006
    mass = 0.03
    damping = 0.3
    poisson_ratio = 0.3
    young_modulus = 126000000
    cable_masses = 15

    cable = Cable(cable_masses, lenght, mass, width, first_mass_initial_pos=np.array([1.0, 0.0, 0.0]))
    cable.setYoungModulus(young_modulus)
    cable.setPoissonRatio(poisson_ratio)
    cable.setDamperCoef(damping)

    
    # cable.show()