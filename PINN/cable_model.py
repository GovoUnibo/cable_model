import numpy as np
from scipy.integrate import solve_ivp, odeint
from plotter import CablePlotter
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from typing import Type

def triple_cross(u1, u2, u3):
    return np.cross(u1, np.cross(u2, u3))

class CableParticle():
    def __init__(self, mass, initial_pos, intial_velocity):
        self.mass = mass
        self.initial_pos = initial_pos 
        self.intial_velocity = intial_velocity 
        
        self.damping_force = np.zeros(3)
        self.linear_elastic_force= np.zeros(3)
        self.bending_elastic_force = np.zeros(3)
        self.twisting_elastic_force = np.zeros(3)
        self.gravity_force = np.array([0.0, 0.0, -mass*9.81])

        self.external_forces = np.zeros(3)

        self.fixed = False

        self.position = initial_pos
        self.velocity = intial_velocity

    def set_fixed(self, fixed):
        self.fixed = fixed

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
        if self.fixed:
            return np.zeros(3)
        
        return (self.gravity_force - self.linear_elastic_force - self.bending_elastic_force - self.twisting_elastic_force - self.external_forces + self.damping_force) / self.mass

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

        # forces and other variables
        self._fixed_axis = np.array([1.0, 0.0, 0.0])
        self.mass_velocities = [np.zeros(3) for _ in range(self.num_of_masses)]
        self.mass_positions = [first_mass_initial_pos + i*self._fixed_axis*self.resolution for i in range(self.num_of_masses)]
        self.relative_velocities = [np.zeros(3) for _ in range(self.num_of_links)]

        self.link_displacement = [0.0 for _ in range(self.num_of_masses)]
        self.beta = [0.0 for _ in range(self.num_of_masses)]
        self.psi = [0.0 for _ in range(self.num_of_masses)]

        self.discretization_particles = [CableParticle(self.discrete_mass, self.mass_positions[i], self.mass_velocities[i]) for i in range(self.num_of_masses)]

        # plot
        self.plotter = CablePlotter()

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
    
    def fix_mass(self, i:Type[int]):
        self.discretization_particles[i].set_fixed(True)

    def compute_damping_forces(self):
        # m2*x_dotdot + d(x2_dot - x1_dot) + d(x2_dot - x3_dot) = 0 ==> mx_dotdot +  d(x2_dot - x1_dot) + d(x2_dot - x3_dot)
        # f_1 = d(x1_dot - x2_dot) --> 0 elemento
        # f_2_d = - x1_dot + 2 x2_dot - x3_dot
        # f_3 = d(x3_dot - x2_dot) --> n-1 elemento dell'array
        self.discretization_particles[0].set_damping_force(
                                self.damping_factor * (self.discretization_particles[0].velocity - self.discretization_particles[1].velocity)
                                ) 
        for i in range(1, self.num_of_masses - 1):
            self.discretization_particles[i].set_damping_force(
                self.damping_factor * (self.discretization_particles[i].velocity - self.discretization_particles[i - 1].velocity) 
                + self.damping_factor * (self.discretization_particles[i].velocity - self.discretization_particles[i + 1].velocity)
                )
        self.discretization_particles[-1].set_damping_force(
                                self.damping_factor * (self.discretization_particles[-1].velocity - self.discretization_particles[-2].velocity)
                                )

    def __get_link_length(self, i):
        return self.discretization_particles[i].position - self.discretization_particles[i - 1].position
    
    def __get_unit_versor(self, i):
        return self.__get_link_length(i) / np.linalg.norm(self.__get_link_length(i))
    
    def linear_spring_forces(self):
        # m1*xdd + k(x1-x2) +k(telaio-> la mass prima non esiste) ==> -k (x2- x1)
        # m2*xdd + k(x2 - x1) + k(x2 - x3) = 0 ==> k(x2 - x1) - k (k3 - x2) ==> consisntency with 1 function getLinkLength(i - (i-1))
        # f_3 = k(x3 - x2) + k(la massa dopo non esiste) 
        # questa formula è presa pari pari dal paper di cable dynamics

        # self.linear_forces[0] = -self.linear_spring * round((np.linalg.norm(self.get_link_length(1)) - self.l0 / self.num_of_links) / 1e-5) * self.get_unit_versor(1)
        self.discretization_particles[0].set_linear_elastic_force(
                                self.linear_spring * round((np.linalg.norm(self.__get_link_length(1)) - self.l0 / self.num_of_links) / 1e-5) * self.__get_unit_versor(1)
                                )
        for i in range(1, self.num_of_masses-1):
            self.discretization_particles[i].set_linear_elastic_force(
                                + self.linear_spring * round((np.linalg.norm(self.__get_link_length(i)) - self.l0 / self.num_of_links) / 1e-5) * self.__get_unit_versor(i) 
                                - self.linear_spring * round((np.linalg.norm(self.__get_link_length(i + 1)) - self.l0 / self.num_of_links) / 1e-5) * self.__get_unit_versor(i + 1)
                                )
        self.discretization_particles[-1].set_linear_elastic_force(
                                + self.linear_spring * round((np.linalg.norm(self.__get_link_length(self.num_of_masses - 1)) - self.l0 / self.num_of_links) / 1e-5) * self.__get_unit_versor(self.num_of_masses - 1)
                                )

    def __update_animation(self, frame):
        x = [self.discretization_particles[i].position[0] for i in range(self.num_of_masses)]
        y = [self.discretization_particles[i].position[1] for i in range(self.num_of_masses)]
        z = [self.discretization_particles[i].position[2] for i in range(self.num_of_masses)]
        self.plotter.draw_point_cloud(x, y, z, self.diameter)

    def show(self):
        ani = FuncAnimation(self.plotter.fig, self.__update_animation, interval=100)
        plt.show()
    
    def _update_internal_forces(self, x, v):
        for i in range(self.num_of_masses):
            self.discretization_particles[i].position = x[i]
            self.discretization_particles[i].velocity = v[i]
        # self.compute_damping_forces()
        # self.linear_spring_forces()
        for i in range(self.num_of_masses):
            print(self.discretization_particles[i].get_acceleration())
        return [self.discretization_particles[i].get_acceleration() for i in range(self.num_of_masses)]
    

    def update_external_forces(self, forces, i:Type[int]):
        self.discretization_particles[i].set_external_forces(forces)
    
    def update(self, x0, t):
        #x and t must be lists
        # if not isinstance(x, list) or not isinstance(t, list):
        #     raise ValueError("x and t must be lists")
        
        half_length = len(x0) // 2
        x = np.array([x0[i:i+3] for i in range(0, half_length, 3)])

        v = np.array([x0[i:i+3] for i in range(half_length, len(x0), 3)])

        accelerations = self._update_internal_forces(x, v)
        
        v = x0[half_length:]

        vx = [v[i] for i in range(0, len(v), 3)]
        vy = [v[i] for i in range(1, len(v), 3)]
        vz = [v[i] for i in range(2, len(v), 3)]

        ax = [accelerations[i][0] for i in range(self.num_of_masses)]
        ay = [accelerations[i][1] for i in range(self.num_of_masses)]
        az = [accelerations[i][2] for i in range(self.num_of_masses)]


        return vx + vy + vz + ax + ay + az


        

    def get_initial_positions(self) -> np.array:
        return np.array([self.discretization_particles[i].initial_pos for i in range(self.num_of_masses)])
    
    def get_initial_velocities(self) -> np.array:
        return np.array([self.discretization_particles[i].intial_velocity for i in range(self.num_of_masses)])

    

        

if __name__ == "__main__":

    lenght = 2
    width = 0.06
    mass = 0.03
    damping = 0.3
    poisson_ratio = 0.3
    young_modulus = 126000000
    cable_masses = 3

    cable = Cable(cable_masses, lenght, mass, width, first_mass_initial_pos=np.array([1.0, 0.0, 0.0]))
    cable.setYoungModulus(young_modulus)
    cable.setPoissonRatio(poisson_ratio)
    cable.setDamperCoef(damping)
    
    cable.fix_mass(0)
    
    x0 = np.concatenate((cable.get_initial_positions().reshape(-1), cable.get_initial_velocities().reshape(-1))).reshape(-1)
    t= np.linspace(0, 1, 1000)

    
    
    # cable.update(x0, t)
    x = odeint(cable.update, x0, t)
    



