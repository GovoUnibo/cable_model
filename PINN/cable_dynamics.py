import torch

def tripleCross(u1, u2, u3):
    return u1.cross(u2.cross(u3))


class MassSpringDamping():
    def __init__(self, num_of_masses, cable_length, total_mass, cable_diameter):
        
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # Constants
        pi = torch.pi

        # Cable properties
        self.A = pi * (cable_diameter**2) / 4 
        self.I = pi * (cable_diameter**4) / 64
        self.I_p = pi * (cable_diameter**4) / 32
        self.E = self.G = 0.0
        # Gravity
        self.gravity_acceleration = torch.tensor([0.0, 0.0, -9.81], device=self.device)
        self.gravity_forces = torch.tensor([0.0, 0.0, 0.0], device=self.device)

        # cable discretizzation parameters
        self.num_of_masses = num_of_masses
        self.l0 = cable_length
        self.total_mass = total_mass
        self.diameter = cable_diameter
        self.num_of_links = self.num_of_masses - 1
        self.discrete_mass = self.total_mass / self.num_of_masses

        # forces and other variables
        self.mass_velocities = [torch.zeros(3, device=self.device) for _ in range(self.num_of_masses)]
        self.mass_positions = [torch.zeros(3, device=self.device) for _ in range(self.num_of_masses)]
        self.mass_initial_positions = [torch.zeros(3, device=self.device) for _ in range(self.num_of_masses)]
        self.relative_velocities = [torch.zeros(3, device=self.device) for _ in range(self.num_of_links)]
        self.damping_forces = [torch.zeros(3, device=self.device) for _ in range(self.num_of_masses)]
        self.linear_forces = [torch.zeros(3, device=self.device) for _ in range(self.num_of_masses)]
        self.bending_forces = [torch.zeros(3, device=self.device) for _ in range(self.num_of_masses)]
        self.twisting_forces = [torch.zeros(3, device=self.device) for _ in range(self.num_of_masses)]
        self.link_displacement = [0.0 for _ in range(self.num_of_masses)]
        self.beta = [0.0 for _ in range(self.num_of_masses)]
        self.psi = [0.0 for _ in range(self.num_of_masses)]
        self._fixed_axis = torch.tensor([1.0, 0.0, 0.0], device=self.device)
    #coef initialization
    def setYoungModulus(self, young_modulus):
        self.E = young_modulus
        print(f"Young modulus: {self.E}")
        self._setLinearElasticCoef(self.l0 / self.num_of_links)
        self._setBendingElasticCoef(self.l0 / self.num_of_links)
        # In generale, E_linear = E_bending; inoltre, la lunghezza iniziale del segmento di piegatura è l_i = (|l_(i)| + |l_(i+1)|)/2 (lunghezza iniziale di Veronoi)
        self._setMaxTensileStress(self.l0)

    def setPoissonRatio(self, poisson_ratio):
        self.G = self.E / (2 * (1 + poisson_ratio))
        print(f"Shear Modulus: {self.G}")
        self._setTwistingElasticCoef(self.l0 / self.num_of_links)  # take Veronoi initial length

    def _setMaxTensileStress(self, length):
        maximum_permissible_deformation = 1E-3  # mm
        strain = maximum_permissible_deformation / self.l0
        tensile_stress = self.E * strain
        self.F_max = tensile_stress * self.A
        print(f"Maximum tensile stress: {self.F_max}")

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

    def updateMassVelocity(self, velocity, i):
        self.mass_velocities[i] = velocity

    def setMassInitialPosition(self, position, i):
        self.mass_initial_positions[i] = position

    def updateMassPosition(self, position, i):
        self.mass_positions[i] = position
    
    def updateRelativeVelocities(self):
        for i in range(self.num_of_links):
            self.relative_velocities[i] = self.mass_velocities[i + 1] - self.mass_velocities[i]
            # print(f"Link {i} relative velocity: {self.relative_velocities[i]}")

    def computeDampingForces(self):
        # f_1 = d(x2_dot - x1_dot) --> 0 elemento
        # f_2 = d(x1_dot- x2_dot) - d(x3_dot - x2_dot) = - d(x2_dot - x1_dot) - d(x3_dot - x2_dot)
        # f_3 = d(x2_dot - x3_dot) --> n-1 elemento dell'array
        self.updateRelativeVelocities()
        self.damping_forces[0] = self.damping_factor * self.relative_velocities[0]
        for i in range(1, self.num_of_masses - 1):
            self.damping_forces[i] = -self.damping_factor * self.relative_velocities[i - 1] + self.damping_factor * self.relative_velocities[i]
        self.damping_forces[-1] = -self.damping_factor * self.relative_velocities[-1]

        # PRINT
        # for i in range(self.num_of_masses):
        #     print(f"Damping force {i}: {self.damping_forces[i]}")

    def linearSpringForces(self):
        self.linear_forces[0] = self.linear_spring * torch.round((self._getLinkLenghtNorm(1) - self.l0 / self.num_of_links) / 1e-5) * self.getUnitVersor(1)
        for i in range(1, self.num_of_masses):
            self.linear_forces[i] = self.linear_spring * torch.round((self._getLinkLenghtNorm(i + 1) - self.l0 / self.num_of_links) / 1e-5) * self.getUnitVersor(i + 1) \
                                    - self.linear_spring * torch.round((self._getLinkLenghtNorm(i) - self.l0 / self.num_of_links) / 1e-5) * self.getUnitVersor(i)
        self.linear_forces[-1] = -self.linear_spring * torch.round((self._getLinkLenghtNorm(self.num_of_masses - 1) - self.l0 / self.num_of_links) / 1e-5) * self.getUnitVersor(self.num_of_masses - 1)
        
    def _getLinkLenghtNorm(self, i):
        return torch.norm(self.getLinkLength(i))

    def getLinkLength(self, i):
        return self.mass_positions[i] - self.mass_positions[i - 1]

    def getUnitVersor(self, i):
        vector = self.getLinkLength(i)
        return vector / torch.norm(vector)

    # @staticmethod
    def tripleCross(self, i, j, k):
        u1 = self.getUnitVersor(i)
        u2 = self.getUnitVersor(j)
        u3 = self.getUnitVersor(k)
        return u1.cross(u2.cross(u3))


    def updateBeta(self):
        self.beta[0] = 0.0
        for i in range(1, self.num_of_masses - 1):
            numerator = torch.norm(torch.cross(self.mass_positions[i + 1] - self.mass_positions[i], self.mass_positions[i] - self.mass_positions[i - 1]))
            denominator = torch.dot(self.mass_positions[i + 1] - self.mass_positions[i], self.mass_positions[i] - self.mass_positions[i - 1])
            self.beta[i] = torch.atan(numerator / denominator)

        self.beta[-1] = 0.0

    def bendingSpringForces(self):
        self.updateBeta()

        self.bending_forces[0] = (self.bending_spring * self.beta[1] / self._getLinkLenghtNorm(1)) * (self.tripleCross(1, 1, 2) / torch.sin(self.beta[1]))

        self.bending_forces[1] = - (self.bending_spring * self.beta[1] / self._getLinkLenghtNorm(1)) * (self.tripleCross(1, 1, 2) / torch.sin(self.beta[1])) \
                                - (self.bending_spring * self.beta[1] / self._getLinkLenghtNorm(2)) * (self.tripleCross(2, 1, 2) / torch.sin(self.beta[1])) \
                                + (self.bending_spring * self.beta[2] / self._getLinkLenghtNorm(2)) * (self.tripleCross(2, 2, 3) / torch.sin(self.beta[2]))

        for i in range(2, self.num_of_masses - 2):
            self.bending_forces[i] = (self.bending_spring * self.beta[i - 1] / self._getLinkLenghtNorm(i)) * (self.tripleCross(i, i - 1, i) / torch.sin(self.beta[i - 1])) \
                                    - (self.bending_spring * self.beta[i] / self._getLinkLenghtNorm(i)) * (self.tripleCross(i, i, i + 1) / torch.sin(self.beta[i])) \
                                    - (self.bending_spring * self.beta[i] / self._getLinkLenghtNorm(i + 1)) * (self.tripleCross(i + 1, i, i + 1) / torch.sin(self.beta[i])) \
                                    + (self.bending_spring * self.beta[i + 1] / self._getLinkLenghtNorm(i + 1)) * (self.tripleCross(i + 1, i + 1, i + 2) / torch.sin(self.beta[i + 1]))

        self.bending_forces[self.num_of_masses - 2] = (self.bending_spring * self.beta[self.num_of_masses - 3] / self._getLinkLenghtNorm(self.num_of_masses - 2)) * (self.tripleCross(self.num_of_masses - 2, self.num_of_masses - 3, self.num_of_masses - 2) / torch.sin(self.beta[self.num_of_masses - 3])) \
                                                    - (self.bending_spring * self.beta[self.num_of_masses - 2] / self._getLinkLenghtNorm(self.num_of_masses - 2)) * (self.tripleCross(self.num_of_masses - 2, self.num_of_masses - 2, self.num_of_masses - 1) / torch.sin(self.beta[self.num_of_masses - 2]))

        self.bending_forces[-1] = (self.bending_spring * self.beta[self.num_of_masses - 2] / self._getLinkLenghtNorm(self.num_of_masses - 1)) * self.tripleCross(self.num_of_masses - 1, self.num_of_masses - 2, self.num_of_masses - 1) / torch.sin(self.beta[self.num_of_masses - 1])

        for i in range(self.num_of_masses):
            # Check if the force vector is finite
            if not torch.isfinite(self.bending_forces[i]).all():
                self.bending_forces[i] = torch.zeros(3, device=self.device)  # Replace with zeros if not finite

    def add_initial_constrain_spring(self):
       pass

# Esempio di utilizzo
mass_spring_damping_system = MassSpringDamping(num_of_masses=10, cable_length=5.0, total_mass=2.0, cable_diameter=0.1)
