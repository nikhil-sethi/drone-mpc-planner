import numpy as np
import matplotlib.pyplot as plt
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from scipy.interpolate import interp1d
from test_control import export_ode_model
import casadi as ca


# Define the ACADOS model for a particle with 2nd-order dynamics
model = AcadosModel()

# model.set('name', 'particle_tracking')

nx = 4  # Number of states (position and velocity for x and y)
nu = 2  # Number of control inputs (acceleration in x and y)

# model.set('T', 0.1)  # Sampling time (adjust as needed)

# # Create symbolic variables for the state and control inputs
x = model.sym('x', nx)
u = model.sym('u', nu)

# # Define the dynamics (2nd-order Newtonian equations)
# f_expr = [x[2], x[3], u[0], u[1]]
# model.set('dyn_expr', f_expr)


# Set the cost function (example: tracking a desired trajectory)
Q = np.diag([1.0, 1.0, 0.1, 0.1])  # State cost matrix
R = np.diag([0.01, 0.01])         # Control cost matrix

cost_expr = x.T @ Q @ x + u.T @ R @ u
model.set('cost_expr', cost_expr)

# Set constraints (example: box constraints on control inputs)
lbx = [-np.inf, -np.inf, -1.0, -1.0]  # Lower bounds on state and control
ubx = [np.inf, np.inf, 1.0, 1.0]      # Upper bounds on state and control
model.set('constr_x0', initial_state)  # Initial state constraint

# Create the ACADOS OCP (Optimal Control Problem) and solver
ocp = AcadosOcp()
model = export_ode_model()
ocp.model = model


N = 20  # Prediction horizon (adjust as needed)
ocp.dims.N = N
ocp.dims.nx = nx
ocp.dims.nu = nu

# Create a time grid for prediction horizon
T = N * model.get('T')
ocp.cost.W_e = Q  # Terminal cost



# Create a reference trajectory (example: sinusoidal)
t_ref = np.linspace(0, T, N+1)
x_ref = np.array([np.sin(t_ref), np.cos(t_ref), np.cos(t_ref), -np.sin(t_ref)]).T
u_ref = np.array([np.cos(t_ref), -np.sin(t_ref)]).T

ocp.cost.yref = x_ref  # Reference trajectory for state
ocp.cost.yref_e = x_ref[-1, :]  # Terminal reference

# Define solver options
opts = {'print_level': 0, 'ipopt.tol': 1e-3}

ocp.solver_options = opts

# Create the solver
solver = AcadosOcpSolver(ocp, opts)

# Initial conditions
initial_state = np.array([0.0, 1.0, 0.0, 0.0])  # [x, y, vx, vy]
ocp.set('constr_x0', initial_state)

# Simulate the MPC controller
x_sim = np.zeros((N+1, nx))
x_sim[0, :] = initial_state

u_sim = np.zeros((N, nu))

for i in range(N):
    # Solve the optimization problem
    ocp.set('constr_x0', x_sim[i, :])
    ocp.set('constr_lbx', lbx)
    ocp.set('constr_ubx', ubx)

    status = solver.solve()

    if status != 0:
        raise Exception("Solver failed!")

    # Extract the control input for the current time step
    u_opt = solver.get('u')

    # Simulate the system dynamics
    x_next = np.array([x_sim[i, 0] + x_sim[i, 2] * model.get('T'),
                        x_sim[i, 1] + x_sim[i, 3] * model.get('T'),
                        u_opt[0], u_opt[1]])

    x_sim[i+1, :] = x_next
    u_sim[i, :] = u_opt

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(x_sim[:, 0], x_sim[:, 1], label='Particle Path')
plt.plot(x_ref[:, 0], x_ref[:, 1], linestyle='--', label='Reference Trajectory')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.legend()
plt.title('MPC Trajectory Tracking for a Particle')
plt.grid(True)
plt.show()
