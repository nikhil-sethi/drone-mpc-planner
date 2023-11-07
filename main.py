from agents import Drone, Moth
from controllers import MPC, Constant
from environment import Env
from models import SecondOrderGravity, FirstOrder

drone_model = SecondOrderGravity(n_states=6, n_actions=3)
agent = Drone(model = drone_model, controller = MPC(model=drone_model), init_state = [0,-1,-1, 0,0,0])

# moth_model = FirstOrder(n_states=6, n_actions=3)
# moth = Particle(model = moth_model, controller=Constant(action=[0.8, 0, 0]), init_state = )

moth = Moth(init_state = [-1,-0.5,-1.2, 1,0,0])
env = Env(agent=agent, target=moth)