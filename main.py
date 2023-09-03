from agents import PATSX, Moth
from controllers import MPCController
from environment import Env

agent = PATSX()
controller = MPCController(agent=agent)
moth = Moth()

env = Env(agent=agent, controller=controller, target=moth)