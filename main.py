from agents import PATSX, Moth
from controllers import MPCController
from environment import Env

geofence = [(-3,3), (-1.8, 0), (-2.95, 0)]

agent = PATSX()
controller = MPCController(agent=agent, gf = geofence)
moth = Moth()

env = Env(agent=agent, controller=controller, target=moth, gf = geofence)