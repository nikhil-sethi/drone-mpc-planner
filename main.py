from agents import PATSX, Moth
from controllers import MPCController
from environment import Env

geofence = [(-1,1), (-1.3, 0), (-1.95, 0)]

agent = PATSX()
controller = MPCController(agent=agent, gf = geofence)
moth = Moth()

env = Env(agent=agent, controller=controller, target=moth, gf = geofence)