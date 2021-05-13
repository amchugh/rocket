# rocket genetic algorithm
import rocketenv
import random
from math import pi
import operator

dt = 1/10
env = rocketenv.RocketEnv(rocketenv.DEFAULT_SIZE, dt)

controller = rocketenv.RocketController(1.301, 10.049, 0.051, 0.031, 4.52, 0.202)

# Returns the number of steps to reach stability
MAX_STEPS = 300
def testController(_cont, seed):
    env.resetRandom(seed)
    _cont.reset()
    for i in range(1,MAX_STEPS+1): # random number of max steps
        f1, f2 = _cont.step(env.rocket.vx, env.rocket.vy, env.rocket.omega, env.rocket.theta)
        env.step((f1,f2))
        if env.isStable() or env.isFailed():
            break
    return i if env.isStable() else MAX_STEPS
    
def watchController(_cont, seed):
    from time import sleep
    env.initrender()
    env.resetRandom(seed)
    _cont.reset()
    env.render()
    for i in range(MAX_STEPS): # random number of max steps
        sleep(1/60)
        f1, f2 = _cont.step(env.rocket.vx, env.rocket.vy, env.rocket.omega, env.rocket.theta)
        env.step((f1,f2))
        env.render()
        if env.isStable() or env.isFailed():
            break
    return i if env.isStable() else MAX_STEPS
    
seeds = [i for i in range(0,10000)]
def evaluateFitness(_cont):
    _cont.fitness = 0
    for s in seeds:
        df = testController(_cont, s)
        _cont.fitness += df
    _cont.fitness /= len(seeds)
    env.closerender()
    return _cont.fitness
    
import time
start = time.time()
print(evaluateFitness(controller))
end = time.time()
print(end - start)