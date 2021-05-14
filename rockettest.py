# Testing forces
import rocketenv
import pandas
from time import sleep
dt = 0.1
env = rocketenv.RocketEnv([200,200], dt)
env.initrender()

controller = rocketenv.RocketController(1.301, 10.049, 0.051, 0.031, 4.52, 0.202)
env.reset()
env.rocket.vx = 6.88531
env.rocket.vy = 7.15891
env.rocket.theta = 3.72495
env.rocket.omega = 4.36369
print("Initial",env.rocket.x,env.rocket.y,env.rocket.vx,env.rocket.vy,env.rocket.theta,env.rocket.omega)
for i in range(1,1000+1):
    sleep(1/60)
    f1, f2 = controller.step(env.rocket.vx, env.rocket.vy, env.rocket.omega, env.rocket.theta)
    f1 = max(0, min(rocketenv.Rocket.ROCKET_MAX_INDIVIDUAL_FORCE, f1))
    f2 = max(0, min(rocketenv.Rocket.ROCKET_MAX_INDIVIDUAL_FORCE, f2))
    env.step((f1,f2))
    print(i,env.rocket.x,env.rocket.y,env.rocket.vx,env.rocket.vy,env.rocket.theta,env.rocket.omega,f1,f2)
    env.render()
    if env.isFailed():
        print("err", i)
        break
    if env.isStable():
        print("win", i)
        break