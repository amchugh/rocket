# Record a simulation
import rocketenv
import os
import pygame
import imageio
import PIL
from PIL import Image

pygame.init()
pygame.font.init()
FONT = pygame.font.SysFont(None, 24)

dt = 1/30
#dt = 1/5
env = rocketenv.RocketEnv(rocketenv.DEFAULT_SIZE, dt)
#controller = rocketenv.RocketController( 1.301, 10.049, 0.051, 0.031, 4.52, 0.202, dt)
#controller = rocketenv.RocketController(1.30267606, 5.08057665, 0.00301693, 0.07480625, 4.60797357, 0.21579250, dt)
#controller = rocketenv.RocketController(1.30180042, 5.07822616, 0.00407172, 0.09638811, 4.64927884, 0.22577127, 0.62695137, dt)
#controller = rocketenv.RocketController(1.30180042, 5.07822616, 0.00407172, 0.09638811, 4.64927884, 0.22577127, 0.62695137, dt)
controller = rocketenv.RocketController(1.3, 5, 0.01, 0.01, 1, 0.1, 1)

#env.rocket.GRAVITY = 0.1
env.NEEDED_STEPS = 60 # We want it to really pause at the end


def makeGifFromEnv(_name, desc):
    # Make a directory for the temporary frames
    try:
        os.makedirs("tmp")
    except OSError:
        pass

    controller.reset()
    env.initrender(None, False)
    
    description = FONT.render(desc, True, (0,0,0))

    for i in range(1_000):
        # Save the current image
        env.render()
        fn = "../tmp/%04d.png" % i
        env.screen.blit(FONT.render(str(i), True, (0,0,0)), (6,36))
        env.screen.blit(description, (400-120,6))
        pygame.draw.circle(env.screen, (255,0,0), (200,200), 2, 0)
        
        # Consume events and stuff
        pygame.display.flip()
        pygame.event.get()
        
        pygame.image.save(env.screen, fn)
        
        #f1, f2 = controller.moveStep(env.rocket.x, env.rocket.y, env.rocket.vx, env.rocket.vy, env.rocket.omega, env.rocket.theta)
        f1, f2 = controller.step(env.rocket.vx, env.rocket.vy, env.rocket.omega, env.rocket.theta)
        env.step((f1,f2))
        if env.isStable() or env.isFailed():
            break

    env.closerender()
    print("Finished making images", i)

    with imageio.get_writer(_name, mode="I", fps=30) as writer:
        for j in range(i):
            # we will only write every n other frames
            if not j % 6 == 0:
                continue
            image = imageio.imread("../tmp/%04d.png" % j)
            writer.append_data(image)
        lastimage = imageio.imread("../tmp/%04d.png" % (i))
        for _ in range(10):
            writer.append_data(lastimage)

    print("Generated '%s'" % _name)

name = '../videos/bad_numbered.mp4'

"""
env.resetRandom(0)
makeGifFromEnv('tmp/final.gif')
"""

for i in range(0,10):
    env.resetRandom(i+201)
    makeGifFromEnv('../gifs/%02d.gif' % i, 'Scenario #%d' % (i+1))

with imageio.get_writer('../tmp/final.gif', mode="I", fps=30) as writer:
    for i in range(10):
        images = imageio.mimread('../gifs/%02d.gif' % i, memtest=False)
        for im in images:
            writer.append_data(im)
print("Generated '../gifs/final.gif'")



# Optimze gif
#from pygifsicle import optimize
#optimize('tmp/final.gif')

# gif to mp4
import ffmpy
ff = ffmpy.FFmpeg(inputs={'../tmp/final.gif':None},outputs={name:'-pix_fmt yuv420p'})
ff.run()
print("Finished. Created '%s'" % name)