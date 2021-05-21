# -- Record Simulations and Generate Videos --
#
# This file requires 'ffmpy' which utilizes ffmpeg to create the video. 
# Make sure that ffmpeg is installed (and on path if you are using windows).
# Go to http://ffmpeg.org/download.html to learn more.
# If you are using windows, go here https://github.com/BtbN/FFmpeg-Builds/releases and select the "ffmpeg-N-...-win64-gpl.zip" build.
# It is probably the right one. Download it, unzip it, then add the "bin" folder to your system PATH. Restart your shells, and it should work.
#
# I would recommend VLC for viewing the videos :)
# https://www.videolan.org/
# Their free software do a great job of playing the video at a smooth and consistent framerate
#
# Call this file with the name of the file to create as the first argument. 
# Edit the values under 'Configureables::' to change the controller, simulation constants, etc

import rocketenv
import os
import pygame
import imageio
import ffmpy

# Configureables::

MAX_FRAMES = 3200
frame_skips = 6

# Other possible controllers
#controller = rocketenv.RocketController( 1.301, 10.049, 0.051, 0.031, 4.52, 0.202, dt)
#controller = rocketenv.RocketController(1.30267606, 5.08057665, 0.00301693, 0.07480625, 4.60797357, 0.21579250, dt)
#controller = rocketenv.RocketController(1.30180042, 5.07822616, 0.00407172, 0.09638811, 4.64927884, 0.22577127, 0.62695137, dt)
#controller = rocketenv.RocketController(1.3, 5, 0.01, 0.01, 1, 0.1, 1)

dt = 1/30
env = rocketenv.RocketEnv((800, 800), dt)
env.reset();
env.rocket.GRAVITY = 1
#controller = rocketenv.MovingRocketController(1.30291835, 5.08818357, 0.00561298, 0.09783819, 4.65615773, 0.20216449, 0.67209208, dt)
#controller = rocketenv.MovingRocketController(1.29985408, 5.06374312, 0.00210933, 0.09633287, 4.64475739, 0.21874083, 0.60593377, dt)
controller = rocketenv.MovingRocketController(1.29980793, 5.05956982, 0.00372743, 0.09874277, 4.64499910, 0.20881341, 0.56076263, dt)
controller.reset()

def makeGifFromEnv(_name, _cont, desc):
    # Make a directory for the temporary frames

    _cont.reset()
    env.initrender(None, False)
    
    font = pygame.font.SysFont(None, 24)
    description = font.render(desc, True, (0,0,0))

    for i in range(MAX_FRAMES):
        # Save the current image
        env.render()
        fn = "../tmp/%04d.png" % i
        env.screen.blit(font.render(str(i), True, (0,0,0)), (6,36))
        env.screen.blit(description, (400-120,6))
        pygame.draw.circle(env.screen, (255,0,0), (200,200), 2, 0)
        
        # Consume events and stuff
        pygame.display.flip()
        pygame.event.get()
        
        pygame.image.save(env.screen, fn)
        
        #f1, f2 = _cont.step(env.rocket.vx, env.rocket.vy, env.rocket.omega, env.rocket.theta)
        f1, f2 = _cont.step(env.rocket.x, env.rocket.y, env.rocket.vx, env.rocket.vy, env.rocket.omega, env.rocket.theta) 
        env.step((f1,f2))
        #if env.isStable() or env.isFailed():
        #    break

    env.closerender()
    print("Finished making images", i)

    with imageio.get_writer(_name, mode="I", fps=30) as writer:
        for j in range(i):
            # we will only write every n other frames
            if not j % frame_skips == 0:
                continue
            image = imageio.imread("../tmp/%04d.png" % j)
            writer.append_data(image)
        lastimage = imageio.imread("../tmp/%04d.png" % (i))
        for _ in range(10):
            writer.append_data(lastimage)

    print("Generated '%s'" % _name)

if __name__ == "__main__":
    import sys
    name = "default"
    if len(sys.argv) > 1:
        name = sys.argv[1]

    # Make the required directories
    try:
        os.makedirs("../tmp")
        os.makedirs("../videos")
    except OSError:
        pass

    # Format the name /w path
    if not name.endswith('.mp4'): name = name + ".mp4"
    name = "../videos/" + name
    print("Creating '%s'" % name)

    if len(sys.argv) > 2 and sys.argv[2] == 'just-video':
        ff = ffmpy.FFmpeg(inputs={'../tmp/final.gif':None},outputs={name:'-pix_fmt yuv420p'})
        ff.run()
        print("Finished. Created '%s'" % name)
        sys.exit(0)

    # Simulate the ten scenarios
    """
    for i in range(0,10):
        env.resetRandom(i+201)
        makeGifFromEnv('../tmp/%02d.gif' % i, controller, 'Scenario #%d' % (i+1))

    # Concatenate all the scenarios into one big gif
    with imageio.get_writer('../tmp/final.gif', mode="I", fps=30) as writer:
        for i in range(10):
            images = imageio.mimread('../tmp/%02d.gif' % i, memtest=False)
            for im in images:
                writer.append_data(im)
    print("Generated '../tmp/final.gif'")
    """
    
    makeGifFromEnv('../tmp/final.gif', controller, '')

    # Convert the gif to mp4
    ff = ffmpy.FFmpeg(inputs={'../tmp/final.gif':None},outputs={name:'-pix_fmt yuv420p'})
    ff.run()
    print("Finished. Created '%s'" % name)