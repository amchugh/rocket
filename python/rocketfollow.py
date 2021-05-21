import rocketenv
import pygame


dt = 1/5.0
env = rocketenv.RocketEnv((800, 800), dt)
env.reset();
env.initrender(None, False)

clock = pygame.time.Clock()

controller = rocketenv.MovingRocketController(1.30180042, 5.07822616, 0.00407172, 0.09638811, 4.64927884, 0.22577127, 0.62695137, dt)
controller.reset()

running = True

while running:
    clock.tick(60)
    
    keys = pygame.key.get_pressed()
    if keys[pygame.K_x]:
        env.resetRandom(0);
        
    env.render()
    
    if pygame.mouse.get_pressed()[0]: controller.target = pygame.mouse.get_pos()
    
    f1, f2 = controller.step(env.rocket.x, env.rocket.y, env.rocket.vx, env.rocket.vy, env.rocket.omega, env.rocket.theta) 
    
    env.step((f1, f2))
    
    pygame.draw.circle(env.screen, (255,0,0), controller.target, 2, 0)
    pygame.display.flip()
    
    # event handling, gets all event from the event queue
    for event in pygame.event.get():
        # only do something if the event is of type QUIT
        if event.type == pygame.QUIT:
            # change the value to False, to exit the main loop
            running = False
            