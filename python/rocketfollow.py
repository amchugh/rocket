import rocketenv
import pygame
import random

SIZE = (800,800)

class Missile:
    def __init__(self, world_size, pos, vel, dt):
        self.world_size = world_size
        self.pos = [pos[0], pos[1]]
        self.vel = vel
        self.dt = dt
    def step(self):
        self.pos[0] += self.vel[0] * self.dt
        self.pos[1] += self.vel[1] * self.dt
        return self.pos[0] < 0 or self.pos[0] > self.world_size[0] or self.pos[1] < 0 or self.pos[1] > self.world_size[1]
    def draw(self, screen):
        LENGTH = 1
        fin = (self.pos[0] - self.vel[0] * LENGTH, self.pos[1] - self.vel[1] * LENGTH)
        pygame.draw.line(screen, (255,0,120), self.pos, fin, 3)

SLANT = 200
def makeMissile():
    startX = random.randint(SLANT,SIZE[0]-SLANT)
    endX = random.randint(-SLANT,SLANT)
    vx, vy = rocketenv.normalize(endX, SIZE[1])
    vel = (vx*MISSILE_SPEED, vy*MISSILE_SPEED)
    return Missile(SIZE, (startX, 0), vel, dt)
    
fps = 60
dt = 1/30.0
env = rocketenv.RocketEnv(SIZE, dt)
env.reset();
env.initrender(None, False)

clock = pygame.time.Clock()

# ----------------
# ::Switch these::

# Old, easy controller
#controller = rocketenv.MovingRocketController(1.30180042, 5.07822616, 0.00407172, 0.09638811, 4.64927884, 0.22577127, 0.62695137, dt)
#env.rocket.ROCKET_MAX_INDIVIDUAL_FORCE = 3.2
#env.rocket.ROCKET_ROTATIONAL_INERTIA = 0.6

# Harder controller FITNESS: 2.51736263
#controller = rocketenv.MovingRocketController(1.29434620, 14.02993530, -0.00853724, 0.11117422, 4.65478087, 0.29473669, 0.76614155, dt)
#env.rocket.ROCKET_MAX_INDIVIDUAL_FORCE = 10
#env.rocket.ROCKET_ROTATIONAL_INERTIA = 0.4

# Slower, but looks better. Evaluated based on time to point FITNESS: 802.6
controller = rocketenv.MovingRocketController(1.30103649, 14.12807184, -0.00416717, 0.11021518, 4.79300645, 0.06620423, 0.81834318)
env.rocket.ROCKET_MAX_INDIVIDUAL_FORCE = 10
env.rocket.ROCKET_ROTATIONAL_INERTIA = 0.4

# ----------------

controller.reset()
running = True

missiles = []
MISSILE_SPEED = 30
ROCKET_INTERVAL = 1.6
DIST = 30
SQR_DIST = DIST * DIST 
c_i = 0

CENTER = (SIZE[0]/2,SIZE[1]/2)

while running:
    clock.tick(fps)
    
    keys = pygame.key.get_pressed()
    if keys[pygame.K_x]:
        env.resetRandom(0);
        
    env.render()
    
    if len(missiles) > 0:
        controller.target = missiles[0].pos
        
        dx = env.rocket.x - missiles[0].pos[0]
        dy = env.rocket.y - missiles[0].pos[1]
        if dx*dx + dy*dy <= SQR_DIST:
            missiles.remove(missiles[0])
    
    else:
        controller.target = CENTER
    
    c_i+=1
    if c_i >= ROCKET_INTERVAL * fps:
        c_i -= ROCKET_INTERVAL * fps
        missiles.append(makeMissile())
    
    for m in missiles:
        if m.step():
            missiles.remove(m)
        m.draw(env.screen)
    
    for i in range(6):
        f1, f2 = controller.step(env.rocket.x, env.rocket.y, env.rocket.vx, env.rocket.vy, env.rocket.omega, env.rocket.theta) 
        env.step((f1, f2))
    
    pygame.draw.circle(env.screen, (0,255,0), controller.target, 2, 0)
    pygame.draw.circle(env.screen, (0,0,255), (env.rocket.x, env.rocket.y), DIST, 1)
    pygame.display.flip()
    
    # event handling, gets all event from the event queue
    for event in pygame.event.get():
        # only do something if the event is of type QUIT
        if event.type == pygame.QUIT:
            # change the value to False, to exit the main loop
            running = False
            