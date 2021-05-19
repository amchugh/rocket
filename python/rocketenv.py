import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
from math import sin, cos, floor, atan2, pi, degrees
import numpy as np
import random

DEFAULT_SIZE = (400,400)
DEBUG_TEXT = True
if DEBUG_TEXT:
    pygame.font.init()
    FONT = pygame.font.SysFont(None, 24)

# Rotates a point around (0,0) and then moves it 
# such that is centered at (dx, dy)
#   s should be sin(theta)
#   c should be cos(theta)
def rotatePoint(px, py, s, c, dx, dy):
    return (px * c - py * s + dx, px * s + py * c + dy)

class Rocket:

    # Constants
    GRAVITY = 1
    ROCKET_MASS = 1
    ROCKET_ROTATIONAL_INERTIA = 0.6
    ROCKET_MAX_INDIVIDUAL_FORCE = 3.2
    ROCKET_VISUAL_SIZE = 10
    
    # Dynamics
    x = DEFAULT_SIZE[0]/2
    y = DEFAULT_SIZE[1]/2
    theta = 0
    vx = 0
    vy = 0
    omega = 0
    
    last_f1 = 0
    last_f2 = 0

    def __init__(self, x0, y0):
        self.x = x0
        self.y = y0
        
    def step(self, actions, dt):
        # Cap forces
        f1 = max(0, min(self.ROCKET_MAX_INDIVIDUAL_FORCE, actions[0]))
        f2 = max(0, min(self.ROCKET_MAX_INDIVIDUAL_FORCE, actions[1]))
        
        # -- Euler Integration --
        # Acceleration
        ay = self.GRAVITY - (f1 + f2) * cos(self.theta) / self.ROCKET_MASS
        ax = (f1 + f2) * sin(self.theta) / self.ROCKET_MASS
        alpha = (f1 - f2) / self.ROCKET_ROTATIONAL_INERTIA
        # Velocity
        self.vy += ay * dt
        self.vx += ax * dt
        self.omega += alpha * dt
        # Position
        self.y += self.vy * dt
        self.x += self.vx * dt
        self.theta += self.omega * dt
        while self.theta > 2*pi:
            self.theta -= 2*pi
        while self.theta < 0:
            self.theta += 2*pi
        
        # Book-keeping
        self.last_f1 = f1
        self.last_f2 = f2
        
    def draw(self, screen):
        size = self.ROCKET_VISUAL_SIZE
        # Draw a big box
        points = [0,0,0,0]
        s = sin(self.theta)
        c = cos(self.theta)
        points[0] = rotatePoint(size, size, s, c, self.x, self.y)
        points[1] = rotatePoint(-size, size, s, c, self.x, self.y)
        points[2] = rotatePoint(-size,-size, s, c, self.x, self.y)
        points[3] = rotatePoint(size,-size, s, c, self.x, self.y)
        pygame.draw.polygon(screen, (0,0,255), points, 1)
        # Draw a little triangle
        p2 = [0,0,0]
        p2[0] = points[2]
        p2[1] = points[3]
        p2[2] = (self.x, self.y)
        pygame.draw.polygon(screen, (0,0,255), p2, 1)
        # Draw two thrusters
        t = [0,0,0]
        t[0] = rotatePoint(size,0,s, c, self.x, self.y)
        t[1] = rotatePoint(size*2,0,s, c, self.x, self.y)
        t[2] = rotatePoint(size*2,size, s, c, self.x, self.y)
        colo = self.last_f2 / self.ROCKET_MAX_INDIVIDUAL_FORCE
        pygame.draw.lines(screen, (255*colo,0,255*(1-colo)), False, t)
        t[0] = rotatePoint(-size,0,s, c, self.x, self.y)
        t[1] = rotatePoint(-size*2,0,s, c, self.x, self.y)
        t[2] = rotatePoint(-size*2,size, s, c, self.x, self.y)
        colo = self.last_f1 / self.ROCKET_MAX_INDIVIDUAL_FORCE
        pygame.draw.lines(screen, (255*colo,0,255*(1-colo)), False, t)

class RocketEnv:
    features = 7
    actions = 2
    rendering = False
    own_window = True
    
    # Delta time is in seconds
    def __init__(self, size=DEFAULT_SIZE, deltaTime=0.1):
        self.world_size = size
        self.rocket = Rocket(size[0]/2,size[1]/2)
        self.deltaTime = deltaTime
        self.reset()
        
    def step(self, actions):
        self.rocket.step(actions, self.deltaTime)
        self.steps += 1
        
    def reset(self):
        self.rocket.x = self.world_size[0]/2
        self.rocket.y = self.world_size[1]/2
        self.rocket.theta = 0
        self.rocket.vx = 0
        self.rocket.vy = 0
        self.rocket.omega = 0
        self.stable_steps = 0
        self.steps = 0
        
    def resetRandom(self, seed=0):
        random.seed(seed)
        self.reset()
        self.rocket.vy = (random.random() - 0.5) * 2 * 10
        self.rocket.vx = (random.random() - 0.5) * 2 * 10
        self.rocket.theta = random.random() * 2 * pi
        self.rocket.omega = (random.random() - 0.5) * 2 * pi
        
    def sense(self):
        target = (self.world_size[0]/2, self.world_size[1]/2)
        senses = [
            target[0] - self.rocket.x,
            target[1] - self.rocket.y,
            sin(self.rocket.theta),
            cos(self.rocket.theta),
            self.rocket.vx * self.deltaTime,
            self.rocket.vy * self.deltaTime,
            self.rocket.omega * self.deltaTime,
        ]
        return senses
        
    VX_TOLERANCE = 0.1
    VY_TOLERANCE = 0.3
    THETA_TOLERANCE = 0.2
    NEEDED_STEPS = 5
    def isStable(self):
        if abs(shortestTurn(self.rocket.theta, 0)) < self.THETA_TOLERANCE and abs(self.rocket.vx) < self.VX_TOLERANCE and abs(self.rocket.vy) < self.VY_TOLERANCE:
            self.stable_steps += 1
            if self.stable_steps >= self.NEEDED_STEPS: return True
        else:
            self.stable_steps = 0
        return False
        
    def isFailed(self):
        return self.rocket.x < 0 or self.rocket.x > self.world_size[0] or self.rocket.y < 0 or self.rocket.y > self.world_size[1]
        
    def render(self):
        global DEBUG_TEXT
        if not self.rendering: return
        self.screen.fill((255,255,255))
        self.rocket.draw(self.screen)
        if DEBUG_TEXT:
            l = lambda x: str(floor(x*1000)/1000)
            t = FONT.render(l(self.rocket.x) + " " + l(self.rocket.y) + " " + l(self.rocket.theta), True, (255,0,0))
            self.screen.blit(t, (0,0))
            t2 = FONT.render(l(self.rocket.vx) + " " + l(self.rocket.vy) + " " + l(self.rocket.omega), True, (0,255,0))
            self.screen.blit(t2, (0, 16))
        if self.own_window:
            pygame.display.flip()
            pygame.event.get()
        
    def initrender(self, screen=None, own_window=True):
        if not self.rendering:
            self.rendering = True
            pygame.init()
            pygame.display.set_caption("Rocket Env")
            if screen is None:
                self.screen = pygame.display.set_mode(self.world_size)
            else:
                self.screen = screen
            self.own_window = own_window
            
    def closerender(self):
        if self.rendering:
            self.rendering = False
            pygame.display.quit()

class MovingRocketController:
    target = (200, 200)
    def __init__(self, safe_turn_speed, dist_scalar, theta_tolerance, rot_scalar, proportional_scalar, moving_scalar, derivative_scalar, dt=0.1):
        self.dist_scalar = dist_scalar
        self.theta_tolerance = theta_tolerance
        self.safe_turn_speed = safe_turn_speed
        self.proportional_scalar = proportional_scalar
        self.derivative_scalar = derivative_scalar
        self.rot_scalar = rot_scalar
        self.moving_scalar = moving_scalar
        self.dt = dt
        self.reset()
        
    def reset(self):
        self.I = 0
        self.e_prev = 0

    def step(self, x, y, vx, vy, omega, theta):
        dx = self.target[0] - x
        dy = self.target[1] - y
        
        target = atan2(dy, dx) 
        
        ndx, ndy = normalize(dx, dy)
        ndx *= self.dist_scalar
        ndy *= self.dist_scalar
        nvx, nvy = normalize(vx, vy)
        
        dvx = ndx - vx
        dvy = ndy - vy
        
        t = atan2(dvy, dvx)
        t += pi / 2
        if t > 2*pi:
            t -= 2*pi
        
        delta = shortestTurn(theta, t)
        
        if abs(delta) > self.theta_tolerance:
            if abs(omega) > self.safe_turn_speed: e = omega
            else: e = omega - (self.safe_turn_speed * (-1 if delta < 0 else 1) )
                
            self.I = self.I + e*self.dt
            mv = (e*self.proportional_scalar + self.I + (e-self.e_prev)/self.dt*self.derivative_scalar)
            
            self.e_prev = e
            
            return -mv*self.rot_scalar, mv*self.rot_scalar
        else:
            d = dist(dx-x, dy-y)
            return self.moving_scalar*d*d, self.moving_scalar*d*d

class RocketController:
    def __init__(self, safe_turn_speed, vel_kill_scalar, theta_tolerance, rot_scalar, proportional_scalar, mag_tolerance, derivative_scalar, dt=0.1):
        self.dt = dt
        self.safe_turn_speed = safe_turn_speed
        self.vel_kill_scalar = vel_kill_scalar
        self.theta_tolerance = theta_tolerance
        self.rot_scalar = rot_scalar
        self.proportional_scalar = proportional_scalar
        self.mag_tolerance = mag_tolerance
        self.derivative_scalar = derivative_scalar
        self.reset()
        self.fitness = -1
        
    def reset(self):
        self.I = 0
        self.e_prev = 0
        
    # Returns desired force for left, right
    def step(self, vx, vy, omega, theta):
        target = atan2(vy, vx) + 3*pi/2
        if target > 2*pi:
            target -= 2*pi
        delta = shortestTurn(theta, target)
        mag = dist(vx, vy)
        if abs(delta) <= self.theta_tolerance and mag > self.mag_tolerance:
            # Use the vector magnitude as the amount to slow by
            return mag*self.vel_kill_scalar, mag*self.vel_kill_scalar
            
        else:
            if abs(omega) > self.safe_turn_speed: e = omega
            else: e = omega - (self.safe_turn_speed * (-1 if delta < 0 else 1) )
                
            self.I = self.I + e*self.dt
            mv = (e*self.proportional_scalar + self.I + (e-self.e_prev)/self.dt*self.derivative_scalar)
            
            self.e_prev = e
            
            return -mv*self.rot_scalar, mv*self.rot_scalar

def normalize(x,y):
    if x == y and y == 0:
        return 1, 0
    mg = (x*x + y*y) ** 0.5
    return x/mg, y/mg
    
def dist(x,y):
    return (x*x+y*y)**0.5

# Given two angles in domain [0, 2pi],
# return the minimal rotation to tranform
# current into target
def shortestTurn(current, target):
    delt = target - current
    if delt > pi:
        delt = -2*pi + delt
    if delt < -pi:
        delt = 2*pi + delt
    return delt
    
def testShortestTurn(c, t, expected):
    assert shortestTurn(c, t) == expected, "Expected {0} ({2}) Got {1} ({3})".format(expected, shortestTurn(c, t), degrees(expected), degrees(shortestTurn(c, t)))

def main():
    dt = 1/30.0
    env = RocketEnv(DEFAULT_SIZE, dt)
    env.initrender(None, False)
    clock = pygame.time.Clock()
    
    f1 = Rocket.GRAVITY / 2
    f2 = Rocket.GRAVITY / 2
    
    p = [False, False, False, False]
    delt = 0.001
    
    pid = RocketController(1.30180042, 5.07822616, 0.00407172, 0.09638811, 4.64927884, 0.22577127, 0.62695137, dt)
    
    running = True
    while running:
        clock.tick(60)
        
        keys = pygame.key.get_pressed()
        
        if keys[pygame.K_SPACE]:
            pygame.event.get()
            continue
            
        if keys[pygame.K_x]:
            env.resetRandom(0);
            
        f1 += (int(keys[pygame.K_q]) - int(keys[pygame.K_a])) * delt
        f2 += (int(keys[pygame.K_w]) - int(keys[pygame.K_s])) * delt
        
        env.render()
        
        pidx, pidy = pid.step(env.rocket.vx, env.rocket.vy, env.rocket.omega, env.rocket.theta) 
        
        if keys[pygame.K_r]:
            env.rocket.vx = 0
            env.rocket.vy = 0
            env.rocket.theta = 0
            env.rocket.omega = 0
        
        if keys[pygame.K_e]:
            f2 = f1
        
        if keys[pygame.K_i]:
            env.step((pidx, pidy))
            # Draw the current force text too
            l1 = FONT.render(str(pidx), True, (0,0,0))
            env.screen.blit(l1, (0, 32))
            l2 = FONT.render(str(pidy), True, (0,0,0))
            env.screen.blit(l2, (0, 48))
        else:
            env.step((f1,f2))
            # Draw the current force text too
            l1 = FONT.render(str(f1), True, (0,0,0))
            env.screen.blit(l1, (0, 32))
            l2 = FONT.render(str(f2), True, (0,0,0))
            env.screen.blit(l2, (0, 48))
            
        if env.isStable():
            l3 = FONT.render("Stable", True, (0,0,0))
            env.screen.blit(l3, (0, 96))
            
        if env.isFailed():
            l3 = FONT.render("Failed", True, (0,0,0))
            env.screen.blit(l3, (0, 112))
        
        t = atan2(env.rocket.vy, env.rocket.vx)
        if t < 0: t += 2*pi
        
        l4 = FONT.render(str(env.rocket.theta), True, (0,0,0))
        env.screen.blit(l4, (0, 80))
        # draw a line
        s = sin(t)
        c = cos(t)
        p = rotatePoint(20,0,s, c, env.rocket.x, env.rocket.y)
        pygame.draw.line(env.screen, (0,0,0), (env.rocket.x, env.rocket.y), p)
        pygame.draw.line(env.screen, (255,0,0), (env.rocket.x, env.rocket.y), (env.rocket.x + env.rocket.vx*10, env.rocket.y))
        pygame.draw.line(env.screen, (0,255,0), (env.rocket.x, env.rocket.y), (env.rocket.x, env.rocket.y + env.rocket.vy*10))
        
        target = t
        s = sin(target)
        c = cos(target)
        p = rotatePoint(20,0,s, c, env.rocket.x, env.rocket.y)
        pygame.draw.line(env.screen, (255,0,255), (env.rocket.x, env.rocket.y), p)
        
        if keys[pygame.K_p]:
            env.rocket.theta = t
        
        l3 = FONT.render(str(env.rocket.theta - target), True, (0,0,0))
        env.screen.blit(l3, (0, 64))
        
        pygame.display.flip()
        
        # event handling, gets all event from the event queue
        for event in pygame.event.get():
            # only do something if the event is of type QUIT
            if event.type == pygame.QUIT:
                # change the value to False, to exit the main loop
                running = False

def autoMain():
    dt = 1/5.0
    env = RocketEnv((800, 800), dt)
    env.reset();
    env.rocket.GRAVITY = 0.1
    env.initrender(None, False)
    clock = pygame.time.Clock()
    controller = MovingRocketController(2, 0.01, 0.1, 1, 1, 0.1, 0.01, dt)
    controller = MovingRocketController(1.30291835, 5.08818357, 0.00561298, 0.09783819, 4.65615773, 0.20216449, 0.67209208, dt)
    controller.reset()
    running = True
    while running:
        clock.tick(60)
        
        keys = pygame.key.get_pressed()
        if keys[pygame.K_x]:
            env.resetRandom(0);
            
        env.render()
        
        f1, f2 = controller.step(env.rocket.x, env.rocket.y, env.rocket.vx, env.rocket.vy, env.rocket.omega, env.rocket.theta) 
        
        env.step((f1, f2))
        
        if env.isStable():
            l3 = FONT.render("Stable", True, (0,0,0))
            env.screen.blit(l3, (0, 96))
            
        if env.isFailed():
            l3 = FONT.render("Failed", True, (0,0,0))
            env.screen.blit(l3, (0, 112))
        
        pygame.draw.circle(env.screen, (255,0,0), (200,200), 2, 0)
        pygame.display.flip()
        
        # event handling, gets all event from the event queue
        for event in pygame.event.get():
            # only do something if the event is of type QUIT
            if event.type == pygame.QUIT:
                # change the value to False, to exit the main loop
                running = False
    
if __name__ == "__main__":
    testShortestTurn(0, pi, pi)
    testShortestTurn(0, pi/2, pi/2)
    testShortestTurn(pi/2, 3/2*pi, pi)
    testShortestTurn(pi/2, 7/4*pi, -3/4*pi)
    testShortestTurn(7/4*pi, pi/2, 3/4*pi)
    testShortestTurn(pi/2, pi/4, -pi/4)
    print("Passed all shortestTurn tests")

    autoMain()
    #main()
