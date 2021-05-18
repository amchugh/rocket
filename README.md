


# Machine Learning Assisted Algorithms | Aidan McHugh

## Summary

I sought to write an algorithm to stabilize a 2d drone. The drone has two thrusters, a left and a right, that it can control independently. Through this, it can control its linear and rotational acceleration. The goal is to reduce linear and rotational velocity to near-zero (accepting some tolerance of error) in the minimum time possible.

## Simulation

The rocket can produce “downward” facing forces of up to 3.2N from either engine independently of the other. The rocket has a mass of 1 kg and a rotational inertia of 0.6 kg*m^2. Gravity is -1m/s^2. The maximum upward vertical acceleration of the rocket is 5.4m/s^2. The rocket is given a random velocity between [-10, 10] m/s, a random rotational velocity between [-π,π], and a random rotation between [0, 2π]. The simulation is 400x400 wide and is updated at a rate of 30x a second.

In rendering of the simulation, the legs of the rocket will turn more red based on the percentage they are being used ( red=100%, blue=0% ).

## Algorithm

I theorized, then hand wrote, this algorithm to stabilize the rocket. It is a three step process that is called 30x a frame:
1. Slow rotational velocity to an acceptable amount
2. Rotate so we are facing away from our direction of travel
3. So long as we are still facing the right direction, fire our engines to decrease our speed until we hit a winning threshold.

The code is in python, and should be understandable with some explanation. atan2(y,x) is a function that supplies tan-1(y/x).  shortestTurn(cur, tar) supplies the directional difference between two angles in domain [0, 2π).

Here is the code. This is taken from `python/rocketenv.py` under `RocketController.step`

```python
def step(self, vx, vy, omega, theta):
# Calculate the target angle
target = atan2(vy, vx) + 3*pi/2
if target > 2*pi:
  target -= 2*pi

# Find the difference between the target
# angle and our current angle
delta = shortestTurn(theta, target)
# Find our speed (magnitude of velocity)
mag = dist(vx, vy)

# If we are pointed in the right direction,
# and moving by an acceptable speed, kill velocity
if abs(delta) <= self.theta_tolerance and mag > self.mag_tolerance:
    return mag*self.vel_kill_scalar, mag*self.vel_kill_scalar
  
# Otherwise, we need to rotate to face the correct direction
else:
    # If we are moving too fast, our error is our rotational speed
    if abs(omega) > self.safe_turn_speed: 
        e = omega
    # If we are moving slow enough, make sure we are rotating in the
    # correct direction
    else: 
        e = omega - (self.safe_turn_speed * (-1 if delta < 0 else 1) )
      
    # This is a PID controller. Used when you directly
    # control the derivative of your target metric.
    # We are using the PID controller to correct our rotational
    # velocity.
    self.I = self.I + e*self.dt
    proportional = e*self.proportional_scalar
    derivative = (e-self.e_prev)/self.dt*self.derivative_scalar
    mv = (proportional + self.I + derivative)
    
    self.e_prev = e
    
    # Note that only one engine will fire here.
    return -mv*self.rot_scalar, mv*self.rot_scalar
```

## Algorithm Constants

You might have noticed a number of “self.something” constants in the algorithm. They are as follows:
- safe_turn_speed
- vel_kill_scalar
- theta_tolerance
- rot_scalar
- proportional_scalar
- mag_tolerance
- derivative_scalar

The algorithm *in theory* works, but what really powers it is the correct assortment of magical constants.

### Initial Algorithm Constants

I hard coded a few constants that I found through some brief trial/error. They are as follows:
- safe_turn_speed = 1 
- vel_kill_scalar = 10
- theta_tolerance = 0.01
- rot_scalar = 0.01
- proportional_scalar = 1
- mag_tolerance = 0.1
- derivative_scalar = 1

The video below shows a few random scenarios with these constants.
You can see that sometimes it almost works.

https://user-images.githubusercontent.com/12225625/118583902-af36d000-b74a-11eb-82fa-a62e58c135b9.mp4

## Genetic Algorithm

I wanted to find better constants for my algorithm. I employed a genetic algorithm to do so. Genetic Algorithms are based off the process of natural selection. We create a generation with, say, 100 members. The “DNA” of these members is the values it has for each of the constants. We go through every member and evaluate them using a “fitness function”. Here, our fitness function is the inverse of the number of steps it takes for the rocket to stabilize itself. We then use this fitness function to create a new generation at random. We create a new generation with 100 members, randomly selecting from the previous generation based off their fitness. Then, we mutate and crossover members based off the inverse fitness. We then run the algorithm for a couple hundred generations until we are satisfied with the average member’s performance.

Here is some pseudocode for such an algorithm. My implementation follows a similar structure.

```python
# Genetic Algorithm Constants
GENERATION_SIZE = 100
CROSSOVER_CHANCE = 0.2

# Create our first generation
generation = []
for i in range(GENERATION_SIZE):
    generation.append(new Member())

while True:
    avg_fitness = 0
    
    # Evaluate our members
    for m in generation:
        m.fitness = evaluate(m)
        avg_fitness += m.fitness
    
    avg_fitness /= GENERATION_SIZE
        
    new_gen = copy(generation)

    # Mutate and crossover the members
    for m in new_gen:
        # Crossover at random
        if random() < CROSSOVER_CHANCE:
            # If we decided to crossover,
            # pick a target from the previous
            # generation.
            target = randomSelection(generation)
            # Iterate over every constant
            # and choose to take crossover
            for index, c in enumerate(m.constants):
                if 0.5 < random():
                    c = target.constants[index]
    
        # Iterate over every trainable parameter
        # and nudge it (we nudge by a smaller amount
        # the more fit the member is)
        for c in m.constants:
            c += (random() - 0.5) * 2 / m.fitness
            
    # Swap the generations
    generation = new_gen
```

### Genetic Algorithm Implementation

I originally implemented this algorithm in python on my local machine. It did not take long to write -- maybe an hour. I started it and walked away, only to come back and realize this was going to take over twelve hours to run 100 generations. My computer has a i9-10900K CPU; it's powerful. This long execution time was not acceptable to me, as I predicted needing around 500 generations and was not willing to wait several days. I rewrote all the code in C++, and then optimized it for execution speed. I got it down to ~600 milliseconds per generation, meaning I could do all 500 generations in around 5 minutes. A significant improvement came from multithreading, but most the improvement came from aggressive compiler optimization and the inherent low-level coding that C++ provides.

## New Algorithm Constants

My genetic algorithm found the following constants:
- safe_turn_speed = 1.30180042
- vel_kill_scalar = 5.07822616
- theta_tolerance = 0.00407172
- rot_scalar = 0.09638811
- proportional_scalar = 4.64927884
- mag_tolerance = 0.22577127
- derivative_scalar = 0.62695137

The video below shows the same random scenarios with these new constants.
You can see how much better it is than the previous set of constants.

https://user-images.githubusercontent.com/12225625/118583956-beb61900-b74a-11eb-9b72-29781c068fa1.mp4

## Conclusion
    
After creating an algorithm by hand, I used a genetic algorithm to find the best possible constants for use with my solution. Genetic Algorithms are a form of artificial evolution best used for optimization of an existing strategy. They were utilized to great effect here and resulted in a 250% increase in fitness over 500 generations (a relatively small amount).
