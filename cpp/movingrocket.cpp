#include <iostream>
#include <algorithm>
#include <fstream>
#include <random>
#include <chrono>
#include <thread>
#include <iomanip>
#include <thread>

#define PI 3.141592653589793
#define HALFPI 1.5707963267948966
#define TAU 6.283185307179586

double shortestTurn(double current, double target) {
    double delt = target - current;
    if (delt > PI) delt = -TAU + delt;
    else if (delt < -PI) delt = TAU + delt;
    return delt;
}

struct RocketController {
    double safe_turn_speed;
    double dist_scalar;
    double theta_tolerance;
    double rot_scalar;
    double proportional_scalar;
    double moving_scalar;
    double derivative_scalar;
    double fitness = 0;
};

//const RocketController BEST = { 1.29942213, 5.04038849, 0.05116001, 0.04144201, 4.54285239, 0.19883754 };
//const RocketController BEST = { 1.29955965, 5.05709521, 0.05169962, 0.05577526, 4.57433680, 0.20266274 };
//const RocketController BEST = { 1.30303623, 5.07266505, 0.04855293, 0.07375364, 4.59165082, 0.21728768 };
//const RocketController BEST = { 1.30324693, 5.07446025, 0.04875870, 0.07423064, 4.59645008, 0.21727312 };
//const RocketController BEST = { 1.30384703, 5.08173363, 0.00137470, 0.07474198, 4.60208684, 0.22406336, 0.89 };
// Some old iterations lol. Some of them use different simulation constants, and some existed before I
// added the derivative scalar from the PID controller as a trainable parameter.

const RocketController BEST = { 1.30180042, 5.07822616, 0.00407172, 0.09638811, 4.64927884, 0.22577127, 0.62695137 };

std::ostream& operator << (std::ostream& o, const RocketController& rc) {
    o << std::fixed << std::setprecision(8) << rc.safe_turn_speed << ", " <<
        rc.dist_scalar << ", " <<
        rc.theta_tolerance << ", " <<
        rc.rot_scalar << ", " <<
        rc.proportional_scalar << ", " <<
        rc.moving_scalar << ", " <<
        rc.derivative_scalar << " FITNESS: " <<
        rc.fitness;
    return o;
}

struct desiredThrust {
    double thrustLeft;
    double thrustRight;
    double I;
    double error;
};

/*
desiredThrust getThrust(RocketController* rc, double dt, double vx, double vy, double omega, double theta, double I, double prev_error) {
    auto thrust = desiredThrust();
    double target = atan2(vy, vx) + 3. * PI / 2.;
    target = target - (target >= TAU ? TAU : 0);
    double delta = shortestTurn(theta, target);
    double mag = sqrt(vx * vx + vy * vy);
    if (fabs(delta) <= rc->theta_tolerance) {
        thrust.I = I;
        thrust.error = prev_error;
        thrust.thrustLeft = mag * rc->vel_kill_scalar;
        thrust.thrustRight = thrust.thrustLeft;
    }
    else {
        if (fabs(omega) > rc->safe_turn_speed) thrust.error = omega;
        else thrust.error = omega - (delta < 0 ? -(rc->safe_turn_speed) : rc->safe_turn_speed);
        thrust.I = I + dt * thrust.error;
        thrust.thrustRight = rc->rot_scalar * (thrust.error * rc->proportional_scalar + thrust.I + (thrust.error - prev_error) / dt * rc->derivative_scalar);
        thrust.thrustLeft = -thrust.thrustRight;
    }
    return thrust;
}
*/

desiredThrust getThrust(RocketController* rc, double dt, double x, double y, double vx, double vy, double omega, double theta, double I, double prev_error) {
    auto thrust = desiredThrust{};

    double dx = -x;
    double dy = -y;
    // Normalize the <dx, dy> according to the dist_scalar
    double mg = sqrt(dx * dx + dy * dy);
    double ndx = dx / mg * rc->dist_scalar;
    double ndy = dy / mg * rc->dist_scalar;
    // Find the needed change to make the <vx, vy> into the <dx, dy> scaled vector
    double dvx = ndx - vx;
    double dvy = ndy - vy;
    // Find the angle of this dv vector
    double targetAdj = atan2(dvy, dvx);
    targetAdj += HALFPI;
    targetAdj = fmod(targetAdj, TAU);
    double delta = shortestTurn(theta, targetAdj);

    if (fabs(delta) > rc->theta_tolerance) {
        if (fabs(omega) > rc->safe_turn_speed) thrust.error = omega;
        else thrust.error = omega - (delta < 0 ? -(rc->safe_turn_speed) : rc->safe_turn_speed);
        thrust.I = I + dt * thrust.error;
        thrust.thrustRight = rc->rot_scalar * (thrust.error * rc->proportional_scalar + thrust.I + (thrust.error - prev_error) / dt * rc->derivative_scalar);
        thrust.thrustLeft = -thrust.thrustRight;
    }
    else {
        double d = dx*dx*4 + pow(dy - y, 2);
        thrust.thrustRight = rc->moving_scalar * d;
        thrust.thrustLeft = thrust.thrustRight;
    }

    return thrust;
}

const int STEPS_PER_SECOND = 30;
const int SIM_SECONDS = 130;
const int MAX_STEPS = SIM_SECONDS * STEPS_PER_SECOND;
const double WORLD_SIZE[] = { 400, 400 };
const double MAX_THRUST = 3.2;
const double GRAVITY = 1.;
const double ROCKET_MASS = 1.;
const double ROCKET_INERTIA = 0.6;
const double VX_TOLERANCE = 0.1;
const double VY_TOLERANCE = 0.3;
const double THETA_TOLERANCE = 0.1;
const double NEEDED_STEPS = 20;
const double MAX_DIST = sqrt(pow(WORLD_SIZE[0], 2) + pow(WORLD_SIZE[1], 2));
// Returns the average distance from <0,0>
double runEnv(RocketController* rc, double seed, double dt) {
    // Create the random number generator
    std::mt19937 gen(seed);
    const std::uniform_real_distribution<double> veldist(-10., 10.);
    const std::uniform_real_distribution<double> thetadist(0., TAU);
    const std::uniform_real_distribution<double> omegadist(-PI, PI);
    const std::uniform_real_distribution<double> posdist(-100, 100);
    // Set up the simulation
    // todo::Figure out a way of setting the start point
    double x = posdist(gen);
	double y = posdist(gen);
    // todo::Should these be zero?
    /*
    double theta = thetadist(gen);
    double vx = veldist(gen);
    double vy = veldist(gen);
    double omega = omegadist(gen);
    */
    double theta = 0, vx = 0, vy = 0, omega = 0;

    double I = 0;
    double prev_error = 0;
    int stable_steps = 0;

    double avgX = 0;
    double avgY = 0;

    for (int i = 1; i <= MAX_STEPS; i++) {
        auto thrust = getThrust(rc, dt, x, y, vx, vy, omega, theta, I, prev_error);

        double f1 = std::max(0., std::min(thrust.thrustLeft, MAX_THRUST));
        double f2 = std::max(0., std::min(thrust.thrustRight, MAX_THRUST));

        // Euler Integration for physics!
        double ay = GRAVITY - (f1 + f2) * cos(theta) / ROCKET_MASS;
        double ax = (f1 + f2) * sin(theta) / ROCKET_MASS;
        double alpha = (f1 - f2) / ROCKET_INERTIA;
        
        vy += ay * dt;
        vx += ax * dt;
        omega += alpha * dt;

        y += vy * dt;
        x += vx * dt;
        theta += omega * dt;
        theta = fmod(theta + TAU, TAU);

        avgX += x;
        avgY += y;

        // Test to see if the rocket has left the bounds
        if (x < -WORLD_SIZE[0] || x > WORLD_SIZE[0] || y < -WORLD_SIZE[1] || y > WORLD_SIZE[1]) {
            return MAX_DIST;
        }

        I = thrust.I;
        prev_error = thrust.error;
    }

    // Return the average distance length
    avgX /= MAX_STEPS;
    avgY /= MAX_STEPS;
    return sqrt(avgX * avgX + avgY * avgY);
}

const int SCENARIOS = 100;
// Evaluates a RocketController over many simulations, averages the fitness
// and stores the value in the RocketController.
void evaluate_fitness(RocketController* rc, double dt) {
    rc->fitness = 0;
    for (int i = 0; i < SCENARIOS; i++) {
        rc->fitness += runEnv(rc, i, dt);
    }
    rc->fitness /= SCENARIOS;
}

// Classically, these are refered to as hyperparameters.
// Hyperparameters:
const std::uniform_real_distribution<double> medium_tweak(-.01, .01);
const std::uniform_real_distribution<double> small_tweak(-.005, .005);
const std::uniform_real_distribution<double> tiny_tweak(-.0005, .0005);
const double TWEAK_SCALAR = 5.;

// Slightly changes all of the RocketController's properties
void mutate_controller(RocketController* rc, double amount, std::mt19937& gen) {
    rc->dist_scalar += small_tweak(gen) * amount * TWEAK_SCALAR;
    rc->proportional_scalar += small_tweak(gen) * amount * TWEAK_SCALAR;
    rc->theta_tolerance += tiny_tweak(gen) * amount * TWEAK_SCALAR;
    rc->rot_scalar += tiny_tweak(gen) * amount * TWEAK_SCALAR;
    rc->safe_turn_speed += tiny_tweak(gen) * amount * TWEAK_SCALAR;
    rc->moving_scalar += small_tweak(gen) * amount * TWEAK_SCALAR;
    rc->derivative_scalar += medium_tweak(gen) * amount * TWEAK_SCALAR;
}

const std::uniform_real_distribution<double> chance(0., 1.);
// Randomly swaps properties with another RocketController's.
void crossover_controller(RocketController* target, RocketController* other, std::mt19937& gen) {
    if (chance(gen) > 0.5) target->dist_scalar = other->dist_scalar;
    if (chance(gen) > 0.5) target->proportional_scalar = other->proportional_scalar;
    if (chance(gen) > 0.5) target->rot_scalar = other->rot_scalar;
    if (chance(gen) > 0.5) target->safe_turn_speed = other->safe_turn_speed;
    if (chance(gen) > 0.5) target->theta_tolerance = other->theta_tolerance;
    if (chance(gen) > 0.5) target->moving_scalar = other->moving_scalar;
    if (chance(gen) > 0.5) target->derivative_scalar = other->derivative_scalar;
}

const double XRANGE = 600.;
double scaleFitness(double fitness) {
    double adj = fitness / XRANGE;
    return adj * adj;
}

const int GEN_SIZE = 200;
const std::uniform_int_distribution<int> selector(0, GEN_SIZE);
const double CHANCE_SCALAR = 0.0004;
// We construct a new generation of GEN_SIZE.
// Index 0 will be the best preforming from the 
// previous generation. There will be a number of rocket controllers
// which are simply mutated over, and the remainder are random crossovers.
void next_gen(RocketController* next, RocketController* prev, std::mt19937& gen) {
    int curr = 1;
    int best_index = -1;
    int best_fitness = MAX_STEPS;
    // Randomly mutate some (also find the best)
    for (int i = 0; i < GEN_SIZE; i++) {
        // See if this controller has the best fitness
        if (prev[i].fitness <= best_fitness) {
            best_index = i;
            best_fitness = prev[i].fitness;
        }
		// Mutate this controller
		next[curr] = RocketController(prev[i]);
		next[curr].fitness = 0;
		mutate_controller(&next[curr], scaleFitness(prev[i].fitness), gen);
		curr++;
    }
    // Set the zeroth to the best performing
    next[0] = RocketController(prev[best_index]);
    // I also want to print the values for this
    std::cout << "Best in generation: " << next[0] << std::endl;
    // Fill the rest with crossovers
    for (; curr < GEN_SIZE; curr++) {
        // Clone a random controller
        next[curr] = RocketController(prev[selector(gen)]);
        crossover_controller(&next[curr], &prev[best_index], gen);
        next[curr].fitness = 0;
        //
        // I think it was at this point that I realized the problem
        // was best solved with simple linear regression.
        // 
        // Oh well. 
        //
        // I had a good time doing this at least.
        //
    }
}

void first_gen(RocketController* generation, std::mt19937& gen) {
    // We will create a bunch based off the values we already have
    generation[0] = BEST;
    for (auto i = 1; i < GEN_SIZE; i++) {
        generation[i] = BEST;
        mutate_controller(&generation[i], 1., gen);
    }
}

// Tools for evaluating an entire generation using multiple threads.
// This is thread safe... but don't change anything. lol.
void evaluate_batch(RocketController* batch, int offset, int batch_size, double dt) {
    for (auto i = 0; i < batch_size; i++) {
        evaluate_fitness(&batch[offset + i], dt);
    }
}
void evaluate_gen(RocketController* generation, double dt) {
    // todo:: Maybe it's more appropriate to use a threadpool here?
    // I couldn't be bothered to create my own Job system. This works
    // fine for my purposes.

    // We can use threading here
    const auto processor_count = std::thread::hardware_concurrency() - 1; // One thread we use for leftover
    const auto batch_size = GEN_SIZE / processor_count;

    // Create a container for all the threads we are about to spawn.
    // This might be dynamically allocated? If so,
    // it's the only dynamic allocation in the entire program.
    // Everything else is in-place or on the stack.
    std::vector<std::thread> threads;

    for (auto i = 0; i < processor_count; i++) {
        threads.emplace_back(std::thread(evaluate_batch, generation, batch_size * i, batch_size, dt));
    }

    // One more batch with the remainder
    auto leftover = GEN_SIZE - (batch_size * processor_count);
    threads.emplace_back(std::thread(evaluate_batch, generation, batch_size * processor_count, leftover, dt));

    // Wait for threads to finish
    for (auto& th : threads) {
        th.join();
    }
}

int main()
{
    using namespace std::chrono;

    const double frameTime = 1.0 / STEPS_PER_SECOND;

    RocketController g1[GEN_SIZE];
	RocketController g2[GEN_SIZE];
    std::mt19937 gen;

    auto t1 = high_resolution_clock::now();
    first_gen(g1, gen);
    auto t2 = high_resolution_clock::now();
    duration<double> dt = duration_cast<duration<double>>(t2 - t1);
    printf("Made first generation in %f seconds\n", dt.count());

    for (auto g = 0; g < 1000; g++) {
		t1 = high_resolution_clock::now();
		evaluate_gen(g1, frameTime);
		t2 = high_resolution_clock::now();
		dt = duration_cast<duration<double>>(t2 - t1);
		printf("Evaluated g1 in %f seconds\n", dt.count());

		t1 = high_resolution_clock::now();
		next_gen(g2, g1, gen);
		t2 = high_resolution_clock::now();
		dt = duration_cast<duration<double>>(t2 - t1);
		printf("Made next generation in %f seconds\n", dt.count());

        // Now do the same, but swap the roles of g1 and g2.
        // We do two generations per loop of 'g'.

		t1 = high_resolution_clock::now();
		evaluate_gen(g2, frameTime);
		t2 = high_resolution_clock::now();
		dt = duration_cast<duration<double>>(t2 - t1);
		printf("Evaluated g2 in %f seconds\n", dt.count());

		t1 = high_resolution_clock::now();
		next_gen(g1, g2, gen);
		t2 = high_resolution_clock::now();
		dt = duration_cast<duration<double>>(t2 - t1);
		printf("Made next generation in %f seconds\n", dt.count());
    }
}
