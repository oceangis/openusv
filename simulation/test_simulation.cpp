/*
   USV Sailboat Simulation Test Program

   Compile:
   g++ -std=c++11 -o test_sim test_simulation.cpp SIM_Sailboat_USV.cpp -lm

   Run:
   ./test_sim
*/

#include "SIM_Sailboat_USV.h"
#include <cstdio>
#include <cmath>

using namespace SITL_USV;

// Test helper: print state summary
void print_state(const SimState& s, float time_s) {
    printf("t=%.1fs | Pos:(%.4f, %.4f) | Speed:%.2f m/s | Heading:%.1f deg | "
           "VMG:%.2f m/s | Heel:%.1f deg | AppWind:%.1f m/s @ %.1f deg\n",
           time_s,
           s.lat_deg * 111320.0,  // Convert to meters
           s.lon_deg * 111320.0,
           s.velocity_bf.x,
           s.yaw_rad * 180.0f / M_PI,
           s.vmg,
           s.heel_angle_deg,
           s.apparent_wind_speed_ms,
           s.apparent_wind_dir_deg);
}

// Test 1: Basic sailing in calm conditions
void test_calm_sailing() {
    printf("\n=== Test 1: Calm Sea Sailing ===\n");

    SailboatUSV sim;
    sim.init(WingsailType::WINGSAIL_FLAP);
    sim.set_beaufort(BeaufortScale::GENTLE_BREEZE);
    sim.set_sea_state(SeaState::SMOOTH);

    // Set wind from North, heading East (beam reach)
    EnvironmentParams env;
    env.wind_speed_ms = 5.0f;
    env.wind_direction_deg = 0.0f;
    env.wave_enable = false;
    env.tide_speed_ms = 0.0f;
    sim.set_environment(env);

    // Initial heading East
    SimState initial = sim.get_state();
    // Modify yaw through simulation by steering

    float dt = 0.02f;  // 50 Hz
    float total_time = 60.0f;  // 60 seconds

    printf("Simulating beam reach (wind from North, heading East)...\n");

    for (float t = 0; t < total_time; t += dt) {
        // Constant flap setting for beam reach
        sim.set_rudder_input(0.0f);
        sim.set_wingsail_input(0.6f);  // Positive flap

        sim.update(dt);

        if (fmod(t, 10.0f) < dt) {
            print_state(sim.get_state(), t);
        }
    }

    printf("Final speed: %.2f m/s\n", sim.get_state().velocity_bf.x);
}

// Test 2: Upwind tacking simulation
void test_upwind_tacking() {
    printf("\n=== Test 2: Upwind Tacking ===\n");

    SailboatUSV sim;
    sim.init(WingsailType::WINGSAIL_FLAP);
    sim.set_scenario_upwind();

    float dt = 0.02f;
    float total_time = 120.0f;
    float tack_interval = 30.0f;
    float last_tack = 0.0f;
    bool tack_port = true;

    printf("Simulating upwind with tacking every %.0f seconds...\n", tack_interval);

    for (float t = 0; t < total_time; t += dt) {
        // Tack every interval
        if (t - last_tack > tack_interval) {
            tack_port = !tack_port;
            last_tack = t;
            printf(">>> TACK to %s at t=%.1fs\n", tack_port ? "PORT" : "STARBOARD", t);
        }

        // Steering for tack
        float target_rudder = tack_port ? -0.5f : 0.5f;
        sim.set_rudder_input(target_rudder * 0.3f);  // Gradual steering

        // Flap follows tack side
        sim.set_wingsail_input(tack_port ? -0.7f : 0.7f);

        sim.update(dt);

        if (fmod(t, 15.0f) < dt) {
            print_state(sim.get_state(), t);
        }
    }

    const SimState& final_state = sim.get_state();
    printf("Final position: %.1f m North\n", final_state.lat_deg * 111320.0);
}

// Test 3: Storm conditions
void test_storm_survival() {
    printf("\n=== Test 3: Storm Survival ===\n");

    SailboatUSV sim;
    sim.init(WingsailType::WINGSAIL_FREE);  // Self-balancing for storm
    sim.set_scenario_storm();

    float dt = 0.02f;
    float total_time = 60.0f;
    float max_heel = 0.0f;

    printf("Simulating storm conditions with self-balancing wingsail...\n");

    for (float t = 0; t < total_time; t += dt) {
        // Just try to maintain heading
        sim.set_rudder_input(0.1f);
        sim.set_wingsail_input(0.0f);  // FREE mode ignores this

        sim.update(dt);

        float heel = fabsf(sim.get_state().heel_angle_deg);
        if (heel > max_heel) max_heel = heel;

        if (fmod(t, 10.0f) < dt) {
            print_state(sim.get_state(), t);
        }
    }

    printf("Maximum heel angle: %.1f degrees\n", max_heel);
    printf("Survival status: %s\n", max_heel < 45.0f ? "PASSED" : "CAPSIZED!");
}

// Test 4: Compare wingsail types
void test_wingsail_comparison() {
    printf("\n=== Test 4: Wingsail Type Comparison ===\n");

    const char* type_names[] = {"ROTATION", "FLAP", "FREE"};
    float speeds[3] = {0};

    for (int type = 0; type < 3; type++) {
        SailboatUSV sim;
        sim.init(static_cast<WingsailType>(type));
        sim.set_beaufort(BeaufortScale::FRESH_BREEZE);
        sim.set_sea_state(SeaState::SLIGHT);

        // Beam reach conditions
        EnvironmentParams env;
        env.wind_speed_ms = 8.0f;
        env.wind_direction_deg = 0.0f;
        env.wave_enable = true;
        env.wave_height_m = 0.5f;
        env.wave_period_s = 5.0f;
        env.wave_direction_deg = 0.0f;
        sim.set_environment(env);

        float dt = 0.02f;
        float warmup_time = 30.0f;

        // Let it reach steady state
        for (float t = 0; t < warmup_time; t += dt) {
            sim.set_rudder_input(0.0f);
            sim.set_wingsail_input(0.6f);
            sim.update(dt);
        }

        speeds[type] = sim.get_state().velocity_bf.x;
    }

    printf("\nBeam reach performance comparison (Beaufort 5):\n");
    printf("----------------------------------------\n");
    for (int i = 0; i < 3; i++) {
        printf("%-10s: %.2f m/s (%.1f knots)\n",
               type_names[i], speeds[i], speeds[i] * 1.944f);
    }
}

int main() {
    printf("========================================\n");
    printf("   USV Sailboat Simulation Tests\n");
    printf("========================================\n");

    test_calm_sailing();
    test_upwind_tacking();
    test_storm_survival();
    test_wingsail_comparison();

    printf("\n========================================\n");
    printf("   All tests completed!\n");
    printf("========================================\n");

    return 0;
}
