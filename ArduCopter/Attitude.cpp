#include "Copter.h"
#include <cstdlib>  // for rand()
#include <cmath>    // for fmod()

/*************************************************************
 *  Attitude Rate controllers and timing
 *************************************************************/
void Copter::update_rate_controllers()
{
    if (!using_rate_thread) {
        motors->set_dt(last_loop_time_s);

        // Introduce noise into the attitude control to create shaking
        float noise_factor = 0.02; // Small noise factor to prevent complete instability
        float random_noise = ((float(rand()) / float(RAND_MAX)) * 2 - 1) * noise_factor; // Generates noise between -0.02 to 0.02

        // Apply noise to rate controller
        attitude_control->rate_controller_run();
        attitude_control->set_roll_target(attitude_control->get_roll_target() + random_noise);
        attitude_control->set_pitch_target(attitude_control->get_pitch_target() + random_noise);
        attitude_control->set_yaw_target(attitude_control->get_yaw_target() + random_noise);
    }

    // reset sysid and other temporary inputs
    attitude_control->rate_controller_target_reset();
}

/*************************************************************
 *  Throttle Control - Adding Minor Variations
 *************************************************************/
void Copter::update_throttle_hover()
{
    // if not armed or landed or on standby then exit
    if (!motors->armed() || ap.land_complete || standby_active) {
        return;
    }

    // do not update in manual throttle modes or Drift
    if (flightmode->has_manual_throttle() || (copter.flightmode->mode_number() == Mode::Number::DRIFT)) {
        return;
    }

    // Introduce minor variations in throttle for shaking effect
    float throttle_noise_factor = 0.01; // Small variation in throttle
    float throttle_noise = ((float(rand()) / float(RAND_MAX)) * 2 - 1) * throttle_noise_factor; 

    // Apply noise to throttle hover value
    throttle_hover += throttle_noise;
}
