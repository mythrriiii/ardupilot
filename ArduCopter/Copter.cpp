#include "Copter.h"
#include <cstdlib> // For rand()

/*************************************************************
 *  Attitude Rate controllers and timing
 ****************************************************************/

/*
  update rate controller when run from main thread (normal operation)
*/
void Copter::run_rate_controller_main()
{
    // set attitude and position controller loop time
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    pos_control->set_dt(last_loop_time_s);
    attitude_control->set_dt(last_loop_time_s);

    if (!using_rate_thread) {
        motors->set_dt(last_loop_time_s);
        // only run the rate controller if we are not using the rate thread
        attitude_control->rate_controller_run();
    }
    // reset sysid and other temporary inputs
    attitude_control->rate_controller_target_reset();

    // Inject gyro noise to simulate erratic behavior
    inject_gyro_noise();
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update estimated throttle required to hover (if necessary)
//  called at 100hz
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

    // do not update while climbing or descending
    if (!is_zero(pos_control->get_vel_desired_cms().z)) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();

    // Introduce random fluctuations in throttle (erratic altitude control)
    float throttle_noise = (rand() % 10 - 5) * 0.01f;  // Random noise (-0.05 to +0.05)
    throttle += throttle_noise;
    throttle = constrain_float(throttle, 0.0f, 1.0f);
    motors->set_throttle(throttle);

    // calc average throttle if we are in a level hover.  accounts for heli hover roll trim
    if (throttle > 0.0f && fabsf(inertial_nav.get_velocity_z_up_cms()) < 60 &&
        fabsf(ahrs.roll_sensor-attitude_control->get_roll_trim_cd()) < 500 && labs(ahrs.pitch_sensor) < 500) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
#if HAL_GYROFFT_ENABLED
        gyro_fft.update_freq_hover(0.01f, motors->get_throttle_out());
#endif
    }
}

// Function to inject artificial noise into gyro readings
void Copter::inject_gyro_noise()
{
    float gyro_noise = (rand() % 20 - 10) * 0.01f;  // Random noise (-0.1 to +0.1)
    ahrs.gyro.x += gyro_noise;
    ahrs.gyro.y += gyro_noise;
    ahrs.gyro.z += gyro_noise;
}

// Modify rate controller to introduce oscillations
void Copter::modify_rate_controller_for_erratic_behavior()
{
    attitude_control->get_rate_roll_pid().kP() *= 2.5;  // Increase roll P-gain (more oscillations)
    attitude_control->get_rate_pitch_pid().kP() *= 2.5; // Increase pitch P-gain
    attitude_control->get_rate_yaw_pid().kP() *= 2.5;   // Increase yaw P-gain

    attitude_control->get_rate_roll_pid().kD() *= 0.3;  // Reduce damping (D-gain)
    attitude_control->get_rate_pitch_pid().kD() *= 0.3;
    attitude_control->get_rate_yaw_pid().kD() *= 0.3;
}

// Call the erratic modifications in the main rate controller loop
void Copter::run_rate_controller_main_with_erratic_behavior()
{
    run_rate_controller_main();
    modify_rate_controller_for_erratic_behavior();
}
