#include "Copter.h"
#include <cstdlib>  // for rand()
#include <cmath>    // for fmod()

/*************************************************************
 *  Attitude Rate controllers and timing
 *************************************************************/
void Copter::run_rate_controller_main()
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



// get_pilot_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float Copter::get_pilot_desired_climb_rate(float throttle_control)
{
    // throttle failsafe check
    if (failsafe.radio || !rc().has_ever_seen_rc_input()) {
        return 0.0f;
    }
 
#if TOY_MODE_ENABLED
    if (g2.toy_mode.enabled()) {
        // allow throttle to be reduced after throttle arming and for
        // slower descent close to the ground
        g2.toy_mode.throttle_adjust(throttle_control);
    }
#endif
 
    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,0.0f,1000.0f);
 
    // ensure a reasonable deadzone
    g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));
 
    float desired_rate = 0.0f;
    const float mid_stick = get_throttle_mid();
    const float deadband_top = mid_stick + g.throttle_deadzone;
    const float deadband_bottom = mid_stick - g.throttle_deadzone;
 
    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate = get_pilot_speed_dn() * (throttle_control-deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate = g.pilot_speed_up * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    } else {
        // must be in the deadband
        desired_rate = 0.0f;
    }
 
    return desired_rate;
}
 
// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors->get_throttle_hover()/2.0f);
}
 
// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void Copter::set_accel_throttle_I_from_pilot_throttle()
{
    // get last throttle input sent to attitude controller
    float pilot_throttle = constrain_float(attitude_control->get_throttle_in(), 0.0f, 1.0f);
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    pos_control->get_accel_z_pid().set_integrator((pilot_throttle-motors->get_throttle_hover()) * 1000.0f);
}
 
// rotate vector from vehicle's perspective to North-East frame
void Copter::rotate_body_frame_to_NE(float &x, float &y)
{
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();
    x = ne_x;
    y = ne_y;
}
 
// It will return the PILOT_SPEED_DN value if non zero, otherwise if zero it returns the PILOT_SPEED_UP value.
uint16_t Copter::get_pilot_speed_dn() const
{
    if (g2.pilot_speed_dn == 0) {
        return abs(g.pilot_speed_up);
    } else {
        return abs(g2.pilot_speed_dn);
    }
}
