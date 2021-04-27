#include "Copter.h"

// CHECK IF THE RIGHT PARAMETERS ARE SET BEFORE INITALISED
bool ModeAngHold::init(bool ignore_checks)
{
    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z())
    {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise lean angles to current attitude
    pilot_roll = 0.0f;
    pilot_pitch = 0.0f;

    // compute brake_gain
    brake_gain = (15.0f * (float)g.poshold_brake_rate + 95.0f) / 100.0f;

    // initialise loiter
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    return true;
}

void ModeAngHold::run()
{
    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);
    loiter_nav->clear_pilot_desired_acceleration();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate (for alt-hold mode and take-off)
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // run loiter controller
    loiter_nav->update();

    // set roll angle based on loiter controller outputs
    float roll = loiter_nav->get_roll();
    float pitch = loiter_nav->get_pitch();

    // constrain target pitch/roll angles
    float angle_max = copter.aparm.angle_max;
    roll = constrain_float(roll, -angle_max, angle_max);
    pitch = constrain_float(pitch, -angle_max, angle_max);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll, pitch, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();
}