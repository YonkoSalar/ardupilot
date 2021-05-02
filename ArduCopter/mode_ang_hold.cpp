#include "Copter.h"
#include "UserVariables.h"

// CHECK IF THE RIGHT PARAMETERS ARE SET BEFORE INITALISED
bool ModeAngHold::init(bool ignore_checks)
{
   // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt(); 
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z()); 
    }

    // only allow flip from Stabilize, AltHold, PosHold or Loiter modes
    if (copter.control_mode != Mode::Number::STABILIZE &&
        copter.control_mode != Mode::Number::ALT_HOLD &&
        copter.control_mode != Mode::Number::LOITER &&
        copter.control_mode != Mode::Number::POSHOLD) {
        return false;
    }

    // ensure throttle is above zero
    if (copter.ap.throttle_zero) {
        return false;
    }

    // ensure we are flying
    if (!copter.motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        return false;
    }

    return true;
}

void ModeAngHold::run()
{
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {
        case AltHold_MotorStopped:
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
            break;

        case AltHold_Landed_Ground_Idle:
            attitude_control->set_yaw_target_to_current_heading();
            FALLTHROUGH;

        case AltHold_Landed_Pre_Takeoff:
            attitude_control->reset_rate_controller_I_terms_smoothly();
            pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
            break;

        case AltHold_Takeoff:
            // initiate take-off
            if (!takeoff.running()) {
                takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            }

            // get take-off adjusted pilot and takeoff climb rates
            takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

            // set position controller targets
            pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
            pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
            break;

        case AltHold_Flying:
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

            pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);

            // add pitch offset from controller
            target_pitch = target_pitch - RC_aoa;

            break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();
}