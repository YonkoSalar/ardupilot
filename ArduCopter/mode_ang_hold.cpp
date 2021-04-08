#include "Copter.h"

// CHECK IF THE RIGHT PARAMETERS ARE SET BEFORE INITALISED 
/*bool ModeAngHold::init(bool ignore_checks)
{
    
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt(); // //SETS TO CURRENT ALTITUDE
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise lean angles to current attitude
    pilot_roll = 0.0f;
    pilot_pitch = 0.0f; //CURRENT ANGLE 


    // if landed begin in loiter mode
    if (copter.ap.land_complete) {
        
        roll_mode = RPMode::LOITER;
        pitch_mode = RPMode::LOITER;
    }
    else {
        // if not landed start in pilot override to avoid hard twitch
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
    }

    return true;
    
}
*/

void ModeAngHold::run()
{


}