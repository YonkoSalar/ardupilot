#include "Copter.h"
#include "UserVariables.h"
#include "../libraries/AP_AHRS/AP_AHRS.h"
#include <math.h>
#include <GCS_MAVLink/GCS.h>

#include "../../ArduCopter/UserVariables.h" //ADDED USER VARIABLES (RC_Pich_Offset)


#include "../libraries/AP_NavEKF3/AP_NavEKF3_core.h" //ADDED EKF3 CORE

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    //Log_Write_Icarus();
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif


#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    
    //0 degress -> val: 0.575 -> RC_pitch_offset: 51.75
    //-90 degrees -> val: 0 -> RC_pitch_offset: 0
    //90 degrees -> val: 1 -> RC_pitch_offset: 90
    //45 degrees -> val: 1 -> RC_pitch_offset: 90
    // -1 degress -> val: 0.5 -> RC-pitch_offset: 45
    
    /*
    static int debugcounter = 0;
    debugcounter++;
    if (debugcounter > 15) {
        debugcounter = 0;
        
        RC_pitch_offset += 10;
        

        if (RC_pitch_offset > 90.f)
        {
            RC_pitch_offset = 0.f;
        }

        //changedCurrentValue = true;
        
        //GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RC pitch offset: %.2f", RC_pitch_offset);
        
    }
    */

   /*
    if (prevValue != RC_pitch_offset)
    {
        changedCurrentValue = true;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RC pitch offset: %.2f", RC_pitch_offset);
    }
    */

   

    /*
    //ROTATION OF BOARD
    if (std::abs(val - RC_pitch_offset) > 0.5f) {
        ahrs._board_orientation = ROTATION_CUSTOM;
        ahrs._custom_yaw = 270;
        ahrs._custom_roll = (int)RC_pitch_offset;
        ahrs.update_orientation();
    }*/
    
    
}
#endif


//#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos& ch_flag)
{
    
    // put your aux switch #1 handler here (CHx_OPT = 47)
    
    RC_Channel* channel = rc().channel(8);
    float val = channel->norm_input();

    RC_pitch_offset = val * 90.f;
    changedCurrentValue = true;
    
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "new pitch: %.2f", RC_pitch_offset);
    
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos& ch_flag)
   
    
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos& ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
//#endif
