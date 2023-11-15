#include "Sub.h"



const uint8_t SERVO_CHAN_LIGHT = 9; // Pixhawk Aux1
const uint8_t SERVO_CHAN_GRIPPER = 10; // Pixhawk Aux2

// manual_init - initialise manual controller
bool Sub::manual_init()
{
    // set target altitude to zero for reporting
    pos_control.set_pos_target_z_cm(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    set_neutral_controls();

    return true;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void Sub::manual_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    motors.set_roll(channel_roll->norm_input());
    motors.set_pitch(channel_pitch->norm_input());
    motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P);
    motors.set_throttle(channel_throttle->norm_input());
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
    
}

//naodai
void Sub::motor_set(float *motor_control)
{
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors.armed(true);
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    motors.set_motor_control_value(motor_control);
}

// naodai 
void Sub::light_set(int16_t light_control)
{
    printf("set light pwm value: %d\n", light_control);
    SRV_Channels::set_default_function(SERVO_CHAN_LIGHT-1, SRV_Channel::k_manual);
    SRV_Channels::set_default_function(SERVO_CHAN_LIGHT, SRV_Channel::k_manual);
    ServoRelayEvents.do_set_servo(SERVO_CHAN_LIGHT, light_control); // 1-indexed
}

// naodai
void Sub::gripper_set(int16_t gripper_control)
{
    // motors.set_gripper_control_value(gripper_control);
    printf("set gripper pwm value %d\n", gripper_control);
    SRV_Channels::set_default_function(SERVO_CHAN_GRIPPER-1, SRV_Channel::k_manual);
    SRV_Channels::set_default_function(SERVO_CHAN_GRIPPER, SRV_Channel::k_manual);
    ServoRelayEvents.do_set_servo(SERVO_CHAN_GRIPPER, gripper_control); // 1-indexed
}

// ServoRelayEvents.do_set_servo(SERVO_CHAN_1, pwm_out); // 1-indexed

// naodai
// void Sub::t265_power_set(int16_t t265_power_control)
// {
    // motors.set_t265_power_control_value(t265_power_control);
// }
