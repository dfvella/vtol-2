#include "flight_controller.h"

Flight_Controller::Flight_Controller() { }

void Flight_Controller::begin()
{
    imu.calibrate();
}

void Flight_Controller::run()
{
    imu.run();
}

void Flight_Controller::calculate_outputs(Input& input, Output& output)
{
    determine_flight_mode(input);
    determine_control_mode(input);

    Input pid_output;
    calculate_pids(input, pid_output);

    map_outputs(pid_output, output);
}

void Flight_Controller::determine_flight_mode(const Input& input)
{
    // determine the requested flight mode
    Flight_Mode target_flight_mode = Flight_Mode::VERTICAL;

    if (input.gear < SWITCH_POS2)
        target_flight_mode = Flight_Mode::SLOW;
    
    if (input.gear < SWITCH_POS1)
        target_flight_mode = Flight_Mode::FORWARD;

    // determine the required flight mode
    switch (flight_mode)
    {
    case Flight_Mode::TO_FORWARD:
        if (target_flight_mode != Flight_Mode::FORWARD)
            flight_mode = Flight_Mode::TO_SLOW;
        else if (transition_state == -TRANSITION_TIME)
            flight_mode = Flight_Mode::FORWARD;
        break;

    case Flight_Mode::FORWARD:
        if (target_flight_mode != Flight_Mode::FORWARD)
            flight_mode = Flight_Mode::TO_SLOW;
        break;

    case Flight_Mode::TO_SLOW:
        if (target_flight_mode == Flight_Mode::FORWARD && transition_state < 0)
                flight_mode = Flight_Mode::TO_FORWARD;
        else if (target_flight_mode == Flight_Mode::VERTICAL && transition_state > 0)
            flight_mode = Flight_Mode::TO_VERTICAL;
        else if (transition_state == 0)
            flight_mode = Flight_Mode::SLOW;
        break;

    case Flight_Mode::SLOW:
        if (target_flight_mode == Flight_Mode::FORWARD)
            flight_mode = Flight_Mode::TO_FORWARD;

        if (target_flight_mode == Flight_Mode::VERTICAL)
            flight_mode = Flight_Mode::TO_VERTICAL;
        break;

    case Flight_Mode::TO_VERTICAL:
        if (target_flight_mode != Flight_Mode::VERTICAL)
            flight_mode = Flight_Mode::TO_SLOW;
        else if (transition_state == TRANSITION_TIME)
            flight_mode = Flight_Mode::VERTICAL;
        break;

    case Flight_Mode::VERTICAL:
        if (target_flight_mode != Flight_Mode::VERTICAL)
            flight_mode = Flight_Mode::TO_SLOW;
        break;
    }

    // update transition state
    switch (flight_mode)
    {
    case Flight_Mode::TO_FORWARD:
        --transition_state;
        break;

    case Flight_Mode::FORWARD:
        transition_state = -TRANSITION_TIME;
        break;

    case Flight_Mode::TO_SLOW:
        if (transition_state > 0)
            --transition_state;
        else if (transition_state < 0)
            ++transition_state;
        break;

    case Flight_Mode::SLOW:
        transition_state = 0;
        break;

    case Flight_Mode::TO_VERTICAL:
        ++transition_state;
        break;

    case Flight_Mode::VERTICAL:
        transition_state = TRANSITION_TIME;
        break;
    }
}

void Flight_Controller::determine_control_mode(const Input& input)
{
    control_mode = Control_Mode::AUTOLEVEL;

    if (input.aux < SWITCH_POS2)
        control_mode = Control_Mode::RATE;
    
    if (input.aux < SWITCH_POS1)
        control_mode = Control_Mode::MANUAL;
}

void Flight_Controller::calculate_pids(Input& input, Input& output)
{
    calculate_targets(input);

    float roll_error = imu.roll() - target_roll;
    float pitch_error = imu.pitch() - target_pitch;
    float yaw_error = imu.yaw() - target_yaw;

    roll_error = constrain_angle(roll_error);
    pitch_error = constrain_angle(pitch_error);
    yaw_error = constrain_angle(yaw_error);

    output.throttle = input.throttle;

    roll_error += ROLL_PID_TRIM;
    pitch_error += PITCH_PID_TRIM;
    yaw_error += YAW_PID_TRIM;

    uint16_t roll_pid_output = (uint16_t)roll_pid.calculate(roll_error);
    uint16_t pitch_pid_output = (uint16_t)pitch_pid.calculate(pitch_error);
    uint16_t yaw_pid_output = (uint16_t)yaw_pid.calculate(yaw_error);

    roll_pid_output += NEUTRAL_STICK;
    pitch_pid_output += NEUTRAL_STICK;
    yaw_pid_output += NEUTRAL_STICK;

    switch (control_mode)
    {
    case Control_Mode::MANUAL:
        output.roll = input.roll;
        output.pitch = input.pitch;
        output.yaw = input.yaw;
        break;

    default:
        output.roll = roll_pid_output;
        output.pitch = pitch_pid_output;
        output.yaw = yaw_pid_output;
        break;
    }
}

void Flight_Controller::calculate_targets(Input& input)
{
    if (abs(NEUTRAL_STICK - (int16_t)input.roll) < DEAD_STICK)
        input.roll = NEUTRAL_STICK;
    if (abs(NEUTRAL_STICK - (int16_t)input.pitch) < DEAD_STICK)
        input.pitch = NEUTRAL_STICK;
    if (abs(NEUTRAL_STICK - (int16_t)input.yaw) < DEAD_STICK)
        input.yaw = NEUTRAL_STICK;

    input.roll = input_roll_filter.calculate(input.roll);
    input.pitch = input_pitch_filter.calculate(input.pitch);
    input.yaw = input_yaw_filter.calculate(input.yaw);

    switch (control_mode)
    {
    case Control_Mode::AUTOLEVEL:
        target_roll = -1 * interpolate(input.roll,
            MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH,
            -AUTO_MAX_ROLL_ANGLE, AUTO_MAX_ROLL_ANGLE
        );
        target_pitch = -1 * interpolate(input.pitch,
            MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH,
            -AUTO_MAX_PITCH_ANGLE, AUTO_MAX_PITCH_ANGLE
        );
        target_yaw += -1 * interpolate(input.yaw,
            MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH,
            -AUTO_MAX_YAW_RATE, AUTO_MAX_YAW_RATE
        );
        target_yaw = constrain_target(target_yaw, imu.yaw(), AUTO_MAX_YAW_ERROR);
        break;

    case Control_Mode::RATE:
        target_roll += -1 * interpolate(input.roll,
            MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH,
            -RATE_MAX_ROLL_RATE, RATE_MAX_ROLL_RATE
        );
        target_roll = constrain_target(target_roll, imu.roll(), RATE_MAX_ROLL_ERROR);

        target_pitch += -1 * interpolate(input.pitch,
            MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH,
            -RATE_MAX_PITCH_RATE, RATE_MAX_PITCH_RATE
        );
        target_pitch = constrain_target(target_pitch, imu.pitch(), RATE_MAX_PITCH_ERROR);

        target_yaw += -1 * interpolate(input.yaw,
            MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH,
            -RATE_MAX_YAW_RATE, RATE_MAX_YAW_RATE
        );
        target_yaw = constrain_target(target_yaw, imu.yaw(), RATE_MAX_YAW_ERROR);
        break;

    case Control_Mode::MANUAL:
        target_roll = imu.roll();
        target_pitch = imu.pitch();
        target_yaw = imu.yaw();
        break;
    }

    target_pitch = constrain(target_pitch, -60, 60);
    target_roll = constrain_angle(target_roll);
    target_yaw = constrain_angle(target_yaw);
}

void Flight_Controller::map_outputs(Input& input, Output& output)
{
    // set the output motor tilt based on flight mode
    if (flight_mode == Flight_Mode::TO_FORWARD || flight_mode == Flight_Mode::FORWARD)
    {
        output.right_tilt = FORWARD_RIGHT_TILT;
        output.left_tilt = FORWARD_LEFT_TILT;
    }
    else if (flight_mode == Flight_Mode::TO_SLOW || flight_mode == Flight_Mode::SLOW)
    {
        output.right_tilt = SLOW_RIGHT_TILT;
        output.left_tilt = SLOW_LEFT_TILT;
    }
    else
    {
        output.right_tilt = VERTICAL_RIGHT_TILT;
        output.left_tilt = VERTICAL_LEFT_TILT;
    }

    // set other outputs based on current motor tilt
    if (transition_state < 0)
    // currently in, or transitioning to or from forward flight
    {
        output.right_motor = input.throttle;
        output.left_motor = input.throttle;

        output.right_motor -= FORWARD_YAW_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.yaw);
        output.left_motor += FORWARD_YAW_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.yaw);

        output.right_alr = (2 * NEUTRAL_STICK) - (int16_t)input.roll;
        output.left_alr = (2 * NEUTRAL_STICK) - (int16_t)input.roll;

        output.elevator = NEUTRAL_STICK + (NEUTRAL_STICK - (int16_t)input.pitch);
    }
    else if (0 <= transition_state && transition_state < TRANSITION_TIME)
    // currently in slow flight or transitioning to or from vertical flight
    {
        // add aileron into mix thrust differential
        output.right_motor = input.throttle;
        output.left_motor = input.throttle;

        output.right_motor -= SLOW_YAW_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.yaw);
        output.left_motor += SLOW_YAW_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.yaw);

        output.right_alr = (2 * NEUTRAL_STICK) - (int16_t)input.roll + SLOW_FLAPS_TRIM;
        output.left_alr = (2 * NEUTRAL_STICK) - (int16_t)input.roll - SLOW_FLAPS_TRIM;

        // add elevator into tilt?
        //output.right_tilt = SLOW_RIGHT_TILT;
        //output.left_tilt = SLOW_LEFT_TILT;

        output.elevator = NEUTRAL_STICK + (NEUTRAL_STICK - (int16_t)input.pitch);
    }
    else // currently in vertical flight mode
    {
        output.right_motor = input.throttle;
        output.left_motor = input.throttle;

        output.right_motor -= VERTICAL_ROLL_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.roll);
        output.left_motor += VERTICAL_ROLL_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.roll);

        output.right_alr = NEUTRAL_STICK + VERTICAL_FLAPS_TRIM;
        output.left_alr = NEUTRAL_STICK - VERTICAL_FLAPS_TRIM;

        output.right_tilt += (VERTICAL_YAW_MOTOR_TILT * (NEUTRAL_STICK - (int16_t)input.yaw));
        output.right_tilt += (VERTICAL_PITCH_MOTOR_TILT * (NEUTRAL_STICK - (int16_t)input.pitch));
        output.right_tilt -= VERTICAL_YAW_TRIM;

        output.left_tilt += (VERTICAL_YAW_MOTOR_TILT * (NEUTRAL_STICK - (int16_t)input.yaw));
        output.left_tilt -= (VERTICAL_PITCH_MOTOR_TILT * (NEUTRAL_STICK - (int16_t)input.pitch));
        output.left_tilt -= VERTICAL_YAW_TRIM;

        output.elevator = NEUTRAL_STICK;
    }

    // for safety: make sure only to spin up motors when throttle is applied.
    // without this flight controller could command motors for control about another axis
    // even if there is no throttle input.
    if (abs(input.throttle - NEUTRAL_THROTTLE) < DEAD_STICK)
    {
        output.right_motor = RIGHT_MOTOR_MIN_PULSE;
        output.left_motor = LEFT_MOTOR_MIN_PULSE;
    }

    // dampen the tilt servos when transitioning between flight modes
    if (flight_mode == Flight_Mode::TO_FORWARD ||
        flight_mode == Flight_Mode::TO_SLOW ||
        flight_mode == Flight_Mode::TO_VERTICAL)
    {
        uint16_t right_tilt_damper;
        uint16_t left_tilt_damper;

        if (transition_state < 0) // tilt position currently between slow and forward
        {
            right_tilt_damper = (SLOW_RIGHT_TILT - FORWARD_RIGHT_TILT) / TRANSITION_TIME;
            left_tilt_damper = (FORWARD_LEFT_TILT - SLOW_LEFT_TILT) / TRANSITION_TIME;
        }
        else
        {
            right_tilt_damper = (VERTICAL_RIGHT_TILT - SLOW_RIGHT_TILT) / TRANSITION_TIME;
            left_tilt_damper = (SLOW_LEFT_TILT - VERTICAL_LEFT_TILT) / TRANSITION_TIME;
        }

        output.right_tilt = constrain(output.right_tilt, right_tilt_last - right_tilt_damper, right_tilt_last + right_tilt_damper);
        output.left_tilt = constrain(output.left_tilt, left_tilt_last - left_tilt_damper, left_tilt_last + left_tilt_damper);
    }
    right_tilt_last = output.right_tilt;
    left_tilt_last = output.left_tilt;
}

float Flight_Controller::get_target_roll()
{
    return target_roll;
}

float Flight_Controller::get_target_pitch()
{
    return target_pitch;
}

float Flight_Controller::get_target_yaw()
{
    return target_yaw;
}

float Flight_Controller::get_roll_error()
{
    return roll_pid.get_error();
}

float Flight_Controller::get_pitch_error()
{
    return pitch_pid.get_error();
}

float Flight_Controller::get_yaw_error()
{
    return yaw_pid.get_error();
}

float Flight_Controller::get_roll_angle()
{
    return imu.roll();
}

float Flight_Controller::get_pitch_angle()
{
    return imu.pitch();
}

float Flight_Controller::get_yaw_angle()
{
    return imu.yaw();
}

Control_Mode Flight_Controller::get_control_mode()
{
    return control_mode;
}

Flight_Mode Flight_Controller::get_flight_mode()
{
    return flight_mode;
}

int16_t Flight_Controller::get_transition_state()
{
    return transition_state;
}

float Flight_Controller::interpolate(
    float val, float min_from, float max_from, float min_to, float max_to)
{
    return (((val - min_from) / (max_from - min_from)) * (max_to - min_to)) + min_to;
}

float Flight_Controller::constrain_angle(float val)
{
    if (val > 180)
        return val - 360;

    if (val < -180)
        return val + 360;

    return val;
}

float Flight_Controller::constrain_target(float val, float angle, float offset)
{
    float boundA = constrain_angle(angle - offset);
    float boundB = constrain_angle(angle + offset);

    float costA = abs(constrain_angle(val - boundA));
    float costB = abs(constrain_angle(val - boundB));

    if (costA < 2 * offset && costB < 2 * offset)
        return val;

    if (costA < costB)
        return boundA;
    else
        return boundB;
}
