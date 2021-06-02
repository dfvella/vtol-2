#include "imu_driver.h"

Imu::Imu() : Imu(0)
{
    orientation.w = 0.70710;
    orientation.x = 0.70710;
    orientation.y = 0.00001;
    orientation.z = 0.00001;
}

Imu::Imu(int8_t pin) : status_led(pin) 
{
    orientation.w = 0.70710;
    orientation.x = 0.70710;
    orientation.y = 0.00001;
    orientation.z = 0.00001;
}

// Waits for rest then zeros gyro with reference to gravity
void Imu::calibrate() 
{
    mpu.begin();

    bool led_state = false;

    uint16_t count = 0, rest = 0;

    int16_t x_prev, y_prev, z_prev;

    // Wait until the derivative of the acceleration along each axis is below 
    // the MIN_ACCEL_DIFF for the duration of the PRE_CALIBRATION_REST_TIMER
    #ifdef WAIT_FOR_REST
    timer = 0;
    while (rest < PRE_CALIBRATION_REST_TIMER) 
    {
        while (micros() - timer < 4000);
        timer = micros();

        mpu.fetch();

        uint16_t x_diff = abs(mpu.get(ACCELX) - x_prev);
        uint16_t y_diff = abs(mpu.get(ACCELY) - y_prev);
        uint16_t z_diff = abs(mpu.get(ACCELZ) - z_prev);

        x_prev = mpu.get(ACCELX);
        y_prev = mpu.get(ACCELY);
        z_prev = mpu.get(ACCELZ);

        if (x_diff < MIN_ACCEL_DIFF 
          && y_diff < MIN_ACCEL_DIFF 
          && z_diff < MIN_ACCEL_DIFF)
            ++rest;
        else 
            rest = 0;

        if (status_led)
        {
            if (++count % 200 == 0) led_state = !led_state;
                digitalWrite(status_led, led_state);
        }
    }
    #endif // WAIT_FOR_REST

    x_zero = 0;
    y_zero = 0;
    z_zero = 0;

    float accel_x = 0;
    float accel_z = 0;
    float accel_y = 0;

    for (uint16_t i = 0; i < GYRO_CALIBRATION_READINGS; ++i) 
    {
        while (micros() - timer < 4000);
        timer = micros();

        mpu.fetch();

        x_zero += mpu.get(GYROX);
        y_zero += mpu.get(GYROY);
        z_zero += mpu.get(GYROZ);

        accel_x += mpu.get(ACCELX);
        accel_y += mpu.get(ACCELY);
        accel_z += mpu.get(ACCELZ);

        if (status_led) 
        {
            if (++count % 25 == 0) led_state = !led_state;
                digitalWrite(status_led, led_state); 
        }
    }

    x_zero /= GYRO_CALIBRATION_READINGS;
    y_zero /= GYRO_CALIBRATION_READINGS;
    z_zero /= GYRO_CALIBRATION_READINGS;

    accel_x /= GYRO_CALIBRATION_READINGS;
    accel_y /= GYRO_CALIBRATION_READINGS;
    accel_z /= GYRO_CALIBRATION_READINGS;

    accel_x -= ACCELX_LEVEL_READING;
    accel_y -= ACCELY_LEVEL_READING;
    accel_z -= ACCELZ_LEVEL_READING;

    Vector net_accel = { accel_x, accel_y, accel_z };

    x_angle_accel = asin(accel_y / norm(net_accel)) * (1 / RADIANS_PER_DEGREE);
    y_angle_accel = asin(accel_x / norm(net_accel)) * (1 / RADIANS_PER_DEGREE) * -1;

    // If GRAVITY_REFERENCED_ZERO is enabled, set the initial orientation of the aircraft
    // using the roll and pitch values computed from the net acceleration vector.
    #ifdef ENABLE_GRAVITY_REFERENCED_ZERO

    Quaternion initial_roll = {
        cos(x_angle_accel * RADIANS_PER_DEGREE * 0.5), 
        sin(x_angle_accel * RADIANS_PER_DEGREE * 0.5),
        0.0001,
        0.0001
    };

    orientation = product(orientation, initial_roll);

    Quaternion initial_pitch = {
        cos(y_angle_accel * RADIANS_PER_DEGREE * 0.5), 
        0.0001, 
        sin(y_angle_accel * RADIANS_PER_DEGREE * 0.5), 
        0.0001 
    };

    orientation = product(orientation, initial_pitch);

    #endif // ENABLE_GRAVITY_REFERENCED_ZERO
}

// Immediately begins sampling accelerometer data and prints the average
// of these readings to the serial monitor. Mpu6050 should be placed
// on a level surface.
// NOTE: Serial.begin() must be called before using this function
void Imu::calibrate_accel() 
{
    calibrate();

    bool led_state;

    Serial.println("Calibrating Accelerometer...");

    float x = 0;
    float y = 0;
    float z = 0;

    timer = 0;
    int8_t count = 0;
    for (uint16_t i = 0; i < ACCEL_CALIBRATION_READINGS; ++i) 
    {
        // regulate the loop at 250 hertz
        while (micros() - timer < 4000);
        timer = micros();

        mpu.fetch();

        x += mpu.get(ACCELX) / (float)ACCEL_CALIBRATION_READINGS;
        y += mpu.get(ACCELY) / (float)ACCEL_CALIBRATION_READINGS;
        z += mpu.get(ACCELZ) / (float)ACCEL_CALIBRATION_READINGS;

        // rapidly flash the led indicator 
        if (status_led) 
        {
            if (++count % 25 == 0) led_state = !led_state;
            digitalWrite(status_led, led_state); 
        }
    }

    // print the averages to the serial monitor
    Serial.print(" x: ");
    Serial.print(x);
    Serial.print(" y: ");
    Serial.print(y);
    Serial.print(" z: ");
    Serial.print(z - Mpu6050::TICKS_PER_G);
    Serial.println();
}

// Computes the IMU's orientation
void Imu::run() 
{
    mpu.fetch();

    static bool start = true;

    if (start) 
    {
        start = false;
        timer = micros();
    }
    else 
    {
        float t_delta = (micros() - timer) / 1000000.0; // t_delta units: seconds
        timer = micros();

        Vector w;
        w.x = (mpu.get(GYROX) - x_zero) * Mpu6050::TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        w.y = (mpu.get(GYROY) - y_zero) * Mpu6050::TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        w.z = (mpu.get(GYROZ) - z_zero) * Mpu6050::TICKS_PER_DEGREE * RADIANS_PER_DEGREE;

        float w_norm = norm(w);

        Quaternion rotation;
        rotation.w = cos((t_delta * w_norm) / 2);
        rotation.x = (sin((t_delta * w_norm) / 2) * w.x) / w_norm;
        rotation.y = (sin((t_delta * w_norm) / 2) * w.y) / w_norm;
        rotation.z = (sin((t_delta * w_norm) / 2) * w.z) / w_norm;

        orientation = product(orientation, rotation);

        x_angle = atan2(2 * orientation.x * orientation.w - 2 * orientation.y * orientation.z, 
                1 - 2 * orientation.x * orientation.x - 2 * orientation.z * orientation.z);

        y_angle = asin(2 * orientation.x * orientation.y + 2 * orientation.z * orientation.w);

        z_angle = atan2(2 * orientation.y * orientation.w - 2 * orientation.x * orientation.z, 
                1 - 2 * orientation.y * orientation.y - 2 * orientation.z * orientation.z);

        x_angle /= RADIANS_PER_DEGREE;
        y_angle /= RADIANS_PER_DEGREE;
        z_angle /= RADIANS_PER_DEGREE;

        if (x_angle < -90) 
            x_angle += 270;
        else 
            x_angle -= 90;

        #ifdef ENABLE_GYRO_DRIFT_FILTER
        // construct a 3 dimensional vector representing the net acceration on the aircraft
        Vector net_accel; 
        net_accel.x = (float)mpu.get(ACCELX) - ACCELX_LEVEL_READING;
        net_accel.y = (float)mpu.get(ACCELY) - ACCELY_LEVEL_READING;
        net_accel.z = (float)mpu.get(ACCELZ) - ACCELZ_LEVEL_READING;

        // compute the magnitude of the net acceration
        float accel_norm = norm(net_accel);

        // compute the roll and pitch angles using the net acceleration vector
        float x_angle_accel_current = asin(net_accel.y / accel_norm) * (1 / RADIANS_PER_DEGREE);
        float y_angle_accel_current = asin(net_accel.x / accel_norm) * (1 / RADIANS_PER_DEGREE) * -1;

        // Apply an IIR filter to the accelerometer angles for noise reduction 
        x_angle_accel = x_angle_accel * ACCEL_FILTER_GAIN + 
                        x_angle_accel_current * (1 - ACCEL_FILTER_GAIN);
        y_angle_accel = y_angle_accel * ACCEL_FILTER_GAIN + 
                        y_angle_accel_current * (1 - ACCEL_FILTER_GAIN);

        // Apply the antidrift filter by computring a weighted average between the angle
        // given by the gyroscope data and the angle given by the accelerometer data.
        x_angle = x_angle * DRIFT_FILTER_GAIN + x_angle_accel * (1 - DRIFT_FILTER_GAIN);
        y_angle = y_angle * DRIFT_FILTER_GAIN + y_angle_accel * (1 - DRIFT_FILTER_GAIN);

        #endif // ENABLE_GYRO_DRIFT_FILTER

        // inevert axes if necessary 
        #ifdef INVERT_ROLL_AXIS
        x_angle *= -1;
        #endif    
        #ifdef INVERT_PITCH_AXIS
        y_angle *= -1;
        #endif
        #ifdef INVERT_YAW_AXIS
        z_angle *= -1;
        #endif
    }
}

// Returns the roll angle of the aircraft in degrees
//  -180 < roll < 180
float Imu::roll() 
{
    return x_angle;
}

// Returns the pitch angle of the aircraft in degrees
//  -90 < pitch < 90
float Imu::pitch() 
{
    return y_angle;
}

// Returns the yaw angle of the aircraft in degrees
//  -180 < yaw < 180
float Imu::yaw() 
{
    return z_angle;
}

int32_t Imu::get_raw(uint8_t val)
{
    return mpu.get(val);
}

// computes then returns the product of two quaternions
Imu::Quaternion Imu::product(const Quaternion &p, const Quaternion &q) 
{
    Quaternion result;

    result.w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z;
    result.x = p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y;
    result.y = p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x;
    result.z = p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w;

    // adjust dimensions to ensure the norm is 1 for noise reduction
    float l = norm(result);
    result.w /= l;
    result.x /= l;
    result.y /= l;
    result.x /= l;

    return result;
}

// Computes the magnitude of quaternion q
float Imu::norm(const Quaternion &q) 
{
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// Computes the magnitude of vector v
float Imu::norm(const Vector &v) 
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Imu::Mpu6050::Mpu6050() { }

// Configures the MPU6050 sensor
void Imu::Mpu6050::begin() 
{
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(MPU6050_POWER_MANAGEMENT_REGISTER);
    Wire.write(MPU_6050_POWER_ON);
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(MPU6050_ACCELEROMETER_CONFIG_REGISTER);
    Wire.write(ACCELEROMETER_ACCURACY);
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(MPU6050_GYRO_CONFIG_REGISTER);
    Wire.write(GYRO_ACCURACY);
    Wire.endTransmission();
}

// Fills data array with the most recent MPU6050 measurements
void Imu::Mpu6050::fetch() 
{
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(MPU6050_ACCEL_XOUT_H_REGISTER);
    Wire.endTransmission();

    Wire.requestFrom(MPU6050_I2C_ADDRESS, 14);
    while (Wire.available() < 14);

    for (int32_t *ptr = data; ptr < data + 7; ++ptr)
    *ptr = Wire.read() << 8 | Wire.read();
}

// Returns the requested raw MPU6050 data
// val - one of the following constant expressions:
// ACCELX, ACCELY, ACCELZ, GYROX, GYROY, GYROZ
int32_t Imu::Mpu6050::get(uint8_t val) 
{
    return data[val];
}
