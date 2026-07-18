#include "gy271.h"
#include "delay.h"
#include "clock.h"
#include "uart2.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define COMPASS_KALMAN_PROCESS_NOISE 0.005f
#define COMPASS_KALMAN_MEASUREMENT_NOISE_STILL 0.5f
#define COMPASS_KALMAN_MEASUREMENT_NOISE_TURN 0.1f
#define COMPASS_KALMAN_TURN_ERROR_THRESHOLD_DEG 2.0f

volatile int16_t raw_x = 0;
volatile int16_t raw_y = 0;
volatile int16_t raw_z = 0;

volatile float mag_x = 0.0f;
volatile float mag_y = 0.0f;
volatile float mag_z = 0.0f;
volatile float heading_uncalibrated_raw = 0.0f;
volatile float heading = 0.0f;
volatile float heading_raw = 0.0f;
volatile float compass_degree = 0.0f;
volatile uint16_t compass_degree_int = 0;

volatile GY271_StatusTypeDef gy271_status = GY271_ERROR;

static float compass_kalman_angle = 0.0f;
static float compass_kalman_error = 1.0f;
static uint8_t compass_kalman_initialized = 0;

static float Compass_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Compass_CalculateHeading(float x, float y)
{
    float result = atan2f(y, x) * 180.0f / (float)M_PI;

    if (result < 0.0f)
    {
        result += 360.0f;
    }

    return result;
}

static float Compass_NormalizeAngle(float angle)
{
    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }

    while (angle < 0.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

static float Compass_WrapAngleError(float error)
{
    while (error > 180.0f)
    {
        error -= 360.0f;
    }

    while (error < -180.0f)
    {
        error += 360.0f;
    }

    return error;
}

static float Compass_GetAdaptiveMeasurementNoise(float angle_error)
{
    float abs_error = Compass_AbsFloat(angle_error);

    if (abs_error < COMPASS_KALMAN_TURN_ERROR_THRESHOLD_DEG)
    {
        return COMPASS_KALMAN_MEASUREMENT_NOISE_STILL;
    }

    return COMPASS_KALMAN_MEASUREMENT_NOISE_TURN;
}

static float Compass_KalmanUpdate(float measurement)
{
    float kalman_gain;
    float error;
    float measurement_noise;

    if (compass_kalman_initialized == 0)
    {
        compass_kalman_angle = Compass_NormalizeAngle(measurement);
        compass_kalman_error = 1.0f;
        compass_kalman_initialized = 1;
        return compass_kalman_angle;
    }

    compass_kalman_error += COMPASS_KALMAN_PROCESS_NOISE;

    error = Compass_WrapAngleError(measurement - compass_kalman_angle);
    measurement_noise = Compass_GetAdaptiveMeasurementNoise(error);
    kalman_gain = compass_kalman_error / (compass_kalman_error + measurement_noise);

    compass_kalman_angle += kalman_gain * error;
    compass_kalman_angle = Compass_NormalizeAngle(compass_kalman_angle);
    compass_kalman_error = (1.0f - kalman_gain) * compass_kalman_error;

    return compass_kalman_angle;
}

int main(void)
{
    clock_init();
    delay_init(RCC_SYS_CLOCK_HZ);
    uart2_init();

    gy271_status = GY271_Init();
    uart2_send_string("GY271 UART2 start\r\n");
    if (gy271_status == GY271_OK)
    {
        compass_kalman_initialized = 0;
    }

    while (1)
    {
        GY271_RawData current_raw;
        float current_raw_heading = 0.0f;
        float current_filtered_heading = 0.0f;

        if (gy271_status == GY271_OK)
        {
            gy271_status = GY271_ReadRawData(&current_raw);
            if (gy271_status == GY271_OK)
            {
                current_raw_heading = Compass_CalculateHeading((float)current_raw.x, (float)current_raw.y);
                current_filtered_heading = Compass_KalmanUpdate(current_raw_heading);

                raw_x = current_raw.x;
                raw_y = current_raw.y;
                raw_z = current_raw.z;

                mag_x = (float)current_raw.x;
                mag_y = (float)current_raw.y;
                mag_z = (float)current_raw.z;

                heading_uncalibrated_raw = current_raw_heading;
                heading_raw = current_raw_heading;
                heading = current_filtered_heading;
                compass_degree = current_filtered_heading;
                compass_degree_int = (uint16_t)current_filtered_heading;


                uart2_send_string(" deg, filtered_heading=");
                uart2_send_float(compass_degree, 2);


                uart2_send_string("raw_heading=");
                uart2_send_float(heading_uncalibrated_raw, 2);

                uart2_send_string(" deg\r\n");
            }
        }
        else
        {
            gy271_status = GY271_Init();
            if (gy271_status == GY271_OK)
            {
                compass_kalman_initialized = 0;
            }
        }

        delay_ms(200);
    }
}
