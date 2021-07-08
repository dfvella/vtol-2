#include "3dmath.h"

/*
 * returns the magnitude of vector v
 */
float vector_norm(const vector_t* v) {
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

/*
 * returns the magnitude of quaternion q
 */
float quaternion_norm(const quaternion_t* q) {
    return sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

/*
 * returns the product of two unit quaternions
 */
quaternion_t quaternion_product(const quaternion_t* p, const quaternion_t* q) {
    quaternion_t result = {
        .w = p->w * q->w - p->x * q->x - p->y * q->y - p->z * q->z,
        .x = p->w * q->x + p->x * q->w + p->y * q->z - p->z * q->y,
        .y = p->w * q->y - p->x * q->z + p->y * q->w + p->z * q->x,
        .z = p->w * q->z + p->x * q->y - p->y * q->x + p->z * q->w
    };

    /* scale for noise reduction */
    float l = quaternion_norm(&result);

    result.w /= l;
    result.x /= l;
    result.y /= l;
    result.x /= l;

    return result;
}

/*
 * rotates unit quaternion orientation about roll axis by angle
 */
quaternion_t quaternion_rotate_roll(const quaternion_t* orientation, float angle) {
    quaternion_t rotation = {
        .w = cos(angle * RADIANS_PER_DEGREE * 0.5), 
        .x = sin(angle * RADIANS_PER_DEGREE * 0.5),
        .y = 0.0001,
        .z = 0.0001
    };

    return quaternion_product(orientation, &rotation);
}

/*
 * rotates unit quaternion orientation about pitch axis by angle
 */
quaternion_t quaternion_rotate_pitch(const quaternion_t* orientation, float angle) {
    quaternion_t rotation = {
        .w = cos(angle * RADIANS_PER_DEGREE * 0.5), 
        .x = 0.0001, 
        .y = sin(angle * RADIANS_PER_DEGREE * 0.5), 
        .z = 0.0001 
    };

    return quaternion_product(orientation, &rotation);
}

/*
 * Returns the roll angle in degrees
 * -180 < roll < 180
 */
float quaternion_get_roll(const quaternion_t* q) {
    float result = atan2(
        2 * q->x * q->w - 2 * q->y * q->z,
        1 - 2 * q->x * q->x - 2 * q->z * q->z
    );

    result /= RADIANS_PER_DEGREE;

    if (result < -90) 
        result += 270;
    else 
        result -= 90;

    return result;
}

/*
 * Returns the pitch angle in degrees
 * -90 < pitch < 90
 */
float quaternion_get_pitch(const quaternion_t* q) {
    float result = asin(2 * q->x * q->y + 2 * q->z * q->w);

    return result / RADIANS_PER_DEGREE;
}

/*
 * Returns the yaw angle in degrees
 * -180 < yaw < 180
 */
float quaternion_get_yaw(const quaternion_t* q) {
    float result = atan2(
        2 * q->y * q->w - 2 * q->x * q->z,
        1 - 2 * q->y * q->y - 2 * q->z * q->z
    );

    return result / RADIANS_PER_DEGREE;
}
