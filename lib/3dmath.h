#ifndef __3D_MATH_H__
#define __3D_MATH_H__

#include <math.h>
#include <stdint.h>

#define RADIANS_PER_DEGREE 0.01745329f

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

struct quaternion {
    float w;
    float x;
    float y;
    float z;
};

typedef struct quaternion quaternion_t;

struct vector {
    float x;
    float y;
    float z;
};

typedef struct vector vector_t;

/*
 * returns the magnitude of vector v
 */
float vector_norm(const vector_t* v);

/*
 * returns the magnitude of quaternion q
 */
float quaternion_norm(const quaternion_t* q);

/*
 * returns the product of two unit quaternions
 */
quaternion_t quaternion_product(const quaternion_t* p, const quaternion_t* q);

/*
 * rotates unit quaternion orientation about roll axis by angle
 */
quaternion_t quaternion_rotate_roll(const quaternion_t* orientation, float angle);

/*
 * rotates unit quaternion orientation about pitch axis by angle
 */
quaternion_t quaternion_rotate_pitch(const quaternion_t* orientation, float angle);

/*
 * Returns the roll angle in degrees
 * -180 < roll < 180
 */
float quaternion_get_roll(const quaternion_t* q);

/*
 * Returns the pitch angle in degrees
 * -90 < pitch < 90
 */
float quaternion_get_pitch(const quaternion_t* q);

/*
 * Returns the yaw angle in degrees
 * -180 < yaw < 180
 */
float quaternion_get_yaw(const quaternion_t* q);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __3D_MATH_H__ */
