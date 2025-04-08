/*
 * vector.h
 *
 *  Created on: Oct 5, 2024
 *      Author: Vincent
 */

#ifndef SRC_COMMONMATH_VECTOR_H_
#define SRC_COMMONMATH_VECTOR_H_

#include "stdbool.h"
#include "maths.h"
typedef union vector2_u
{
    float v[2];
    struct
	{
       float x, y;
    };
} vector2_t;

typedef union vector3_u
{
    float v[3];
    struct
	{
       float x, y, z;
    };
} vector3_t;


typedef struct matrix33_s
{
    float m[3][3];
} matrix33_t;
typedef enum
{
    X = 0,
    Y,
    Z
} axis_e;

#define XYZ_AXIS_COUNT 3

// See http://en.wikipedia.org/wiki/Flight_dynamics
typedef enum
{
    FD_ROLL = 0,
    FD_PITCH,
    FD_YAW
} flight_dynamics_index_t;

#define FLIGHT_DYNAMICS_INDEX_COUNT 3

typedef enum
{
    AI_ROLL = 0,
    AI_PITCH
} angle_index_t;

#define ANGLE_INDEX_COUNT 2
#define XYZ_AXIS_COUNT 3
#define GET_DIRECTION(isReversed) ((isReversed) ? -1 : 1)


bool vector2Equal(const vector2_t *a, const vector2_t *b);
void vector2Zero(vector2_t *v);
void vector2Add(vector2_t *result, const vector2_t *a, const vector2_t *b);
void vector2Scale(vector2_t *result, const vector2_t *v, const float k);
float vector2Dot(const vector2_t *a, const vector2_t *b);
float vector2Cross(const vector2_t *a, const vector2_t *b);
float vector2NormSq(const vector2_t *v);
float vector2Norm(const vector2_t *v);
void vector2Normalize(vector2_t *result, const vector2_t *v);

bool vector3Equal(const vector3_t *a, const vector3_t *b);
void vector3Zero(vector3_t *v);
void vector3Add(vector3_t *result, const vector3_t *a, const vector3_t *b);
void vector3Scale(vector3_t *result, const vector3_t *v, const float k);
float vector3Dot(const vector3_t *a, const vector3_t *b);
void vector3Cross(vector3_t *result, const vector3_t *a, const vector3_t *b);
float vector3NormSq(const vector3_t *v);
float vector3Norm(const vector3_t *v);
void vector3Normalize(vector3_t *result, const vector3_t *v);

void matrixVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v);
void matrixTrnVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v);

void buildRotationMatrix(matrix33_t *result, const fp_angles_t *rpy);
void applyRotationMatrix(vector3_t *v, const matrix33_t *rotationMatrix);

void yawToRotationMatrixZ(matrix33_t *result, const float yaw);

#endif /* SRC_COMMONMATH_VECTOR_H_ */
