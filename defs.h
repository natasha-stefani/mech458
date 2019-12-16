#pragma once

// Stepper motor direction tracking
typedef enum
{
    STEPPER_CW,
    STEPPER_CCW,
    STOP
} direction_t;

// Item classification
typedef enum
{
    WHITE,
    BLACK,
    ALUM,
    STEEL,
    UNKNOWN,
    DUMMY
} material_t;
