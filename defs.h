#pragma once


typedef enum
{
    STEPPER_CW,
    STEPPER_CCW,
    STOP
} direction_t;

typedef enum
{
    WHITE,
    BLACK,
    ALUM,
    STEEL,
    UNKNOWN,
    DUMMY
} material_t;

typedef enum
{
    MOVE,
    WAIT
} stepper_instr_t;

typedef struct stepper_task_t
{
    stepper_instr_t instr;
    uint16_t val;
} stepper_task_t;