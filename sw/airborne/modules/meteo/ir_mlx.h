#ifndef IR_MLX_H
#define IR_MLX_H

#include "std.h"

#define MLX90614_TA   0x06
#define MLX90614_TOBJ 0x07

#define IR_MLX_UNINIT         0
#define IR_MLX_IDLE           1
#define IR_MLX_RD_CASE_TEMP   2
#define IR_MLX_RD_OBJ_TEMP    3

void ir_mlx_init(void);
void ir_mlx_periodic(void);
void ir_mlx_event(void);

#endif
