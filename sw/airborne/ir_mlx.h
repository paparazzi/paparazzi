#ifndef IR_MLX_H
#define IR_MLX_H

#include "std.h"

#define IR_MLX_UNINIT         0
#define IR_MLX_IDLE           1
#define IR_MLX_RD_CASE_TEMP   2
#define IR_MLX_RD_OBJ_TEMP    3

extern uint8_t  ir_mlx_status;
extern uint16_t ir_mlx_itemp_case;
extern int32_t  ir_mlx_temp_case;
extern uint16_t ir_mlx_itemp_obj;
extern int32_t  ir_mlx_temp_obj;
extern bool_t   ir_mlx_available;
extern volatile bool_t ir_mlx_i2c_done;

void ir_mlx_init(void);
void ir_mlx_periodic(void);
void ir_mlx_event(void);

#endif
