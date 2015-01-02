#ifndef IR_MLX_H
#define IR_MLX_H

#include "std.h"

#define MLX90614_TA   0x06
#define MLX90614_TOBJ 0x07
#define MLX90614_SADR 0x2E
#define MLX90614_ID_0 0x3C
#define MLX90614_ID_1 0x3D
#define MLX90614_ID_2 0x3E
#define MLX90614_ID_3 0x3F

enum mlx_type {
  IR_MLX_ADDR_CHANGE,
  IR_MLX_ADDR_ERASE,
  IR_MLX_ADDR_SET,
  IR_MLX_UNINIT,
  IR_MLX_RD_ID_0,
  IR_MLX_RD_ID_1,
  IR_MLX_RD_ID_2,
  IR_MLX_RD_ID_3,
  IR_MLX_IDLE,
  IR_MLX_RD_CASE_TEMP,
  IR_MLX_RD_OBJ_TEMP
};

void ir_mlx_crc(unsigned char addr, volatile unsigned char *data);
void ir_mlx_init(void);
void ir_mlx_periodic(void);
void ir_mlx_event(void);

#endif
