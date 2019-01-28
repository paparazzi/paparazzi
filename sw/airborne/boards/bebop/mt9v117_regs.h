#ifndef MT9V117_REGS_H
#define MT9V117_REGS_H

#define MT9V117_ADDRESS 0xBA      ///< The i2c address of the chip

/* Registers */
#define MT9V117_CHIP_ID                       0x0000      ///< Request the chip ID
#define MT9V117_CHIP_ID_RESP                  0x2282    ///< Should be the response to CHIP_ID
#define MT9V117_RESET_MISC_CTRL               0x001A
#define MT9V117_RESET_SOC_I2C                 (1 << 0)
#define MT9V117_PAD_SLEW                      0x0030
#define MT9V117_COMMAND                       0x0040
#define MT9V117_COMMAND_OK                    (1 << 15)
#define MT9V117_COMMAND_WAIT_FOR_EVENT        (1 << 3)
#define MT9V117_COMMAND_REFRESH               (1 << 2)
#define MT9V117_COMMAND_SET_STATE             (1 << 1)
#define MT9V117_COMMAND_APPLY_PATCH           (1 << 0)
#define MT9V117_ACCESS_CTL_STAT               0x0982
#define MT9V117_PHYSICAL_ADDRESS_ACCESS       0x098A
#define MT9V117_LOGICAL_ADDRESS_ACCESS        0x098E
#define MT9V117_AE_TRACK_JUMP_DIVISOR         0xA812
#define MT9V117_CAM_AET_SKIP_FRAMES           0xC868

/* Variables */
#define MT9V117_AE_RULE_VAR                              9
#define MT9V117_AE_RULE_ALGO_OFFSET                      4
#define MT9V117_AE_RULE_ALGO_AVERAGE                     0
#define MT9V117_AE_RULE_ALGO_WEIGHTED                    1
#define MT9V117_AE_TRACK_VAR                             10
#define MT9V117_AWB_VAR                                  11
#define MT9V117_AWB_PIXEL_THRESHOLD_COUNT_OFFSET         64
#define MT9V117_LOW_LIGHT_VAR                            15
#define MT9V117_CAM_CTRL_VAR                             18
#define MT9V117_CAM_SENSOR_CFG_Y_ADDR_START_OFFSET       0
#define MT9V117_CAM_SENSOR_CFG_X_ADDR_START_OFFSET       2
#define MT9V117_CAM_SENSOR_CFG_Y_ADDR_END_OFFSET         4
#define MT9V117_CAM_SENSOR_CFG_X_ADDR_END_OFFSET         6
#define MT9V117_CAM_SENSOR_CFG_FRAME_LENGTH_LINES_OFFSET 14
#define MT9V117_CAM_SENSOR_CFG_CPIPE_LAST_ROW_OFFSET     20
#define MT9V117_CAM_SENSOR_CFG_FDPERIOD_60HZ             22
#define MT9V117_CAM_SENSOR_CFG_FDPERIOD_50HZ             24
#define MT9V117_CAM_SENSOR_CFG_MAX_FDZONE_60_OFFSET      26
#define MT9V117_CAM_SENSOR_CFG_MAX_FDZONE_50_OFFSET      28
#define MT9V117_CAM_SENSOR_CFG_TARGET_FDZONE_60_OFFSET   30
#define MT9V117_CAM_SENSOR_CFG_TARGET_FDZONE_50_OFFSET   32
#define MT9V117_CAM_SENSOR_CONTROL_READ_MODE_OFFSET      40
#define MT9V117_CAM_SENSOR_CONTROL_Y_SKIP_EN             (1 << 2)
#define MT9V117_CAM_SENSOR_CONTROL_VERT_FLIP_EN          (1 << 1)
#define MT9V117_CAM_SENSOR_CONTROL_HORZ_MIRROR_EN        (1 << 0)
#define MT9V117_CAM_FLICKER_PERIOD_OFFSET                62
#define MT9V117_CAM_FLICKER_PERIOD_60HZ                  0
#define MT9V117_CAM_FLICKER_PERIOD_50HZ                  1
#define MT9V117_CAM_CROP_WINDOW_XOFFSET_OFFSET           72
#define MT9V117_CAM_CROP_WINDOW_YOFFSET_OFFSET           74
#define MT9V117_CAM_CROP_WINDOW_WIDTH_OFFSET             76
#define MT9V117_CAM_CROP_WINDOW_HEIGHT_OFFSET            78
#define MT9V117_CAM_CROP_MODE_OFFSET                     80
#define MT9V117_CAM_OUTPUT_WIDTH_OFFSET                  84
#define MT9V117_CAM_OUTPUT_HEIGHT_OFFSET                 86
#define MT9V117_CAM_OUTPUT_FORMAT_OFFSET                 88
#define MT9V117_CAM_OUTPUT_FORMAT_RGB_565                (0 << 12)
#define MT9V117_CAM_OUTPUT_FORMAT_RGB_555                (1 << 12)
#define MT9V117_CAM_OUTPUT_FORMAT_RGB_444X               (2 << 12)
#define MT9V117_CAM_OUTPUT_FORMAT_RGB_X444               (3 << 12)
#define MT9V117_CAM_OUTPUT_FORMAT_BAYER_10               (0 << 10)
#define MT9V117_CAM_OUTPUT_FORMAT_YUV                    (0 << 8)
#define MT9V117_CAM_OUTPUT_FORMAT_RGB                    (1 << 8)
#define MT9V117_CAM_OUTPUT_FORMAT_BAYER                  (2 << 8)
#define MT9V117_CAM_OUTPUT_FORMAT_BT656_ENABLE           (1 << 3)
#define MT9V117_CAM_OUTPUT_FORMAT_MONO_ENABLE            (1 << 2)
#define MT9V117_CAM_OUTPUT_FORMAT_SWAP_BYTES             (1 << 1)
#define MT9V117_CAM_OUTPUT_FORMAT_SWAP_RED_BLUE          (1 << 0)
#define MT9V117_CAM_STAT_AWB_HG_WINDOW_XSTART_OFFSET     236
#define MT9V117_CAM_STAT_AWB_HG_WINDOW_YSTART_OFFSET     238
#define MT9V117_CAM_STAT_AWB_HG_WINDOW_XEND_OFFSET       240
#define MT9V117_CAM_STAT_AWB_HG_WINDOW_YEND_OFFSET       242
#define MT9V117_CAM_STAT_AE_INITIAL_WINDOW_XSTART_OFFSET 244
#define MT9V117_CAM_STAT_AE_INITIAL_WINDOW_YSTART_OFFSET 246
#define MT9V117_CAM_STAT_AE_INITIAL_WINDOW_XEND_OFFSET   248
#define MT9V117_CAM_STAT_AE_INITIAL_WINDOW_YEND_OFFSET   250
#define MT9V117_CAM_LL_START_GAIN_METRIC_OFFSET          278
#define MT9V117_CAM_LL_STOP_GAIN_METRIC_OFFSET           280
#define MT9V117_SYSMGR_VAR                               23
#define MT9V117_SYSMGR_NEXT_STATE_OFFSET                 0
#define MT9V117_SYS_STATE_ENTER_CONFIG_CHANGE            0x28
#define MT9V117_SYS_STATE_STREAMING                      0x31
#define MT9V117_SYS_STATE_START_STREAMING                0x34
#define MT9V117_SYS_STATE_ENTER_SUSPEND                  0x40
#define MT9V117_SYS_STATE_SUSPENDED                      0x41
#define MT9V117_SYS_STATE_ENTER_STANDBY                  0x50
#define MT9V117_SYS_STATE_STANDBY                        0x52
#define MT9V117_SYS_STATE_LEAVE_STANDBY                  0x54
#define MT9V117_PATCHLDR_VAR                             24
#define MT9V117_PATCHLDR_LOADER_ADDRESS_OFFSET           0
#define MT9V117_PATCHLDR_PATCH_ID_OFFSET                 2
#define MT9V117_PATCHLDR_FIRMWARE_ID_OFFSET              4

#endif /* MT9V117_REGS_H */
