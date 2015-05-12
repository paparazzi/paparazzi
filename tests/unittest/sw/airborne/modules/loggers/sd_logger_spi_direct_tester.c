#include "unity.h"
#include "subsystems/datalink/Mocktelemetry.h"
#include "Mockmessages_testable.h"
#include "peripherals/Mocksdcard_spi.h"
#include "subsystems/Mockimu.h"
#include "subsystems/actuators/Mockactuators_pwm_arch.h"
#include "loggers/sd_logger_spi_direct.h"

#define S(x) #x
#define S_(x) S(x)
#define S__LINE__ "Line: " S_(__LINE__)

/* Actually defined in sdcard.c */
struct SDCard sdcard1;

/* Actually defined in spi.c */
struct spi_periph spi2;

/* Actually defined in imu.c */
struct Imu imu;

/* Actually defined in actuators_pwm_arch.c */
int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

/* Actually defined in uart.c */
struct uart_periph uart1;

/* Actually defined in pprz_transport.c */
struct pprz_transport pprz_tp;

/* Actually defined in sd_logger.c */
struct SdLogger sdlogger;


void setUp(void)
{
  /* Set incorrect values to ensure proper initialization */
  sdlogger.block_addr = 0x12345678;
  sdlogger.buffer_addr = 0x1234;
  sdlogger.cmd = 0x57;
  sdlogger.packet_count = 12345;
  sdlogger.error_count = 12344;
  for (uint16_t i=0; i<520; i++){
    sdcard1.output_buf[i] = 0xAB;
    sdcard1.input_buf[i] = 0xBA;
  }
  sdlogger.packet.time = 56187645;
  sdlogger.packet.data_1 = 568686;
  sdlogger.packet.data_2 = 568686;
  sdlogger.packet.data_3 = 568686;
  sdlogger.packet.data_4 = 568686;
  sdlogger.packet.data_5 = 568686;
  sdlogger.packet.data_6 = 568686;
  sdlogger.packet.data_7 = 568686;
  sdlogger.packet.data_8 = 568686;
  sdlogger.packet.data_9 = 568686;
  sdlogger.packet.data_10 = 23432;
  sdlogger.packet.data_11 = 23432;
  sdlogger.packet.data_12 =  23432;
  sdlogger.timeout_counter = 87;
  Mocksdcard_spi_Init();
}

void tearDown(void)
{
  Mocksdcard_spi_Verify();
  Mocksdcard_spi_Destroy();
}

void helper_ExpectSdLoggerPeriodic(void)
{
  sdcard_spi_periodic_Expect(&sdcard1);
  sd_logger_periodic();
}

void test_Initialize(void)
{
  sdcard_spi_init_Expect(&sdcard1, &(SD_LOGGER_SPI_LINK_DEVICE), SD_LOGGER_SPI_LINK_SLAVE_NUMBER);

  /* Call the start function */
  sd_logger_start();
  TEST_ASSERT_EQUAL(SdLogger_Initializing, sdlogger.status);
}

void test_SdCardBusyWithInitialization(void)
{
  sdlogger.status = SdLogger_Initializing;
  sdcard1.status = SDCard_SendingCMD0;

  /* Run periodic loop */
  helper_ExpectSdLoggerPeriodic();
  TEST_ASSERT_EQUAL(SdLogger_Initializing, sdlogger.status);
}

void test_SdCardInitializationComplete(void)
{
  sdlogger.status = SdLogger_Initializing;
  sdcard1.status = SDCard_Idle;

  /* Run periodic loop */
  helper_ExpectSdLoggerPeriodic();
  TEST_ASSERT_EQUAL(SdLogger_Idle, sdlogger.status);
}

void test_SdCardInitializationFailed(void)
{
  sdlogger.status = SdLogger_Initializing;
  sdcard1.status = SDCard_Error;

  /* Run periodic loop */
  helper_ExpectSdLoggerPeriodic();
  TEST_ASSERT_EQUAL(SdLogger_Error, sdlogger.status);
}

void test_DontAcceptStartCommandIfNotIdle(void)
{
  sdlogger.status = SDCard_SendingCMD0;
  sdlogger.cmd = SdLoggerCmd_StartLogging;

  /* Call the command function */
  sd_logger_command();

  /* Always reset command */
  TEST_ASSERT_EQUAL_HEX8(SdLoggerCmd_Nothing, sdlogger.cmd);
}

void test_AcceptStartCommandIfIdle(void)
{
  sdcard1.status = SDCard_Idle;
  sdlogger.cmd = SdLoggerCmd_StartLogging;

  sdcard_spi_multiwrite_start_Expect(&sdcard1, 0x00000001);

  /* Call the command function */
  sd_logger_command();

  TEST_ASSERT_EQUAL(0, sdlogger.error_count);
  TEST_ASSERT_EQUAL(SdLogger_BeforeLogging, sdlogger.status);
  TEST_ASSERT_EQUAL(0, sdlogger.packet_count);
  TEST_ASSERT_EQUAL(4, sdlogger.buffer_addr);
  TEST_ASSERT_EQUAL_HEX8(SdLoggerCmd_Nothing, sdlogger.cmd); // Always reset
}

void test_SwitchToLoggingWhenSdIsIdle(void)
{
  sdlogger.status = SdLogger_BeforeLogging;
  sdcard1.status = SDCard_MultiWriteIdle;

  /* Periodic loop */
  helper_ExpectSdLoggerPeriodic();
  TEST_ASSERT_EQUAL(SdLogger_Logging, sdlogger.status);
}

void test_DontSwitchToLoggingIfSdNotIdle(void)
{
  sdlogger.status = SdLogger_BeforeLogging;
  sdcard1.status = SDCard_Busy;

  /* Periodic loop */
  helper_ExpectSdLoggerPeriodic();
  TEST_ASSERT_EQUAL(SdLogger_BeforeLogging, sdlogger.status);
}

void test_AcceptStopCommandIfLogging(void)
{
  sdlogger.status = SdLogger_Logging;
  sdlogger.cmd = SdLoggerCmd_StopLogging;
  sdlogger.buffer_addr = 200; /* not at the end */
  sdlogger.packet_count = 1876;
  for (uint16_t i=0; i<sdlogger.buffer_addr; i++) {
    sdcard1.output_buf[1+i] = 0xDD;
  }

  /* Expect call to sdcard write block */
  sdcard_spi_multiwrite_next_Expect(&sdcard1);

  /* Call the command function */
  sd_logger_command();

  /* Dont modify logged values */
  for (uint32_t i=0; i<sdlogger.buffer_addr; i++) {
    TEST_ASSERT_EQUAL_HEX8(0xDD, sdcard1.output_buf[1+i]);
  }
  /* Fill with trailing zeros */
  for (uint32_t i=sdlogger.buffer_addr; i<(SD_LOGGER_BUFFER_OFFSET + SD_BLOCK_SIZE); i++) {
    TEST_ASSERT_EQUAL_HEX8(0x00, sdcard1.output_buf[1+i]);
  }

  TEST_ASSERT_EQUAL(SdLogger_StopLogging, sdlogger.status);

  /* Always reset command */
  TEST_ASSERT_EQUAL_HEX8(SdLoggerCmd_Nothing, sdlogger.cmd);
}

void test_RejectStopCommandIfNotStarted(void)
{
  sdlogger.status = SdLogger_Idle;
  sdlogger.cmd = SdLoggerCmd_StopLogging;

  /* Call the command function */
  sd_logger_command();

  TEST_ASSERT_EQUAL(SdLogger_Idle, sdlogger.status);

  /* Always reset command */
  TEST_ASSERT_EQUAL_HEX8(SdLoggerCmd_Nothing, sdlogger.cmd);
}

void test_StopLoggingAfterLastMultWriteTransferIsDone(void)
{
  sdlogger.status = SdLogger_StopLogging;
  sdcard1.status = SDCard_MultiWriteIdle;

  /* Expect stop command */
  sdcard_spi_multiwrite_stop_Expect(&sdcard1);

  /* Check periodically if last transfer is done */
  helper_ExpectSdLoggerPeriodic();

  TEST_ASSERT_EQUAL(SdLogger_WriteStatusPacket, sdlogger.status);
}

void test_StopLoggingLastTransferNotFinishedYet(void)
{
  sdlogger.status = SdLogger_StopLogging;
  sdcard1.status = SDCard_MultiWriteBusy;

  helper_ExpectSdLoggerPeriodic();

  /* Nothing changes */
  TEST_ASSERT_EQUAL(SdLogger_StopLogging, sdlogger.status);
}

void helper_CompareInt32FromAddress(int32_t value, uint8_t *ptr, const char *line)
{
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(value >> 24, ptr[0], line);
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(value >> 16, ptr[1], line);
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(value >> 8, ptr[2], line);
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(value >> 0, ptr[3], line);
}

void helper_CompareUInt32FromAddress(uint32_t value, uint8_t *ptr, const char *line)
{
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(value >> 24, ptr[0], line);
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(value >> 16, ptr[1], line);
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(value >> 8, ptr[2], line);
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(value >> 0, ptr[3], line);
}

void test_LoggingDataBufferNotFull(void)
{
  sdlogger.status = SdLogger_Logging;
  sdlogger.block_addr = 0x00001234;
  sdlogger.buffer_addr = 47; /* not at the end */
  sdlogger.packet_count = 20;

  imu.accel_unscaled.x = 111222333;
  imu.accel_unscaled.y = 444555666;
  imu.accel_unscaled.z = 777888999;
  imu.gyro_unscaled.p = 123123123;
  imu.gyro_unscaled.q = 456456456;
  imu.gyro_unscaled.r = 789789789;
  actuators_pwm_values[0] = 321321321;
  actuators_pwm_values[2] = 654654654;
  actuators_pwm_values[3] = 987987987;
  actuators_pwm_values[4] = 159159159;
  actuators_pwm_values[5] = 753753753;

  /* Call the periodic loop */
  helper_ExpectSdLoggerPeriodic();

  TEST_ASSERT_EQUAL(21, sdlogger.packet_count); /* Increment 1 */
  TEST_ASSERT_EQUAL(47+(13*4), sdlogger.buffer_addr); /* Added 13*4 bytes */
  helper_CompareUInt32FromAddress(21, &sdcard1.output_buf[1+47], S__LINE__);
  helper_CompareInt32FromAddress(111222333, &sdcard1.output_buf[1+47+4], S__LINE__);
  helper_CompareInt32FromAddress(444555666, &sdcard1.output_buf[1+47+8], S__LINE__);
  helper_CompareInt32FromAddress(777888999, &sdcard1.output_buf[1+47+12], S__LINE__);
  helper_CompareInt32FromAddress(123123123, &sdcard1.output_buf[1+47+16], S__LINE__);
  helper_CompareInt32FromAddress(456456456, &sdcard1.output_buf[1+47+20], S__LINE__);
  helper_CompareInt32FromAddress(789789789, &sdcard1.output_buf[1+47+24], S__LINE__);
  helper_CompareInt32FromAddress(321321321, &sdcard1.output_buf[1+47+28], S__LINE__);
  helper_CompareInt32FromAddress(654654654, &sdcard1.output_buf[1+47+32], S__LINE__);
  helper_CompareInt32FromAddress(987987987, &sdcard1.output_buf[1+47+36], S__LINE__);
  helper_CompareInt32FromAddress(159159159, &sdcard1.output_buf[1+47+40], S__LINE__);
  helper_CompareInt32FromAddress(753753753, &sdcard1.output_buf[1+47+44], S__LINE__);
  helper_CompareInt32FromAddress(0, &sdcard1.output_buf[1+47+48], S__LINE__); /* reserved */
}


void test_LoggingDataBufferFullWriteToSD(void)
{
  sdlogger.status = SdLogger_Logging;
  sdlogger.block_addr = 0x00001234;
  sdlogger.buffer_addr = 440; /* After next, buffer is full. Only possible to add once more 40 bytes */
  sdlogger.packet_count = 51; /* Arbitrary number */
  sdlogger.unique_id = 0xCAFEBABE;

  sdcard1.status = SDCard_MultiWriteIdle;
  sdlogger.error_count = 57; /* Should stay the same */

  imu.accel_unscaled.x = 111222333;
  imu.accel_unscaled.y = 444555666;
  imu.accel_unscaled.z = 777888999;
  imu.gyro_unscaled.p = 123123123;
  imu.gyro_unscaled.q = 456456456;
  imu.gyro_unscaled.r = 789789789;
  actuators_pwm_values[0] = 321321321;
  actuators_pwm_values[2] = 654654654;
  actuators_pwm_values[3] = 987987987;
  actuators_pwm_values[4] = 159159159;
  actuators_pwm_values[5] = 753753753;

  /* Expection a write action on the SD card since buffer will be full after this one. */
  sdcard_spi_multiwrite_next_Expect(&sdcard1);

  /* Call the periodic loop */
  helper_ExpectSdLoggerPeriodic();

  helper_CompareUInt32FromAddress(0xCAFEBABE, &sdcard1.output_buf[1], S__LINE__);

  /* Value resets to zero because buffer is written to SD and new buffer will be filled. */
  TEST_ASSERT_EQUAL(4, sdlogger.buffer_addr);

  /* Adding 1 to the counter and also the block address */
  TEST_ASSERT_EQUAL(52, sdlogger.packet_count);
  TEST_ASSERT_EQUAL_HEX32(0x00001235, sdlogger.block_addr);

  /* No errors occured */
  TEST_ASSERT_EQUAL(57, sdlogger.error_count);

  helper_CompareInt32FromAddress(52, &sdcard1.output_buf[1+440], S__LINE__);
  helper_CompareInt32FromAddress(111222333, &sdcard1.output_buf[1+440+4], S__LINE__);
  helper_CompareInt32FromAddress(444555666, &sdcard1.output_buf[1+440+8], S__LINE__);
  helper_CompareInt32FromAddress(777888999, &sdcard1.output_buf[1+440+12], S__LINE__);
  helper_CompareInt32FromAddress(123123123, &sdcard1.output_buf[1+440+16], S__LINE__);
  helper_CompareInt32FromAddress(456456456, &sdcard1.output_buf[1+440+20], S__LINE__);
  helper_CompareInt32FromAddress(789789789, &sdcard1.output_buf[1+440+24], S__LINE__);
  helper_CompareInt32FromAddress(321321321, &sdcard1.output_buf[1+440+28], S__LINE__);
  helper_CompareInt32FromAddress(654654654, &sdcard1.output_buf[1+440+32], S__LINE__);
  helper_CompareInt32FromAddress(987987987, &sdcard1.output_buf[1+440+36], S__LINE__);
  helper_CompareInt32FromAddress(159159159, &sdcard1.output_buf[1+440+40], S__LINE__);
  helper_CompareInt32FromAddress(753753753, &sdcard1.output_buf[1+440+44], S__LINE__);
  helper_CompareInt32FromAddress(0, &sdcard1.output_buf[1+440+48], S__LINE__); /* reserved */
}

void test_IncrementErrorCountWhenWritingWhileSdBusy(void)
{
  sdlogger.status = SdLogger_Logging;
  sdlogger.block_addr = 0x00001234;
  sdlogger.buffer_addr = 440; /* After next, buffer is full. Only possible to add once more 40 bytes */
  sdlogger.packet_count = 51; /* Arbitrary */

  sdcard1.status = SDCard_MultiWriteBusy;
  sdlogger.error_count = 57;

  /* Expection a write action on the SD card since buffer will be full after this one. */
  sdcard_spi_multiwrite_next_Expect(&sdcard1);

  /* Call the periodic loop */
  helper_ExpectSdLoggerPeriodic();

  /* Value resets to zero because buffer is written to SD and new buffer will be filled. */
  TEST_ASSERT_EQUAL(4, sdlogger.buffer_addr);
  TEST_ASSERT_EQUAL_HEX32(0x00001235, sdlogger.block_addr);

  /* Because card was busy, increase error */
  TEST_ASSERT_EQUAL(58, sdlogger.error_count);

}

/**
 * Logging just stopped, waiting for sdcard to be idle to write the last block (summary)
 */
void test_WriteSummaryBlockSdCardNotReady(void)
{
  sdlogger.status = SdLogger_WriteStatusPacket;
  sdcard1.status = SDCard_Busy;

  /* Call periodic loop */
  helper_ExpectSdLoggerPeriodic();
}

void test_WriteSummaryBlockSdCardReady(void)
{
  sdlogger.status = SdLogger_WriteStatusPacket;
  sdcard1.status = SDCard_Idle; /* Ready to accept new write command */
  sdlogger.block_addr = 0x1234BEEF; /* Random, should not matter */
  sdlogger.buffer_addr = 0x22; /* Random, should not matter */
  sdlogger.error_count = 0xBEEFBEEF;
  sdlogger.packet_count = 0xABBACAFE;
  sdlogger.unique_id = 0xCAFEBABE;

  /* Expect call to write summary data block at first address */
  sdcard_spi_write_block_Expect(&sdcard1, 0x00000000);

  /* Call periodic loop */
  helper_ExpectSdLoggerPeriodic();

  helper_CompareUInt32FromAddress(0xABBACAFE, &sdcard1.output_buf[6], S__LINE__);
  helper_CompareUInt32FromAddress(0xBEEFBEEF, &sdcard1.output_buf[6+4], S__LINE__);
  helper_CompareUInt32FromAddress(0xCAFEBABE, &sdcard1.output_buf[6+8], S__LINE__);
  for (uint16_t i=12; i<SD_BLOCK_SIZE; i++) {
    TEST_ASSERT_EQUAL_HEX8(0x00, sdcard1.output_buf[6+i]);
  }

  TEST_ASSERT_EQUAL(SdLogger_Idle, sdlogger.status);
}

void helper_ExpectSendLogPacket(void)
{
  testable_pprz_msg_send_LOG_DATAPACKET_Expect(&pprz_tp.trans_tx, &uart1.device, 5, &sdlogger.packet.time, &sdlogger.packet.data_1, &sdlogger.packet.data_2, &sdlogger.packet.data_3, &sdlogger.packet.data_4, &sdlogger.packet.data_5, &sdlogger.packet.data_6, &sdlogger.packet.data_7, &sdlogger.packet.data_8, &sdlogger.packet.data_9, &sdlogger.packet.data_10, &sdlogger.packet.data_11, &sdlogger.packet.data_12);
  testable_pprz_msg_send_LOG_DATAPACKET_IgnoreArg_ac_id();
}

void helper_WriteInt32ToAddress(int32_t value, uint8_t *ptr)
{
  ptr[0] = value >> 24;
  ptr[1] = value >> 16;
  ptr[2] = value >> 8;
  ptr[3] = value >> 0;
}

void helper_WriteUInt32ToAddress(uint32_t value, uint8_t *ptr)
{
  ptr[0] = value >> 24;
  ptr[1] = value >> 16;
  ptr[2] = value >> 8;
  ptr[3] = value >> 0;
}

/**
 * The first log packet on block 0 is the status packet
 * Only possible if not logging
 */
void test_RequestStatusPacket(void)
{
  sdlogger.status = SdLogger_DataAvailable;
  sdlogger.block_addr = 0x00000000;
  sdlogger.cmd = SdLoggerCmd_RequestStatusPacket;
  helper_ExpectSendLogPacket();

  helper_WriteUInt32ToAddress(12341234, &sdcard1.input_buf[0]);
  helper_WriteInt32ToAddress(123123, &sdcard1.input_buf[4]);
  helper_WriteInt32ToAddress(456456, &sdcard1.input_buf[8]);
  helper_WriteInt32ToAddress(789789, &sdcard1.input_buf[12]);
  helper_WriteInt32ToAddress(112233, &sdcard1.input_buf[16]);
  helper_WriteInt32ToAddress(445566, &sdcard1.input_buf[20]);
  helper_WriteInt32ToAddress(778899, &sdcard1.input_buf[24]);
  helper_WriteInt32ToAddress(321321, &sdcard1.input_buf[28]);
  helper_WriteInt32ToAddress(654654, &sdcard1.input_buf[32]);
  helper_WriteInt32ToAddress(987987, &sdcard1.input_buf[36]);
  helper_WriteInt32ToAddress(753753, &sdcard1.input_buf[40]);
  helper_WriteInt32ToAddress(159159, &sdcard1.input_buf[44]);
  helper_WriteInt32ToAddress(951951, &sdcard1.input_buf[48]);

  /* Call the command function */
  sd_logger_command();

  TEST_ASSERT_EQUAL(12341234, sdlogger.packet.time);
  TEST_ASSERT_EQUAL(123123, sdlogger.packet.data_1);
  TEST_ASSERT_EQUAL(456456, sdlogger.packet.data_2);
  TEST_ASSERT_EQUAL(789789, sdlogger.packet.data_3);
  TEST_ASSERT_EQUAL(112233, sdlogger.packet.data_4);
  TEST_ASSERT_EQUAL(445566, sdlogger.packet.data_5);
  TEST_ASSERT_EQUAL(778899, sdlogger.packet.data_6);
  TEST_ASSERT_EQUAL(321321, sdlogger.packet.data_7);
  TEST_ASSERT_EQUAL(654654, sdlogger.packet.data_8);
  TEST_ASSERT_EQUAL(987987, sdlogger.packet.data_9);
  TEST_ASSERT_EQUAL(753753, sdlogger.packet.data_10);
  TEST_ASSERT_EQUAL(159159, sdlogger.packet.data_11);
  TEST_ASSERT_EQUAL(951951, sdlogger.packet.data_12);
  TEST_ASSERT_EQUAL_HEX8(SdLoggerCmd_Nothing, sdlogger.cmd); /* Always reset */
}

/**
 * Data not ready in buffer, first read the data from sd card. Use callback to send log packet
 */
void test_RequestStatusPacketNotInBuffer(void)
{
  sdlogger.status = SdLogger_DataAvailable; /* Should also work with Idle */
  sdlogger.cmd = SdLoggerCmd_RequestStatusPacket;
  sdlogger.block_addr = 0x0000CAFE; /* Note that the wrong address is available */

  /* Using callback function to proceed when data is available */
  sdcard_spi_read_block_Expect(&sdcard1, 0x00000000, &sd_logger_statusblock_ready);

  /* Call the command function */
  sd_logger_command();

  TEST_ASSERT_EQUAL_HEX8(SdLoggerCmd_Nothing, sdlogger.cmd); /* Always reset */
}

void test_RequestStatusPacketNotIdle(void)
{
  sdlogger.status = SdLogger_Logging; /* Not Idle nor DataAvailable */
  sdlogger.cmd = SdLoggerCmd_RequestStatusPacket;

  /* Call the command function */
  sd_logger_command();

  /* Expecting no calls */

  /* Always reset command */
  TEST_ASSERT_EQUAL_HEX8(SdLoggerCmd_Nothing, sdlogger.cmd);
}

void test_SendStatusPacketByCallback(void)
{
  sdlogger.status = SdLogger_Idle; /* Should also work with DataAvailable */
  sdlogger.cmd = SdLoggerCmd_Nothing; /* There is no command associated with this */
  sdlogger.block_addr = 0x0000CAFE;
  helper_ExpectSendLogPacket();

  helper_WriteUInt32ToAddress(12341234, &sdcard1.input_buf[0]);
  helper_WriteInt32ToAddress(123123, &sdcard1.input_buf[4]);
  helper_WriteInt32ToAddress(456456, &sdcard1.input_buf[8]);
  helper_WriteInt32ToAddress(789789, &sdcard1.input_buf[12]);
  helper_WriteInt32ToAddress(112233, &sdcard1.input_buf[16]);
  helper_WriteInt32ToAddress(445566, &sdcard1.input_buf[20]);
  helper_WriteInt32ToAddress(778899, &sdcard1.input_buf[24]);
  helper_WriteInt32ToAddress(321321, &sdcard1.input_buf[28]);
  helper_WriteInt32ToAddress(654654, &sdcard1.input_buf[32]);
  helper_WriteInt32ToAddress(987987, &sdcard1.input_buf[36]);
  helper_WriteInt32ToAddress(753753, &sdcard1.input_buf[40]);
  helper_WriteInt32ToAddress(159159, &sdcard1.input_buf[44]);
  helper_WriteInt32ToAddress(951951, &sdcard1.input_buf[48]);

  /* Call the function */
  sd_logger_statusblock_ready();

  TEST_ASSERT_EQUAL(12341234, sdlogger.packet.time);
  TEST_ASSERT_EQUAL(123123, sdlogger.packet.data_1);
  TEST_ASSERT_EQUAL(456456, sdlogger.packet.data_2);
  TEST_ASSERT_EQUAL(789789, sdlogger.packet.data_3);
  TEST_ASSERT_EQUAL(112233, sdlogger.packet.data_4);
  TEST_ASSERT_EQUAL(445566, sdlogger.packet.data_5);
  TEST_ASSERT_EQUAL(778899, sdlogger.packet.data_6);
  TEST_ASSERT_EQUAL(321321, sdlogger.packet.data_7);
  TEST_ASSERT_EQUAL(654654, sdlogger.packet.data_8);
  TEST_ASSERT_EQUAL(987987, sdlogger.packet.data_9);
  TEST_ASSERT_EQUAL(753753, sdlogger.packet.data_10);
  TEST_ASSERT_EQUAL(159159, sdlogger.packet.data_11);
  TEST_ASSERT_EQUAL(951951, sdlogger.packet.data_12);

  TEST_ASSERT_EQUAL(0x00000000, sdlogger.block_addr);
  TEST_ASSERT_EQUAL(SdLogger_DataAvailable, sdlogger.status);

  /* Always reset command */
  TEST_ASSERT_EQUAL_HEX8(SdLoggerCmd_Nothing, sdlogger.cmd);
}

void test_RequestPacketFromBuffer(void)
{
  sdlogger.unique_id = 0xCAFEBABE;
  sdlogger.status = SdLogger_DataAvailable;
  sdlogger.block_addr = 0x00000002;
  sdlogger.cmd = SdLoggerCmd_Nothing;
  sdlogger.request_id = 11; /* 2nd packet at block address 2 */
  helper_ExpectSendLogPacket();

  helper_WriteUInt32ToAddress(0xCAFEBABE, &sdcard1.input_buf[0]);
  helper_WriteUInt32ToAddress(12341234, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+0]);
  helper_WriteInt32ToAddress(123123, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+4]);
  helper_WriteInt32ToAddress(456456, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+8]);
  helper_WriteInt32ToAddress(789789, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+12]);
  helper_WriteInt32ToAddress(112233, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+16]);
  helper_WriteInt32ToAddress(445566, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+20]);
  helper_WriteInt32ToAddress(778899, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+24]);
  helper_WriteInt32ToAddress(321321, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+28]);
  helper_WriteInt32ToAddress(654654, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+32]);
  helper_WriteInt32ToAddress(987987, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+36]);
  helper_WriteInt32ToAddress(753753, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+40]);
  helper_WriteInt32ToAddress(159159, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+44]);
  helper_WriteInt32ToAddress(951951, &sdcard1.input_buf[4+SD_LOGGER_PACKET_SIZE+48]);

  /* Command callback */
  sd_logger_command();

  TEST_ASSERT_EQUAL(12341234, sdlogger.packet.time);
  TEST_ASSERT_EQUAL(123123, sdlogger.packet.data_1);
  TEST_ASSERT_EQUAL(456456, sdlogger.packet.data_2);
  TEST_ASSERT_EQUAL(789789, sdlogger.packet.data_3);
  TEST_ASSERT_EQUAL(112233, sdlogger.packet.data_4);
  TEST_ASSERT_EQUAL(445566, sdlogger.packet.data_5);
  TEST_ASSERT_EQUAL(778899, sdlogger.packet.data_6);
  TEST_ASSERT_EQUAL(321321, sdlogger.packet.data_7);
  TEST_ASSERT_EQUAL(654654, sdlogger.packet.data_8);
  TEST_ASSERT_EQUAL(987987, sdlogger.packet.data_9);
  TEST_ASSERT_EQUAL(753753, sdlogger.packet.data_10);
  TEST_ASSERT_EQUAL(159159, sdlogger.packet.data_11);
  TEST_ASSERT_EQUAL(951951, sdlogger.packet.data_12);

}

void test_ParseRequestOnlyGreaterThanZero(void)
{
  sdlogger.status = SdLogger_Idle;
  sdlogger.request_id = 0;
  sdlogger.cmd = SdLoggerCmd_Nothing;

  /* One of two parameters set, but both are zero */
  sd_logger_command();

  /* Expect no calls or whatsoever */
}

void test_ParsePacketRequestOnlyIfIdleOrDataAvailable(void)
{
  sdlogger.status = SdLogger_Logging; /* Not idle nor data available */
  sdlogger.request_id = 15;
  sdlogger.cmd = SdLoggerCmd_Nothing;

  sd_logger_command();

  /* Expect no calls */
}

void test_RequestAnotherPacketFromBuffer(void)
{
  sdlogger.unique_id = 0xCAFEBABE;
  sdlogger.status = SdLogger_DataAvailable;
  sdlogger.block_addr = 0x00000002;
  sdlogger.cmd = SdLoggerCmd_Nothing;
  sdlogger.request_id = 12; /* 3rd packet at block 2 */
  helper_ExpectSendLogPacket();

  helper_WriteUInt32ToAddress(0xCAFEBABE, &sdcard1.input_buf[0]);
  helper_WriteUInt32ToAddress(12341234, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+0]);
  helper_WriteInt32ToAddress(123123, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+4]);
  helper_WriteInt32ToAddress(456456, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+8]);
  helper_WriteInt32ToAddress(789789, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+12]);
  helper_WriteInt32ToAddress(112233, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+16]);
  helper_WriteInt32ToAddress(445566, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+20]);
  helper_WriteInt32ToAddress(778899, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+24]);
  helper_WriteInt32ToAddress(321321, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+28]);
  helper_WriteInt32ToAddress(654654, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+32]);
  helper_WriteInt32ToAddress(987987, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+36]);
  helper_WriteInt32ToAddress(159159, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+40]);
  helper_WriteInt32ToAddress(951951, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+44]);
  helper_WriteInt32ToAddress(753753, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+48]);

  /* Command callback */
  sd_logger_command();

  TEST_ASSERT_EQUAL(12341234, sdlogger.packet.time);
  TEST_ASSERT_EQUAL(123123, sdlogger.packet.data_1);
  TEST_ASSERT_EQUAL(456456, sdlogger.packet.data_2);
  TEST_ASSERT_EQUAL(789789, sdlogger.packet.data_3);
  TEST_ASSERT_EQUAL(112233, sdlogger.packet.data_4);
  TEST_ASSERT_EQUAL(445566, sdlogger.packet.data_5);
  TEST_ASSERT_EQUAL(778899, sdlogger.packet.data_6);
  TEST_ASSERT_EQUAL(321321, sdlogger.packet.data_7);
  TEST_ASSERT_EQUAL(654654, sdlogger.packet.data_8);
  TEST_ASSERT_EQUAL(987987, sdlogger.packet.data_9);
  TEST_ASSERT_EQUAL(159159, sdlogger.packet.data_10);
  TEST_ASSERT_EQUAL(951951, sdlogger.packet.data_11);
  TEST_ASSERT_EQUAL(753753, sdlogger.packet.data_12);
}

void test_RequestLastPacketFromBuffer(void)
{
  sdlogger.unique_id = 0xCAFEBABE;
  sdlogger.status = SdLogger_DataAvailable;
  sdlogger.block_addr = 0x00000001;
  sdlogger.cmd = SdLoggerCmd_Nothing;
  sdlogger.request_id = 9; /* last packet at block 1 */
  helper_ExpectSendLogPacket();

  helper_WriteUInt32ToAddress(0xCAFEBABE, &sdcard1.input_buf[0]);
  helper_WriteUInt32ToAddress(12341234, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+0]);
  helper_WriteInt32ToAddress(123123, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+4]);
  helper_WriteInt32ToAddress(456456, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+8]);
  helper_WriteInt32ToAddress(789789, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+12]);
  helper_WriteInt32ToAddress(112233, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+16]);
  helper_WriteInt32ToAddress(445566, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+20]);
  helper_WriteInt32ToAddress(778899, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+24]);
  helper_WriteInt32ToAddress(321321, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+28]);
  helper_WriteInt32ToAddress(654654, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+32]);
  helper_WriteInt32ToAddress(987987, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+36]);
  helper_WriteInt32ToAddress(159159, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+40]);
  helper_WriteInt32ToAddress(951951, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+44]);
  helper_WriteInt32ToAddress(753753, &sdcard1.input_buf[4+8*SD_LOGGER_PACKET_SIZE+48]);

  /* Command callback */
  sd_logger_command();

  TEST_ASSERT_EQUAL(12341234, sdlogger.packet.time);
  TEST_ASSERT_EQUAL(123123, sdlogger.packet.data_1);
  TEST_ASSERT_EQUAL(456456, sdlogger.packet.data_2);
  TEST_ASSERT_EQUAL(789789, sdlogger.packet.data_3);
  TEST_ASSERT_EQUAL(112233, sdlogger.packet.data_4);
  TEST_ASSERT_EQUAL(445566, sdlogger.packet.data_5);
  TEST_ASSERT_EQUAL(778899, sdlogger.packet.data_6);
  TEST_ASSERT_EQUAL(321321, sdlogger.packet.data_7);
  TEST_ASSERT_EQUAL(654654, sdlogger.packet.data_8);
  TEST_ASSERT_EQUAL(987987, sdlogger.packet.data_9);
  TEST_ASSERT_EQUAL(159159, sdlogger.packet.data_10);
  TEST_ASSERT_EQUAL(951951, sdlogger.packet.data_11);
  TEST_ASSERT_EQUAL(753753, sdlogger.packet.data_12);
}

void test_ReplyWithZerosIfUniqueIdMismatch(void)
{
  sdlogger.unique_id = 0xCAFEBABE;
  sdlogger.status = SdLogger_DataAvailable;
  sdlogger.block_addr = 0x00000001;
  sdlogger.cmd = SdLoggerCmd_Nothing;
  sdlogger.request_id = 9; /* last packet at block 1 */
  helper_ExpectSendLogPacket();

  helper_WriteUInt32ToAddress(0x0000BEEF, &sdcard1.input_buf[0]); /* Not matching with unique_id */

  /* Command callback */
  sd_logger_command();

  TEST_ASSERT_EQUAL(0, sdlogger.packet.time);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_1);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_2);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_3);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_4);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_5);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_6);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_7);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_8);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_9);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_10);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_11);
  TEST_ASSERT_EQUAL(0, sdlogger.packet.data_12);
}

void test_RequestPacketFromAnotherBlock(void)
{
  sdlogger.status = SdLogger_DataAvailable;
  sdlogger.block_addr = 0x00000001; /* wrong block */
  sdlogger.cmd = SdLoggerCmd_Nothing;
  sdlogger.request_id = 10; /* 1st packet at block 2 */

  /* Using callback function to proceed when data is available */
  sdcard_spi_read_block_Expect(&sdcard1, 0x00000002, &sd_logger_packetblock_ready);

  sd_logger_command();

  TEST_ASSERT_EQUAL(SdLogger_ReadingBlock, sdlogger.status);
  TEST_ASSERT_EQUAL(0, sdlogger.timeout_counter);
  TEST_ASSERT_EQUAL(0x00000002, sdlogger.block_addr);
}

/**
 * Also read the block (again) if status is idle
 */
void test_RequestPacketFromBlockWhenIdle(void)
{
  sdlogger.status = SdLogger_Idle;
  sdlogger.block_addr = 0x00000002; /* correct block, but still read it again because we're not in dataready */
  sdlogger.cmd = SdLoggerCmd_Nothing;
  sdlogger.request_id = 13;

  /* Using callback function to proceed when data is available */
  sdcard_spi_read_block_Expect(&sdcard1, 0x00000002, &sd_logger_packetblock_ready);

  sd_logger_command();

  TEST_ASSERT_EQUAL(SdLogger_ReadingBlock, sdlogger.status);
  TEST_ASSERT_EQUAL(0, sdlogger.timeout_counter);
  TEST_ASSERT_EQUAL(0x00000002, sdlogger.block_addr);
}

/**
 * If the block reading goes wrong, the callback will never be called. Then, revert to idle state
 */
void test_ReadingBlockTimesOutWhenNotResponding(void)
{
  sdlogger.status = SdLogger_ReadingBlock;
  sdlogger.timeout_counter = 199;

  /* Periodic loop */
  helper_ExpectSdLoggerPeriodic();

  TEST_ASSERT_EQUAL(SdLogger_Idle, sdlogger.status);
  TEST_ASSERT_EQUAL(0, sdlogger.timeout_counter);
}

void test_ReadingBlockNotTimingOutBefore200Cycles(void)
{
  sdlogger.status = SdLogger_ReadingBlock;
  sdlogger.timeout_counter = 198;

  /* Periodic loop */
  helper_ExpectSdLoggerPeriodic();

  TEST_ASSERT_EQUAL(SdLogger_ReadingBlock, sdlogger.status);
  TEST_ASSERT_EQUAL(199, sdlogger.timeout_counter);
}

void test_CallbackProcessingPacketRequest(void)
{
  sdlogger.unique_id = 0xCAFEBABE;
  sdlogger.status = SdLogger_ReadingBlock;
  sdlogger.request_id = 10; /* 1st packet at block 2 */

  helper_WriteUInt32ToAddress(0xCAFEBABE, &sdcard1.input_buf[0]);
  helper_WriteUInt32ToAddress(12341234, &sdcard1.input_buf[4+0]);
  helper_WriteInt32ToAddress(123123, &sdcard1.input_buf[4+4]);
  helper_WriteInt32ToAddress(456456, &sdcard1.input_buf[4+8]);
  helper_WriteInt32ToAddress(789789, &sdcard1.input_buf[4+12]);
  helper_WriteInt32ToAddress(112233, &sdcard1.input_buf[4+16]);
  helper_WriteInt32ToAddress(445566, &sdcard1.input_buf[4+20]);
  helper_WriteInt32ToAddress(778899, &sdcard1.input_buf[4+24]);
  helper_WriteInt32ToAddress(321321, &sdcard1.input_buf[4+28]);
  helper_WriteInt32ToAddress(654654, &sdcard1.input_buf[4+32]);
  helper_WriteInt32ToAddress(987987, &sdcard1.input_buf[4+36]);
  helper_WriteInt32ToAddress(753753, &sdcard1.input_buf[4+40]);
  helper_WriteInt32ToAddress(159159, &sdcard1.input_buf[4+44]);
  helper_WriteInt32ToAddress(951951, &sdcard1.input_buf[4+48]);

  helper_ExpectSendLogPacket();

  /* The callback from sdcard read */
  sd_logger_packetblock_ready();

  TEST_ASSERT_EQUAL(12341234, sdlogger.packet.time);
  TEST_ASSERT_EQUAL(123123, sdlogger.packet.data_1);
  TEST_ASSERT_EQUAL(456456, sdlogger.packet.data_2);
  TEST_ASSERT_EQUAL(789789, sdlogger.packet.data_3);
  TEST_ASSERT_EQUAL(112233, sdlogger.packet.data_4);
  TEST_ASSERT_EQUAL(445566, sdlogger.packet.data_5);
  TEST_ASSERT_EQUAL(778899, sdlogger.packet.data_6);
  TEST_ASSERT_EQUAL(321321, sdlogger.packet.data_7);
  TEST_ASSERT_EQUAL(654654, sdlogger.packet.data_8);
  TEST_ASSERT_EQUAL(987987, sdlogger.packet.data_9);
  TEST_ASSERT_EQUAL(753753, sdlogger.packet.data_10);
  TEST_ASSERT_EQUAL(159159, sdlogger.packet.data_11);
  TEST_ASSERT_EQUAL(951951, sdlogger.packet.data_12);
  TEST_ASSERT_EQUAL(SdLogger_DataAvailable, sdlogger.status);

}

void test_CallbackProcessingPacketRequestAnother(void)
{
  sdlogger.unique_id = 0xCAFEBABE;
  sdlogger.status = SdLogger_ReadingBlock;
  sdlogger.request_id = 12; /* 3rd packet at block 2 */

  helper_WriteUInt32ToAddress(0xCAFEBABE, &sdcard1.input_buf[0]);
  helper_WriteUInt32ToAddress(12341234, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+0]);
  helper_WriteInt32ToAddress(123123, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+4]);
  helper_WriteInt32ToAddress(456456, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+8]);
  helper_WriteInt32ToAddress(789789, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+12]);
  helper_WriteInt32ToAddress(112233, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+16]);
  helper_WriteInt32ToAddress(445566, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+20]);
  helper_WriteInt32ToAddress(778899, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+24]);
  helper_WriteInt32ToAddress(321321, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+28]);
  helper_WriteInt32ToAddress(654654, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+32]);
  helper_WriteInt32ToAddress(987987, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+36]);
  helper_WriteInt32ToAddress(753753, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+40]);
  helper_WriteInt32ToAddress(159159, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+44]);
  helper_WriteInt32ToAddress(951951, &sdcard1.input_buf[4+2*SD_LOGGER_PACKET_SIZE+48]);
  helper_ExpectSendLogPacket();

  /* The callback from sdcard read */
  sd_logger_packetblock_ready();

  TEST_ASSERT_EQUAL(12341234, sdlogger.packet.time);
  TEST_ASSERT_EQUAL(123123, sdlogger.packet.data_1);
  TEST_ASSERT_EQUAL(456456, sdlogger.packet.data_2);
  TEST_ASSERT_EQUAL(789789, sdlogger.packet.data_3);
  TEST_ASSERT_EQUAL(112233, sdlogger.packet.data_4);
  TEST_ASSERT_EQUAL(445566, sdlogger.packet.data_5);
  TEST_ASSERT_EQUAL(778899, sdlogger.packet.data_6);
  TEST_ASSERT_EQUAL(321321, sdlogger.packet.data_7);
  TEST_ASSERT_EQUAL(654654, sdlogger.packet.data_8);
  TEST_ASSERT_EQUAL(987987, sdlogger.packet.data_9);
  TEST_ASSERT_EQUAL(753753, sdlogger.packet.data_10);
  TEST_ASSERT_EQUAL(159159, sdlogger.packet.data_11);
  TEST_ASSERT_EQUAL(951951, sdlogger.packet.data_12);
  TEST_ASSERT_EQUAL(SdLogger_DataAvailable, sdlogger.status);
}
