/*
 * Copyright (C) 2015 Bart Slinger <bartslinger@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file tests/unittest/sw/airborne/peripherals/sdcard_spi_tester.c
 *  @brief Test code for sdcard_spi using unity and cmock.
 */

/* By prepending "Mock" to an include, a mock object is generated automatically by cmock. */
#include "unity.h"
#include "mcu_periph/Mockspi.h"
#include "peripherals/sdcard_spi.h"

/* Variable to check if the spi_submit stub was called */
uint8_t SpiSubmitNrCalls;

/* Boolean to check if the sdcard.read_callback was called */
bool_t CallbackWasCalled;

/* Is 1 by default, but can be more. Used in SpiSubmitCall_RequestNBytes() */
uint8_t NBytesToRequest;

/* Declared in spi.c during normal operation */
struct spi_periph spi2;

/* Declared in sdcard_spi.c during normal operation */
struct SDCard sdcard1;

/* Struct to revert to orginial state before each unit test */
struct SDCard sdcard_original;

/**
 * @brief Called before each test by the unity framework
 */
void setUp(void)
{
  /* Remember initial state */
  sdcard_original = sdcard1;

  /* Reset counter to keep track of calls */
  SpiSubmitNrCalls = 0;

  /* Reset to keep track of calls */
  CallbackWasCalled = FALSE;

  /* In SpiSubmitCall_RequestNBytes(), request 1 byte by default */
  NBytesToRequest = 1;

  /* Initialize Mock spi interface */
  Mockspi_Init();

  /* The init function should called before use of any other function.
   * In normal operation, it is called by the user of the sdcard, for example the sd_logger. */
  sdcard_spi_init(&sdcard1, &spi2, SPI_SLAVE3); /* Works also with other peripheral or slave */


  sdcard1.response_counter = 57; /* Non-zero value to make sure this gets set to zero everywhere */
  sdcard1.timeout_counter = 5700; /* Non-zero value to make sure this gets set to zero everywhere */
}

/**
 * @brief Called after each test by the unity framework
 */
void tearDown(void)
{
  /* revert back to original state */
  sdcard1 = sdcard_original;

  /* Handle spi mock object after each test */
  Mockspi_Verify();
  Mockspi_Destroy();
}

/**
 * @brief Test that initial values are set correctly
 */
void test_SdCardInitializeStructInitialValues(void)
{
  /* First, set some random non-zero variables to the values in the struct */
  struct spi_periph random_spip;
  sdcard1.spi_p = &random_spip;
  sdcard1.status = 0x57;
  sdcard1.spi_t.slave_idx = 0x57;
  sdcard1.spi_t.select = 0x57;
  sdcard1.spi_t.status = 0x57;
  sdcard1.spi_t.cpol = 0x57;
  sdcard1.spi_t.cpha = 0x57;
  sdcard1.spi_t.dss = 0x57;
  sdcard1.spi_t.bitorder = 0x57;
  sdcard1.spi_t.cdiv = 0x57;
  sdcard1.spi_t.input_buf = NULL;
  sdcard1.spi_t.output_buf = NULL;
  sdcard1.spi_t.input_length = 57;
  sdcard1.spi_t.output_length = 57;
  sdcard1.card_type = 57;


  /* Call the function */
  sdcard_spi_init(&sdcard1, &spi2, SPI_SLAVE3);

  /* Then, verify the initial values after initialization are correct */
  TEST_ASSERT_EQUAL_PTR(&spi2, sdcard1.spi_p);
  TEST_ASSERT_EQUAL(SPI_SLAVE3, sdcard1.spi_t.slave_idx);
  TEST_ASSERT_EQUAL(SPISelectUnselect, sdcard1.spi_t.select);
  TEST_ASSERT_EQUAL(SPITransDone, sdcard1.spi_t.status);
  TEST_ASSERT_EQUAL(SPICpolIdleLow, sdcard1.spi_t.cpol);
  TEST_ASSERT_EQUAL(SPICphaEdge1, sdcard1.spi_t.cpha);
  TEST_ASSERT_EQUAL(SPIDss8bit, sdcard1.spi_t.dss);
  TEST_ASSERT_EQUAL(SPIMSBFirst, sdcard1.spi_t.bitorder);
  TEST_ASSERT_EQUAL(SPIDiv64, sdcard1.spi_t.cdiv);
  TEST_ASSERT_EQUAL_PTR(&sdcard1.input_buf, sdcard1.spi_t.input_buf);
  TEST_ASSERT_EQUAL_PTR(&sdcard1.output_buf, sdcard1.spi_t.output_buf);
  TEST_ASSERT_EQUAL(0, sdcard1.spi_t.input_length);
  TEST_ASSERT_EQUAL(0, sdcard1.spi_t.output_length);
  TEST_ASSERT_EQUAL(SDCardType_Unknown, sdcard1.card_type);

  /* Also, the state for upcoming periodic loop is set */
  TEST_ASSERT_EQUAL(SDCard_BeforeDummyClock, sdcard1.status);
}

bool_t SpiSubmitCall_SendDummyClock(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; // ignore unused
  SpiSubmitNrCalls++;

  /* First call, this sends 80 (>74) clock pulses with CS and MOSI high */
  TEST_ASSERT_EQUAL(SPINoSelect, t->select);
  TEST_ASSERT_EQUAL(10, t->output_length);
  TEST_ASSERT_EQUAL(0, t->input_length);

  for (uint8_t i=0; i<10; i++){
    TEST_ASSERT_EQUAL(0xFF, t->output_buf[i]);
  }

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

//! Initialize the SD card with special sequence
/*!
 * For the SD card to switch to SPI mode, both the CS and
 * the MOSI line need to be held HIGH while sending at least
 * 74 clock pulses. This is accomplished by using SPINoSelect
 * to keep the line high and sending 0xFF to keep MOSI
 * high.
 */
void test_SendDummyClockPulses(void)
{
  sdcard1.status = SDCard_BeforeDummyClock;
  spi_submit_StubWithCallback(SpiSubmitCall_SendDummyClock);

  /* Call period loop */
  sdcard_spi_periodic(&sdcard1);

  TEST_ASSERT_EQUAL(SDCard_SendingDummyClock, sdcard1.status);

  /* Also, test that nothing is done in the next periodic loop */
  sdcard_spi_periodic(&sdcard1);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
}

void test_DoNothingWhenTransactionInProgress(void)
{
  sdcard1.status = SDCard_BeforeDummyClock;
  sdcard1.spi_t.status = SPITransPending;

  /* Call the periodic loop */
  sdcard_spi_periodic(&sdcard1);

  /* Test also for this status */
  sdcard1.spi_t.status = SPITransRunning;

  /* Call the periodic loop again */
  sdcard_spi_periodic(&sdcard1);
}

void test_DoNothingInPeriodicLoopWhenNotInitialized(void)
{
  sdcard1.status = SDCard_UnInit;

  /* Call the periodic loop */
  sdcard_spi_periodic(&sdcard1);
}

bool_t SpiSubmitCall_SendCMD0(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; /* ignore unused variables */
  SpiSubmitNrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);

  TEST_ASSERT_EQUAL_HEX8(0x40, t->output_buf[0]); /* CMD byte */
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); /* paramter bytes (ignored) */
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x95, t->output_buf[5]); /* CRC7 for CMD0 */

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

void test_DummyClockPulsesCallback(void)
{
  sdcard1.status = SDCard_SendingDummyClock;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD0);

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_SendingCMD0, sdcard1.status);
}

bool_t SpiSubmitCall_RequestNBytes(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; /* ignore unused variables */
  SpiSubmitNrCalls++;

  TEST_ASSERT_EQUAL(NBytesToRequest, t->output_length);
  TEST_ASSERT_EQUAL(NBytesToRequest, t->input_length);  /* reading response later */

  for (uint8_t i=0; i<NBytesToRequest; i++) {
    TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[i]);
  }

  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);
  return TRUE;
}

void test_ReadySendingCMD0(void)
{
  sdcard1.status = SDCard_SendingCMD0;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Run the callback function
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(1, sdcard1.response_counter); /* Is already one because first byte has been requested */
  TEST_ASSERT_EQUAL(SDCard_ReadingCMD0Resp, sdcard1.status);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
}

bool_t SpiSubmitCall_SendCMD8(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; /* ignore unused variables */
  SpiSubmitNrCalls++;

  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);

  TEST_ASSERT_EQUAL_HEX8(0x48, t->output_buf[0]); /* CMD8 */
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0xAA, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x87, t->output_buf[5]); /* CRC7 for CMD8(0x000001AA) */

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

void test_PollingCMD0ResponseDataReady(void)
{
  sdcard1.status = SDCard_ReadingCMD0Resp;
  sdcard1.input_buf[0] = 0x01;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD8);

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_SendingCMD8, sdcard1.status);
}

void helper_RequestFirstResponseByte(void)
{
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(1, sdcard1.response_counter); /* Is already one because first byte has been requested */
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");

}

void helper_ResponseLater(void)
{
  sdcard1.response_counter = 3; /* random */
  sdcard1.input_buf[0] = 0xFF; /* Not ready */
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(4, sdcard1.response_counter);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
}

void helper_ResponseTimeout(uint8_t limit)
{
  sdcard1.response_counter = 1;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  /* Delay is maximal 9 bytes, abort after this */
  for (uint8_t i=0; i<limit; i++) {
    sdcard1.input_buf[0] = 0xFF; /* Not ready */

    /* Run the callback function */
    sdcard_spi_spicallback(&sdcard1.spi_t);
  }
  /* The last time, don't call spi_submit again. (therefore expect 8 instead of 9) */
  TEST_ASSERT_EQUAL_MESSAGE(limit-1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_Error, sdcard1.status);
}

void test_PollingCMD0ResponseLater(void)
{
  sdcard1.status = SDCard_ReadingCMD0Resp;
  helper_ResponseLater();
}

void test_PollingCMD0ResponseTimeout(void)
{
  sdcard1.status = SDCard_ReadingCMD0Resp;
  helper_ResponseTimeout(9);
}

//! Callback of CMD8
/*!
 * CMD8 sending has completed. Start polling bytes for response.
 */
void test_ReadySendingCMD8(void)
{
  sdcard1.status = SDCard_SendingCMD8;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(1, sdcard1.response_counter); /* Is already one because first byte has been requested */
  TEST_ASSERT_EQUAL(SDCard_ReadingCMD8Resp, sdcard1.status);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
}

void test_PollingCMD8ResponseLater(void)
{
  sdcard1.status = SDCard_ReadingCMD8Resp;
  helper_ResponseLater();
}

/**
 * When 0x01 received, the next four bytes is the 32bit parameter value
 */
void test_PollingCMD8ResponseReady(void)
{
  sdcard1.status = SDCard_ReadingCMD8Resp;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes); NBytesToRequest = 4;
  sdcard1.response_counter = 5; /* somewhat random */
  sdcard1.input_buf[0] = 0x01;

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_ReadingCMD8Parameter, sdcard1.status);
}

void test_PollingCMD8Timeout(void)
{
  sdcard1.status = SDCard_ReadingCMD8Resp;
  helper_ResponseTimeout(9);
}

/**
 * Reading parameter in response to CMD8, 0x1AA mismatch case
 */
void test_ReadCMD8ParameterMismatch(void)
{
  sdcard1.status = SDCard_ReadingCMD8Parameter;
  sdcard1.input_buf[0] = 0x00;
  sdcard1.input_buf[1] = 0x00;
  sdcard1.input_buf[2] = 0x01;
  sdcard1.input_buf[3] = 0xBB; /* Mismatch! */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);
  TEST_ASSERT_EQUAL(SDCard_Error, sdcard1.status);
}

/**
 * Reading parameter in response to CMD8, 0x1AA match case
 */
void test_ReadCMD8ParameterMatch(void)
{
  sdcard1.status = SDCard_ReadingCMD8Parameter;
  sdcard1.input_buf[0] = 0x00;
  sdcard1.input_buf[1] = 0x00;
  sdcard1.input_buf[2] = 0x01;
  sdcard1.input_buf[3] = 0xAA; /* Match! */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_SendingACMD41v2, sdcard1.status);
  TEST_ASSERT_EQUAL(0, sdcard1.timeout_counter); /* Reset the timout counter for ACMD41 */
}

bool_t SpiSubmitCall_SendACMD41(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; /* ignore unused variables */
  SpiSubmitNrCalls++;

  /* Perform ACMD call with argument ACMD_ARG */
  TEST_ASSERT_EQUAL(6+8+1+6, t->output_length); /* CMD55 + Ncr (max 8) + R1 + CMD41 */
  TEST_ASSERT_EQUAL(6+8+1+6, t->input_length);

  /* CMD55 */
  TEST_ASSERT_EQUAL_HEX8(0x77, t->output_buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[5]);

  /* Response time CMD55 (8+1) */
  for(uint8_t i=0; i<9; i++){
    TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[i+6]);
  }

  /* ACMD_CMD */
  TEST_ASSERT_EQUAL_HEX8(0x40 | 41, t->output_buf[15]);
  TEST_ASSERT_EQUAL_HEX8(0x40000000 >> 24, t->output_buf[16]);
  TEST_ASSERT_EQUAL_HEX8(0x40000000 >> 16, t->output_buf[17]);
  TEST_ASSERT_EQUAL_HEX8(0x40000000 >> 8, t->output_buf[18]);
  TEST_ASSERT_EQUAL_HEX8(0x40000000 >> 0, t->output_buf[19]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[20]);

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

void test_SendACMD41NextPeriodicLoop(void)
{
  sdcard1.status = SDCard_SendingACMD41v2;
  sdcard1.timeout_counter = 0;
  spi_submit_StubWithCallback(SpiSubmitCall_SendACMD41);

  /* Function is called in the periodic loop */
  sdcard_spi_periodic(&sdcard1);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(1, sdcard1.timeout_counter);
  /* No need to change state to capture event */
}

void test_ReadySendingACMD41v2(void)
{
  sdcard1.status = SDCard_SendingACMD41v2;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(1, sdcard1.response_counter); /* Is already one because first byte has been requested */
  TEST_ASSERT_EQUAL(SDCard_ReadingACMD41v2Resp, sdcard1.status);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
}

void test_PollingACMD41v2ResponseLater(void)
{
  sdcard1.status = SDCard_ReadingACMD41v2Resp;
  helper_ResponseLater();
}

void test_PollingACMD41v2ResponseTimeout(void)
{
  sdcard1.status = SDCard_ReadingACMD41v2Resp;
  helper_ResponseTimeout(9);
}

/**
 * ACMD41 response is 0x01, try again next periodic loop
 */
void test_PollingACMD41v2Response0x01(void)
{
  sdcard1.timeout_counter = 0;
  sdcard1.status = SDCard_ReadingACMD41v2Resp;
  sdcard1.input_buf[0] = 0x01;

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_SendingACMD41v2, sdcard1.status);
}

/**
 * ACMD41 command try only 500 times (this command checks status until it is initialized (or not))
 */
void test_TryACMD41OnlyLimitedNumberOfTimes(void)
{
  sdcard1.status = SDCard_ReadingACMD41v2Resp;
  sdcard1.timeout_counter = 499; /* Already tried 499 times */
  sdcard1.input_buf[0] = 0x01; /* Response is still not 0x00 */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  /* Error because after 500 times still not the right response */
  TEST_ASSERT_EQUAL(SDCard_Error, sdcard1.status);
}

bool_t SpiSubmitCall_SendCMD58(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; /* ignore unused variables */
  SpiSubmitNrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);  /* R3 response */
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0x7A, t->output_buf[0]); /* CMD byte */
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); /* paramter bytes (ignored) */
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[5]); /* Stop bit */

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

/**
 * Sd card ready with initialization, start CMD58
 */
void test_PollingACMD41v2Response0x00(void)
{
  sdcard1.status = SDCard_ReadingACMD41v2Resp;
  sdcard1.timeout_counter = 57; /* Tried limited number of times, not exceeded timeout */
  sdcard1.input_buf[0] = 0x00;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD58);

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_SendingCMD58, sdcard1.status);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
}

void test_ReadySendingCMD58(void)
{
  sdcard1.status = SDCard_SendingCMD58;
  helper_RequestFirstResponseByte();
  TEST_ASSERT_EQUAL(SDCard_ReadingCMD58Resp, sdcard1.status);
}

void test_PollingCMD58ResponseLater(void)
{
  sdcard1.status = SDCard_ReadingCMD58Resp;
  helper_ResponseLater();
}

void test_PollingCMD58Timeout(void)
{
  sdcard1.status = SDCard_ReadingCMD58Resp;
  helper_ResponseTimeout(9);
}

//! CMD58 has responded with 0x00, then read the next 4 bytes (OCR register)
void test_PollingCMD58DataReady(void)
{
  sdcard1.status = SDCard_ReadingCMD58Resp;
  sdcard1.input_buf[0] = 0x00;
  sdcard1.response_counter = 3;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes); NBytesToRequest = 4;

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_ReadingCMD58Parameter, sdcard1.status);
}

bool_t SpiSubmitCall_SendCMD16(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls;
  SpiSubmitNrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);  /* R1 response */
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0x50, t->output_buf[0]); /* CMD byte */
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); /* paramter bytes */
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x02, t->output_buf[3]); /* force blocksize 512 bytes. */
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[5]); /* Stop bit */

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

/**
 * Check 32-bit OCR register for CCS bit
 */
void test_ReadCMD58ParameterCCSBitSet(void)
{
  sdcard1.status = SDCard_ReadingCMD58Parameter;
  sdcard1.input_buf[0] = 0xCF; /* bit 30 and 31 set */
  sdcard1.input_buf[1] = 0xFF;
  sdcard1.input_buf[2] = 0xFF;
  sdcard1.input_buf[3] = 0xFF; /* last byte */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_Idle, sdcard1.status);
  TEST_ASSERT_EQUAL(SDCardType_SdV2block, sdcard1.card_type);

}

void test_ReadCMD58ParameterCCSBitUnSet(void)
{
  sdcard1.status = SDCard_ReadingCMD58Parameter;
  sdcard1.input_buf[0] = 0x8F; /* bit 30 (CCS) not set, bit 31 set */
  sdcard1.input_buf[1] = 0xFF;
  sdcard1.input_buf[2] = 0xFF;
  sdcard1.input_buf[3] = 0xFF; /* last byte */
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD16);

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_SendingCMD16, sdcard1.status);
  TEST_ASSERT_EQUAL(SDCardType_SdV1, sdcard1.card_type);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
}

/**
 * If bit 31 is not set, the CCS bit is not valid. Abort initialization
 */
void test_ReadCMD58ParameterBit31NotSet(void)
{
  sdcard1.status = SDCard_ReadingCMD58Parameter;
  sdcard1.input_buf[0] = 0x4F; /* bit 31 not set (then bit 30 is not valid) */
  sdcard1.input_buf[1] = 0xFF;
  sdcard1.input_buf[2] = 0xFF;
  sdcard1.input_buf[3] = 0xFF; /* last byte */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_Error, sdcard1.status);
  TEST_ASSERT_EQUAL(SDCardType_Unknown, sdcard1.card_type);
}

void test_ReadySendingCMD16(void)
{
  sdcard1.status = SDCard_SendingCMD16;
  helper_RequestFirstResponseByte();
  TEST_ASSERT_EQUAL(SDCard_ReadingCMD16Resp, sdcard1.status);
}

void test_PollingCMD16ResponseLater(void)
{
  sdcard1.status = SDCard_ReadingCMD16Resp;
  helper_ResponseLater();
}

void test_PollingCMD16Timeout(void)
{
  sdcard1.status = SDCard_ReadingCMD16Resp;
  helper_ResponseTimeout(9);
}

void test_PollingCMD16ResponseDataReady(void)
{
  sdcard1.status = SDCard_ReadingCMD16Resp;
  sdcard1.response_counter = 4;
  sdcard1.input_buf[0] = 0x00; /* correct response = ready */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_Idle, sdcard1.status);
}

void test_DoNotWriteDataIfNotIdle(void)
{
  sdcard1.status = SDCard_Error;

  /* Call the write data function */
  sdcard_spi_write_block(&sdcard1, 0x00000000);

  /* Expect zero calls to spi_submit */
}

bool_t SpiSubmitCall_SendCMD24(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls;
  SpiSubmitNrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(SPIDiv64, t->cdiv);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);  /* R1 response */
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0x58, t->output_buf[0]); /* CMD byte */
  if (sdcard1.card_type == SDCardType_SdV2block) {
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); /* 4 bytes for the address */
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x14, t->output_buf[4]);
  }
  else {
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); /* 4 bytes for the address */
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x28, t->output_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  }
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[5]); /* Stop bit */

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

void test_WriteDataBlockWithBlockAddress(void)
{
  sdcard1.status = SDCard_Idle;
  sdcard1.card_type = SDCardType_SdV2block;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD24);

  /* Call the write data function */
  sdcard_spi_write_block(&sdcard1, 0x00000014); /* is decimal 20 * 512 */

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_SendingCMD24, sdcard1.status);
}


void test_WriteDataBlockWithByteAddress(void)
{
  sdcard1.status = SDCard_Idle;
  sdcard1.card_type = SDCardType_SdV2byte;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD24);

  /* Call the write data function */
  sdcard_spi_write_block(&sdcard1, 0x00000014); /* = decimal 20 */

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_SendingCMD24, sdcard1.status);
}

void test_ReadySendingCMD24(void) {
  sdcard1.status = SDCard_SendingCMD24;
  helper_RequestFirstResponseByte();
  TEST_ASSERT_EQUAL(SDCard_ReadingCMD24Resp, sdcard1.status);
}

void test_PollingCMD24ResponseLater(void)
{
  sdcard1.status = SDCard_ReadingCMD24Resp;
  helper_ResponseLater();
}

void test_PollingCMD24Timeout(void)
{
  sdcard1.status = SDCard_ReadingCMD24Resp;
  helper_ResponseTimeout(9);
}

/**
 * When CMD24 responds, another dummy byte needs to be requested before the block with data is transferred
 */
void test_PollingCMD24DataReady(void)
{
  sdcard1.status = SDCard_ReadingCMD24Resp;
  sdcard1.input_buf[0] = 0x00; // Ready
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_BeforeSendingDataBlock, sdcard1.status);
  /* Value of the response counter does not matter any more */
}

bool_t SpiSubmitCall_SendDataBlock(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls;
  SpiSubmitNrCalls++;

  /* Use a different offset for the output buffer */
  TEST_ASSERT_EQUAL_PTR(&sdcard1.output_buf[5], t->output_buf);

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(516, t->output_length);
  TEST_ASSERT_EQUAL(516, t->input_length);
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0xFE, t->output_buf[0]); /* Data Token */
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[256]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[257]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[258]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[512]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[513]); /* CRC byte 1 */
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[514]); /* CRC byte 2 */
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[515]); /* Request data response */

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

void test_SendDataBlock(void)
{
  sdcard1.status = SDCard_BeforeSendingDataBlock;
  spi_submit_StubWithCallback(SpiSubmitCall_SendDataBlock);

  for (uint16_t i=0; i<256; i++) {
    sdcard1.output_buf[6+i] = 0x00;
    sdcard1.output_buf[6+i+256] = i;
  }

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_SendingDataBlock, sdcard1.status);
}

void test_ReadySendingDataBlockAccepted(void)
{
  sdcard1.status = SDCard_SendingDataBlock;
  sdcard1.input_buf[515] = 0x05; /* B00000101 = data accepted */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  /* Reset different offset for the output buffer */
  TEST_ASSERT_EQUAL_PTR(&sdcard1.output_buf, sdcard1.spi_t.output_buf);

  TEST_ASSERT_EQUAL(SDCard_Busy, sdcard1.status);
}

void test_ReadySendingDataBlockRejected(void)
{
  sdcard1.status = SDCard_SendingDataBlock;
  sdcard1.input_buf[515] = 0x0D; /* B00001101 = data rejected */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  /* Reset different offset for the output buffer */
  TEST_ASSERT_EQUAL_PTR(&sdcard1.output_buf, sdcard1.spi_t.output_buf);
  TEST_ASSERT_EQUAL(SDCard_Error, sdcard1.status);
}

void test_RequestBytePeriodicallyWhileBusy(void)
{
  sdcard1.status = SDCard_Busy;
  sdcard1.input_buf[0] = 0x00; /* LOW = busy */
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  /* Run the periodic function */
  sdcard_spi_periodic(&sdcard1);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_Busy, sdcard1.status);
}

void test_RevertToIdleWhenNoLongerBusy(void)
{
  sdcard1.status = SDCard_Busy;
  sdcard1.input_buf[0] = 0xFF; /* line = high = no longer busy */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_Idle, sdcard1.status);
}

bool_t SpiSubmitCall_SendCMD17(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls;
  SpiSubmitNrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(SPIDiv32, t->cdiv);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length); /* R1 response */
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0x51, t->output_buf[0]); /* CMD byte */
  if (sdcard1.card_type == SDCardType_SdV2block) {
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); /* 4 bytes for the address */
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]); /* is just 20 */
    TEST_ASSERT_EQUAL_HEX8(0x14, t->output_buf[4]);
  }
  else {
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); /* 4 bytes for the address */
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x28, t->output_buf[3]); /* is 20 * 512 */
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  }
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[5]); /* Stop bit */

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

void helper_ExampleCallbackFunction(void)
{
  CallbackWasCalled = TRUE;
}

void test_DoNotReadDataIfNotIdle(void)
{
  sdcard1.status = SDCard_Busy;

  /* Call the read data function */
  sdcard_spi_read_block(&sdcard1, 0x00000000, &helper_ExampleCallbackFunction);

  /* Expect zero calls to spi_submit */
  TEST_ASSERT_EQUAL(NULL, sdcard1.read_callback);
}

void test_ReadDataBlockWithBlockAddress(void)
{
  sdcard1.status = SDCard_Idle;
  sdcard1.card_type = SDCardType_SdV2block;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD17);

  /* Call the read data function */
  sdcard_spi_read_block(&sdcard1, 0x00000014, &helper_ExampleCallbackFunction);

  TEST_ASSERT_EQUAL_PTR(&helper_ExampleCallbackFunction, sdcard1.read_callback);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_SendingCMD17, sdcard1.status);
}


void test_ReadDataBlockWithByteAddress(void)
{
  sdcard1.status = SDCard_Idle;
  sdcard1.card_type = SDCardType_SdV2byte;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD17);

  /* Call the read data function */
  sdcard_spi_read_block(&sdcard1, 0x00000014, &helper_ExampleCallbackFunction);

  TEST_ASSERT_EQUAL_PTR(&helper_ExampleCallbackFunction, sdcard1.read_callback);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_SendingCMD17, sdcard1.status);
}

void test_ReadySendingCMD17(void) {
  sdcard1.status = SDCard_SendingCMD17;
  helper_RequestFirstResponseByte();
  TEST_ASSERT_EQUAL(SDCard_ReadingCMD17Resp, sdcard1.status);
}

void test_PollingCMD17ResponseLater(void)
{
  sdcard1.status = SDCard_ReadingCMD17Resp;
  helper_ResponseLater();
}

void test_PollingCMD17Timeout(void)
{
  sdcard1.status = SDCard_ReadingCMD17Resp;
  helper_ResponseTimeout(9);
}

/**
 * When CMD17 response is ready, switch to mode waiting for data token
 */
void test_PollingCMD17DataReady(void)
{
  sdcard1.status = SDCard_ReadingCMD17Resp;
  sdcard1.input_buf[0] = 0x00; // data ready
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  /* Call the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(0, sdcard1.timeout_counter); /* reset the timout counter for data token response */
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_WaitingForDataToken, sdcard1.status);
}

void test_PollDataTokenPeriodically(void)
{
  sdcard1.status = SDCard_WaitingForDataToken;
  sdcard1.timeout_counter = 5;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  /* Call the periodic function */
  sdcard_spi_periodic(&sdcard1);

  TEST_ASSERT_EQUAL(6, sdcard1.timeout_counter);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
}

void test_PollingDataTokenTimeout(void)
{
  sdcard1.status = SDCard_WaitingForDataToken;
  sdcard1.timeout_counter = 499; /* Already tried 499 times */
  sdcard1.input_buf[0] = 0xFF; /* Still no data token */

  /* Call the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_Error, sdcard1.status);
}

void test_PollingDataTokenNotReady(void)
{
  sdcard1.status = SDCard_WaitingForDataToken;
  sdcard1.timeout_counter = 5;
  sdcard1.input_buf[0] = 0xFF; /* Not ready */

  /* Call the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);
  TEST_ASSERT_EQUAL(SDCard_WaitingForDataToken, sdcard1.status);
}

bool_t SpiSubmitCall_ReadDataBlock(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; /* Ignore unused variables */
  SpiSubmitNrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(SPIDiv8, t->cdiv);
  TEST_ASSERT_EQUAL(512+2, t->output_length);
  TEST_ASSERT_EQUAL(512+2, t->input_length);
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  for (uint16_t i=0; i<512; i++) {
    TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[i]);
  }
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[512]); /* CRC byte 1 */
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[513]); /* CRC byte 2 */

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

void test_PollingDataTokenReady(void)
{
  sdcard1.status = SDCard_WaitingForDataToken;
  sdcard1.input_buf[0] = 0xFE; /* Data token */
  spi_submit_StubWithCallback(SpiSubmitCall_ReadDataBlock);

  /* Call the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_ReadingDataBlock, sdcard1.status);
}

void test_ReadDataBlockContent(void)
{
  sdcard1.status = SDCard_ReadingDataBlock;
  sdcard1.read_callback = &helper_ExampleCallbackFunction;

  /* Call the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_TRUE(CallbackWasCalled);
  TEST_ASSERT_EQUAL(SDCard_Idle, sdcard1.status);

}

void test_ReadDataBlockContentWithoutCallback(void)
{
  sdcard1.status = SDCard_ReadingDataBlock;
  sdcard1.read_callback = NULL;

  /* Call the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_FALSE(CallbackWasCalled);
  TEST_ASSERT_EQUAL(SDCard_Idle, sdcard1.status);
}

bool_t SpiSubmitCall_SendCMD25(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls;
  SpiSubmitNrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(SPIDiv64, t->cdiv);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);  /* R1 response */
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0x59, t->output_buf[0]); /* CMD byte */
  if (sdcard1.card_type == SDCardType_SdV2block) {
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); /* 4 bytes for the address */
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x14, t->output_buf[4]);
  }
  else {
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); /* 4 bytes for the address */
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x28, t->output_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  }
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[5]); /* Stop bit */

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

void test_StartMultiWriteStartWhenIdleWithBlockAddress(void)
{
  sdcard1.status = SDCard_Idle;
  sdcard1.card_type = SDCardType_SdV2block;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD25);

  /* Call the multiwrite start function */
  sdcard_spi_multiwrite_start(&sdcard1, 0x00000014); /* is decimal 20 * 512 */

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_SendingCMD25, sdcard1.status);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");

}

void test_StartMultiWriteStartWhenIdleWithByteAddress(void)
{
  sdcard1.status = SDCard_Idle;
  sdcard1.card_type = SDCardType_SdV2byte;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD25);

  /* Call the write data function */
  sdcard_spi_multiwrite_start(&sdcard1, 0x00000014); /* = decimal 20 */

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_SendingCMD25, sdcard1.status);
}

void test_StartMultiWriteOnlyIfIdle(void)
{
  sdcard1.status = SDCard_Busy; /* Not idle */

  sdcard_spi_multiwrite_start(&sdcard1, 0x00000014);
}


void test_ReadySendingCMD25(void) {
  sdcard1.status = SDCard_SendingCMD25;
  helper_RequestFirstResponseByte();
  TEST_ASSERT_EQUAL(SDCard_ReadingCMD25Resp, sdcard1.status);
}

void test_PollingCMD25ResponseLater(void)
{
  sdcard1.status = SDCard_ReadingCMD25Resp;
  helper_ResponseLater();
}

void test_PollingCMD25Timeout(void)
{
  sdcard1.status = SDCard_ReadingCMD25Resp;
  helper_ResponseTimeout(9);
}

//! When CMD25 responds, another dummy byte needs to be requested before the block with data is transferred
void test_PollingCMD25DataReady(void)
{
  sdcard1.status = SDCard_ReadingCMD25Resp;
  sdcard1.input_buf[0] = 0x00; /* Ready */
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_MultiWriteIdle, sdcard1.status);
  /* Value of the response counter does not matter any more */
}

bool_t SpiSubmitCall_SendMultiWriteDataBlock(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls;
  SpiSubmitNrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(516, t->output_length);
  TEST_ASSERT_EQUAL(516, t->input_length);
  TEST_ASSERT_EQUAL(SPITransDone, t->status);
  TEST_ASSERT_EQUAL(SPIDiv32, t->cdiv);

  TEST_ASSERT_EQUAL_HEX8(0xFC, t->output_buf[0]); /* Data Token for CMD25 */
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[256]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[257]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[258]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[512]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[513]); /* CRC byte 1 */
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[514]); /* CRC byte 2 */
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[515]); /* Request data response */

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

void test_WriteMultiWriteBlockWhenIdle(void)
{
  sdcard1.status = SDCard_MultiWriteIdle;
  spi_submit_StubWithCallback(SpiSubmitCall_SendMultiWriteDataBlock);

  for (uint16_t i=0; i<256; i++) {
    sdcard1.output_buf[1+i] = 0x00;
    sdcard1.output_buf[1+i+256] = i;
  }

  /* Call the write function */
  sdcard_spi_multiwrite_next(&sdcard1);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_MultiWriteWriting, sdcard1.status);
}

void test_DoNotWriteMultiWriteBlockIfNotIdle(void)
{
  sdcard1.status = SDCard_MultiWriteBusy;

  /* Multiwrite write command */
  sdcard_spi_multiwrite_next(&sdcard1);

  /* Should not do anything */
}

//!
void test_ReadyMultiWriteSendingDataBlockAccepted(void)
{
  sdcard1.status = SDCard_MultiWriteWriting;
  sdcard1.input_buf[515] = 0x05; /* B00000101 = data accepted */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  /* Reset different offset for the output buffer */
  TEST_ASSERT_EQUAL_PTR(&sdcard1.output_buf, sdcard1.spi_t.output_buf);

  TEST_ASSERT_EQUAL(SDCard_MultiWriteBusy, sdcard1.status);
}

void test_ReadyMultiWriteSendingDataBlockRejected(void)
{
  sdcard1.status = SDCard_MultiWriteWriting;
  sdcard1.input_buf[515] = 0x0D; /* B00001101 = data rejected */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  /* Reset different offset for the output buffer */
  TEST_ASSERT_EQUAL_PTR(&sdcard1.output_buf, sdcard1.spi_t.output_buf);
  TEST_ASSERT_EQUAL(SDCard_Error, sdcard1.status);
}

void test_RequestBytePeriodicallyWhileMultiWriteBusy(void)
{
  sdcard1.status = SDCard_MultiWriteBusy;
  sdcard1.input_buf[0] = 0x00; /* LOW = busy */
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  /* Run the periodic function */
  sdcard_spi_periodic(&sdcard1);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_MultiWriteBusy, sdcard1.status);
}

void test_RevertToIdleWhenNoLongerMultiWriteBusy(void)
{
  sdcard1.status = SDCard_MultiWriteBusy;
  sdcard1.input_buf[0] = 0xFF; // line = high = no longer busy

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_MultiWriteIdle, sdcard1.status);
}

void test_RemainMultiWriteBusy(void)
{
  sdcard1.status = SDCard_MultiWriteBusy;
  sdcard1.input_buf[0] = 0x00; /* line = low = busy */

  /* Run the callback function */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_MultiWriteBusy, sdcard1.status);
}

bool_t SpiSubmitCall_SendStopMultiWrite(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; /* ignore unused variables */
  SpiSubmitNrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(2, t->output_length);
  TEST_ASSERT_EQUAL(2, t->input_length);
  TEST_ASSERT_EQUAL(SPITransDone, t->status);
  TEST_ASSERT_EQUAL(SPIDiv32, t->cdiv);

  TEST_ASSERT_EQUAL_HEX8(0xFD, t->output_buf[0]); /* Stop Token for CMD25 */
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[1]); /* Poll busy flag */

  /* Callback */
  TEST_ASSERT_EQUAL_PTR(&sdcard_spi_spicallback, t->after_cb);

  return TRUE;
}

void test_StopWithMultiWrite(void)
{
  sdcard1.status = SDCard_MultiWriteIdle;
  spi_submit_StubWithCallback(SpiSubmitCall_SendStopMultiWrite);

  /* Stop command */
  sdcard_spi_multiwrite_stop(&sdcard1);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SDCard_MultiWriteStopping, sdcard1.status);
}

void test_AfterStopMultiWriteContinueInIdleState(void)
{
  sdcard1.status = SDCard_MultiWriteStopping;

  /* Called back when spi ready */
  sdcard_spi_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SDCard_Busy, sdcard1.status);
}

void test_DoNotStopIfNotMultiWriteIdleOrBusy(void)
{
  sdcard1.status = SDCard_Idle;

  /* Stop command */
  sdcard_spi_multiwrite_stop(&sdcard1);
  /* Expect nothing to happen */
}

void test_SendErrorMessage(void)
{
  //TEST_IGNORE();
}
