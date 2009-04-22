#include "csc_can.h"

#include "LPC21xx.h"
#include "armVIC.h"

#include "led.h"



static void CAN_Err_ISR ( void ) __attribute__((naked));



#ifdef USE_CAN1

bool_t can1_msg_received;
struct CscCanMsg can1_rx_msg;

static void CAN1_Rx_ISR ( void ) __attribute__((naked));
static void CAN1_Tx_ISR ( void ) __attribute__((naked));

void csc_can1_init(void) {

  // Set bit 18
  PINSEL1 |= _BV(18);
  // Acceptance Filter Mode Register = filter off, receive all
  AFMR = 0x00000002;
  // Go into Reset mode
  C1MOD =  1; 
  // Disable All Interrupts
  C1IER = 0;
  // Clear Status register
  C1GSR = 0;
  // Set bit timing
  C1BTR = CAN1_BTR;
  
  // Initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_CAN1_RX);               // VIC_CAN1_RX selected as IRQ
  VICIntEnable = VIC_BIT(VIC_CAN1_RX);                 // VIC_CAN1_RX interrupt enabled
  _VIC_CNTL(CAN1_VIC_SLOT) = VIC_ENABLE | VIC_CAN1_RX; //
  _VIC_ADDR(CAN1_VIC_SLOT) = (uint32_t)CAN1_Rx_ISR;    // address of the ISR

  
#ifdef  CAN_ERR_VIC_SLOT
  C1IER =  (1<<0) | /* RIE */
           (1<<2) | /* EIE */
           (1<<6) | /* ALIE */ 
           (1<<7) | /* BEIE */ 
           (1<<7)   /* BEIE */ 
    ;


  VICIntSelect &= ~VIC_BIT(VIC_CAN);                  // VIC_CAN selected as IRQ
  VICIntEnable = VIC_BIT(VIC_CAN);                    // VIC_CAN interrupt enabled
  _VIC_CNTL(CAN_ERR_VIC_SLOT) = VIC_ENABLE | VIC_CAN; //
  _VIC_ADDR(CAN_ERR_VIC_SLOT) = (uint32_t)CAN_Err_ISR; 
#endif

  // Enable Interrupts
  //  C1IER = (1<<0) | /* RIE */;
  // Get out of reset Mode
  C1MOD = 0;


}


void csc_can1_send(struct CscCanMsg* msg) {

  if (!(C1SR & 0x00000004L)) { /* transmit channel not available */
    //    LED_ON(2);
    return;
  }
  //  LED_OFF(2);
  
  // Write DLC, RTR and FF
  C1TFI1 = (msg->frame &  0xC00F0000L);
  // Write CAN ID
  C1TID1 = msg->id;
  // Write first 4 data bytes 
  C1TDA1 = msg->dat_a;
  // Write second 4 data bytes 
  C1TDB1 = msg->dat_b;
  // Write self reception request
  //  C1CMR = 0x30;
  // write transmission request
  C1CMR = 0x21;
  
}

void CAN1_Rx_ISR ( void ) {
 ISR_ENTRY();
 
 can1_rx_msg.frame  = C1RFS;
 can1_rx_msg.id     = C1RID;
 can1_rx_msg.dat_a  = C1RDA;
 can1_rx_msg.dat_b  = C1RDB;
 if (BOARDID_OF_CANMSG_ID(can1_rx_msg.id) == CSC_BOARD_ID)
   can1_msg_received = TRUE;

 C1CMR = 0x04;             // release receive buffer
 VICVectAddr = 0x00000000; // acknowledge interrupt
 
 ISR_EXIT();
}


void CAN1_Tx_ISR ( void ) {
 ISR_ENTRY();

 ISR_EXIT();
}

#endif /* USE_CAN1 */


#ifdef USE_CAN2

bool_t can2_msg_received;
struct CscCanMsg can2_rx_msg;

static void CAN2_Rx_ISR ( void ) __attribute__((naked));
static void CAN2_Tx_ISR ( void ) __attribute__((naked));

void csc_can2_init(void) {

  //  Set bits 14 and 16
  PINSEL1 |= _BV(14) | _BV(16);
  // Acceptance Filter Mode Register = filter off, receive all
  AFMR = 0x00000002L;
  // Go into Reset mode
  C2MOD =  1; 
  // Disable All Interrupts
  C2IER = 0;
  // Clear Status register
  C2GSR = 0;
  // Set bit timing
  C2BTR = CAN2_BTR;
  
  // initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_CAN2_RX);               // VIC_CAN1_RX selected as IRQ
  VICIntEnable = VIC_BIT(VIC_CAN2_RX);                 // VIC_CAN1_RX interrupt enabled
  _VIC_CNTL(CAN2_VIC_SLOT) = VIC_ENABLE | VIC_CAN2_RX; //
  _VIC_ADDR(CAN2_VIC_SLOT) = (uint32_t)CAN2_Rx_ISR;    // address of the ISR

  // Enable Interrupts
  C2IER = 1;
  // Get out of reset Mode
  C2MOD = 0;
  // error passive mode
  //  C2MOD = 2;
  can2_msg_received = FALSE;

}


void csc_can2_send(struct CscCanMsg* msg) {

  if (!(C2SR & 0x00000004L)) { /* transmit channel not available */
    //    LED_ON(2);
    return;
  }
  
  // Write DLC, RTR and FF
  C2TFI1 = (msg->frame &  0xC00F0000L);
  // Write CAN ID
  C2TID1 = msg->id;
  // Write first 4 data bytes 
  C2TDA1 = msg->dat_a;
  // Write second 4 data bytes 
  C2TDB1 = msg->dat_b;
  // Write self transmission request
  //  C2CMR = 0x30;
  // write transmission request
  C2CMR = 0x21;

}


void CAN2_Rx_ISR ( void ) {
 ISR_ENTRY();
 
 can2_rx_msg.frame  = C2RFS;
 can2_rx_msg.id = C2RID;
 can2_rx_msg.dat_a  = C2RDA;
 can2_rx_msg.dat_b  = C2RDB;
 if (BOARDID_OF_CANMSG_ID(can2_rx_msg.id) == CSC_BOARD_ID)
   can2_msg_received = TRUE;
 
 C2CMR = 0x04;             // release receive buffer
 VICVectAddr = 0x00000000; // acknowledge interrupt
 ISR_EXIT();
}


void CAN2_Tx_ISR ( void ) {
 ISR_ENTRY();

 ISR_EXIT();
}


#endif /* USE_CAN2 */

#include "downlink.h"
#include "uart.h"

static uint32_t err_cnt = 0;

void CAN_Err_ISR ( void ) {
 ISR_ENTRY();
 
 err_cnt++;
 LED_ON(2); 
 uint32_t c1icr = C1ICR;
 DOWNLINK_SEND_CSC_CAN_DEBUG(&err_cnt, &c1icr);


 VICVectAddr = 0x00000000; // acknowledge interrupt
 ISR_EXIT();
}

