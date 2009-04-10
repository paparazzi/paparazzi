#include "csc_can.h"

#include "LPC21xx.h"
#include "armVIC.h"

#include "led.h"

struct CscCanServoCmd {
  
  
};


struct CscCanMotorCmd {
  

};

struct CscCanMotorStatus {


};


static void CAN_Err_ISR ( void ) __attribute__((naked));


#ifdef USE_CAN1

static void CAN1_Rx_ISR ( void ) __attribute__((naked));
static void CAN1_Tx_ISR ( void ) __attribute__((naked));

void csc_can1_init(void) {

  // Set bit 18
  PINSEL1 |= (1<<18);//0x00040000L;
  // Acceptance Filter Mode Register = filter off, receive all
  AFMR = 0x00000002L;
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

  // Enable Interrupts
  C1IER = 1;
  // Get out of reset Mode
  C1MOD = 0;
}


void csc_can1_send(struct CscCanMsg* msg) {
#if 0
  if (C1SR & 0x00000004L) { /* transmit channel not available */
    LED_ON(2);
    return;
  }
#endif
  // Write DLC, RTR and FF
  C1TFI1 = (msg->frame &  0xC00F0000L);
  // Write CAN ID
  C1TID1 = msg->msg_id;
  // Write first 4 data bytes 
  C1TDA1 = msg->dat_a;
  // Write second 4 data bytes 
  C1TDB1 = msg->dat_b;
  // Write self transmission request
  C1CMR = 0x30;

}

void CAN1_Rx_ISR ( void ) {
 ISR_ENTRY();
 
 // LED_ON(2);

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
  PINSEL1 |= 0x00014000L;
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

  can2_msg_received = FALSE;

}


void csc_can2_send(struct CscCanMsg* msg) {

#if 0
  if (C2SR & 0x00000004L) { /* transmit channel not available */
    LED_ON(2);
    return;
  }
#endif
  // Write DLC, RTR and FF
  C2TFI1 = (msg->frame &  0xC00F0000L);
  // Write CAN ID
  C2TID1 = msg->msg_id;
  // Write first 4 data bytes 
  C2TDA1 = msg->dat_a;
  // Write second 4 data bytes 
  C2TDB1 = msg->dat_b;
  // Write self transmission request
  C2CMR = 0x30;

}

void CAN2_Rx_ISR ( void ) {
 ISR_ENTRY();
 
 can2_rx_msg.frame  = C2RFS;
 can2_rx_msg.msg_id = C2RID;
 can2_rx_msg.dat_a  = C2RDA;
 can2_rx_msg.dat_b  = C2RDB;
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


void CAN_Err_ISR ( void ) {
 ISR_ENTRY();

 ISR_EXIT();
}

