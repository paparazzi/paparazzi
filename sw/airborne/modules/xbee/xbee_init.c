
#include "mcu_periph/uart.h"
#include "xbee_init.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "led.h"

bool_t init_xbee = TRUE;
uint8_t repeat_cnt = 0;

static uint32_t get_serial(uint8_t* buf)
{
  uint32_t res = 0;
  while(*buf != 13)
  {
    res = res<<4;
    switch(*buf)
    {
    case '0':
      break;
    case '1':
      res += 1; break;
    case '2':
      res += 2; break;
    case '3':
      res += 3; break;
    case '4':
      res += 4; break;
    case '5':
      res += 5; break;
    case '6':
      res += 6; break;
    case '7':
      res += 7; break;
    case '8':
      res += 8; break;
    case '9':
      res += 9; break;
    case 'A':
      res += 10; break;
    case 'B':
      res += 11; break;
    case 'C':
      res += 12; break;
    case 'D':
      res += 13; break;
    case 'E':
      res += 14; break;
    case 'F':
      res += 15; break;
    default:
      return -1;
    }
    buf++;
  }
  return res;
}

void xbee_module_init( void ) {

#ifdef INIT_XBEE
  static uint8_t mode = 0;
  static uint16_t count;
  static uint8_t n;
  static uint8_t buf[20];
	while (init_xbee && repeat_cnt < 10)
	{
#ifdef XBEE_LED
    LED_TOGGLE(XBEE_LED)
#endif
    switch(mode)
    {
    case 0:
			repeat_cnt++;
      //In-Buffer leeren
      while(XbBuffer())
        XbGetch();
      n = 0;
      count = 0;
      //Mode ändern;
      XbTransmit('+');
      mode++;
      break;
    case 1:
      //In-XbBuffer leeren
      while(XbBuffer())
        XbGetch();
      //Mode ändern;
      XbTransmit('+');
      mode++;
      break;
    case 2:
      //In-XbBuffer leeren
      while(XbBuffer())
        XbGetch();
      //Mode ändern;
      XbTransmit('+');
      mode++;
      break;
    case 3:
      if(XbBuffer())
      {
        buf[n] = XbGetch();
        if(buf[n] == 13)
          mode++;
        n++;
      }
      else
        count++;
      if(count > 100 || n >= 20)
        mode = 0;
      break;
    case 4:
      if(n == 3) //OK empfangen?
      {
        if(buf[0] == 'O' && buf[1] == 'K')
        {
          XbTransmit('A');
          XbTransmit('T');
          XbTransmit('S');
          XbTransmit('H');
          XbTransmit(13);
          n = 0;
          mode++;
          break;
        }
      }
      mode = 0;
      break;
    case 5:
      if(XbBuffer())
      {
        buf[n] = XbGetch();
        if(buf[n] == 13)
          mode++;
        n++;
      }
      else
        count++;
      if(count > 100 || n >= 20)
        mode = 0;
      break;
    case 6:
      if(n > 0)
        xbee_sh = get_serial(buf);
      else
      {
        mode = 0;
        break;
      }
      XbTransmit('A');
      XbTransmit('T');
      XbTransmit('S');
      XbTransmit('L');
      XbTransmit(13);
      n = 0;
      mode++;
      break;
    case 7:
      if(XbBuffer())
      {
        buf[n] = XbGetch();
        if(buf[n] == 13)
          mode++;
        n++;
      }
      else
        count++;
      if(count > 100 || n >= 20)
        mode = 0;
      break;
    case 8:
      if(n > 0)
        xbee_sl = get_serial(buf);
      else
      {
        mode = 0;
        break;
      }
      mode++;
      break;
    default:
#ifdef XBEE_LED
      LED_ON(XBEE_LED)
#endif
      mode = 0;
      init_xbee = FALSE;
    }
		/** - busy wait 20ms */
		sys_time_usleep(20000);
	}
#endif

}
