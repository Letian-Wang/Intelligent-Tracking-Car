/*!
 * @file       LED.C
 * @brief      LED���ú���

 */
#include "include.h"
 /*
    �������� :����led��ʼ��
    ע�����İ����������ɱ��LED���˿ڷֱ��� PTA19,PTE6,PTC18,�͵�ƽ����
      �˿ڶ����� led.h
 */  
void led_init()
{
  gpio_init (LED_RED, GPO,1);
  gpio_init (LED_GREEN, GPO,1); 
  gpio_init (LED_BLUE, GPO,1);

}

void water_lights()
{
  LED_GREEN_ON;
  LED_RED_OFF;
  LED_BLUE_OFF;
  DELAY_MS(150);
  LED_GREEN_OFF;
  LED_RED_ON;
  LED_BLUE_OFF;
  DELAY_MS(150);
  LED_GREEN_OFF;
  LED_RED_OFF;
  LED_BLUE_ON;
  DELAY_MS(150);

}
void led_flash()
{
  LED_GREEN_ON;
  LED_RED_ON;
  LED_BLUE_ON;
  DELAY_MS(500);
  LED_GREEN_OFF;
  LED_RED_OFF;
  LED_BLUE_OFF;
  DELAY_MS(500);
}