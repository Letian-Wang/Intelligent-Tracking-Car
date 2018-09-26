/*!
 * @file       button.h
 * @brief      按键相关头文件
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_

#define BT_CANCEL    PTB22 //BT3
#define BT_RIGHT     PTB23 //BT6
#define BT_UP        PTB20 //BT2
#define BT_DOWN      PTB21 //BT5
#define BT_YES       PTB3  //BT1
#define BT_LEFT      PTB2  //BT4

#define SW1      PTA5
#define SW2      PTA13
#define SW3      PTA16
#define SW4      PTA17

#define SW1_IN   gpio_get(SW1)
#define SW2_IN   gpio_get(SW2)
#define SW3_IN   gpio_get(SW3)
#define SW4_IN   gpio_get(SW4)

#define BEEP      PTE4
#define BEEP_ON   gpio_set (BEEP,1)
#define BEEP_OFF  gpio_set (BEEP,0)


#define BT_CANCEL_IN gpio_get(BT_CANCEL)
#define BT_LEFT_IN   gpio_get(BT_LEFT)
#define BT_UP_IN     gpio_get(BT_UP)
#define BT_DOWN_IN   gpio_get(BT_DOWN)
#define BT_YES_IN    gpio_get(BT_YES)
#define BT_RIGHT_IN  gpio_get(BT_RIGHT)

void button_init();
void switch_init();

#endif /* _BUTTON_H_ */