/*!
 * @file       misc.h
 * @brief      k60������Ҫ�õ��ĺ�����ʵ��
 */

#ifndef __MISC_H__
#define __MISC_H__

#include "common.h"


void write_vtor (int);                                              //�����ж�������ĵ�ַ
void set_vector_handler(VECTORn_t , void pfunc_handler(void));      //�����жϺ������ж���������


//���ݾɴ���
#define enable_irq(irq)                 NVIC_EnableIRQ(irq)         //ʹ��IRQ
#define disable_irq(irq)                NVIC_DisableIRQ(irq)        //��ֹIRQ
#define set_irq_priority(irq,pri0)      NVIC_SetPriority(irq,pri0)  //�������ȼ�

#if 1    //���ַ�����һ����CMSIS�Դ���һ����ֱ�ӻ������
#define EnableInterrupts                __enable_irq()              //ʹ��ȫ���ж�
#define DisableInterrupts               __disable_irq()             //��ֹȫ���ж�
#else
#define EnableInterrupts                asm(" CPSIE i");            //ʹ��ȫ���ж�
#define DisableInterrupts               asm(" CPSID i");            //��ֹȫ���ж�
#endif

void vcan_cpy( uint8 *dst, uint8 *src, uint32 count);

#endif  /* __MISC_H__ */

