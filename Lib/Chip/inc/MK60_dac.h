/*!
 * @file       MK60_dac.h
 * @brief      DAC����
 */

#ifndef     _MK60_DAC_H_
#define     _MK60_DAC_H_ 1u

typedef enum DACn       //DACģ��
{
    DAC0,
    DAC1
} DACn_e;

extern void dac_init(DACn_e);               //DACһ��ת����ʼ��
extern void dac_out(DACn_e, uint16 val);    //DACһ��ת������

#endif  //_MK60_DAC_H_

