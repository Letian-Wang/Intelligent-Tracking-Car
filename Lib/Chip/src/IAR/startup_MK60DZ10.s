 /*!
  * @file       startup_MK60DZ10.s
  * @brief      ϵͳ������λ����
  */
  
;         AREA   Crt0, CODE, READONLY      ; name this block of code



    SECTION .noinit : CODE          ; //ָ������Σ�.noinit
    EXPORT  Reset_Handler           ; //���� Reset_Handler ����
Reset_Handler
    CPSIE   i                       ; //ʹ��ȫ���ж�
    import start                    ; //��������
    BL      start                   ; //���� C���Ժ��� start
__done
    B       __done


        END
