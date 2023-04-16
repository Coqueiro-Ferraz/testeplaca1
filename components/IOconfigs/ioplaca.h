#ifndef __IOPLACA_H
    #define __IOPLACA_H
   // #include "esp_err.h"
    //void Enviar_lcd595(uint8_t dado);
    void ioinit(void);
    uint8_t io_le_escreve(uint8_t saidas);
    uint8_t exp_le_escreve (uint8_t enviar);
    char le_teclado ();
#endif