#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include "esp_system.h"

// LCD -> fazer o aterramento dos pinos flutuantes, 2 bits menos significativos inutilizados
#define LCD_DT_WR   GPIO_NUM_18
#define LCD_SH_LD   GPIO_NUM_5
#define LCD_CK      GPIO_NUM_17
// IOs -> 7 a 4 BJT coletor, 3 e 2 Triac, 1 e 0 Relé
#define IO_DT_WR    GPIO_NUM_27
#define IO_SH_LD    GPIO_NUM_14
#define IO_CK       GPIO_NUM_12
#define IO_DT_RD    GPIO_NUM_13
// Teclado -> Linhas são saídas e colunas são entradas
#define TEC_DT_WR   GPIO_NUM_16
#define TEC_CK      GPIO_NUM_4
#define TEC_SH_LD   GPIO_NUM_2
#define TEC_DT_RD   GPIO_NUM_15
// Motor DRV8825 / A4988 -> fazer o aterramento de pinos flutuantes
#define MP_SLP      GPIO_NUM_22
#define MP_STEP     GPIO_NUM_21
#define MP_DIR      GPIO_NUM_19
// Expansor 
#define EXP_CK      GPIO_NUM_32
#define EXP_SH_LD   GPIO_NUM_33
#define EXP_DT_WR   GPIO_NUM_25
#define EXP_DT_RD   GPIO_NUM_26

void ioinit(void)
{
    gpio_pad_select_gpio(LCD_DT_WR);
    gpio_pad_select_gpio(LCD_CK);
    gpio_pad_select_gpio(LCD_SH_LD);

    gpio_set_direction(LCD_DT_WR, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_SH_LD, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_CK, GPIO_MODE_OUTPUT);

    gpio_set_level(LCD_DT_WR, 0);
    gpio_set_level(LCD_SH_LD, 0);
    gpio_set_level(LCD_CK, 0);

    gpio_pad_select_gpio(TEC_DT_WR);
    gpio_pad_select_gpio(TEC_CK);
    gpio_pad_select_gpio(TEC_SH_LD);
    gpio_pad_select_gpio(TEC_DT_WR);
    gpio_set_direction(TEC_DT_WR, GPIO_MODE_OUTPUT);
    gpio_set_direction(TEC_SH_LD, GPIO_MODE_OUTPUT);
    gpio_set_direction(TEC_CK, GPIO_MODE_OUTPUT);
    gpio_set_direction(TEC_DT_RD, GPIO_MODE_INPUT);
    gpio_set_level(TEC_DT_WR,0);
    gpio_set_level(TEC_CK,0);
    gpio_set_level(TEC_SH_LD,0);

    gpio_pad_select_gpio(EXP_DT_WR);
    gpio_pad_select_gpio(EXP_CK);
    gpio_pad_select_gpio(EXP_SH_LD);
    gpio_pad_select_gpio(EXP_DT_WR);
    gpio_set_direction(EXP_DT_WR, GPIO_MODE_OUTPUT);
    gpio_set_direction(EXP_SH_LD, GPIO_MODE_OUTPUT);
    gpio_set_direction(EXP_CK, GPIO_MODE_OUTPUT);
    gpio_set_direction(EXP_DT_RD, GPIO_MODE_INPUT);
    gpio_set_level(EXP_DT_WR,0);
    gpio_set_level(EXP_CK,0);
    gpio_set_level(EXP_SH_LD,0);
    
    gpio_pad_select_gpio(IO_DT_WR);
    gpio_pad_select_gpio(IO_CK);
    gpio_pad_select_gpio(IO_SH_LD);
    gpio_set_direction(IO_DT_WR, GPIO_MODE_OUTPUT);
    gpio_set_direction(IO_SH_LD, GPIO_MODE_OUTPUT);
    gpio_set_direction(IO_CK, GPIO_MODE_OUTPUT);
    gpio_set_direction(IO_DT_RD, GPIO_MODE_INPUT);
    gpio_set_level(IO_DT_WR,0);
    gpio_set_level(IO_CK,0);
    gpio_set_level(IO_SH_LD,0);


}



