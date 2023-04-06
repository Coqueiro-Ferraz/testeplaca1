#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdio.h>
#include "esp_system.h"
//#include "ioplaca.h"
/*
uint8_t dado_atual;

void Enviar_dado_lcd(uint8_t dado)
{
    int i;    
   // ESP_LOGI(TAG_LCD, "%x", dado);
    gpio_set_level(LCD_CK, 0);
    for(i = 7; i >= 0; i--)
    { 
        gpio_set_level(LCD_DT_WR, (dado >> i) & 1 );
        vTaskDelay(1 / portTICK_RATE_MS);
        gpio_set_level(LCD_CK, 1);
        vTaskDelay(1 / portTICK_RATE_MS);
        gpio_set_level(LCD_CK, 0);
        vTaskDelay(1 / portTICK_RATE_MS);
    }
    gpio_set_level(LCD_SH_LD, 1);
    vTaskDelay(1 / portTICK_RATE_MS);
    gpio_set_level(LCD_SH_LD, 0);
}

void lcd_pulse_enable(void)
{
    Enviar_dado_lcd(dado_atual | 0b00001000);
    vTaskDelay(2 / portTICK_RATE_MS);

    Enviar_dado_lcd(dado_atual & 0b11110111);
    vTaskDelay(2 / portTICK_RATE_MS);
}

void lcd595_byte(uint8_t byte, uint8_t rs)
{
    dado_atual &= 0b11111011;
    dado_atual |= (rs * 0b100);                   // pino RS
    Enviar_dado_lcd(dado_atual);
    dado_atual &= 0x0F;                         // limpa dados
    dado_atual |= (byte & 0xF0);                // nibble alto
    Enviar_dado_lcd(dado_atual);
    lcd_pulse_enable();
    dado_atual &= 0x0F;
    dado_atual |= ((byte & 0x0F) * 0b10000);    // nibble baixo
    Enviar_dado_lcd(dado_atual); 
    lcd_pulse_enable();

    vTaskDelay(1 / portTICK_RATE_MS);
}


void lcd595_write(const char *str)
{
    while (*str)
    {
        lcd595_byte(*str++, 1);
    }
}

void lcd595_init(void)
{
    vTaskDelay(200 / portTICK_RATE_MS); 
    dado_atual = 0x00;

    vTaskDelay(15 / portTICK_RATE_MS); 

  //  gpio_set_direction(LCD_DT_WR, GPIO_MODE_OUTPUT);
  //  gpio_set_direction(LCD_SH_LD, GPIO_MODE_OUTPUT);
  //  gpio_set_direction(LCD_CK, GPIO_MODE_OUTPUT);

    vTaskDelay(15 / portTICK_RATE_MS);

    dado_atual = 0x30;
    Enviar_dado_lcd(dado_atual);
    lcd_pulse_enable();
    vTaskDelay(5 / portTICK_RATE_MS);

    dado_atual = 0x30;
    Enviar_dado_lcd(dado_atual);
    lcd_pulse_enable();
    vTaskDelay(1 / portTICK_RATE_MS);

    dado_atual = 0x30;
    Enviar_dado_lcd(dado_atual);
    lcd_pulse_enable();
    vTaskDelay(1 / portTICK_RATE_MS);

    dado_atual = 0x20;
    Enviar_dado_lcd(dado_atual);
    lcd_pulse_enable();
    vTaskDelay(1 / portTICK_RATE_MS);

    lcd595_byte(0x28, 0);
    lcd595_byte(0x0C, 0);
    lcd595_byte(0x01, 0);
    vTaskDelay(2 / portTICK_RATE_MS);
    dado_atual = 0x00;
    vTaskDelay(200 / portTICK_RATE_MS); 
}

void lcd595_clear(void)
{
    lcd595_byte(0x01, 0);
    vTaskDelay(2 / portTICK_RATE_MS);
}
*/