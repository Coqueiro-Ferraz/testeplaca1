#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc_cal.h"//?
#include <stdio.h>//?
#include "esp_system.h"//?
#include "driver/adc.h"//?
#include "driver/ledc.h"
#include "connect.h"
#include "nvs_flash.h"

// Definições de pinos de entrada e saída via registradores
// Strap pins para evitar: 0, 1, 2, 3, 5, 12, 15
// Apenas entradas: 34, 35, 36, 37, 38, 39 
// roteados para ADC: 36 (ADC1_CH0) e 39 (ADC1_CH3)
// roteados para DHT: 22 e 23

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
// Experiência com servo motor 9g
//#define SERVO_GPIO  GPIO_NUM_17
// Para o micro servo motor Min Pulse = 1000us, max pulse = 2000us
#define MAX_DUTY 2500
#define MIN_DUTY 500

#define TAG_WIFI "WIFI CON"

uint8_t dado_atual;
uint8_t linha = 1;
uint8_t coluna;//, oldmostra;
volatile uint8_t mostra = 0;
uint8_t oldMostra = 0;
char tecla = '_';
bool flagMudou;
static const char* TAG_LCD = "LCD";
static const char* TAG_TEC = "TEC";
static const char* TAG_IO = "I/O";
uint8_t ativado = 0;
int angulo = 0;
uint8_t entradas = 0;
uint8_t oldEntradas = 0;


/* //função PWM
void pwm_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = Externo,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}
void app_main(void)
{
    pwm_init();
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    gpio_pad_select_gpio(BT_PIN);
    gpio_set_direction (BT_PIN, GPIO_MODE_INPUT); 
   // gpio_pad_select_gpio(Externo);
   // gpio_set_direction (Externo, GPIO_MODE_OUTPUT);    
    gpio_pad_select_gpio(Embarcado);
    gpio_set_direction (Embarcado, GPIO_MODE_OUTPUT);    
    bool status = 0;
    bool entrada = 0;
    uint percentual = 0;

    while(1)
    {
        entrada = gpio_get_level(BT_PIN);
        if(entrada==true)
        {
            if(percentual>7000)
            {
                percentual = 0;
            }
            else
            {
                percentual += 1000;
            }
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, percentual);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        }

    }

}

*/
void set_servo_angle(int angle)
{
    //código do teste do servo motor
    int i;
    int duty = (angle / 180.0) * (MAX_DUTY - MIN_DUTY) + MIN_DUTY;
    for (i=0;i<10;i++)
    {
        gpio_set_level(EXP_CK, 1);
        ets_delay_us(duty);
        gpio_set_level(EXP_CK, 0);//servo_gpio
        ets_delay_us(20000 - duty);    
    }
    //fim do teste do servo motor
       
}

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

uint8_t Comunica_exp (uint8_t enviar)//>26 microssegundos
{
    uint8_t recebido = 0;
    int j;
    gpio_set_level(EXP_SH_LD,1);
    ets_delay_us(1);  
    for (j = 7; j >= 0; j--)
    {
        recebido <<= 1;
        recebido += gpio_get_level(EXP_DT_RD);
        gpio_set_level(EXP_DT_WR, ( enviar >> j ) & 1);
        ets_delay_us(1);  
        gpio_set_level(EXP_CK,1);
        ets_delay_us(1);  
        gpio_set_level(EXP_CK,0);
        ets_delay_us(1);  
    } 
    gpio_set_level(EXP_SH_LD,0);
    ets_delay_us(1);  

    return recebido;
}


void lcd_pulse_enable(void)
{
    Enviar_dado_lcd(dado_atual | 0b00001000);
    vTaskDelay(2 / portTICK_RATE_MS);

    Enviar_dado_lcd(dado_atual & 0b11110111);
    vTaskDelay(2 / portTICK_RATE_MS);
}

void lcd_write_byte(uint8_t byte, uint8_t rs)
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

void lcd_write_string(const char *str)
{
    while (*str)
    {
        lcd_write_byte(*str++, 1);
    }
}

void lcd_init(void)
{
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

    lcd_write_byte(0x28, 0);
    lcd_write_byte(0x0C, 0);
    lcd_write_byte(0x01, 0);
    vTaskDelay(2 / portTICK_RATE_MS);
    dado_atual = 0x00;
}

void lcd_clear(void)
{
    lcd_write_byte(0x01, 0);
    vTaskDelay(2 / portTICK_RATE_MS);
}

void Ativa_tudo(uint8_t estado)
{
    int j;
    gpio_set_level(IO_SH_LD,1);
    ets_delay_us(10); //vTaskDelay(10 / portTICK_RATE_MS);  
    for (j = 7; j >= 0; j--)
    {
        entradas <<= 1;
        entradas += gpio_get_level(IO_DT_RD);
        gpio_set_level(IO_DT_WR, ( estado >> j ) & 1);
        ets_delay_us(10); //vTaskDelay(10 / portTICK_RATE_MS);
        gpio_set_level(IO_CK,1);
        ets_delay_us(10); //vTaskDelay(10 / portTICK_RATE_MS);
        gpio_set_level(IO_CK,0);
        ets_delay_us(10); //vTaskDelay(10 / portTICK_RATE_MS);
    } 
    gpio_set_level(IO_SH_LD,0);
    ets_delay_us(10); //vTaskDelay(10 / portTICK_RATE_MS); 

}

void motor_avanca (int passos)
{
    int i,j;
    uint8_t steps [8] = {0x80,0xC0,0x40,0x60,0x20,0x30,0x10,0x90};
    ativado &= 0x0F;
    for (j=0;j<passos;j++)
    {
        for(i=0;i<8;i++)
        {
            ativado &= 0x0F;
            ativado |= steps[i];
            Ativa_tudo(ativado);
            vTaskDelay(10 / portTICK_RATE_MS);
        }
    }
    mostra = 0;
    ativado &= 0x0F;
    Ativa_tudo(ativado);
    vTaskDelay(10 / portTICK_RATE_MS);
}

void setalinha_2(void)
{
    if(linha <= 1)
    {
        linha = 0b1000;
    }
    else
    {
        linha >>= 1;
    }
}
void le_teclado_2 ()
{
    while(1)
    {        
        int i;
        int k = 0;
        int j = 0;  
        gpio_set_level(TEC_SH_LD,1);
        coluna = 0;
        for(i = 7; i >= 0; i--)
        {    
            if(i < 4 )
            {
                if(gpio_get_level(TEC_DT_RD) == 1)
                {
                    coluna = i;
                    mostra = linha * 10 + coluna;
                  //  ESP_LOGI("captura", "L %i - C %i", linha, coluna);
                    vTaskDelay(100 / portTICK_RATE_MS);
                }
            }


            gpio_set_level(TEC_DT_WR, (linha >> i) & 1 );
            vTaskDelay(2 / portTICK_RATE_MS);
            gpio_set_level(TEC_CK, 1);
            vTaskDelay(2 / portTICK_RATE_MS);
            gpio_set_level(TEC_CK, 0);
            
        }
        gpio_set_level(TEC_SH_LD,0);
        setalinha_2();
    }
}

void selTec()
{
    
    switch (mostra)
    {
        case 10: tecla = '1';
                ativado |= 0x01;
                break;
        case 11: tecla = '2';
                ativado |= 0x02;
                break;
        case 12: tecla = '3';
                ativado |= 0x04;
                break;  
        case 13: tecla = '-';
                if(angulo>10)angulo-=10;
                break; 
        case 20: tecla = 'C';
                lcd_init();
                break; 
        case 21: tecla = '0';
                ativado = 0;                
                angulo = 0;
                break; 
        case 22: tecla = '=';
                angulo = 90;
                break; 
        case 23: if(angulo<170)angulo+=10;
                motor_avanca(64);
                tecla = '+';
                break; 
        case 40: tecla = '7';
                ativado |= 0x40;
                break; 
        case 41: tecla = '8';
                ativado |= 0x80;
                break; 
        case 42: tecla = '9';
                angulo = 180;
                break; 
        case 43: tecla = '/';
                break;
        case 80: tecla = '4';
                ativado |= 0x08;
                break; 
        case 81: tecla = '5';
                ativado |= 0x10;
                break; 
        case 82: tecla = '6';
                ativado |= 0x20;
                break; 
        case 83: tecla = 'x';
                break; 
        default: tecla = '_';
                break;                
    }
    Ativa_tudo(ativado);
    set_servo_angle(angulo);
}

void chama_adc()
{
    int valor = 0;
    valor = adc1_get_raw(ADC1_CHANNEL_0);
    printf("ADC1:%d\n", valor);
}

void rotina()
{
    while (1)
    {
        char exibe[9] = {'x','x','x','x','x','x','x','x'};
        int i;
        exibe[8] = 0;

    //    ESP_LOGE("capturado", "%i", mostra);
        for(i=0;i<8;i++)
        {
            exibe[i] = ((entradas >> i) & 1) + 48;
        }
        
        vTaskDelay(200 / portTICK_RATE_MS); 

        //chama_adc();

        selTec();
        vTaskDelay(10 / portTICK_RATE_MS); 
        lcd_write_byte(0x80,0);
        lcd_write_string(&exibe[0]);
        lcd_write_byte(0xC0,0);
        lcd_write_byte(tecla,1);
     
    }
}

void wifi_connect_task (void *args)
{
    esp_err_t err = wifi_connect_sta("Hcflaf", "$4Xcoqueiro", 10000);
    if(err)
    {
        ESP_LOGE( TAG_WIFI, "Conexao falhou");
        vTaskDelete(NULL);
    }
    printf("aguardando... \n");
    vTaskDelay(5000 / portTICK_RATE_MS);
    wifi_disconnect();
    vTaskDelete(NULL);
}

void app_main(void)
{
    int i;

    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();
    xTaskCreate(wifi_connect_task, "wifi_connect_task", 1024*5, NULL, 5, NULL);

    gpio_pad_select_gpio(LCD_DT_WR);
    gpio_pad_select_gpio(LCD_CK);
    gpio_pad_select_gpio(LCD_SH_LD);

    gpio_set_direction(LCD_DT_WR, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_SH_LD, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_CK, GPIO_MODE_OUTPUT);

    gpio_set_level(LCD_CK, 0);

/*
    vTaskDelay(10 / portTICK_RATE_MS); 
   // lcd_write_byte(0xC0,0);
    vTaskDelay(10 / portTICK_RATE_MS); 
    //lcd_write_string("SENAI SELDI/MSEL");
    vTaskDelay(200 / portTICK_RATE_MS); 
    //lcd_clear();*/

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
 
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    
    vTaskDelay(200 / portTICK_RATE_MS); 
    lcd_init();
    vTaskDelay(200 / portTICK_RATE_MS); 

    for(i=0;i<4;i++)
    {
        lcd_write_string(">   HAGACEEF   <");
        vTaskDelay(70 / portTICK_RATE_MS); 
        lcd_write_byte (0x80,0);
        lcd_write_string(">>  HAGACEEF  <<");
        vTaskDelay(10 / portTICK_RATE_MS); 
        lcd_write_byte (0x80,0);
        lcd_write_string(" >  HAGACEEF  < ");
        vTaskDelay(70 / portTICK_RATE_MS); 
        lcd_write_byte (0x80,0);
        lcd_write_string(" >> HAGACEEF << ");
        vTaskDelay(10 / portTICK_RATE_MS); 
        lcd_write_byte (0x80,0);
        lcd_write_string("  > HAGACEEF <  ");
        vTaskDelay(70 / portTICK_RATE_MS); 
        lcd_write_byte (0x80,0);
        lcd_write_string("  >>HAGACEEF<<  ");
        vTaskDelay(10 / portTICK_RATE_MS); 
        lcd_write_byte (0x80,0);
        lcd_write_string("   >HAGACEEF<   ");
        vTaskDelay(80 / portTICK_RATE_MS); 
        lcd_write_byte (0x80,0);
        lcd_write_string("    HAGACEEF    ");
        vTaskDelay(100 / portTICK_RATE_MS); 
        lcd_write_byte (0x80,0);
    }
    lcd_write_string("   HAGA()CEEF   ");
    vTaskDelay(10 / portTICK_RATE_MS); 
    lcd_write_byte (0x80,0);
    lcd_write_string(" H AG (()) EE F ");
    vTaskDelay(10 / portTICK_RATE_MS); 
    lcd_write_byte (0x80,0);
    lcd_write_string("( ( (( () )) ) )");
    vTaskDelay(10 / portTICK_RATE_MS); 
    lcd_write_byte (0x80,0);
    lcd_write_string("((((((    ))))))");
    vTaskDelay(10 / portTICK_RATE_MS); 
    lcd_write_byte (0x80,0);
    lcd_write_string("(((          )))");
    vTaskDelay(10 / portTICK_RATE_MS); 
    lcd_write_byte (0x80,0);        
    lcd_write_string("(              )");
    vTaskDelay(10 / portTICK_RATE_MS); 
    lcd_write_byte (0x80,0);

    vTaskDelay(500 / portTICK_RATE_MS); 
    lcd_clear();

    

    xTaskCreate(&le_teclado_2,"Leitura do teclado",2048,NULL,2,NULL);
    xTaskCreate(&rotina,"Rotina",2048,NULL,3,NULL);

}


