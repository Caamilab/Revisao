#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/util/datetime.h"

#include "Alarme.pio.h"
#include "inc/ssd1306.h"

// === Definições de pinos e constantes ===
#define LED_MATRIX_PIN     7
#define BUTTON_ENABLE_PIN  5
#define BUTTON_DISABLE_PIN 6
#define JOY_PIN_X          26
#define JOY_PIN_Y          27
#define BUZZER_PIN         21
#define I2C_SDA            14
#define I2C_SCL            15
#define I2C_PORT           i2c1
#define OLED_ADDR          0x3C

#define LED_ROWS           5
#define LED_COLS           5
#define JOY_CENTER         2048
#define ADC_MAX            4095
#define JOY_DEADZONE       200
#define PWM_WRAP           12500
#define SQUARE_SIZE        8
#define DEBOUNCE_DELAY_MS  200
#define LED_RGB_GREEN_PIN  11
#define LED_RGB_RED_PIN    13

// === Estruturas e variáveis globais ===
typedef struct { PIO pio; uint sm; } PioConfig;

static PioConfig led_cfg;
static ssd1306_t ssd;
static uint buzzer_slice;
static bool alarm_enabled = false;
static absolute_time_t alarm_start_time;
static uint8_t square_x = 60;
static uint8_t square_y = 28;

volatile static uint32_t last_interrupt_time = 0;
volatile static bool enable_triggered = false;
volatile static bool disable_triggered = false;
volatile static bool button_event_pending = false;
volatile static uint gpio_pin_pending = 0;

// === Protótipos ===
void init_matrix(PioConfig *cfg, uint pin);
void draw_zero(PioConfig *cfg);
void clear_matrix(PioConfig *cfg);
void setup_peripherals(void);
void process_joystick(void);
void enable_alarm(void);
void disable_alarm(void);
void update_square_position(int delta_x, int delta_y);
void draw_interface(void);
void gpio_callback(uint gpio, uint32_t events);
void blink_led(uint pin, int times);
void handleInput(void);

// === Função principal ===
int main(void) {
    stdio_init_all();
    printf("Inicializando sistema de alarme...\n");

    init_matrix(&led_cfg, LED_MATRIX_PIN);
    setup_peripherals();
    clear_matrix(&led_cfg);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
    pwm_set_gpio_level(BUZZER_PIN, 0);
    set_sys_clock_khz(128000, false);

    while (true) {
        if (enable_triggered && !alarm_enabled) {
            enable_triggered = false;
            enable_alarm();
        } else if (disable_triggered && alarm_enabled) {
            disable_triggered = false;
            disable_alarm();
        }

        process_joystick();
        sleep_ms(20);
    }
}

// === Inicializa a matriz de LEDs com programa PIO ===
void init_matrix(PioConfig *cfg, uint pin) {
    cfg->pio = pio0;
    cfg->sm = pio_claim_unused_sm(cfg->pio, true);
    uint offset = pio_add_program(cfg->pio, &pio_matrix_program);
    pio_matrix_program_init(cfg->pio, cfg->sm, offset, pin);
    printf("Matriz de LEDs inicializada.\n");
}

// === Desenha o número 0 na matriz de LEDs ===
void draw_zero(PioConfig *cfg) {
    static const uint8_t zero_map[LED_ROWS][LED_COLS] = {
        {1,1,1,1,1}, {1,0,0,0,1}, {1,0,0,0,1}, {1,0,0,0,1}, {1,1,1,1,1}
    };
    for (int r = LED_ROWS - 1; r >= 0; --r) {
        for (int c = 0; c < LED_COLS; ++c) {
            pio_sm_put_blocking(cfg->pio, cfg->sm, (zero_map[r][c] ? 255 : 0) << 16);
        }
    }
    printf("Número 0 desenhado na matriz.\n");
}

// === Limpa a matriz de LEDs ===
void clear_matrix(PioConfig *cfg) {
    for (int i = 0; i < LED_ROWS * LED_COLS; ++i) {
        pio_sm_put_blocking(cfg->pio, cfg->sm, 0);
    }
    printf("Matriz de LEDs limpa.\n");
}

// === Inicializa periféricos: joystick, I2C, OLED, PWM e botões ===
void setup_peripherals(void) {
    // Joystick (ADC)
    adc_init();
    adc_gpio_init(JOY_PIN_X);
    adc_gpio_init(JOY_PIN_Y);
    adc_fifo_setup(true, false, 1, false, true);
    adc_set_clkdiv(960);

    // OLED via I2C
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&ssd, 128, 64, false, OLED_ADDR, I2C_PORT);
    ssd1306_config(&ssd);

    // Buzzer (PWM)
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(buzzer_slice, &cfg, true);

    // Botões com interrupção
    gpio_init(BUTTON_ENABLE_PIN);
    gpio_set_dir(BUTTON_ENABLE_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_ENABLE_PIN);
    gpio_set_irq_enabled_with_callback(BUTTON_ENABLE_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(BUTTON_DISABLE_PIN);
    gpio_set_dir(BUTTON_DISABLE_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_DISABLE_PIN);
    gpio_set_irq_enabled_with_callback(BUTTON_DISABLE_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    printf("Periféricos configurados.\n");
}

// === Leitura do joystick e atualização da tela/buzzer ===
void process_joystick(void) {
    adc_select_input(1);
    uint16_t vx = adc_read();
    adc_select_input(0);
    uint16_t vy = adc_read();

    int dx_raw = (int)vx - JOY_CENTER;
    int dy_raw = (int)vy - JOY_CENTER;
    int dx = dx_raw / 512;
    int dy = dy_raw / 512;

    update_square_position(dx, -dy);
    draw_interface();

    if (alarm_enabled) {
        int mag = sqrtf(dx_raw * dx_raw + dy_raw * dy_raw);
        if (mag > JOY_DEADZONE) {
            int duty = (mag * PWM_WRAP) / ADC_MAX;
            pwm_set_gpio_level(BUZZER_PIN, duty > PWM_WRAP ? PWM_WRAP : duty);
            draw_zero(&led_cfg);
            alarm_start_time = get_absolute_time();
            printf("Movimento detectado. Alarme tocando.\n");
        } else if (absolute_time_diff_us(alarm_start_time, get_absolute_time()) >= 3000000) {
            pwm_set_gpio_level(BUZZER_PIN, 0);
            clear_matrix(&led_cfg);
            printf("Sem movimento. Alarme silenciado.\n");
        }
    }
}

// === Atualiza posição do quadrado (joystick) ===
void update_square_position(int delta_x, int delta_y) {
    int new_x = square_x + delta_x;
    int new_y = square_y + delta_y;

    if (new_x < 0) new_x = 0;
    else if (new_x > (128 - SQUARE_SIZE)) new_x = 128 - SQUARE_SIZE;

    if (new_y < 0) new_y = 0;
    else if (new_y > (64 - SQUARE_SIZE)) new_y = 64 - SQUARE_SIZE;

    square_x = new_x;
    square_y = new_y;
}

// === Atualiza display OLED com novo quadrado ===
void draw_interface(void) {
    ssd1306_fill(&ssd, false);
    ssd1306_rect(&ssd, square_y, square_x, SQUARE_SIZE, SQUARE_SIZE, true, false);
    ssd1306_send_data(&ssd);
}

// === Ativa alarme ===
void enable_alarm(void) {
    alarm_enabled = true;
    alarm_start_time = get_absolute_time();
    ssd1306_send_data(&ssd);
    blink_led(LED_RGB_GREEN_PIN, 2);
    printf("Alarme ativado.\n");
}

// === Desativa alarme ===
void disable_alarm(void) {
    alarm_enabled = false;
    pwm_set_gpio_level(BUZZER_PIN, 0);
    clear_matrix(&led_cfg);
    ssd1306_send_data(&ssd);
    blink_led(LED_RGB_RED_PIN, 2);
    printf("Alarme desativado.\n");
}

// === Callback de interrupção dos botões ===
void gpio_callback(uint gpio, uint32_t events) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_interrupt_time >= DEBOUNCE_DELAY_MS) {
        last_interrupt_time = current_time;
        button_event_pending = true;
        gpio_pin_pending = gpio;
        handleInput();
    }
}

// === Pisca LED RGB ===
void blink_led(uint pin, int times) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    for (int i = 0; i < times; ++i) {
        gpio_put(pin, 1);
        sleep_ms(150);
        gpio_put(pin, 0);
        sleep_ms(150);
    }
    printf("LED %d piscou %d vezes.\n", pin, times);
}

// === Trata evento de botão pressionado ===
void handleInput(void) {
    if (gpio_pin_pending == BUTTON_ENABLE_PIN) {
        enable_triggered = true;
        printf("Botão ATIVAR pressionado.\n");
    } else if (gpio_pin_pending == BUTTON_DISABLE_PIN) {
        disable_triggered = true;
        printf("Botão DESATIVAR pressionado.\n");
    }
}
