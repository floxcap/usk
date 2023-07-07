#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/vreg.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include <string.h>
#include <math.h>
#include "leds.h"
#include "pins.h"
#include "ws2812.pio.h"
#include "board_detect.h"
#include "misc.h"
#include "board_detect.h"

extern int ws_pio_offset;

static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000; // 100 kHz

// The slave implements a 256 byte memory. To write a series of bytes, the master first
// writes the memory address, followed by the data. The address is automatically incremented
// for each byte transferred, looping back to 0 upon reaching the end. Reading is done
// sequentially from the current memory address.
static struct
{
    volatile uint8_t mem[256];
    volatile uint8_t mem_address;
    volatile bool mem_address_written;
    volatile bool update_ready;
    volatile bool write;
} i2c_context;

uint32_t pixel_buf[NUM_PIXELS];

void start_pixel()
{
    static bool led_enabled = false;
    ws2812_program_init(pio0, 3, ws_pio_offset, led_pin(), 800000, false);
    sleep_us(50);
    if (!led_enabled && pwr_pin() != 31)
    {
        led_enabled = true;
        gpio_init(pwr_pin());
        gpio_set_drive_strength(pwr_pin(), GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_dir(pwr_pin(), true);
        gpio_put(pwr_pin(), 1);
        sleep_us(200);
    }
}

void stop_pixel()
{
    sleep_us(50);
    pio_sm_set_enabled(pio0, 3, false);
    gpio_init(led_pin());
}

uint32_t HSVtoRGB(float fH, float fS, float fV)
{
    double fR, fG, fB;
    double fC = fV * fS; // Chroma
    double fHPrime = fmod(fH / 60.0, 6);
    double fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
    double fM = fV - fC;

    if (0. <= fHPrime && fHPrime < 1.)
    {
        fR = fC;
        fG = fX;
        fB = 0;
    }
    else if (1. <= fHPrime && fHPrime < 2.)
    {
        fR = fX;
        fG = fC;
        fB = 0;
    }
    else if (2. <= fHPrime && fHPrime < 3.)
    {
        fR = 0;
        fG = fC;
        fB = fX;
    }
    else if (3. <= fHPrime && fHPrime < 4.)
    {
        fR = 0;
        fG = fX;
        fB = fC;
    }
    else if (4. <= fHPrime && fHPrime < 5.)
    {
        fR = fX;
        fG = 0;
        fB = fC;
    }
    else if (5. <= fHPrime && fHPrime < 6.)
    {
        fR = fC;
        fG = 0;
        fB = fX;
    }
    else
    {
        fR = 0;
        fG = 0;
        fB = 0;
    }

    fR += fM;
    fG += fM;
    fB += fM;

    return ((uint32_t)(fR * 255) << 16) | ((uint32_t)(fG * 255) << 8) | (uint32_t)(fB * 255);
}

void put_pixel(uint32_t pixel_grb)
{
    if (is_pico())
    {
        gpio_init(led_pin());
        if (pixel_grb) {
            gpio_set_dir(led_pin(), true);
            gpio_put(led_pin(), 1);
        }
        return;
    }

    start_pixel();
    for (uint i = 0; i < NUM_PIXELS; ++i) {
        pio_sm_put_blocking(pio0, 3, pixel_grb << 8u);
    }
    stop_pixel();
}

void set_pixels(uint32_t* pixel_grb, uint len)
{
    for (uint i = 0; i < len; ++i) {
        pixel_buf[i] = (pixel_grb[i] << 8u);
    }
}

void set_pixel(uint32_t pixel_grb, int index)
{
    if (index < 0) {
        for (uint i = 0; i < NUM_PIXELS; ++i) {
            pixel_buf[i] = (pixel_grb << 8u);
        }
    } else if (index < NUM_PIXELS) {
        pixel_buf[index] = (pixel_grb << 8u);
    }
}

void update_pixels()
{
    for (uint i = 0; i < NUM_PIXELS; ++i) {
        pio_sm_put_blocking(pio0, 3, pixel_buf[i]);
    }
}


// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
    switch (event) {
        case I2C_SLAVE_RECEIVE: // master has written some data
            if (!i2c_context.mem_address_written) {
                // writes always start with the memory address
                i2c_context.mem_address = i2c_read_byte_raw(i2c);
                i2c_context.mem_address_written = true;
            } else {
                // save into memory
                i2c_context.mem[i2c_context.mem_address] = i2c_read_byte_raw(i2c);
                i2c_context.mem_address++;
                i2c_context.write = true;
            }
            break;
        case I2C_SLAVE_REQUEST: // master is requesting data
            // load from memory
            i2c_write_byte_raw(i2c, i2c_context.mem[i2c_context.mem_address]);
            i2c_context.mem_address++;
            i2c_context.write = false;
            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            i2c_context.mem_address_written = false;
            if (i2c_context.write)
            {
                i2c_context.update_ready = true;
            }
            break;
        default:
            break;
    }
}

static void setup_slave()
{
    memset(&i2c_context, 0, sizeof(i2c_context));

    gpio_init(sda_pin());
    gpio_set_function(sda_pin(), GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin());

    gpio_init(scl_pin());
    gpio_set_function(scl_pin(), GPIO_FUNC_I2C);
    gpio_pull_up(scl_pin());

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

bool repeating_timer_callback(struct repeating_timer *t) {
    return true;
}

void leds_mode()
{
    display_error(0, 1);
    /*
    if (is_jmp_leds())
    {
        display_error(0, 1);
    }
    else
    {
        halt_with_error(0, 1);
    }
    */

    setup_slave();

    start_pixel();

    uint8_t mode = LED_ANIM_NONE;
    uint8_t speed = 100;
    uint32_t color = 0x00FFFFFF;

    while (1)
    {
        uint32_t ms = to_ms_since_boot(get_absolute_time());

        if (i2c_context.update_ready)
        {
            mode = i2c_context.mem[0];
//            speed = i2c_context.mem[1];
//            color = i2c_context.mem[2];
//            color |= i2c_context.mem[3] << 16;
//            color |= i2c_context.mem[4] << 8;

            i2c_context.update_ready = false;
        }

        switch (mode)
        {
            default:
            case LED_ANIM_NONE:
                set_pixel(PIX_off, -1);
                break;

            case LED_ANIM_ONE_COLOR:
                set_pixel(color, -1);
                break;

            case LED_ANIM_MULTI_COLOR:
                set_pixel(color, -1);
                break;

            case LED_ANIM_CYCLE:
            {
                long s = speed*100;
                float p2 = (ms % s)/(float)s;
                uint32_t c = HSVtoRGB(p2*360, 1, 1);
                set_pixel(c, -1);
            }
            break;

            case LED_ANIM_RAINBOW:
            {
                long s = speed*100;
                float p2 = (ms % s)/(float)s;
                uint step = 360/NUM_PIXELS;
                for (uint i=0; i<NUM_PIXELS; i++)
                {
                    pixel_buf[i] = (HSVtoRGB((int)(p2*360+step*i) % 360, 1, 1) << 8u);
                }
            }
            break;
        }

        update_pixels();

        sleep_ms(50);
    }

    //stop_pixel();
}
