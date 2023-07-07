#ifndef USK_LEDS_H
#define USK_LEDS_H

#define PIX_blu 0x00003F
#define PIX_yel 0x151500
#define PIX_whi 0x111111

#define PIX_b 0x00000F

#define PIX_off 0x000000

//#define NUM_PIXELS 9
#define NUM_PIXELS 30

enum
{
    LED_ANIM_NONE,
    LED_ANIM_ONE_COLOR,
    LED_ANIM_MULTI_COLOR,
    LED_ANIM_CYCLE,
    LED_ANIM_RAINBOW,
};

void put_pixel(uint32_t pixel_grb);

uint32_t HSVtoRGB(float fH, float fS, float fV);

void leds_mode();

#endif //USK_LEDS_H
