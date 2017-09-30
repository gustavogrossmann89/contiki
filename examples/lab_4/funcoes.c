#include "dev/leds.h"
#include "project-conf.h"

void controlaLed(int inst){
    if(inst == 0){
        leds_on(LEDS_GREEN);
    } else if(inst == 1){
        leds_on(LEDS_RED);
    } else if(inst == 2){
        leds_on(LEDS_GREEN);
        leds_on(LEDS_RED);
    } else if(inst == 3){
        leds_off(LEDS_GREEN);
        leds_off(LEDS_RED);
    }
}
