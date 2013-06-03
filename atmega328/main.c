#define F_CPU 1000000UL                                    /* Clock Frequency = 1Mhz */

#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>

int main(){                         // The main function
    DDRC = (1 << DDC1);
    DDRD = (1 << DDD2) | (0 << DDD3) | (1 << DDD4);

    PORTC = 0;
    PORTD = (1 << PORTD3);

    while (1);
}
