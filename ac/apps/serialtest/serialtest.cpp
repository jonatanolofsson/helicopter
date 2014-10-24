#include <wirish/wirish.h>

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {init();Serial3.begin(115200);}

int main(void) {
    pinMode(BOARD_LED_PIN, OUTPUT);
    bool a = true;
    do {
        digitalWrite(BOARD_LED_PIN, a);
        a = !a;
        Serial3.write("Hej\r\n");
    } while(Serial3.read());
    return 0;
}
