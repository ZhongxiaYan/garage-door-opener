#include <avr/sleep.h>
#include <avr/interrupt.h>

const int STATUS_LED = 4;
const int PIN_CHANGE_INTERRUPT[] = { PCINT0, PCINT1, PCINT2, PCINT3 };
const int NUM_INTERRUPT_PINS = sizeof(PIN_CHANGE_INTERRUPT) / sizeof(PIN_CHANGE_INTERRUPT[0]);

const int NUM_SECONDS_TO_WAIT = 5;
const int NUM_TICKS_PER_SECOND = 4;
const int NUM_TICKS_TO_WAIT = NUM_SECONDS_TO_WAIT * NUM_TICKS_PER_SECOND;

const int NUM_PASSWORD_DIGITS = 4;

int num_ticks_waiting = 0;

int pins_entered[NUM_PASSWORD_DIGITS];
int next_pin_index = 0;

void setup() {
    pinMode(STATUS_LED, OUTPUT);

    digitalWrite(STATUS_LED, HIGH);
    delay(1000);
    digitalWrite(STATUS_LED, LOW);
    delay(1000);

    /** configure keypad input pins to interrupt when MCU sleeping **/
    for (int i = 0; i < NUM_INTERRUPT_PINS; i++) {
        int pin = PIN_CHANGE_INTERRUPT[i];
        pinMode(pin, INPUT);
        digitalWrite(pin, HIGH);
    }
    GIMSK |= (1 << PCIE);                     // enable pin change interrupts
    for (int i = 0; i < NUM_INTERRUPT_PINS; i++) { // enable pin interrupt for the given pins
        PCMSK |= (1 << PIN_CHANGE_INTERRUPT[i]);
    }
    ADCSRA &= ~(1 << ADEN);                   // ADC off

    /** configure timer to sleep after a duration **/
    OCR0A = 244; // value for TCNT0 to count to
    OCR0B = 244;
    TCCR0A |= (1 << WGM01); // CTC mode
    TCCR0B = (1 << CS02) | (1 << CS00); // clear and set prescaler to 1024
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // allows MCU to sleep

    /** enable interrupts **/
    sei();
    sleep_mode();
}

/** keypad switches are inverted (i.e. LOW when pressed) **/
int read_inputs() {
    return (~(digitalRead(3) << 3 | digitalRead(2) << 2 | digitalRead(1) << 1 | digitalRead(0) << 0)) & 0xF;
}

/** wake CPU upon keypad pin interrupt, set timer for sleep **/
ISR(PCINT0_vect) {
    digitalWrite(STATUS_LED, 1);
    delay(20); // debouncing time
    int keypad_digit = read_inputs();
    pins_entered[next_pin_index++] = keypad_digit;
    if (next_pin_index >= NUM_PASSWORD_DIGITS) { // 4 digits entered
        next_pin_index = 0;
        for (int i = 0; i < 3; i++) {
            digitalWrite(STATUS_LED, 0);
            delay(200);
            digitalWrite(STATUS_LED, 1);
            delay(200);
        }
    }
    while (read_inputs() != 0x0) { // wait for switches to be switched back to original state
        delay(20);
    }
    delay(20); // debouncing time
    
    GIFR |= (1 << PCIF); // clear queued PCINT0 interrupts

    /** start timer to sleep MCU after some time **/
    num_ticks_waiting = 0;
    TIMSK |= (1 << OCIE0A); // activate timer 0 interrupt for compare with OCR0A
}

/** timer tick for sleep **/
ISR(TIM0_COMPA_vect) {
    num_ticks_waiting += 1;
    
    /** sleep is activated **/
    if (num_ticks_waiting % NUM_TICKS_TO_WAIT == 0) {
        next_pin_index = 0; // clear all the digits upon sleep
        
        TIMSK &= ~(1 << OCIE0A); // deactivate timer 0 interrupt for compare with OCR0A
        digitalWrite(STATUS_LED, 0);

        sleep_enable();
        sleep_bod_disable();
        sei();
        sleep_cpu();
        sleep_disable();
    }
}

void loop() {
//    digitalWrite(STATUS_LED, HIGH);
//    delay(400);
//    digitalWrite(STATUS_LED, LOW);
//    delay(400);
}
