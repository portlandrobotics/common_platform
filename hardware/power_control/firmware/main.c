#define F_CPU (20000000L/6)
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

enum powerstate {POWER_OFF=0, POWER_SYSONLY, POWER_ON};
#define SLEEP_RESET 15
// address chosen randomly
#define I2C_ADDR 0x6e

void stateupdate(enum powerstate newstate);

// *** global state variables ***
//  note that a few of the ISRs also have static variables

// current power state (leave this POWER_OFF, if you want a different
//    startup default, uncomment the line in main() )
volatile static enum powerstate curstate = POWER_OFF;

// most recent ADC reading
volatile static int16_t lastvoltage;

// shift register of recent ADC readings against LVC
volatile static uint8_t voltage_status=0xff;

// shutdown countdown timer in seconds, disabled when 0
volatile static uint8_t shutdown_pending = 0;

// LVC voltage level (settable at runtime by I2C)
//   492 = 6 cells * 0.8 volts per cell / 4x voltage divider / 2.5V reference * 1024 counts
volatile static uint16_t low_voltage_cutoff = 492;

// timer to actually put microcontroller into sleep mode
//   go to sleep if the power to the robot is OFF and the button
//   hasn't been pressed for 30 seconds
volatile static uint8_t sleep_countdown = SLEEP_RESET;

// *** end global state variables ***

// fires every 2 seconds (16-bit overflow of a 32.768 KHz counter)
ISR(RTC_CNT_vect) {
    // clear interrupt flag
    RTC.INTFLAGS = RTC_OVF_bm;

    // only decriment the sleep timer if the power state is off
    if(curstate == POWER_OFF && sleep_countdown > 0)
        sleep_countdown--;

    // if the shutdown timer is enabled (0 is disabled),
    //   count down.  When the timer hits 1, power down
    if(shutdown_pending != 0) {
        if(shutdown_pending==1) {
            stateupdate(POWER_OFF);
            shutdown_pending=0;
        }
        else if(shutdown_pending > 3)
            shutdown_pending-=2;
        else
            shutdown_pending=1;
    }

    // ask for an ADC measurement
    ADC0.COMMAND = ADC_STCONV_bm;
}

// whenever an ADC result is availabe, read and store the result
//   also check it against the LVC, and schedule a shutdown if needed
ISR(ADC0_RESRDY_vect) {
    lastvoltage=ADC0.RES;
    voltage_status = voltage_status<<1;
    if(lastvoltage >= low_voltage_cutoff)
        voltage_status |= 1;

    // shutdown if 3 consecutive samples were below LVC
    if(curstate != POWER_OFF && (voltage_status & 0x07)==0) {
        // immediately cut motors, give system 60 seconds for an orderly shutdown
        stateupdate(POWER_SYSONLY);
        if(shutdown_pending == 0)
            shutdown_pending=60;
    }


}

// button activity wakes the device from sleep, and resets the sleep countdown timer
ISR(PORTA_PORT_vect) {
    // clear the interrupt flag
    PORTA.INTFLAGS = 1<<7;

    sleep_countdown=15;
}

// handle I2C
ISR(TWI0_TWIS_vect) {
    // current address pointer, preserved between transactions
    static uint8_t twi_addr;
    // the first byte of each write transaction sets the address, not the data
    static uint8_t twi_first_write=0;

    uint8_t status = TWI0.SSTATUS;

    // address or stop
    if(status & TWI_APIF_bm) {
        if(status & TWI_AP_bm) {
            // address interrupt
            TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
            if(!(status & TWI_DIR_bm))
                twi_first_write=1;
        } else {
            // stop
            TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
        }
    }
    // data interrupt (read or write)
    if(status & TWI_DIF_bm) {
        if(status & TWI_DIR_bm) {
            // read
            switch(twi_addr) {
            case 0:
                TWI0.SDATA = curstate;
                break;
            case 1:
                TWI0.SDATA = shutdown_pending;
                break;
            case 2:
                TWI0.SDATA = lastvoltage >> 2;
                break;
            case 3:
                TWI0.SDATA = low_voltage_cutoff >> 2;
                break;
            case 4:
                TWI0.SDATA = sleep_countdown;
                break;
            case 5:
                TWI0.SDATA = voltage_status;
                break;
            default:
                TWI0.SDATA = 0;
            }
            twi_addr++;
        } else {
            // write
            if(twi_first_write) {
                twi_addr=TWI0.SDATA;
                twi_first_write=0;
            }
            else {
                uint8_t rxdata = TWI0.SDATA;
                switch(twi_addr) {
                case 0:
                    stateupdate(rxdata);
                    break;
                case 1:
                    shutdown_pending = rxdata;
                    break;
                case 2:
                    // lastvoltage measurement is read only, ignore
                    break;
                case 3:
                    low_voltage_cutoff = rxdata << 2;
                    break;
                }
                twi_addr++;
            }
        }
        // acknowlege the read or write
        TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
    }
}

// PIT fires every 1/1024th of a second while awake
ISR(RTC_PIT_vect) {
    // clear interrupt flag
    RTC.PITINTFLAGS = RTC_PI_bm;

    static uint8_t button=0xff;
    static enum {BUTTON_UP, BUTTON_DOWN, BUTTON_LINGER} button_state=BUTTON_UP;
    static uint16_t button_state_time_ms;

    // shift register for button history
    //   we recognize a button as down when it gets 3 "down" readings in a row
    //   we recognize a button rise when it gets 4 "up" readings in a row
    button = button << 1;
    if(PORTA.IN & (1<<7))
        button |= 1;

    if(button_state==BUTTON_UP && (button&0x07)==0) {
        // button debounced fall
        button_state=BUTTON_DOWN;
        button_state_time_ms=0;
    }
    else if(button_state==BUTTON_DOWN) {
        // measure how long the button has been down in milliseconds, up to a ceiling
        //    of ~64 seconds (don't wrap around)
        if(button_state_time_ms < 0xffff)
            button_state_time_ms ++;

        uint8_t buttonaction=0;
        if ((button&0x0f)==0x0f) {
            // button debounced rise

            // short press (less than 1 second)
            if(button_state_time_ms < 1000)
                buttonaction=1;

            // long press (between 1 and 6 seconds)
            else if(button_state_time_ms < 6000L)
                buttonaction=2;

            // button held more than six seconds
            //    here for completeness, but this case probably can't happen becuase
            //    we detect the hold time and act on it prior to the user releasing
            //  I guess it could happen if the user releases the button at the exact
            //     millisecond the counter reaches 6000
            else
                buttonaction=3;

            button_state=BUTTON_UP;
        }
        // button held more than six seconds
        else if(button_state_time_ms > 6000L) {
            buttonaction=3;
            button_state=BUTTON_LINGER;
        }

        if(buttonaction==1) {
            // short press, power on or toggle motor power if already on
            switch(curstate) {
            case POWER_OFF:
                if(voltage_status & 1)
                    stateupdate(POWER_ON);
                break;
            case POWER_SYSONLY:
                stateupdate(POWER_ON);
                break;
            case POWER_ON:
                stateupdate(POWER_SYSONLY);
                break;
            }
        }
        else if(buttonaction==2) {
            // long press, schedule a shutdown
            switch(curstate) {
            case POWER_OFF:
                break;
            case POWER_ON:
                stateupdate(POWER_SYSONLY);
                shutdown_pending = 60;
                break;
            case POWER_SYSONLY:
                shutdown_pending = 60;
                break;
            }
        }
        else if(buttonaction==3) {
            // button has been held for 6 seconds, immediate shutdown
            stateupdate(POWER_OFF);
        }
    }
    else if(button_state==BUTTON_LINGER) {
        // we already acted on the long press, now we just wait for the button to reset
        if ((button&0x0f)==0x0f)
            button_state=BUTTON_UP;
    }

    // keep countdown at max while button is active
    if(button_state != BUTTON_UP)
        sleep_countdown=SLEEP_RESET;
}

// update the state variable and actually set the power switches
void stateupdate(enum powerstate newstate) {
    if(curstate == newstate)
        return;

    switch(newstate) {
    case POWER_OFF:
        PORTA.OUTCLR = (1<<2) | (1<<1);//motor and sys
        break;
    case POWER_SYSONLY:
        PORTA.OUTCLR = 1<<2; //motor
        PORTA.OUTSET = 1<<1; //sys
        break;
    case POWER_ON:
        PORTA.OUTSET = 1<<1; //sys
        PORTA.OUTSET = 1<<2; //motor
        break;
    }
    curstate=newstate;
}

#if 0
// these can be useful in debugging, but we don't use serial in production
static inline void serial_write8(uint8_t v) {
    while(!(USART0.STATUS & SPI_DREIF_bm))
            ;
    USART0.TXDATAL = v;
}
static inline void serial_write16(uint16_t v) {
    while(!(USART0.STATUS & SPI_DREIF_bm))
            ;
    USART0.TXDATAL = v>>8;
    while(!(USART0.STATUS & SPI_DREIF_bm))
            ;
    USART0.TXDATAL = v;
}
#endif

int main() {
    // default clock rate is 3.3MHz, which is fine (20MHz/6)
    //_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0); // set to 20Mhz

    // Pin connections on the board

    // PORTA1 = PSYS_CTRL [out]
    // PORTA2 = PMTR_CTRL [out]
    // PORTA7 = PWR_BUTTON [in]
    // PORTA4 = VBAT_DIV [ain4]

    // PORTB0/B1 = SCL/SDA
    // PORTB2/B3 = TX/RX


    // enable output for P*_CTRL
    PORTA.DIRSET = (1<<1)|(1<<2);

    // enable falling edge interrupt for PORTA7 (for wakeup)
    PORTA.PIN7CTRL = PORT_ISC_BOTHEDGES_gc;

    // *** ADC setup ***
    // disable input buffers for ADC pin
    PORTA.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
    // set up 2.5V reference
    VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc;

    // 2pF sample cap, internal voltage reference, sysclk/8
    ADC0.CTRLC = ADC_SAMPCAP_bm |ADC_REFSEL_INTREF_gc| ADC_PRESC_DIV8_gc;
    // select pin A4
    ADC0.MUXPOS=ADC_MUXPOS_AIN4_gc;
    // enable ADC interrupt
    ADC0.INTCTRL = ADC_RESRDY_bm;
    // enable
    ADC0.CTRLA |= ADC_ENABLE_bm;
    // do a conversion
    ADC0.COMMAND = ADC_STCONV_bm;
    // *** end ADC Setup


    // set up I2C
    TWI0.SADDR = I2C_ADDR << 1;
    TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm | TWI_ENABLE_bm;



    // set up Perioidic Interrupt Timer to fire an interrupt at 1KHz frequency

    // use internal 32.768 KHz oscillator
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
    // enable PIT interrupt
    RTC.PITINTCTRL = RTC_PI_bm;

    // make sure PITCTRLA is available for writing!
    while(RTC.PITSTATUS & RTC_CTRLBUSY_bm)
        ;
    // set up for ~1khz interrupt (32.768khz/32)
    RTC.PITCTRLA = RTC_PERIOD_CYC32_gc;

    // make sure PITCTRLA is available for writing!
    while(RTC.PITSTATUS & RTC_CTRLBUSY_bm)
        ;
    // enable PIT
    RTC.PITCTRLA |= RTC_PITEN_bm;

    // set up RTC
    // enable overflow interrupt
    RTC.INTCTRL = RTC_OVF_bm;

    // makse sure CTRLA is available for writing!
    while(RTC.STATUS & RTC_CTRLABUSY_bm)
        ;
    // enable RTC
    RTC.CTRLA |= RTC_RTCEN_bm;

#if 0
    // can be useful for debugging, but not needed in production
    USART0.CTRLB = USART_TXEN_bm;
    USART0.BAUD = F_CPU * 64L / (16L*9600);
#endif

    // Set pin 2 as an output and blink it
    PORTB.DIRSET = (1<<2);

    // enable this line to start in POWER_SYSONLY (instead of OFF)
    //stateupdate(POWER_SYSONLY);

    // enable interrupts
    sei();

    while(1) {
        // if the power is off and the button hasn't been touched in a while, go to sleep
        //     wake sources are the button and I2C address match
        if(curstate == POWER_OFF && sleep_countdown==0) {
            // go to sleep
            SLPCTRL.CTRLA = SLPCTRL_SMODE_PDOWN_gc | SLEEP_ENABLED_gc;
            sleep_cpu();
        }
    }
}

