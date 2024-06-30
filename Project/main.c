/*
 * Authors:
 * Ouassim Milous
 * Younes Hebik
 * Mohamed Aimen Boulala
 */

#include "xc.h"
#include "tools.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include<stdbool.h>


// initializing the variables
parser_state pstate;
int data[2]; // data[0] is the direction and data[1] is the time

CMDCircularBuffer cbCMD; // the FIFO for the command
CircularBuffer cb; // the circular buffer for printing
CircularBuffer cbr; // the circular buffer for reciving

int state = STATE_WAIT;

int counter1ms = 0;
int counterMotor = 0; //to set the time for the motor to execute the command correctly

bool printingFlag=0; 

// an interrupt triggered when the UART is ready to print
void __attribute__((__interrupt__, __auto_psv__))_U1TXInterrupt() {
    //resetting the interrupt flag
    IFS0bits.U1TXIF = 0;
    //printing the queued data
    char printable;
    if (dequeue(&cb, &printable)) {
        //printing the next character on the circular buffer if any are available
        U1TXREG = printable;
    }
}

// an interrupt triggered when the UART is ready to print
void __attribute__((__interrupt__, __auto_psv__))_U1RXInterrupt() {
    //resetting the interrupt flag
    IFS0bits.U1RXIF = 0;
    
    //reading the characte
    char reced = U1RXREG;

    // enqueuing it in the recieved cgaracgters ciecular buffer
    enqueue(&cbr,reced);
}

// Using timer1 to avoid debouncing and changing the state when the button is clicked
void __attribute__((__interrupt__, __auto_psv__))_T1Interrupt() {

    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 0;
    T1CONbits.TON = 0;

    if (PORTEbits.RE8 == 1) {
        state = !state;
    }
}

// the interrupt for the button
void __attribute__((__interrupt__, __auto_psv__))_INT1Interrupt() {
    IFS1bits.INT1IF = 0;
    IEC0bits.T1IE = 1;
    tmr_setup_period(TIMER1, 50);
}

int main(void) {
    //initializing
    init_adc();
    initCircularBuffer(&cb);
    initCircularBuffer(&cbr);
    CMDinitCircularBuffer(&cbCMD);
    init_UART1(1, 1); //initiating UART to print results and enabling RX and TX interrupts

    // initializing parser state
    pstate.state = STATE_DOLLAR;
    pstate.index_type = 0;
    pstate.index_payload = 0;

    // setting up the button that changes the state with the interrupt 1
    TRISEbits.TRISE8 = 1;
    RPINR0bits.INT1R = 88;
    INTCON2bits.GIE = 1;
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;

    set_up_PWM_wheels();

    init_LED();

    double battery, distance;
    tmr_setup_period(TIMER2, 1);

    // infinite loop repeating at 1KHz
    while (1) {

        get_distance_and_battery(&distance, &battery);

        unsigned char m[BUFFER_SIZE];
        
        // parsing the recieved data
        char tmp;
        if (dequeue(&cbr, &tmp)) {
            //parsing the received data until the command is complete
            if (parse_byte(&pstate, tmp)) {
                if (CMDisFull(&cbCMD)) {
                    //sending the acknowledgment that the FIFO is full, thus, the command was ignored
                    enqueue_buffer(&cb,"$MACK,0*");

                } else {
                    if (strcmp(pstate.msg_type, "PCCMD") == 0) {
                        // enqueuing the command
                        CMDenqueue(&cbCMD, pstate);
                        //sending acknowledgment that the command was received and enqueued
                        enqueue_buffer(&cb,"$MACK,1*");
                    }
                }
            }
        }
        
        // printing the distance
        if (counter1ms % 100 == 0) {
            sprintf(m, "$MDIST,%.2f*", distance); // Format the output
            enqueue_buffer(&cb, m);
            printingFlag=1;        
        }
        
        // printing the battery
        if (counter1ms % 1000 == 0) {
            sprintf(m, "$MBATT,%.2f*", battery); // Format the output
            enqueue_buffer(&cb, m);
        }
        
        // forcing the first print
        if(!isEmpty(&cb) && printingFlag){
            IFS0bits.U1TXIF = 1;
            printingFlag=0;
        }

        if (counter1ms % 500 == 0) {
            LED = !LED;
        }
        
        //handling our different states
        switch (state) {
            case STATE_WAIT:
                stop_moving();
                // toggling o=the lights on a 1Hz rate
                if (counter1ms % 500 == 0) {
                    LIGHTLEFT = !LIGHTLEFT;
                    LIGHTRIGHT = !LIGHTRIGHT;
                }
                break;
            case STATE_EXECUTE:
                LIGHTLEFT = 0;
                LIGHTRIGHT = 0;
                // if there is a command, parse the data
                if (!CMDisEmpty(&cbCMD)) {
                    int nxt = 0;
                    parser_state temp;
                    CMDdequeue(&cbCMD, &temp);
                    for (int i = 0; nxt < temp.index_payload; i++) {
                        data[i] = extract_integer(&temp.msg_payload[nxt]);
                        nxt = next_value(&temp.msg_payload, nxt);
                    }
                    
                    // execute the command
                    state = STATE_EXECUTING;
                    // reset the executing time for the motor
                    counterMotor = 0;
                }
                break;
                
            case STATE_EXECUTING:

                // check for obstacles
                if (distance >= 20) {
                    move(data[0], PWMFREQUENCY * 0.80);
                } else {
                    stop_moving();
                }

                // checking if the set time has passed
                if (data[1] == 0 || counterMotor % (data[1]) == 0) {
                    state = STATE_EXECUTE;
                    stop_moving();
                    data[0]= 0;
                    data[1]= 0;
                }
                break;
        }

        // incrementing the counter to count different times as needed (for instance: 1000ms is 1 second) 
        counter1ms++;
        counterMotor++;
        
        // wait for the timer to keep a frequency of 1KHZ
        tmr_wait_period(TIMER2);
    }

    return 0;
}