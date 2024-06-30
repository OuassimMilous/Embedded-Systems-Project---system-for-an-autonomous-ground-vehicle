#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> 
#include <stdbool.h>

//defining constants
#define TIMER1 1
#define TIMER2 2
#define CS LATDbits.LATD6
#define BUFFER_SIZE 37
#define BUFFER_SIZE_CMD 10

//defining registers
#define MAGCHIPID 0x40
#define MAGSLEEPMODE 0x4B
#define MAGOPMODE 0x4C

#define MAGXLSB 0x42
#define MAGXMSB 0x43

#define MAGYLSB 0x44
#define MAGYMSB 0x45

#define MAGZLSB 0x46
#define MAGZMSB 0x47

#define MAGHALLLSB 0x48
#define MAGHALLMSB 0x49

#define RIGHTB OC1R 
#define RIGHTF OC2R 
#define LEFTB OC3R 
#define LEFTF OC4R 


#define FORWARD 1 
#define BACKWARDS 2 
#define RIGHT 3 
#define LEFT 4 

#define PWMFREQUENCY 7200

#define STATE_DOLLAR  (1) // we discard everything until a dollar is found
#define STATE_TYPE    (2) // we are reading the type of msg until a comma is found
#define STATE_PAYLOAD (3) // we read the payload until an asterix is found

#define NEW_MESSAGE (1) // new message received and parsed completely
#define NO_MESSAGE (0) // no new messages

#define MAX_CMD 10
#define IRENABLE LATBbits.LATB9

#define LIGHTLEFT LATBbits.LATB8
#define LIGHTRIGHT LATFbits.LATF1
#define LIGHTBREAKS LATFbits.LATF0
#define LIGHTLOW LATGbits.LATG1
#define LIGHTBEAM LATAbits.LATA7
#define LED LATAbits.LATA0

#define STATE_WAIT 0 // the car is waiting for commands
#define STATE_EXECUTE 1 // the car is parsing the commands and ordering the robot to start executing
#define STATE_EXECUTING 2 // the car is moving

typedef struct {
    int state;
    char msg_type[6]; // type is 5 chars + string terminator
    char msg_payload[100]; // assume payload cannot be longer than 100 chars
    int index_type;
    int index_payload;
} parser_state;


//defining structs

typedef struct {
    char buffer[BUFFER_SIZE];
    int head;
    int tail;
    int count;
} CircularBuffer;


//defining structs

typedef struct {
    parser_state buffer[MAX_CMD];
    int head;
    int tail;
    int count;
} CMDCircularBuffer;


// Function prototypes
void tmr_setup_period(int8_t timer, int ms);
int tmr_wait_period(int8_t timer);
void tmr_wait_ms(int8_t timer, int ms);
void init_UART1(bool en_tx, bool en_rx);
void print_UART1(unsigned char msg);
void print_buffer_UART1(char buffer[]);
void init_SPI1();
void write_SPI1(unsigned char addr, unsigned char msg);
unsigned char read_SPI1(unsigned char addr);
void setup_mag();
int16_t mag_get_x();
int16_t mag_get_y();
int16_t mag_get_z();
void initCircularBuffer(CircularBuffer *cb);
int isEmpty(CircularBuffer *cb);
int isFull(CircularBuffer *cb);
void enqueue(CircularBuffer *cb, char value);
int dequeue(CircularBuffer *cb, char *value);
void enqueue_buffer(CircularBuffer *cb, char* m);



void CMDinitCircularBuffer(CMDCircularBuffer *cb);
int CMDisEmpty(CMDCircularBuffer *cb);
int CMDisFull(CMDCircularBuffer *cb);
void CMDenqueue(CMDCircularBuffer *cb, parser_state value);
int CMDdequeue(CMDCircularBuffer *cb, parser_state *value);

void init_adc();

void set_up_PWM_wheels();
void move(int dir, int speed);
void stop_moving();

void get_distance_and_battery(double* distance, double* battery);

void turnoff_lights();

void init_LED();
/*
Requires a pointer to a parser state, and the byte to process.
returns NEW_MESSAGE if a message has been successfully parsed.
The result can be found in msg_type and msg_payload.
Parsing another byte will override the contents of those arrays.
 */
int parse_byte(parser_state* ps, char byte);

/*
Takes a string as input, and converts it to an integer. Stops parsing when reaching
the end of string or a ","
the result is undefined if a wrong string is passed
 */
int extract_integer(const char* str);

/*
The function takes a string, and an index within the string, and returns the index where the next data can be found
Example: with the string "10,20,30", and i=0 it will return 3. With the same string and i=3, it will return 6.
 */
int next_value(const char* msg, int i);


#endif	/* XC_HEADER_TEMPLATE_H */