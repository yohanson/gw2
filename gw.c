#include "defines.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <string.h>
#include "clunet.h"
#include "hex2int.h"
#include "int2hex.h"

#define UART_BAUD_RATE 9600
#define MYUBRR         (F_CPU / 16 / UART_BAUD_RATE - 1)
#define UART_BUFF_SIZE 254
#define DATA_BUFF_SIZE 70
#define UART_DATA_AVAILABLE (UCSRA & (1<<RXC))
#define LED_PORTNAME B
#define LED_PIN  1
#define LED_PORT CLUNET_OUTPORT(LED_PORTNAME)
#define LED_DDR  CLUNET_DDRPORT(LED_PORTNAME)
#define ENDL "\n"

const char *error_prefix = "E:";
const char *error_too_long_string = "too long string" ENDL;
char check_crc(const char* data, const uint8_t size);

volatile char uart_buff[UART_BUFF_SIZE];
char packet[DATA_BUFF_SIZE];
volatile unsigned char uart_buff_content_size = 0;
volatile char uart_data_ready = 0;
char overflow = 0;


void led_on()
{
    LED_PORT |= (1 << LED_PIN);
}
void led_off()
{
    LED_PORT &= ~(1 << LED_PIN);
}

void send_data(const char *s, unsigned char ln)
{
    int i;
    for (i=0; i < ln; i++)    {
        while ( !( UCSRA & (1<<UDRE)) ) {}
        UDR=s[i];
    }
}

void send_hex(unsigned char byte)
{
    char hex[2];
    int2hexbyte(byte, hex);
    send_data(hex, 2);
}

void send_string(const char *s)
{
   send_data(s, strlen(s));
}

void send_crlf()
{
    send_string(ENDL);
}

void send_clunet_message_to_uart_incoming(unsigned char src_address, unsigned char dst_address, unsigned char command, char* data, unsigned char size)
{
    send_string("i:");
    send_hex(src_address);
    send_hex(dst_address);
    send_hex(command);
    send_hex(size);
    unsigned char i;
    for (i=0; i<size; i++)
    {
        send_hex(data[i]);
    }
    send_crlf();
}

void send_discovery_response_to_uart()
{
    send_clunet_message_to_uart_incoming(CLUNET_DEVICE_ID, CLUNET_BROADCAST_ADDRESS, CLUNET_COMMAND_DISCOVERY_RESPONSE, CLUNET_DEVICE_NAME, strlen(CLUNET_DEVICE_NAME));
}

void data_received(unsigned char src_address, unsigned char dst_address, unsigned char command, char* data, unsigned char size)
{
    led_on();
    send_string("i:");
    send_hex(src_address);
    send_hex(dst_address);
    send_hex(command);
    send_hex(size);
    unsigned char i;
    for (i=0; i<size; i++)
    {
        send_hex(data[i]);
    }
    send_crlf();
    led_off();
}
void initUART(unsigned int ubrr)
{
    UBRRH = (unsigned char)(ubrr >> 8);
    UBRRL = (unsigned char)ubrr;
    UCSRB |= _BV(RXEN) | _BV(TXEN) | _BV(RXCIE);
    UCSRC |= _BV(UMSEL);

}

void process_uart_command(volatile const char *cmd)
{
    // cmd format: 0401080103cf, 04=prio, 01=dst, 08=cmd, 01=data_size, 03=data, cf=crc. data is optional
    if (strlen(cmd) >= 6 && (strlen(cmd) & 1) == 0) {
        send_string("addr:");
        send_hex(((int)cmd) & 0x00ff);
        send_hex((((int)cmd) & 0xff00) >> 8);
        send_crlf();
        int packet_size = strlen(cmd) / 2;
        if (packet_size > DATA_BUFF_SIZE)
        {
            char dmsg[4];
            send_string(error_prefix);
            int2hexbyte(packet_size, dmsg);
            dmsg[2] = '\0';
            send_string(dmsg);
            send_string(" bytes, ");
            send_string(error_too_long_string);
            return;
        }
        char *p = cmd;
        char *packet_p = packet;
        while (*p != '\0')
        {
            *(packet_p++) = hexbyte2int(p);
            p += 2;
        }
        uint8_t crc = check_crc(packet, packet_p - packet);
        if (crc) {
            send_string("bad crc:");
            send_hex(crc);
            send_crlf();
            send_string("bytes:");
            send_string(cmd);
            send_crlf();
            return;
        }
        send_string("o:");
        uint8_t data_size = packet[CLUNET_OFFSET_SIZE];
        char *data = packet + CLUNET_OFFSET_DATA;
        for (packet_p = packet; packet_p < packet + CLUNET_OFFSET_DATA + data_size; packet_p++) {
            send_hex(*packet_p);
        }
        send_crlf();
        unsigned char need_to_process = (packet[CLUNET_OFFSET_DST_ADDRESS] == CLUNET_DEVICE_ID || (uint8_t)packet[CLUNET_OFFSET_DST_ADDRESS] == CLUNET_BROADCAST_ADDRESS);
        unsigned char need_to_relay = (packet[CLUNET_OFFSET_DST_ADDRESS] != CLUNET_DEVICE_ID);
        if (need_to_process) {
            switch ((unsigned char)packet[CLUNET_OFFSET_COMMAND]) {
                case CLUNET_COMMAND_PING:
                    send_clunet_message_to_uart_incoming(CLUNET_DEVICE_ID, CLUNET_BROADCAST_ADDRESS, CLUNET_COMMAND_PING_REPLY, data, data_size);
                    break;

                case CLUNET_COMMAND_DISCOVERY:
                    send_discovery_response_to_uart();
                    break;

                case CLUNET_COMMAND_REBOOT:
                    cli();
                    set_bit(WDTCR, WDE);
                    while(1);
                    break;
            }
        }
        if (need_to_relay) {
            while (clunet_ready_to_send()) {}; // wait
            clunet_send(
                (uint8_t)packet[CLUNET_OFFSET_DST_ADDRESS],
                (uint8_t)packet[CLUNET_OFFSET_SRC_ADDRESS], // prio here actually
                (uint8_t)packet[CLUNET_OFFSET_COMMAND],
                data,
                data_size
            );
        }
    }
}

ISR(USART_RXC_vect)
{
    char rc;
    while (UART_DATA_AVAILABLE)
    {
        rc = UDR;
        if (rc == '\r' || rc == '\n') // enter
        {
            if (!overflow)
            {
                if (uart_buff_content_size)
                {
                    uart_buff[uart_buff_content_size] = '\0';
                    //process_uart_command(uart_buff);
                    uart_data_ready = 1;
                    uart_buff_content_size = 0;
                    return;
                }
            }
            else
            {
                send_string(error_prefix);
                send_string("UART buff is 0x");
                send_hex(UART_BUFF_SIZE);
                send_string(" bytes, ");
                send_string(error_too_long_string);
                overflow = 0;
            }
            uart_buff_content_size = 0;
        }
        else // if (rc >= 0x20 && rc <= 0x7E)
        {
            if (uart_buff_content_size < UART_BUFF_SIZE-2) {
                uart_buff[uart_buff_content_size++] = rc;
            } else {
                overflow = 1;
            }
        } /*else {
            uart_buff_content_size = 0;
            }*/
    }
}

int main (void)
{
    LED_DDR = (1 << LED_PIN);
    led_on();
    initUART(MYUBRR);
    send_discovery_response_to_uart();
    clunet_init();
    clunet_set_on_data_received(data_received);
    sei();

    char buffer[1];
    buffer[0] = 1;
    clunet_send(CLUNET_BROADCAST_ADDRESS, CLUNET_PRIORITY_MESSAGE, CLUNET_COMMAND_DEVICE_POWER_INFO, buffer, sizeof(buffer));
    while (clunet_ready_to_send()) {}; // wait
    while (1)
    {
        if (uart_data_ready) {
            process_uart_command(uart_buff);
            uart_data_ready = 0;
        }
    }
    return 0;
}

