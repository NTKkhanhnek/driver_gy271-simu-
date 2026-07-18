#ifndef INC_UART2_H_
#define INC_UART2_H_

#include <stdint.h>

#define UART2_BAUDRATE 115200U

void uart2_init(void);
void uart2_send_char(char data);
void uart2_send_string(const char *str);
void uart2_send_uint(uint32_t value);
void uart2_send_int(int32_t value);
void uart2_send_float(float value, uint8_t decimal);

#endif /* INC_UART2_H_ */
