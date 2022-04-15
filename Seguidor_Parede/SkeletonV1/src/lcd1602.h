#ifndef __LCD1602_H__
#define __LCD1602_H__


#define LED_PIN 0

#define LCD_ADDR 0b00100111
#define SLA_W LCD_ADDR << 1
#define LCD_COMMAND 0
#define LCD_DATA 1


/*
 * LCD initialization
*/
void lcd1602_init(void);

/**
 * Sending byte to lcd
 * @c: byte
 * @rs: type of data
 */
void lcd1602_send_byte(char c, char rs);

/**
 * Sending char to lcd
 * @c: char
 */
void lcd1602_send_char(char c);

/*
 * Clear LCD
 */
void lcd1602_clear(void);

/**
 * Move cursor on lcd
 * @col: X
 * @row: Y
 */
void lcd1602_goto_xy(char col, char row);

/**
 * Writing string on the lcd
 * @str: string
 */
void lcd1602_send_string(const char *str);


#endif