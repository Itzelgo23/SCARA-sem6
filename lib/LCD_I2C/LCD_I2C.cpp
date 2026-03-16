#include LCD_I2C

#define LCD_MASK_RS 0x01
#define LCD_MASK_RW 0x02
#define LCD_MASK_EN 0x04
#define LCD_MASK_BACKLIGHT 0x08


LCD_I2C::LCD_I2C()
{
    i2c.setup(0x27, 400000, 21, 22);
}

LCD_I2C::~LCD_I2C()
{
    
}

void LCD_I2C:: //poner algo