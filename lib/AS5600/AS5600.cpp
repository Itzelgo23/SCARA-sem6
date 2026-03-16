#include <AS5600.h>

AS5600::AS5600(SimpleI2C &i2c) : _i2c(i2c) {}

void AS5600::begin()
{
    _i2c.setup(ADDRESS);
}

uint8_t AS5600::readMagnet()
{
    magnet_status = read8(STATUS);
    return magnet_status;
}

uint8_t AS5600::MagnetDetection()
{

    magnet_status = readMagnet();
    if (magnet_status & 0x20)
        status = MD;

    else if (magnet_status & 0x10)
        status = MH;

    else if (magnet_status & 0x08)
        status = ML;

    else
        status = NO_Magnet;

    switch (status)
    {
    case MD:
        printf("Magnet detected: %d\n", magnet_status);
        return MD;
    case ML:
        printf("Magnet too weak - decrease distance or use stronger magnet: %d\n", magnet_status);
        return ML;
    case MH:
        printf("Magnet too strong - increase distance or use weaker magnet: %d\n", magnet_status);
        return MH;
    case NO_Magnet:
        printf("No magnet detected: %d\n", magnet_status);
        return NO_Magnet;

    default:
        return NO_Magnet;
    }
}

void AS5600::correctAngle() //-15 --> 345
{
    corrected_Angle = (degAngle - start_Angle);
    if (corrected_Angle < 0)
    {
        corrected_Angle = corrected_Angle + 360;
    }
}

void AS5600::quadrantAngle()
{
    /*
    //Quadrants
    4 | 1
    -----
    3 | 2
    */
    // quadrant detection can be changed from every 90 to every 45 degrees
    if (degAngle >= 0 && degAngle < 90)
        quadrant = 1;
    else if (degAngle >= 90 && degAngle < 180)
        quadrant = 2;
    else if (degAngle >= 180 && degAngle < 270)
        quadrant = 3;
    else
        quadrant = 4;

    if (quadrant != prev_Quadrant)
    {
        if (quadrant == 1 && prev_Quadrant == 4)
            number_of_turns++;
        else if (quadrant == 4 && prev_Quadrant == 1)
            number_of_turns--;

        prev_Quadrant = quadrant;
    }
}

uint16_t AS5600::readRawAngle()
{
    read16(RAW_ANGLE, rawAngle);
    degAngle = (rawAngle * 360.0) / resolution;

    correctAngle();
    quadrantAngle();

    return rawAngle;
}

float AS5600::getTotalAngle()
{
    totalAngle = corrected_Angle + (number_of_turns * 360);
    return totalAngle;
}

void AS5600::write8(uint8_t reg, uint8_t value)
{
    uint8_t cmd = COMMAND_BIT | reg;
    uint8_t data[] = {cmd, value};
    _i2c.write(data, 2);
}

void AS5600::read16(uint8_t reg, uint16_t &value)
{
    uint8_t cmd = COMMAND_BIT | reg;
    uint8_t buffer[2];
    _i2c.read(&cmd, 1, buffer, 2);
    value = buffer[1] << 8 | buffer[0];
}

uint8_t AS5600::read8(uint8_t reg)
{
    uint8_t cmd = COMMAND_BIT | reg;
    uint8_t buffer[1];
    _i2c.read(&cmd, 1, buffer, 1);
    return buffer[0];
}
