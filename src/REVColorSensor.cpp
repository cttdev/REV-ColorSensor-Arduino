#include "REVColorSensor.h"
#include <Wire.h>

REVColorSensor::REVColorSensor() {}

void REVColorSensor::begin() {
    if (!checkForDevice()) {
        return;
    }

    if (!checkDeviceID()) {
        return;
    }

    initializeDevice();
    Serial.println("REV Color Sensor Initialized!");

    hasReset();
}

void REVColorSensor::configureProximitySensorLED(LEDPulseFrequency freq, LEDCurrent curr, unsigned int pluses) {
    write8((byte) Register::PROXIMITY_SENSOR_LED, (byte) freq | (byte) curr);
    write8((byte) Register::PROXIMITY_SENSOR_PULSES, (byte) pluses);
}

void REVColorSensor::configureProximitySensor(ProximitySensorResolution res, ProximitySensorMeasurementRate rate) {
    write8((byte) Register::PROXIMITY_SENSOR_RATE, (byte) res | (byte) rate);
}

void REVColorSensor::configureColorSensor(ColorSensorResolution res, ColorSensorMeasurementRate rate, GainFactor gain) {
    write8((byte) Register::LIGHT_SENSOR_MEASURMENT_RATE, (byte) res | (byte) rate);
    write8((byte) Register::LIGHT_SENSOR_GAIN, (byte) gain);
}

REVColorSensor::Color REVColorSensor::getColor() {
    uint32_t red = getRed();
    uint32_t green = getGreen();
    uint32_t blue = getBlue();

    uint32_t mag = red + green + blue;

    return Color(red / mag, green / mag, blue / mag);
}

uint32_t REVColorSensor::getProximity() {
    return read11BitRegister((byte) Register::PROXIMITY_DATA);
}

REVColorSensor::RawColor REVColorSensor::getRawColor() {
    return RawColor(
        getRed(),
        getGreen(),
        getBlue(),
        getIR()
    );
}

uint32_t REVColorSensor::getRed() {
    return read20BitRegister((byte) Register::DATA_RED);
}

uint32_t REVColorSensor::getGreen() {
    return read20BitRegister((byte) Register::DATA_GREEN);
}

uint32_t REVColorSensor::getBlue() {
    return read20BitRegister((byte) Register::DATA_BLUE);
}

uint32_t REVColorSensor::getIR() {
    return read20BitRegister((byte) Register::DATA_INFRARED);
}

REVColorSensor::CIEColor REVColorSensor::getCIEColor() {
    RawColor raw = getRawColor();
    return CIEColor(
        CMatrix[0] * raw.red + CMatrix[1] * raw.green + CMatrix[2] * raw.blue,
        CMatrix[3] * raw.red + CMatrix[4] * raw.green + CMatrix[5] * raw.blue,
        CMatrix[6] * raw.red + CMatrix[7] * raw.green + CMatrix[8] * raw.blue
    );
}

bool REVColorSensor::hasReset() {
    return (readSingleRegister((byte) Register::MAIN_STATUS) & 0x20) != 0;
}

bool REVColorSensor::checkForDevice() {
    Wire.beginTransmission(SENSOR_ADDRESS);
    uint32_t error = Wire.endTransmission();

    if (error == 0) {
        Serial.println("Found REV Color Sensor!");
        return true;
    } else {
        Serial.println("Failed to find REV Color Sensor!");
        return false;
    }
}

bool REVColorSensor::checkDeviceID() {
    if (readSingleRegister((byte) Register::PART_ID_REG) != PART_ID) {
        Serial.println("Unknown device found with same I2C addres as REV color sensor!");
        return false;
    } else {
        return true;
    }
}

void REVColorSensor::initializeDevice() {
    write8(
        (byte) Register::MAIN_CTRL,
        (byte) MainControl::RGB_MODE | (byte) MainControl::LIGHT_SENSOR_ENABLE | (byte) MainControl::PROXIMITY_SENSOR_ENABLE
    );

    write8(
        (byte) Register::PROXIMITY_SENSOR_RATE,
        (byte) ProximitySensorResolution::RESOLUTION_11BIT | (byte) ProximitySensorMeasurementRate::RATE_100MS
    );

    write8(
        (byte) Register::PROXIMITY_SENSOR_PULSES,
        (byte) 32
    );
}

byte REVColorSensor::readSingleRegister(byte reg) {
    Wire.beginTransmission(SENSOR_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(SENSOR_ADDRESS, 1);
    return Wire.read();
}


uint32_t REVColorSensor::read11BitRegister(byte reg) {
    Wire.beginTransmission(SENSOR_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(SENSOR_ADDRESS, 3);

    if (3 <= Wire.available()) { // if two bytes were received
        byte raw[3];
        Wire.readBytes(raw, 3);

    return (((uint32_t)raw[0] & 0xFF) | (((uint32_t)raw[1] & 0xFF) << 8) |
                    (((uint32_t)raw[2] & 0xFF) << 16)) & 0x03FFFF;
    } else {
    return 0;
    }
}

uint32_t REVColorSensor::read20BitRegister(byte reg) {
    Wire.beginTransmission(SENSOR_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(SENSOR_ADDRESS, 3);

    if (3 <= Wire.available()) { // if two bytes were received
        byte raw[3];
        Wire.readBytes(raw, 3);

    return (((uint32_t)raw[0] & 0xFF) | (((uint32_t)raw[1] & 0xFF) << 8) |
                    (((uint32_t)raw[2] & 0xFF) << 16)) & 0x03FFFF;
    } else {
    return 0;
    }
}

void REVColorSensor::write8(byte reg, uint32_t value) {
    Wire.beginTransmission(SENSOR_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    if (Wire.endTransmission() == 0) Serial.println("Write Successful!");
    else Serial.println("Write Failed!");
}
