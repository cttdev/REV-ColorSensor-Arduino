/*
 * REVColorSensor - An Arduino Library to communicate with the REV Robotics Color Sensor.
 * Copyright (c) 2022 Claudius Tewari.  All right reserved.
 * 
 * Based on https://github.com/REVrobotics/Color-Sensor-v3/blob/main/src/main/java/com/revrobotics/ColorSensorV3.java  
 * Copyright (c) 2019 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef REVCOLORSENSOR_H
#define REVCOLORSENSOR_H

#include <Arduino.h>

#define SENSOR_ADDRESS 0x52
#define PART_ID 0xC2

class REVColorSensor
{
  public:
    // Class constructor. Does nothing- all setup is deferred to .begin()
    REVColorSensor();

    enum class MainControl : byte {
      RGB_MODE = 0x04,
      LIGHT_SENSOR_ENABLE = 0x02,
      PROXIMITY_SENSOR_ENABLE = 0x01,
      OFF = 0x00
    };

    enum class GainFactor : byte {
      GAIN_1X = 0x00,
      GAIN_3X = 0x01,
      GAIN_6X = 0x02,
      GAIN_9X = 0x03,
      GAIN_18X = 0x04
    };

    enum class LEDCurrent : byte {
      PULSE_2MA = 0x00,
      PULSE_5MA = 0x01,
      PULSE_10MA = 0x02,
      PULSE_25MA = 0x03,
      PULSE_50MA = 0x04,
      PULSE_75MA = 0x05,
      PULSE_100MA = 0x06,
      PULSE_125MA = 0x07
    };

    enum class LEDPulseFrequency : byte {
      FREQ_60KHZ = 0x18,
      FREQ_70KHZ = 0x40,
      FREQ_80KHZ = 0x28,
      FREQ_90KHZ = 0x30,
      FREQ_100KHZ = 0x38
    };

    enum class ProximitySensorResolution : byte {
      RESOLUTION_8BIT = 0x00,
      RESOLUTION_9BIT = 0x08,
      RESOLUTION_10BIT = 0x10,
      RESOLUTION_11BIT = 0x18
    };

    enum class ProximitySensorMeasurementRate : byte {
      RATE_6MS = 0x01,
      RATE_12MS = 0x02,
      RATE_25MS = 0x03,
      RATE_50MS = 0x04,
      RATE_100MS = 0x05,
      RATE_200MS = 0x06,
      RATE_400MS = 0x07
    };

    enum class ColorSensorResolution : byte {
      RESOLUTION_20BIT = 0x00,
      RESOLUTION_19BIT = 0x10,
      RESOLUTION_18BIT = 0x20,
      RESOLUTION_17BIT = 0x30,
      RESOLUTION_16BIT = 0x40,
      RESOLUTION_13BIT = 0x50
    };

    enum class ColorSensorMeasurementRate : byte {
      RATE_25MS = 0,
      RATE_50MS = 1,
      RATE_100MS = 2,
      RATE_200MS = 3,
      RATE_500MS = 4,
      RATE_1000MS = 5,
      RATE_2000MS = 7
    };

    class Color {
      public:
        Color();
        Color(double red, double green, double blue);
        double red;
        double green;
        double blue;
    };

    class RawColor {
      public:
        RawColor();
        RawColor(uint32_t red, uint32_t green, uint32_t blue, uint32_t ir);
        uint32_t red;
        uint32_t green;
        uint32_t blue;
        uint32_t ir;
    };

    class CIEColor {
      public:
        CIEColor();
        CIEColor(double x, double y, double z);
        double x;
        double y;
        double z;
    };

    void begin();

    /**
     * Configure the the IR LED used by the proximity sensor. 
     * 
     * These settings are only needed for advanced users, the defaults 
     * will work fine for most teams. Consult the APDS-9151 for more 
     * information on these configuration settings and how they will affect
     * proximity sensor measurements.
     * 
     * @param freq      The pulse modulation frequency for the proximity 
     *                  sensor LED
     * @param curr      The pulse current for the proximity sensor LED
     * @param pulses    The number of pulses per measurement of the 
     *                  proximity sensor LED (0-255)
     */
    void configureProximitySensorLED(LEDPulseFrequency freq, LEDCurrent curr, unsigned int pluses);

    /**
     * Configure the proximity sensor.
     * 
     * These settings are only needed for advanced users, the defaults 
     * will work fine for most teams. Consult the APDS-9151 for more 
     * information on these configuration settings and how they will affect
     * proximity sensor measurements.
     * 
     * @param res   Bit resolution output by the proximity sensor ADC.
     * @param rate  Measurement rate of the proximity sensor
     */
    void configureProximitySensor(ProximitySensorResolution res, ProximitySensorMeasurementRate rate);

    /**
     * Configure the color sensor.
     * 
     * These settings are only needed for advanced users, the defaults 
     * will work fine for most teams. Consult the APDS-9151 for more 
     * information on these configuration settings and how they will affect
     * color sensor measurements.
     * 
     * @param res   Bit resolution output by the respective light sensor ADCs
     * @param rate  Measurement rate of the light sensor
     * @param gain  Gain factor applied to light sensor (color) outputs
     */
    void configureColorSensor(ColorSensorResolution res, ColorSensorMeasurementRate rate, GainFactor gain);

    /**
     * Get the most likely color. Works best when within 2 inches and 
     * perpendicular to surface of interest.
     * 
     * @return  Color enum of the most likely color, including unknown if
     *          the minimum threshold is not met
     */
    Color getColor();

    /**
     * Get the raw proximity value from the sensor ADC (11 bit). This value 
     * is largest when an object is close to the sensor and smallest when 
     * far away.
     * 
     * @return  Proximity measurement value, ranging from 0 to 2047
     */
    uint32_t getProximity();

    /**
     * Get the raw color values from their respective ADCs (20-bit).
     * 
     * @return  ColorValues struct containing red, green, blue and IR values
     */
    RawColor getRawColor();

    /**
     * Get the raw color value from the red ADC
     * 
     * @return  Red ADC value
     */
    uint32_t getRed();

    /**
     * Get the raw color value from the green ADC
     * 
     * @return  Green ADC value
     */
    uint32_t getGreen();

    /**
     * Get the raw color value from the blue ADC
     * 
     * @return  Blue ADC value
     */
    uint32_t getBlue();

    /**
     * Get the raw color value from the IR ADC
     * 
     * @return  IR ADC value
     */
    uint32_t getIR();

    /**
     * Get the color converted to CIE XYZ color space using factory
     * calibrated constants. 
     * 
     * https://en.wikipedia.org/wiki/CIE_1931_color_space
     * 
     * @return  CIEColor value from sensor
     */
    CIEColor getCIEColor();


    /**
     * Indicates if the device reset. Based on the power on status flag in the
     * status register. Per the datasheet:
     * 
     * Part went through a power-up event, either because the part was turned
     * on or because there was power supply voltage disturbance (default at 
     * first register read).
     * 
     * This flag is self clearing
     * 
     * @return  bool indicating if the device was reset
     */
    bool hasReset();

  private:
    enum class Register : byte {
      MAIN_CTRL = 0x00,
      PROXIMITY_SENSOR_LED = 0x01,
      PROXIMITY_SENSOR_PULSES = 0x02,
      PROXIMITY_SENSOR_RATE = 0x03,
      LIGHT_SENSOR_MEASURMENT_RATE = 0x04,
      LIGHT_SENSOR_GAIN = 0x05,
      PART_ID_REG = 0x06,
      MAIN_STATUS = 0x07,
      PROXIMITY_DATA = 0x08,
      DATA_INFRARED = 0x0A,
      DATA_GREEN = 0x0D,
      DATA_BLUE = 0x10,
      DATA_RED = 0x13
    };

    bool checkForDevice();

    bool checkDeviceID();

    void initializeDevice();

    byte readSingleRegister(byte reg);

    uint32_t read11BitRegister(byte reg);

    uint32_t read20BitRegister(byte reg);

    void write8(byte reg, uint32_t value);
    
    double CMatrix[9] = {
       0.048112847, 0.289453437, -0.084950826,
      -0.030754752, 0.339680186, -0.071569905,
      -0.093947499, 0.072838494,  0.34024948
    };
};

#endif
