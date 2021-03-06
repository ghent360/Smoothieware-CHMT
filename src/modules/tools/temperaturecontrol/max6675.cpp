/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include <math.h>
#include "libs/Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

#include "max6675.h"

#include "MRI_Hooks.h"
#include "board_pins.h"
#define chip_select_checksum CHECKSUM("chip_select_pin")
#define spi_channel_checksum CHECKSUM("spi_channel")

Max6675::Max6675() :
    spi(nullptr)
{
    this->read_flag=true;
}

Max6675::~Max6675()
{
    delete spi;
}

// Get configuration from the config file
void Max6675::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
    // Chip select
    this->spi_cs_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, chip_select_checksum)->by_default("0.16")->as_string());
    this->spi_cs_pin.set(true);
    this->spi_cs_pin.as_output();
    
    // select which SPI channel to use
    int spi_channel = THEKERNEL->config->value(module_checksum, name_checksum, spi_channel_checksum)->by_default(0)->as_number();
    PinName miso;
    PinName mosi;
    PinName sclk;
    if(spi_channel == 0) {
        // Channel 0
        mosi=SPI1_MOSI; miso=SPI1_MISO; sclk=SPI1_SCK;
    } else {
        // Channel 1
        mosi=SPI2_MOSI; miso=SPI2_MISO; sclk=SPI2_SCK;
    }

    delete spi;
    spi = new mbed::SPI(mosi, miso, sclk);

    // Spi settings: 1MHz (default), 16 bits, mode 0 (default)
    spi->format(16);
}

float Max6675::get_temperature()
{
    // allow read from hardware via SPI on next call to on_idle()
    this->read_flag=true;

    // Return an average of the last readings
    if(readings.size()==0) return infinityf();

    float sum = 0;
    for (int i=0; i<readings.size(); i++) {
        sum += *readings.get_ref(i);
    }

    return sum / readings.size();
}

// ask the temperature sensor hardware for a value, store it in a buffer
void Max6675::on_idle()
{
    // this rate limits SPI access
    if(!this->read_flag) return;

    this->spi_cs_pin.set(false);
    wait_us(1); // Must wait for first bit valid

    // Read 16 bits (writing something as well is required by the api)
    uint16_t data = spi->write(0);

    this->spi_cs_pin.set(true);

    float temperature;

    //Process temp
    if (data & 0x4) {
        // Error flag.
        temperature = infinityf();
    } else {
        data >>= 3;
        temperature = data * 0.25f;
    }

    if (readings.size() >= readings.capacity()) {
        readings.delete_tail();
    }

    // Discard occasional errors...
    if(!isinf(temperature))
    {
        readings.push_back(temperature);
    }

    // don't read again until get_temperature() is called
    this->read_flag=false;
}
