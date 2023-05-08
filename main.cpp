#include <string.h>
#include <math.h>
#include <vector>
#include <cstdlib>

#include "libraries/pico_display_2/pico_display_2.hpp"
#include "drivers/st7789/st7789.hpp"
#include "libraries/pico_graphics/pico_graphics.hpp"
#include "rgbled.hpp"
#include "button.hpp"

#include "serial.h"
#include "sensor.h"
#include "pico/stdlib.h"

using namespace pimoroni;

ST7789 st7789(320, 240, ROTATE_0, false, get_spi_pins(BG_SPI_FRONT));
PicoGraphics_PenRGB332 graphics(st7789.width, st7789.height, nullptr);

RGBLED led(PicoDisplay2::LED_R, PicoDisplay2::LED_G, PicoDisplay2::LED_B);

Button button_a(PicoDisplay2::A);
Button button_b(PicoDisplay2::B);
Button button_x(PicoDisplay2::X);
Button button_y(PicoDisplay2::Y);

static char printBuf[4096];
serial_modes_t mode = mode_stream_data;

int main() {


  static repeating_timer_t timer;

  st7789.set_backlight(255);

  Point text_location(110, 115);

  Pen BG = graphics.create_pen(0, 0, 0);
  Pen WHITE = graphics.create_pen(0, 255, 0);

  printf("\nBegin main loop");
  serial_init();


  while(true) {
    if(button_a.raw()) text_location.x -= 1;
    if(button_b.raw()) text_location.x += 1;

    if(button_x.raw()) text_location.y -= 1;
    if(button_y.raw()) text_location.y += 1;
  
    graphics.set_pen(BG);
    graphics.clear();

    graphics.set_pen(WHITE);
    graphics.text("Hello World", text_location, 320);

    serial_sendSensorPacket();
    sensor_fillBufWithCurrentData(printBuf);
    // note this is static for now - the data doesn't change
    sensor_generateData();
    sleep_ms(10);


    // update screen
    st7789.update(&graphics);
  }

    return 0;
}
