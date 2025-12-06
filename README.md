# chordbob

daisyseed chord player

uses Pimoroni RGB keypad:
* https://cdn.shopify.com/s/files/1/0174/1800/files/pico_rgb_keypad_base_schematic.pdf?v=1629994431
* https://github.com/pimoroni/pimoroni-pico/blob/main/libraries/pico_rgb_keypad/pico_rgb_keypad.cpp


## wiring

NOTE: seems to need external 5V power connected to daisy Vin and keypad VBUS
```
daisy 3v3 out (pin 38) -> keypad +3V3
daisy DGND (pin 40) -> keypad GND
daisy D7 (pin 8) -> keypad GP17 (pin 22)    // RGB SPI chip select 
daisy D8 (pin 9) -> keypad GP18 (pin 24)    // RGB SPI clk
daisy D10 (pin 11) -> keypad GP19 (pin 25)  // RGB SPI mosi
daisy D11 (pin 12) -> keypad GP5 (pin 7)    // keys I2C SCL
daisy D12 (pin 13) -> keypad GP4 (pin 6)    // keys I2C SDA
```

linux setup notes -- needed for vscode debugging on debian 13:
* https://forum.electro-smith.com/t/st-link-and-cortex-debugger-on-ubuntu-24-04/5260
