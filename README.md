# KeymoteControl

This is the firmware for the keymote control.

I flashed it using the sparkfun "Pro Micro 3.3V 8Mhz" definition, but I had to overwrite the following #defines because they were interfering with my pin usage:

#define TX_RX_LED_INIT
#define TXLED0
#define TXLED1
#define RXLED0
#define RXLED1
