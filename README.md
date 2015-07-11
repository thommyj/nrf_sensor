# nrf_sensor
Simple nrf24le1 project that transmits status of a gpio pin.
Client also send info about battery voltage and how many packets
that has been lost. Server will display status on UART, SPI, and GPIO.

I use it for keeping track of a mousetrap. Client is attached to
the mousetrap, using a microswitch. Server is attached to a RPI,
which regulary checks status over SPI. Server also drives a LED for
quick view.


uses brennens nrf24le1 library, can found @ http://www.diyembedded.com/
