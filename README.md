# Microchip MCP4725

This is a HAL based library for I²C MCP4725 12-bit Digital-to-Analog Converter with EEPROM. This library has been ported from [this Arduino library](https://github.com/enjoyneering/MCP4725) to STM32 using HAL drivers.

- operating/reference voltage 2.7v - 5.5v
- output voltage from 0 to operating voltage
- maximum output current 25mA
- output impedance 1 Ohm
- maximum output load 1000pF/0.001μF in parallel with 5 kOhm
- voltage settling time 6 μsec - 10 μsec 
- slew rate 0.55 V/μs
- add 100μF & 0.1 μF bypass capacitors within 4mm to Vdd
- device has 14-bit EEPROM with on-chip charge pump circuit for fail-safe writing
- estimated EEPROM endurance 1 million write cycles
- if Vdd < 2v all circuits & output disabled, when Vdd
  increases above Vpor device takes a reset state & upload data from EEPROM

Supports all MCP4725 features:

- Fast write
- Register read & write
- EEPROM read & write
- Power down
- General reset
- General wake-up

# Donate
Is it helpfull?

<p align="left">
  <a href="http://smotlaq.ir/LQgQF">
  <img src="https://raw.githubusercontent.com/SMotlaq/LoRa/master/bmc.png" width="200" alt="Buy me a Coffee"/>
  </a>
</p>

# How to use

This section will be completed asap

# Contact
[Telegram](http://t.me/s_motlaq) or E-mail: pilot.motlaq@gmail.com

- - - -
