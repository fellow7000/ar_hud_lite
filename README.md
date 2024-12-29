Head-Up (HUD) Project for Dynon SkyView Classic Avionic - get your data projected on the canopy!

This is a simple version of the implementation for Arduino Board with a CAN Shield.

Hardware used:
Arduino Uno R4 WiFi

CAN Shield keyestudio (works with R4 as well)
buy: https://www.keyestudio.com/products/keyestudio-can-bus-shield-mcp2515-chip-with-sd-socket-for-arduino-uno-r3
wiki: https://wiki.keyestudio.com/KS0411_keyestudio_CAN-BUS_Shield
libs: https://www.dropbox.com/scl/fo/jk2e59975wvn1j37d8r0o/AJuU9H-LcM_m-T1rr-U0Fu0?rlkey=lk33jgk3sfwo2fs07yam3wclz&e=1&st=1jizph0q&dl=0
you need to download and install CAN Shield libs before compile this code

Aftermarket standard OBD-II based Head-Up Display (Amazon, Aliexpress, Temu etc)

Algorithm:
Board connects to the Dynon SkyView Avionic via WiFi and starts listening UDP on port Nr. 49155
HUD conntects to Arduino via CAN D-SUB9 (see picture)
HUD emulates an OBD-II tester and sends in loop request for specific values (speed, water temperature, RPM etc)
Power supply in the aircraft if done via OBD-II connector (see picture)
Avionic streams data in text format (see Dynon Installation Manual for protocol details)
Arduino received text packages, parses and responses on HUD requests with necessary values
If HUD does not get proper / no data on requests, it stops working within several seconds

See code for details

You are free to reuse / modify existing code for your non-commercial projects with providing credits on this repo and its author

Fellow7000 2020-2024