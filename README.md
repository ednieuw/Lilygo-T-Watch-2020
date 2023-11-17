# Lilygo-WristWatch
# Still under construction
![AllWatchesK](https://github.com/ednieuw/Lilygo-WristWatch/assets/12166816/9c859ab7-900d-4b10-8df6-623f4591e446)

A wrist watch that displays the time in words in Dutch, English, French and 
German,as an analogue or digital clock.<br />
The clock receives time via NTP from the internet. <br />
Settings can be controlled via a webpage, PC and Bluetooth LE.<br />
Also info here: <a href="https://ednieuw.home.xs4all.nl/Woordklok/ESP32C3AndMore/ESP32C3ClockAndMoreV.html">  
On my home page for the latest info</a>.	
<p>The clock is built with a [LillyGo T-watch 2020 V3]( https://github.com/Xinyuan-LilyGO/TTGO_TWatch_Library) that runs on a 16 Mb ESP32-S3.</br>
The software is written with the Arduino IDE 1.8.19.</br>
The software contains coding to use:</br>
1 ST7789 colour display</br>
2 BLE nRF UART connection with a phone with an option to send strings longer than 20 bytes</br>
3 Time zone corrected time with daylight savings from a NTP server via WIFI</br>
4 Analog, word and digital clock.</br>
5 RTC for time keeping when off line</br>
6 Storage of the settings in the ESP32-S3 SPIFSS Flash memor</br>
7 Menu driven control of preferences with serial monitor, BLE and WIFI-html page</br>
8 Four languages to display time</br>
9 OTA. Upgrade the software over the air without USB-cable</br>
10 Swipe or touch the display to switch between options

This software is derived from the [ILI9341 + ESP32-C3 clock](https://github.com/ednieuw/ESP32-C3-Clock-and-more).
https://github.com/ednieuw/Lilygo-WristWatch/assets/12166816/82f1b46f-4dc8-43d3-bf7c-c0fba49e676e

A fast method to load the software in the watch OTA (Over The Air) can be used.  Instal the Arduino IDE as described in the point 1-4 below and install an OTA library in the IDE. </br>
AsyncElegantOTA works fine. </br>
Open the demo program in the example folder of AsyncElegantOTA and enter the WIFI details over the dots in const char* ssid = "........"; and const char* password = "........";. </br>
Compile and upload the program in the watch.</br>
Find the IP-address of the watch in your router and enter this in a browser followed with /update.</br>
in my case: 192.168.178.88/update  </br>
Select Firware and Choose File. Find and open the bin file: Liygo-WristWatchESP32-V011.ino.twatch.bin</br>
Now enter in the browser the IP-address without /update. in my case: 192.168.178.88</br>
The watch menu will open. </br>
Enter the character a followed with your WIFI networkname.  aSSID</br>
Enter the character b followed with your WIFI password.  aPassword</br>
Send a @ to rest the watch and voila the watch is running on time.</br>

The rest is reading the manual for this watch</br>


<h1 align = "center">🌟LilyGO T-Watch V2020 V3🌟</h1></br>
<h3>From [LillyGo T-watch 2020 V3]( https://github.com/Xinyuan-LilyGO/TTGO_TWatch_Library) the next instructions:</h3>

## 1️⃣ Arduino IDE Quick Start

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Install `Arduino ESP32 2.0.9 or higher`. `Tools` -> `Board` -> `Boards Manager` and search ESP32
    ** I used 2.0.14. **
3. Install [TTGO_TWatch_Library](https://github.com/Xinyuan-LilyGO/TTGO_TWatch_Library)
   1. Download a zipfile from the LilyGo TTGO github page using the "Download ZIP" button and install it using the IDE ("Sketch" -> "Include Library" -> "Add .ZIP Library...")
   2. Clone this git repository into your sketchbook/libraries folder. For more info, see https://www.arduino.cc/en/Guide/Libraries
</br>
Install the library NTP Client from https://github.com/gmag11/ESPNtpClient or </br>
</br>
Following are Arduino libraries and can be found in the Arduino IDE library manager but also in the ZIP-file downloaded for this watch that contains all libraries needed to compile for the watch.</br>
NimBLEDevice.h      
AsyncTCP.h          
ESPAsyncWebServer.h 
AsyncElegantOTA.h



5. In the Arduino IDE Go to Tools -> Board --> ESP32  and search in the list 'TTGO T-watch'
![image](https://github.com/ednieuw/Lilygo-WristWatch/assets/12166816/d0ffc660-1a1b-4799-8abd-a21b5881e2cd)


1. Unable to upload to watch
    1. Make sure that the T-Watch is turned on, you can check it according to the following method, open the computer device manager, check the port, plug the USB port into the computer, and if the new COM device is displayed, it has been turned on, if it is not displayed, press the crown Press the button on the button for one second, and then the device port will pop up, click upload at this time
2. The USB port keeps flashing in the computer
* This is a phenomenon caused by the abnormal operation of the program, or the selection of the wrong configuration, and the continuous restart of the esp32. At this time, the problem of not being able to upload can only be solved by manually entering the download mode of the watch
Please follow the steps below
   1. Remove the back cover
   2. Insert Micro-USB
   3. Open Windows Device Manager
   4. Press and hold the crown of the watch until the USB device does not appear in the Windows COM port
   5. Press the button in the picture below and keep pressing
![image](https://github.com/ednieuw/Lilygo-WristWatch/assets/12166816/dd244d32-bac4-4052-b45b-571f5e504322)

   6. Press the crown button for one second
   7. Now the COM port is fixed
   8. Select Port in Arduino IDE
   9. Click Upload
3. Where to query the pin definition?
    1. Look [here](./src/utilities.h)
4. The screen is not displayed after uploading the sketch?
    1. Please check the fourth line of [Arduino IDE Quick Start]()
5. Power Domain 
    | Power Domain | Role                          |
    | ------------ | ----------------------------- |
    | ALDO1        | RTC backup battery (3.1-3.3v) |
    | ALDO2        | Backlight                     |
    | ALDO3        | 3V3 for FT6336 and st7889     |
    | ALDO4        | SX1262                        |
    | BLDO2        | DRV2605 Enable pin            |
    | DC1          | ESP32 3V3                     |
    | VRTC         | Nothing                       |

6. Battery 
   1. Lilygo T-Watch fits 502530 size (5x25x30mm) batteries of any chemistry supported by the AXP2101.
7. **esp_vad.h: No such file or directory**
   * **Please use 2.0.9. The new version has changed too much and has not yet been adapted.**
