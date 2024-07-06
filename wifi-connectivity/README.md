Note the following assumes you are using Windows 11 and may or may not work depending on hardware/drivers.

# Setting Up Hotspot
Note only turn on your hotspot if someone does not already have one. Otherwise, join theirs if or have them turn it off.
* Goto Settings->Network & Internet->Mobile hotspot.
* Under properties click Edit.
* Set the network name to "Kirby ESP32 Testing Server"
* Set the network password to "KirbySucc"
* Set the network band to 2.4GHz
* Turn on the mobile hotspot

When devices connect to the hotspot you'll see the IP address which will be needed to upload code or communicate with TCP.

# Firmware Over-the-Air
## The code
Add the following includes and constants:
```
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "Kirby ESP32 Testing Server";
const char* password = "KirbySucc";
```

In the setup add the following code to make the ESP .......
