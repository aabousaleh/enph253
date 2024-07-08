Note the following assumes you are using Windows 11 and may or may not work depending on hardware/drivers.

# Setting Up The Hotspot
Note only turn on your hotspot if someone does not already have one. Otherwise, join theirs if or have them turn it off.
* Goto Settings->Network & Internet->Mobile hotspot.
* Under properties click Edit.
* Set the network name to "Kirby ESP32 Testing Server"
* Set the network password to "KirbySucc"
* Set the network band to 2.4GHz
* Turn on the mobile hotspot

When devices connect to the hotspot you'll see the IP address which will be needed to upload code or communicate with TCP.

# Firmware Over-The-Air
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

In the `void setup()` add the following code to make the ESP start a station and setup OTA:
```
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, password);
while (WiFi.waitForConnectResult() != WL_CONNECTED) {
  delay(5000);
  ESP.restart();
}

ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

ArduinoOTA.begin();
```

In the main loop the following function needs to be called every so often to listen for OTA requests:
```
ArduinoOTA.handle();
```

## Uploading OTA
Initially, the code must be uploaded over regular USB. Once the above OTA code is included, every subsequent upload can be done over the air as long as the above code is there.
### Arduino IDE
Simply go to Tools->Port and under "network port" select the ESP32 desired.

### VS Code
At the very bottom of Visual Studio where the upload and compile buttons are, to the left there is a plug icon that says "auto" next to it. Click on this then select custom and type in the IP address of the desired ESP32.

Now when you upload it will upload OTA.

# Printing To Terminal With TCP
This is the wifi equivalent of `Serial.print();`

## The Code
Declare the following constants. Note the parameter in `server()` is the port and 80 is the standard port for a server.
```
WiFiServer server(80);
WiFiClient RemoteClient;
```

In `void setup()` add the following line of code to start the server:
```
server.begin();
```

In the `void loop()` add the following lines of code. The first if statement checks if we don't have a client and makes itself available for connections otherwise making itself unavailable. The second if statement checks if there is a connection and then prints to it.
```
if (server.hasClient()) {
  if (RemoteClient.connected()) {
    Serial.println("Connection rejected");
    server.available().stop();
  } else {
    Serial.println("Connection accepted");
    RemoteClient = server.available();
  }
}

if (RemoteClient.connected()) {
  RemoteClient.println("Hello World!");
}
```
## Setting Up The Terminal
For the first time only you need to enable a command line feature:
* Run command prompt as administrator
* Execute the following command `dism /online /Enable-Feature /FeatureName:TelnetClient`

For all subsequent connections simply go to the command prompt and execute the following command `telnet 192.168.~~~.~~~ 80` with the appropriate IP replaced.

Pro tip if the ESP32 disconnects; for example if you reset it. Simply type a few random characters in the command line, then it will automatically disconnect and you can press the up arrow and enter to reconnect.
