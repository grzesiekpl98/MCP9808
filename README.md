# MCP9808
 MCP9808 sensor connected with Raspberry Pi Pico W programmed in micropython and sending data to Thingspeak

Pod linkiem znajduje się projekt czujnika MCP9808 połączonego z Raspberry Pi Pico W za pomocą języka Micropython. W projekcie użyłem biblioteki do czujnika MCP9808 zapożyczonej od jednego z użytkowników Github, poniżej znajduje się link do użytej biblioteki:

https://github.com/kfricke/micropython-mcp9808

W projekcie dane zebrane z czujnika są wysyłane na serwer chmurowy Thingspeak, wzorując się na moim projekcie konieczne jest dodanie pliku secrets.py, w którym znajdują się SSID oraz hasło do użytej sieci Wi-Fi. Projekt potrzebuje także indywidualnego kodu API wykreowanego przez Thingspeak.

*********************
Under the link is the project of the MCP9808 sensor connected to the Raspberry Pi Pico W using the Micropython language. In the project I used a library for the MCP9808 sensor borrowed from one of the Github users, below is a link to the library used:

https://github.com/kfricke/micropython-mcp9808

In the project, the data collected from the sensor is sent to the Thingspeak cloud server, following my project, it is necessary to add the secrets.py file, which contains the SSID and password for the used Wi-Fi network. The project also needs an individual API code created by Thingspeak.
