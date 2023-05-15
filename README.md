# MCP9808
 MCP9808 sensor connected with Raspberry Pi Pico W programmed in micropython and sending data to Thingspeak
Pod linkiem znajduje się projekt czujnika MCP9808 połączonego z Raspberry Pi Pico W za pomocą języka Micropython. W projekcie użyłem biblioteki do czujnika MCP9808 zapożyczonej od jednego z użytkowników Github, poniżej znajduje się link do użytej biblioteki:

https://github.com/kfricke/micropython-mcp9808
W projekcie dane zebrane z czujnika są wysyłane na serwer chmurowy Thingspeak, wzorując się na moim projekcie konieczne jest dodanie pliku secrets.py, w którym znajdują się SSID oraz hasło do użytej sieci Wi-Fi. Projekt potrzebuje także indywidualnego kodu API wykreowanego przez Thingspeak.
