#ifndef EMGSERVER_H
#define EMGSERVER_H

#include <Arduino.h>

#include <WiFi.h>
#include <AsyncTCP.h>

#define WIFI_SSID ""
#define WIFI_PASSWORD ""

#define SERVER_PORT 8081
#define WIFI_CONNECT_TIMEOUT_MS 2000

void init_wifi();

class EMGServer
{
public:
  EMGServer() {}

  void init();

  bool has_client();

  void write(int32_t sample);

private:
  void handle_client_error(AsyncClient *client, int error);
  void handle_client_disconnect(AsyncClient *client);
  void handle_client(AsyncClient *client);

  AsyncServer *async_server = nullptr;
  AsyncClient *async_client = nullptr;
};

#endif