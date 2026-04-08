#include "emg_server.h"

#include <AsyncTCP.h>

void init_wifi()
{
  if (strlen(WIFI_SSID) && strlen(WIFI_PASSWORD))
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t start_time = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
      Serial.print('.');
      delay(200);
      if (millis() - start_time > WIFI_CONNECT_TIMEOUT_MS)
      {
        Serial.println();
        Serial.println("WiFi: could not connect to access point in reasonable time.");
        break;
      }
    }
    Serial.print("ip: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("WiFi: no credentials configured");
  }
}

void EMGServer::init()
{
  if (WiFi.isConnected())
  {
    async_server = new AsyncServer(SERVER_PORT);
    async_server->onClient([](void *server, AsyncClient *client)
                           { ((EMGServer *)server)->handle_client(client); }, this);
    async_server->begin();
    Serial.printf("Server (%s) started on port %d\n", WiFi.localIP().toString().c_str(), SERVER_PORT);
  }
}

bool EMGServer::has_client()
{
  return async_client;
}

void EMGServer::write(int32_t sample)
{
  if (has_client())
  {
    async_client->write(reinterpret_cast<const char *>(&sample), 4);
  }
}

// It handles errors that are not normal disconnections.
void EMGServer::handle_client_error(AsyncClient *client, int error)
{
  // The error codes are defined in esp_err.h
  Serial.printf("Client error! Code: %d, Message: %s\n", error, client->errorToString(error));

  // If the client is the one we have stored, clean it up.
  if (async_client == client)
  {
    Serial.println("Cleaning up global client pointer due to error.");
    async_client = nullptr;
  }
  // We do not need to call "delete client" here because onDisconnect will do it.
  // If the error is critical, we will do it.
  if (client->connected())
  {
    client->close();
  }
}

// Called when the client disconnects.
void EMGServer::handle_client_disconnect(AsyncClient *client)
{
  Serial.println("Client disconnected.");
  // Set the global client pointer to null to allow a new client to connect.
  if (async_client == client)
  {
    async_client = nullptr;
  }
  delete client;
}

// Called when a new client tries to connect.
void EMGServer::handle_client(AsyncClient *client)
{
  // If there is already a client connected, reject the new one.
  if (async_client)
  {
    Serial.printf("New connection from %s rejected. Server is busy.\n", client->remoteIP().toString().c_str());
    client->close();
    return;
  }

  // Accept the new client.
  Serial.printf("New client connected from %s\n", client->remoteIP().toString().c_str());
  async_client = client;

  // Called when a communication error (e.g., protocol failure or timeout) occurs.
  // Essential for cleaning up the global client pointer and preventing resource leaks.
  async_client->onError([](void *server, AsyncClient *client, int error)
                        { ((EMGServer *)server)->handle_client_error(client, error); }, this);

  // Called when the client actively closes the connection or if a fatal error occurs.
  // Responsible for resetting the global client pointer and freeing memory.
  async_client->onDisconnect([](void *server, AsyncClient *client)
                             { ((EMGServer *)server)->handle_client_disconnect(client); }, this);
}