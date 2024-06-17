#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WiFiMulti.h> 
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include "Credentials.h"
#include <wpa2_enterprise.h>

ESP8266WiFiMulti wifiMulti;

ESP8266WebServer server(80);

const char* ssid = SSID;
const char* password = PASSWORD;
const char* domainName = "xplite";

void handleRoot();
void handleNotFound();

void setupServer() {
  WiFi.softAP(ssid, password);
  Serial.print("Access point: ");
  Serial.print(ssid);
  Serial.print(" started!"); 
  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());

  wifiMulti.addAP(ssid, password);

  // clear old data
  wifi_station_clear_cert_key();
  wifi_station_clear_enterprise_ca_cert();
  wifi_station_clear_enterprise_identity();
  wifi_station_clear_enterprise_username();
  wifi_station_clear_enterprise_password();
  wifi_station_clear_enterprise_new_password();
  
//  wifi_station_set_wpa2_enterprise_auth(1);
//  wifi_station_set_enterprise_username("");
//  wifi_station_set_enterprise_password("");
//  wifi_station_connect();
//  Serial.print(wifi_station_get_connect_status);
//  while (wifiMulti.run() != WL_CONNECTED) {
//    delay(1000);
//    Serial.print('.');
//  }

//  Serial.print("Connected to ");
//  Serial.println(WiFi.SSID());
//  Serial.print("IP address:\t");
//  Serial.println(WiFi.localIP());
  

  if (MDNS.begin(domainName)) {
    Serial.println("mDNS responder started");
  } else {
    Serial.println("Error setting up MDNS responder!");
  }
  MDNS.addService("http", "tcp", 80);

  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void runServer() {
  server.handleClient();
}

void handleRoot() {
  server.send(200, "text/plain", "<p>XP: " + String(LED_XP) + "</p><p>Level: " + String(LED_level) + "</p>");
}

void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found");
}
