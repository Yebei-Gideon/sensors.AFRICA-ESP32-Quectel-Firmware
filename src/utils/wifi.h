#ifndef WIFI_H
#define WIFI_H

#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <array>

static const IPAddress AP_IP(192, 168, 4, 1);
static DNSServer dnsServer;
#define DNS_PORT 53

// AP channel selection
static int selectChannelForAp()
{
  std::array<int, 14> channels_rssi;
  channels_rssi.fill(-100);

  // Read the number of scanned networks directly from the ESP32 Wi-Fi stack
  int16_t numNetworks = WiFi.scanComplete();
  if (numNetworks < 0)
  {
    numNetworks = 0; // If no scan data is present, fallback gracefully
  }

  for (uint8_t i = 0; i < numNetworks; i++)
  {
    int ch = WiFi.channel(i);

    if (ch >= 1 && ch <= 13)
    {
      channels_rssi[ch] = max(channels_rssi[ch], (int)WiFi.RSSI(i));
    }
  }

  int bestChannel = 6;
  int bestRSSI = -1000;

  // Evaluate least congested channels among 1, 6, and 11
  for (int ch : {1, 6, 11})
  {
    if (channels_rssi[ch] > bestRSSI)
    {
      bestRSSI = channels_rssi[ch];
      bestChannel = ch;
    }
  }

  return bestChannel;
}

// start AP mode
static void wifiAPbegin(const char *ssid, const char *pwd)
{
  Serial.printf("Starting AP: %s\n", ssid);

  // Set to AP_STA so it doesn't drop station operations while serving AP requests
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));

  int channel = selectChannelForAp();

  WiFi.softAP(ssid, pwd, channel);

  Serial.printf("AP started on channel %d\n", channel);

  dnsServer.setTTL(0);
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", AP_IP);
}

// stop AP mode
static void wifiAPstop()
{
  WiFi.softAPdisconnect(true);
  dnsServer.stop();
  delay(200);
}

// wait for WiFi to connect with a timeout
static void waitForWifiToConnect(int maxRetries)
{
  int retryCount = 0;

  while (WiFi.status() != WL_CONNECTED && retryCount < maxRetries)
  {
    delay(500);
    Serial.print(".");
    retryCount++;
  }

  Serial.println();
}

// connect to WiFi with fallback to AP mode if connection fails
static void connectWifi(const char *ssid, const char *pwd)
{
  Serial.printf("Connecting to %s\n", ssid);

  WiFi.mode(WIFI_AP_STA); // Maintain AP architecture alongside Station deployment
  WiFi.setAutoReconnect(true);

  WiFi.begin(ssid, pwd);

  waitForWifiToConnect(40);

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("Connected! IP: %s\n", WiFi.localIP().toString().c_str());

    if (MDNS.begin(ssid))
    {
      MDNS.addService("http", "tcp", 80);
    }
  }
  else
  {
    Serial.println("WiFi failed → switching to AP mode");
    wifiAPbegin(ssid, pwd);
  }
}

// Captive portal loop to wait for user to connect and view the portal
static void startCaptivePortal(bool &isViewed, unsigned long startTime, unsigned long timeout)
{
  while (!isViewed && (millis() - startTime < timeout))
  {
    dnsServer.processNextRequest();
    delay(1);
  }

  dnsServer.stop();
}

// Check if WiFi has internet connectivity by attempting to connect to a known host
inline bool wifiHasInternet(const char *host = "8.8.8.8", uint16_t port = 53, uint32_t timeoutMs = 3000)
{
  WiFiClient client;
  bool ok = client.connect(host, port, timeoutMs);
  if (ok)
    client.stop();
  return ok;
}

// Connect to WiFi with fallback to AP mode if connection fails, and verify internet connectivity
inline bool wifiConnect(const char *ssid, const char *pwd)
{
  Serial.printf("Connecting to %s\n", ssid);

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, pwd);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 60000)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Connection failed");
    return false;
  }

  Serial.printf("Connected IP: %s\n", WiFi.localIP().toString().c_str());

  if (!wifiHasInternet())
  {
    Serial.println("No internet → disconnecting");
    WiFi.disconnect();
    return false;
  }

  return true;
}

#endif
