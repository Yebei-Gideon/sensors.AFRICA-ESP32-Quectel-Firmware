#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "LittleFS.h"
// #include "sensors-africa-logo.h"
#include "asyncserver.h"
#include <ArduinoJson.h>
#include "../utils/deviceconfig.h"
#include "../utils/wifi.h"
#include "../utils/SD_handler.h"
#include "../../include/helpers.h"

AsyncWebServer server(80);
extern uint8_t count_wifiInfo;
extern JsonDocument getCurrentSensorData();
extern char ROOT_DIR[24];
extern char AP_SSID[64];
String pendingFileList = "{}";
AsyncWebServerRequest *pendingRequest = nullptr;
extern JsonDocument device_info;

void setup_webserver()
{
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/style.css"); });
  server.on("/style.min.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/style.min.css"); });
  server.on("/script.min.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/script.min.js"); });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html"); });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              if (request->hasParam("skip")) {
                  DeviceConfigState.captivePortalAccessed = true;
                  // Skip the configuration page and send a simple response
                  request->send(200, "text/plain", "Skipped");
                  return;
              }

              // Check if config.html exists in LittleFS
              if (LittleFS.exists("/config.html")) {
                  Serial.println("Serving config.html from LittleFS");
                  request->send(LittleFS, "/config.html", "text/html");
              } else {
                  // Fallback
                  Serial.println("config.html missing from LittleFS");
                  request->send(404, "text/plain", "Configuration page missing.");
              } });
  server.on("/device-details.html", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/device-details.html"); });
  server.on("/ota.html", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/ota.html"); });
  server.on("/file-system.html", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/file-system.html"); });
  server.on("/advanced-settings.html", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/advanced-settings.html"); });
  // server.on("/sensors_logo.png", HTTP_GET, [](AsyncWebServerRequest *request)
  // { request->send(200, "image/png", SENSORSAFRICA_LOGO, SENSORSAFRICA_LOGO_PNG_SIZE); });
  server.on("/images/sensor_logo.png", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/images/sensor_logo.png", "image/png"); });
  server.on("/icons/wifi.svg", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/icons/wifi.svg", "image/svg+xml"); });
  server.on("/icons/simcard.svg", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/icons/simcard.svg", "image/svg+xml"); });
  server.on("/icons/lock.svg", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/icons/lock.svg", "image/svg+xml"); });
  server.on("/icons/cell_tower.svg", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/icons/cell_tower.svg", "image/svg+xml"); });
  server.on("/device-id", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", AP_SSID); });
  server.on("/device-config.json", [](AsyncWebServerRequest *request)
            {
        JsonDocument data=getDeviceConfig();
        String data_str;
        serializeJson(data,data_str);
        request->send(200,"application/json",data_str); });

  server.on("/available-hotspots", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              JsonDocument doc;

              int16_t numNetworks = WiFi.scanComplete();

              if (numNetworks == WIFI_SCAN_RUNNING)
              {
                request->send(202, "application/json", "{\"status\":\"scanning\"}");
                return;
              }

              if (numNetworks < 0)
              {
                numNetworks = WiFi.scanNetworks(false, true, false, 100);
              }

              Serial.printf("Serving %d hotspots as keyed objects\n", numNetworks);

              for (int i = 0; i < numNetworks; i++)
              {
                String ssid = WiFi.SSID(i);
                if (ssid.length() == 0)
                  continue;

                // Convert the encryption integer to a human-readable string
                String encString = "Open";
                switch (WiFi.encryptionType(i))
                {
                case WIFI_AUTH_WEP:
                  encString = "WEP";
                  break;
                case WIFI_AUTH_WPA_PSK:
                  encString = "WPA";
                  break;
                case WIFI_AUTH_WPA2_PSK:
                  encString = "WPA2";
                  break;
                case WIFI_AUTH_WPA_WPA2_PSK:
                  encString = "WPA/WPA2";
                  break;
                case WIFI_AUTH_WPA3_PSK:
                  encString = "WPA3";
                  break;
                default:
                  encString = "Other";
                  break;
                }

                // Create a nested object using the SSID as the key
                JsonObject networkDetails = doc[ssid].to<JsonObject>();
                networkDetails["rssi"] = WiFi.RSSI(i);
                networkDetails["encType"] = encString;
              }

              String hotspots;
              serializeJson(doc, hotspots);

              Serial.printf("Hotspots JSON size: %d bytes\n", hotspots.length());
              Serial.println(hotspots);

              request->send(200, "application/json", hotspots);

              // Kick off a fresh background scan after responding
              // so the NEXT time the user refreshes, the data is perfectly up-to-date.
              WiFi.scanNetworks(true, true); // 'true' makes it asynchronous background task
            });

  server.on("/save-config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            {
              JsonDocument new_config;
              DeserializationError error = deserializeJson(new_config, data, len);
              if (error)
              {
                request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
                return;
              }
              request->send(200, "application/json", "{\"status\":\"Config received\"}");
              DeviceConfigState.captivePortalAccessed = true; // Mark captive portal as accessed when config is saved
              saveConfig(new_config); });

  server.on("/sensor-data", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              // get JSON sensor data
              JsonDocument sensor_data = getCurrentSensorData();
              String sensor_data_str;
              serializeJson(sensor_data, sensor_data_str);
              request->send(200, "application/json", sensor_data_str); });

  server.on("/upload-firmware", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              request->send(200, "application/json", "{\"status\":\"Upload started\"}");
              Serial.println("Checking file system if firmware was uploaded");
              listFiles(LittleFS); }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
            {
              static File uploadFile;
              if (index == 0)
              {
                // Start upload
                Serial.printf("Upload started: %s\n", filename.c_str());
                uploadFile = LittleFS.open("/" + filename, "w");
                if (!uploadFile)
                {
                  Serial.println("Failed to open file for writing");
                  request->send(500, "application/json", "{\"error\":\"Failed to open file\"}");
                  return;
                }
              }
              if (uploadFile)
              {
                 uploadFile.write(data, len);
              }
              if (final)
              {
                if (uploadFile)
                {
                  uploadFile.close();
                  Serial.printf("Upload finished: %s\n", filename.c_str());
                  request->send(200, "application/json", "{\"status\":\"Upload successful\"}");
                }
                else
                {
                  request->send(500, "application/json", "{\"error\":\"File not open\"}");
                }
              } });

  // server.on("/list-files", HTTP_GET, [](AsyncWebServerRequest *request)
  //           { request->send(200, "application/json", listFiles(SD)); });

  server.on("/list-files", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      // Always generate a fresh list.
      // This ensures new files created by the logger are visible immediately.
      String currentFileList = listFiles(SD, String(ROOT_DIR));

      request->send(200, "application/json", currentFileList); });

  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    if (!request->hasParam("file"))
    {
        request->send(400, "text/plain", "Missing 'file' parameter");
        return;
    }

    String filePath = urlDecode(request->getParam("file")->value());
    filePath = normalizePath(filePath);

    if (isPathTraversal(filePath))
    {
        request->send(400, "text/plain", "Invalid file path");
        return;
    }

    String resolvedPath;
    if (!resolvePath(SD,filePath, resolvedPath, String(ROOT_DIR)))
    {
        Serial.printf("[download] Not found. raw=%s, decoded=%s, root=%s\n",
                      request->getParam("file")->value(), filePath.c_str(), ROOT_DIR);
        request->send(404, "text/plain", "File not found");
        return;
    }

    Serial.printf("[download] Serving: %s\n", resolvedPath.c_str());
    String filename = resolvedPath.substring(resolvedPath.lastIndexOf('/') + 1);

    AsyncWebServerResponse *response =
        request->beginResponse(SD, resolvedPath, "application/octet-stream");
    response->addHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    request->send(response); });

  server.on("/device-details", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String res;
              serializeJson(device_info,res);
              request->send(200,"application/json", res); });

  // Captive portal redirect
  server.onNotFound([](AsyncWebServerRequest *request)
                    { if(!DeviceConfigState.captivePortalAccessed) request->redirect("/config"); else request->send(404, "text/plain", "Not found"); });

  //! For comparison
  // void uploadFiles()
  // {
  //   // upload a new file to the SPIFFS
  //   HTTPUpload &upload = server.upload();
  //   if (upload.status == UPLOAD_FILE_START)
  //   {

  //     fname = upload.filename;
  //     if (!fname.startsWith("/"))
  //       fname = "/" + fname;
  //     Serial.print("Upload File Name: ");
  //     Serial.println(fname);
  //     uploadFile = SPIFFS.open(fname, "w"); // Open the file for writing in SPIFFS (create if it doesn't exist)
  //     if (uploadFile)
  //     {
  //       Serial.println("File opened");
  //     }
  //     // fname = String();
  //     Serial.print("fname: ");
  //     Serial.println(fname);
  //   }
  //   else if (upload.status == UPLOAD_FILE_WRITE)
  //   {
  //     if (uploadFile)
  //     {
  //       uploadFile.write(upload.buf, upload.currentSize);
  //       // Serial.println("written");
  //     }
  //   }

  //   else if (upload.status == UPLOAD_FILE_END)
  //   {
  //     if (uploadFile)
  //     {                     // If the file was successfully created
  //       uploadFile.close(); // Close the file again
  //       Serial.print("File Upload Size: ");
  //       Serial.println(upload.totalSize);
  //       String msg = "201: Successfully uploaded file ";
  //       msg += fname;
  //       server.send(200, "text/plain", msg);
  //       Serial.println(msg);

  //       if (fname == new_firmware_filename)
  //       {
  //         firmware_bin_saved = true;
  //       }
  //     }
  //     else
  //     {
  //       String err_msg = "500: failed creating file ";
  //       err_msg += fname;
  //       server.send(500, "text/plain", err_msg);
  //       Serial.println(err_msg);
  //     }
  //   }
  // }

  server.begin();
}
