
#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include <ArduinoJson.h>
#include <LittleFS.h>
#include "helpers.h"
#include "SD_handler.h"
#include "../global_configs.h"

static JsonDocument getDeviceConfig();
static void updateDeviceConfig();
static void saveConfig(JsonDocument &doc);
static void loadSavedDeviceConfigs(bool setConfigStates = true);

enum ConfigurationState
{
  CONFIG_BOOT_INIT,              // Reading existing config
  CONFIG_CAPTIVE_PORTAL_ACTIVE,  // Waiting for user input
  CONFIG_CAPTIVE_PORTAL_TIMEOUT, // Auto-continue
  CONFIG_WIFI,
  CONFIG_GSM,
  CONFIG_APPLIED, // Configs loaded & applied
  CONFIG_COMPLETE // Ready for runtime
};

struct DeviceConfigState
{
  ConfigurationState state;
  unsigned long captivePortalStartTime;
  unsigned long captivePortalTimeoutMs; // 5-10 mins
  bool configurationRequired = false;   // False if valid config exists
  bool captivePortalAccessed = false;   // Tracks if user accessed it
  bool wifiConnected = false;
  bool gsmConnected = false;
  bool wifiInternetAvailable = false;
  bool gsmInternetAvailable = false;
  bool internetAvailable = false;
  bool timeSet = false;
  bool sdCardInitialized = false;
  bool isMQTTConfigured = false;
  bool restartRequired = false;
};

extern struct DeviceConfigState DeviceConfigState;

struct DeviceConfig
{
  char wifi_sta_ssid[64] = {};
  char wifi_sta_pwd[64] = {};
  char gsm_apn[32] = {};
  char gsm_apn_pwd[32] = {};
  char sim_pin[8] = {};
  bool power_saving_mode = false;
  bool useWiFi;
  bool useGSM;
  bool isLive;
  char active_api_host[64] = {};
  char staging_host[64] = {};
  char production_host[64] = {};
};

extern struct DeviceConfig DeviceConfig;

static JsonDocument getDeviceConfig()
{
  JsonDocument doc;
  String config_data = readFile(LittleFS, "/config.json");
  if (config_data != "" && validateJson(config_data.c_str()))
  {
    deserializeJson(doc, config_data);
    return doc;
  }

  return doc;
}

static void updateDeviceConfig()
{

  JsonDocument config = getDeviceConfig();

  config["gsm_capable"] = gsm_capable;
  config["wifi_available"] = use_wifi;

  saveConfig(config);
}

static void saveConfig(JsonDocument &doc)
{
  // Helper: recursively merge src into dst ( adds new keys, updates differing values )
  static std::function<void(const JsonObjectConst &, JsonObject)> mergeJsonObjects =
      [&](const JsonObjectConst &src, JsonObject dst)
  {
    for (JsonPairConst kv : src)
    {
      const char *key = kv.key().c_str();
      JsonVariantConst v = kv.value();

      if (v.is<JsonObject>())
      {
        if (!dst[key].is<JsonObject>())
        {
          dst[key] = src[key];
        }
        else
        {
          JsonObject child = dst[key].as<JsonObject>();
          mergeJsonObjects(v.as<JsonObjectConst>(), child);
        }
      }
      else
      {
        dst[key] = v;
      }
    }
  };

  const char *new_config_file = "/config.json.new";
  String incoming;
  String existing = readFile(LittleFS, "/config.json");
  if (existing != "")
  {
    Serial.println("\nExisting config: ");
    Serial.println(existing);
  };
  Serial.println("\nIncoming doc");
  serializeJsonPretty(doc, incoming);
  Serial.println(incoming);

  // If there is no existing config or it's invalid, just write the passed doc
  if (existing == "" || !validateJson(existing.c_str()))
  {
    Serial.println("Invalid or empty json. Writing incoming config as new config");
    const char *c_string = incoming.c_str();
    writeFile(LittleFS, "/config.json", c_string);
    DeviceConfigState.configurationRequired = true;
    return;
  }

  JsonDocument existingDoc;
  DeserializationError err = deserializeJson(existingDoc, existing);
  JsonObject existingObj = existingDoc.as<JsonObject>();
  JsonObject newObj = doc.as<JsonObject>();

  if (!newObj.isNull())
  {
    mergeJsonObjects(newObj, existingObj);
  }

  String mergedString = "";
  serializeJson(existingDoc, mergedString);
  Serial.println("\nNew merged doc");
  Serial.println(mergedString);
  writeFile(LittleFS, new_config_file, mergedString.c_str());
  updateFileContents(LittleFS, "/config.json", new_config_file); //? merged string already has everything we need so overwrite is safe
  DeviceConfigState.configurationRequired = true;
}

static void loadSavedDeviceConfigs(bool setConfigStates)
{
  Serial.println("Loading saved configs");
  JsonDocument config = getDeviceConfig(); // {"ssid":"myssid ","wifiPwd":"mypwd","apn":"myapn","apnPwd":"myapnpwd","powerSaver":"off"}
  if (config.isNull())
  {
    Serial.println("Config non-existent or broken");
    return;
  }

  auto hasString = [&](JsonVariant v)
  {
    return !v.isNull() && v.is<const char *>() && v.as<const char *>()[0] != '\0';
  };

  bool wiFiUpdated = false, wifiSSIDUpdated = false, wifiPwdUpdated = false, gsmUpdated = false, apnUpdated = false, apnPwdUpdated = false, pinUpdated = false, useGSMUpdated = false, useWiFiUpdated = false;
  if (hasString(config["ssid"]))
  {
    wifiSSIDUpdated = !strstr(DeviceConfig.wifi_sta_ssid, config["ssid"]);
    strcpy(DeviceConfig.wifi_sta_ssid, config["ssid"]);
    if (hasString(config["wifiPwd"]))
    {
      wifiPwdUpdated = !strstr(DeviceConfig.wifi_sta_pwd, config["wifiPwd"]);
      strcpy(DeviceConfig.wifi_sta_pwd, config["wifiPwd"]);
    }
  }

  if (hasString(config["apn"]))
  {
    apnUpdated = !strstr(DeviceConfig.gsm_apn, config["apn"]);
    strcpy(DeviceConfig.gsm_apn, config["apn"]);
    if (hasString(config["apnPwd"]))
    {
      apnPwdUpdated = !strstr(DeviceConfig.gsm_apn_pwd, config["apnPwd"]);
      strcpy(DeviceConfig.gsm_apn_pwd, config["apnPwd"]);
    }
  }

  if (hasString(config["simPin"]))
  {
    pinUpdated = !strstr(DeviceConfig.sim_pin, config["simPin"]);
    strcpy(DeviceConfig.sim_pin, config["simPin"]);
  }

  if (hasString(config["powerSaver"]))
  {
    DeviceConfig.power_saving_mode = (config["powerSaver"] == "on") ? true : false;
    Serial.printf("Power saving mode updated to: %d\n", DeviceConfig.power_saving_mode);
  }
  if (hasString(config["useGSM"]))
  {
    bool val = (config["useGSM"] == "true");
    useGSMUpdated = (DeviceConfig.useGSM != val);
    DeviceConfig.useGSM = val;
  }
  if (hasString(config["useWiFi"]))
  {
    bool val = (config["useWiFi"] == "true" || config["useWiFi"] == "1");
    useWiFiUpdated = (DeviceConfig.useWiFi != val);
    DeviceConfig.useWiFi = val;
  }
  if (hasString(config["stagingHost"]))
  {
    strncpy(DeviceConfig.staging_host, config["stagingHost"].as<const char *>(), sizeof(DeviceConfig.staging_host) - 1);
  }
  if (hasString(config["productionHost"]))
  {
    strncpy(DeviceConfig.production_host, config["productionHost"].as<const char *>(), sizeof(DeviceConfig.production_host) - 1);
  }
  if (!config["isLive"].isNull())
  {
    // Safely extract regardless of whether it arrives as a Boolean (true) or a String ("true")
    bool val = false;
    if (config["isLive"].is<bool>())
    {
      val = config["isLive"].as<bool>();
    }
    else if (config["isLive"].is<const char *>())
    {
      val = (strcmp(config["isLive"].as<const char *>(), "true") == 0);
    }

    DeviceConfig.isLive = val;
    if (val)
    {
      Serial.println("Switching target environment to Production...");
      // Copy the parsed dynamic configuration value into active memory
      strncpy(DeviceConfig.active_api_host, DeviceConfig.production_host, sizeof(DeviceConfig.active_api_host) - 1);
    }
    else
    {
      Serial.println("Switching target environment to STAGING...");
      // Copy the parsed dynamic configuration value into active memory
      strncpy(DeviceConfig.active_api_host, DeviceConfig.staging_host, sizeof(DeviceConfig.active_api_host) - 1);
    }
  }

  gsmUpdated = apnPwdUpdated || apnPwdUpdated || pinUpdated;
  wiFiUpdated = wifiSSIDUpdated || wifiPwdUpdated;

  if (setConfigStates)
  {
    if (DeviceConfig.useGSM || DeviceConfig.useWiFi)
      DeviceConfigState.restartRequired = gsmUpdated || wifiPwdUpdated || useWiFiUpdated || useGSMUpdated;
  }
}

#endif
