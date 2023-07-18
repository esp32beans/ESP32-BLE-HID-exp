/*
 * This program is based on https://github.com/h2zero/NimBLE-Arduino/tree/master/examples/NimBLE_Client.
 * My changes are covered by the MIT license.
 */

/*
 * MIT License
 *
 * Copyright (c) 2023 esp32beans@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#define XBOX_ONE_GAMEPAD 1
#define DUMP_REPORT_MAP 1
#define DEV_INFO_SERVICE 1    // BLE device information

// Set to 0 to remove the CDC ACM serial port but XAC seems to tolerate it
// so it is on by default. Set to 0 if it causes problems. Disabling the
// CDC port means you must press button(s) on the ESP32S3 to put in bootloader
// upload mode before using the IDE to upload.
#define USB_DEBUG 1

#if USB_DEBUG
#define DBG_begin(...)    Serial.begin(__VA_ARGS__)
#define DBG_end(...)      Serial.end(__VA_ARGS__)
#define DBG_print(...)    Serial.print(__VA_ARGS__)
#define DBG_println(...)  Serial.println(__VA_ARGS__)
#define DBG_printf(...)   Serial.printf(__VA_ARGS__)
#else
#define DBG_begin(...)
#define DBG_end(...)
#define DBG_print(...)
#define DBG_println(...)
#define DBG_printf(...)
#endif

#include <stdint.h>

typedef struct __attribute__((packed)) {
  uint16_t x;           // 0..65534
  uint16_t y;           // 0..65534
  uint16_t z;           // 0..65534
  uint16_t rz;          // 0..65534
  uint16_t brake:10;    // 0..1023
  uint16_t filler1:6;
  uint16_t accelerator:10; // 0..1023
  uint16_t filler2:6;
  uint8_t  hat:4;
  uint8_t  filler3:4;
  uint16_t buttons:15;
  uint16_t filler4:1;
  uint8_t  record:1;
  uint8_t  filler5:7;
} xbox_one_gamepad_t;

typedef struct {
  uint8_t report[32];
  size_t report_len;
  uint32_t last_millis = 0;
  bool isNotify;
  bool available;
} Report_xfer_state_t;

volatile Report_xfer_state_t Report_xfer;

// Install NimBLE-Arduino by h2zero using the IDE library manager.
#include <NimBLEDevice.h>

const uint16_t APPEARANCE_HID_GENERIC = 0x3C0;
const uint16_t APPEARANCE_HID_KEYBOARD = 0x3C1;
const uint16_t APPEARANCE_HID_MOUSE   = 0x3C2;
const uint16_t APPEARANCE_HID_JOYSTICK = 0x3C3;
const uint16_t APPEARANCE_HID_GAMEPAD = 0x3C4;
const uint16_t APPEARANCE_HID_DIGITIZER_TABLET = 0x3C5;
const uint16_t APPEARANCE_HID_CARD_READER = 0x3C6;
const uint16_t APPEARANCE_HID_DIGITAL_PEN = 0x3C7;
const uint16_t APPEARANCE_HID_BARCODE_SCANNER = 0x3C8;
const uint16_t APPEARANCE_HID_TOUCHPAD = 0x3C9;
const uint16_t APPEARANCE_HID_PRESENTATION_REMOTE = 0x3CA;

const char DEVICE_INFORMATION_SERVICE[] = "180A";
const char DIS_SYSTEM_ID_CHAR[] = "2A23";
const char DIS_MODEL_NUM_CHAR[] = "2A24";
const char DIS_SERIAL_NUM_CHAR[] = "2A25";
const char DIS_FIRMWARE_REV_CHAR[] = "2A26";
const char DIS_HARDWARE_REV_CHAR[] = "2A27";
const char DIS_SOFTWARE_REV_CHAR[] = "2A28";
const char DIS_MANUFACTURE_NAME_CHAR[] = "2A29";
const char DIS_PNP_ID_CHAR[] = "2A50";
const char HID_SERVICE[] = "1812";
const char HID_INFORMATION[] = "2A4A";
const char HID_REPORT_MAP[] = "2A4B";
const char HID_CONTROL_POINT[] = "2A4C";
const char HID_REPORT_DATA[] = "2A4D";
const char HID_PROTOCOL_MODE[] = "2A4E";
const char HID_BOOT_KEYBOARD_OUTPUT_REPORT[] = "2A32";
const char HID_BOOT_MOUSE_INPUT_REPORT[] = "2A33";

void scanEndedCB(NimBLEScanResults results);

static NimBLEAdvertisedDevice* advDevice;

static bool doConnect = false;
static uint32_t scanTime = 0; /** 0 = scan forever */

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) {
    DBG_println("Connected");
    /** After connection we should change the parameters if we don't need fast response times.
     *  These settings are 150ms interval, 0 latency, 450ms timout.
     *  Timeout should be a multiple of the interval, minimum is 100ms.
     *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
     *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
     */
    pClient->updateConnParams(120,120,0,60);
    DBG_printf("%s: peer MTU %u\n", __func__, pClient->getMTU());
  };

  void onDisconnect(NimBLEClient* pClient) {
    DBG_print(pClient->getPeerAddress().toString().c_str());
    DBG_println(" Disconnected - Starting scan");
    NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
  };

  /** Called when the peripheral requests a change to the connection parameters.
   *  Return true to accept and apply them or false to reject and keep
   *  the currently used parameters. Default will return true.
   */
  bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) {
    // Failing to accepts parameters may result in the remote device
    // disconnecting.
    return true;
  };

  /********************* Security handled here **********************
   ****** Note: these are the same return values as defaults ********/
  uint32_t onPassKeyRequest(){
    DBG_println("Client Passkey Request");
    /** return the passkey to send to the server */
    return 123456;
  };

  bool onConfirmPIN(uint32_t pass_key){
    DBG_print("The passkey YES/NO number: ");
    DBG_println(pass_key);
    /** Return false if passkeys don't match. */
    return true;
  };

  /** Pairing process complete, we can check the results in ble_gap_conn_desc */
  void onAuthenticationComplete(ble_gap_conn_desc* desc){
    if(!desc->sec_state.encrypted) {
      DBG_println("Encrypt connection failed - disconnecting");
      /** Find the client with the connection handle provided in desc */
      NimBLEDevice::getClientByID(desc->conn_handle)->disconnect();
      return;
    }
  };
};

/** Define a class to handle the callbacks when advertisments are received */
class AdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {

  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    uint8_t advType = advertisedDevice->getAdvType();
    if ((advType == BLE_HCI_ADV_TYPE_ADV_DIRECT_IND_HD) ||
        (advType == BLE_HCI_ADV_TYPE_ADV_DIRECT_IND_LD) ||
        (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(NimBLEUUID(HID_SERVICE))))
    {
      DBG_printf("onResult: AdvType= %d\r\n", advType);
      DBG_print("Advertised HID Device found: ");
      DBG_println(advertisedDevice->toString().c_str());

      /** stop scan before connecting */
      NimBLEDevice::getScan()->stop();
      /** Save the device reference in a global for the client to use*/
      advDevice = advertisedDevice;
      /** Ready to connect now */
      doConnect = true;
    }
  };
};


/** Notification / Indication receiving handler callback */
// Notification from 4c:75:25:xx:yy:zz: Service = 0x1812, Characteristic = 0x2a4d, Value = 1,0,0,0,0,
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic,
    uint8_t* pData, size_t length, bool isNotify) {
  if (Report_xfer.available) {
    static uint32_t dropped = 0;
    printf("drops=%u\r\n", ++dropped);
  } else {
    // DBG_println(pRemoteCharacteristic->toString().c_str());
    Report_xfer.isNotify = isNotify;
    Report_xfer.report_len = length;
    memcpy((void *)Report_xfer.report, pData, min(length, sizeof(Report_xfer.report)));
    Report_xfer.available = true;
    Report_xfer.last_millis = millis();
  }
}

/** Callback to process the results of the last scan or restart it */
void scanEndedCB(NimBLEScanResults results){
  DBG_println("Scan Ended");
}


/** Create a single global instance of the callback class to be used by all clients */
static ClientCallbacks clientCB;

static NimBLEAddress LastBLEAddress;

/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToServer()
{
  NimBLEClient* pClient = nullptr;
  bool reconnected = false;

  DBG_printf("Client List Size: %d\r\n", NimBLEDevice::getClientListSize());
  /** Check if we have a client we should reuse first **/
  if(NimBLEDevice::getClientListSize()) {
    /** Special case when we already know this device, we send false as the
     *  second argument in connect() to prevent refreshing the service database.
     *  This saves considerable time and power.
     */
    pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
    if(pClient) {
      if (pClient->getPeerAddress() == LastBLEAddress) {
        if(!pClient->connect(advDevice, false)) {
          DBG_println("Reconnect failed");
          return false;
        }
        DBG_println("Reconnected client");
        reconnected = true;
      }
    }
    /** We don't already have a client that knows this device,
     *  we will check for a client that is disconnected that we can use.
     */
    else {
      pClient = NimBLEDevice::getDisconnectedClient();
    }
  }

  /** No client to reuse? Create a new one. */
  if(!pClient) {
    if(NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS) {
      DBG_println("Max clients reached - no more connections available");
      return false;
    }

    pClient = NimBLEDevice::createClient();

    DBG_println("New client created");

    pClient->setClientCallbacks(&clientCB, false);
    /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
     *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
     *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
     *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
     */
    pClient->setConnectionParams(12,12,0,51);
    /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
    pClient->setConnectTimeout(5);


    if (!pClient->connect(advDevice)) {
      /** Created a client but failed to connect, don't need to keep it as it has no data */
      NimBLEDevice::deleteClient(pClient);
      DBG_println("Failed to connect, deleted client");
      return false;
    }
  }

  if(!pClient->isConnected()) {
    if (!pClient->connect(advDevice)) {
      DBG_println("Failed to connect");
      return false;
    }
  }

  LastBLEAddress = pClient->getPeerAddress();
  DBG_print("Connected to: ");
  DBG_println(pClient->getPeerAddress().toString().c_str());
  DBG_print("RSSI: ");
  DBG_println(pClient->getRssi());

  /** Now we can read/write/subscribe the charateristics of the services we are interested in */
  NimBLERemoteService* pSvc = nullptr;
  NimBLERemoteCharacteristic* pChr = nullptr;
  NimBLERemoteDescriptor* pDsc = nullptr;

#if DEV_INFO_SERVICE
  // Device Information Service
  pSvc = pClient->getService(DEVICE_INFORMATION_SERVICE);
  if(pSvc) {     /** make sure it's not null */
    DBG_println(pSvc->toString().c_str());
    std::vector<NimBLERemoteCharacteristic*>*charvector;
    charvector = pSvc->getCharacteristics(true);
    for (auto &it: *charvector) {
      DBG_print(it->toString().c_str());
      if(it->canRead()) {
        DBG_print(" Value: ");
        DBG_println(it->readValue().c_str());
      }
    }
    pChr = pSvc->getCharacteristic(DIS_PNP_ID_CHAR);
    if(pChr) {     /** make sure it's not null */
      if(pChr->canRead()) {
        typedef struct __attribute__((__packed__)) {
          uint8_t vendor_id_source;
          uint16_t vendor_id;
          uint16_t product_id;
          uint16_t product_version;
        } pnp_id_t;
        pnp_id_t *pnp_id = (pnp_id_t *)pChr->readValue().data();
        uint16_t vid = pnp_id->vendor_id;
        uint16_t pid = pnp_id->product_id;
        DBG_printf("PNP ID: source: %02x, vendor: %04x, product: %04x, version: %04x\r\n",
            pnp_id->vendor_id_source, vid, pid, pnp_id->product_version);
      }
    }
  } else {
    DBG_println("Device Information Service not found.");
  }
#endif

  pSvc = pClient->getService(HID_SERVICE);
  if(pSvc) {     /** make sure it's not null */
      // This returns the HID report descriptor like this
      // HID_REPORT_MAP 0x2a4b Value: 5,1,9,2,A1,1,9,1,A1,0,5,9,19,1,29,5,15,0,25,1,75,1,
      // Copy and paste the value digits to http://eleccelerator.com/usbdescreqparser/
      // to see the decoded report descriptor.
      pChr = pSvc->getCharacteristic(HID_REPORT_MAP);
      if(pChr) {     /** make sure it's not null */
        if(pChr->canRead()) {
          std::string value = pChr->readValue();
          if (!reconnected) {
#if DUMP_REPORT_MAP
          DBG_print("HID_REPORT_MAP ");
          DBG_print(pChr->getUUID().toString().c_str());
          DBG_print(" Value: ");
          uint8_t *p = (uint8_t *)value.data();
          for (size_t i = 0; i < value.length(); i++) {
            DBG_print(p[i], HEX);
            DBG_print(',');
          }
          DBG_println();
#endif
          }
        }
      }
      else {
        DBG_println("HID REPORT MAP char not found.");
      }

    // Subscribe to characteristics HID_REPORT_DATA.
    // One real device reports 2 with the same UUID but
    // different handles. Using getCharacteristic() results
    // in subscribing to only one.
    std::vector<NimBLERemoteCharacteristic*>*charvector;
    charvector = pSvc->getCharacteristics(true);
    for (auto &it: *charvector) {
      if (it->getUUID() == NimBLEUUID(HID_REPORT_DATA)) {
        DBG_println(it->toString().c_str());
        if (it->canNotify()) {
          if(it->subscribe(true, notifyCB)) {
            DBG_println("subscribe notification OK");
          } else {
            /** Disconnect if subscribe failed */
            DBG_println("subscribe notification failed");
            pClient->disconnect();
            return false;
          }
        }
        if (it->canIndicate()) {
          if(it->subscribe(false, notifyCB)) {
            DBG_println("subscribe indication OK");
          } else {
            /** Disconnect if subscribe failed */
            DBG_println("subscribe indication failed");
            pClient->disconnect();
            return false;
          }
        }
      }
    }
  }
  DBG_println("Done with this device!");
  return true;
}

void setup ()
{
  DBG_begin(115200);

#if USB_DEBUG
  while ( !Serial ) delay(10);   // wait for native usb
#endif
  DBG_println("Starting NimBLE HID Client");
  /** Initialize NimBLE, no device name spcified as we are not advertising */
  NimBLEDevice::init("");
  if (NimBLEDevice::setMTU(512)) {
    DBG_printf("%s: setMTU(512) failed\n", __func__);
  }
  DBG_printf("%s: getMTU %d\n", __func__, NimBLEDevice::getMTU());

  /** Set the IO capabilities of the device, each option will trigger a different pairing method.
   *  BLE_HS_IO_KEYBOARD_ONLY    - Passkey pairing
   *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
   *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
   */
  //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
  //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

  /** 2 different ways to set security - both calls achieve the same result.
   *  no bonding, no man in the middle protection, secure connections.
   *
   *  These are the default values, only shown here for demonstration.
   */
  NimBLEDevice::setSecurityAuth(true, true, true);
  //NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

  /** Optional: set the transmit power, default is 3db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */

  /** Optional: set any devices you don't want to get advertisments from */
  // NimBLEDevice::addIgnored(NimBLEAddress ("aa:bb:cc:dd:ee:ff"));

  /** create new scan */
  NimBLEScan* pScan = NimBLEDevice::getScan();

  /** create a callback that gets called when advertisers are found */
  pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());

  /** Set scan interval (how often) and window (how long) in milliseconds */
  pScan->setInterval(22);
  pScan->setWindow(11);

  /** Active scan will gather scan response data from advertisers
   *  but will use more energy from both devices
   */
  pScan->setActiveScan(false);
  /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
   *  Optional callback for when scanning stops.
   */
  DBG_println("Scanning");
  pScan->start(scanTime, scanEndedCB);
}

void loop () {
  /** Loop here until we find a device we want to connect to */
  if (doConnect) {
    doConnect = false;

    /** Found a device we want to connect to, do it now */
    if(connectToServer()) {
      DBG_println("Success! we should now be getting notifications!");
    } else {
      DBG_println("Failed to connect, starting scan");
      NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
    }
  }
  else if (Report_xfer.available) {
    DBG_printf("%s:", __func__);
    for (size_t i = 0; i < Report_xfer.report_len; i++) {
      DBG_printf(" %02x", Report_xfer.report[i]);
    }
    DBG_println();
#if XBOX_ONE_GAMEPAD
    if (Report_xfer.report_len == sizeof(xbox_one_gamepad_t)) {
      xbox_one_gamepad_t *gp = (xbox_one_gamepad_t *)Report_xfer.report;
      DBG_printf("x=%u,y=%u,z=%u,rz=%u,brake=%u,gas=%u,hat=%u,buttons=x%04x,record=%u\n",
          gp->x, gp->y, gp->z, gp->rz, gp->brake, gp->accelerator,
          gp->hat, gp->buttons, gp->record);
    }
#endif
    Report_xfer.available = false;
  }
}
