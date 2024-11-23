#include "bluetoothService.h"

BLEUart bleuart;

BLEService blesStatus(STATUS_UUID_SERVICE);
BLECharacteristic blecStatus(STATUS_UUID_CHR_DATA);

String strADV_NAME = "Secora Oy       "; // 16 characters

extern byte LoRaAddress[8];

/*------------------------------------------------------------------*/
/* Peripheral
 *------------------------------------------------------------------*/
void prph_connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = {0};
  connection->getPeerName(peer_name, sizeof(peer_name));

  // Serial.print("[Prph] Connected to ");
  //  Serial.println(peer_name);
}

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void)conn_handle;
  (void)reason;

  // Serial.println();
  // Serial.println("[Prph] Disconnected");
}

/*------------------------------------------------------------------*/
/* Central
 *------------------------------------------------------------------*/

void cent_connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = {0};
  connection->getPeerName(peer_name, sizeof(peer_name));

  // Serial.print("[Cent] Connected to ");
  //  Serial.println(peer_name);
  ;

  // disconnect since we couldn't find bleuart service
  Bluefruit.disconnect(conn_handle);
}

void cent_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void)conn_handle;
  (void)reason;

  // Serial.println("[Cent] Disconnected");
}

void bluetoothInit()
{
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(1, 1);
  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  Bluefruit.setName("Secora");

  /**
   * ----------------------------------------------------
   * blecStatus
   * ----------------------------------------------------
   **/

  blesStatus.begin();
  blecStatus.setUserDescriptor("statusService");
  blecStatus.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  blecStatus.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  blecStatus.setMaxLen(200);
  blecStatus.begin();
  bt_write_status();
}

void bluetoothStartAdv(void)
{

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(blesStatus);

  // Include Name
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void bt_write_status(void)
{
  blecStatus.write(&LoRaAddress, 8);
}