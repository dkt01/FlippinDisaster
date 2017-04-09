#include <SoftwareSerial.h>

#include <Printers.h>
#include <XBee.h>

const uint8_t PIN_LATCH = 13;

XBee xbee;
SoftwareSerial serXBee(2,3);
Rx16Response rx16 = Rx16Response();
Tx16Request  tx16 = Tx16Request();

const unsigned long commsTimeout = 200;

typedef struct
{
  bool BUTTON_SELECT;
  bool BUTTON_L3;
  bool BUTTON_R3;
  bool BUTTON_START;
  bool BUTTON_UP;
  bool BUTTON_RIGHT;
  bool BUTTON_DOWN;
  bool BUTTON_LEFT;
  bool BUTTON_L2;
  bool BUTTON_R2;
  bool BUTTON_L1;
  bool BUTTON_R1;
  bool BUTTON_TRIANGLE;
  bool BUTTON_O;
  bool BUTTON_X;
  bool BUTTON_SQUARE;
  bool BUTTON_PS;

  uint8_t AXIS_LX;
  uint8_t AXIS_LY;
  uint8_t AXIS_RX;
  uint8_t AXIS_RY;

  bool deSerialize(uint8_t* data, uint8_t dataLength)
  {
    if(dataLength != 7)
    {
      // Should be 3B for buttons and 4B for axes
      Serial.write(dataLength);
      Serial.println("Bad length");
      return false;
    }
    // Deserialize buttons
    BUTTON_SELECT =   (data[0] >> 0) & 0x1;
    BUTTON_L3 =       (data[0] >> 1) & 0x1;
    BUTTON_R3 =       (data[0] >> 2) & 0x1;
    BUTTON_START =    (data[0] >> 3) & 0x1;
    BUTTON_UP =       (data[0] >> 4) & 0x1;
    BUTTON_RIGHT =    (data[0] >> 5) & 0x1;
    BUTTON_DOWN =     (data[0] >> 6) & 0x1;
    BUTTON_LEFT =     (data[0] >> 7) & 0x1;
    BUTTON_L2 =       (data[1] >> 0) & 0x1;
    BUTTON_R2 =       (data[1] >> 1) & 0x1;
    BUTTON_L1 =       (data[1] >> 2) & 0x1;
    BUTTON_R1 =       (data[1] >> 3) & 0x1;
    BUTTON_TRIANGLE = (data[1] >> 4) & 0x1;
    BUTTON_O =        (data[1] >> 5) & 0x1;
    BUTTON_X =        (data[1] >> 6) & 0x1;
    BUTTON_SQUARE =   (data[1] >> 7) & 0x1;
    BUTTON_PS =       (data[2] >> 0) & 0x1;
    // Deserialize axes
    AXIS_LX = data[3];
    AXIS_LY = data[4];
    AXIS_RX = data[5];
    AXIS_RY = data[6];

    return true;
  }

  void zero()
  {
    BUTTON_SELECT =   false;
    BUTTON_L3 =       false;
    BUTTON_R3 =       false;
    BUTTON_START =    false;
    BUTTON_UP =       false;
    BUTTON_RIGHT =    false;
    BUTTON_DOWN =     false;
    BUTTON_LEFT =     false;
    BUTTON_L2 =       false;
    BUTTON_R2 =       false;
    BUTTON_L1 =       false;
    BUTTON_R1 =       false;
    BUTTON_TRIANGLE = false;
    BUTTON_O =        false;
    BUTTON_X =        false;
    BUTTON_SQUARE =   false;
    BUTTON_PS =       false;

    AXIS_LX = 128;
    AXIS_LY = 128;
    AXIS_RX = 128;
    AXIS_RY = 128;
  }
  
} controllerState;

void setup()
{
  pinMode(PIN_LATCH, OUTPUT);
  
  xbee = XBee();

  // Setup USB serial
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect
  }

  serXBee.begin(9600);
  xbee.setSerial(serXBee);
//  xbee.setSerial(Serial);
}

void loop()
{
  static controllerState psState;
  static uint32_t count = 0;
  static unsigned long latestUpdate = 0;

  xbee.readPacket();
  if(xbee.getResponse().isAvailable())
  {
    if(xbee.getResponse().getApiId() == RX_16_RESPONSE)
    {
      latestUpdate = millis();
      xbee.getResponse().getRx16Response(rx16);
      if(psState.deSerialize(rx16.getFrameData() + rx16.getDataOffset(),rx16.getDataLength()))
      {
        tx16.setAddress16(rx16.getRemoteAddress16());
//        Serial.println(rx16.getRemoteAddress16());
        tx16.setPayload(rx16.getFrameData() + rx16.getDataOffset(), 1);//rx16.getDataLength());
//        Serial.println(count);
//        Serial.write(rx16.getFrameData() + rx16.getDataOffset(), rx16.getDataLength());
        xbee.send(tx16);
        count++;
      }
    }
  }
  if(millis() - latestUpdate >= commsTimeout)
  {
    psState.zero();
  }

  digitalWrite(PIN_LATCH, psState.BUTTON_X?HIGH:LOW);
}
