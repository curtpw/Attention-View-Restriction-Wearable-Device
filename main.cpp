
//VIEW RESTRICTION EYE GLASS MOUNTED WEARABLE DEVICE FOR CONTROL OF ATTENTION
//FOR ACTIVITY TRACKER WITH KX002 ACCELEROMETER, 55MAH BATTERY, NRF51822-AA VARIANT, NO HR
//OLED PINS: 0, 1, 2, 29, 30
//BUTTON PIN: 4
//VIBRATE PIN: 7
//SERIAL PINS: 17, 18  RX/TX
//KX002 PINS: 16, 14   SCL/SDA

/****************************************************************************************************
// Includes
 ****************************************************************************************************/
#include "mbed.h"
#include <string>
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "SFE_MicroOLED.h"  //SSD1306 64x32 micro OLED
#include "BLE.h"            //Bluetooth Low Energy
#include "DFUService.h"     //Firmware Over The Air (FOTA
#include "UARTService.h"    //Serial over USB
#include "wire.h"           //for KX022-1020 Accelerometer

/****************************************************************************************************
// Defines
 ****************************************************************************************************/

#define SCL             16  
#define SDA             14

TwoWire Wire = TwoWire(NRF_TWI1); //Two Wire Interface (TWI) for KX022 Accelerometer I2C

#define BATTERY_PIN     p1
#define BUTTON_PIN1     p4
#define VIBRATE_PIN     p7
#define UART_TX         p18
#define UART_RX         p17

#define LOG(...)    { pc.printf(__VA_ARGS__); }

//Accelerometer i2c addresses
#define KX022_addr_w              0x3E
#define KX022_addr_r              0x3F
#define KX022_Accel_CNTL1_1       0x18
#define KX022_Accel_CNTL1_2       0x41
#define KX022_Accel_ODCNTL_1      0x1B
#define KX022_Accel_ODCNTL_2      0x02
#define KX022_Accel_CNTL3_1       0x1A
#define KX022_Accel_CNTL3_2       0xD8
#define KX022_Accel_TILT_TIMER_1  0x22
#define KX022_Accel_TILT_TIMER_2  0x01
#define KX022_Accel_CNTL2_1       0x18
#define KX022_Accel_CNTL2_2       0xC1  

#define XOUT_L 0x06
#define XOUT_H 0x07
#define YOUT_L 0x08
#define YOUT_H 0x09
#define ZOUT_L 0x0A
#define ZOUT_H 0x0B

/****************************************************************************************************
// Declarations
 ****************************************************************************************************/

Serial pc(UART_TX, UART_RX);

SPI my_spi(p2, p3, p1);     // mosi, miso, sclk   - SPI Interface for micro OLED
MicroOLED oled_display(my_spi, p30, p0, p29); // spi, rst, dc, cs

DigitalOut vibrate(VIBRATE_PIN, 0);

InterruptIn button1(BUTTON_PIN1);

//AnalogIn    battery(BATTERY_PIN);

//For Bluetooth
int read_none_count = 0;

BLEDevice  ble;
//UARTService *uartServicePtr;
UARTService *uart;

//#define NEED_BLE_CONSOLE_OUTPUT 0 /* if need debug messages on the console */

//#if NEED_BLE_CONSOLE_OUTPUT
//#define DEBUG(STR) { if (uart) uart->write(STR, strlen(STR)); }
//#else
//#define DEBUG(...) /* nothing */
//#endif /* #if NEED_BLE_CONSOLE_OUTPUT */

volatile bool bleIsConnected = false;
volatile uint8_t tick_event = 0;

//For KX022 Accelerometer
uint8_t        KX022_Content_ReadData[6];
#define KX022_Addr_Accel_ReadData 0x06    

float       KX022_Accel_X;
float       KX022_Accel_Y;                               
float       KX022_Accel_Z;

short int   KX022_Accel_X_RawOUT = 0;
short int   KX022_Accel_Y_RawOUT = 0;
short int   KX022_Accel_Z_RawOUT = 0;

int         KX022_Accel_X_LB = 0;
int         KX022_Accel_X_HB = 0;
int         KX022_Accel_Y_LB = 0;
int         KX022_Accel_Y_HB = 0;
int         KX022_Accel_Z_LB = 0;
int         KX022_Accel_Z_HB = 0;
float       KX022_Accel_X_OUT = 0;
float       KX022_Accel_Y_OUT = 0;
float       KX022_Accel_Z_OUT = 0;

//TOGGLE VIEW RESTRICTION
bool Toggle_View_Restriction;
//TRIGGER VIEW RESTRICTION TARGET
bool Get_Target;

//let ticker know to start display
bool dataOnOLED = false;

/****************************************************************************************************
// Function Prototypes   
 ****************************************************************************************************/
 
// KX022-1020 Accelerometer -------------------------------------------------------------------------
void writeTwoBytes (int one, int two);
void initSensor();
int getByte (int address);
float getAccelX();
float getAccelY();
float getAccelZ();

// Utility Functions --------------------------------------------------------------------------------
//void i2c_scanner(void);
void loadScreen(void);
void detect(void);
void tick(void);

/****************************************************************************************************
// Evothings BLE GATT
 ****************************************************************************************************/

//Evothings App control toggle state
int toggleState = 0;

uint16_t customServiceUUID  = 0xA000;
uint16_t readCharUUID       = 0xA001;
uint16_t writeCharUUID      = 0xA002;

const static char     DEVICE_NAME[]        = "ChildMind"; 
static const uint16_t uuid16_list[]        = {0xFFFF};    //Custom UUID, FFFF is reserved for development

/* Set Up custom Characteristics */
static uint8_t readValue[10] = {0,1,2,3,4,5,6};
ReadOnlyArrayGattCharacteristic<uint8_t, sizeof(readValue)> readChar(readCharUUID, readValue);

static uint8_t writeValue[10] = {0};
WriteOnlyArrayGattCharacteristic<uint8_t, sizeof(writeValue)> writeChar(writeCharUUID, writeValue);

/* Set up custom service */
GattCharacteristic *characteristics[] = {&readChar, &writeChar};
GattService customService(customServiceUUID, characteristics, sizeof(characteristics) / sizeof(GattCharacteristic *));

/*
 *  Restart advertising when phone app disconnects
*/
void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *)
{
    BLE::Instance(BLE::DEFAULT_INSTANCE).gap().startAdvertising();
}

/*
 *  Handle writes to writeCharacteristic
*/
void writeCharCallback(const GattWriteCallbackParams *params)
{
    /* Check to see what characteristic was written, by handle */
    if(params->handle == writeChar.getValueHandle()) {
        /* toggle LED if only 1 byte is written */
        if(params->len == 1) {
            
            int beforeState = toggleState;
            toggleState = params->data[0];
            
            //treat Evothing app toggle as button press ie restrict angle target/off
            if(beforeState != toggleState)
            {
                detect();
            }
            
         //   vibrate = params->data[0]; //turn vibration motor on
            
            (params->data[0] == 0x00) ? printf("BLE toggle on\n\r") : printf("BLE toggle off\n\r"); // print BLE toggle
        }
        /* Print the data if more than 1 byte is written */
        else {
            
            printf("Data received: length = %d, data = 0x",params->len);
            for(int x=0; x < params->len; x++) {
                printf("%x", params->data[x]);
            }
            printf("\n\r");
        }
        /* Update the readChar with the value of writeChar */
        BLE::Instance(BLE::DEFAULT_INSTANCE).gattServer().write(readChar.getValueHandle(), params->data, params->len);
    }
}
/*
 * Initialization callback
 */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE &ble          = params->ble;
    ble_error_t error = params->error;
    
    if (error != BLE_ERROR_NONE) {
        printf("BLE error");
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);
    ble.gattServer().onDataWritten(writeCharCallback);

    /* Setup advertising */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE); // BLE only, no classic BT
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED); // advertising type
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME)); // add name
    
    //choose Bluetooth BLE console GAP broadcast or App connectivity GATT service 
 //   #if NEED_BLE_CONSOLE_OUTPUT
 //   ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
 //                                    (const uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));
 //   #else                                
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list)); // UUID's broadcast in advertising packet
 //   #endif /* #if NEED_BLE_CONSOLE_OUTPUT */
    
    ble.gap().setAdvertisingInterval(100); // 100ms.

    /* Add our custom service */
    ble.addService(customService);

    /* Start advertising */
    ble.gap().startAdvertising();
}


/****************************************************************************************************
// main
 ****************************************************************************************************/
int main(void)
{
        
    pc.baud(115200);
 
    //view restriction on/off
    Toggle_View_Restriction = false;
    
    //Get target angle
    Get_Target = false;
 
    //center of allowed view area
    float Accel_X,
          Accel_Y,
          Accel_Z,
          Target_Accel_X = 999,
          Target_Accel_Y = 999,                               
          Target_Accel_Z = 999;  
 
    float restrictAngleSize = 45 / 2;
    
    wait(1);
    
    printf("\n\r********* Starting Main *********\n\r");
    
    Ticker ticker;              //Initialize timed interupt 
    ticker.attach(tick, 0.5);
    
    //monitor button
    button1.fall(detect); 


    
/****************************************************************************************************
    Initialize I2C & SPI Devices ************
****************************************************************************************************/

    /* START OLED */
 /*   oled_display.init(0, 8000000);  //start display
    

    oled_display.setFontType(0); //big characters
    oled_display.puts("TEST TEST");
    oled_display.display();
        wait(2.0);
     //   oled_display.clear(ALL);
        
    loadScreen();                   //load CMI logo
  //  oled_display.clear(ALL);        //clear load screen
    
    wait(0.2);
    
  //  oled_display.clear(ALL);
    */
   /* 
    oled_display.setFontType(0); //big characters
    oled_display.puts("CMI");
    oled_display.setCursor(0,11);
    oled_display.puts("Wearables");
    oled_display.setCursor(0,22);
    oled_display.puts("Curt White");   
    */
  //  oled_display.circle(16, 31, 16);
   // oled_display.line(0, 9, 63, 47);
   // oled_display.rectFill(33, 32, 8, 8);
 //   oled_display.drawBitmap(loadScreen);
  //  oled_display.display();
    /* END OLED TEST */

    
/****************************************************************************************************
    Initialize Bluetooth BLE ************
****************************************************************************************************/
    BLE& ble = BLE::Instance(BLE::DEFAULT_INSTANCE);
    ble.init(bleInitComplete);
    
   // #if NEED_BLE_CONSOLE_OUTPUT
 //   uart = new UARTService(ble);
  //  #endif /* #if NEED_BLE_CONSOLE_OUTPUT */
    
    /* Enable over-the-air firmware updates. Instantiating DFUSservice introduces a
     * control characteristic which can be used to trigger the application to
     * handover control to a resident bootloader. */

    /* SpinWait for initialization to complete. This is necessary because the
     * BLE object is used in the main loop below. */
    while (ble.hasInitialized()  == false) { /* spin loop */ }

    /* Infinite loop waiting for BLE interrupt events */
//    while (true) {  ******* DISABLED  1-16-17
//        ble.waitForEvent(); /* Save power */
//    }   

    Wire.begin(SCL, SDA, TWI_FREQUENCY_100K); //I2C bus for KX022 Accelerometer
 
    initSensor();               //Begin KX022 Accelerometer

/****************************************************************************************************
    Loop ************
****************************************************************************************************/
    printf("\n\r********* Starting Infinate Loop *********\n\r");
    
    while (true) {  
    
    dataOnOLED = true; //start display data on OLED
    
/****************************************************************************************************
// Read KX022 Accelerometer
 ****************************************************************************************************/
      wait(0.05);
          
      Accel_X = getAccelX()*100;
      char array1[15];
      sprintf(array1, "%f", Accel_X );
      printf("X = ");
      printf(array1);
    //  printf(" g\n");
    
      Accel_Y = getAccelY()*100;
      char array2[15];
      sprintf(array2, "%f", Accel_Y );
      printf(" Y = ");
      printf(array2);
  //    printf(" g\n");
         
      Accel_Z = getAccelZ()*100;
      char array3[15];
      sprintf(array3, "%f", Accel_Z );
      printf(" Z = ");
      printf(array3);
      printf(" g\n");
      
      
      printf("TX = ");
      char array4[15];
      sprintf(array4, "%f", Target_Accel_X );
      printf(array4);
      
      printf(" TY = ");
      char array5[15];
      sprintf(array5, "%f", Target_Accel_Y );
      printf(array5);
      
        printf(" TZ = ");
      char array6[15];
      sprintf(array6, "%f", Target_Accel_Z );
      printf(array6);
      
      printf(" g\n");
      
      //compare view restriction target angle to current angular position
      if(Toggle_View_Restriction)
      {
        if( Target_Accel_X > Accel_X + restrictAngleSize
        || Target_Accel_X < Accel_X - restrictAngleSize
        || Target_Accel_Z > Accel_Z + restrictAngleSize
        || Target_Accel_Z < Accel_Z - restrictAngleSize)
        {
            vibrate = 1;
            printf("\n\r* RESTRICTED *\n\r");
        }
        else
        {
            vibrate = 0;
        }
      }
      
      //get target angle if triggered
      if(Get_Target)
      {
          Target_Accel_X = Accel_X;
          Target_Accel_Y = Accel_Y; 
          Target_Accel_Z = Accel_Z;  
          Get_Target = false;
       }
       
    } 
}

/****************************************************************************************************
// KX022-1020 Accelerometer
 ****************************************************************************************************/
 
void writeTwoBytes (int one, int two)
{
    Wire.beginTransmission(KX022_addr_w);
    Wire.write(one);
    Wire.write(two);
    Wire.endTransmission();
}

void initSensor(){
    writeTwoBytes(KX022_Accel_CNTL1_1,KX022_Accel_CNTL1_2);
    writeTwoBytes(KX022_Accel_ODCNTL_1,KX022_Accel_ODCNTL_2);
    writeTwoBytes(KX022_Accel_CNTL3_1,KX022_Accel_CNTL3_2);
    writeTwoBytes(KX022_Accel_TILT_TIMER_1,KX022_Accel_TILT_TIMER_2);
    writeTwoBytes(KX022_Accel_CNTL2_1,KX022_Accel_CNTL2_2);
}

int getByte (int address)
{
  int readedValue;
  Wire.beginTransmission(KX022_addr_w);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(KX022_addr_r , 1);  // Or-ed with "1" for read bit
  if(1 <= Wire.available())    // if two bytes were received
    {
  readedValue = Wire.read();
  }
  return readedValue;
}
float getAccelX(){
  KX022_Accel_X_LB = getByte(XOUT_L);
  KX022_Accel_X_HB = getByte(XOUT_H);
  KX022_Accel_X_RawOUT = (KX022_Accel_X_HB<<8) | (KX022_Accel_X_LB);
  KX022_Accel_X_OUT = (float)KX022_Accel_X_RawOUT / 16384;
  return KX022_Accel_X_OUT;
}
float getAccelY(){
  KX022_Accel_Y_LB = getByte(YOUT_L); 
  KX022_Accel_Y_HB = getByte(YOUT_H);
  KX022_Accel_Y_RawOUT = (KX022_Accel_Y_HB<<8) | (KX022_Accel_Y_LB);
  KX022_Accel_Y_OUT = (float)KX022_Accel_Y_RawOUT / 16384;
  return KX022_Accel_X_OUT;
}
float getAccelZ(){
   KX022_Accel_Z_LB = getByte(ZOUT_L); 
   KX022_Accel_Z_HB = getByte(ZOUT_H); 
   KX022_Accel_Z_RawOUT = (KX022_Accel_Z_HB<<8) | (KX022_Accel_Z_LB);
   KX022_Accel_Z_OUT = (float)KX022_Accel_Z_RawOUT / 16384;
   return KX022_Accel_Z_OUT;
}


/****************************************************************************************************
// UTILITY FUNCTIONS
 ****************************************************************************************************/

/*
void i2c_scanner(void)
{
    int error, address;
    int nDevices;
    printf("Scanning i2c...\n");
    nDevices = 0;
         
    for(address = 1; address < 127; address++ ) 
    {
        i2c.start();
        error = i2c.write(address << 1); //We shift it left because mbed takes in 8 bit addreses
        i2c.stop();
        if (error == 1)
        {
            printf("I2C device found at address 0x%X", address); //Returns 7-bit addres
            nDevices++;
        }
    }
    if (nDevices == 0)
        printf("No I2C devices found\n");
    else
        printf("\ndone\n");
 //   wait(3);           // wait 5 seconds for next scan
}
*/

void tick(void)
{
    static uint32_t count = 0;
    char countText[ 16 ];
    LOG("%d\r\n", count++);
    sprintf(countText,"%lu", count);

/* ~~~~ TEST CODE ~~~~ */
  //  i2c_scanner();
  
  /*  vibrate = 1;
    wait(1.0); 
    vibrate = 0; */
}

void loadScreen(void)
{
    const unsigned char loadBitmap [] = {
    0x00, 0xC0, 0x20, 0x90, 0x48, 0x24, 0x92, 0x92, 0x92, 0x92, 0x12, 0x12, 0x22, 0x22, 0xC2, 0x02,
    0x04, 0x04, 0x18, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0x7E, 0x81, 0x00, 0x00, 0x00, 0x00, 0x09, 0xDE, 0x20, 0x08, 0x08, 0x1F,
    0xA0, 0x48, 0x14, 0x14, 0x08, 0x1F, 0x60, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x42,
    0x42, 0x42, 0x24, 0x00, 0x7E, 0x04, 0x38, 0x04, 0x7E, 0x00, 0x42, 0x7E, 0x42, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x07, 0x08, 0x11, 0x62, 0x84, 0x1B, 0xE4, 0x04, 0x04, 0x03, 0x01, 0x80, 0x70, 0x10, 0x11,
    0x0F, 0x00, 0x80, 0x80, 0x80, 0x88, 0x7F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x40, 0x30, 0x40,
    0x3E, 0x00, 0x38, 0x54, 0x54, 0x18, 0x00, 0x20, 0x54, 0x54, 0x78, 0x40, 0x00, 0x7C, 0x08, 0x04,
    0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x30, 0x2F, 0x20, 0x20, 0x21, 0x11, 0x11, 0x11, 0x10, 0x10, 0x10, 0x10,
    0x1E, 0x11, 0x10, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    
    oled_display.clear(ALL);
    oled_display.drawBitmap(loadBitmap);
    oled_display.display();
    wait(2.0); 
}


void detect(void)
{
    LOG("Button pressed\n");  
    
    if(Toggle_View_Restriction)
    {
        //turn view restriction off
        Toggle_View_Restriction = false;
        vibrate = 1;
        wait(1.5); 
        vibrate = 0;
        wait(1.0);
        vibrate = 1;
        wait(1.5); 
        vibrate = 0;
    } 
    else 
    {
        //turn view restriction on
        Toggle_View_Restriction = true;
        
        //capture target angle for view restriction
        Get_Target = true;
      
        vibrate = 1;
        wait(1.5); 
        vibrate = 0;
    }
}