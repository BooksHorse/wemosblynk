#define BLYNK_TEMPLATE_ID "TMPL6vx2PSTjS"
#define BLYNK_TEMPLATE_NAME "Bocchi"
#define BLYNK_AUTH_TOKEN "IdHmxo7v-5oCX7LlEslb84pA7tWHXksG"
#include <Arduino.h>
#include <BlynkSimpleEsp8266_SSL.h>
#include <SPI.h>
// put function declarations here:

#define BLYNK_PRINT Serial

#include <U8g2lib.h>
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 

void setup();
void loop();
void updatePattern(int);
void BocchiTask1();
void BocchiDevice();

class PinMutex
{

    bool locked;
    int lock_id;

  public:
    uint8_t pin;
    int lock(uint8_t lockId)
    {
        if (locked)
        {
            return -1;
        }
        locked = true;
        lock_id = lockId;
        return lockId;
    }
    int unlock()
    {
        locked = false;
        lock_id = -1;
        return 0;
    }
    PinMutex(uint8_t Pin)
    {
        pin = Pin;
        locked = false;
        lock_id = -1;
    }
};

class ToggleButton
{
    bool lastState;
    bool internalState;
    void (*BocchicallbackFunc)(bool);

  public:
    ToggleButton()
    {
        internalState = false;
        lastState = false;
    }
    void setraw(bool statusReading)
    {
        if (statusReading == true && lastState == false)
        {
            internalState = !internalState;
            BocchicallbackFunc(internalState);
        }
        lastState = statusReading;
    }
    bool getState()
    {
        return internalState;
    }
    void setCallbackOnStateUpdate(void (*callbackFunc)(bool))
    {
        BocchicallbackFunc = callbackFunc;
    }
    void setStateFromBlynk(bool state)
    {
        internalState = state;
    }
};

class Device
{
    uint8_t DeviceNumber;

  public:
    ToggleButton *btn;
    bool timerRunning;
    int timer;

    void (*BocchicallbackFunc)(uint8_t);
    void onTimerExpire(void (*callbackFunc)(uint8_t))
    {
        BocchicallbackFunc = (*callbackFunc);
    }
    void setTimer(int time)
    {
        timer = time;
    }
    bool isTimerOn()
    {
        return timerRunning;
    }
    void timerStart()
    {
        timerRunning = true;
    }
    void timerStop()
    {
        timer = -1;
        timerRunning = false;
    }
    bool timerReduce()
    {
        timer -= 1;
        if (timer == -1)
        {
            return false;
        }
        return true;
    }
    Device(uint8_t number, ToggleButton *button)
    {
        DeviceNumber = number;
        timerRunning = false;
        timer = -1;
        btn = button;
    }
};

char *transformIntoChar(Device *dev,bool display)
{
    char *result = (char *)malloc(20 * sizeof(char));
    char timerstr[10];
    bool state = dev->btn->getState();
    bool t_running = dev->timerRunning;
    if (t_running)
    {
        sprintf(timerstr, " (%is)", dev->timer);
    }
    else
    {
        if (state)
        {
            sprintf(timerstr, " (M)");
        }
        else
        {
            sprintf(timerstr, " (M)");
        }
    }
    switch (state)
    {
    case false: {
        if (display) {
            sprintf(result, "X %s", timerstr);
        } 
        else {
        sprintf(result, "✘ %s", timerstr);
        }
        break;
    }
    case true: {
        if (display) {
            sprintf(result, "/ %s", timerstr);
        }
        else {
        sprintf(result, "✓ %s", timerstr);
        }
        break;
    }
    }
    return result;
}

class DeviceStatus
{
    Device *dev0;
    Device *dev1;
    Device *dev2;
    char statusString[64];

  public:
    char *getStatusAsString()
    {
        char *dev0s = transformIntoChar(dev0,false);
        char *dev1s = transformIntoChar(dev1,false);
        char *dev2s = transformIntoChar(dev2,false);
        sprintf(statusString, "0: %s,1: %s,2: %s\n", dev0s, dev1s, dev2s);
        free(dev0s);
        free(dev1s);
        free(dev2s);
        char statusStringdisplay0[64];
        char statusStringdisplay1[64];
        char statusStringdisplay2[64];
        char *dev0sd = transformIntoChar(dev0,true);
        char *dev1sd = transformIntoChar(dev1,true);
        char *dev2sd = transformIntoChar(dev2,true);
        sprintf(statusStringdisplay0, "0: %s\n", dev0sd);
        sprintf(statusStringdisplay1, "1: %s\n", dev1sd);
        sprintf(statusStringdisplay2, "2: %s\n", dev2sd);
        free(dev0sd);
        free(dev1sd);
        free(dev2sd);
        u8x8.clear();
        u8x8.draw1x2String(0, 0, statusStringdisplay0);
        u8x8.draw1x2String(0,2,statusStringdisplay1);
        u8x8.draw1x2String(0,4,statusStringdisplay2);

        return statusString;
    }
    DeviceStatus(Device *btn0, Device *btn1, Device *btn2)
    {
        dev0 = btn0;
        dev1 = btn1;
        dev2 = btn2;
        sprintf(statusString, "uninited");
    }
    // void updateStatus(int device,bool status) {
    //     switch (device ) {
    //         case 0: {
    //             dev0 = status;
    //             break;
    //         }
    //         case 1: {
    //             dev1 = status;
    //             break;
    //         }
    //         case 2: {
    //             dev2 = status;
    //             break;
    //         }
    //     }
    // }
    // bool getStatus(int device) {
    //     switch (device ) {
    //         case 0: {
    //             return dev0;
    //             break;
    //         }
    //         case 1: {
    //             return dev1;
    //             break;
    //         }
    //         case 2: {
    //             return dev2;
    //             break;
    //         }
    //     }
    // }
};

#define bocchi_devcount 3
ToggleButton btn0;
ToggleButton btn1;
ToggleButton btn2;
Device dev0(0, &btn0);
Device dev1(1, &btn1);
Device dev2(2, &btn2);
Device *devices[bocchi_devcount] = {&dev0, &dev1, &dev2};

DeviceStatus kitachan(&dev0, &dev1, &dev2);

SPISettings spi_settings(100000, LSBFIRST, SPI_MODE0);
// uint8_t latchPin = D8;
// namespace constPin {
// uint8_t latchPin = D4;
// uint8_t clockPin = D5;
// uint8_t dataPin = D7;
// uint8_t S0_pin = D7;
// uint8_t S1_pin = D5;
// }; // namespace constPin

PinMutex d4(D4);
PinMutex d5(D5);
PinMutex d7(D7);

BlynkTimer timerBocchiTask1;
BlynkTimer timerBocchiDevice;
BlynkTimer timerStatusUpdateTask;

#define latchPin d4
#define clockPin d5
#define dataPin d7
#define S0Pin d7
#define S1Pin d5

#define WIFI_SSID "kitakitan"
#define WIFI_PASSWORD "bocchichan"

class bocchiMultiplex
{
  public:
};

enum State
{
    PINREAD,
    PINWRITE
};
void update0(bool state)
{
    dev0.timerStop();
    // kitachan.updateStatus(0, state);
    // Blynk.virtualWrite(V0, state);
}
void update1(bool state)
{
    dev1.timerStop();
    // kitachan.updateStatus(1, state);
    // Blynk.virtualWrite(V1, state);
}
void update2(bool state)
{
    dev2.timerStop();
    // kitachan.updateStatus(2, state);
    // Blynk.virtualWrite(V2, state);
}

void updateStatus()
{
    Blynk.virtualWrite(V0, kitachan.getStatusAsString());
    Serial.println(kitachan.getStatusAsString());
}

void turnoff(uint8_t dev)
{
    switch (dev)
    {
    case 0: {
        btn0.setStateFromBlynk(false);
        break;
    }
    case 1: {
        btn1.setStateFromBlynk(false);
        break;
    }
    case 2: {
        btn2.setStateFromBlynk(false);
        break;
    }
    }
}

void setup()
{
    u8x8.begin();
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
    u8x8.clear();
    u8x8.println("Initializing");
    dev0.onTimerExpire(turnoff);
    dev1.onTimerExpire(turnoff);
    dev2.onTimerExpire(turnoff);
    btn0.setCallbackOnStateUpdate(update0);
    btn1.setCallbackOnStateUpdate(update1);
    btn2.setCallbackOnStateUpdate(update2);

    timerBocchiTask1.setInterval(10L, BocchiTask1);
    timerStatusUpdateTask.setInterval(1000L, updateStatus);
    timerBocchiDevice.setInterval(1000L, BocchiDevice);
    // put your setup code here, to run once:
    Serial.begin(9600);
    Serial.println("asdad");
    u8x8.print(".");
    if (latchPin.lock(1) || clockPin.lock(1) || dataPin.lock(1))
    {
        pinMode(latchPin.pin, OUTPUT);
        pinMode(clockPin.pin, OUTPUT);
        pinMode(dataPin.pin, OUTPUT);

        clockPin.unlock();
        dataPin.unlock();

        digitalWrite(latchPin.pin, LOW);
        SPI.begin();
        SPI.beginTransaction(spi_settings);
        SPI.transfer(0b11111111);
        SPI.endTransaction();
        SPI.end();

        digitalWrite(latchPin.pin, HIGH);
        latchPin.unlock();
    };
    u8x8.println(".");
    // btn0.setCallbackOnStateUpdate(update0);
    // btn1.setCallbackOnStateUpdate(update1);
    // btn2.setCallbackOnStateUpdate(update2);
    u8x8.println("Connecting WIFI");
    u8x8.println(WIFI_SSID);
    Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
    Blynk.virtualWrite(V1, 0);
    Blynk.virtualWrite(V2, 0);
    Blynk.virtualWrite(V3, 0);
    Blynk.virtualWrite(V4, 0);
    u8x8.clear();
}

void loop()
{
    Blynk.run();
    timerBocchiTask1.run();
    timerBocchiDevice.run();
    timerStatusUpdateTask.run();
}

void updatePattern(int pattern)
{
    digitalWrite(latchPin.pin, LOW);
    SPI.begin();
    SPI.beginTransaction(spi_settings);
    SPI.transfer(pattern);
    SPI.endTransaction();
    SPI.end();
    digitalWrite(latchPin.pin, HIGH);
}

void BocchiTask1()
{
    pinMode(latchPin.pin, OUTPUT);
    pinMode(clockPin.pin, OUTPUT);
    pinMode(dataPin.pin, OUTPUT);
    static State state = State::PINREAD;
    static bool buttonValue[4] = {0, 0, 0, 0};
    switch (state)
    {
    case (State::PINREAD): {
        int bitloop;

        for (bitloop = 0b00000000; bitloop <= 0b00000011; bitloop++)
        {
            pinMode(D0, INPUT_PULLDOWN_16);
            pinMode(S0Pin.pin, OUTPUT);
            pinMode(S1Pin.pin, OUTPUT);
            digitalWrite(D0, LOW);

            digitalWrite(S0Pin.pin, bitRead(bitloop, 0));
            digitalWrite(S1Pin.pin, bitRead(bitloop, 1));
            // digitalWrite(S0Pin.pin, LOW);
            // digitalWrite(S1Pin.pin, LOW);
            buttonValue[bitloop] = digitalRead(D0);
        }

        state = State::PINWRITE;
        break;
    }
    case (State::PINWRITE): {
        char result[64];
        sprintf(result, "btn %i %i %i %i", buttonValue[0], buttonValue[1], buttonValue[2], buttonValue[3]);
        btn0.setraw(buttonValue[0]);
        btn1.setraw(buttonValue[1]);
        btn2.setraw(buttonValue[2]);
        //   Serial.println(result);

        int btn00 = 0b00000100 & (btn0.getState() * 0xff);
        int btn11 = 0b00000010 & (btn1.getState() * 0xff);
        int btn22 = 0b00000001 & (btn2.getState() * 0xff);

        int pattern = btn00 | btn11 | btn22;
        // Serial.print(pattern, BIN);
        // Serial.println();
        updatePattern(pattern);

        state = State::PINREAD;
        break;
    }
    }
}
int Selected_device;
int Selected_timer;
bool Selected_manual;
void BocchiDevice()
{
    int i = 0;

    for (i = 0; i < bocchi_devcount; i++)
    {
        Serial.printf("%i : %i (%i s) %i\n", i, devices[i]->isTimerOn(), devices[i]->timer, devices[i]->timerRunning);
        if (devices[i]->isTimerOn())
        {

            if (!devices[i]->timerReduce())
            {

                devices[i]->BocchicallbackFunc(i);
                devices[i]->timerStop();
                bool state = devices[Selected_device]->btn->getState();
                if (Selected_device == i && state != Selected_manual)
                {
                    Blynk.virtualWrite(V3, state);
                }
            }
        }
    }
}
BLYNK_WRITE(V1) // button
{
    Serial.printf("sel:%i,val:%i\n", Selected_device, Selected_timer);

    devices[Selected_device]->timer = Selected_timer;
    devices[Selected_device]->timerStart();
    devices[Selected_device]->btn->setStateFromBlynk(true);
    Blynk.virtualWrite(V3, devices[Selected_device]->btn->getState());
    Blynk.virtualWrite(V1, 0);
}

BLYNK_WRITE(V2) // sel device
{
    int value = param.asInt();
    Selected_device = value;
    Blynk.virtualWrite(V3, devices[Selected_device]->btn->getState());
}

BLYNK_WRITE(V3) // manual
{
    int value = param.asInt();
    Selected_manual = value;
    devices[Selected_device]->timerStop();
    devices[Selected_device]->btn->setStateFromBlynk(Selected_manual == 1);
}

BLYNK_WRITE(V4) // sel timer
{
    int value = param.asInt();
    Selected_timer = value;
}
