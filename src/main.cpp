#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <EEPROM.h>

// P I N O U T S

// ENCODER
#define ENC_DT_PIN 2
#define ENC_CLK_PIN 3
#define ENC_SW_PIN 4

// TEMP/HUMIDITY
#define DHT_PIN 7
#define DHT_DELAY 500000

// LIGHT SENSOR
#define SOLAR_PIN A0
#define SOLAR_DELAY 500000

// FANS
#define FAN1_PIN 5
#define FAN2_PIN 10
#define FAN3_PIN 6
#define FAN4_PIN 9

// DISPLAY
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_TOP 16       // Top yellow area
#define SCREEN_BOTTOM 48    // Bottom blue area

// Display Screens
#define SCRN_TEMP 0
#define SCRN_HUMIDITY 1
#define SCRN_SOLAR 2
#define SCRN_FAN1 3
#define SCRN_FAN2 4
#define SCRN_FAN3 5
#define SCRN_FAN4 6
#define SCRN_POWER 7
#define SCRN_CLICKS 1
#define SCRN_FIRST SCRN_TEMP
#define SCRN_LAST SCRN_POWER


// FAN OPTIONS
#define FAN_OFF 0
#define FAN_ON 1
#define FAN_AUTO 2

// POWER OPTIONS
#define POWER_OFF 0
#define POWER_ON 1
#define POWER_SOLAR 2

#define MAX_SAMPLES 32

#define GUID 27381
#define GUID_ADDR 0
#define TEMPERATURE_ADDR 10
#define HUMIDITY_ADDR 11
#define SOLAR_ADDR 12
#define FAN_1_ADDR 13
#define FAN_2_ADDR 14
#define FAN_3_ADDR 15
#define FAN_4_ADDR 16
#define POWER_ADDR 17

// G L O B A L S

DHT dht(DHT_PIN, DHT11);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int buttonState;            // the current reading from the input pin
int lastButtonState = LOW;  //
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
constexpr unsigned long debounceDelay = 50;

int encLastState;

int setHumidity;
int setTemperature;
float currentHumidity;
float currentTemperature;
int currentTemperatureInt;
int currentHumidityInt;
int lastHumidity;
int lastTemperature;
int temperatureSamples[MAX_SAMPLES];
int temperatureIndex;


int setSolar;
int currentSolar;
int lastSolar;
int solarSamples[MAX_SAMPLES];
int solarIndex;

int fan1Option;
int fan2Option;
int fan3Option;
int fan4Option;

int powerOption;

bool editMode;

int currentScreen;

void beginDisplay();
int getTextWidth(const char* text);
int getTextHeight(const char* text);
int updateEditMode();
int updateEncoder();
void updateDHT();
void displayTitle(const char* title);
void displayFanTitle(int fan);
void displayValues(int lastValue, int currentValue, int setValue);
void displayFanOption(int option);
void updateFans(int fan, int option);
int updateFanOptionForward(int option);
int updateFanOptionBackward(int option);
int updatePowerOptionForward(int option);
int updatePowerOptionBackward(int option);
void displayPowerOption(int option);
void updateSolar();
int average(const int samples[MAX_SAMPLES]);

void setup()
{
    currentScreen = SCRN_TEMP;

    int id;
    EEPROM.get(GUID_ADDR, id);

    // Set defaults if not initialized yet
    if (id != GUID)
    {
        EEPROM.put(0, GUID);
        EEPROM.write(TEMPERATURE_ADDR, 78);
        EEPROM.write(HUMIDITY_ADDR, 85);
        EEPROM.write(SOLAR_ADDR, 30);
        EEPROM.write(FAN_1_ADDR, FAN_AUTO);
        EEPROM.write(FAN_2_ADDR, FAN_AUTO);
        EEPROM.write(FAN_3_ADDR, FAN_AUTO);
        EEPROM.write(FAN_4_ADDR, FAN_AUTO);
        EEPROM.write(POWER_ADDR, POWER_ON);
    }

    setTemperature = min(EEPROM.read(TEMPERATURE_ADDR), 99);
    setHumidity = min(EEPROM.read(HUMIDITY_ADDR), 99);
    setSolar = min(EEPROM.read(SOLAR_ADDR), 99);

    fan1Option = EEPROM.read(FAN_1_ADDR);
    fan1Option = (fan1Option > 2) ? 2 : fan1Option;
    fan2Option = EEPROM.read(FAN_2_ADDR);
    fan2Option = (fan2Option > 2) ? 2 : fan2Option;
    fan3Option = EEPROM.read(FAN_3_ADDR);
    fan3Option = (fan3Option > 2) ? 2 : fan3Option;
    fan4Option = EEPROM.read(FAN_4_ADDR);
    fan4Option = (fan4Option > 2) ? 2 : fan4Option;

    powerOption = EEPROM.read(POWER_ADDR);
    powerOption = (powerOption > 2) ? 2 : powerOption;

    lastTemperature = 0;
    lastHumidity = 0;
    lastSolar = 0;

    editMode = false;

    Serial.begin(9600);

    // Initialize PIN configurations
    pinMode(ENC_CLK_PIN, INPUT);
    pinMode(ENC_DT_PIN, INPUT);
    pinMode(ENC_SW_PIN, INPUT_PULLUP);

    pinMode(FAN1_PIN, OUTPUT);
    pinMode(FAN2_PIN, OUTPUT);
    pinMode(FAN3_PIN, OUTPUT);
    pinMode(FAN4_PIN, OUTPUT);
    digitalWrite(FAN1_PIN, 0);
    digitalWrite(FAN2_PIN, 0);
    digitalWrite(FAN3_PIN, 0);
    digitalWrite(FAN4_PIN, 0);

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
    display.clearDisplay();

    // Read current temp and humidity...
    dht.begin();
    currentHumidity = dht.readHumidity();
    currentTemperature = dht.readTemperature();
    currentTemperatureInt = static_cast<int>(dht.convertCtoF(currentTemperature));
    currentHumidityInt = static_cast<int>(currentHumidity);

    // Build the initial averaging array
    temperatureIndex = 0;
    for (int & temperatureSample : temperatureSamples)
        temperatureSample = currentTemperatureInt;

    // Read current solar...
    currentSolar = static_cast<int>(static_cast<float>(analogRead(SOLAR_PIN)) / 10.23f);

    // Build the initial averaging array
    solarIndex = 0;
    for (int & solarSample : solarSamples)
        solarSample = currentSolar;

    encLastState = digitalRead(ENC_CLK_PIN);
}

void loop()
{
    int reading = updateEditMode();
    int encState = updateEncoder();
    updateDHT();
    updateSolar();

    beginDisplay();

    switch (currentScreen / SCRN_CLICKS)
    {
        case SCRN_TEMP:
        {
            displayTitle("Temperature");
            displayValues(lastTemperature, currentTemperatureInt, setTemperature);
        }
        break;

        case SCRN_HUMIDITY:
        {
            displayTitle("Humidity");
            displayValues(lastHumidity, currentHumidityInt, setHumidity);
        }
        break;

        case SCRN_SOLAR:
        {
            displayTitle("Solar");
            displayValues(lastSolar, currentSolar, setSolar);
        }
        break;

        case SCRN_FAN1:
        {
            displayFanTitle(SCRN_FAN1);
            displayFanOption(fan1Option);
        }
        break;

        case SCRN_FAN2:
        {
            displayFanTitle(SCRN_FAN2);
            displayFanOption(fan2Option);
        }
        break;

        case SCRN_FAN3:
        {
            displayFanTitle(SCRN_FAN3);
            displayFanOption(fan3Option);
        }
        break;

        case SCRN_FAN4:
        {
            displayFanTitle(SCRN_FAN4);
            displayFanOption(fan4Option);
        }
        break;

        case SCRN_POWER:
        {
            displayTitle("Power");
            displayPowerOption(powerOption);
        }
            break;

    }

    updateFans(FAN1_PIN, fan1Option);
    updateFans(FAN2_PIN, fan2Option);
    updateFans(FAN3_PIN, fan3Option);
    updateFans(FAN4_PIN, fan4Option);

    lastButtonState = reading;
    encLastState = encState;

    display.display();
}

void updateFans(int fan, int option)
{
    if ((powerOption == POWER_OFF) || (powerOption == POWER_SOLAR && currentSolar < setSolar))
    {
        digitalWrite(fan, LOW);
        return;
    }

    // Turn each fan on/off based on options set
    if (option == FAN_AUTO)
    {
        if (currentTemperatureInt >= setTemperature || currentHumidityInt >= setHumidity)
            digitalWrite(fan, HIGH);
        else
            digitalWrite(fan, LOW);
    }
    else if (option == FAN_ON)
        digitalWrite(fan, HIGH);
    else
        digitalWrite(fan, LOW);
}

void displayValues(int lastValue, int currentValue, int setValue)
{
    // Set the text size and color for displaying temperature and humidity values
    display.setTextSize(3);
    display.setTextColor(SSD1306_WHITE);

    // Easy way to figure where to center text based on largest numbers
    const char* oneDigits = "0";
    const char* twoDigits = "00";
    const char* threeDigits = "100";
    const char* numText = (lastValue < 10) ? oneDigits : (lastValue < 100) ? twoDigits : threeDigits;

    // Get the text extents
    int width = getTextWidth(numText);
    int height = getTextHeight(numText);

    // Figure where center is positioned
    int x = ((display.width() / 2) - width) / 2;
    int y = ((SCREEN_BOTTOM-height) / 2) + SCREEN_TOP + 1;

    // Display the LEFT value centered
    display.setCursor(x, y);
    display.print(currentValue);

    // Line between two numbers
    display.drawLine(display.width()/2, SCREEN_TOP+1, display.width()/2, display.height()-1, SSD1306_WHITE);

    // Display the RIGHT value centered
    display.setCursor(x + display.width()/2, y);
    display.print(setValue);
}

void displayPowerOption(int option)
{
    // Set the font size and color
    display.setTextSize(3);
    display.setTextColor(SSD1306_WHITE);

    // Determine what word to display based on the option
    const char* autoText = "SOLAR";
    const char* onText = "ON";
    const char* offText = "OFF";
    const char* displayText = (option == POWER_SOLAR) ? autoText : (option == POWER_ON) ? onText : offText;

    // Get the extents of the text for centering
    int width = getTextWidth(displayText);
    int height = getTextHeight(displayText);

    // Center in the display region
    int x = (display.width() - width) / 2;
    int y = ((SCREEN_BOTTOM-height) / 2) + SCREEN_TOP + 1;

    // Display text in the centered position
    display.setCursor(x, y);
    display.print(displayText);
}

void displayFanOption(int option)
{
    // Set the font size and color
    display.setTextSize(3);
    display.setTextColor(SSD1306_WHITE);

    // Determine what word to display based on the option
    const char* autoText = "AUTO";
    const char* onText = "ON";
    const char* offText = "OFF";
    const char* displayText = (option == FAN_AUTO) ? autoText : (option == FAN_ON) ? onText : offText;

    // Get the extents of the text for centering
    int width = getTextWidth(displayText);
    int height = getTextHeight(displayText);

    // Center in the display region
    int x = (display.width() - width) / 2;
    int y = ((SCREEN_BOTTOM-height) / 2) + SCREEN_TOP + 1;

    // Display text in the centered position
    display.setCursor(x, y);
    display.print(displayText);
}

void displayTitle(const char* title)
{
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(4, 4); // For Title
    display.print(title);
}

void displayFanTitle(int fan)
{
    // Create a filled square to represent the fan box
    display.fillRect(3,3,10,10,SSD1306_WHITE);

    // Display the title adding the fan number below
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(16, 4); // For Title
    display.print("Fan ");

    // Clear out a square representing the position of this fan
    if (fan == SCRN_FAN1)
    {
        display.print(1);
        display.fillRect(4,4,4,4,SSD1306_BLACK);
    }
    else if (fan == SCRN_FAN2)
    {
        display.print(2);
        display.fillRect(8, 4, 4, 4, SSD1306_BLACK);
    }
    else if (fan == SCRN_FAN3)
    {
        display.print(3);
        display.fillRect(4, 8, 4, 4, SSD1306_BLACK);
    }
    else
    {
        display.print(4);
        display.fillRect(8, 8, 4, 4, SSD1306_BLACK);
    }
}

int average(const int samples[MAX_SAMPLES])
{
    int sum = 0;
    for (int i = 0; i < MAX_SAMPLES; i++)
        sum += samples[i];
    return sum / MAX_SAMPLES;
}

void updateSolar()
{
    // Update the temperature and humidity readings
    static unsigned long lastReading = 0L;
    unsigned long thisMicros = micros();

    // We only want to update the temperature and humidity when the sensor is ready.
    if (thisMicros - lastReading > SOLAR_DELAY || thisMicros < lastReading)
    {
        currentSolar = static_cast<int>(static_cast<float>(analogRead(SOLAR_PIN)) / 20.46f) * 2;
        solarIndex = (solarIndex + 1 >= MAX_SAMPLES) ? 0 : solarIndex + 1;
        solarSamples[solarIndex] = currentSolar;
        currentSolar = average(solarSamples);

        if (lastSolar != currentSolar)
            lastSolar = currentSolar;

        lastReading = micros();
    }
}

void updateDHT()
{
    // Update the temperature and humidity readings
    static unsigned long lastReading = 0L;
    unsigned long thisMicros = micros();

    // We only want to update the temperature and humidity when the sensor is ready.
    if (thisMicros - lastReading > DHT_DELAY || thisMicros < lastReading)
    {
        currentHumidity = dht.readHumidity();
        currentTemperature = dht.readTemperature();

        if (lastTemperature != static_cast<int>(currentTemperature))
        {
            lastTemperature = static_cast<int>(currentTemperature);
            currentTemperatureInt = static_cast<int>(dht.convertCtoF(currentTemperature));

            temperatureIndex = (temperatureIndex + 1 >= MAX_SAMPLES) ? 0 : temperatureIndex + 1;
            temperatureSamples[temperatureIndex] = currentTemperatureInt;
            currentTemperatureInt = average(temperatureSamples);
        }

        if (lastHumidity != static_cast<int>(currentHumidity))
        {
            lastHumidity = static_cast<int>(currentHumidity);
            currentHumidityInt = static_cast<int>(currentHumidity);
        }

        lastReading = micros();
    }
}

int updateEncoder()
{
    // Check if the encoder dial is being turned in any direction
    int encState = digitalRead(ENC_CLK_PIN);
    if (encState != encLastState)
    {
        // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
        if (digitalRead(ENC_DT_PIN) != encState)
        {
            Serial.println("CW");

            if (!editMode)
            {
                // When not in edit mode, advance to the next screen
                currentScreen++;

                if (currentScreen > SCRN_LAST * SCRN_CLICKS)
                    currentScreen = SCRN_FIRST;
            }
            else
            {
                // In edit mode check which screen and preform forward edit, store the new
                // value in EEPROM for when the unit restarts
                switch (currentScreen / SCRN_CLICKS)
                {
                    case SCRN_TEMP:
                        setTemperature = (setTemperature == 99) ? 99 : setTemperature + 1;
                        EEPROM.write(TEMPERATURE_ADDR, static_cast<uint8_t>(setTemperature));
                        break;
                    case SCRN_HUMIDITY:
                        setHumidity = (setHumidity == 99) ? 99 : setHumidity + 1;
                        EEPROM.write(HUMIDITY_ADDR, static_cast<uint8_t>(setHumidity));
                        break;
                    case SCRN_SOLAR:
                        setSolar = (setSolar == 99) ? 99 : setSolar + 1;
                        EEPROM.write(SOLAR_ADDR, static_cast<uint8_t>(setSolar));
                        break;
                    case SCRN_FAN1:
                        fan1Option = updateFanOptionForward(fan1Option);
                        EEPROM.write(FAN_1_ADDR, static_cast<uint8_t>(fan1Option));
                        break;
                    case SCRN_FAN2:
                        fan2Option = updateFanOptionForward(fan2Option);
                        EEPROM.write(FAN_2_ADDR, static_cast<uint8_t>(fan2Option));
                        break;
                    case SCRN_FAN3:
                        fan3Option = updateFanOptionForward(fan3Option);
                        EEPROM.write(FAN_3_ADDR, static_cast<uint8_t>(fan3Option));
                        break;
                    case SCRN_FAN4:
                        fan4Option =updateFanOptionForward(fan4Option);
                        EEPROM.write(FAN_4_ADDR, static_cast<uint8_t>(fan4Option));
                        break;
                    case SCRN_POWER:
                        powerOption = updatePowerOptionForward(powerOption);
                        EEPROM.write(POWER_ADDR, static_cast<uint8_t>(powerOption));
                        break;
                }
            }
        }
        else
        {
            Serial.println("CCW");

            if (!editMode)
            {
                // When not in edit mode, go to the prior screen
                currentScreen--;

                if (currentScreen < 0)
                    currentScreen = SCRN_LAST;
            }
            else
            {
                // In edit mode check which screen and preform backward edit, store the new
                // value in EEPROM for when the unit restarts
                switch (currentScreen / SCRN_CLICKS)
                {
                    case SCRN_TEMP:
                        setTemperature = (setTemperature <= 1) ? 1 : setTemperature - 1;
                        EEPROM.write(TEMPERATURE_ADDR, static_cast<uint8_t>(setTemperature));
                        break;
                    case SCRN_HUMIDITY:
                        setHumidity = (setHumidity <= 1) ? 1 : setHumidity - 1;
                        EEPROM.write(HUMIDITY_ADDR, static_cast<uint8_t>(setHumidity));
                        break;
                    case SCRN_SOLAR:
                        setSolar = (setSolar <= 1) ? 1 : setSolar - 1;
                        EEPROM.write(SOLAR_ADDR, static_cast<uint8_t>(setSolar));
                        break;
                    case SCRN_FAN1:
                        fan1Option = updateFanOptionBackward(fan1Option);
                        EEPROM.write(FAN_1_ADDR, static_cast<uint8_t>(fan1Option));
                        break;
                    case SCRN_FAN2:
                        fan2Option = updateFanOptionBackward(fan2Option);
                        EEPROM.write(FAN_2_ADDR, static_cast<uint8_t>(fan2Option));
                        break;
                    case SCRN_FAN3:
                        fan3Option = updateFanOptionBackward(fan3Option);
                        EEPROM.write(FAN_3_ADDR, static_cast<uint8_t>(fan3Option));
                        break;
                    case SCRN_FAN4:
                        fan4Option = updateFanOptionBackward(fan4Option);
                        EEPROM.write(FAN_4_ADDR, static_cast<uint8_t>(fan4Option));
                        break;
                    case SCRN_POWER:
                        powerOption = updatePowerOptionBackward(powerOption);
                        EEPROM.write(POWER_ADDR, static_cast<uint8_t>(powerOption));
                        break;
                }
            }
        }
    }

    return encState;
}

int updateFanOptionForward(int option)
{
    // Move the fan options forward: AUTO -> ON -> OFF -> repeat
    if (option == FAN_AUTO)
        return FAN_ON;
    else if (option == FAN_ON)
        return FAN_OFF;
    else
        return FAN_AUTO;
}

int updateFanOptionBackward(int option)
{
    // Move the fan options backward: AUTO -> OFF -> ON -> repeat
    if (option == FAN_AUTO)
        return FAN_OFF;
    else if (option == FAN_OFF)
        return FAN_ON;
    else
        return FAN_AUTO;
}

int updatePowerOptionForward(int option)
{
    // Move the fan options forward: SOLAR -> ON -> OFF -> repeat
    if (option == POWER_SOLAR)
        return POWER_ON;
    else if (option == POWER_ON)
        return POWER_OFF;
    else
        return POWER_SOLAR;
}

int updatePowerOptionBackward(int option)
{
    // Move the fan options backward: SOLAR -> OFF -> ON -> repeat
    if (option == POWER_SOLAR)
        return POWER_OFF;
    else if (option == POWER_OFF)
        return POWER_ON;
    else
        return POWER_SOLAR;
}

int updateEditMode()
{
    // Check for encoder knob push (debouncing), and toggle edit mode on/off
    int reading = digitalRead(ENC_SW_PIN);

    // reset the debouncing timer
    if (reading != lastButtonState)
        lastDebounceTime = millis();

    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        if (reading != buttonState)
        {
            buttonState = reading;

            // Toggle the edit mode
            if (buttonState == LOW)
                editMode = !editMode;

            Serial.println((editMode) ? "EDIT" : "DISPLAY");
        }
    }

    return reading;
}

void beginDisplay()
{
    // Display consistent display items here
    display.clearDisplay();

    // Normal 5x7 font
    display.setTextSize(1);

    // Top yellow rect
    display.drawRect(0,0, display.width()-1, SCREEN_TOP, SSD1306_WHITE);

    // Bottom blue rect.
    display.drawRect(0,SCREEN_TOP+1, display.width()-1, (display.height()-SCREEN_TOP)-1, SSD1306_WHITE);

    if (editMode)
    {
        // Display the "EDIT" symbol when in edit mode
        const int margin = 2;
        const char* editText = "EDIT";
        static int width = getTextWidth(editText) + margin * 2;
        static int height = getTextHeight(editText) + margin * 2;

        int x = display.width()- width - margin * 2;
        int y = (SCREEN_TOP-height) / 2;
        display.fillRoundRect(x, y, width, height, 2, SSD1306_WHITE);

        display.setTextColor(SSD1306_BLACK);
        display.setCursor(x+2, y+2);
        display.print(editText);
    }
}

int getTextWidth(const char* text)
{
    int16_t x, y;
    uint16_t w, h;

    display.getTextBounds(text, 0, 0, &x, &y, &w, &h);

    return static_cast<int>(w);
}

int getTextHeight(const char* text)
{
    int16_t x, y;
    uint16_t w, h;

    display.getTextBounds(text, 0, 0, &x, &y, &w, &h);

    return static_cast<int>(h);
}
