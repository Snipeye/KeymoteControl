#include <SPI.h>

#define DEMOD_CONTROL A3
#define DEMOD_INPUT A2
#define LAYER_SWITCH A4
#define ADC_ENABLE 7
#define USB_DETECT 8
#define IR_LED_PWM 5
#define BATTERY_IND 13
#define VOLTAGE_DETECT A5
#define CHIP_SELECT LED_BUILTIN_RX

#define C1 10
#define C2 9
#define C3 11

#define R1 3
#define R2 2
#define R3 0
#define R4 1
#define R5 4
#define R6 LED_BUILTIN_TX
#define R7 12
#define R8 6

#define RECORD_SIZE 256 // The number of int8_t dedicated to recording storage.  Each 2 can hold up to 262ms of state
#define PRONTO_SIZE 512 // At 256 bytes/page, and 4 pages/key, that's 1024 bytes (or 512 uint16_t)
#define DEBOUNCE_AMOUNT 10 // how many cycles should we debounce, usually 1ms/cycle
#define STRING_SIZE 768 // max number of chars

#define MEMORY_WRITE_ENABLE 0x06 // This is the command for enabling write in our SPI EEPROM, must be called before a write
#define MEMORY_WRITE 0x02 // This is the command to actually write, followed by address then data
#define MEMORY_READ 0x03 // The command to read, followed by address
#define READ_STATUS_REGISTER 0x05 // This is the command to read the status register to see if it's still writing the old data

#define BUTTON_PRESS_TIMEOUT 10000 // 10 s (recorded in ms) before we go to 'record' mode
#define RECORD_TIMEOUT 20000 // 20 s (recorded in ms) before we assume it's errored
#define RECORD_STATE_TIMEOUT 200000 // .2s in microseconds, longer than this = it's done recording.

#define MIN_VOLTAGE (1023/5*3) // If the ADC measures the battery level beneath this, it will be considered "low battery"

// This typedef/enum is used when deciding what type of ir code we're dealing with.
enum irType {
  IR_SONY,
  IR_NEC,
  IR_UNKNOWN,
  IR_ERROR
};

// These are the rows/cols pins for the matrix
const uint8_t rows[8] = {R1, R2, R3, R4, R5, R6, R7, R8};
const uint8_t cols[3] = {C1, C2, C3};

// This array is sued by the scanMatrix debounce algorithm to efficiently "pre-bounce" keys
static volatile uint32_t debounce[DEBOUNCE_AMOUNT];

// Used in scanMatrix debounce and referenced later, we don't want to sleep if there's any bouncing
static volatile uint32_t anyBounce = 0;

// Populated by the scanMatrix, we can reference this to determine what keys are actually pressed
static volatile uint32_t buttonState = 0;

// This is used to cycle through the debounce array for efficiency
static volatile uint8_t debouncePosition = 0;

// This stores the pronto codes, uses 1024 byes. Pushing max RAM size, since we only have 2560 bytes.
static volatile int16_t hexBurst[PRONTO_SIZE];

// When recording an IR code, we can put raw values here.  Only 256 bytes, so we can increase if necessary
static volatile int8_t recordedData[RECORD_SIZE];

// This is used to record where the 'maximum' of the useful (real) data in the recordedData is.
static volatile uint16_t maxIndex;

// Used in the ISR to make sure the main loop calls the matrix scan function
static volatile bool needToScanMatrix = false;

// This tracks whether we're plugged in or not, and need to activate the USB peripheral stuff
static volatile bool usbActive = false;

// This tracks what key is currently pressed so we can monitor it (we only really handle one at a time)
static volatile uint8_t keyPressed = 0;
static volatile uint8_t layerPressed = 0;

// This tracks how long the button has been pressed
static volatile uint32_t keyTimer = 0;

// This is just to track if we've tried recording or not yet
static volatile bool triedRecording = false;

// This flag is high whenever the interrupt comes from the watchdog timer
static volatile bool fromWatchdog = false;

// When the matrix is disabled, we'll have a flag so we know we have to re-enable it.
static volatile bool reenableMatrix = false;

// This flag is used when the main loop detects a battery voltage beneath a threshold
static volatile bool lowBattery = false;

// This flag is used when it's recording so we can check to exit from it if necessary
static volatile bool isRecording = false;

// This flag is used when the LED is in a flash routine that may need to be cut short
static volatile bool isFlashing = false;

// Used to accept inbound messages
static char charString[STRING_SIZE];

// FUNCTION STUBS WITH EXPLANATIONS:
bool dataToPronto(); // This looks through the recordedData and converts it into a pronto code in hexBurst[].
irType determineType(); // Used during dataToPronto, this figures out what (most likely) type of IR code is recorded
int32_t calcDelay(uint16_t &index); // Also used for dataToPronto, this decodes the compact storage for delay times
bool handleSerialCommand(char* data, int len); // Takes the string of data recieved over serial and fulfills the request
bool extractPronto(char* data, uint16_t startIndex, uint16_t len); // Takes a string and decodes the pronto data.
bool recordPronto(bool force); // needs to wait for interrupts and record the pronto code, outputting it to the hexBurst[] array
bool transmitPronto(uint32_t len, bool isRepeat); // uses the IR LED to transmit the pronto in hexBurst for (len) ms.
void serialPrintPronto(); // Takes the pronto currently in the hexBurst[] and prints over serial
bool loadPronto(uint8_t button, uint8_t layer); // Gets (from SPI) the saved pronto for that address, put in hexBurst[]
bool savePronto(uint8_t button, uint8_t layer); // takes the pronto in hexBurst[] and saves via SPI for button/layer
bool writeDataAt(uint32_t address, uint16_t len, bool clearing); // This is just a subroutine for handling EEPROM writing, it's weird.
uint32_t scanMatrix(); // This runs the actual algorithm to determine what buttons are pressed, also debounces
void handleKey(); // Works with the output of scanMatrix to actually handle state changes and such
void lowPowerConfig(); // should be called during setup, disables most everything it can
void disableUSB(); // This is called during setup (sometimes) and will shut off USB peripherals
void enableUSB(); // This is called on wake (sometimes) and will enable USB peripherals/power
void sleep(); // This is called when the main loop has nothing to do; we should sleep most of the time to save power
void wake(); // Whenever an interrupt triggers some action for the main loop, we'll need to reenable peripherals with this
void setPWM(uint16_t freq, uint8_t duty); // configures the IR output pin to have PWM activated with the provided config
uint8_t getLayer(); // This will query the ADC and find out what layer the switch is on.
uint8_t getButton(); // This looks through the buttonState to find the lowest-index pressed button
void disableMatrix(); // This sets all the cols/rows to default (non-power-consuming) state
void watchDog(uint8_t timeLength); // 0 is 'disable,' 1 is 16ms, 2 is twice that, 3 is twice that... turns on WDT for interrupt
void flashLED(uint8_t count, uint16_t onTime, uint16_t offTime); // Flashes LED <count> times
void matrixInterrupts(bool intIsActive); // this will turn on/off the interrupt vectors for the pin change stuff
void matrixWait(); // Puts the matrix into the state to wait for more keypresses
void checkUSB(); // Checks to see if USB has been plugged/unplugged and reacts accordingly
bool compStr(char *comp, char *fixed); // Compares char array comp to fixed, loking for differences
void cleanMatrix(); // Short-circuits the debouncing since we've just spent a long time transmitting.

ISR (PCINT0_vect) { // triggered by state change on PortB
  if (isRecording || isFlashing) {
    scanMatrix();
    if (isFlashing) {
      isFlashing = false;
      matrixInterrupts(false);
    }
  } else {
    needToScanMatrix = true;
    reenableMatrix = true;
    matrixInterrupts(false); // Disconnect PCI from col pins
    wake();
  }
}

ISR (WDT_vect) { // If the WDT triggers an interrupt, this is the handler
  fromWatchdog = true;
  needToScanMatrix = true;
  matrixInterrupts(false);
}

void setup() {
  pinMode(DEMOD_CONTROL, OUTPUT);
  digitalWrite(DEMOD_CONTROL, LOW);
  pinMode(DEMOD_INPUT, INPUT);
  pinMode(LAYER_SWITCH, INPUT_PULLUP);
  pinMode(ADC_ENABLE, OUTPUT);
  digitalWrite(ADC_ENABLE, LOW);
  pinMode(BATTERY_IND, OUTPUT);
  digitalWrite(BATTERY_IND, LOW);
  pinMode(VOLTAGE_DETECT, INPUT);
  pinMode(USB_DETECT, INPUT_PULLUP);
  pinMode(CHIP_SELECT, OUTPUT);
  digitalWrite(CHIP_SELECT, HIGH);
  pinMode(IR_LED_PWM, OUTPUT);
  digitalWrite(IR_LED_PWM, LOW);
  pinMode(A0, INPUT_PULLUP); // Unused, set to low for battery-saving
//  digitalWrite(A0, LOW);
  pinMode(A1, INPUT_PULLUP); // Unused, set to low for battery-saving
//  digitalWrite(A1, LOW);

  for (uint8_t i=0; i<8; i++) {
    pinMode(rows[i], OUTPUT);
    digitalWrite(rows[i], LOW);
  }

  for (uint8_t i=0; i<3; i++) {
    pinMode(cols[i], OUTPUT);
    digitalWrite(cols[i], LOW);
  }

  for (uint16_t i=0; i<PRONTO_SIZE; i++) {
    hexBurst[i] = 0;
  }

  for (uint8_t deb=0; deb<DEBOUNCE_AMOUNT; deb++) {
    debounce[deb] = 0;
  }

  lowPowerConfig();
  if (digitalRead(USB_DETECT)) {
    Serial.begin(115200);
    delay(1000);
    flashLED(3, 75, 75);
    Serial.println(F("Detected USB"));
    usbActive = true;
  } else {
    disableUSB();
    flashLED(2, 75, 75);
    usbActive = false;
  }

  matrixWait();
  matrixInterrupts(true);
}

void loop() {
  // General program flow should look something like this:
  // - On boot, check to see if we're plugged into USB or just running on battery
  // - In either case, set a flag saying the current power status (USB vs not)
  // -- In PCI ISR we should check the power source and activate/de-activate the USB peripherals depending on that flag
  // - If we're in USB mode, we don't go to sleep - instead, the main loop should be waiting for a serial command.
  // - If we're in battery mode, we should sleep all peripherals we can, and wait for a button press
  //checkUSB();
  
  if (needToScanMatrix || anyBounce || keyPressed) {
    scanMatrix();
    if (needToScanMatrix || keyPressed) {
      handleKey();
    }
    if (!lowBattery) {
      delayMicroseconds(500); // need the voltage to stabalize after wakeup, takes 70-80us.  It's so small.
      uint16_t voltage = analogRead(VOLTAGE_DETECT);
      delayMicroseconds(500); // Found that it wasn't really stabilized.  Try again.
      voltage = analogRead(VOLTAGE_DETECT);
      if (voltage < MIN_VOLTAGE) {
        lowBattery = true;
      }
    }
    needToScanMatrix = false;
  }

  if (reenableMatrix) {
    reenableMatrix = false;
    matrixWait();
    matrixInterrupts(true);
  }

  if (!needToScanMatrix && !anyBounce && !keyPressed && !usbActive) {
    matrixInterrupts(true);
    sleep();
  }
  
  if (usbActive) {
    // check to see if we have a pending serial
//    bool readStuff = true;
//    int index = 0;
//    while (readStuff) {
//      while (Serial.available()) {
//        charString[index++] = Serial.read();
//        if (index >= STRING_SIZE-1) {
//          readStuff = false;
//          break;
//        }
//      }
//      delay(100);
//      readStuff = index < STRING_SIZE - 1 ? Serial.available() : false;
//    }
//    charString[index] = '\0';


    int index = 0;
    if (Serial.available()) {
      index = Serial.readBytesUntil('\n', charString, STRING_SIZE-1);
    }
    charString[index] = '\0';
    if (index > 0) {
      if (handleSerialCommand(charString, index)) {
        Serial.println(F("Done!"));
      } else {
        Serial.println(F("Finished with errors."));
      }
    }
  }
  // - In either case, the button press interrupt should trigger the matrix scan
  // - When the matrix scans, we should pick the lowest-index button and set a "pressed" flag indicating so.
  // -- At the same time as setting that flag, we should measure what layer we're on, and start the transmit
  // -- We should ALSO set a timer (from millis()) to make sure we can track how long the button is pressed
  // - Transmit is a blocking operation.  First time we transmit, we should transmit the non-repeat code
  // - In main loop, if we detect a button is pressed, we check how long it's been pressed.
  // -- If longer than 10 seconds, we go into 'record' mode.  Record mode can be cancelled by pressing the button again?
  // -- If 'record' mode lasts longer than 20 seconds, the remote is probably mashed between couch cushions, wasting battery
  // --- We'll need set the row/col pins appropriately, then enter power-save mode.
  // ---- Power-save mode means we shut off basically everything, and wake up periodically (period tbd)
  // ----- On wake-up, we check button status to see if it's back to 0.  One we get back to 0, we can resume normal operation.
  // -- If less than 10 seconds, go back to transmit(), but this time we'll send the repeat code
  // - As soon as the flagged button is no longer pressed, we check to see if any other buttons are pressed.
  // -- If so, we move on to the next-lowest and repeat everything from "determine layer" and "set flag" and such
  // -- If there is NOT another button pressed, we'll probably scan the matrix until bouncing is done, then sleep 
}

// This looks through the recordedData and converts it into a pronto code in hexBurst[].
bool dataToPronto() {
  for (uint16_t i=0; i<PRONTO_SIZE; i++) {
    hexBurst[i] = 0;
  }
  irType decodedType = determineType();
  uint16_t freqVal;
  uint16_t burstPairTwoStart;
  uint16_t index = 0;
  int32_t agc = calcDelay(index); // this will increment index as much as necessary -1
  int32_t agcLow;
  index++; // NOW we're on the index AFTER the agc, which is probably a low period of waiting.
  switch (decodedType) {
    case IR_SONY: {
      freqVal = (1000000/40000)/0.241246; // Sony operates at 40khz
      calcDelay(index); // ignore the low period after the agc
      index++;
      burstPairTwoStart = index; // for sony, we only really have a burst pair 2 (repeating)
      uint32_t maxLen = 5000; // If we encounter something longer than this, we've hit the end of the sequence.
      hexBurst[1] = freqVal;
      hexBurst[2] = 0; // There is no burst pair one, it's all repeat.
      // hexBurst[3] is unknown since sony can have 12, 15, or 20-bit codes.
      hexBurst[4] = 96; // start with 2.4ms agc
      hexBurst[5] = 24; // and go low for .6ms
      uint16_t hexIndex = 6;
      uint32_t totalDelay = 2400+600; // start with the us we've already used
      while (index < maxIndex) {
        int32_t delayVal = calcDelay(index);
        delayVal = abs(delayVal);
        if (!(hexIndex % 2)) {
          // this means it's a 'high' val, so we should see if it's > 1ms.  If so, it's a 1.
          if (delayVal > 1000) {
            hexBurst[hexIndex] = 48;
            totalDelay += 1200;
          } else {
            hexBurst[hexIndex] = 24;
            totalDelay += 600;
          }
        } else {
          // it's a 'low' val, so we can just push it in.
          if (delayVal > maxLen) {
            // we've probably found the end-terminator, so now we know the bit length.
            hexBurst[3] = (hexIndex - 3)/2;
            hexBurst[hexIndex] = ((45000-totalDelay)*40000)/1000000;
            index = maxIndex;
          } else {
            hexBurst[hexIndex] = 24;
            totalDelay += 600; 
          }
        }
        hexIndex++;
        index++;
      }
      return true;
    }
    break;
    case IR_NEC: {
      freqVal = (1000000/38000)/0.241246; // NEC operates at 38khz
      calcDelay(index); // ignore the low period after the agc
      index++;
      hexBurst[1] = freqVal;
      hexBurst[2] = 34; // This is already defined for NEC, 32 bits + initial burst + end burst
      hexBurst[3] = 2; // This is also defined for NEC, burst + end
      hexBurst[4] = 342; // This is 9ms at 38khz
      hexBurst[5] = 171; // This is 4.5ms at 38khz
      uint32_t totalDelay = 9000+4500; // start with those pre-programmed in there.
      for (uint16_t hexIndex = 6; hexIndex < 72; hexIndex++) {
        calcDelay(index); // this is the "on" val but it should be the same every time.
        index++;
        int32_t offVal = calcDelay(index);
        offVal = abs(offVal);
        index++;
        if (offVal > 800) {
          // it's a 1, the off period is too long
          hexBurst[hexIndex++] = 21;
          hexBurst[hexIndex] = 65;
          totalDelay += 2263;
        } else {
          // it's a 0, 
          hexBurst[hexIndex++] = 21;
          hexBurst[hexIndex] = 21;
          totalDelay += 1105;
        }
      }
      totalDelay -= 1710;
      hexBurst[71] = ((110000-totalDelay)*38000)/1000000; // set the proper delay on the last burst pair
      hexBurst[72] = 342; // 9ms header
      hexBurst[73] = 86; // 2.25ms low
      hexBurst[74] = 21; // burst to finish repeat signal
      hexBurst[75] = 3731; // long low delay to wait for next repeat
      return true;
    }
    break;
    case IR_UNKNOWN: {
      freqVal = (1000000/38000)/0.241246; // Assume 38khz, since that's the demodulator we have and it worked.
      int32_t temp = calcDelay(index);
      agcLow = abs(temp); // We'll just take the raw value we measured.  This is our first burst pair.
      index++;
      // Now we need to find a length of time that is reasonable for our 'timeout' - that is, if we find
      // something that long, it's probably a break (for either a repeat or a sequence 2)
      // We'll start by finding the average time of every packet, and the standard deviation
      uint32_t biggestVal = 0;
      uint32_t secondBiggestVal = 0;
      uint32_t total = 0;
      uint16_t valCount = 0;
      uint16_t indexHolder = index;
      index = 0;
      while (index < maxIndex) {
        int32_t realVal = calcDelay(index);
        uint32_t currentVal = abs(realVal);
        if (currentVal > biggestVal) {
          secondBiggestVal = biggestVal;
          biggestVal = currentVal;
        } else if (currentVal == biggestVal || currentVal > secondBiggestVal) {
          secondBiggestVal = currentVal;
        }
        total += currentVal;
        valCount++;
        index++;
      }

      // We're now at the end, so we have the biggest val, second biggest val, and the average.
      uint32_t averageVal = total/valCount;
      // We'll take the average of all 3 and call that a setting point: anything larger is noteworthy.
      uint32_t importantVal = ((averageVal*2 + biggestVal + secondBiggestVal)/4)*0.85;
      index = indexHolder;

      // Here's how we'll handle this: we go from index until we hit something > imporantVal.  That will be a
      // burst sequence.  We'll then keep going until we hit importantVal again.  That's the next burst sequence.
      // If we don't hit importantVal a second time, we'll assume it's a repeat-only sequence.
      // hexBurst[0] is already 0 for 'learned code'
      hexBurst[1] = freqVal;
      // hexBurst[2] is the OTB length, dunno it yet
      // hexBurst[3] is the repeating burst length, dunno it yet
      hexBurst[4] = (agc * 38000)/1000000;
      hexBurst[5] = (agcLow * 38000)/1000000;
      uint16_t hexIndex = 6;
      uint16_t bsEnd = 0;
      while (index < maxIndex) {
        int32_t tempVal = calcDelay(index);
        uint32_t delayVal = abs(tempVal);
        hexBurst[hexIndex] = (uint16_t)(((uint32_t)(delayVal*38000))/1000000);

        if ((hexIndex % 2)) { // this means it's not an even hexValue, so it's a 'low'/wait
          if (delayVal > importantVal) {
            // we've found the end of the burst sequence.  We'll assume it's burst sequence two unless we find another
            // burst sequence ending, which would make this one BS1.
            if (bsEnd > 0) {
              // This means we've already found the end of a bs, so it was bs 1.  We just concluded bs2.
              hexBurst[2] = (bsEnd - 3)/2;
              hexBurst[3] = (hexIndex - bsEnd)/2;
              index = maxIndex; // break out, effectively.
            } else {
              // We haven't found the end of a bs yet, so we'll just roll with this.
              bsEnd = hexIndex;
            }
          }
        }
        hexIndex++;
        index++;
      }
      // OK, we've reached the end.  does BS2 have a value?
      if (hexBurst[3] == 0) {
        // No, we never assigned a value to BS2, which means we must not have ever found the importantVal again.
        //... did we ever find it?
        if (bsEnd > 0) {
          // yes, we did.  Great.  Then we found BS2.
          hexBurst[3] = (bsEnd - 3)/2;
          return true;
        } else {
          // no, we never found it.  That means we failed to decode.  Oh, well. Sorry!
          return false;
        }
      } else {
        // yes, BS2 has an end, which means B1 must have an end, so we did it.  Good job us.

        // Now, there's one last thing to check: are BS1 and BS2 basically the same?
        if (hexBurst[2] == hexBurst[3]) {
          // yeah, there's a chance.
          bool isSame = true;
          for (uint16_t index = 0; index < hexBurst[2]; index++) {
            uint16_t firstIndex = index*2 + 4;
            uint16_t secondIndex = firstIndex + hexBurst[2]*2;
            int16_t diffOn = hexBurst[firstIndex] - hexBurst[secondIndex];
            int16_t diffOff = hexBurst[firstIndex + 1] - hexBurst[secondIndex + 1];
            if (
              diffOn < -2
              || diffOn > 2
              || diffOff < -2
              || diffOff > 2
             ) {
              // tOK, these are NOT the same.
              isSame = false;
              break;
            }
          }
          if (isSame) {
            // wow, they're basically the same.  That means we can pretend BS 1 never happened, and it's all BS2.
            // Since they're the same, we'll swap them easily:
            hexBurst[2] = 0;
          }
        }
        
        return true;
      }
    }
    break;
  }

  return false;
  // Output length is 4 + hexBurst[2]*2 + hexBurst[3]*2.
}

// Used during dataToPronto, this figures out what (most likely) type of IR code is recorded
irType determineType() {
  // first thing to do is look at the length of the AGC burst.  2.4ms = probably sony, 9ms = probably NEC
  uint16_t index = 0;
  int32_t agcBurst = calcDelay(index); // For some reason if you wrap it in the abs() here, it doesn't work right.  Ugh.
  agcBurst = abs(agcBurst);
  index++;
  int32_t agcLow = calcDelay(index); // Same as the agcBurst declaration, don't wrap in abs.  Separate instruction.
  agcLow = abs(agcLow);
  int32_t total = agcBurst + agcLow;
  if (agcBurst < 0) {
    return IR_ERROR; // Somehow the first burst was low... that doesn't make sense.
  }
  
  if (total > 2500 && total < 3500) {
    return IR_SONY;
  } else if (total > 8500 && total < 18500) {
//  } else if (total > 12500 && total < 14500) { // This is what the timing SHOULD be but some remotes (particularly samsung) play fast and loose.
    return IR_NEC;
  } else { // it's not a known protocol?  I guess we'll mark it as "custom."
    return IR_UNKNOWN;
  }
}

// Also used for dataToPronto, this decodes the compact storage for delay times
int32_t calcDelay(uint16_t &index) {
  int32_t valToUse = recordedData[index];
  if (
    ((index+1) < maxIndex)
    && (
      (
        (recordedData[index] < 0) == (recordedData[index+1] < 0)
      ) || (
        recordedData[index+1] == 0
      )
    )
  ) {
    valToUse *= 128;
    valToUse += recordedData[++index];
  }
  valToUse *= 16;
  return valToUse;
}

// Takes the string of data recieved over serial and fulfills the request
bool handleSerialCommand(char* data, int len) {
  // First we should get a command - this will be one of "record", "transmit", "clear", "get" or "put"
  if (compStr(data, "record")) {
    // "record" is the whole thing, we'll save a code from the demodulator and send it over the serial
    Serial.println(F("Recording, please point remote at demodulator and press desired button."));
    if (!recordPronto(true)) {
      Serial.print(F("Pronto record failed."));
      return false;
    }
    serialPrintPronto();
    return true;
  } else if (compStr(data, "transmit")) {
    if (!compStr(data, "transmit ")) {
      Serial.println(F("Improperly formatted command - try 'transmit <time in ms> <pronto code>'"));
      return false;
    }
    // "transmit bleh 0000 xxxx xxxx xxxx..." where 'bleh' is the number of ms to transmit for
    int msEnd = charFind(data, " ", 9);
    if (msEnd == -1) {
      Serial.println(F("Improperly formatted command - try 'transmit <time in ms> <pronto code>'"));
      return false;
    }
    uint32_t transmitMs = strtol(data + 9, NULL, 10); // ends at data + msEnd, but stotol might automatically do it
    if (!extractPronto(data, msEnd+1, (uint16_t)len)) {
      Serial.println(F("Failed to extract pronto code."));
      return false;
    }
    // OK, now we've extracted the pronto code, we can transmit it for as long as necessary
    return transmitPronto(transmitMs, false);
  } else if (compStr(data, "clear")) {
    if (!compStr(data, "clear ")) {
      Serial.println(F("Improperly formatted command - try 'clear <button>_<layer>'"));
      return false;
    }
    if (compStr(data, "clear all")) {
      bool allClear = true;
      for (uint8_t layer = 1; layer <=4; layer++) {
        for (uint8_t button = 1; button <= 24; button++) {
          allClear &= clearButton(button, layer);
        }
      }
      return allClear;
    }
    int buttonEnd = charFind(data, "_", 6);
    if (buttonEnd == -1) {
      Serial.println(F("Improperly formatted command - try 'clear <button>_<layer>'"));
      return false;
    }
    int layerEnd = len;
    uint8_t button = strtol(data + 6,  NULL, 10); // ends at data + buttonEnd, but stotol might automatically do it
    uint8_t layer = strtol(data + buttonEnd + 1, NULL, 10); // ends at data + layerEnd, but stotol might automatically do it
    if (button > 24 || layer > 4 || !button || !layer) {
      Serial.print(F("Invalid button/layer combination: button ("));
      Serial.print(button);
      Serial.print(F(") must be in range 1-24, and layer ("));
      Serial.print(layer);
      Serial.println(F(") must be in range 1-4.  Please fix and try again."));
      return false;
    }
    return clearButton(button, layer);
  } else if (compStr(data, "get")) {
    if (!compStr(data, "get ")) {
      Serial.println(F("Improperly formatted command - try 'get <button>_<layer>'"));
      return false;
    }
    // "get 23_0"
    int buttonEnd = charFind(data, "_", 4);
    if (buttonEnd == -1) {
      Serial.println(F("Improperly formatted command - try 'get <button>_<layer>'"));
      return false;
    }
    int layerEnd = len;
    uint8_t button = strtol(data + 4, NULL, 10); // ends at data + buttonEnd, but stotol might automatically do it
    uint8_t layer = strtol(data + buttonEnd + 1, NULL, 10); // ends at data + layerEnd, but stotol might automatically do it
    if (button > 24 || layer > 4 || !button || !layer) {
      Serial.print(F("Invalid button/layer combination: button ("));
      Serial.print(button);
      Serial.print(F(") must be in range 1-24, and layer ("));
      Serial.print(layer);
      Serial.println(F(") must be in range 1-4.  Please fix and try again."));
      return false;
    }
    if (loadPronto(button, layer)) {
      serialPrintPronto();
      return true;
    } else {
      Serial.print(F("There is no recorded code for button "));
      Serial.print(button);
      Serial.print(F(" on layer "));
      Serial.println(layer);
      return false;
    }
  } else if (compStr(data, "put")) {
    if (!compStr(data, "put ")) {
      Serial.println(F("Improperly formatted command - try 'put <button>_<layer> <pronto code>'"));
      return false;
    }
    // "put 23_0 0000 xxxx xxxx xxxx"
    int buttonEnd = charFind(data, "_", 4);
    if (buttonEnd == -1) {
      Serial.println(F("Improperly formatted command - try 'put <button>_<layer> <pronto code>'"));
      return false;
    }
    int layerEnd = charFind(data, " ", buttonEnd);
    if (layerEnd == -1) {
      Serial.println(F("Improperly formatted command - try 'put <button>_<layer> <pronto code>'"));
      return false;
    }
    uint8_t button = strtol(data + 4, NULL, 10); // ends at data + buttonEnd, but stotol might automatically do it
    uint8_t layer = strtol(data + buttonEnd + 1, NULL, 10); // ends at data + layerEnd, but strtol might automatically do it
    if (button > 24 || layer > 4 || !button || !layer) {
      Serial.print(F("Invalid button/layer combination: button ("));
      Serial.print(button);
      Serial.print(F(") must be in range 1-24, and layer ("));
      Serial.print(layer);
      Serial.println(F(") must be in range 1-4.  Please fix and try again."));
      return false;
    }
    if (!extractPronto(data, layerEnd + 1, (uint16_t)len)) {
      return false;
    }
    return savePronto(button, layer);
  } else {
    Serial.println(F("Command not recognized.  Available commands are 'record', 'transmit', 'clear', 'get', or 'put'"));
    return false;
  }
}

// Takes a string and decodes the pronto data.
bool extractPronto(char* data, uint16_t startIndex, uint16_t len) {
  uint16_t currentIndex = startIndex;
  uint16_t endIndex = (uint16_t)charFind(data, " ", startIndex);
  uint16_t hexIndex = 0;
  while (currentIndex > 0 && endIndex <= len) {
    hexBurst[hexIndex++] = strtol(data + currentIndex, NULL, 16); // ends at data + endIndex, but strtol might automatically do it
    if (hexIndex >= PRONTO_SIZE) {
      Serial.print(F("Maximum pronto length ("));
      Serial.print(PRONTO_SIZE);
      Serial.print(F(" hex values) exceeded, decode failed."));
      return false;
    }
    if (endIndex < len) {
      if (!compStr(data + endIndex, " ")) {
        Serial.print(F("Encountered unexpected character while decoding hex string on character "));
        Serial.print(endIndex);
        Serial.println(F("! Please fix and try again."));
        return false;
      }
    }
    currentIndex = (uint16_t)charFind(data, " ", endIndex) + 1;
    endIndex = (uint16_t)charFind(data, " ", currentIndex);
    if (endIndex > len) {
      endIndex = len;
    }
  }
  return true;
}

bool clearButton(uint8_t button, uint8_t layer) {
  button--;
  layer--;
  uint32_t address = (uint32_t)layer*24*1024 + (uint32_t)button*1024;
  hexBurst[0] = 0;
  return writeDataAt(address, 1, true);
}

// needs to wait for interrupts and record the pronto code, outputting it to the hexBurst[] array
bool recordPronto(bool force) {
  isRecording = true;
  digitalWrite(DEMOD_CONTROL, HIGH);
  delay(1); // Too much instability after switching the input unless I do this, I guess?
  uint32_t startTime = millis();
  bool keepRecording = true;
  bool isActive = true;
  uint8_t recordIndex = 0;
  uint32_t currentTimer;
  uint32_t realTimeInState;
  uint32_t newTimer;

  // We wait for the DEMOD pin to drop low (meaning it's receiving a signal)
  while (PINF & (0x01 << 5)) {
    // Though, if we timeout (longer that 20 seconds) give up
    if (
      (millis() - startTime) > RECORD_TIMEOUT
      || (
        !(buttonState & ((uint32_t)0x00000001 << ((uint32_t)keyPressed - 1)))
        && !force
      )
    ) {
      isRecording = false;
      digitalWrite(DEMOD_CONTROL, LOW);
      return false;
    }
  }

  // OK, the pin went low.  Now we need to record stuff.  Start the timer.
  currentTimer = micros();

  // As long as we have space (or until we hit another reason to stop) we're going to record.
  while (keepRecording) {

    // As long as the pin matches our status, we continue to wait
    while (!(PINF & (0x01 << 5)) == isActive) {
      // We WILL read the timer and see if we've timed out (max .2s in any state)
      newTimer = micros();
      realTimeInState = newTimer - currentTimer;
      if (realTimeInState > RECORD_STATE_TIMEOUT) {
        // we're done
        keepRecording = false;
        break;
      }
    }

    // Now we convert the actual time into our very compact storage method
    uint32_t timeInState = realTimeInState/16;
    currentTimer = newTimer;
    int8_t topVal = timeInState/128;
    int8_t bottomVal = timeInState % 128;
    if (topVal > 0) {
      recordedData[recordIndex++] = isActive ? topVal : -topVal;
    }
    recordedData[recordIndex++] = isActive ? bottomVal : -bottomVal;

    // And if we're at the end of our recording space, we should stop
    if (recordIndex >= (RECORD_SIZE - 2)) {
      keepRecording = false;
    }

    // Otherwise, flip the status bool and keep going.
    isActive = !isActive;
  }
  maxIndex = recordIndex;
  digitalWrite(DEMOD_CONTROL, LOW);
  isRecording = false;
  return dataToPronto();
}

// uses the IR LED to transmit the pronto in hexBurst for (len) ms.
// If len is 0, just transmits the whole burst.
// if "isRepeat" it uses the second burst (if present), otherwise uses the first burst
bool transmitPronto(uint32_t len, bool isRepeat) {
  uint16_t dataLength = hexBurst[2]*2 + hexBurst[3]*2;
  if (!dataLength) {
    return false;
  }
  uint16_t freq = (1000000/hexBurst[1])/0.241246;
  uint16_t cleanFreq = (freq + 250)/1000;
  cleanFreq *= 1000; // This rounds it nicely to the nearest 500, which should help clean it up a bit?
  uint32_t timerStart = millis();
  uint16_t iStart;
  uint16_t iCond;
  uint16_t minLoop;
  bool loopingDone = false;
  setPWM(cleanFreq, 33);
  TCCR3A = (0 << COM3A1) | (0 << COM3A0) | (1 << WGM31) | (0 << WGM30); // 8'b00000010 to disable PWM override of PC6
  while (!loopingDone) {
    if (isRepeat || !hexBurst[2]) {
      // send the second one
      iStart = hexBurst[2]*2 + 4;
      iCond = hexBurst[3]*2 + hexBurst[2]*2 + 4;
      if (!isRepeat && cleanFreq == 40000) { // Sony codes need to be repeated at least 3 times.
        minLoop = 3;
      } else {
        minLoop = 1;
      }
    } else {
      // send the first one
      iStart = 4;
      iCond = hexBurst[2]*2 + 4;
      minLoop = 1;
    }
    for (uint8_t loopCount = 0; loopCount < minLoop; loopCount++) {
      uint32_t lastMicros = micros();
      bool isHigh = true;
      for (uint16_t i=iStart; i<iCond; i++) {
        if (isHigh) {
          TCCR3A = (1 << COM3A1) | (0 << COM3A0) | (1 << WGM31) | (0 << WGM30); // 8'b10000010 to enable PWM override of PC6
        } else {
          TCCR3A = (0 << COM3A1) | (0 << COM3A0) | (1 << WGM31) | (0 << WGM30); // 8'b00000010 to disable PWM override of PC6
        }
        uint32_t microsToDelay = ((uint32_t)hexBurst[i])*1000000/cleanFreq;
        uint32_t waitForMicros = lastMicros + microsToDelay;
        bool wasOverflow = waitForMicros < lastMicros;
        while (wasOverflow && micros() > waitForMicros) {} // busy loop accounting for overflow
        while (micros() < waitForMicros) {} // Do nothing, busy loop
        lastMicros = waitForMicros;
        //delayMicroseconds((((uint32_t)hexBurst[i])*1000000/cleanFreq));
        isHigh = !isHigh;
        if (
          len
          && (millis() - timerStart) >= len
        ) {
          // have to exit the function
          TCCR3A = (0 << COM3A1) | (0 << COM3A0) | (1 << WGM31) | (0 << WGM30); // 8'b00000010 to disable PWM override of PC6
          return true;
        }
      }
      // We've finished a broadcast, turn off the PWM
      TCCR3A = (0 << COM3A1) | (0 << COM3A0) | (1 << WGM31) | (0 << WGM30); // 8'b00000010 to disable PWM override of PC6
    }
    if (len && !isRepeat) {
      isRepeat = true;
    } else if (!len) {
      loopingDone = true;
    }
  }
  return true;
}

// Takes the pronto currently in the hexBurst[] and prints over serial
void serialPrintPronto() {
  char out[4];
  uint16_t printCount = hexBurst[2]*2 + hexBurst[3]*2 + 4;
  for (uint16_t i=0; i<printCount; i++) {
    sprintf(out, "%04X", hexBurst[i]);
    Serial.print(out);
    if ((i+1) != printCount) {
      Serial.print(F(" "));
    }
  }
  Serial.println(F(""));
}

bool writeDataAt(uint32_t address, uint16_t len, bool clearing) {
  // writes LEN data from the hexBurst to the EEPROM
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  bool starting = true;
  for (uint16_t index = 0; index<len; index++) {
    if (starting || !(address & 0xFF)) {
      starting = false;
      delay(1);
      digitalWrite(CHIP_SELECT, HIGH);
      delay(1);
      digitalWrite(CHIP_SELECT, LOW);
      delay(1);
      SPI.transfer(READ_STATUS_REGISTER);
      bool currentlyWriting = true;
      while (currentlyWriting) {
        uint8_t statusReg = SPI.transfer(0x0);
        currentlyWriting = statusReg & 0x01;
      }
      delay(1);
      digitalWrite(CHIP_SELECT, HIGH);
      delay(1);
      digitalWrite(CHIP_SELECT, LOW);
      delay(1);
      SPI.transfer(MEMORY_WRITE_ENABLE);
      delay(1);
      digitalWrite(CHIP_SELECT, HIGH);
      delay(1);
      digitalWrite(CHIP_SELECT, LOW);
      delay(1);
      SPI.transfer(MEMORY_WRITE);
      SPI.transfer((uint8_t)((address >> 16) & 0xFF));
      SPI.transfer((uint8_t)((address >> 8) & 0xFF));
      SPI.transfer((uint8_t)((address >> 0) & 0xFF)); // 24-bit address
    }
    if (!index && !clearing) {
      SPI.transfer((uint8_t)(((uint16_t)1337) >> 8));
      SPI.transfer((uint8_t)(((uint16_t)1337) & 0xFF));
    } else {
      SPI.transfer((uint8_t)(hexBurst[index] >> 8));
      SPI.transfer((uint8_t)(hexBurst[index] & 0xFF));
    }
    address+=2;
  }
  delay(1);
  digitalWrite(CHIP_SELECT, HIGH);
  delay(1);
  digitalWrite(CHIP_SELECT, LOW);
  delay(1);
  SPI.transfer(READ_STATUS_REGISTER);
  bool currentlyWriting = true;
  while (currentlyWriting) {
    uint8_t statusReg = SPI.transfer(0x0);
    currentlyWriting = statusReg & 0x01;
  }
  delay(1);
  digitalWrite(CHIP_SELECT, HIGH);
  delay(1);
  SPI.endTransaction();
  SPI.end();
  return true;
}

// Gets (from SPI) the saved pronto for that address, put in hexBurst[]
bool loadPronto(uint8_t button, uint8_t layer) {
  // we have 24 buttons/layer, each button has 1024 bytes.
  button--;
  layer--;
  uint32_t address = (uint32_t)24*1024*layer + (uint32_t)1024*button;
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CHIP_SELECT, LOW);
  SPI.transfer(MEMORY_READ);
  SPI.transfer((uint8_t)((address >> 16) & 0xFF));
  SPI.transfer((uint8_t)((address >> 8) & 0xFF));
  SPI.transfer((uint8_t)((address >> 0) & 0xFF)); // 24-bit address
  for (uint16_t i=0; i<4; i++) {
    hexBurst[i] = (uint16_t)(((uint16_t)SPI.transfer(0x0) << 8) | ((uint16_t)SPI.transfer(0x0)));
  }
  if (hexBurst[0] == 1337) {
    // that means it's a valid (populated) address.
    hexBurst[0] = 0;
    uint16_t dataLen = hexBurst[2]*2 + hexBurst[3]*2 + 4;
    for (uint16_t i=4; i<dataLen; i++) {
      hexBurst[i] = (uint16_t)(((uint16_t)SPI.transfer(0x0) << 8) | ((uint16_t)SPI.transfer(0x0)));
    }
    SPI.endTransaction();
    digitalWrite(CHIP_SELECT, HIGH);
    SPI.end();
  } else {
    SPI.endTransaction();
    digitalWrite(CHIP_SELECT, HIGH);
    SPI.end();
    return false;
  }
}

// takes the pronto in hexBurst[] and saves via SPI for button/layer
bool savePronto(uint8_t button, uint8_t layer) {
  uint16_t dataLength = hexBurst[2]*2 + hexBurst[3]*2;
  if (!dataLength) {
    // There's... no data to write?
    return false;
  } else {
    dataLength += 4;
  }
  // we have 24 buttons/layer, each button has 1024 bytes.
  button--;
  layer--;
  uint32_t address = (uint32_t)layer*24*1024 + (uint32_t)button*1024;
  return writeDataAt(address, dataLength, false);
}

// This runs the actual algorithm to determine what buttons are pressed, also debounces
uint32_t scanMatrix() {
  uint32_t matrixData = 0;

  PORTB &= 0x1F; // This sets the cols values to LOW
  DDRB |= 0xE0; // This sets the cols pins to OUTPUTs

  //PORTD = 0x00; // This sets every row LOW output
  DDRD = 0x00; // This changes from output to input, so now the pulldowns are doing their job

  for (uint8_t colIndex = 0; colIndex < 3; colIndex++) {
    digitalWrite(cols[colIndex], HIGH);
    //delayMicroseconds(100);
    matrixData |= ((uint32_t)PIND << (colIndex << 3));
    digitalWrite(cols[colIndex], LOW);
  }

  /*
  // We'll loop through each row, setting
  for (uint8_t rowIndex = 0; rowIndex < 8; rowIndex++) {
    // Each pin, in turn, low
    pinMode(rows[rowIndex], OUTPUT);
    digitalWrite(rows[rowIndex], LOW);

    // While the row is low, we'll loop through each column.
    for (uint8_t colIndex = 0; colIndex < 3; colIndex++) {
      // We'll set the column pin to be an input with a pullup.
      pinMode(cols[colIndex], INPUT_PULLUP);
      // If the column pin is LOW when we read it, that means
      // the button connecting the row we're on and the column
      // we're looking at is pressed, so current (from the pullup
      // resistor) is going through the column pin, through the
      // switch/button, and into the LOW OUTPUT sink of the row pin,
      // causing us to read a LOW (ground, 0v) voltage on the col pin.
      if (!digitalRead(cols[colIndex])) {
        // We just need to store a "1" for "active" in the proper place
        // We can calculate that position by taking 8*colIndex + rowIndex
        // (<< 3 is a bit shift, it's a fast way of multiplying.)
        matrixData |= ((uint32_t)0x00000001 << ((colIndex << 3) + rowIndex));
      }
      // Now that we've checked this column, we need to set it back to
      // be a LOW OUTPUT so it doesn't interfere with the reads on other
      // column pins.  Note that I set it to LOW first so I don't drive
      // 1 pin high feeding into another pin that's low.  That'd be bad.
      digitalWrite(cols[colIndex], LOW);
      pinMode(cols[colIndex], OUTPUT);
    }

    // Now we set the row we're on back to high, and move on to the next
    pinMode(rows[rowIndex], INPUT_PULLUP);
  }
  */

  // Now we have our "matrix data", but it's not debounced.  We need to see
  // what buttons are 'allowed' to be pressed right now: if they've changed
  // within the past 5 reads (which should happen ~1ms apart), then it's
  // probably still bouncing.  First we'll calculate the buttons that ARE
  // 'allowed' to change by looking through the 'debounce' array.

  // We're also going to track to see if ANYTHING has bounced for sleep purposes
  anyBounce = 0;
  
  uint32_t allowedButtons = 0xFFFFFFFF; // Start by assuming all are allowed
  for (uint8_t i=1; i<DEBOUNCE_AMOUNT; i++) {
    // we'll make a variable that describes the buttons that changed between
    // the previous (i-1th) state and the current (ith) state
    uint32_t changedState = debounce[i-1] ^ debounce[i];
    // If a bit (button) has changed state, we knock it out of the 'allowed'
    allowedButtons &= ~changedState;

    // And add to the 'anyBounce'
    anyBounce |= changedState;
  }

  // Now we calculate the buttons that have changed from our recorded value
  uint32_t changedButtons = matrixData ^ buttonState;

  // We'll also go ahead and store the 'dirty' state in the debounce array
  debouncePosition++;
  debouncePosition %= DEBOUNCE_AMOUNT;
  debounce[debouncePosition] = matrixData;

  // Now we'll filter the "changedButtons" to only those that are allowed
  changedButtons &= allowedButtons;

  // Finally, we'll modify the current button state by flipping the changed ones
  buttonState ^= changedButtons;

  // Now we'll set the cols back to INPUT_PULLUP,
  DDRB &= ~0xE0;
  PORTB |= ~0x1F;
  // And the rows back to OUTPUT_LOW
  DDRD = 0xFF;
  //PORTD = 0x00;
  
  // And return the debounced buttons.
  return buttonState;
}

void cleanMatrix() {
  for (uint8_t i=0; i<DEBOUNCE_AMOUNT; i++) {
    scanMatrix();
  }
}

// Works with the output of scanMatrix to actually handle state changes and such
void handleKey() {
  if (keyPressed) {
    if (buttonState & ((uint32_t)0x00000001 << ((uint32_t)keyPressed - 1))) {
      // it's still pressed!
      if (!fromWatchdog && !triedRecording) {
        // we have a key that is currently pressed that we're monitoring.  Let's check on it and see if the timer is going.
        if ((millis() - keyTimer) > BUTTON_PRESS_TIMEOUT) {
          // we need to start recording
          flashLED(2, 75, 75);
          matrixInterrupts(true);
          if (recordPronto(false) && savePronto(keyPressed, layerPressed)) {
            matrixInterrupts(false);
            triedRecording = true;
            flashLED(2, 75, 75);
          } else {
            matrixInterrupts(false);
            // We failed to record the pronto, it's probably mashed between couch cushions.
            // turn everything to low and set WDT to wake us up in .5 seconds? Maybe .25
            triedRecording = true;
            flashLED(3, 75, 75);
            if (!usbActive) {
              // NOTE: we only do this (sleeping) if not plugged into USB
              disableMatrix();
              watchDog(5); // 5 = 250ms, 6 = 500ms, etc
            }
          }
        } else {
          // this means we should send the repeat burst
          bool didTransmit = transmitPronto(0, true); // no specific length to transmit for, but it IS a repeat
          if (didTransmit) {
            cleanMatrix();
          }
        }
      } else if (fromWatchdog) {
        // this means we're coming here FROM the WDT, waiting for the keyPressed to be unpressed.
        // ... it's still pressed.  It's PROBABLY debounced, but I don't know for sure.  I'm going to mark it as such.
        if (anyBounce & ((uint32_t)0x00000001 << ((uint32_t)keyPressed - 1))) {
          for (uint8_t i=0; i<DEBOUNCE_AMOUNT; i++) {
            debounce[i] |= ((uint32_t)0x00000001 << ((uint32_t)keyPressed - 1));
          }
          anyBounce &= ~((uint32_t)0x00000001 << ((uint32_t)keyPressed - 1));
        }
        // great, now we go back to sleep and wait for the WDT again.
        fromWatchdog = false;
        disableMatrix();
        sleep();
      } else {
        // this means it's not from the watchdog, but we already tried recording.  In this
        // case, we should just continue on scanning the matrix.  We should only ever get here
        // if we're plugged into USB.
        // Alternatively, we'll hit this if we successfully record. They have to let go of the button.
        if (!usbActive) {
          // NOTE: we only do this (sleeping) if not plugged into USB
          disableMatrix();
          watchDog(5); // 5 = 250ms, 6 = 500ms, etc
        } // just to be on the safe side, we'll go to watchdog mode here to make sure we don't waste power
      }
    } else {
      keyPressed = 0;
      layerPressed = 0;
      triedRecording = false;
      if (fromWatchdog) {
        // turn off watchdog
        watchDog(0);
      }
      handleKey();
    }
  } else {
    // We're starting a NEW key press
    if (buttonState) {
      layerPressed = getLayer();
      keyPressed = getButton();
      keyTimer = millis();
      if (loadPronto(keyPressed, layerPressed)) {
        if (lowBattery) {
          analogWrite(BATTERY_IND, 3); // turns it on
        }
        bool didTransmit = transmitPronto(0, false); // No hard timeout, just go as long as necessary
        if (lowBattery) {
          digitalWrite(BATTERY_IND, LOW); // turns it off
        }
        if (didTransmit) {
          cleanMatrix();
        }
      } else {
        // There wasn't a valid pronto code there, meaning we should go straight to recording.
        keyTimer -= BUTTON_PRESS_TIMEOUT;
        handleKey();
      }
    } else {
      // There aren't any buttons pressed, this probably shouldn't have been called.
      // Compiler should optimize this branch out if there are no instructions here.
    }
  }
}

// should be called during setup, disables most everything it can
void lowPowerConfig() {
  SMCR |= (1 << SM1); // Sets the sleep mode to "deep sleep"
  ACSR &= ~(1 << ACIE); // Disallows interrupts from the analog comparator
  ACSR |= (1 << ACD) | (1 << ACI); // Turns off the analog comparitor
  DIDR1 |= (1 << AIN0D); // Disables analog pin connected to analog comparator

  // Need to be sure to disable the BOD in the fuses.  Think I already did, but not sure.
  // Note that with BOD, ADC, and Analog Comparator off, the bandgap voltage ref is turned off
  // During sleep - that will take 40-70us to start up again, so we can't run an analog comparison
  // for a bit.  Frankly we could probably just delay a ms after boot and be fine.
  // That said, the bandgap only consumes 10uA, so maybe we leave it on?  Hmm...
  // The challenge is that to leave it enabled we would need to leave some other peripheral enabled,
  // which incrases power draw.

  watchDog(0);

  PRR0 |= (1 << PRTWI) | (1 << PRTIM1); // Turn off TWI
  PRR1 |= (1 << PRUSART1); // timer 4 is the const there
}

// This is called during setup (sometimes) and will shut off USB peripherals
void disableUSB() {
  Serial.end();
  PLLCSR &= ~(1 << PLLE); // Disable PLL oscillator, which is used for USB
  USBCON = (1 << FRZCLK); //0; // Shuts of USB
  USBINT = 0;
  UHWCON = 0;
  PRR1 |= (1 << PRUSB); // Disable USB power
}

// This is called on wake (sometimes) and will enable USB peripherals/power
void enableUSB() {
  UHWCON = 1;
  PRR1 &= ~(1 << PRUSB); // Reenable USB power
  PLLCSR |= (1 << PLLE); // turn on PLL oscillator
  USBDevice.attach(); // Re-init USB
  Serial.begin(115200);
}

// This is called when the main loop has nothing to do; we should sleep most of the time to save power
void sleep() {
  ADCSRA &= ~(1 << 7); // disable ADC
  ADCSRA |= (1 << ADIF); // Clears the ADC flag if it's set
  SMCR |= 0x1; // Enables the sleep to happen, doesn't trigger it yet
  PRR1 |= (1 << 4) | (1 << PRTIM3); // Disable power for timer 3 and timer 4
  PRR0 |= (1 << PRTIM0) | (1 << PRADC) | (1 << PRSPI); // Turn off timer 0 and ADC and SPI
  sei();
  __asm__ __volatile__("sleep");
}

// Whenever an interrupt triggers some action for the main loop, we'll need to reenable peripherals with this
void wake() {
  SMCR &= 0xFE; // Disables the sleep from happening so we don't trigger again accidentally
  PRR0 &= ~(1 << PRTIM0) & ~(1 << PRADC) & ~(1 << PRSPI); // reenable timer 0 (used for delays) and ADC and SPI
  ADCSRA |= (1 << 7); // Re-enables the ADC module
  PRR1 &= ~(1 << 4) & ~(1 << PRTIM3); // reenable timer 3 and timer 4.
}

// configures the IR output pin to have PWM activated with the provided config
// duty is passed in an int from 0-100 representing percent, freq is in hz
void setPWM(uint16_t freq, uint8_t duty) {
  uint16_t topVal = (F_CPU/freq - 1);
  uint16_t compVal = (uint16_t)(topVal*duty/100);

  //ICR3H is top 8 bits for TOP, write that first (0) then write ICR3L (lower 8 bits), 209
  ICR3H = topVal >> 8;
  ICR3L = topVal & 0xFF;

  // Set the compare value for the PWM
  OCR3AH = compVal >> 8;
  OCR3AL = compVal & 0xFF;

  // Now we set TCCR3A 7:6 (COM3A1 COM3A0) to 10 for "SET on TOP, CLEAR at MATCH"
  // And 1:0 to 10 as bits 1:0 of WGM3, which needs to be 4'b1110 for Fast PWM with ICR3 as TOP
  TCCR3A = (1 << COM3A1) | (0 << COM3A0) | (1 << WGM31) | (0 << WGM30); // 8'b10000010

  // Finally TCCR3B bits 4:3 are WGM3 bits 3:2
  // WGM3 needs to be 4'b1110 for Fast PWM with ICR3 as TOP
  // TCCR3B bits 2:0 are clock select, need to be 3'b001 for no prescaling
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (0 << CS32) | (0 << CS31) | (1 << CS30); // 8'b00011001
}

// This will query the ADC and find out what layer the switch is on.
uint8_t getLayer() {
  digitalWrite(ADC_ENABLE, HIGH);
  delayMicroseconds(100); // The bandgap voltage is shut off during sleep, and takes 70-80us to stabalize.
  uint16_t analogVal = analogRead(LAYER_SWITCH);
  digitalWrite(ADC_ENABLE, LOW);
  // If it's close to the top, it's layer 1.  Close to the bottom is layer 4.
  // set up some hard breakpoints given the voltages we should read are 1023, 682, 341, and 0.
  if (analogVal > 852) {
    return 1;
  } else if (analogVal > 511) {
    return 2;
  } else if (analogVal > 170) {
    return 3;
  } else {
    return 4;
  }
}

// This looks through the buttonState to find the lowest-index pressed button
uint8_t getButton() {
  for (uint32_t i=1; i<=24; i++) {
    if (buttonState & ((uint32_t)0x00000001 << (i-1))) {
      return (uint8_t)i;
    }
  }
  return 0;
}

// This sets all the cols/rows to default (non-power-consuming) state
void disableMatrix() {
  PORTB &= 0x1F; // This sets the cols values to LOW
  DDRB |= 0xE0; // This sets the cols pins to OUTPUTs
  //PORTD = 0xFF; // This sets every row HIGH output
  DDRD = 0x00; // This changes from output to input, so now it's just the external pulldowns
}

// 0 is 'disable,' 1 is 16ms, 2 is twice that, 3 is twice that... turns on WDT for interrupt
void watchDog(uint8_t timeLength) {
  if (timeLength > 10) {
    timeLength = 10;
  }
  
  uint8_t regVal = 0x0;
  if (timeLength) {
    fromWatchdog = true;
    matrixInterrupts(false);
    regVal = (1 << WDIE);
    regVal |= ((timeLength - 1) & 0x07);
    regVal |= (((timeLength - 1) & 0x80) << 2);
  }

  __asm__ __volatile__("wdr");
  MCUSR &= ~(1<<WDRF); // clear the watchdog flag if it's set
  WDTCSR |= (1 << WDIF) | (1<<WDCE) | (1<<WDE); // timed sequence to disable the WDT, first unlock
  WDTCSR = regVal; // Then set the interrupt + timeout value

  if (!timeLength) {
    fromWatchdog = false;
    reenableMatrix = true;
  } else {
    sleep();
  }
}

// Flashes the led <count> times with a <timeLength> ms delay in between
void flashLED(uint8_t count, uint16_t timeOn, uint16_t timeOff) {
  isFlashing = true;
  matrixInterrupts(true);
  for (uint8_t i=0; i<count; i++) {
    analogWrite(BATTERY_IND, 3);
    uint32_t currentTime = millis();
    uint32_t waitUntil = (currentTime + timeOn);
    bool wasOverflow = waitUntil < currentTime;
    while (wasOverflow && (millis() > waitUntil)) {
      if (!isFlashing) {
        digitalWrite(BATTERY_IND, LOW);
        return;
      }
    }
    while (millis() < waitUntil) {
      if (!isFlashing) {
        digitalWrite(BATTERY_IND, LOW);
        return;
      }
    }
    digitalWrite(BATTERY_IND, LOW);
    currentTime = millis();
    waitUntil = (currentTime + timeOff);
    wasOverflow = waitUntil < currentTime;
    if ((i + 1) < count) {
      while (wasOverflow && millis() > waitUntil) {
        if (!isFlashing) {
          return;
        }
      }
      while (millis() < waitUntil) {
        if (!isFlashing) {
          return;
        }
      }
    }
  }
  isFlashing = false;
  matrixInterrupts(false);
}

// will activate/deactivate the interrupts for the pin change vector
void matrixInterrupts(bool intIsActive) {
  if (intIsActive) {
    PCMSK0 = 0xF0; // Pins 7-5 are mask for the cols, 4 is USB detect
  } else {
    PCMSK0 = 0x10;
  }
  PCICR = 0x1; // interrupt enable
  PCIFR = 0x0; // Clear any flags
}

// This function puts the matrix into the state to wait for more keypresses
void matrixWait() {
  // Now we'll set the cols back to INPUT_PULLUP,
  DDRB &= ~0xE0;
  PORTB |= ~0x1F;
  // And the rows back to OUTPUT_LOW
  DDRD = 0xFF;
  //PORTD = 0x00;
}

// This function checks to see if USB has been plugged/unplugged and reacts accordingly
void checkUSB() {
  if (digitalRead(USB_DETECT) != usbActive) {
    if (usbActive) {
      // USB is currently recorded as plugged in, but we need to change that to 'unplugged.'
      usbActive = false;
      disableUSB();
      if (triedRecording || fromWatchdog) {
        // this means we need to activate the watchdog timer, we're stuck with a key pressed down
        matrixInterrupts(false);
        disableMatrix();
        watchDog(5); // 5 = 250ms, 6 = 500ms
      }
    } else {
      // USB is currently recorded as not plugged, but it just got plugged in
      usbActive = true;
      enableUSB();
      if (triedRecording || fromWatchdog) {
        // This means the watchdog timer is active and the PCI for the matrix are off.
        watchDog(0);
      }
    }
  }
}

bool compStr(char *comp, char *fixed) {
  int index = 0;
  while (comp[index] != '\0' && fixed[index] != '\0') {
    if (comp[index] != fixed[index]) {
      return false;
    }
    index++;
  }
  return true;
}

int charFind(char *comp, char *toFind, int after) {
  int i=0;
  for (i=0; i<after; i++) {
    if (comp[i] == '\0') {
      return -1;
    }
  }
  bool found = false;
  while (!found) {
    int x=0;
    int temp = i;
    while (comp[temp] != toFind[x]) {
      if (comp[i] == '\0') {
        return -1;
      }
      temp++;
      i++;
    }
    while (comp[temp] == toFind[x] && comp[temp] != '\0') {
      temp++;
      x++;
    }
    if (toFind[x] == '\0') {
      return i;
    } else if (comp[temp] == '\0') {
      return -1;
    } else {
      i++;
    }
  }
}
