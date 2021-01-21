#include <SoftwareSerial.h>
//#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <string.h>

SoftwareSerial *sserial = NULL;
Servo servos[8];
int servo_pins[] = {0, 0, 0, 0, 0, 0, 0, 0};
boolean connected = false;

char* parseparams(char* data, char* sdata[], int numexpected)
{
  char* retval = data;
  numexpected--;
  for (int i = 0; i < numexpected; i++)
  {
    sdata[i] = data;
    data = strchr(data,'%');
    if (NULL == data) return NULL;
    *(data++) = 0; // null terminate param and point to start of next
  }
  // last one is different since data is null terminated already
  sdata[numexpected] = data;
  
  return retval;
}

char* parsevarlenparams(char* data, char* sdata[], int numexpected)
{
  //  data = "8%9%262%4%196%8%196%8%220%4%196%4%None%4%247%4%262%4$!";
  //Serial.print("(int)data = "); Serial.println((int)data);
  char* retval = data;
  numexpected--;

  for (int i = 0; i < numexpected; i++)
  {
    sdata[i] = data;
    data = strchr(data,'%');

    if (NULL == data) return NULL;
    *(data++) = 0; // null terminate param and point to start of next
  }
  
  // last one is different since it is either terminated with null or '%'
  sdata[numexpected] = data;
  char terminator;
  while (true)
  {
    terminator = *(data++);
    if (terminator == 0) break;
    if (terminator == '%')
    {
      retval = data;
      break;
    }
  }
  return retval;
}

void Version(){
  Serial.println("version");
}

uint8_t readCapacitivePin(char* data) {
  int pinToMeasure = atoi(data);
  // readCapacitivePin
  //  Input: Arduino pin number
  //  Output: A number, from 0 to 17 expressing
  //  how much capacitance is on the pin
  //  When you touch the pin, or whatever you have
  //  attached to it, the number will get higher
  //  http://playground.arduino.cc/Code/CapacitiveSensor
  //
  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  byte bitmask;
  port = portOutputRegister(digitalPinToPort(pinToMeasure));
  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
  bitmask = digitalPinToBitMask(pinToMeasure);
  pin = portInputRegister(digitalPinToPort(pinToMeasure));
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  delay(1);
  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
       if (*pin & bitmask) { cycles =  0;}
  else if (*pin & bitmask) { cycles =  1;}
  else if (*pin & bitmask) { cycles =  2;}
  else if (*pin & bitmask) { cycles =  3;}
  else if (*pin & bitmask) { cycles =  4;}
  else if (*pin & bitmask) { cycles =  5;}
  else if (*pin & bitmask) { cycles =  6;}
  else if (*pin & bitmask) { cycles =  7;}
  else if (*pin & bitmask) { cycles =  8;}
  else if (*pin & bitmask) { cycles =  9;}
  else if (*pin & bitmask) { cycles = 10;}
  else if (*pin & bitmask) { cycles = 11;}
  else if (*pin & bitmask) { cycles = 12;}
  else if (*pin & bitmask) { cycles = 13;}
  else if (*pin & bitmask) { cycles = 14;}
  else if (*pin & bitmask) { cycles = 15;}
  else if (*pin & bitmask) { cycles = 16;}

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;

  //return cycles;
  Serial.println(cycles);
}

void Tone(char* data)
{
  // notes of the moledy followed by the duration.
  // a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
  // !!negative numbers are used to represent dotted notes,
  // so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
  int retval=0;
  // params are length, pin, (note, duration){length}
  char* sdata[2];
  char* next = parsevarlenparams(data, sdata, 2);
  if ((NULL == next) || (data == next))
  {
    Serial.println(retval);
    return;
  }
  
  int numnotes = atoi(sdata[0]);
  int pin = atoi(sdata[1]);

  // parse the note|duration tuples and play them
  for (retval = 0; retval < numnotes; retval++)
  {
    next = parsevarlenparams(next, sdata, 2);
    if (NULL == next) break;
    int note = atoi(sdata[0]); // non-integers parse to 0 (such as 'None')
    int duration = atoi(sdata[1]);
    if (duration < 0) // handle dotted notes
    {
      duration = -duration;
      duration += duration >> 1;
    }

    int noteDuration = 1000/duration;
    tone(pin, note, noteDuration);
    int pause = (int)(((long)noteDuration) * 13 / 10); // * 1.30;
    delay(pause);
    noTone(pin);
  }
  Serial.println(retval);
}


void ToneNo(char* data){
  int pin = atoi(data);
  noTone(pin);
} 

void DigitalHandler(int mode, char* data){
    int pin = atoi(data);
    if(mode<=0){ //read
        Serial.println(digitalRead(pin));
    }
    else{
        if(pin <0){
            digitalWrite(-pin,LOW);
        }else{
            digitalWrite(pin,HIGH);
        }
    }
}

void AnalogHandler(int mode, char* data){
     if(mode<=0){ //read
        int pin = atoi(data);
        Serial.println(analogRead(pin));
    }else{
        // b'%aw4%236$!'
        // b'%aw4%129$!'
        // b'%aw4%23$!'
        //String sdata[2];
        char* sdata[2];
        
        //split(sdata,2,data,'%');
        data = strchr(data, '%');
        if (data) // return if not formatted properly
        {
          int pin = atoi(sdata[0]);
          int pv = atoi(sdata[1]);
          analogWrite(pin,pv);
        }
    }
}

void ConfigurePinHandler(char* data){
    int pin = atoi(data);
    if(pin <=0){
        pinMode(-pin,INPUT);
    }else{
        pinMode(pin,OUTPUT);
    }
}


void shiftOutHandler(char* data) {    
    char* sdata[4];
    data = parseparams(data, sdata, 4);
    if (NULL == data) return;
    int dataPin = atoi(sdata[0]);
    int clockPin = atoi(sdata[1]);
    sdata[2]; // bit Order 'M' or 'L'
    byte value = (byte) atoi(sdata[3]);
    if (*sdata[2] == 'M') {
       shiftOut(dataPin, clockPin, MSBFIRST, value);
    }
    else
    {
       shiftOut(dataPin, clockPin, LSBFIRST, value);
    }
}

void shiftInHandler(char* data) {
    char* sdata[3];
    data = parseparams(data, sdata, 3);
    if (NULL == data) return;
    
    int dataPin = atoi(sdata[0]);
    int clockPin = atoi(sdata[1]);
    
    int incoming;
    if (*sdata[2] == 'M') {
       incoming = (int)shiftIn(dataPin, clockPin, MSBFIRST);
    } else {
       incoming = (int)shiftIn(dataPin, clockPin, LSBFIRST);
    }
    Serial.println(incoming);
}

void SS_set(char* data){
  delete sserial;
  char* sdata[3];
  data = parseparams(data, sdata, 3);
  if (NULL == data) return;
  
  int rx_ = atoi(sdata[0]);
  int tx_ = atoi(sdata[1]);
  int baud_ = atoi(sdata[2]);
  sserial = new SoftwareSerial(rx_, tx_);
  sserial->begin(baud_);
  Serial.println("ss OK");
}

void SS_write(char* data) {
 sserial->write(data); 
 Serial.println("ss OK");
}

void SS_read(/*String data*/) {
 char c = sserial->read(); 
 Serial.println(c);
}

void pulseInHandler(char* data){
    int pin = atoi(data);
    long duration;
    if(pin <=0){
          pinMode(-pin, INPUT);
          duration = pulseIn(-pin, LOW);      
    }else{
          pinMode(pin, INPUT);
          duration = pulseIn(pin, HIGH);      
    }
    Serial.println(duration);
}

void pulseInSHandler(char* data){
    int pin = atoi(data);
    long duration;
    if(pin <=0){
          pinMode(-pin, OUTPUT);
          digitalWrite(-pin, HIGH);
          delayMicroseconds(2);
          digitalWrite(-pin, LOW);
          delayMicroseconds(5);
          digitalWrite(-pin, HIGH);
          pinMode(-pin, INPUT);
          duration = pulseIn(-pin, LOW);      
    }else{
          pinMode(pin, OUTPUT);
          digitalWrite(pin, LOW);
          delayMicroseconds(2);
          digitalWrite(pin, HIGH);
          delayMicroseconds(5);
          digitalWrite(pin, LOW);
          pinMode(pin, INPUT);
          duration = pulseIn(pin, HIGH);      
    }
    Serial.println(duration);
}

void SV_add(char* data) {
    char* sdata[3];
    data = parseparams(data, sdata, 3);
    if (NULL == data) return;
    int pin = atoi(sdata[0]);
    int min = atoi(sdata[1]);
    int max = atoi(sdata[2]);
    int pos = -1;
    for (int i = 0; i<8;i++) {
        if (servo_pins[i] == pin) { //reset in place
            servos[pos].detach();
            servos[pos].attach(pin, min, max);
            servo_pins[pos] = pin;
            Serial.println(pos);
            return;
            }
        }
    for (int i = 0; i<8;i++) {
        if (servo_pins[i] == 0) {pos = i;break;} // find spot in servo array
        }
    if (pos == -1) {;} //no array position available!
    else {
        servos[pos].attach(pin, min, max);
        servo_pins[pos] = pin;
        Serial.println(pos);
        }
}

void SV_remove(char* data) {
    int pos = atoi(data);
    servos[pos].detach();
    servo_pins[pos] = 0;
}

void SV_read(char* data) {
    int pos = atoi(data);
    int angle;
    angle = servos[pos].read();
    Serial.println(angle);
}

void SV_write(char* data) {
    char* sdata[2];
    data = parseparams(data, sdata, 2);
    if (NULL == data) return;
    int pos = atoi(sdata[0]);
    int angle = atoi(sdata[1]);
    servos[pos].write(angle);
}

void SV_write_ms(char* data) {
    char* sdata[2];
    data = parseparams(data, sdata, 2);
    if (NULL == data) return;
    int pos = atoi(sdata[0]);
    int uS = atoi(sdata[1]);
    servos[pos].writeMicroseconds(uS);
}

void sizeEEPROM() {
    Serial.println(E2END + 1);
}

void EEPROMHandler(int mode, char* data) {
    char* sdata[2];
    data = parseparams(data, sdata, 2);
    if (NULL == data) return;
    if (mode == 0) {  
        EEPROM.write(atoi(sdata[0]), atoi(sdata[1]));  
    } else {
        Serial.println(EEPROM.read(atoi(sdata[0])));
    }
}

void SerialParser(void) {
  char readChar[64]={0x0};
  int numread = Serial.readBytesUntil(33,readChar,64);
  if (numread <= 0) return;

  // Find the command
  int cmd = 0;
  int i;
  for (i = 0; i<numread;)
  {
    if (readChar[i++] == '%')
    {
      cmd = readChar[i++]<<8 | readChar[i++];
      break;
    }
  }

  if (cmd == 0) return;

  // parse the payload portion of the command
  char* databuf;
  if (readChar[--numread] == '$')
  {
    readChar[numread]=0; // null terminate
    databuf = readChar + i;
  }
  else
  {
    return;
  }
  // example '%dr6$!'
  // example '%dw4$!'
  // example '%pm4$!%dw4$!%dw-4$!%dw4$!%dw-4$!%dw4$!%dw-4$!%dw4$!%dw-4$!'
  // example '%dr6$!'
  // example '%tO8%9%262%4%196%8%196%8%220%4%196%4%None%4%247%4%262%4$!'
  // determine command sent

  switch (cmd)
  {
    case int('dw'):
      DigitalHandler(1, databuf);
      break;
    case int('dr'):
      DigitalHandler(0, databuf);
      break;
    case int('aw'):
      AnalogHandler(1, databuf);
      break;
    case int('ar'):
      AnalogHandler(0, databuf);
      break;
    case int('pm'):
      ConfigurePinHandler(databuf);
      break;
    case int('ps'):
      pulseInSHandler(databuf);
      break;
    case int('pi'):
      pulseInHandler(databuf);
      break;
    case int('ss'):
      SS_set(databuf);
      break;
    case int('sw'):
      SS_write(databuf);
      break;
    case int('sr'):
      SS_read(/*data*/);
      break;
    case int('va'):
      SV_add(databuf);
      break;
    case int('vr'):
      SV_read(databuf);
      break;
    case int('vw'):
      SV_write(databuf);
      break;
    case int('vu'):
      SV_write_ms(databuf);
      break;
    case int('vd'):
      SV_remove(databuf);
      break;
    case int('ve'):
      Version();
      break;
    case int('to'):
      Tone(databuf);
      break;
    case int('tn'):
      ToneNo(databuf);
      break;
    case int('ca'):
      readCapacitivePin(databuf);
      break;
    case int('so'):
      shiftOutHandler(databuf);
      break;
    case int('si'):
      shiftInHandler(databuf);
      break;
    case int('ew'):
      EEPROMHandler(0, databuf);
      break;
    case int('er'):
      EEPROMHandler(1, databuf);
      break;
    case int('sz'):  
      sizeEEPROM();
      break;
    default:
      break;
  }  
}

void setup()  {
  Serial.begin(9600); 
    while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
    //Serial.println("waiting");
  }
}

void loop() {
   SerialParser();
   }
