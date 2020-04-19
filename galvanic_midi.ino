//Uncomment this to enter debug mode, where values are printed to serial rather than MIDI packets. Useful for plotting.
//#define DEBUG_MODE (1)

#include "MIDIUSB.h"

const byte interruptPin = 1; //galvanometer input
const byte knobPin = A2; //potentiometer (knob) input
const byte buttonPin = 2;
const byte ledPin = 13;

enum KnobMode {THRESHOLD, SCALE, CHANNEL};
KnobMode currKnobMode = THRESHOLD;
bool modeChanged = false;

const byte samplesize = 10; //set sample array size
const byte analysize = samplesize - 1;  //trim for analysis array

byte timeout = 0;
int value = 0;
int prevValue = 0;

volatile unsigned long sample_time; //sampling timer
volatile byte sample_idx = 0;
volatile unsigned long samples[samplesize];

unsigned long previousMillis = 0;
unsigned long currentMillis = 1;

float threshold = 1.7;   //2.3;  //change threshold multiplier
float threshMin = 1.61; //scaling threshold min
float threshMax = 3.71; //scaling threshold max
float knobMin = 1;
float knobMax = 1024;

//MIDI note config
const byte polyphony = 5; //above 8 notes may run out of ram
int channel = 1;  //setting channel to 11 or 12 often helps simply computer midi routing setups
int noteMin = 36; //C2  - keyboard note minimum
int noteMax = 96; //C7  - keyboard note maximum

//set scaled values, sorted array, first element scale length
const int scaleCount = 5;
const int scaleLen = 13; //maximum scale length plus 1 for 'used length'
int currScale = 0; //current scale, default Chrom
int note_scales[scaleCount][scaleLen] = {
  {12,1,2,3,4,5,6,7,8,9,10,11,12}, //Chromatic
  {7,1, 3, 5, 6, 8, 10, 12}, //Major
  {7,1, 3, 4, 6, 8, 9, 11}, //DiaMinor
  {7,1, 2, 2, 5, 6, 9, 11}, //Indian
  {7,1, 3, 4, 6, 8, 9, 11} //Minor
};
int root = 0; //initialize for root, pitch shifting

int noteIndex = 0;
typedef struct _MIDImessage { //build structure for Note and Control MIDImessages
  unsigned int type;
  int value;
  int velocity;
  long duration;
  long period;
  int channel;
} MIDImessage;
MIDImessage noteArray[polyphony];
MIDImessage controlMessage; //manage MIDImessage data for Control Message (CV out)
byte controlNumber = 80; //set to mappable control, low values may interfere with other soft synth controls.

//Helper function to map one range onto another
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif
  controlMessage.value = 0;  //begin CV at 0

  pinMode(knobPin, INPUT);
  pinMode(ledPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(buttonPin), button_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin), sensor_isr, RISING);
}

void button_isr()
{
  static unsigned long last_button_time = 0;
 
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (currentMillis - last_button_time > 200) 
  {
    switch (currKnobMode)
    {
      case THRESHOLD:
        currKnobMode = SCALE;
        break;
      case SCALE:
        currKnobMode = CHANNEL;
        break;
      case CHANNEL:
        currKnobMode = THRESHOLD;
        break;
    }
    modeChanged = true;
#ifdef DEBUG_MODE
    Serial.print("Mode: "); Serial.println(currKnobMode);
#endif
  }
  last_button_time = currentMillis;
}

void loop()
{
  currentMillis = millis();   //manage time

  if(sample_idx >= samplesize) {
    analyzeSample();
  }

  if (modeChanged)
  {
    modeChanged = false;
    doModeBlinks(currKnobMode);
  }

  checkKnob();
  checkNotes();
  checkControl();
}

//interrupt timing sample array
void sensor_isr()
{
  if(sample_idx < samplesize) {
    samples[sample_idx] = micros() - sample_time;
    sample_time = samples[sample_idx] + sample_time; //rebuild micros() value w/o recalling
    sample_idx += 1;
  }
}

void analyzeSample()
{
  unsigned long averg = 0;
  unsigned long maxim = 0;
  unsigned long minim = 100000;
  float stdevi = 0;
  unsigned long delta = 0;
  byte change = 0;

  if (sample_idx == samplesize) { //array is full
    unsigned long sampanalysis[analysize];
    for (byte i = 0; i < analysize; i++) {
      //skip first element in the array
      sampanalysis[i] = samples[i+1];  //load analysis table (due to volatile)

      if(sampanalysis[i] > maxim) { maxim = sampanalysis[i]; }
      if(sampanalysis[i] < minim) { minim = sampanalysis[i]; }
      averg += sampanalysis[i];
      stdevi += sampanalysis[i] * sampanalysis[i];  //prep stdevi
    }

    averg = averg / analysize;
    stdevi = sqrt(stdevi / analysize - averg * averg); //calculate stdevu
    if (stdevi < 1) { stdevi = 1.0; } //min stdevi of 1
    delta = maxim - minim; 

#ifdef DEBUG_MODE
      Serial.println(delta);
#endif
    
    //**********perform change detection 
    if (delta > (stdevi * threshold)){
      change = 1;
    }
    //*********

    if(change) { // set note and control vector
      int dur = 150+(map(delta%127,1,127,100,2500)); //length of note
      int ramp = 3 + (dur%100) ; //control slide rate, min 25 (or 3 ;)
      
      //set scaling, root key, note
      int note = map(averg%127,1,127,noteMin,noteMax);  //derive note, min and max note
      note = scaleNote(note, root);  //scale the note

      setNote(note, 100, dur, channel);

      //derive control parameters and set    
      setControl(controlNumber, controlMessage.value, delta%127, ramp); //set the ramp rate for the control
    }
     
    //reset array for next sample
    sample_idx = 0;
  }
}

int scaleSearch(int note) {
  int scalesize = (note_scales[currScale][0]);
  for (byte i = 1; i < scalesize; i++) {
    if (note == note_scales[currScale][i]) { return note; }
    else { if (note < note_scales[currScale][i]) { return note_scales[currScale][i]; } } //highest scale value less than or equal to note
    //otherwise continue search
  }
  //didn't find note and didn't pass note value, uh oh!
  return 6;//give arbitrary value rather than fail
}

int scaleNote(int note, int root) {
  //input note mod 12 for scaling, note/12 octave
  //search array for nearest note, return scaled*octave
  int scaled = note%12;
  int octave = note/12;
  //search entire array and return closest scaled note
  scaled = scaleSearch(scaled);
  scaled = (scaled + (12 * octave)) + root; //apply octave and root
  return scaled;
}

// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).

void MIDINoteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void MIDINoteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

// First parameter is the event type (0x0B = control change).
// Second parameter is the event type, combined with the channel.
// Third parameter is the control number number (0-119).
// Fourth parameter is the control value (0-127).

void MIDIControlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

void setNote(int value, int velocity, long duration, int notechannel)
{
  //find available note in array (velocity = 0);
  for(int i = 0; i < polyphony; i++) {
    if(!noteArray[i].velocity){
      //if velocity is 0, replace note in array
      noteArray[i].type = 0;
      noteArray[i].value = value;
      noteArray[i].velocity = velocity;
      noteArray[i].duration = currentMillis + duration;
      noteArray[i].channel = notechannel;
      
      MIDINoteOn(channel, value, velocity);
      break;
    }
  }
}

void checkKnob()
{
  int val = analogRead(knobPin);

  if (currKnobMode == THRESHOLD)
  {
    threshold = mapfloat(threshold, knobMin, knobMax, threshMin, threshMax);
  }
  else if (currKnobMode == SCALE)
  {
    currScale = map(currScale, knobMin, knobMax, 0, scaleCount);
  }
  else if (currKnobMode == CHANNEL)
  {
    channel = map(channel, knobMin, knobMax, 1, 17);
  }
}

void checkNotes()
{
  //send noteOff for all notes with non-zero velocity and expired duration
  for (int i = 0; i < polyphony; i++) {
    if((noteArray[i].velocity) && (noteArray[i].duration <= currentMillis)) {
      MIDINoteOff(channel, noteArray[i].value, 0);
      noteArray[i].velocity = 0;
    }
  }
}

void setControl(int type, int value, int velocity, long duration)
{
  controlMessage.type = type;
  controlMessage.value = value;
  controlMessage.velocity = velocity;
  controlMessage.period = duration;
  controlMessage.duration = currentMillis + duration; //schedule for update cycle
}

void checkControl()
{
  //need to make this a smooth slide transition, using high precision 
  //distance is current minus goal
  signed int distance =  controlMessage.velocity - controlMessage.value; 
  //if still sliding
  if((distance != 0) && (currentMillis > controlMessage.duration)) {
    controlMessage.duration = currentMillis + controlMessage.period; //extend duration
    if(distance > 0) {
      controlMessage.value += 1;
    }
    else {
      controlMessage.value -= 1;
    }
       
    //send MIDI control message after ramp duration expires, on each increment
    MIDIControlChange(channel, controlMessage.type, controlMessage.value);
  }
}

void doModeBlinks(KnobMode mode)
{
  int blinkLoops = 0;
  
  switch (mode)
  {
    case THRESHOLD:
      blinkLoops = 1;
      break;
    case SCALE:
      blinkLoops = 2;
      break;
    case CHANNEL:
      blinkLoops = 3;
      break;
    default:
      blinkLoops = 0;
  }

#ifdef DEBUG_MODE
    Serial.print("Blinks: "); Serial.println(blinkLoops);
#endif

  for (int i = 0; i < blinkLoops; i++)
  {
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(50);
  }
}
