////////////////////////////////////////////////////
////////////////////////////////////////////////////
// AOA voltage example for OSH 2018
// Christopher Jones 7/9/2018
//
// Ver 1.0
//

#include <DueTimer.h>         // timer lib functions for using DUE timers and callbacks.
#include <stdint.h>
#include <Arduino.h>

// Output serial debug infomation. (comment out the following line to turn off serial debug output)
#define SHOW_SERIAL_DEBUG 
#define LOOP_DELAY_MILLISECONDS 100  // apply a small delay betwen voltage samples.

// set Freq of callback function used to create the pulse tones.
#define FREQ_OF_FUNCTION      100

// AOA values & Tone Pulse Per Sec (PPS) 
// these are voltage values (reads 0 to 1023 as the analog input on the arduino)
#define HIGH_TONE_STALL_PPS   20      // how many PPS to play during stall
#define HIGH_TONE_AOA_STALL   920     // voltage (and above) where stall happens.  (0 to 1023)
#define HIGH_TONE_AOA_START   775      // voltage (and above) where high tone starts (0 to 1023)
#define HIGH_TONE_PPS_MAX     6.5     // 6.5   
#define HIGH_TONE_PPS_MIN     1.5     // 1.5
#define HIGH_TONE_HZ          1600    // freq of high tone
//#define HIGH_TONE2_HZ         1500    // a 2nd high tone that it will cycle between (if defined)
#define LOW_TONE_AOA_SOLID    695      // voltage (and above) where a solid low tone is played.  (0 to 1023)
#define LOW_TONE_AOA_START    400      // voltage (and above) where low beeps (0 to 1023)
#define LOW_TONE_PPS_MAX      6.5
#define LOW_TONE_PPS_MIN      0.5
#define LOW_TONE_HZ           400     // freq of low tone

#define ANALOG_IN_PIN         3
#define TONE_PIN              2     // TIOA0
#define PIN_LED1              13    // internal LED for showing AOA status.
#define PIN_LED2              54    // aka A0. external LED for showing serial input.
#define PULSE_TONE            1
#define SOLID_TONE            2
#define TONE_OFF              3
#define STARTUP_TONES_DELAY   120

uint8_t toneState = false;
unsigned char toneMode = PULSE_TONE;  // current mode of tone.  PULSE_TONE, SOLID_TONE, or TONE_OFF
boolean highTone = false;             // are we playing high tone or low tone?
uint32_t toneFreq = 0;                // store current freq of tone playing.
float pps = 0;                        // store current PPS of tone (used for debuging) 
int AOA = 0;                          // avaraged AOA value is stored here.
int lastAOA = 0;                      // save last AOA value here.

static Tc *chTC = TC0;
static uint32_t chNo = 0;

// vars for converting AOA scale value to PPS scale value.
int OldRange,  OldValue;
float NewRange, NewValue;

void setup() {
  // put your setup code here, to run once:
  pinMode(TONE_PIN, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(ANALOG_IN_PIN, INPUT);

  Serial.begin(115200);   //Init hardware serial port (ouput to computer for debug)
  configureToneTimer();   //setup timer used for tone

  digitalWrite(PIN_LED2, 1);
  setFrequencytone(400);
  delay(STARTUP_TONES_DELAY);
  setFrequencytone(600);
  delay(STARTUP_TONES_DELAY);
  setFrequencytone(800);
  delay(STARTUP_TONES_DELAY);
  setFrequencytone(1000);
  delay(STARTUP_TONES_DELAY);
  setFrequencytone(1200);
  delay(STARTUP_TONES_DELAY);
  setFrequencytone(1000);
  delay(STARTUP_TONES_DELAY);
  setFrequencytone(1200);
  delay(STARTUP_TONES_DELAY);
  setFrequencytone(1000);
  delay(STARTUP_TONES_DELAY);
  setFrequencytone(1200);
  delay(STARTUP_TONES_DELAY);
  setFrequencytone(0);
  digitalWrite(PIN_LED2, 0);
  
  // timer callback is used for turning tone on/off.
  Timer4.attachInterrupt(tonePlayHandler);
  Timer4.setFrequency(FREQ_OF_FUNCTION);  // set how often we should run the callback function.
  Timer4.start();

}

// main loop of app
void loop() {
  //read in AOA analog voltage value.
  AOA = analogRead(ANALOG_IN_PIN);  // reads in value  0 to 1023

// if there is a delay then delay for a bit before displaying aoa values.
#ifdef LOOP_DELAY_MILLISECONDS
  delay(LOOP_DELAY_MILLISECONDS);
#endif

  // run the function that checks AOA value and plays the tones.
  checkAOA();
}


// We use our own counter for how often we should pause between tones (pulses per sec PPS)
int cycleCounter = 0;
int cycleCounterResetAt = FREQ_OF_FUNCTION;
uint8_t Tone2FlipFlop = false;
void tonePlayHandler(){
  cycleCounter++;
  // check if our counter has reach the reset mark.  if so flip the tone on/off and start again.
  // cycleCounterResetAt is set by the PPS set function.
  if(cycleCounter >= cycleCounterResetAt) {
    toneState = toneState ^ 1;  // flip tone state
    cycleCounter = 0;
  } else {
    return;
  }
  
  if(toneMode==TONE_OFF) {
    setFrequencytone(0);  // if tone off skip the rest.
    return;
  }
  if(toneMode==SOLID_TONE) {  // check for a solid tone.
#ifdef SHOW_SERIAL_DEBUG    
    Serial.println("SOLID TONE");
#endif    
    setFrequencytone(LOW_TONE_HZ);
    return; // skip the rest
  }

  // cylce tone on/off depending on toneState which is flipped in code above.
  if(toneState) {
     digitalWrite(PIN_LED1, digitalRead(PIN_LED1)^1);  // cycle led on/off
     //sprintf(tempBuf, "handler() AOA:%i ASI:%i tone: %i PPS:%f handlerFreq: %i",AOA,ASI,toneFreq,pps, handlerFreq);
     //Serial.println(tempBuf);
     if(highTone) {
// check if we want 2 different tones for the high tone mode.      
#ifdef HIGH_TONE2_HZ
        Tone2FlipFlop = Tone2FlipFlop ^ 1; 
        if(Tone2FlipFlop)
          setFrequencytone(HIGH_TONE_HZ);
        else
          setFrequencytone(HIGH_TONE2_HZ);
#else
        setFrequencytone(HIGH_TONE_HZ);
#endif
     } else {
        setFrequencytone(LOW_TONE_HZ);
     }

  } else {
    setFrequencytone(0);
  }
}

void checkAOA() {
  
  Serial.print(AOA);  // output to serial 
  
  //if(lastAOA == AOA) {
  //  return; // skip this if the AOA value has not changed.
  //}
  
  // check AOA value and set tone and pauses between tones according to 
  if(AOA >= HIGH_TONE_AOA_STALL) {
    // play 20 pps HIGH tone
    highTone = true;
    setPPSTone(HIGH_TONE_STALL_PPS);
    toneMode = PULSE_TONE;
#ifdef SHOW_SERIAL_DEBUG
  Serial.println(" > HIGH_TONE_AOA_STALL");
#endif

  } else if(AOA >= HIGH_TONE_AOA_START) {
    // play HIGH tone at Pulse Rate 1.5 PPS to 6.2 PPS (depending on AOA value)
    highTone = true;
    OldValue = AOA-(HIGH_TONE_AOA_START-1);
    toneMode = PULSE_TONE;
    // scale number using this. http://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
    OldRange = HIGH_TONE_AOA_STALL - HIGH_TONE_AOA_START;  //20 - 1;  //(OldMax - OldMin)  
    NewRange = HIGH_TONE_PPS_MAX - HIGH_TONE_PPS_MIN; // (NewMax - NewMin)  
    NewValue = (((OldValue - 1) * NewRange) / OldRange) + HIGH_TONE_PPS_MIN; //(((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    setPPSTone(NewValue);
#ifdef SHOW_SERIAL_DEBUG
  Serial.println(" > HIGH_TONE_AOA_START");
#endif
  } else if(AOA >= LOW_TONE_AOA_SOLID) {
    // play a steady LOW tone
    highTone = false;
    toneMode = SOLID_TONE;
#ifdef SHOW_SERIAL_DEBUG    
  Serial.println(" > LOW_TONE_AOA_SOLID");
#endif
  } else if(AOA > LOW_TONE_AOA_START) {
    toneMode = PULSE_TONE;
    highTone = false;
    // play LOW tone at Pulse Rate 1.5 PPS to 8.2 PPS (depending on AOA value)
    // scale number using this. http://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
    OldValue = AOA-LOW_TONE_AOA_START;
    OldRange = LOW_TONE_AOA_SOLID - LOW_TONE_AOA_START;  //40 - 1;  //(OldMax - OldMin)  
    NewRange = LOW_TONE_PPS_MAX - LOW_TONE_PPS_MIN; // (NewMax - NewMin)  
    NewValue = (((OldValue - 1) * NewRange) / OldRange) + LOW_TONE_PPS_MAX; //(((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    setPPSTone(NewValue);
#ifdef SHOW_SERIAL_DEBUG    
  Serial.println(" > LOW_TONE_AOA_START");
#endif
  } else {
    toneMode = TONE_OFF;
#ifdef SHOW_SERIAL_DEBUG    
  Serial.println(" - NO TONE");
#endif
  }

  lastAOA = AOA;
#ifdef SHOW_SERIAL_DEBUG    
  // show serial debug info.
  //sprintf(tempBuf, "AOA:%i Live:%i ASI:%ikts ALT:%i PPS:%f cycleCounterResetAt: %i",AOA, liveAOA, ASI, ALT, pps, cycleCounterResetAt);
  //Serial.println(tempBuf);
#endif
}

void setPPSTone(float newPPS) {
  // set PPS by setting cycleCounterResetAt which is used in the tonePlayHandler()
  cycleCounterResetAt = (1/(newPPS*1.5)) * FREQ_OF_FUNCTION;
  pps = newPPS;  // store pps for debug purposes.
}



void configureToneTimer() {
  // Configure TONE_PIN pin as timer output
  // pmc_enable_periph_clk( ID_PIOB ) ;
  int result = PIO_Configure( PIOB,
            PIO_PERIPH_B,
            PIO_PB25B_TIOA0,
            PIO_DEFAULT);

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
 TC_Configure(chTC, chNo,
         TC_CMR_TCCLKS_TIMER_CLOCK4 |
         TC_CMR_WAVE |         // Waveform mode
         TC_CMR_WAVSEL_UP_RC | // Counter running up and reset when equals to RC
         TC_CMR_ACPA_SET |     // RA compare sets TIOA
         TC_CMR_ACPC_CLEAR );  // RC compare clears TIOA
         
  chTC->TC_CHANNEL[chNo].TC_IER=TC_IER_CPCS;  // RC compare interrupt
  chTC->TC_CHANNEL[chNo].TC_IDR=~TC_IER_CPCS;
  //NVIC_EnableIRQ(TC0_IRQn);
}

const uint32_t volHigh = 50; 
const uint32_t volMed = 10; 
const uint32_t volLow = 1; 

void setFrequencytone(uint32_t frequency)
{
  setFrequencytoneAndVol(frequency,volHigh);
}

void setFrequencytoneAndVol(uint32_t frequency,uint32_t vol)
{
  
  if(frequency < 20 || frequency > 20000) {
    //Serial.print("cancel tone: ");Serial.println(frequency);
    TC_Stop(chTC, chNo);
    toneFreq = frequency;
    return;
  }

  if(toneFreq == frequency) {
    // if the new frequency is the same as the current freq then don't do anything.
    return;
  }
  
  const uint32_t rc = VARIANT_MCK / 128 / frequency; 
  //const uint32_t ra = 50; //rc >> 2; // 50% duty cycle 
  //const uint32_t rb = 50;//ra >> 2; // 20% duty cycle 

  //Serial.print("rc=");Serial.println(rc);
  //Serial.print("ra=");Serial.println(ra);
  //Serial.print("rb=");Serial.println(rb);

  TC_Stop(chTC, chNo);
  TC_SetRC(chTC, chNo, rc);    // set frequency
  TC_SetRA(chTC, chNo, vol);   // duty cycle 
  TC_SetRB(chTC, chNo, vol);    
  TC_Start(chTC, chNo);
  toneFreq = frequency;
  
}

