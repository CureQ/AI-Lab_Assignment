#include <Adafruit_NeoPixel.h>

#define analogInPin 34
#define DELAY 1
#define STRIP_HEART_PIN 33
#define STRIP_HEART_NUMLEDS 38
#define STRIP_CHAIN_PIN 32
#define STRIP_CHAIN_NUMLEDS 46
#define BPM_LOW 50
#define BPM_HIGH 110
#define BPM_TIMEOUT 20000
#define FIRST_PIXEL_HUE -12000

int sensorValue = 0;  // value read from the pot
int smoothValue = 100;
bool beatDetectable = true;
int lastBeat = 0;
int ledValue = 0;
int valHistory[100];
int beats[BPM_HIGH];
int beatCursor = 0;
int lastUpdate = 0;
int timeOld = 0;
int timeDif = 0;
int timeNow = 0;

Adafruit_NeoPixel stripHeart(STRIP_HEART_NUMLEDS, STRIP_HEART_PIN, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel stripChain(STRIP_CHAIN_NUMLEDS, STRIP_CHAIN_PIN, NEO_GRBW + NEO_KHZ800);

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);

  stripHeart.begin();           // INITIALIZE NeoPixel stripHeart object (REQUIRED)
  stripHeart.show();            // Turn OFF all pixels ASAP
  stripHeart.setBrightness(255);
  
  stripChain.begin();           // INITIALIZE NeoPixel stripHeart object (REQUIRED)
  stripChain.show();            // Turn OFF all pixels ASAP
  stripChain.setBrightness(100);
}

void loop() {
  timeNow = millis();
  timeDif = timeNow - timeOld;
  // Serial.print(timeDif);
  // read the analog in value:
  sensorValue = analogRead(analogInPin)*10;
  smoothValue = smoothValue * 0.98 + sensorValue * 0.02;
  bool aboveAll = true;
  bool belowAll = true;
  for(int i = 59; i >= 0; i--){
    valHistory[i+1] = valHistory[i];
    if(smoothValue > valHistory[i+1]){
      belowAll = false;
    }
    else if(smoothValue < valHistory[i+1]){
      aboveAll = false;
    }
  }
  valHistory[0] = smoothValue;

  // Serial.print(", Measurement: ");
   Serial.println(smoothValue);
  // Serial.print(", aboveAll: ");
  // Serial.print(aboveAll);
  // Serial.print(", belowAll: ");
  // Serial.println(belowAll);

  if(aboveAll){
    if(beatDetectable){// && (timeDif > 100)){     // Beat is detected
      beatDetectable = false;
      timeOld = timeNow;
      ledValue = 255;

      beats[beatCursor] = millis();
      if(beatCursor == (BPM_HIGH-1)){
        beatCursor = 0;
      }  
      else{
        beatCursor++;
      }
      updateOutput();
      
    }
  }
  else if (belowAll){
    beatDetectable = true;
  }

  // Display heartbeat in heart
  for(int i=0; i<stripHeart.numPixels(); i++) { 
    stripHeart.setPixelColor(i, ledValue,0,0,ledValue/10);
  }
  stripHeart.show();
  ledValue = ledValue*0.99;
  delay(DELAY);

  if(lastUpdate > 500){ // if a beat isn't detected for a while
    updateOutput();
  }
  else{
    lastUpdate++;
  }
}


void updateOutput(){
  int beatsWithinTimeout = 0;
  int beatTimeOut = millis() - BPM_TIMEOUT;
  for(int i = 0; i < BPM_HIGH; i++){
    if((beats[i] > beatTimeOut) && (beats[i] != 0)){
      beatsWithinTimeout++;
    }
  }
  int bpm = beatsWithinTimeout * 60000 / BPM_TIMEOUT;

  int output = map(bpm, BPM_LOW, BPM_HIGH, 0, STRIP_CHAIN_NUMLEDS/2);

  // Serial.print("BPM: ");
  // Serial.println(bpm);
  // Serial.print(", output: ");
  // Serial.println(output);

  // turn chain leds on or off
  for (int i = 0; i < STRIP_CHAIN_NUMLEDS/2; i++){
    if(i < output){
      int pixelHue = FIRST_PIXEL_HUE + (i * 65536L / (STRIP_CHAIN_NUMLEDS/2));

      stripChain.setPixelColor(i, stripChain.gamma32(stripChain.ColorHSV(pixelHue)));
      stripChain.setPixelColor(STRIP_CHAIN_NUMLEDS - i - 1, stripChain.gamma32(stripChain.ColorHSV(pixelHue)));
    }
    else{
      stripChain.setPixelColor(i, 0,0,0,0);
      stripChain.setPixelColor(STRIP_CHAIN_NUMLEDS - i - 1, 0, 0, 0, 0);
    }
  }
  stripChain.show();
  lastUpdate = 0;
}