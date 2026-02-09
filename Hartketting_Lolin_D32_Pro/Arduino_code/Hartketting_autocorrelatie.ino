/*
  Hartslagmeting met autocorrelatie (ESP32 + PPG-sensor)
  --------------------------------------------------------
  - Meet de lichtintensiteit via een PPG-sensor op de oorlel
  - Berekent de Beats Per Minute op basis van autocorrelatie van de signaalenvelope
  - Stuurt twee LED-strips aan:
      * Grote hartstrip: Knippert ritmisch op de hartslag
      * Kettingstrip: Laat meer LEDs branden bij een hogere hartslag
  - Geschreven voor Lolin D32 Pro (ESP32)
*/

#include <Adafruit_NeoPixel.h> // Library voor de LED strip objecten

// === Hardware configuratie ===
#define PIN_PPG_SENSOR 34        // Analoge ingang waar de PPG-sensor op is aangesloten
#define PIN_HEART_LED_STRIP 33   // Pin voor LED-strip in het hart
#define PIN_CHAIN_LED_STRIP 32   // Pin voor LED-strip in de ketting

#define X_LEDS_HEART 38          // Aantal LEDs in het hart
#define X_LEDS_CHAIN 48          // Aantal LEDs in de ketting

// LED-strip objecten
Adafruit_NeoPixel LED_Heart(X_LEDS_HEART, PIN_HEART_LED_STRIP, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel LED_Chain(X_LEDS_CHAIN, PIN_CHAIN_LED_STRIP, NEO_GRBW + NEO_KHZ800);

// === Sampling instellingen ===
const int SAMPLE_FREQUENTION = 125; // Aantal Hertz (125 metingen per seconde)
const int SAMPLE_AMOUNT = 256;      // Aantal samples per meting (~2 seconden)

// === Buffers voor signaalverwerking ===
double raw_data[SAMPLE_AMOUNT]; // Ruwe data waardes uit de PPG sensor 
double envelope[SAMPLE_AMOUNT]; // Envelop kijkt naar de langzaam veranderende amplitude over de tijd heen. Dit laat de hoofdperiodiek duidelijker zien

// === Parameters voor autocorrelatie ===
const double MIN_BPM = 40.0;
const double MAX_BPM = 180.0;

// === Hulpparameters ===
const int ENVELOPE_SMOOTH_WINDOW = 5; // Aantal samples voor gladstrijken van de envelope
double average_BPM = 0.0;   // Continue aanpasbaar gemiddelde BPM
int LED_value = 0;

// === Motion detectie ===
const double MOVEMENT_THRESHOLD = 200.0; // Als variantie groter is dan drempelwaarde, dan beweegt persoon en is meting onbetrouwbaar

// === Tijdmeting voor hart-LED ===
unsigned long previous_heartbeat_time = 0;
unsigned long heartbeat_interval_ms = 1000; // beginwaarde (wordt aangepast aan BPM)
bool heart_is_bonzing = false;

// === Variabelen voor LED ketting ===
int last_LED_chain_level = 0;
unsigned long last_chain_update = 0;
const unsigned long CHAIN_UPDATE_INTERVAL = 100; // ms tussen kettingupdates


// === Functie: Ruwe PPG-data inlezen ===
void Read_PPG() {
  unsigned long start_time = micros();
  // Lees X aantal analoge waardes en vul buffer
  for (int i = 0; i < SAMPLE_AMOUNT; i++) {
    raw_data[i] = analogRead(PIN_PPG_SENSOR); // waarde tussen 0 en 4095
    unsigned long doelTijd = start_time + (unsigned long)((i + 1) * (1000000.0 / SAMPLE_FREQUENTION));
    while (micros() < doelTijd); // wacht tot volgende sample
  }
}


// === Functie: Bereken BPM via autocorrelatie van de envelop ===
double Calculate_BPM(bool &reliable, double &std_envelope) {
  // 1. Pas DC-filter toe door gemiddelde te nemen. Dit om omgevingslicht te negeren
  double average = 0;
  for (int i = 0; i < SAMPLE_AMOUNT; i++) average += raw_data[i];
  average /= SAMPLE_AMOUNT;

  // Bereken absolute waarde
  for (int i = 0; i < SAMPLE_AMOUNT; i++)
    envelope[i] = fabs(raw_data[i] - average);

  // 2. Gladmaken van de envelope (moving average) (binnen een ongeveer 5 seconde window)
  for (int i = 0; i < SAMPLE_AMOUNT; i++) {
    double sum = 0;
    int counter = 0;
    for (int w = 0; w < ENVELOPE_SMOOTH_WINDOW; w++) {
      int idx = i - w;
      if (idx < 0) break;
      sum += envelope[idx];
      counter++;
    }
    envelope[i] = sum / counter;
  }

  // 3. Bereken standaardafwijking voor bewegingsdetectie
  double average_envelope = 0;
  for (int i = 0; i < SAMPLE_AMOUNT; i++) average_envelope += envelope[i];
  average_envelope /= SAMPLE_AMOUNT;

  double variance = 0;
  for (int i = 0; i < SAMPLE_AMOUNT; i++)
    variance += pow(envelope[i] - average_envelope, 2);
  variance /= SAMPLE_AMOUNT;
  std_envelope = sqrt(variance);

  bool movement = (std_envelope > MOVEMENT_THRESHOLD);

  // 4. Bereken genormaliseerde autocorrelatie
  // De autocorrelatie is de kruiscorrelatie van het signaal met zichzelf
  // In dit geval wordt de correlatie bekeken van het signaal met een in de tijd verschoven kopie van ditzelfde signaal
  int lag_min = max(2, (int)floor(SAMPLE_FREQUENTION / (MAX_BPM / 60.0)));
  int lag_max = min(SAMPLE_AMOUNT - 2, (int)ceil(SAMPLE_FREQUENTION / (MIN_BPM / 60.0)));

  int best_lag = lag_min;
  double best_correlation = -1.0;

  // Lag is de tijdverschuiving (vertraging)
  for (int lag = lag_min; lag <= lag_max; lag++) {
    double sumAB = 0, sumA2 = 0, sumB2 = 0;
    for (int i = 0; i < SAMPLE_AMOUNT - lag; i++) {
      double a = envelope[i];
      double b = envelope[i + lag];
      sumAB += a * b;
      sumA2 += a * a;
      sumB2 += b * b;
    }
    // Zoekt naar beste autocorrelatie --> beste lag
    double correlation = sumAB / (sqrt(sumA2 * sumB2) + 1e-12);
    if (correlation > best_correlation) {
      best_correlation = correlation;
      best_lag = lag;
    }
  }

  // 5. Zet lag om naar BPM
  double frequention_Hz = (double)SAMPLE_FREQUENTION / best_lag;
  double calculated_BPM = frequention_Hz * 60.0;

  // 6. Betrouwbaarheidscontrole (Geen beweging & Prima correlatie)
  reliable = !movement && (best_correlation > 0.25);

  // 7. Update continue aanpasbaar gemiddelde
  if (calculated_BPM < MIN_BPM || calculated_BPM > MAX_BPM) {
    if (average_BPM <= 0.1)
      average_BPM = constrain(calculated_BPM, MIN_BPM, MAX_BPM);
  } else {
    if (average_BPM <= 0.1) average_BPM = calculated_BPM;
    else {
      average_BPM = 0.83 * average_BPM + 0.17 * calculated_BPM; // Past gemiddelde aan. Deze waardes kunnen worden gefinetuned !!!
      LED_value = 255; // Hartslag gedetecteerd, dus activeer Hart LED strip
    }
  }

  // Bereken hartslag-interval in milliseconden
  heartbeat_interval_ms = (unsigned long)(60000.0 / max(1.0, average_BPM));

  // Seriële output
  Serial.print("BPM: "); Serial.print(calculated_BPM, 2);                     // Berekend met autocorrelarie en beste lag
  Serial.print(" | Gemiddeld BPM: "); Serial.print(average_BPM, 1);           // Gemiddeld BPM. Recentere metingen spelen zwaarder mee
  Serial.print(" | Correlatie: "); Serial.print(best_correlation, 3);         // Beste correlatiecoeëfficiënt geeft aan hoe sterk het signaal zich herhaalt. (Hoe betrouwbaar is de detectie)
  Serial.print(" | std_envelop: "); Serial.print(std_envelope, 2);            // Standaard deviatie van de envelop
  Serial.print(" | Betrouwbaar: "); Serial.println(reliable ? "JA" : "NEE");  // Meting is betrouwbaar wanneer er niet te veel beweging is en er een prima correlatie is

  return calculated_BPM;
}


// === LED Functie: Laat het hart bonzen ===
void Update_LED_Heart() {
  unsigned long now = millis(); // Verkrijg tijd in milliseconden
  static bool second_pulse = false;

  // Wacht tot er een hart klopping wordt gedetecteerd 
  if (now - previous_heartbeat_time >= heartbeat_interval_ms) {
    previous_heartbeat_time = now;
    heart_is_bonzing = true;
    second_pulse = false;
  }

  // Laat hart 2x achter elkaar bonzen als hartslag wordt gedetecteerd
  if (heart_is_bonzing) {
    // Tijd sinds start van bonzing
    unsigned long phase = now - previous_heartbeat_time;

    // Eerste bons (0-120ms), tweede bons (200-320ms)
    if ((phase < 120) || (phase >= 200 && phase < 320)) {
      for (int i = 0; i < X_LEDS_HEART; i++) {
        LED_Heart.setPixelColor(i, LED_Heart.Color(255, 0, 0)); // Fel rood
      }
    } else {
      // Geleidelijke afzwakking
      int fade = map(phase, 0, heartbeat_interval_ms, 255, 0);
      fade = constrain(fade, 0, 255);
      for (int i = 0; i < X_LEDS_HEART; i++) {
        LED_Heart.setPixelColor(i, LED_Heart.Color(fade, 0, 0));
      }
    }

    LED_Heart.show();

    // Stop na één volledige hartslagperiode
    if (phase > heartbeat_interval_ms) {
      heart_is_bonzing = false;
    }
  }
}


// === LED Functie: Laat de ketting oplopen met BPM ===
void Update_LED_Chain() {
  unsigned long now = millis();
  if (now - last_chain_update < CHAIN_UPDATE_INTERVAL) return; // beperk updatefrequentie
  last_chain_update = now;

  // Bereken aantal LEDs dat moet branden op basis van BPM
  int active_leds = map((int)average_BPM, MIN_BPM, MAX_BPM, 0, X_LEDS_CHAIN / 2);
  active_leds = constrain(active_leds, 0, X_LEDS_CHAIN / 2);

  // Maak een vloeiende overgang
  if (active_leds > last_LED_chain_level)
    last_LED_chain_level++;
  else if (active_leds < last_LED_chain_level)
    last_LED_chain_level--;

  // Update ketting: symmetrisch van buiten naar binnen
  for (int i = 0; i < X_LEDS_CHAIN / 2; i++) {
    if (i < last_LED_chain_level) {
      int pixelHue = (i * 65536L / (X_LEDS_CHAIN / 2));
      uint32_t color = LED_Chain.gamma32(LED_Chain.ColorHSV(pixelHue));
      LED_Chain.setPixelColor(i, color);
      LED_Chain.setPixelColor(X_LEDS_CHAIN - i - 1, color);
    } else {
      LED_Chain.setPixelColor(i, 0);
      LED_Chain.setPixelColor(X_LEDS_CHAIN - i - 1, 0);
    }
  }
  LED_Chain.show();
}


void setup() {
  Serial.begin(115200);

  LED_Heart.begin(); // Initialiseer NeoPixel LED strip Heart object
  LED_Heart.setBrightness(0);
  LED_Heart.show();  // Turn OFF all pixels ASAP
  LED_Heart.setBrightness(255);

  LED_Chain.begin(); // Initialiseer NeoPixel LED strip Chain object
  LED_Chain.setBrightness(0);
  LED_Chain.show();
  LED_Chain.setBrightness(100);
  
  Serial.println("\nHartslagdetector met autocorrelatie - gestart");
}


void loop() {
  Read_PPG(); // Lees ~2 seconden aan data in
  bool reliable = false;
  double std_envelop = 0;
  Calculate_BPM(reliable, std_envelop);
  Update_LED_Heart();
  Update_LED_Chain();
}
