// Version 1.1:
// Bluetooth-String geändert, er beginnt nun mit dem Start-Byte "A"
// Variablen für Baud-Raten für die serielle Übertragung ergänzt
// Variablen für Häufigkeit der seriellen Übertragung (maximaler Counter-Wert) ergänzt

#include <PID_v1.h>
#include <math.h>

// Zeile auskommentieren, wenn keine Rückmeldung über seriellen Monitor gewünscht
#define DEBUG

// Zeile auskommentieren, wenn keine Kommunikation via Bluetooth gewünscht ist
#define SOFTSERIAL
#ifdef SOFTSERIAL
#define RX 2
#define TX 3
#endif
// Über Bluetooth werden die wichtigsten Daten nach folgender Struktur gesendet:
// A+Spannung+Raumtemperatur+Heizbetttemperatur+Betriebsmodus+Zieltemperatur+Fehlercode
// Temperaturen alle in Grad Celsius
// Betriebsmodi: 0 = normal, 1 = Pause, 3 = Fehler
// Fehlercodes:
// 0 = OK,                  1 = Box offen,  2 = Heizbett zu warm, 3 = Box zu kühl
// -1 = Spannung zu gering  -2 =            -3 = 

#define PIN_IN_Heizbett A1
#define PIN_IN_Raum A2
#define PIN_OUT_Heizbett 3
#define PIN_OUT_Luefter 5
#define PIN_IN_Spannung A7
#define PIN_OUT_LedR 6
#define PIN_OUT_LedG 7
#define PIN_OUT_LedB 8
#define PIN_IN_Photoresistor A0

// Thermistor Messschaltung
double rVor = 1000; // Ohm
double messspannung = 5; // Volt

// Thermistor-Kennwerte
double thermA = 0.00115221;
double thermB = 0.000241147;
double thermC = -2.62255E-8;

// PID-Regler Steuerwerte
double zielTemp = 40; //°C
double Kp=300, Ki=0, Kd=0;
double KpNaheZiel=100, KiNaheZiel=0.5, KdNaheZiel=0;

// Schwellwerte
int schwellePhotoresistor = 800;
double schwelleSpannung = 10; // V

// Baud-Rate für serielle Übertragung
int baudDebug = 9600;
int baudBluetooth = 9600;

// Häufigkeit der seriellen Übertragung
int counterMax = 3000;
int counterBluetoothMax = 1000;

//~~~~~~UNTERHALB DIESER LINIE NICHT BEARBEITEN~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Koffermodus
enum Modus
{
  normal,
  pause,
  error
};
Modus modus = normal;
Modus modusAlt = normal;
bool modusGewechselt = true;

// PID-Regler
double istTemp, regelwert; // °C
PID pid(&istTemp, &regelwert, &zielTemp, Kp, Ki, Kd, P_ON_E, DIRECT);
bool pidAnpassen=false, naheZiel=false;

// Rohdaten Sensoren
int batterieRoh;
int photoresistorRoh;

// Fehlerbehandlung
int fehlercode = 0;

#ifdef DEBUG
int counter = 0;
String debugString;
#endif

#ifdef SOFTSERIAL
#include <SoftwareSerial.h>
SoftwareSerial BluetoothSerial(RX,TX);
int bluetoothCounter = 0;
String bluetoothString;
#endif


void setup() {
  #ifdef DEBUG
  Serial.begin(baudDebug);
  #endif

  #ifdef SOFTSERIAL
  BluetoothSerial.begin(baudBluetooth);
  #endif

  pinMode(PIN_OUT_Heizbett, OUTPUT);
  pinMode(PIN_OUT_Luefter, OUTPUT);
  pinMode(PIN_OUT_LedR, OUTPUT);
  pinMode(PIN_OUT_LedG, OUTPUT);
  pinMode(PIN_OUT_LedB, OUTPUT);

  // PID-Regler
  istTemp = getTemp(PIN_IN_Raum);
  pid.SetMode(AUTOMATIC);

  // Spannungsüberwachung (gemessen wird 1/3 der Gesamtspannung, 1024 == 5 V
  schwelleSpannung = (1024.0/5.0)*(schwelleSpannung/3.0);
}

void loop() {
  // Debug-Ausgaben nur in jedem 10000. Schleifendurchlauf
  #ifdef DEBUG
  counter = (counter==counterMax) ? 0 : ++counter;
  #endif
  // Bluetooth-Ausgaben nur in jedem 10000. Schleifendurchlauf
  #ifdef SOFTSERIAL
  bluetoothCounter = (bluetoothCounter==counterBluetoothMax) ? 0 : ++bluetoothCounter;
  #endif

  // Kofferverhalten abhängig vom Modus
  #ifdef DEBUG
  if (!counter) fehlercode = steuereKoffer(modus, true);
  else fehlercode = steuereKoffer(modus, false);
  #endif
  #ifndef DEBUG
  fehlercode = steuereKoffer(modus, false);
  #endif
  
  // Ermittlung neuer Modus
  batterieRoh = analogRead(PIN_IN_Spannung);
  photoresistorRoh = analogRead(PIN_IN_Photoresistor);

  // TODO Error auch bei falschen Sensorwerten
  // TODO in Funktion verpacken. Fehlercode sollte in einer Lese-FUnktion ermittelt werden u in steuereKoffer nur behandelt
  modusAlt = modus;
  if (batterieRoh<schwelleSpannung) 
  {
    fehlercode = -1;
    modus = error;
  }
  else if (photoresistorRoh<schwellePhotoresistor) 
  {
    fehlercode = 1;
    modus = pause;
  }
  else 
  {
    // Fehlercode aus steuereKoffer
    modus = normal;
  }
  if (modus != modusAlt) modusGewechselt = true;
  
  #ifdef DEBUG
      if (!counter) 
      {
        debugString = String("Modus: ") + modus + String("\n");
        debugString += String("Versorgungsspannung: ") + (5.0/1024.0)*3*batterieRoh + String("\n");
        debugString += String("Sensordaten: Thermistor Heizbett: ") + analogRead(PIN_IN_Heizbett) 
            + String(", Thermistor Raum: ") + analogRead(PIN_IN_Raum)
            + String(", Versorgungsspannung: ") + batterieRoh
            + String(", Photoresistor: ") + photoresistorRoh;
        debugString += String("\n-------------------------\n");
        Serial.println(debugString);
      }
  #endif

  #ifdef SOFTSERIAL
    if (!bluetoothCounter)
    {
      // Unsaubere Fehlerbehandlung:
      bluetoothString = String("A+");
      bluetoothString += (5.0/1024.0)*3*batterieRoh;
      bluetoothString += String("+");
      bluetoothString += getTemp(PIN_IN_Raum);
      bluetoothString += String("+");
      bluetoothString += getTemp(PIN_IN_Heizbett);
      bluetoothString += String("+");
      bluetoothString += modus;
      bluetoothString += String("+");
      bluetoothString += zielTemp;
      bluetoothString += String("+");
      bluetoothString += fehlercode;
      BluetoothSerial.print(bluetoothString);
    }
  #endif
}

double getTemp(uint8_t pin)
{
  int wertPin = analogRead(pin);
  double uTherm = (5.0 / 1024.0) * wertPin;
  double rTherm = rVor * (uTherm / (messspannung - uTherm));
  return pow((thermA + thermB * log(rTherm) - thermC * pow(log(rTherm),3)),(-1)) -273.15;
}

int steuereKoffer(Modus modus, bool debug)
{
  int returnWert = 0;
  switch (modus)
  {
    case normal:
      // Steuerung pausieren, wenn Heizbett zu warm
      if (getTemp(PIN_IN_Heizbett)>50)
      {
        #ifdef DEBUG
          if (debug) 
          {
            debugString = String("Heizbett zu warm: ") + getTemp(PIN_IN_Heizbett) + String(" Grad");
            Serial.println(debugString);
          }
        #endif
        returnWert = 2;
        regelwert = 0;
      }
      // Heizbett regeln
      else
      {
        istTemp = getTemp(PIN_IN_Raum);
        #ifdef DEBUG
          if (debug) 
          {
            debugString = String("Raumtemperatur in Grad: ") + istTemp
              + String(", Solltemperatur in Grad: ") + zielTemp;
            Serial.println(debugString);
          }
        #endif
        pid.Compute();
        #ifdef DEBUG
          if (debug) 
          {
            Serial.print("Neuer Regelwert: ");
            Serial.println(regelwert);
          }
        #endif
      }
      analogWrite(PIN_OUT_Heizbett, regelwert);
      // Falls vorher in einem anderen Modus gewesen
      if (modusGewechselt)
      {
        // Lüfter an
        digitalWrite(PIN_OUT_Luefter, HIGH);
        #ifdef DEBUG
            if (debug) Serial.println("Lüfter angeschaltet");
        #endif
        //PID-Regler an
        pid.SetMode(1);
        modusGewechselt = false;
      }
      // LED-Farbe abhängig von Temperatur
      if (abs(istTemp-zielTemp)<1.0)
      {
        // LED grün
        digitalWrite(PIN_OUT_LedR, LOW);
        digitalWrite(PIN_OUT_LedG, HIGH);
        digitalWrite(PIN_OUT_LedB, LOW);
        // Flag für PID-Anpassung
        if (!naheZiel)
        {
          naheZiel = true;
          pidAnpassen = true;
        }
      }
      else
      {
        returnWert = 3;
        // LED blau
        digitalWrite(PIN_OUT_LedR, LOW);
        digitalWrite(PIN_OUT_LedG, LOW);
        digitalWrite(PIN_OUT_LedB, HIGH);
        // Flag für PID-Anpassung
        if (naheZiel)
        {
          naheZiel = false;
          pidAnpassen = true;
        }
      }
      // PID anpassen, falls nötig
      if (pidAnpassen)
      {
        pidAnpassen = false;
        if (naheZiel) pid.SetTunings(KpNaheZiel, KiNaheZiel, KdNaheZiel);
        else pid.SetTunings(Kp, Ki, Kd);
      }
      break;
    case pause:
      // Falls vorher in einem anderen Modus gewesen
      if (modusGewechselt)
      {
        // LED rot
        digitalWrite(PIN_OUT_LedR, HIGH);
        digitalWrite(PIN_OUT_LedG, LOW);
        digitalWrite(PIN_OUT_LedB, LOW);
        // Lüfter aus
        digitalWrite(PIN_OUT_Luefter, LOW);
        #ifdef DEBUG
            if (debug) Serial.println("Lüfter aus, Box geöffnet");
        #endif
        // PID-Regler aus
        pid.SetMode(0);
        // Heizplatte aus
        analogWrite(PIN_OUT_Heizbett, 0);
        modusGewechselt = false;
      }
      break;
    case error:
      // TODO Pieper
      if (modusGewechselt)
      {
        // LED rot
        digitalWrite(PIN_OUT_LedR, HIGH);
        digitalWrite(PIN_OUT_LedG, LOW);
        digitalWrite(PIN_OUT_LedB, LOW);
        // Lüfter aus
        digitalWrite(PIN_OUT_Luefter, LOW);
        #ifdef DEBUG
            if (debug) Serial.println("Lüfter aus, Versorgungsspannung zu gering");
        #endif
        // PID-Regler aus
        pid.SetMode(0);
        // Heizplatte aus
        analogWrite(PIN_OUT_Heizbett, 0);
        modusGewechselt = false;
      }
      break;
  }
  return returnWert;
}

