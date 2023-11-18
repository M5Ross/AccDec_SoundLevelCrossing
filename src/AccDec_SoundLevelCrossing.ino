// Production 17 Function DCC Decoder 
// Version 5.1  Geoff Bunza 2014,2015,2016
// NO LONGER REQUIRES modified software servo Lib
// Software restructuring mods added from Alex Shepherd and Franz-Peter
//   With sincere thanks

      /*********************************************
       *                                           *
       *  modified by M5Ross => AccDecPLSound      *
       *                                           *
       *********************************************/

// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
#define DECODER_LOADED

#define Accessory_Address    1           // THIS ADDRESS IS THE SWITCH ADDRESS TO CONTROL
                                         // THE OPENING AND CLOSING OF LEVEL PASS

// ******** REMOVE THE "//" IN THE FOOLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR
//#define DEBUG

// ******** REMOVE THE "//" IN THE FOOLOWING LINE TO SEND / RECEIVE
// ******** CONFIGURATION TO THE SERIAL MONITOR
#define SERIALCOM

////////////////////////////////////////////////////////////////////////////////////////////

#define DECODER_ID 7

#define HW_VERSION 10
#define SW_VERSION 31

#include <NmraDcc.h>
#include <Servo.h>

Servo servo[4];

#define OCCUPATO  1
//#define LIBERO  0
#define RICHIEDI_CHIUSURA 2
#define RICHIEDI_APERTURA 3
#define STA_CHIUDENDO 4
#define STA_APRENDO 5

#define LIBERO    0b00
#define DESTRO    0b01
#define SINISTRO  0b10

char occ[] = {'L','D','S'};

byte LampLed[] = {A4,A5};
byte RailSensor[] = {3,4,5,6,7,8,9,10};
byte servoPins[] = {A0,A1,A2,A3};

const int puls_aperto = 11;
const int puls_chiuso = 12;
const int LED = 13;

byte tipoControllo;
byte tipobarriera;
byte velocitaServi;
byte posizioneApertoServi[4]; // in gradi °
byte posizioneChiusoServi[4]; // in gradi °
byte posizioneAttualeServi[4]; // in gradi °
int ritardoChiusuraBarriera[4];
int ritardoAperturaBarriera[4];
byte tipoLuci;
int frequenzaLampeggioLuci;
byte attivazioneSuono;
byte volumeSuono;
byte fileAudio;
int soundStopTimeout;
long closeTimeout;
int sensorTimeout;
byte lowThr;
byte highThr;
byte releseCounter;

byte inputMap[8];

byte STATO = LIBERO;
byte statoDCC = LIBERO;

byte occupazioneBinario[] = {0,0,0,0};
byte sensorState[] = {0,0,0,0,0,0,0,0};
byte last_sensorState[] = {0,0,0,0,0,0,0,0};

unsigned long sensorTime[8];
unsigned long servoTime[4];
unsigned long servoTime2[4];
unsigned long ledTime = 0;
unsigned long LEDTime = 0;
unsigned long closeTime = 0;
unsigned long soundStopTime = 0;

long counter[] = {0,0,0,0,0,0,0,0};
long counter_max[] = {0,0,0,0,0,0,0,0};
byte fine[] = {0,0,0,0};
byte finito = 0;
byte vai[] = {0,0,0,0};
byte occupato;
byte releseCount[] = {0,0,0,0,0,0,0,0};
boolean prima = true;
boolean aggiornacv = true;
boolean sound = false;

NmraDcc  Dcc ;
DCC_MSG  Packet ;

#define SET_CV_Address  99                // THIS ADDRESS IS FOR SETTING CV'S Like a Loco

                                          // WHICH WILL EXTEND FOR 16 MORE SWITCH ADDRESSES
uint8_t CV_DECODER_MASTER_RESET =   120;  // THIS IS THE CV ADDRESS OF THE FULL RESET
#define CV_To_Store_SET_CV_Address	121
#define CV_Accessory_Address CV_ACCESSORY_DECODER_ADDRESS_LSB


struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, Accessory_Address},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},
  {CV_DECODER_MASTER_RESET, 0},
  {CV_To_Store_SET_CV_Address, SET_CV_Address},
  {CV_To_Store_SET_CV_Address+1, 0},

  {30, 1},    // tipo di controllo:
              // 0 = manuale
              // 1 = sensori onboard
              // 2 = DCC
  
  {31, 2},    // tipo di barriera:
              // 0 = nessuna barriera = 0 servo
              // 1 = barriera singola o semibarriere singole = 2 servo
              // 2 = semibarriere doppie = 4 servo
              
  {32, 50},    // velocità servi: 10 = veloce, 50 = lento
  
  {33, 20},   // posizione aperto servo 1
  {34, 100},  // posizione chiuso servo 1
  {35, 20},   // posizione attuale servo 1
  
  {36, 20},   // posizione aperto servo 2
  {37, 100},  // posizione chiuso servo 2
  {38, 20},   // posizione attuale servo 2
  
  {39, 20},   // posizione aperto servo 3
  {40, 100},  // posizione chiuso servo 3
  {41, 20},   // posizione attuale servo 3
  
  {42, 20},   // posizione aperto servo 4
  {43, 100},  // posizione chiuso servo 4
  {44, 20},   // posizione attuale servo 4
  
  {45, 40},   // ritardo chiusura barriera servo 1 (50ms)
  {46, 50},   // ritardo chiusura barriera servo 2 (50ms)
  {47, 80},   // ritardo chiusura barriera servo 3 (50ms)
  {48, 90},   // ritardo chiusura barriera servo 4 (50ms)
  
  {49, 10},   // ritardo apertura barriere servo 1 (50ms)
  {50, 10},   // ritardo apertura barriere servo 2 (50ms)
  {51, 10},   // ritardo apertura barriere servo 3 (50ms)
  {52, 10},   // ritardo apertura barriere servo 4 (50ms)

  {53, 2},    // tipo di luci:
              // 0 = nessuna luce
              // 1 = singola fissa
              // 2 = doppia lampeggiante
              
  {54, 40},   // frequenza lampeggio led (10ms acceso e spento)

  {55, 1},    // suono chiusura barriere:
              // 0 = disattivato
              // 1 = attivo solo durante la chiusura
              // 2 = attivo sempre
              
  {56, 10},   // volume 1-30

  {57, 1},    // scelta file audio:
              // 0 = PL barriere campana lenta
              // 1 = PL semibarriere campana veloce
              // 2 = PL semibarriere doppie campana veloce

  {58, 100},  // ritardo spegnimento suono in modalità 55=1 (50ms)
  {59, 60},   // timeout riaperturta automatica (secondi)
  {60, 30},   // timeout ciclo verifica input (10ms)
  {61, 50},   // low Thr  (%)
  {62, 80},   // high Thr (%)
  {63, 5},    // release counter (n° cicli verifica input consecutivi off)

  //     SSSSDDDD
  //     43214321
  {70, 0b00000001},   // input R1
  {71, 0b00000010},   // input R2
  {72, 0b00000100},   // input R3
  {73, 0b00001000},   // input R4
  {74, 0b00010000},   // input L1
  {75, 0b00100000},   // input L2
  {76, 0b01000000},   // input L3
  {77, 0b10000000},   // input L4
              
  {120, 0},   // 120 = reset decoder
};

uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);

#if defined(SERIALCOM)
  #undef DEBUG
  #include <DccSerialCom.h>
  
  DccSerialCom SerialCom(FactoryDefaultCVIndex);

  void notifyExecuteFunction(uint8_t function, uint8_t state) {
    digitalWrite(LED, 1);
    LEDTime = millis();
    if(state)
      STATO = RICHIEDI_APERTURA;
    else
      STATO = RICHIEDI_CHIUSURA;
  }
  
  uint16_t notifyGetCVnum(uint16_t index) {
    return FactoryDefaultCVs[index].CV;
  }

  uint16_t notifyGetCVval(uint16_t CV) {
    return Dcc.getCV(CV);
  }

  void notifySetCV(uint16_t CV, uint16_t value) {
    Dcc.setCV(CV, value);
  }
#endif

//#include <SoftwareSerial.h>
//#include <DFPlayer_Mini_Mp3.h>
#include <DFRobotDFPlayerMini.h>
DFRobotDFPlayerMini myDFPlayer;
#define FPSerial Serial

//SoftwareSerial SoundSerial(sound_RX, sound_TX); // RX, TX


void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
}

void notifyCVChange(uint16_t CV, uint8_t Value)
{
  aggiornacv = true;
}

void aggiornaCV() {
  if(aggiornacv == true) {
    aggiornacv = false;
    
    digitalWrite(LED, 1);
    LEDTime = millis();
    
    tipoControllo = (int)Dcc.getCV( 30 );
    tipobarriera = (int)Dcc.getCV( 31 );
    velocitaServi = (int)Dcc.getCV( 32 );
  
    for (byte i=0; i < 4; i++) {
      posizioneApertoServi[i] = (int)Dcc.getCV( 33+i*3 ); // in gradi °
      posizioneChiusoServi[i] = (int)Dcc.getCV( 34+i*3 ); // in gradi °
      posizioneAttualeServi[i] = (int)Dcc.getCV( 35+i*3 ); // in gradi °
      ritardoChiusuraBarriera[i] = (int)Dcc.getCV( 45+i ) * 50;
      ritardoAperturaBarriera[i] = (int)Dcc.getCV( 49+i ) * 50;
      // attaches servo on pin to the servo object
      servo[i].attach(servoPins[i]);
      servo[i].write(posizioneAttualeServi[i]);
    }
  
    tipoLuci = (int)Dcc.getCV( 53 );
    frequenzaLampeggioLuci = (int)Dcc.getCV( 54 ) * 10;
    attivazioneSuono = (int)Dcc.getCV( 55 );
    volumeSuono = (int)Dcc.getCV( 56 );
    fileAudio = (int)Dcc.getCV( 57 );
    soundStopTimeout = (int)Dcc.getCV( 58 ) * 50;
    closeTimeout = (long)Dcc.getCV( 59 ) * 1000;
    sensorTimeout = (int)Dcc.getCV( 60 ) * 10;
    lowThr = (int)Dcc.getCV( 61 );
    highThr = (int)Dcc.getCV( 62 );
    releseCounter = (int)Dcc.getCV( 63 );
  
    for (byte i=0; i < 8; i++) {
      inputMap[i] = (int)Dcc.getCV( 70+i );
    }
  
    if(volumeSuono > 30)
      volumeSuono = 30;
    if(volumeSuono < 1)
      volumeSuono = 1;
  
    if(sound) {
      if(attivazioneSuono == 0) {
        myDFPlayer.volume(0);
      }
      else {
        myDFPlayer.volume(volumeSuono);
      }
    }
  
    if(fileAudio > 2 || fileAudio < 0)
      fileAudio = 0;
  
    digitalWrite(LED, 0);
  }
}

void forceOpen() {
  if(STATO == OCCUPATO) {
    // richiedi l'apertura
    for (byte i=0; i < 4; i++) {
      occupazioneBinario[i] = LIBERO;
    }
    STATO = RICHIEDI_APERTURA;
    digitalWrite(LED, 1);
    LEDTime = millis();
  }
}

void setup()   //******************************************************
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);
  pinMode(puls_aperto, INPUT_PULLUP);
  pinMode(puls_chiuso, INPUT_PULLUP);
  byte i;
  
  Serial.begin(9600);
  #ifndef DEBUG
  if (myDFPlayer.begin(Serial, /*isACK = */true, /*doReset = */true)) {
    sound = true;
    myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
    myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  }
  else
  #endif
  {  
    sound = false;
    #ifdef DEBUG
    Serial.println("AccDec_PLsound");
    #endif
    #if defined(SERIALCOM)
    SerialCom.init(Serial);
    #endif
    for (i=0; i < 20; i++) {
      #if defined(SERIALCOM)
      if(!sound) {
        SerialCom.process();
      }
      #endif
      digitalWrite(LED, !digitalRead(LED));
      delay(100);
    }
  }

  // initialize the digital pins
  for (i=0; i < 2; i++) {
      pinMode(LampLed[i], OUTPUT);
      digitalWrite(LampLed[i], 0);
  }
  for (i=0; i < 8; i++) {
      pinMode(RailSensor[i], INPUT_PULLUP);
  }
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  Dcc.pin(0, 2, 1);
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 100, FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER, CV_To_Store_SET_CV_Address);

  #if defined(SERIALCOM)
  if(!sound) {
    SerialCom.process();
  }
  #endif
  delay(400);
  #if defined(SERIALCOM)
  if(!sound) {
    SerialCom.process();
  }
  #endif
  delay(400);
  #if defined(SERIALCOM)
  if(!sound) {
    SerialCom.process();
  }
  #endif
   
  #if defined(DECODER_LOADED)
  if ( Dcc.getCV(CV_DECODER_MASTER_RESET)== CV_DECODER_MASTER_RESET ) 
  #endif  
     {
       for (int j=0; j < sizeof(FactoryDefaultCVs)/sizeof(CVPair); j++ )
         Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
     }

  aggiornaCV();

  digitalWrite(LED, 0);
}

void loop()   //**********************************************************************
{
  //MUST call the NmraDcc.process() method frequently 
  // from the Arduino loop() function for correct library operation
  Dcc.process();

  #if defined(SERIALCOM)
    if(!sound) {
      SerialCom.process();
    }
  #endif

  aggiornaCV();

  if(isTime(&LEDTime, 500))
    digitalWrite(LED, 0);

  if(tipoControllo == 0) { // controllo manuale
    if(digitalRead(puls_chiuso) == LOW) {
      delay(50);
      if(digitalRead(puls_chiuso) == LOW) {
        // DCC chiede la chiusura
        if(STATO == LIBERO /*|| STATO == STA_APRENDO*/) {
          // richiedi la chiusura
          STATO = RICHIEDI_CHIUSURA;
          digitalWrite(LED, 1);
          LEDTime = millis();
        }
      }
    }
  }
  // forza apertura da pulsante
  if(digitalRead(puls_aperto) == LOW) {
    delay(50);
    if(digitalRead(puls_aperto) == LOW) {
      // DCC chiede l'apertura
      forceOpen();
    }
  }
  // forza apertura se scade il tempo massimo
  if(isTime(&closeTime, closeTimeout)) {
    forceOpen();
  }
  
  if(tipoControllo == 1) { // controllo con i sensori onboard
    
    for (byte i=0; i < 8; i++) {
      
      // è pasato il tempo di rilevazione?
      if(isTime(&sensorTime[i], sensorTimeout)) {
        
        #ifdef DEBUG
        Serial.print("c");
        Serial.print(i);
        Serial.print(":");
        Serial.print(counter[i]);
        Serial.print(",x:");
        Serial.println(counter_max[i]);
        #endif

        long res = ((counter[i]*100) / counter_max[i]);
        if(counter_max[i] > 2) {
          last_sensorState[i] = sensorState[i];
          
          if(res < lowThr) { // rilevata per la maggior parte del tempo qualcosa, logica bassa
            sensorState[i] = OCCUPATO; // salva lo stato definitivo
            closeTime = millis();
            releseCount[i] = 0;
  
            #ifdef DEBUG
            Serial.print("RILEVATO: s");
            Serial.print(i);
            Serial.print(":r");
            Serial.println(res);
            #endif
          }
          else if(res > highThr) {
            releseCount[i]++;
            if(releseCount[i] > releseCounter) {
              sensorState[i] = LIBERO;
            }
            else {
              sensorState[i] = last_sensorState[i];
              if(sensorState[i] == OCCUPATO) {
                closeTime = millis();
              }
            }
          }
          else {
            sensorState[i] = last_sensorState[i];
            if(sensorState[i] == OCCUPATO) {
              closeTime = millis();
            }
            releseCount[i] = 0;
          }
        }
        else {
          sensorState[i] = last_sensorState[i];
          if(sensorState[i] == OCCUPATO) {
            closeTime = millis();
          }
          releseCount[i] = 0;
        }
        counter[i] = 0;
        counter_max[i] = 0;
      }

      // rilevo lo stato istantaneo del sensore
      counter[i] += digitalRead(RailSensor[i]);
      counter_max[i] += 1;
      /*Serial.print("leggo sensore : ");
      Serial.print(i+1);
      Serial.print(" = ");
      Serial.println(!digitalRead(RailSensor[i]));*/
    }

    occupato = 0;

    // controllo logica sensori
    for (byte i=0; i < 8; i++) {

      // cancella occupazione
      if(sensorState[i] == LIBERO && last_sensorState[i] == OCCUPATO) {
        for (byte j=0; j < 8; j++) {
          if(inputMap[i] & (1 << j)) {
            byte binario = (j > 3) ? (j - 4) : j;
            
            if(occupazioneBinario[binario] == SINISTRO) {
              for (byte u=0; u < 8; u++) {
                for (byte v=0; v < 8; v++) {
                  if(inputMap[u] & (1 << v)) {
                    byte bin = (v > 3) ? (v - 4) : v;
                    if(binario == bin && v > 3 && j < 4) {
                      if(sensorState[u] == LIBERO) {
                        #ifdef DEBUG
                        Serial.print("MapS:U");
                        Serial.print(u);
                        Serial.print("V");
                        Serial.print(v);
                        Serial.print("J");
                        Serial.print(j);
                        Serial.print("B");
                        Serial.print(binario);
                        Serial.print("Bi");
                        Serial.println(bin);
                        #endif
                        occupazioneBinario[binario] = LIBERO;
                        goto libera;
                      }
                    }
                  }
                }
              }
            }
            
            if(occupazioneBinario[binario] == DESTRO) {
              for (byte u=0; u < 8; u++) {
                for (byte v=0; v < 8; v++) {
                  if(inputMap[u] & (1 << v)) {
                    byte bin = (v > 3) ? (v - 4) : v;
                    if(binario == bin && j > 3 && v < 4) {
                      if(sensorState[u] == LIBERO) {
                        #ifdef DEBUG
                        Serial.print("MapD:U");
                        Serial.print(u);
                        Serial.print("V");
                        Serial.print(v);
                        Serial.print("J");
                        Serial.print(j);
                        Serial.print("B");
                        Serial.print(binario);
                        Serial.print("Bi");
                        Serial.println(bin);
                        #endif
                        occupazioneBinario[binario] = LIBERO;
                        goto libera;
                      }
                    }
                  }
                }
              }
            }
            
          }
        }
      }
      libera:;

      // scrittura occupazione
      if(sensorState[i] == OCCUPATO && last_sensorState[i] == LIBERO) {
        for (byte j=0; j < 8; j++) {
          if(inputMap[i] & (1 << j)) {
            byte binario = (j > 3) ? (j - 4) : j;
            if(occupazioneBinario[binario] == LIBERO) {
              occupazioneBinario[binario] = (j > 3) ? SINISTRO : DESTRO;
              #ifdef DEBUG
              Serial.print("Map:I");
              Serial.print(i);
              Serial.print("J");
              Serial.print(j);
              Serial.print("B");
              Serial.print(binario);
              Serial.print("O");
              Serial.println(occupazioneBinario[binario] == SINISTRO ? "s":"d");
              #endif
            }
          }
        }
      }
      
    }

    // controllo logica sensori
    #ifdef DEBUG
    Serial.print("Occupazione: ");
    #endif
    for (byte i=0; i < 4; i++) {
/*      
      // rilevato primo sensore = scrittura occupazione
      if(sensorState[i] == OCCUPATO && last_sensorState[i] == LIBERO && occupazioneBinario[i] == LIBERO) {
        occupazioneBinario[i] = DESTRO;
        //Serial.println("rilevato primo sensore DESTRO");
      }
      if(sensorState[i+4] == OCCUPATO && last_sensorState[i+4] == LIBERO && occupazioneBinario[i] == LIBERO) {
        occupazioneBinario[i] = SINISTRO;
        //Serial.println("rilevato primo sensore SINISTRO");
      }

      // cancella occupazione
      if(sensorState[i] == LIBERO && last_sensorState[i] == OCCUPATO && occupazioneBinario[i] == SINISTRO) {
        //Serial.println("rilevato secondo sensore DESTRO");
        if(sensorState[i+4] == LIBERO)
          occupazioneBinario[i] = LIBERO;
      }
      if(sensorState[i+4] == LIBERO && last_sensorState[i+4] == OCCUPATO && occupazioneBinario[i] == DESTRO) {
        //Serial.println("rilevato secondo sensore SINISTRO");
        if(sensorState[i] == LIBERO)
          occupazioneBinario[i] = LIBERO;
      }
*/
      #ifdef DEBUG
      Serial.print(" ");
      Serial.print((char)occ[occupazioneBinario[i]]);
      #endif
      // se c'è almeno una occupazione registrala
      occupato += occupazioneBinario[i];
    }
    #ifdef DEBUG
    Serial.println();
    #endif

    if(occupato) {
      // almeno uno dei binari è occupato
      if(STATO == LIBERO || STATO == STA_APRENDO) {
        
        #ifdef DEBUG
        Serial.println("aperto o sta aprendo");
        #endif
        
        // richiedi la chiusura
        STATO = RICHIEDI_CHIUSURA;

        #ifdef DEBUG
        Serial.print("STATO chiusura");
        #endif
        
        digitalWrite(LED, 1);
        LEDTime = millis();
      }
    }
    else {
      // tutti i binari liberi
      if(STATO == OCCUPATO) {
        // richiedi l'apertura
        STATO = RICHIEDI_APERTURA;
        
        #ifdef DEBUG
        Serial.println("STATO apertura");
        #endif
        
        digitalWrite(LED, 1);
        LEDTime = millis();
        closeTime = millis();
      }
    }
  } 

  if(tipoControllo == 2 && occupato == LIBERO) { // controllo con DCC
    if(statoDCC) {
      // DCC chiede la chiusura
      if(STATO == LIBERO || STATO == STA_APRENDO) {
        // richiedi la chiusura
        STATO = RICHIEDI_CHIUSURA;
        digitalWrite(LED, 1);
        LEDTime = millis();
      }
    }
    else {
      // DCC chiede l'apertura
      if(STATO == OCCUPATO) {
        // richiedi l'apertura
        STATO = RICHIEDI_APERTURA;
        digitalWrite(LED, 1);
        LEDTime = millis();
      }
    }
  }

  // effettua apertura o chiusura se richiesto
  if(STATO == RICHIEDI_CHIUSURA) {

    #ifdef DEBUG
    Serial.println("richiesta chiusura");
    #endif
    
    for (byte i=0; i < 4; i++) {
      servoTime[i] = millis();
      //posizioneAttualeServi[i] = posizioneApertoServi[i];
      fine[i] = 0;
      prima = true;
    }
    STATO = STA_CHIUDENDO;
  }
  
  if(STATO == RICHIEDI_APERTURA) {

    #ifdef DEBUG
    Serial.println("richiesta apertura");
    #endif
    
    for (byte i=0; i < 4; i++) {
      //posizioneAttualeServi[i] = posizioneChiusoServi[i];
      fine[i] = 0;
    }
    STATO = STA_APRENDO;
  }
  
  if(STATO == STA_CHIUDENDO) {

    #ifdef DEBUG
    Serial.println("sta chiudendo");
    #endif
    
    // luci
    if(tipoLuci == 1) { // singola fissa
      digitalWrite(LampLed[0], 1);
      digitalWrite(LampLed[1], 1);
    }
    if(tipoLuci == 2) { // doppia lampeggiante
      if(isTime(&ledTime, frequenzaLampeggioLuci)) {
        digitalWrite(LampLed[0], !digitalRead(LampLed[0]));
        digitalWrite(LampLed[1], !digitalRead(LampLed[0]));
      }
    }

    // controlla se hanno finito di muoversi tutti i servi
    finito = fine[0] & fine[1] & fine[2] & fine[3];

    #ifdef DEBUG
    Serial.print("finito = ");
    Serial.println(finito);
    #endif
    
    // suono
    if(finito) {

      #ifdef DEBUG
      Serial.println("ciclo chiusura finito");
      #endif
      
      for (byte i=0; i < 4; i++) {
        vai[i] = 0;
        fine[i] = 0;
      }

      // stop audio se richiesto
      if(attivazioneSuono < 2 && sound) { // suono disattivo o attivo solo durante chiusura
        soundStopTime = millis();
      }

      closeTime = millis();
      STATO = OCCUPATO;
    }
    else {
      // audio
      if(prima) {
        prima = false;
        if (sound) {
          myDFPlayer.play(fileAudio);
          //mp3_play (fileAudio); // mp3/0000.mp3
          if(attivazioneSuono == 2)
            myDFPlayer.enableLoop();
        }
      }
    }

    // servo
    for (byte i=0; i < 4; i++) {
      if((isTime(&servoTime[i], ritardoChiusuraBarriera[i]) || vai[i] == 1) && !fine[i]) {
        vai[i] = 1;
        if(isTime(&servoTime2[i], velocitaServi)) {
          if(posizioneApertoServi[i] <= posizioneChiusoServi[i]) {
            posizioneAttualeServi[i]++;
          }
          else {
            posizioneAttualeServi[i]--;
          }
          if(posizioneAttualeServi[i] > posizioneChiusoServi[i]) {
            posizioneAttualeServi[i] = posizioneChiusoServi[i];
            fine[i] = 1;
          }
          servo[i].write(posizioneAttualeServi[i]);
          /*Serial.print("posizione C servo ");
          Serial.print(i+1);
          Serial.print(" = ");
          Serial.println(posizioneAttualeServi[i]);*/
        }
      }
    }
  }
  
  if(STATO == STA_APRENDO) {

    #ifdef DEBUG
    Serial.println("sta aprendo");
    #endif
    
    // servo
    for (byte i=0; i < 4; i++) {
      if((isTime(&servoTime[i], ritardoAperturaBarriera[i]) || vai[i] == 1) && !fine[i]) {
        vai[i] = 1;
        if(isTime(&servoTime2[i], velocitaServi)) {
          if(posizioneApertoServi[i] <= posizioneChiusoServi[i]) {
            posizioneAttualeServi[i]--;
          }
          else {
            posizioneAttualeServi[i]++;
          }
          if(posizioneAttualeServi[i] < posizioneApertoServi[i]) {
            posizioneAttualeServi[i] = posizioneApertoServi[i];
            fine[i] = 1;
          }
          servo[i].write(posizioneAttualeServi[i]);
          /*Serial.print("posizione A servo ");
          Serial.print(i+1);
          Serial.print(" = ");
          Serial.println(posizioneAttualeServi[i]);*/
        }
      }
    }
    
    // controlla se hanno finito di muoversi tutti i servi
    finito = fine[0] & fine[1] & fine[2] & fine[3];

    if(tipoLuci == 2) { // doppia lampeggiante
      if(isTime(&ledTime, frequenzaLampeggioLuci)) {
        digitalWrite(LampLed[0], !digitalRead(LampLed[0]));
        digitalWrite(LampLed[1], !digitalRead(LampLed[0]));
      }
    }

    // suono
    if(finito) {
      
      #ifdef DEBUG
      Serial.println("ciclo apertura finito");
      #endif
      
      for (byte i=0; i < 4; i++) {
        vai[i] = 0;
        fine[i] = 0;
      }

      if (sound) {
        // stop audio
        myDFPlayer.pause();
        myDFPlayer.disableLoop();
      }

      // stop luci
      digitalWrite(LampLed[0], 0);
      digitalWrite(LampLed[1], 0);
        
      STATO = LIBERO;
    }
  }
  
  if(STATO == LIBERO) {
    //Serial.println("aperto");
    // non fa niente
  }
  
  if(STATO == OCCUPATO) {
    // mantieni le luci e il suono se richiesto
    // luci
    if(tipoLuci == 2) { // doppia lampeggiante
      if(isTime(&ledTime, frequenzaLampeggioLuci)) {
        digitalWrite(LampLed[0], !digitalRead(LampLed[0]));
        digitalWrite(LampLed[1], !digitalRead(LampLed[0]));
        //Serial.println("chiuso");
      }
    }
    // stoppa il suono dopo un ritardo predefinito
    if(isTime(&soundStopTime, soundStopTimeout)) {
      if (sound) {
        myDFPlayer.pause();
        myDFPlayer.disableLoop();
      }
    }
  }
}


boolean isTime(unsigned long *timeMark, unsigned long timeInterval) {
    unsigned long timeNow = millis();
    if ( timeNow - *timeMark >= timeInterval) {
        *timeMark = timeNow;
        return true;
    }    
    return false;
}

// This function is called by the NmraDcc library
// when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain
// on the power supply for 6ms to ACK a CV Read
void notifyCVAck(void) {
  digitalWrite( LED, HIGH );
  digitalWrite( LampLed[0], HIGH );
  digitalWrite( LampLed[1], HIGH );
  delay(10);
  digitalWrite( LED, LOW );
  digitalWrite( LampLed[0], LOW );
  digitalWrite( LampLed[1], LOW );
}

extern void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower ) {
  uint16_t Current_Decoder_Addr = Dcc.getAddr();
  if ( Addr == Current_Decoder_Addr && tipoControllo == 2 ) { //Controls Accessory_Address

#ifdef DEBUG
   Serial.print("Addr = ");
   Serial.println(Addr);
   Serial.print("direction = ");
   Serial.println(Direction);
#endif

    statoDCC = Direction ? 1 : 0;
  } 
}

/*
extern void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State) {
  uint16_t Current_Decoder_Addr;
  uint8_t Bit_State;
  Current_Decoder_Addr = Dcc.getAddr();
  Bit_State = OutputAddr & 0x01;
  
  if ( Addr == Current_Decoder_Addr && tipoControllo == 2 ) { //Controls Accessory_Address
#ifdef DEBUG
	 Serial.print("Addr = ");
	 Serial.println(Addr);
     Serial.print("BoardAddr = ");
	 Serial.println(BoardAddr);
	 Serial.print("Bit_State = ");
	 Serial.println(Bit_State);
#endif

	  statoDCC = Bit_State;
 
  } 
}
*/
