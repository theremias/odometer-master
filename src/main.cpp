#include <Arduino.h>
#include <SoftwareSerial.h>

#include "common.h"

SoftwareSerial bluetooth { PIN_BLUETOOTH_RX, PIN_BLUETOOTH_TX, false };


/**
 * @brief reading on distance encoder
 */
volatile int dist_position = 0;

/**
 * @brief reading on direction encoder
 * 
 */
volatile int dirc_position = 0;


void ISR_distance() {
    if ( digitalRead( dist_PinB ) == LOW ) {
      dist_position--;
    } else {
      dist_position++;
    }
    DEBUGG("Ctena zmena na vzdalenostnim encoderu");
}

void ISR_direction() {
    if ( digitalRead( dirc_PinB ) == LOW ) {
      dirc_position--;
    } else {
      dirc_position++;
    }
    DEBUGG("Ctena zmena na smerovem encoderu");
}

/**
 * @brief number of differential computing since last enumeration of MP and Ref 
 */
int diffCount = 0;

/**
 * @brief time from estabilishing origin of current coordinate system
 */
int originMilisec = 0;


// ================================================================
// ===             BLUETOOTH MODULE DEFINIG & STUFF             ===
// ================================================================
// // Arduino Bluetooth module HC-06
// // Bluetooth connecting pin settings
// #define RX 11
// #define TX 10

// Bluetooth module inicialization from SoftwareSerial library
// SoftwareSerial bluetooth(RX, TX);
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// ^^^             BLUETOOTH MODULE DEFINIG & STUFF             ^^^
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^




// === === === DISTANCE ENCODER === === ===
/**
 * @brief distance traveled with one pulse of distance encoder
 */
float dist_step = M_PI * 2 * wheelRadius / dist_Ppr;

/**
 * @brief differential (center) angular distance between TPs 
 */
float phi;

/**
 * @brief angular distance of one pulse of direction encoder
 */
float dirc_step = 2 * M_PI / dirc_Ppr;

/**
 * @brief measured direction angle
 */
float theta;

// === === === COORDINATE SYSTEM === === ===
/**
 * @brief coordinates of the TP of wheel
 */
coord_t XY_TP = {0, 0};

/**
 * @brief angle between +x axis and OLD TP to NEW TP line 
 */
float inclination_TP = 0;

/**
 * @brief XY coordinates of the NEW Ref position (while sending NMEA)
 */
coord_t XY_Ref = { hdist_TP_MP + parallel_MP_Ref, perpendicular_MP_Ref};

/**
 * @brief horizontal distance between MP and Ref
 */
float hdist_MP_Ref = sqrt( pow( parallel_MP_Ref, 2 ) + pow( perpendicular_MP_Ref, 2 ) );

/**
 * @brief angle between ateroposterior axis of vehicle and MP to Ref line 
 */
float kappa = atan2( perpendicular_MP_Ref, parallel_MP_Ref );

/**
 * @brief angle between +x axis and OLD MP to NEW MP line
 */
float inclination_MP;

/**
 * @brief horizontal distance between OLD MP and NEW MP
 */
float hdist_MP;

void setup_encoders() {
  pinMode (dist_PinA, INPUT_PULLUP);
  pinMode (dist_PinB, INPUT_PULLUP);
  pinMode (dirc_PinA, INPUT);
  pinMode (dirc_PinB, INPUT);
  DEBUGG("Encodery nastaveny...");
  attachInterrupt(digitalPinToInterrupt(dist_PinA), ISR_distance, RISING);
  attachInterrupt(digitalPinToInterrupt(dirc_PinA), ISR_direction, RISING);
}


coord_t computeXY_polar(coord_t XY0, float distance, float inclination) {
    coord_t newXY;
    newXY.X = XY0.X + distance * cos( inclination );
    newXY.Y = XY0.Y + distance * sin( inclination );
    return newXY;
}

void compute_newTP(int *dirc_reading, int dist_reading) {
    theta = dirc_step * dirc_reading;
    DEBUGG("Mereny uhel smeroveho enkoderu [rad]:"); DEBUGG(theta);
    //BDEBUGG("Mereny uhel smeroveho enkoderu [imp]:"); 
    BDEBUGG(dirc_reading);

    /**
     * @brief differential horizontal distance between TPs
     */
    float diff_hdist_TP = dist_step * dist_reading;
    // * cos( ypr[2] );

    DEBUGG("Diferencialni vzdalenost OLD TP a NEW TP [m]:"); DEBUGG(diff_hdist_TP);
    
    phi = 2.0 * asin( diff_hdist_TP * sin(theta) / ( 2.0 * ( hdist_axle_MP + hdist_TP_MP * cos(theta) ) ) );
    DEBUGG("Uhlovy posun z OLD TP do NEW TP [rad]:"); DEBUGG(phi);

    inclination_TP = inclination_TP + phi;
    DEBUGG("Smernik z OLD TP na NEW TP [rad]:"); DEBUGG(inclination_TP);
    XY_TP = computeXY_polar(XY_TP, diff_hdist_TP, inclination_TP);
    DEBUGG("Souradnice NEW TP [X/m; Y/m]:");
    DEBUGG(XY_TP.X); DEBUGG(XY_TP.Y);
}

polar_t compute_dist_inclin(coord_t XY_OLD, coord_t XY_NEW) {
    polar_t result;
    float dx = XY_NEW.X - XY_OLD.X;
    float dy = XY_NEW.Y - XY_OLD.Y;
    result.distance = sqrt( pow( dx, 2 ) + pow( dy, 2 ) );
    result.inclination = atan2( dy, dx );
    return result;
}

polar_t oldRef2newRef() {
    /**
     * @brief angle between +x axis and TP to MP line 
     */
    float inclination_TP_MP = inclination_TP + phi / 2.0;
    DEBUGG("Smernik z TP na MP:"); DEBUGG(inclination_TP_MP);

    /**
     * @brief coordinates of MP
     */
    coord_t XY_MP = computeXY_polar(XY_TP, hdist_TP_MP, inclination_TP_MP);
    DEBUGG("Souradnice NEW MP [X/m; Y/m]:");
    DEBUGG(XY_MP.X); DEBUGG(XY_MP.Y);

    /**
     * @brief coordinates of the old position of Ref; to compute distance and inclination
     */
    coord_t XY_Ref_Old = XY_Ref;

    /**
     * @brief angle between +x axis and MP to Ref line 
     */
    float inclination_MP_Ref = inclination_TP_MP + theta - kappa;
    DEBUGG("Smernik z MP na Ref:"); DEBUGG(inclination_MP_Ref);

    XY_Ref = computeXY_polar(XY_MP, hdist_MP_Ref, inclination_MP_Ref);
    DEBUGG("Souradnice NEW Ref [X/m; Y/m]:");
    DEBUGG(XY_Ref.X); DEBUGG(XY_Ref.Y);

    polar_t disInc = compute_dist_inclin(XY_Ref_Old, XY_Ref);
    DEBUGG("Smernik a delka z OLD Ref na NEW Ref [rad; m]:");
    DEBUGG(disInc.inclination); DEBUGG(disInc.inclination);

    return disInc;
}



void doReset() {
  // time of estabilishing origin of the system
  originMilisec = millis();
  // XY coordinates of the touch point of wheel
  XY_TP = {0, 0};
  // angle between +x axis and TP(n-1) to TP(n) line 
  inclination_TP = 0;
  // XY coordinates of the NEW MP position (while sending NMEA)
  XY_Ref = {hdist_TP_MP + parallel_MP_Ref, perpendicular_MP_Ref};
  DEBUGG("Origin set.");
  BDEBUGG("Origin set.");
}


/**
 * @brief Set the up Bluetooth object
 * 
 */
void setup_Bluetooth() {
  // zahájení komunikace s Bluetooth modulem
  // skrze Softwarovou sériovou linku rychlostí 38400 baud
  bluetooth.begin(38400);
  bluetooth.write("Arduino zapnuto, test Bluetooth..");
  DEBUGG("Arduino test, pri testu spojeni...");
}

/**
 * @brief calculates checksum of a sentence (XOR)
 * 
 * @param NMEA_Sentence sentence to be calculated checksum for
 * @param sentenceLength length of the sentence
 * @return byte two hexadecimal characters
 */
byte calculateCheckSum(const char *NMEA_Sentence, int sentenceLength) {
  byte result = 0;
  for( int i = 0; i < sentenceLength; i++) {
     result ^= NMEA_Sentence[i];
  }
  return result;
}

void create_NMEA(int originTime, float distance, float inclination) {
  // computingCount = 0;
  DEBUGG("Zacinam resit NMEA zpravu...");
  char NMEA_inside[80] = "";
  char NMEA_identificator[] = "SNODO";
  int currentTime = (millis() - originTime) / 1000;
  String NMEA_distance = String(distance * 1000, 1);
  String NMEA_inclination = String(inclination, 4);
  sprintf(NMEA_inside, "%s,%d,%s,%c,%s,%c", 
                        NMEA_identificator,
                        currentTime, 
                        NMEA_distance.c_str(),
                        distance >= 0 ? 'P' : 'N',
                        NMEA_inclination.c_str(),
                        inclination >= 0 ? 'P' : 'N' );

  byte NMEA_checksum = calculateCheckSum(NMEA_inside, sizeof(NMEA_inside));

  char NMEA_whole[81];
  sprintf(NMEA_whole, "$%s*%02hhx\r\n", NMEA_inside, NMEA_checksum);
  DEBUGG("=== === === NMEA SENTENCE === === ==="); 
  DEBUGG(NMEA_whole);
  DEBUGG("^^^ ^^^ ^^^ NMEA SENTENCE ^^^ ^^^ ^^^");
  bluetooth.write( NMEA_whole );

}

// sets the origin of reference coordinate system
void checkForBluetooth() {
  // if you want to reset the origin, send '0'
  if ( bluetooth.available() ) {
    char BTinput = bluetooth.read(); 
    if( BTinput == '0' ) {
      doReset();
    } else if (BTinput == '1' ) {
      DEBUGG("Jed asi metr rovne..."); BDEBUGG("Jed asi metr rovne...");
      set_dircZero();
    }

    DEBUGG( int(BTinput) ); //BDEBUGG( int(BTinput) );
    DEBUGG("BT jede");
    BDEBUGG("BT jede...");
  }
}

void set_dircZero() {
  int count = 200;
  int reading = 0;
  while (dist_position < count)
  {
    
    for (int i = 0; i <= dist_position; i++)
    {
      reading += dirc_position;
    }
    
  }

  dirc_position -= reading / count; 
  dist_position = 0;
  DEBUGG("Smer 0 nastaven"); BDEBUGG("Smer 0 nastaven");
  DEBUGG( dirc_position); BDEBUGG( dirc_position );
}

void setup() {
  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  DEBUGG("Komunikace na serial portu zapocala...");
  //setup_MPU();
  //DEBUGG("MPU nastaveno");
  setup_encoders();
  setup_Bluetooth();
}

void loop() {
 //read_MPU();
  //DEBUGG("MPU precteno");
  if ( abs( dist_position ) > dist_steps2compute) {
    compute_newTP(dirc_position, dist_position);
    dist_position = 0;
    diffCount++;
    DEBUGG("Diferencialni vypocet cislo:");
    DEBUGG(diffCount);
  }

  if( diffCount >= compute2sendMultiplier) {
    diffCount = 0;
    polar_t toSend = oldRef2newRef();
    create_NMEA(originMilisec, toSend.distance, toSend.inclination);
    // bluetooth.write(NMEA);
  }
  
  checkForBluetooth();
}