#pragma once

#define PIN_BLUETOOTH_RX    11
#define PIN_BLUETOOTH_TX    10

#include "math.h"
#include "SoftwareSerial.h"

// #define DEBUGG(...) Serial.println(__VA_ARGS__)
#define DEBUGG(...)

// #define BDEBUGG(...) bluetooth.println(__VA_ARGS__)
#define BDEBUGG(...)

/**
 * @brief calculates chcecksum for NMEA sentence
 * 
 * @param NMEA_Sentence sentence to calculate chceksum from
 * @param sentenceLength length of sentence
 * @return byte == checksum
 */
byte calculateCheckSum(const char *NMEA_Sentence, int sentenceLength);

/**
 * @brief creates NMEA sentence 
 * 
 * @param originTime        [milisec] number of seconds since estabilishing the origin of coordinate system
 * @param hdist_MP          [m] horizontal distance traveled
 * @param inclination_MP    [rad] angle of inclination between the +x axis and the movement direction (in current epoch)
 * @return char $SNODO,[time/sec],[distance/mm],[P/N],[inclination/rad],[P/N]*[checksum]<CR><LF>
 */
void create_NMEA(int originTime, float hdist_MP, float inclination_MP);

// ====================================================
// === === === ABBREVIATIONS & NOMENCLATURE === === ===
// ====================================================
//  TP    ==  touchpoint; where the wheel meets ground
//  MP    ==  mountpoint; where the direction encoder is mounted
//  Ref   ==  reference point of vehicle

// === === === ODOMETER PROPORTIONS === === ===
/**
 * @brief horizontal distance between non-swivel axle and MP in [meters]
 * in    <-oo;0): for axle being at the back of odometer (and thus MP)
 *        <0;oo>: for axle being in front of odometer 
 */
#define hdist_axle_MP 0.3

/**
 * @brief horizontal distances between MP and Ref parallel to the anteroposterior axis of vehicle in [meters]
 * in    <-oo;0): for Ref being at the back of odometer
 *        <0;oo>: for Ref being in front of odometer
 */
#define parallel_MP_Ref 0

/**
 * @brief horizontal distances between MP and Ref perpendicular to the anteroposterior axis of vehicle in [meters]
 * in    <-oo;0): for Ref being on the right to the odometer
 *        <0,oo>: for Ref being on the left to the odometer
 */
#define perpendicular_MP_Ref 0

/**
 * @brief horizontal distance between TP and MP in [meters] 
 */
#define hdist_TP_MP 0.077

/**
 * @brief radius of distance wheel in [meters]
 */
#define wheelRadius 0.048

// =================================================
// === === === DISTANCE ENCODER DEFINING === === ===
// =================================================
/**
 * @brief numer of pulses per revolution of distance encoder
 */
#define dist_Ppr 600

/**
 * @brief to which Arduino pin is A phase of distance encoder attached
 */
#define dist_PinA 2

/**
 * @brief to which Arduino pin is B phase of distance encoder attached
 */
#define dist_PinB 14

/**
 * @brief number of steps of distance encoder to compute NEW position of TP
 */
#define dist_steps2compute 20

/**
 * @brief how many computing of TP position occurs before computing NEW position of MP & Ref and sending NMEA sentence
 */
#define compute2sendMultiplier 15

// ==================================================
// === === === DIRECTION ENCODER DEFINING === === ===
// ==================================================
/**
 * @brief number of pulses per revolution of direction encoder
 */
#define dirc_Ppr 2000

/**
 * @brief to which Arduino pin is A phase of direction encoder attached
 */
#define dirc_PinA 3

/**
 * @brief to which Arduino pin is B phase of direction encoder attached
 */
#define dirc_PinB 7

/**
 * @brief 2D cartesian coordinates
 */
typedef struct coordinates {
  float X;
  float Y;
} coord_t;

typedef struct polar {
    float distance;
    float inclination;
} polar_t;

/**
 * @brief makes reading on distance encoder
 */
void ISR_distance();

/**
 * @brief makes reading on direction encoder
 */
void ISR_direction();

/**
 * @brief Set the up encoders object
 */
void setup_encoders();

/**
 * @brief estabilishes NEW reference coordinates system
 */
void doReset();

/**
 * @brief computes XY coordinates of desired destination
 * 
 * @param XY0           current coordinates
 * @param distance      distance to desired destination
 * @param inclination   inclination to desired destination
 * @return coord_t 
 */
coord_t computeXY_polar(coord_t XY0, float distance, float inclination);

/**
 * @brief computes differentially NEW coordinates of TP
 * 
 * @param dirc_reading  reading on direction encoder
 * @param dist_reading  reading on dostance encoder
 */
void compute_newTP(volatile int &dirc_reading, volatile int &dist_reading);

/**
 * @brief computes distance and inclination from coordinates
 * 
 * @param XY_OLD    coordinates of origin
 * @param XY_NEW    coordinates of end
 * @return polar_t  distance [m] and inclination [rad]
 */
polar_t compute_dist_inclin(coord_t XY_OLD, coord_t XY_NEW);

/**
 * @brief computes distance and inclination between OLD Ref and NEW Ref
 * 
 * @return polar_t 
 */
polar_t oldRef2newRef();

/**
 * @brief Set the direction encoder ZERO reading
 * 
 * @param dirc_reading 
 */
void set_dircZero();

// // Arduino Bluetooth module HC-06
// // Bluetooth connecting pin settings
// #define RX 11
// #define TX 10
