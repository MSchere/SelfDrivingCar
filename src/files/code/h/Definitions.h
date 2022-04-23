/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Definitions.h
 * Author: ruben
 *
 * Created on 21 de noviembre de 2016, 17:29
 */

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <vector>

/* Lateral planner defines ****************************************************/
#define LP_TP_DIST  0
#define LP_TP_TIME  1
#define LP_LANE_0   0
#define LP_LANE_1   1
/******************************************************************************/

/* Longitudnal planner defines ************************************************/
#define LP_DELTA   0
#define LP_SLOPE   1
#define LP_TIME_AHEAD     5.0  
#define LP_RELA_SPEED    0.95
/******************************************************************************/

/* Context info defines *******************************************************/
#define CTX_LANE_WIDTH  3.5
#define CTX_CAR_CORNERS_ABS     1
#define CTX_CAR_CORNERS_REL     2
/******************************************************************************/


typedef struct {
    double currentTrack; // 0 straight 1 curve
    double currentRadius; // meters
    double nextRadius; // meters
    double timeToNextStretch; // seconds
} speedFisData_t;

typedef struct {
    double lateralError; // meters
    double angularError; // degrees
} steerFisData_t;

typedef struct {
    double posX;
    double posY;
    double heading;
    double speed;
    double yawRate;
    double accel;
    double time;
    char* robotName;
} carData_t;

typedef struct {
    double startTime; // seconds
    double startSpeed; // m/s
    double endTime; // seconds
    double endSpeed; // m/s
    int mode; // zero or one
} SpeedTime_t;

typedef struct {
    double currentSpeed;
    double finalSpeed;
} Speed_t;

typedef struct {
    double ko0, ko1; // parameter of overtaken function
    double acuLostSpeed; // acumulation of speed lost during overtaken 
    double probabilityThreshold;
    bool overtakenDecission;
} overTakenParameters_t;

typedef struct {
    double desiredSpeed; // cruise speed
    overTakenParameters_t overtaken;
    double frontWatchingLenght;
    double rearWatchingLenght;
    double accelParameter;
} driverParameters_t;

typedef struct {
    double startEval;
    double startLatPos;
    bool laneChangeStarted;
    double endEval;
    double endLatPos;
    bool laneChangeFinished;
    double params[6];
    int typeProcess;
} parametricLaneChange_t;

typedef struct {
    double x;
    double y;
} carCorner_t;

typedef struct {
    carCorner_t FL;
    carCorner_t FR;
    carCorner_t RL;
    carCorner_t RR;
} carCorners_t;

typedef struct {
    double width;
    int numberLanes;
    double laneWidth;
    std::vector<double> laneCenter;
    std::vector<double> laneCenterFromRightLimit;
    std::vector<double> laneStartFromRightLimit;
    std::vector<double> laneEndFromRightLimit;
} roadStruct_t;

typedef struct {
    int id;
    double frontDist;
    double rearDist;
    double usedDist;
    int mainLaneId;
    int secondLaneId;
} relativePosition_t;

typedef struct {
    double posX, posY;
    double speedX, speedY;
    double accelX, accelY;
    double heading;
    double yawRate;
    double dist2RoadCenter;
    double headingError;
} dynamics_t;

typedef struct {
    dynamics_t dynamics;
    roadStruct_t road;
    double dist2RoadCenter;
    double dist2RightLimit;
    double distRaced;
    int mainLaneId;
    double dist2MainLaneCenter;
    int secondLaneId;
    double dist2SecondLaneCenter;
    carCorners_t absoluteCorners;
    carCorners_t relativeCorners;
} contextInfo_t;

typedef struct {
    bool car;
    int id;
    double dist;
    double speed;
} posVehicle_t;

typedef struct {
    bool lane;
    int laneId;
    posVehicle_t front;
    posVehicle_t middle;
    posVehicle_t rear;
} sideLane_t;

typedef struct {
    sideLane_t left;
    sideLane_t center;
    sideLane_t right;
} surroundingVehicles_t;





#endif /* DEFINITIONS_H */

