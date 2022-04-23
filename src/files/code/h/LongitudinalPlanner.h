/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LongitudinalPlanner.h
 * Author: ruben
 *
 * Created on 5 de octubre de 2016, 18:33
 */

#ifndef LONGITUDINALPLANNER_H
#define LONGITUDINALPLANNER_H

#include <stdio.h>
#include <vector>

#include <raceman.h>  

//#include "IDM.h"
#include "LongitudinalController.h"
#include "ContextInfo.h"
#include "NormalDistribution.h"
#include "CarFollower.h"
#include "Definitions.h"


class LongitudinalPlanner {
public:
    LongitudinalPlanner(tCarElt* car, tSituation *s, ContextInfo *context, driverParameters_t *driverParams);
    virtual ~LongitudinalPlanner();

    void addSpeedProfileToSpeedTimeSeries(double endTime, double endSpeed, int mode);
    int control();
    int getBestLaneFronLP();


private:

    Speed_t getLongitudinalSpeedReference(double currentTime);
    bool evaluateOvertaken(double currentLaneSpeed, double leftLaneSpeed, driverParameters_t *driverParams);

    std::vector<SpeedTime_t> m_speedBehaviour;
    double m_lastSpeed, m_lastTime;
    double m_currentSpeed;

    //IDM* m_IDM;
    CarFollower *m_follower;
    LongitudinalController::reference_t m_reference;


    driverParameters_t *m_driverParams;
    int m_requestLane;

    ContextInfo *m_context;
    contextInfo_t m_myContext;
    std::vector<contextInfo_t> m_AllContext;
    std::vector<relativePosition_t> m_relativePositions;
    surroundingVehicles_t m_surround;

    LongitudinalController* m_LC;

    tCarElt *m_car;
    tSituation *m_s;
};

#endif /* LONGITUDINALPLANNER_H */

