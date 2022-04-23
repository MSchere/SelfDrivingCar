/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Driver.h
 * Author: ruben
 *
 * Created on 9 de noviembre de 2016, 11:51
 */

#ifndef DRIVER_H
#define DRIVER_H

#include "GeneratingTorcsData.h"
#include "TorcsLogger.h"
#include "ContextInfo.h"
#include "LateralPlanner.h"
#include "LongitudinalPlanner.h"
#include "NormalDistribution.h"
#include "Definitions.h"

#include "CpuTime.h"

class Driver {
public:

    Driver(tCarElt *car, tSituation *s);
    virtual ~Driver();

    void drive();

    int getMyId() {
        return m_robotId;
    }

private:
    
    driverParameters_t buildNewDriver(double dangerous);
    

    GeneratingTorcsData* m_torcsData;
    TorcsLogger* m_log;

    ContextInfo* m_context;

    LateralPlanner* m_lateralPlanner;
    LongitudinalPlanner* m_longitudinalPlanner;

    driverParameters_t m_driverParams;
    tCarElt *m_car;
    tSituation *m_s;

    int m_robotId;

    NormalDistribution m_ND;

};

#endif /* DRIVER_H */

