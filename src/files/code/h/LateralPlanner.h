/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LateralPlanner.h
 * Author: ruben
 *
 * Created on 5 de octubre de 2016, 18:43
 */

#ifndef LATERALPLANNER_H
#define LATERALPLANNER_H

#include <math.h>
#include <raceman.h> 

#include "LateralController.h"
#include "ContextInfo.h"
#include "Definitions.h"


class LateralPlanner {
public:

    LateralPlanner(tCarElt* car, tSituation *s, ContextInfo *context, driverParameters_t *driverParams);
    virtual ~LateralPlanner();

    void control(int desiredLaneId, double lengthProcess, int typeProcess);
    void abortCurrentLaneChange();
    bool laneChangePending();

private:

    bool setBestLaneFromLP(int laneId, double lengthProcess, int typeProcess);
    void requestParametricLaneChangeTypeDist(double startLonPos, double startLatPos, double endLonPos, double endLatPos);
    void requestParametricLaneChangeTypeTime(double startTime, double startLatPos, double endTime, double endLatPos);
    double evalParams(double dist);

    double m_currentReference;
    parametricLaneChange_t m_laneChange;
    bool m_firstIteration;
    bool m_laneChangeIsPending;

    // OJO ESTO HAY QUE METERLO ESTA VACIO / ACTUALIZAR
    driverParameters_t *m_driverParams;
    ContextInfo *m_context;
    contextInfo_t m_myContext;
    std::vector<contextInfo_t> m_AllContext;
    std::vector<relativePosition_t> m_relativePositions;
    surroundingVehicles_t m_surround;

    tCarElt *m_car;
    tSituation *m_s;

    LateralController* m_LC;

};

#endif /* LATERALPLANNER_H */

