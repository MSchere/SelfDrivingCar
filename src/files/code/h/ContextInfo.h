/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ContextInfo.h
 * Author: ruben
 *
 * Created on 6 de octubre de 2016, 11:02
 */

#ifndef CONTEXTINFO_H
#define CONTEXTINFO_H

#include <iostream>
#include <vector>
#include <raceman.h>
#include <robottools.h>

#include "Definitions.h"



class ContextInfo {
public:

    ContextInfo(tCarElt* car, tSituation *s);
    virtual ~ContextInfo();

    void updateContext();

    contextInfo_t getMyContextInfo();
    contextInfo_t getCarContextInfo(int id);
    std::vector<contextInfo_t> getAllCarsContextInfo();
    std::vector<relativePosition_t> getRelativePositions();
    surroundingVehicles_t getSurround();
    
    double getCurrentTime();
    double getCenterForLaneId(int laneId);

private:

    contextInfo_t computeContext(tCarElt* car);
    dynamics_t computeDynamics(tCarElt* car);
    surroundingVehicles_t computeSurround(tCarElt* car, tSituation *s);
    posVehicle_t getFrontCar(tCarElt* car, int evalLaneID, std::vector<relativePosition_t> positions);
    posVehicle_t getMiddleCar(tCarElt* car, int evalLaneID, std::vector<relativePosition_t> positions);
    posVehicle_t getRearCar(tCarElt* car, int evalLaneID, std::vector<relativePosition_t> positions);

    roadStruct_t computeRoadStruct(tCarElt* car, double minLaneWidth);
    double computeLaneCenter(roadStruct_t road, int laneId);
    int computeMainLaneId(roadStruct_t road, double dist2RoadCenter);
    int computeSecondLaneId(roadStruct_t road, carCorners_t relativeCorners, int mainLaneId);
    double computeDist2Lane(roadStruct_t road, int laneId, double dist2RoadCenter);
    carCorners_t computeCarCorners(tCarElt* car, int mode);

    std::vector<relativePosition_t> carsPosition(tCarElt* car, tSituation *s, std::vector<contextInfo_t> context);

    contextInfo_t m_context;
    std::vector<contextInfo_t> m_allCarsContext;
    std::vector<relativePosition_t> m_relativePositions;
    surroundingVehicles_t m_surround;

    double m_currentTime, m_acuCurrentTime;

    tCarElt *m_car;
    tSituation *m_s;

};

#endif /* CONTEXTINFO_H */

