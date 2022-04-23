/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GeneratingTorcsData.h
 * Author: ruben
 *
 * Created on 5 de octubre de 2016, 18:31
 */

#ifndef GENERATINGTORCSDATA_H
#define GENERATINGTORCSDATA_H

#include <car.h>
#include <raceman.h>
#include <vector>

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <robottools.h>
#include <robot.h>

#include "linalg.h"
#include "Definitions.h"

class GeneratingTorcsData {
public:
    GeneratingTorcsData();
    virtual ~GeneratingTorcsData();

    void collectData(tCarElt* car, tSituation *s);

    carData_t getMyCarData() {
        return m_myCar;
    }

    std::vector<carData_t> getAllCarsData() {
        return m_allCars;
    }

    carData_t getCarData(int carID);

    speedFisData_t getSpeedFisInputData() {
        return m_mySpeedFisData;
    }

    steerFisData_t getSteerFisInputData() {
        return m_mySteerFisData;
    }

private:

    carData_t computeCarData(tCarElt* car);
    speedFisData_t computeSpeedFisInputData(carData_t carData, tCarElt* car);
    steerFisData_t computeSteerFisInputData(carData_t carData, tCarElt* car);

    double getDistToNextStretch(tTrkLocPos trkPos);
    double getDistToSegEnd(tTrkLocPos trkPos);
    double getNextRadius(tTrackSeg *seg);

    carData_t m_myCar;
    std::vector<carData_t> m_allCars;

    speedFisData_t m_mySpeedFisData;
    steerFisData_t m_mySteerFisData;

    double m_currentTime, m_acuCurrentTime;
};

#endif /* GENERATINGTORCSDATA_H */

