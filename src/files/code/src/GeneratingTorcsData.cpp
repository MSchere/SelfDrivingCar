/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GeneratingTorcsData.cpp
 * Author: ruben
 * 
 * Created on 5 de octubre de 2016, 18:31
 */

#include "GeneratingTorcsData.h"

GeneratingTorcsData::GeneratingTorcsData() {
    m_currentTime = -1e+5;
    m_acuCurrentTime = 0;
}

GeneratingTorcsData::~GeneratingTorcsData() {
}

void GeneratingTorcsData::collectData(tCarElt* car, tSituation *s) {
    m_allCars.clear();
    for (int i = 0; i < s->raceInfo.ncars; i++)
        m_allCars.push_back(this->computeCarData(s->cars[i]));
    m_myCar = this->computeCarData(car);

    m_mySpeedFisData = this->computeSpeedFisInputData(m_myCar, car);
    m_mySteerFisData = this->computeSteerFisInputData(m_myCar, car);
}

carData_t GeneratingTorcsData::getCarData(int carID) {
    if (carID >= 0 && carID <= m_allCars.size())
        return m_allCars.at(carID);
    else {
        printf("Error, los datos del vehiculo solicitado no existen\n");
        exit - 1;
    }
}

carData_t GeneratingTorcsData::computeCarData(tCarElt* car) {

    if (car->race.curLapTime < m_currentTime)
        m_acuCurrentTime = m_currentTime;
    m_currentTime = car->race.curLapTime + m_acuCurrentTime;

    carData_t carData;
    carData.posX = car->pub.DynGC.pos.x;
    carData.posY = car->pub.DynGC.pos.x;
    carData.heading = car->pub.DynGC.pos.az;
    carData.speed = car->pub.DynGC.vel.x;
    carData.yawRate = car->pub.DynGC.vel.az;
    carData.accel = car->pub.DynGC.acc.x;
    carData.time = m_currentTime;
    carData.robotName = car->priv.modName;
    return carData;
}

speedFisData_t GeneratingTorcsData::computeSpeedFisInputData(carData_t carData, tCarElt* car) {

    speedFisData_t data;

    // distancia en metros al siguiente tramo (curva si estamos en recta y viceversa; entre curvas diferentes se suele pasar siempre por recta)
    // range: between 0 and inf
    double distToNextStretch = getDistToNextStretch(car->pub.trkPos);
    if (carData.speed == 0)
        data.timeToNextStretch = 1000;
    else
        data.timeToNextStretch = distToNextStretch / carData.speed;

    // tipo de tramo en el que estamos
    // range: 1 right curve; 2 left curve; 3 straight
    // para simplificar pasamos 0 si recta, 1 si curva
    double currentTrack = car->pub.trkPos.seg->type;
    if (currentTrack == 3)
        data.currentTrack = 0;
    else
        data.currentTrack = 1;

    // radio de curvatura del tramo actual
    // range: 0 straight >0 to inf curve 40, 100, 400, etc..
    data.currentRadius = car->pub.trkPos.seg->radius;

    // radio de curvatura del siguiente tramo actual
    //   range: 0 straight >0 to inf curve 40, 100, 400, etc..
    data.nextRadius = getNextRadius(car->pub.trkPos.seg);


    return data;
}

steerFisData_t GeneratingTorcsData::computeSteerFisInputData(carData_t carData, tCarElt* car) {
    steerFisData_t data;

    data.angularError = RtTrackSideTgAngleL(&(car->pub.trkPos)) - carData.heading;
    NORM_PI_PI(data.angularError); // put the angle back in the range from -PI to PI
    data.angularError = data.angularError * 180 / 3.14159;

    data.lateralError = -car->pub.trkPos.toMiddle;

    return data;
}

double GeneratingTorcsData::getDistToNextStretch(tTrkLocPos trkPos) {
    tTrackSeg *seg = trkPos.seg;
    float length = getDistToSegEnd(trkPos);
    float cRadius = seg->radius;

    // TR_RGT 1 Right curve
    // TR_LFT 2 Left curve
    // TR_STR 3 Straight

    if (seg->type == TR_STR) {
        seg = seg->next;
        while (seg->type == TR_STR) {
            length += seg->length;
            seg = seg->next;
        }
    } else {
        seg = seg->next;
        while (seg->type != TR_STR && seg->radius >= cRadius) {
            length += seg->length;
            seg = seg->next;
        }
    }

    return length;
}

double GeneratingTorcsData::getDistToSegEnd(tTrkLocPos trkPos) {
    tTrackSeg *seg = trkPos.seg;
    float dist;

    if (seg->type == TR_STR)
        dist = seg->length - trkPos.toStart;
    else
        dist = (seg->arc - trkPos.toStart) * seg->radius;

    return dist;
}

double GeneratingTorcsData::getNextRadius(tTrackSeg *seg) {
    // TR_RGT 1 Right curve
    // TR_LFT 2 Left curve
    // TR_STR 3 Straight

    float cRadius = seg->radius;
    float radius = 1e+6;

    if (seg->type == TR_STR) {
        while (seg->type == TR_STR)
            seg = seg->next;
        while (seg->type != TR_STR) {
            if (seg->radius < radius)
                radius = seg->radius;
            seg = seg->next;
        }
    } else {
        while (seg->type != TR_STR) {
            if (seg->radius < cRadius)
                radius = seg->radius;
            seg = seg->next;
        }
    }

    return radius;
}