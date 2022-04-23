/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LateralController.cpp
 * Author: ruben
 * 
 * Created on 17 de octubre de 2016, 17:30
 */

#include "LateralController.h"

LateralController::LateralController(tCarElt* car, std::string fisPath) {
    m_car = car;
    m_fuzzySteer = new Fuzzy(fisPath);
}

LateralController::~LateralController() {
}

void LateralController::control(float lateralReference) {
    /* Fuzzy STEERING Controller Inputs-------------------------------------------------- */
    m_fuzzyInput.clear();
    double angularError = RtTrackSideTgAngleL(&(m_car->pub.trkPos)) - m_car->pub.DynGC.pos.az;
    NORM_PI_PI(angularError); // put the angle back in the range from -PI to PI
    m_fuzzyInput.push_back(angularError * 180 / M_PI);
    double lateralError = lateralReference - m_car->pub.trkPos.toMiddle;
    lateralError = double(round(lateralError * 5)) / 5.0;  // ERROR DE APRECIACION
    m_fuzzyInput.push_back(lateralError);
    /* ---------------------------------------------------------------------------------- */

    m_fuzzyOutput = m_fuzzySteer->evalFis(m_fuzzyInput);
    this->m_targetSteer = m_fuzzyOutput.at(0);

    /* Correcting steer with the predefined steer value in curve------------------------- */
    m_forwardSteer = 0.0;
    if (m_car->pub.trkPos.seg->type != TR_STR) {
        double axelDist = m_car->priv.wheel[FRNT_LFT].relPos.x - m_car->priv.wheel[REAR_LFT].relPos.x;
        double steerLock = m_car->info.steerLock;
        printf("%lf %lf\n", axelDist, steerLock);
        double offset;
        if (m_car->pub.trkPos.seg->type == TR_RGT)
            offset = m_car->pub.trkPos.toMiddle;
        else
            offset = -m_car->pub.trkPos.toMiddle;
        double alpha = atan2(axelDist, m_car->pub.trkPos.seg->radius + offset);
        m_forwardSteer = alpha / steerLock;
        if (m_car->pub.trkPos.seg->type == TR_RGT)
            m_forwardSteer = -m_forwardSteer;
    }
    /* ---------------------------------------------------------------------------------- */

    /* Actuate over the vehicle */
    m_car->_steerCmd = m_forwardSteer + m_targetSteer;
}

