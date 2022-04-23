/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LongitudinalController.cpp
 * Author: ruben
 * 
 * Created on 6 de octubre de 2016, 13:27
 */

#include "LongitudinalController.h"

LongitudinalController::LongitudinalController(tCarElt* car) {
    m_car = car;
    m_k00 = 0.20;
    m_k01 = 0.00;
    m_k10 = 0.20;
    m_k11 = 0.04;
    m_lastUpdateTime = 0.0;
    m_lastUpdateAccel = false;
}

LongitudinalController::~LongitudinalController() {
}

void LongitudinalController::control(reference_t reference, double time) {

    double u;

    if (reference.referenceType == LC_REF_TYPE_SPEED) {
        u = this->controlSpeed(reference.reference);
        m_lastUpdateAccel = false;
    } else if (reference.referenceType == LC_REF_TYPE_ACCEL) {
        u = this->controlAccel(reference.reference, time);
        m_lastUpdateAccel = true;
    } else
        printf("Tipo de referencia erronea en funcion LongitudinalController::control\n");

    if (u >= 0) {
        m_car->ctrl.accelCmd = u;
        m_car->ctrl.brakeCmd = 0;
    } else {
        u = -u;
        m_car->ctrl.accelCmd = 0;
        m_car->ctrl.brakeCmd = u;
    }
    m_car->ctrl.gear = this->getGear();
    m_lastUpdateTime = time;
}

void LongitudinalController::changeControlConstants(double k00, double k01, double k10, double k11) {
    if (k00 == 0.0 && k01 == 0.0)
        printf("Constantes k00 y k01 no puede tomar valor 0 simultaneamente\n");
    else {
        m_k00 = k00;
        m_k01 = k01;
    }
    if (k10 == 0.0 && k11 == 0.0)
        printf("Constantes k10 y k11 no puede tomar valor 0 simultaneamente\n");
    else {
        m_k10 = k10;
        m_k11 = k11;
    }
}

double LongitudinalController::controlSpeed(double referenceSpeed) {
    double error = referenceSpeed - m_car->pub.DynGC.vel.x;
    return this->evalPA(error, m_car->pub.DynGC.vel.x);
}

double LongitudinalController::controlAccel(double referenceAccel, double time) {
    if (!m_lastUpdateAccel) {
        m_referenceSpeedForAccelControl = m_car->pub.DynGC.vel.x;
        m_lastUpdateTime = time;
    }

    double controlTime = time - m_lastUpdateTime;
    double speedIncrement = referenceAccel * controlTime;
    m_referenceSpeedForAccelControl += speedIncrement;

    double upperLimit = this->upperLimitForSpeedReference(m_car->pub.DynGC.vel.x);
    double lowerLimit = this->lowerLimitForSpeedReference(m_car->pub.DynGC.vel.x);
    if (m_referenceSpeedForAccelControl > upperLimit)
        m_referenceSpeedForAccelControl = upperLimit;
    else if (m_referenceSpeedForAccelControl < lowerLimit)
        m_referenceSpeedForAccelControl = lowerLimit;

    double error = m_referenceSpeedForAccelControl - m_car->pub.DynGC.vel.x;
    return this->evalPA(error, m_car->pub.DynGC.vel.x);
    ;
}

double LongitudinalController::evalPA(double error, double speed) {
    double k;
    if (error > 0)
        k = (m_k10 + m_k11 * speed);
    else
        k = (m_k00 + m_k01 * speed);

    double u = k * error;
    if (u > +1) u = +1;
    if (u < -1) u = -1;
    return u;
}

double LongitudinalController::upperLimitForSpeedReference(double speed) {
    double upperLimit = speed + 1 / (m_k10 + m_k11 * speed);
    return upperLimit;
}

double LongitudinalController::lowerLimitForSpeedReference(double speed) {
    double lowerLimit = speed - 1 / (m_k00 + m_k01 * speed);
    return lowerLimit;
}

int LongitudinalController::getGear() {

    const float SHIFT = 0.9; /* [-] (% of rpmredline) */
    const float SHIFT_MARGIN = 4.0; /* [m/s] */

    if (m_car->_gear <= 0) return 1;
    float gr_up = m_car->_gearRatio[m_car->_gear + m_car->_gearOffset];
    float omega = m_car->_enginerpmRedLine / gr_up;
    float wr = m_car->_wheelRadius(2);

    if (omega * wr * SHIFT < m_car->_speed_x) {
        return m_car->_gear + 1;
    } else {
        float gr_down = m_car->_gearRatio[m_car->_gear + m_car->_gearOffset - 1];
        omega = m_car->_enginerpmRedLine / gr_down;
        if (m_car->_gear > 1 && omega * wr * SHIFT > m_car->_speed_x + SHIFT_MARGIN) {
            return m_car->_gear - 1;
        }
    }
    return m_car->_gear;
}


