/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   IDM.cpp
 * Author: ruben
 * 
 * Created on 21 de octubre de 2016, 8:36
 */

#include "IDM.h"

IDM::IDM(double accel, double decel, double speed, double exp, double r, double th) {
    m_model = IDM_t{accel, decel, speed, exp, r, th, 0, 0, 0, 0};
}

IDM::~IDM() {
}

IDM::IDM_t IDM::updateIDM(double speed, bool carInfront, double frontDist, double frontSpeed, bool carInBack, double backDist, double backSpeed) {

    m_model.term1 = 0;
    if (m_model.speed > 0)
        m_model.term1 = pow(speed / m_model.speed, m_model.exp);

    m_model.term2 = 0;
    double relativeSpeedFront = speed - frontSpeed;
    if (carInfront && relativeSpeedFront > 0) {
        m_model.term2 = m_model.r + m_model.th * speed + (speed * relativeSpeedFront) / (2 * sqrt(m_model.accel * m_model.decel));
        m_model.term2 = pow(m_model.term2 / frontDist, 2.0);
    }

    m_model.term3 = 0;
    double relativeSpeedBack = backSpeed - speed;
    if (carInBack) {
        m_model.term3 = m_model.r + m_model.th * speed + (speed * relativeSpeedBack) / (2 * sqrt(m_model.accel * m_model.decel));
        m_model.term3 = pow(m_model.term3 / (-backDist), 2.0);
    }

    m_model.resultAccel = m_model.accel * (1 - m_model.term1 - m_model.term2 + m_model.term3);
    return m_model;
}

IDM::IDM_t IDM::getIDMParameters() {
    return m_model;
}

IDM::IDM_t IDM::setIDMParameters(double speed) {
    m_model.speed = speed;
    return m_model;
}

IDM::IDM_t IDM::setIDMParameters(double accel, double decel, double speed, double exp, double r, double th) {
    m_model = IDM_t{accel, decel, speed, exp, r, th, 0, 0, 0};
    return m_model;
}
