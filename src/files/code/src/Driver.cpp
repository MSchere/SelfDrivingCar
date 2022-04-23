/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Driver.cpp
 * Author: ruben
 * 
 * Created on 9 de noviembre de 2016, 11:51
 */

#include "Driver.h"

Driver::Driver(tCarElt *car, tSituation *s) {

    m_car = car;
    m_s = s;

    m_driverParams = this->buildNewDriver(m_ND.getNumberLimited(0, 0.2, ND_NT_GAUSS, 0, 1));

    m_robotId = car->index;
    m_log = new TorcsLogger("/home/ruben/Dropbox/Doctorado/Thesis/programacion_robots_torcs/files/logsFiles", car->index);
    m_torcsData = new GeneratingTorcsData;

    m_context = new ContextInfo(m_car, m_s);
    m_lateralPlanner = new LateralPlanner(m_car, m_s, m_context, &m_driverParams);

    m_longitudinalPlanner = new LongitudinalPlanner(m_car, m_s, m_context, &m_driverParams);
}

Driver::~Driver() {
}

void Driver::drive() {

    /* ---------------------------------------------------------------------------------- */

    /* ---------------------------------------------------------------------------------- */

    /* ---------------------------------------------------------------------------------- */
    m_context->updateContext();
    m_torcsData->collectData(m_car, m_s);
    m_log->newEntry(m_torcsData->getMyCarData(), m_context->getMyContextInfo());
    /* ---------------------------------------------------------------------------------- */

    /* ---------------------------------------------------------------------------------- */
    int desiredLane = m_longitudinalPlanner->control();
    double randomTime = m_ND.getNumberLimited(10, 5, ND_NT_GAUSS, 5, 20);
    if (m_context->getCurrentTime() > 0 && m_context->getCurrentTime() < 100)
        m_lateralPlanner->control(0, randomTime, LP_TP_TIME);
    else if (m_context->getCurrentTime() > 100 && m_context->getCurrentTime() < 180)
        m_lateralPlanner->control(1, randomTime, LP_TP_TIME);
    else
        m_lateralPlanner->control(desiredLane, randomTime, LP_TP_TIME);




    /* ---------------------------------------------------------------------------------- */
}

driverParameters_t Driver::buildNewDriver(double dangerous) {

    if (dangerous > 1) dangerous = 1;
    if (dangerous < 0) dangerous = 0;

    driverParameters_t parameters;

    //    m_driverParams.desiredSpeed = m_ND.getNumber(120.0, 20.0, ND_NT_GAUSS) / 3.6;
    //    m_driverParams.overtaken.acuLostSpeed = 0.0;
    //    m_driverParams.ko0 = 2.0;
    //    m_driverParams.ko1 = 1.0 / (90.0 / 120.0 * 50.0 * 20.0);
    //    m_driverParams.overtakenDecission = false;
    //    m_driverParams.probabilityOfOvertaken = 0.0;

    parameters.desiredSpeed = m_ND.getNumberLimited(100.0 + dangerous * 40.0, 10.0, ND_NT_GAUSS, 60.0, 200.0) / 3.6;
    parameters.accelParameter = m_ND.getNumberLimited(1.0 + dangerous * 2.0, 1.0, ND_NT_GAUSS, 1.0, 6.0);
    parameters.frontWatchingLenght = m_ND.getNumberLimited(100.0 + dangerous * 20.0, 10.0, ND_NT_GAUSS, 50.0, 200.0);
    parameters.rearWatchingLenght = m_ND.getNumberLimited(100.0 + dangerous * 20.0, 10.0, ND_NT_GAUSS, 50.0, 200.0);
    parameters.overtaken.acuLostSpeed = 0;
    parameters.overtaken.ko0 = m_ND.getNumberLimited(1.0 + dangerous, 0.1, ND_NT_GAUSS, 0.1, 2.0);
    parameters.overtaken.ko1 = m_ND.getNumberLimited(0.01 + dangerous * 0.01, 0.001, ND_NT_GAUSS, 0.001, 0.1);
    parameters.overtaken.overtakenDecission = false;
    parameters.overtaken.probabilityThreshold = m_ND.getNumberLimited(1.0 - dangerous, 0.1, ND_NT_GAUSS, 0.1, 1.0);

    return parameters;
}