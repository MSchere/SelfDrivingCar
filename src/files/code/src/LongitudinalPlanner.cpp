/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LongitudinalPlanner.cpp
 * Author: ruben
 * 
 * Created on 5 de octubre de 2016, 18:33
 */

#include "LongitudinalPlanner.h"
#include "src/drivers/Robot14/CarFollower.h"

LongitudinalPlanner::LongitudinalPlanner(tCarElt* car, tSituation *s, ContextInfo *context, driverParameters_t *driverParams) {

    m_car = car;
    m_s = s;
    m_context = context;
    m_driverParams = driverParams;

    m_speedBehaviour.clear();
    m_speedBehaviour.push_back(SpeedTime_t{0, 0, 0, 0, LP_DELTA});
    m_lastTime = 0.0;
    m_lastSpeed = 0.0;
    this->addSpeedProfileToSpeedTimeSeries(10.0, 100 / 3.6, LP_SLOPE);
    this->addSpeedProfileToSpeedTimeSeries(30.0, 120 / 3.6, LP_SLOPE);
    this->addSpeedProfileToSpeedTimeSeries(40.0, 160 / 3.6, LP_SLOPE);
    this->addSpeedProfileToSpeedTimeSeries(90.0, 120 / 3.6, LP_SLOPE);
    this->addSpeedProfileToSpeedTimeSeries(100.0, 60 / 3.6, LP_SLOPE);
    this->addSpeedProfileToSpeedTimeSeries(120.0, 120 / 3.6, LP_SLOPE);
    this->addSpeedProfileToSpeedTimeSeries(180.0, 200 / 3.6, LP_SLOPE);

    m_requestLane = LP_LANE_0;

    m_LC = new LongitudinalController(m_car);
    //m_IDM = new IDM(2, 3, 0, 4, 10, 1);
    m_follower = new CarFollower(10, 1, 2, 0.02);
}

LongitudinalPlanner::~LongitudinalPlanner() {
}

void LongitudinalPlanner::addSpeedProfileToSpeedTimeSeries(double endTime, double endSpeed, int mode) {

    if (mode == LP_DELTA || mode == LP_SLOPE) {
        int lastIndex = m_speedBehaviour.size() - 1;
        double startTime = m_speedBehaviour.at(lastIndex).endTime;
        double startSpeed = m_speedBehaviour.at(lastIndex).endSpeed;
        m_lastTime = endTime;
        m_lastSpeed = endSpeed;
        m_speedBehaviour.push_back(SpeedTime_t{startTime, startSpeed, m_lastTime, m_lastSpeed, mode});
    } else
        printf("Error: LongitudinalPlanner::addSpeedProfileToSpeedTimeSeries(), mode not match\n");
}

int LongitudinalPlanner::control() {

    m_myContext = m_context->getMyContextInfo();
    m_AllContext = m_context->getAllCarsContextInfo();
    m_relativePositions = m_context->getRelativePositions();
    m_surround = m_context->getSurround();

    // Update speed profile
    Speed_t desiredSpeed = this->getLongitudinalSpeedReference(m_context->getCurrentTime());
    m_driverParams->desiredSpeed = desiredSpeed.currentSpeed;
    double accel = m_follower->continueFollow(m_surround.center.front.dist, m_myContext.dynamics.speedX, m_surround.center.front.speed, 0.0, m_surround.center.front.car, desiredSpeed.currentSpeed);

    //    m_IDM->setIDMParameters(desiredSpeed.currentSpeed);
    //    IDM::IDM_t parametersMyLane = m_IDM->updateIDM(m_myContext.dynamics.speedX, m_surround.center.front.car, m_surround.center.front.dist, m_surround.center.front.speed, false, m_surround.center.rear.dist, m_surround.center.rear.speed);

    // Compute max speeds in each lane
    double leftSpeed = -1;
    if (m_surround.left.lane) {
        if (m_surround.left.front.car && m_surround.left.front.dist < 100)
            leftSpeed = m_surround.left.front.speed;
        else
            leftSpeed = desiredSpeed.currentSpeed;
    }
    if (leftSpeed > desiredSpeed.currentSpeed)
        leftSpeed = desiredSpeed.currentSpeed;

    double centerSpeed = desiredSpeed.finalSpeed;
    if (m_surround.center.front.car && m_surround.center.front.dist < 100)
        centerSpeed = m_surround.center.front.speed;
    if (centerSpeed > desiredSpeed.currentSpeed)
        centerSpeed = desiredSpeed.currentSpeed;

    double rightSpeed = -1;
    if (m_surround.right.lane) {
        if (m_surround.right.front.car && m_surround.right.front.dist < 100)
            rightSpeed = m_surround.right.front.speed;
        else
            rightSpeed = desiredSpeed.currentSpeed;
    }
    if (rightSpeed > desiredSpeed.currentSpeed)
        rightSpeed = desiredSpeed.currentSpeed;

    bool overTaken = this->evaluateOvertaken(centerSpeed, leftSpeed, m_driverParams);
    m_requestLane = m_surround.center.laneId;
    if (overTaken && leftSpeed > centerSpeed)
        m_requestLane = m_surround.left.laneId;
    if (rightSpeed >= centerSpeed)
        m_requestLane = m_surround.right.laneId;


    if (m_myContext.mainLaneId == 0)
        m_requestLane = 1;
    else
        m_requestLane = 0;

    //m_reference.reference = parametersMyLane.resultAccel;
    m_reference.reference = accel;
    m_reference.referenceType = LC_REF_TYPE_ACCEL;
    m_LC->control(m_reference, m_context->getCurrentTime());

    return m_requestLane;
}

Speed_t LongitudinalPlanner::getLongitudinalSpeedReference(double currentTime) {

    SpeedTime_t data;

    if (currentTime < 0)
        data = m_speedBehaviour.at(0);
    else if (currentTime > m_lastTime)
        data = m_speedBehaviour.at(m_speedBehaviour.size() - 1);
    else
        for (int i = 0; i < m_speedBehaviour.size(); i++)
            if (currentTime >= m_speedBehaviour.at(i).startTime && currentTime < m_speedBehaviour.at(i).endTime) {
                data = m_speedBehaviour.at(i);
                break;
            }

    if (data.mode == LP_DELTA) {
        m_currentSpeed = data.endSpeed;
    } else if (data.mode == LP_SLOPE) {
        if (currentTime > data.endTime) {
            m_currentSpeed = data.endSpeed;
        } else {
            double elapsedTime = currentTime - data.startTime;
            double slope = (data.endSpeed - data.startSpeed) / (data.endTime - data.startTime);
            m_currentSpeed = data.startSpeed + elapsedTime * slope;
        }
    } else
        printf("Modo de ajuste no valido\n");

    Speed_t speed;
    speed.currentSpeed = m_currentSpeed;
    speed.finalSpeed = data.endSpeed;

    return speed;
}

int LongitudinalPlanner::getBestLaneFronLP() {
    return m_requestLane;
}

bool LongitudinalPlanner::evaluateOvertaken(double currentLaneSpeed, double leftLaneSpeed, driverParameters_t *driverParams) {

    double val0 = 0;
    if (driverParams->desiredSpeed > 0) {
        val0 = (leftLaneSpeed - currentLaneSpeed) / driverParams->desiredSpeed;
        if (val0 < 0) val0 = 0;
        driverParams->overtaken.acuLostSpeed += val0;
    }

    if (currentLaneSpeed >= driverParams->desiredSpeed)
        driverParams->overtaken.acuLostSpeed = 0;

    double prob = driverParams->overtaken.ko0 * val0 * val0 + driverParams->overtaken.ko1 * driverParams->overtaken.acuLostSpeed;
    if (prob > driverParams->overtaken.probabilityThreshold)
        driverParams->overtaken.overtakenDecission = true;
    else
        driverParams->overtaken.overtakenDecission = false;

    return driverParams->overtaken.overtakenDecission = true;
}