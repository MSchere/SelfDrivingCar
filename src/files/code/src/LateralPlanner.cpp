/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LateralPlanner.cpp
 * Author: ruben
 * 
 * Created on 5 de octubre de 2016, 18:43
 */

#include "LateralPlanner.h"

LateralPlanner::LateralPlanner(tCarElt* car, tSituation *s, ContextInfo *context, driverParameters_t *driverParams) {
    m_car = car;
    m_s = s;
    m_context = context;
    m_driverParams = driverParams;

    m_myContext = m_context->getMyContextInfo();
    m_AllContext = m_context->getAllCarsContextInfo();
    m_relativePositions = m_context->getRelativePositions();
    m_surround = m_context->getSurround();
    m_laneChangeIsPending = false;
    m_firstIteration = true;
    m_LC = new LateralController(m_car, std::string("/tmp/torcs-1.3.7/controllers/Steering.fis"));
}

LateralPlanner::~LateralPlanner() {
}

void LateralPlanner::abortCurrentLaneChange() {
    m_laneChangeIsPending = false;
    m_currentReference = m_laneChange.startLatPos;
    m_driverParams->overtaken.overtakenDecission = false;
    m_driverParams->overtaken.acuLostSpeed = 0.0;
}

bool LateralPlanner::setBestLaneFromLP(int laneId, double lengthProcess, int typeProcess) {
    bool laneChangeAproved = true;

    if (m_laneChangeIsPending)
        laneChangeAproved = false;
    else {
        if (laneId < 0 || laneId > m_myContext.road.numberLanes - 1) {
            printf("Error: LateralPlanner::setBestLaneFromLP() laneID > 0 && laneID < numberLanes\n");
            laneChangeAproved = false;
        } else {
            if (laneId == m_myContext.mainLaneId)
                laneChangeAproved = false;
            else {
                for (int i = 0; i < m_relativePositions.size(); i++) {
                    if (m_relativePositions.at(i).mainLaneId == laneId || m_relativePositions.at(i).secondLaneId == laneId) {
                        if (m_relativePositions.at(i).usedDist < (2 * m_car->info.dimension.x) && m_relativePositions.at(i).usedDist > -(2 * m_car->info.dimension.x)) {
                            laneChangeAproved = false;
                        } else {
                            double t2c = m_relativePositions.at(i).usedDist / (m_s->cars[i]->pub.DynGC.vel.x - m_car->pub.DynGC.vel.x);
                            if (t2c > -10 && t2c < 0)
                                laneChangeAproved = false;
                        }
                    }
                }
            }
        }
    }

    if (laneChangeAproved) {
        printf("%s lane change aproved: %d\n",m_car->info.name, laneId);
        double lateralReference = m_myContext.road.laneCenter.at(laneId);
        if (typeProcess == LP_TP_DIST)
            this->requestParametricLaneChangeTypeDist(m_car->pub.DynGC.pos.x, m_car->pub.trkPos.toMiddle, m_car->pub.DynGC.pos.x + lengthProcess, lateralReference);
        else if (typeProcess == LP_TP_TIME)
            this->requestParametricLaneChangeTypeTime(m_context->getCurrentTime(), m_car->pub.trkPos.toMiddle, m_context->getCurrentTime() + lengthProcess, lateralReference);
    }

    return laneChangeAproved;
}

void LateralPlanner::requestParametricLaneChangeTypeDist(double startLonPos, double startLatPos, double endLonPos, double endLatPos) {

    m_laneChangeIsPending = true;

    m_laneChange.startEval = startLonPos;
    m_laneChange.startLatPos = startLatPos;
    m_laneChange.laneChangeStarted = false;

    m_laneChange.endEval = endLonPos;
    m_laneChange.endLatPos = endLatPos;
    m_laneChange.laneChangeFinished = false;

    double dist = endLonPos - startLonPos;
    m_laneChange.params[0] = startLatPos;
    m_laneChange.params[1] = 0;
    m_laneChange.params[2] = 0;
    m_laneChange.params[3] = 10 * (endLatPos - startLatPos) / pow(dist, 3.0);
    m_laneChange.params[4] = 15 * (startLatPos - endLatPos) / pow(dist, 4.0);
    m_laneChange.params[5] = 6 * (endLatPos - startLatPos) / pow(dist, 5.0);

    m_laneChange.typeProcess = LP_TP_DIST;
}

void LateralPlanner::requestParametricLaneChangeTypeTime(double startTime, double startLatPos, double endTime, double endLatPos) {
    m_laneChangeIsPending = true;

    m_laneChange.startEval = startTime;
    m_laneChange.startLatPos = startLatPos;
    m_laneChange.laneChangeStarted = false;

    m_laneChange.endEval = endTime;
    m_laneChange.endLatPos = endLatPos;
    m_laneChange.laneChangeFinished = false;

    double dist = endTime - startTime;
    m_laneChange.params[0] = startLatPos;
    m_laneChange.params[1] = 0;
    m_laneChange.params[2] = 0;
    m_laneChange.params[3] = 10 * (endLatPos - startLatPos) / pow(dist, 3.0);
    m_laneChange.params[4] = 15 * (startLatPos - endLatPos) / pow(dist, 4.0);
    m_laneChange.params[5] = 6 * (endLatPos - startLatPos) / pow(dist, 5.0);

    m_laneChange.typeProcess = LP_TP_TIME;
}

void LateralPlanner::control(int desiredLaneId, double lengthProcess, int typeProcess) {

    m_myContext = m_context->getMyContextInfo();
    m_AllContext = m_context->getAllCarsContextInfo();
    m_relativePositions = m_context->getRelativePositions();
    m_surround = m_context->getSurround();

    if (m_firstIteration) {
        m_firstIteration = false;
        m_currentReference = m_car->pub.trkPos.toMiddle;
        double lateralReference = m_myContext.road.laneCenter.at(m_myContext.mainLaneId);
        this->requestParametricLaneChangeTypeTime(m_context->getCurrentTime(), m_myContext.dist2RoadCenter, m_context->getCurrentTime() + 5.0, lateralReference);
    }
    
    this->setBestLaneFromLP(desiredLaneId, lengthProcess, typeProcess);

    if (m_laneChangeIsPending) {
        double evalVar;
        if (m_laneChange.typeProcess == LP_TP_DIST)
            evalVar = m_car->pub.DynGC.pos.x;
        else if (m_laneChange.typeProcess == LP_TP_TIME) {
            evalVar = m_context->getCurrentTime();
        }

        m_currentReference = m_laneChange.startLatPos;

        if (!m_laneChange.laneChangeStarted && evalVar >= m_laneChange.startEval) {
            m_laneChange.laneChangeStarted = true;
            m_currentReference = m_laneChange.startLatPos;
        }
        if (!m_laneChange.laneChangeFinished && evalVar >= m_laneChange.endEval) {
            m_laneChange.laneChangeFinished = true;
            m_currentReference = m_laneChange.endLatPos;
        }

        if (m_laneChange.laneChangeStarted && !m_laneChange.laneChangeFinished) {
            double dist = evalVar - m_laneChange.startEval;
            m_currentReference = this->evalParams(dist);
        }

        if (m_laneChange.laneChangeFinished) {
            m_laneChangeIsPending = false;
            m_driverParams->overtaken.overtakenDecission = false;
            m_driverParams->overtaken.acuLostSpeed = 0.0;
            m_currentReference = m_laneChange.endLatPos;
        }
    }

    m_LC->control(m_currentReference);
}

bool LateralPlanner::laneChangePending() {
    return m_laneChangeIsPending;
}

double LateralPlanner::evalParams(double dist) {
    double acu = 0;
    acu += m_laneChange.params[0];
    acu += m_laneChange.params[1] * dist;
    acu += m_laneChange.params[2] * pow(dist, 2.0);
    acu += m_laneChange.params[3] * pow(dist, 3.0);
    acu += m_laneChange.params[4] * pow(dist, 4.0);
    acu += m_laneChange.params[5] * pow(dist, 5.0);
    return acu;
}