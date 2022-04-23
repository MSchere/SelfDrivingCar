/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CarFollower.cpp
 * Author: ruben
 * 
 * Created on 23 de noviembre de 2016, 17:23
 */

#include "CarFollower.h"

CarFollower::CarFollower(double r, double th, double dT, double sampleTime) {
    m_r = r;
    m_th = th;
    m_dT = dT;
    m_integratingEnd = true;
    m_r_integrating = r;
    m_dr_integrating = 0.0;
    m_sampleTime = sampleTime;
    m_lastAccel = 0.0;
}

CarFollower::~CarFollower() {
}

double CarFollower::restartFollow(double dist, double speedHost, double speedLeader, double accelLeader, bool car, double desiredSpeed) {

    m_distance = dist;
    m_r_integrating = dist - m_th * speedHost;
    m_dr_integrating = (m_r - m_r_integrating) / (TIME_GAP_CF / m_sampleTime);
    m_integratingEnd = false;

    m_lastAccel = this->continueFollow(dist, speedHost, speedLeader, accelLeader, car, desiredSpeed);

    return m_lastAccel;
}

double CarFollower::continueFollow(double dist, double speedHost, double speedLeader, double accelLeader, bool car, double desiredSpeed) {

    m_distance = dist;
    double targetAccel = 0.0;

    if (car) {
        double used_dT = m_dT;
        if (speedHost >= speedLeader) {
            double min_dT = (speedLeader - speedHost) / MIN_ACCEL_CF;
            if (min_dT > m_dT) used_dT = min_dT;
        }


        if (m_integratingEnd == false) {
            m_r_integrating += m_dr_integrating;
            if (fabs(m_r_integrating - m_r) <= m_dr_integrating) {
                m_r_integrating = m_r;
                m_integratingEnd = true;
            }
            m_desiredDistance = m_r_integrating + m_th * speedHost;
            targetAccel += (m_distance - m_r_integrating) / (used_dT * (m_th + used_dT / 2.0));
        } else {
            m_desiredDistance = m_r + m_th * speedHost;
            targetAccel += (m_distance - m_r) / (used_dT * (m_th + used_dT / 2.0));
        }

        targetAccel += (speedLeader - speedHost * (1.0 + m_th / used_dT)) / (m_th + used_dT / 2.0);
        targetAccel += (accelLeader * used_dT) / (2.0 * m_th + used_dT);
    }
    else {
        if(desiredSpeed > 0)
        targetAccel = MAX_ACCEL_CF * (1 - pow(speedHost / desiredSpeed, 2.0));
    }

    if (targetAccel > MAX_ACCEL_CF) targetAccel = MAX_ACCEL_CF;
    if (targetAccel < MIN_ACCEL_CF) targetAccel = MIN_ACCEL_CF;

    m_lastAccel = targetAccel;

    return m_lastAccel;
}
