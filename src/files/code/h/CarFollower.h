/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CarFollower.h
 * Author: ruben
 *
 * Created on 23 de noviembre de 2016, 17:23
 */

#ifndef CARFOLLOWER_H
#define CARFOLLOWER_H

#include <math.h>

#define MAX_ACCEL_CF    +2.0
#define MIN_ACCEL_CF    -2.0
#define TIME_GAP_CF     +10.0

class CarFollower {
public:

    CarFollower(double r, double th, double dT, double sampleTime);
    virtual ~CarFollower();
    
    double restartFollow(double dist, double speedHost, double speedLeader, double accelLeader, bool car, double desiredSpeed);
    double continueFollow(double dist, double speedHost, double speedLeader, double accelLeader, bool car, double desiredSpeed);


    double getLastDistanceToCar() {
        return m_distance;
    }

    double getDesiredDistance() {
        return m_desiredDistance;
    }

    double getLastAccel() {
        return m_lastAccel;
    }

private:

    double m_r, m_th, m_dT, m_carLength, m_sampleTime;

    bool m_integratingEnd;
    double m_r_integrating, m_dr_integrating;

    double m_dirNorth, m_dirEast;
    double m_distance;
    double m_desiredDistance;

    double m_lastAccel;
};

#endif /* CARFOLLOWER_H */

