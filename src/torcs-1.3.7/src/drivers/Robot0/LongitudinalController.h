/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LongitudinalController.h
 * Author: ruben
 *
 * Created on 6 de octubre de 2016, 13:27
 */

#ifndef LONGITUDINALCONTROLLER_H
#define LONGITUDINALCONTROLLER_H

#include <car.h>

#define LC_REF_TYPE_SPEED 0
#define LC_REF_TYPE_ACCEL 1

class LongitudinalController {
public:

    typedef struct {
        double reference;
        int referenceType;
    } reference_t;

    LongitudinalController(tCarElt* car);
    virtual ~LongitudinalController();

    void control(reference_t reference, double time);
    void changeControlConstants(double k00, double k01, double k10, double k11);

private:

    double controlSpeed(double referenceSpeed);
    double controlAccel(double referenceAccel, double time);
    double evalPA(double error, double speed);
    double upperLimitForSpeedReference(double speed);
    double lowerLimitForSpeedReference(double speed);

    int getGear();


    double m_k00, m_k01, m_k10, m_k11;

    double m_referenceSpeedForAccelControl;
    double m_lastUpdateTime;

    bool m_lastUpdateAccel;
    
    tCarElt* m_car;

};

#endif /* LONGITUDINALCONTROLLER_H */

