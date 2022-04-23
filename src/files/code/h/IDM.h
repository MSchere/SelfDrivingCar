/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   IDM.h
 * Author: ruben
 *
 * Created on 21 de octubre de 2016, 8:36
 */

#ifndef IDM_H
#define IDM_H

#include <stdio.h>
#include <math.h>

class IDM {
public:

    typedef struct {
        double accel;
        double decel;
        double speed;
        double exp;
        double r;
        double th;
        double term1;
        double term2;
        double term3;
        double resultAccel;
    } IDM_t;

    IDM(double accel, double decel, double speed, double exp, double r, double th);
    virtual ~IDM();

    IDM_t updateIDM(double speed, bool carInfront, double distFront, double speedFront, bool carInBack, double distBack, double speedBack);

    IDM_t getIDMParameters();
    IDM_t setIDMParameters(double speed);
    IDM_t setIDMParameters(double accel, double decel, double speed, double exp, double r, double th);

private:

    IDM_t m_model;

};

#endif /* IDM_H */

