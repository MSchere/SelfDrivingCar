/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Robot.h
 * Author: ruben
 *
 * Created on 18 de octubre de 2016, 12:02
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

#include <stdlib.h>
#include "Driver.h"

/* Functions definition */
static int InitFuncPt(int index, void *pt);
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newrace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
void initRobotIdentity(int id, tModInfo *modInfo);

/* Class */
Driver* myDriver;


#endif /* ROBOT_H */