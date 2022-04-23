/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TorcsLogger.h
 * Author: ruben
 *
 * Created on 6 de octubre de 2016, 13:36
 */

#ifndef TORCSLOGGER_H
#define TORCSLOGGER_H

#include <time.h>
#include <stdio.h>
#include <math.h>
#include "Definitions.h"

class TorcsLogger {
public:
    TorcsLogger(char* logFileName, int robotNumber);
    virtual ~TorcsLogger();
    
    void newEntry(carData_t carData, contextInfo_t contextInfo);
    
private:

    FILE* m_logFile;
    

};

#endif /* TORCSLOGGER_H */

