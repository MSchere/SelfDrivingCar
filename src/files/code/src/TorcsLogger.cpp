/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TorcsLogger.cpp
 * Author: ruben
 * 
 * Created on 6 de octubre de 2016, 13:36
 */

#include "TorcsLogger.h"

TorcsLogger::TorcsLogger(char* logFileName, int robotNumber) {
    time_t aux = time(NULL);
    struct tm *cT = localtime(&aux);
    int year = cT->tm_year + 1900;
    int month = cT->tm_mon + 1;
    int day = cT->tm_mday;
    int hour = cT->tm_hour;
    int min = cT->tm_min;

    char path[500];
    sprintf(path, "%s/%04d_%02d_%02d_%02d_%02d_Robot%d.txt", logFileName, year, month, day, hour, min, robotNumber);
    FILE* auxFile = fopen(path, "w+");

    if (auxFile == NULL) {
        printf("Error creando archivo:\n");
        printf("%s\n", path);
    } else
        this->m_logFile = auxFile;
}

TorcsLogger::~TorcsLogger() {
    fclose(m_logFile);
}

void TorcsLogger::newEntry(carData_t carData, contextInfo_t contextInfo) {
    char logData[1000];

    if (contextInfo.road.numberLanes == 2) {
        sprintf(logData, "%+.10e %+.10e %+.10e %+.10e %+.10e %+.10e %+.10e %+.10e %+.10e %+.10e",
                contextInfo.distRaced,
                contextInfo.dist2RightLimit,
                contextInfo.dynamics.headingError,
                contextInfo.dynamics.speedX,
                contextInfo.dynamics.accelX,
                contextInfo.road.laneStartFromRightLimit.at(0),
                contextInfo.road.laneCenterFromRightLimit.at(0),
                contextInfo.road.laneEndFromRightLimit.at(0),
                contextInfo.road.laneCenterFromRightLimit.at(1),
                contextInfo.road.laneEndFromRightLimit.at(1)
                );
        fprintf(m_logFile, "%s\n", logData);
    }
}