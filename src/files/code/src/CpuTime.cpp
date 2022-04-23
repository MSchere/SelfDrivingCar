

#include "CpuTime.h"
#include <stdio.h>



CpuTime::CpuTime() {

}

CpuTime::CpuTime(const CpuTime& orig) {

}

CpuTime::~CpuTime() {
}

CpuTime::CpuTime(string logFile) {
    m_logFile = logFile;
}

void CpuTime::start() {
    gettimeofday(&m_timeSt, NULL);
}

string CpuTime::getText() {
    char txt[100];        
    sprintf(txt, "%02d:%02d:%.6f",m_hh,m_mm,m_ss);
    return string(txt);
}

int CpuTime::stop(string logInfo) {
    int ret = 0;
    char msg[500];
    
    stop();
    
    //write info to log file
    FILE *fd = fopen(m_logFile.data(), "a");

    if (fd != NULL) {
        fprintf(fd, "%s %s\n",logInfo.data(),getText().data());
        fclose(fd);
    }
    else {
        ret = -1;
        printf("Error writing CpuTime info\n");
    }
    return ret;
}

void CpuTime::stop() {
    //calculate elapsed time
    gettimeofday(&m_timeEnd, NULL);

    
    double total = (m_timeEnd.tv_sec-m_timeSt.tv_sec) + (m_timeEnd.tv_usec-m_timeSt.tv_usec)*1e-6;
    int aux = (int) total;

    m_hh = aux / 3600;
    aux = aux % 3600;

    m_mm = aux / 60;
    aux = aux % 60;

    m_ss = total - (3600 * m_hh + 60 * m_mm);
}