

#ifndef CPUTIME_H
#define	CPUTIME_H

#include <string>
#include <x86_64-linux-gnu/sys/time.h>


using namespace std;


class CpuTime {
    
    
public:
    CpuTime();
    CpuTime(string logFile);
    CpuTime(const CpuTime& orig);
    virtual ~CpuTime();

    void start();
    int stop(string logInfo);
    void stop();
    string getText();

    double getSs() const {
        return m_ss;
    }

    int getMm() const {
        return m_mm;
    }

    int getHh() const {
        return m_hh;
    }
   
    
    
private:

    
    string m_logFile;
    struct timeval m_timeSt, m_timeEnd;
    int m_hh;
    int m_mm;
    double m_ss;
    

};

#endif	/* CPUTIME_H */

