/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LateralController.h
 * Author: ruben
 *
 * Created on 17 de octubre de 2016, 17:30
 */

#ifndef LATERALCONTROLLER_H
#define LATERALCONTROLLER_H

#include <robottools.h>
#include <robot.h> 

#include "fis.h"
#include "Fuzzy.h"


class LateralController {
public:
    LateralController(tCarElt* car, std::string fisPath);
    virtual ~LateralController();
    
    void control(float lateralReference);
    
private:

    Fuzzy* m_fuzzySteer;
    std::vector<double> m_fuzzyInput, m_fuzzyOutput;
    double m_targetSteer, m_forwardSteer;

    tCarElt* m_car;
    
};

#endif /* LATERALCONTROLLER_H */

