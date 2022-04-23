/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Fuzzy.h
 * Author: ruben
 *
 * Created on 18 de octubre de 2016, 11:25
 */

#ifndef FUZZY_H
#define FUZZY_H

#include <string>
#include <vector>
#include "fis.h"

class Fuzzy {
public:
    Fuzzy(std::string fisPath);
    virtual ~Fuzzy();
    
    std::vector<double> evalFis(std::vector<double> input);
    
private:
    
    char *m_fisFile;
    FIS* m_fis;
    double **m_inputs, **m_outputs;
    std::vector<double> m_vectorInput, m_vectorOutput;

};

#endif /* FUZZY_H */

