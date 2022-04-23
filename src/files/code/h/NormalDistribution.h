/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   NormalDistribution.h
 * Author: ruben
 *
 * Created on 21 de noviembre de 2016, 12:54
 */

#ifndef NORMALDISTRIBUTION_H
#define NORMALDISTRIBUTION_H

#include <cstdlib>
#include <cmath>

#define ND_NT_GAUSS             0
#define ND_NT_GAUSS_POS_STD     1
#define ND_NT_GAUSS_NEG_STD     2

class NormalDistribution {
public:
    NormalDistribution();
    virtual ~NormalDistribution();

    double getNumber(double mean, double stddev, int normalType);
    double getNumberLimited(double mean, double stddev, int normalType, double min, double max);

    bool evaluateBinaryProbability(double trueProbability);

private:

    double m_result, m_n2;
    int m_n2_cached;

};

#endif /* NORMALDISTRIBUTION_H */

