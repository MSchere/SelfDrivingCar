/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   NormalDistribution.cpp
 * Author: ruben
 * 
 * Created on 21 de noviembre de 2016, 12:54
 */

#include "NormalDistribution.h"

NormalDistribution::NormalDistribution() {
    m_n2 = 0.0;
    m_n2_cached = 0;
}

NormalDistribution::~NormalDistribution() {
}

double NormalDistribution::getNumber(double mean, double stddev, int normalType) {

    double result;

    if (!m_n2_cached) {
        double x, y, r;
        do {
            x = 2.0 * rand() / RAND_MAX - 1.0;
            y = 2.0 * rand() / RAND_MAX - 1.0;
            r = x * x + y * y;
        } while (r == 0 || r > 1.0);
        double d = sqrt(-2.0 * log(r) / r);
        double n1 = x * d;
        m_n2 = y * d;
        if (normalType == ND_NT_GAUSS)
            result = mean + n1 * stddev;
        if (normalType == ND_NT_GAUSS_POS_STD)
            result = mean + fabs(n1 * stddev);
        if (normalType == ND_NT_GAUSS_NEG_STD)
            result = mean - fabs(n1 * stddev);
        m_n2_cached = 1;
    } else {
        m_n2_cached = 0;
        if (normalType == ND_NT_GAUSS)
            result = mean + m_n2 * stddev;
        if (normalType == ND_NT_GAUSS_POS_STD)
            result = mean + fabs(m_n2 * stddev);
        if (normalType == ND_NT_GAUSS_NEG_STD)
            result = mean - fabs(m_n2 * stddev);
    }

    return result;
}

double NormalDistribution::getNumberLimited(double mean, double stddev, int normalType, double min, double max){
    double val = this->getNumber(mean, stddev, normalType);
    if(val > max) val = max;
    if(val < min) val = min;
    return val;
}

bool NormalDistribution::evaluateBinaryProbability(double trueProbability) {
    bool result = false;

    if (trueProbability >= 1.0)
        result = true;
    else if (trueProbability <= 0.0)
        result = false;
    else {
        double prob = double(rand()) / double(RAND_MAX);
        if (prob <= trueProbability)
            result = true;
        else
            result = false;
    }

    return result;
}