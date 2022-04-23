/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Fuzzy.cpp
 * Author: ruben
 * 
 * Created on 18 de octubre de 2016, 11:25
 */

#include <vector>

#include "Fuzzy.h"
#include "fis.c"

Fuzzy::Fuzzy(std::string fisPath) {
    m_fisFile = strdup(fisPath.c_str()); //fisPath.c_str();  "/media/ruben/TESIS_1TB/TORCS/FisControllers/Steering.fis";

    /* obtenemos la matriz del fichero FIS */
    int fis_row_n, fis_col_n;
    double **fisMatrix = returnFismatrix(m_fisFile, &fis_row_n, &fis_col_n);
    /* construimos la estructura de datos tipo FIS */
    m_fis = (FIS*) fisCalloc(1, sizeof (FIS));
    fisBuildFisNode(m_fis, fisMatrix, fis_col_n, MF_POINT_N);
    /* creamos los vectores/matrices que albergan los datos de entrada/salida */
    m_inputs = (double**) fisCreateMatrix(1, m_fis->in_n, sizeof (double));
    m_outputs = (double**) fisCreateMatrix(1, m_fis->out_n, sizeof (double));
}

Fuzzy::~Fuzzy() {
}

std::vector<double> Fuzzy::evalFis(std::vector<double> input) {
    if (input.size() == m_fis->in_n) {
        this->m_vectorInput = input;
        for (int i = 0; i < input.size(); i++)
            m_inputs[0][i] = input.at(i);
        getFisOutput(m_inputs[0], m_fis, m_outputs[0]);
        m_vectorOutput.clear();
        for(int i = 0; i < m_fis->out_n; i++)
            m_vectorOutput.push_back(m_outputs[0][i]);
    }
    return m_vectorOutput;
}