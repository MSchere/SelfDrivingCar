/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   fis.h
 * Author: ruben
 *
 * Created on 6 de octubre de 2016, 11:02
 */

#ifndef FIS_H
#define FIS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/***********************************************************************
 Macros and definitions
 **********************************************************************/
/* Define portable printf and double */
#if defined(MATLAB_MEX_FILE)
#define PRINTF mexPrintf
#define DOUBLE real_T
#elif defined(__SIMSTRUC__)
#define PRINTF ssPrintf
#define DOUBLE real_T
#else
#define PRINTF printf
#define DOUBLE double
#endif

#ifndef ABS
#define ABS(x)   ( (x) > (0) ? (x): (-(x)) )
#endif
#ifndef MAX
#define MAX(x,y) ( (x) > (y) ? (x) : (y) )
#endif
#ifndef MIN
#define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#endif
#define MF_PARA_N 4
#define STR_LEN 500
#define MF_POINT_N 101

/* debugging macros */
/*
#define PRINT(expr) printf(#expr " = %g\n", (double)expr)
#define PRINTMAT(mat,m,n) printf(#mat " = \n"); fisPrintMatrix(mat,m,n)
#define FREEMAT(mat,m) printf("Free " #mat " ...\n"); fisFreeMatrix(mat,m)
#define FREEARRAY(array) printf("Free " #array " ...\n"); free(array)
 */

#if (defined(MATLAB_MEX_FILE) && !defined(__SIMSTRUC__))
#define FREE mxFree
#else
#define FREE free
#endif

#define FREEMAT(mat,m) fisFreeMatrix(mat,m)
#define FREEARRAY(array) FREE(array)

/***********************************************************************
 Data types
 **********************************************************************/

typedef struct fis_node {
    int handle;
    int load_param;
    char name[STR_LEN];
    char type[STR_LEN];
    char andMethod[STR_LEN];
    char orMethod[STR_LEN];
    char impMethod[STR_LEN];
    char aggMethod[STR_LEN];
    char defuzzMethod[STR_LEN];
    int userDefinedAnd;
    int userDefinedOr;
    int userDefinedImp;
    int userDefinedAgg;
    int userDefinedDefuzz;
    int in_n;
    int out_n;
    int rule_n;
    int **rule_list;
    DOUBLE *rule_weight;
    int *and_or; /* AND-OR indicator */
    DOUBLE *firing_strength;
    DOUBLE *rule_output;
    /* Sugeno: output for each rules */
    /* Mamdani: constrained output MF values of rules */
    struct io_node **input;
    struct io_node **output;
    DOUBLE(*andFcn)(DOUBLE, DOUBLE);
    DOUBLE(*orFcn)(DOUBLE, DOUBLE);
    DOUBLE(*impFcn)(DOUBLE, DOUBLE);
    DOUBLE(*aggFcn)(DOUBLE, DOUBLE);
    DOUBLE(*defuzzFcn)(fis_node*, int, double*, int);
    DOUBLE *BigOutMfMatrix; /* used for Mamdani system only */
    DOUBLE *BigWeightMatrix; /* used for Mamdani system only */
    DOUBLE *mfs_of_rule; /* MF values in a rule */

    DOUBLE *bias; /*bias, to be tuned when no rules are fired*/
    int isbias;

    struct fis_node *next;
} FIS;


#endif /* FIS_H */
