#ifndef ROBOT_FUNCTIONS_H
#define ROBOT_FUNCTIONS_H
#include "robot_variables.h"


void initialiseParametersNew();

void calculateTrajecotryPolynomials(double tf);

void finaltrajectories(double t,double tf);

void controller(int count, double tf, double t);






#endif