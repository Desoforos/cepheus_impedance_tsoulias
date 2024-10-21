#ifndef ROBOT_FUNCTIONS_H
#define ROBOT_FUNCTIONS_H
#include "includes.h"
#include "robot_variables.h"




void initialiseParameters();

void calculateTrajecotryPolynomials(double tf);

void finalTrajectories(double t,double tf);

void controller(int count, double tf, double t);






#endif