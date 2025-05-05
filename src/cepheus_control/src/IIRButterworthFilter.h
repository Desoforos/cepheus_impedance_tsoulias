#ifndef BUTTERWORTH_FILTER_H
#define BUTTERWORTH_FILTER_H

class IIRButterworthFilter {
private:
    double a1, a2, b0, b1, b2;
    double x_prev1, x_prev2; // Previous input values
    double y_prev1, y_prev2; // Previous output values

public:
    IIRButterworthFilter(double cutoff_freq, double sampling_freq);
    double filter(double x);
};

#endif // BUTTERWORTH_FILTER_H
