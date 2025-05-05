#include <cmath>
#include <vector>
#include "IIRButterworthFilter.h"

class IIRButterworthFilter {
private:
    double a1, a2, b0, b1, b2;
    double x_prev1, x_prev2; // Previous input values (raw ydot)
    double y_prev1, y_prev2; // Previous output values (filtered ydot)

public:
    IIRButterworthFilter(double cutoff_freq, double sampling_freq) {
        double omega = 2.0 * M_PI * cutoff_freq / sampling_freq;
        double sin_omega = sin(omega);
        double cos_omega = cos(omega);
        double alpha = sin_omega / (2.0 * sqrt(2.0)); // Butterworth Q-factor

        // Butterworth filter coefficients (2nd-order)
        b0 = (1.0 - cos_omega) / 2.0;
        b1 = 1.0 - cos_omega;
        b2 = (1.0 - cos_omega) / 2.0;
        a1 = -2.0 * cos_omega;
        a2 = 1.0 - alpha;

        // Normalize coefficients
        double a0 = 1.0 + alpha;
        b0 /= a0;
        b1 /= a0;
        b2 /= a0;
        a1 /= a0;
        a2 /= a0;

        // Initialize previous values to zero
        x_prev1 = x_prev2 = 0.0;
        y_prev1 = y_prev2 = 0.0;
    }

    double filter(double x) {
        // Apply the IIR difference equation
        double y = b0 * x + b1 * x_prev1 + b2 * x_prev2 - a1 * y_prev1 - a2 * y_prev2;

        // Update history for next iteration
        x_prev2 = x_prev1;
        x_prev1 = x;
        y_prev2 = y_prev1;
        y_prev1 = y;

        return y;
    }
};
