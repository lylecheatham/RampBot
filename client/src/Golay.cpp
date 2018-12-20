/* Golay.cpp
 *  Design Project
 * Original Author(s): Eric Murphy-Zaremba
 * Creation Date: Nov 5 /2018
 *
 * This provides the implementation for the Golay class
 */

#include "Golay.h"


/* Function: Golay()
 *		Standard constructor
 * Inputs:
 * 	 None
 * Outputs:
 *	 None
 */
Golay::Golay() : y_idx(0) {}

/* Function: ~Golay
 *		Standard destructor
 * Inputs:
 * 	 None
 * Outputs:
 *	 None
 */
Golay::~Golay() {}

/* Function: filter
 *		Called whenever new data is received, finds the coefficient matrix
 * Inputs:
 * 	 new_point  : the new data point recorded
 *
 * Outputs:
 *	 next_point : the estimated next point after the window (after recorded point)
 *	 next_deriv : the derivative at that point of the fitted polynomial
 */
void Golay::filter(float new_point, float *next_point, float *next_deriv) {
    y[y_idx] = new_point;
    y_idx = (y_idx + 1) % window;

    // ub<float> new_y(window);
    // for (int32_t i = 0; i < window; i++) {
    // new_y(window - 1 - i) = (*y)[(y_idx + i) % window];
    //}


    // Get the new coefficient vector
    for (int32_t i = 0; i < order + 1; i++) {
        c[i] = 0;
        for (int32_t j = 0; j < window; j++) { c[i] += T_star[i][window - j - 1] * y[(y_idx + j) % window]; }
    }

    // Get the next point
    *next_point = 0;
    for (int32_t i = 0; i < order + 1; i++) *next_point += t[i] * c[i];

    // Get the next derivative
    if (next_deriv) {
        *next_deriv = 0;
        for (int32_t i = 0; i < order + 1; i++) *next_deriv += t_prime[i] * c[i];
    }
}
