/* Golay.h
 *  Design Project
 * Original Author(s): Eric Murphy-Zaremba
 * Creation Date: Nov 5 /2018
 *
 * This class implements a savitzky-golay filter which acts in the following manner:
 *
 * 		T = matrix of time terms (constant for constant sample rate)
 *
 * 		  = [[t_n^i      t_n^(i-1)     ....  t_n     ],
 * 		     [t_(n-1)^i  t_(n-1)^(i-1) ....  t_(n-1) ],
 *		     .									.
 * 		     .			....				    .
 * 		     .			....					.
 * 		     .			....					.
 *			 .
 * 		     [t_0^i      t_0^(i-1)     ....  t_0     ]]
 *
 *
 * 		y = vector of recorded values over window
 *
 * 		sample_points = window size (number of points to consider for each iteration)
 *
 * 		order = order of the polynomial to fit
 *
 *
 * 		1) First, T* is calculated on construction which is then used to calculate the coefficient
 * 		   matrix for our polynomial in each iteration
 *
 * 		2) Filter is called each time new data is received
 *
 * 		3) Filter takes the recorded values (y) and multiplies them by our T* matrix
 *
 * 		4) Filter then returns the evaluation of our polynomial at the next point (t_(sample_points+1))
 */

#ifndef GOLAY_H
#define GOLAY_H

#include <array>

class Golay {
public:
    Golay();
    ~Golay();

    void filter(float new_point, float* next_point, float* next_deriv = nullptr);

private:
    const static int32_t window = 5;
    const static int32_t order = 2;

    // (T'T)^(-1) T' CHANGES WITH SAMPLE_FREQ
    std::array<std::array<float, window>, order + 1> T_star = {{
        {{
            (float)5714.285714,
            (float)-2857.142857,
            (float)-5714.285714,
            (float)-2857.142857,
            (float)5714.285714,
        }},
        {{
            (float)-74.285714,
            (float)77.142857,
            (float)114.285714,
            (float)37.142857,
            (float)-154.285714,
        }},
        {{
            (float)0.085714,
            (float)-0.142857,
            (float)-0.085714,
            (float)0.257143,
            (float)0.885714,
        }},
    }};
    std::array<float, order + 1> c = {{0}};
    std::array<float, order + 1> t = t = {{
        (float)0.000625,
        (float)0.025000,
        (float)1.000000,
    }};
    ;
    std::array<float, order + 1> t_prime = {{
        (float)0.050000,
        (float)1.000000,
        (float)0.000000,
    }};

    uint32_t y_idx;
    std::array<float, window> y = {{0}};
};


#endif
