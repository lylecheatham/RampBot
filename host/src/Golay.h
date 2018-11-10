/* Golay.h
 * MTE 380 Design Project
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

#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <vector>

#ifdef DEBUG
#include <string>
#endif

namespace ublas = boost::numeric::ublas;

class Golay {
public:
    Golay(uint32_t sample_freq, uint32_t sample_points = 5, uint32_t order = 2);
    ~Golay();

    void filter(float new_point, float& next_point, float& next_deriv);

private:
    bool inverse(ublas::matrix<float> A, ublas::matrix<float>& A_inv);

#ifdef DEBUG
    void print(std::string title, ublas::vector<float>* vec);
    void print(std::string title, ublas::matrix<float>* mat);
#endif

private:
    ublas::matrix<float>* T_star;  // (T'T)^(-1) T'
    ublas::vector<float>* c;
    ublas::vector<float>* t;
    ublas::vector<float>* t_prime;

    std::vector<float>* y;
    uint32_t y_idx;
    uint32_t window;
};


#endif
