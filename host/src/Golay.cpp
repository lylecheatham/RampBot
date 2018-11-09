/* Golay.cpp
 * MTE 380 Design Project
 * Original Author(s): Eric Murphy-Zaremba
 * Creation Date: Nov 5 /2018
 *
 * This provides the implementation for the Golay class
 */

#include "Golay.h"
#include <cmath>
#include <iostream>


/* Function: Golay()
 *		Standard constructor
 * Inputs:
 * 	 sample_freq   : the sampling frequency of the system
 * 	 sample_points : the number of points to consider in the window
 * 	 order         : the order of the polynomial to fit
 * 	 
 * Outputs:
 *	 None
 */
Golay::Golay(uint32_t sample_freq, uint32_t sample_points, uint32_t order)
	: window(sample_points)
{
	float dt = 1.0/sample_freq;

	// Set up t and t'
	t       = new ublas::vector<float>(order + 1);
	t_prime = new ublas::vector<float>(order + 1);

	float curr_t = dt*(sample_points);

	for (uint32_t i = 0; i < order + 1; i++)
	{
		(*t)(i) = pow(curr_t, order - i);

		(*t_prime)(i) = i != order ? pow(curr_t, order - i - 1)*(order - i) : 0;
	}

	// Set up T
	ublas::matrix<float> T(sample_points, order + 1);
	for (uint32_t i = 0; i < T.size1(); i++)
	{
		for (uint32_t j = 0; j < T.size2(); j++)
		{
			T(i,j) = pow(((sample_points-1-i)*dt), order-j);
		}
	}

	// Set up T* = (T'T)^(-1) T'
	ublas::matrix<float> T_prime = ublas::trans(T);
	T_star  = new ublas::matrix<float>(order + 1, sample_points); 
	*T_star = ublas::prod(T_prime, T);
	inverse(*T_star, *T_star);
	*T_star = ublas::prod( *T_star, T_prime );

	// Initialize c 
	c = new ublas::vector<float>(order + 1);
	for (auto it = c->begin(); it != c->end(); it++)
		*it = 0;

	// Setup y
	y     = new std::vector<float>(sample_points, 0);
	y_idx = 0;
}

/* Function: ~Golay
 *		Standard destructor
 * Inputs:
 * 	 None
 * Outputs:
 *	 None
 */
Golay::~Golay()
{
	delete T_star;
	delete c;
	delete t;
	delete t_prime;
	delete y;
}

/* Function: filter
 *		Called whenever new data is received, finds the coefficient matrix
 * Inputs:
 * 	 new_point  : the new data point recorded
 *
 * Outputs:
 *	 next_point : the estimated next point after the window (after recorded point)
 *	 next_deriv : the derivative at that point of the fitted polynomial
 */
void Golay::filter(float new_point, float &next_point, float &next_deriv)
{
	(*y)[y_idx] = new_point;
	y_idx = (y_idx + 1) % window;

	ublas::vector<float> new_y(window);
	for (int32_t i = 0; i < window; i++)
	{
		new_y(window-1-i) = (*y)[(y_idx + i) % window];
	}

	*c = ublas::prod(*T_star, new_y); 

#ifdef DEBUG
	print("c: ", c);
	print("y: ", &new_y);
#endif

	next_point = ublas::inner_prod(*t, *c);

	next_deriv = ublas::inner_prod(*t_prime, *c);
}


/* Function: inverse
 *		Finds the inverse of the provided matrix
 * Inputs:
 * 	 A     : The matrix to invert
 *
 * Outputs:
 *	 A_inv : The inverse of the given matrix
 *	 bool  : Whether able to invert or not
 * 
 * Note: https://gist.github.com/lilac/2464434
 */
bool Golay::inverse(ublas::matrix<float> A, ublas::matrix<float> &A_inv)
{
	// partial pivoting (rows only)
	ublas::permutation_matrix<int32_t> pm(A.size1());

	// Factorize
	int32_t luf = ublas::lu_factorize(A, pm);
	if (luf)
		return 0;

	A_inv.assign(ublas::identity_matrix<float>(A.size1()));

	// Use substitution to get the inverse
	ublas::lu_substitute(A, pm, A_inv);

	return 1;
}

#ifdef DEBUG
void Golay::print(std::string title, ublas::vector<float> *vec)
{
	std::cout << title;
	for (auto it = vec->begin(); it != vec->end(); it++)
	{
		std::cout << *it << ", ";
	}
	std::cout << std::endl;
}

void Golay::print(std::string title, ublas::matrix<float> *mat)
{
	std::cout << title;
	for (uint32_t i = 0; i < mat->size1(); i++)
	{
		for (uint32_t j = 0; j < mat->size2(); j++)
		{
			std::cout << (*mat)(i, j) << ", ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

#endif