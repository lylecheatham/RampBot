/* Golay.cpp
 * MTE 380 Design Project
 * Original Author(s): Eric Murphy-Zaremba
 * Creation Date: Nov 5 /2018
 *
 * This provides the implementation for the Golay class
 */

#include "Golay.h"
#include <cmath>


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
Golay::Golay(uint32_t sample_freq, uint32_t sample_points = 5, uint32_t order = 2)
{
	float dt = 1.0/sample_freq;

	// Set up t and t'
	t       = new ublas::vector<float>(order + 1);
	t_prime = new ublas::vector<float>(order + 1);

	float curr_t = dt*(order+1);

	for (int32_t i = 0; i < order + 1; i++)
	{
		(*t)(i) = pow(curr_t, order - i);

		(*t_prime)(i) = pow(curr_t, order - i - 1)*(order - i);
	}

	// Set up T
	ublas::matrix<float> T(sample_points, order + 1);
	for (int32_t i = 0; i < T.size1(); i++)
	{
		for (int32_t j = 0; j < T.size2(); j++)
		{
			T(i,j) = pow((i*dt), order-j);
		}
	}

	// Set up T* = (T'T)^(-1) T'
	ublas::matrix<float> T_prime = ublas::trans(T);
	T_star  = new ublas::matrix<float>(order + 1, sample_points); 
	*T_star = ublas::prod( inverse(ublas::prod(T_prime, *T)), T_prime );

	// Initialize c and y
	y = new ublas::vector<float>(sample_points);
	c = new ublas::vector<float>(order + 1);
	for (auto it = y->begin(); it != y->end(); it++)
		*it = 0;
	for (auto it = c->begin(); it != c->end(); it++)
		*it = 0;
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
	delete y;
	delete c;
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
	*c = ublas::prod(*T_star, *y); 

	next_point = inner_prod(*t, *c);

	next_deriv = inner_prod(*t_prime, *c);
}


/* Function: inverse
 *		Finds the inverse of the provided matrix
 * Inputs:
 * 	 A     : The matrix to invert
 *
 * Outputs:
 *	 A_inv : The inverse of the given matrix
 *	 bool  : Whether able to invert or not
 */
bool Golay::inverse(ublas::matrix<float> A, ublas::matrix<float> A_inv)
{
	// partial pivoting (rows only)
	ublas::permutation_matrix<int32_t> p(A.size1());

	// Factorize
	int32_t luf = ublas::lu_factorize(A, p);
	if (luf)
		return 0;

	// Use substitution to get the inverse
	lu_substitute(A, pm, A_inv);

	return 1;
}
