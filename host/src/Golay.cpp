/* Golay.cpp
 * MTE 380 Design Project
 * Original Author(s): Eric Murphy-Zaremba
 * Creation Date: Nov 5 /2018
 *
 * This provides the implementation for the Golay class
 */

#include "Golay.h"
#include <cmath>

Golay::Golay(uint32_t sample_freq, uint32_t sample_points = 5, uint32_t order = 2)
{
	float dt = 1.0/sample_freq;

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
	ublas::matrix<float> T_prime = trans(T);
	T_star = new ublas::matrix<float>(order + 1, sample_points); 
	*T_star = prod( inverse(prod(T_prime, T)), T_prime );

	// Initialize c and y
	y = new ublas::vector<float>(sample_points);
	c = new ublas::vector<float>(order + 1);
	for (auto it = y->begin(); it != y->end(); it++)
		*it = 0;
	for (auto it = c->begin(); it != c->end(); it++)
		*it = 0;
}

Golay::~Golay()
{
	delete T_star;
	delete y;
	delete c;
}

void Golay::inverse(ublas::matrix<float> A, ublas::matrix<float> A_inv)
{
	// partial pivoting (rows only)
	ublas::permutation_matrix<int32_t> p(A.size1());

	// Factorize
	int32_t luf = ublas::lu_factorize(A, p);
	if (luf)
		return;

	// Use substitution to get the inverse
	lu_substitute(A, pm, A_inv);
}
