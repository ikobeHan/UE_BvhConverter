#include "common.h"
#include <math.h>
#include <algorithm>

//std::vector<int> get_random_vec( int size )
//{
//	std::vector<int> rv;
//	for (int i = 0; i < size; ++i)
//		rv.push_back(i);
//	random_shuffle(rv.begin(), rv.end());
//
//	return rv;
//}

double safe_double(double val)
{
	#ifdef WIN32
	if (_isnan(val) || !_finite(val))
		return 0.0;
	else
		return val;
	#else
	if (isnan(val) || isinf(val))
		return 0.0;
	else
		return val;

	#endif

};	

int safe_int(int val)
{
	#ifdef WIN32
	if (_isnan(val) || !_finite(val))
		return 0.0;
	else
		return val;
	#else
	if (isnan(val) || isinf(val))
		return 0.0;
	else
		return val;

	#endif

};	

