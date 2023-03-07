#include "ccml.h"
#include <cmath>
#include <cassert>

using namespace cml;

double cml::pi()
{
	return cml::constants<double>::pi();
}


double cml::deg2rad( double deg )
{
	return deg / 180.0 * pi();
}

double cml::rad2deg( double rad )
{
	return rad / pi() *180.0;
}

cml::vector2 cml::vec2( const vector3& v )
{
	return vector2(v[2],v[0]);
}

cml::vector3 cml::vec3( const vector2& v )
{
	return vector3(v[1], 0.0, v[0]);
}

cml::vector3 cml::vec3( const vector4& v )
{
	return vector3(v[0], v[1], v[2]);
}

cml::vector3 cml::plane_vec3( const vector3& v )
{
	return vector3(v[0], 0, v[2]);
}

double cml::cross_vec2( const vector2& v1, const vector2 &v2 )
{
	return v1[0]*v2[1] - v1[1]*v2[0];
}

cml::transf cml::identity_transf()
{
	return transf(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);
}

cml::transf cml::trans_transf( double x, double y, double z )
{
	return transf(1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1);
}

cml::transf cml::trans_transf( const vector3& v )
{
	return transf(1, 0, 0, v[0],
		0, 1, 0, v[1],
		0, 0, 1, v[2],
		0, 0, 0, 1);
}

cml::transf cml::rotx_transf( double theta )
{
	double c = ::cos(theta);
	double s = ::sin(theta);

	return cml::transf(
		1, 0, 0, 0,
		0, c, -s, 0,
		0, s, c, 0,
		0, 0, 0, 1
		);
}

cml::transf cml::roty_transf( double theta )
{
	double c = ::cos(theta);
	double s = ::sin(theta);

	return cml::transf(
		c, 0, s, 0,
		0, 1, 0, 0,
		-s, 0, c, 0,
		0, 0, 0, 1
		);
}

cml::transf cml::make_transf( const matrix3& mat, const vector3& v )
{
	return cml::transf(
		mat(0,0),	mat(0,1),	mat(0,2),	v[0],
		mat(1,0),	mat(1,1),	mat(1,2),	v[1],
		mat(2,0),	mat(2,1),	mat(2,2),	v[2],
		0		,	0		,	0		,	1
		);
}

cml::vector3 cml::trans( const transf& t )
{
	return cml::vector3(t(0,3), t(1,3), t(2,3));
}

cml::matrix3 cml::mat3( const transf& f )
{
	return cml::matrix3(
		f(0,0), f(0,1), f(0,2),
		f(1,0), f(1,1), f(1,2),
		f(2,0), f(2,1), f(2,2));
}

cml::matrix3 cml::rotx_mat( double theta )
{
	double c = ::cos(theta);
	double s = ::sin(theta);

	return cml::matrix3(
		1, 0, 0,
		0, c, -s,
		0, s, c
		);
}

cml::matrix3 cml::roty_mat( double theta )
{
	double c = ::cos(theta);
	double s = ::sin(theta);

	return cml::matrix3(
		c, 0, s,
		0, 1, 0,
		-s, 0, c
		);
}

cml::matrix3 cml::rotz_mat( double theta )
{
	double c = ::cos(theta);
	double s = ::sin(theta);

	return cml::matrix3(
		c, -s, 0, 
		s, c, 0, 
		0, 0, 1
		);
}

cml::matrix3 cml::identity_mat()
{
	return cml::matrix3(
		1, 0, 0,
		0, 1, 0,
		0, 0, 1);
}

cml::matrix3 cml::interpolate( const matrix3& m1, const matrix3& m2, double ratio )
{
	return m1 * exp_mat3(ratio * log_mat3(transpose(m1) * m2));
}

cml::vector3 cml::log_mat3( const matrix3& mat )
{
	double cosTheta = 0.5 * (mat(0,0) + mat(1,1) + mat(2,2) - 1.0);
	double LIE_EPS = 1e-6;
	double M_PI_SQRT2 = 2.221441469;

	if(cosTheta < LIE_EPS -1.0)
	{
		if(mat(0,0) > 1.0 - LIE_EPS)
			return cml::vector3(cml::pi(), 0, 0);
		else if (mat(1,1) > 1.0 - LIE_EPS)
			return cml::vector3(0, cml::pi(), 0);
		else if (mat(2,2) > 1.0 - LIE_EPS)
			return cml::vector3(0, 0, cml::pi());
		else {
			return cml::vector3(
				M_PI_SQRT2 * ::sqrt((mat(1,0) * mat(1,0) + mat(2,0) * mat(2,0)) / (1.0 - mat(0,0))),
				M_PI_SQRT2 * ::sqrt((mat(0,1) * mat(0,1) + mat(2,1) * mat(2,1)) / (1.0 - mat(1,1))),
				M_PI_SQRT2 * ::sqrt((mat(0,2) * mat(0,2) + mat(1,2) * mat(1,2)) / (1.0 - mat(2,2)))
			);
		}
	}
	else
	{
		if(cosTheta > 1.0) 
			cosTheta = 1.0;
		double theta = ::acos(cosTheta);

		double cof;
		if (theta < LIE_EPS)
			cof = 3.0 / (6.0 - theta * theta);
		else
			cof = theta / (2.0 * ::sin(theta));

		return cml::vector3(
			cof * (mat(2,1) - mat(1,2)),
			cof * (mat(0,2) - mat(2,0)),
			cof * (mat(1,0) - mat(0,1))
			); 
	}
}

//cml::vector3 cml::log_mat3_old( const matrix3& mat )
//{
//	double cosTheta = 0.5 * (mat(0,0) + mat(1,1) + mat(2,2) - 1.0);
//	if(cosTheta > 1.0) cosTheta = 1.0;
//	else if(cosTheta < -1.0) cosTheta = -1.0;
//
//	if (::fabs(cosTheta) > 1.0 - 1e-6)
//		return cml::vector3(0., 0., 0.);
//
//	double theta = ::acos(cosTheta);
//	double cof = theta / (2.0 * ::sin(theta));
//
//	return cml::vector3(
//		cof * (mat(2,1) - mat(1,2)),
//		cof * (mat(0,2) - mat(2,0)),
//		cof * (mat(1,0) - mat(0,1))
//		);
//}

cml::matrix3 cml::exp_mat3( const vector3& v )
{
	double theta = length(v);
	cml::vector3 new_axis = v;

	if (theta != 0.0)
		new_axis /= theta;

	double x = new_axis[0];
	double y = new_axis[1];
	double z = new_axis[2];

	double c = ::cos(theta);
	double s = ::sin(theta);

	return cml::matrix3(
		c + (1.0-c)*x*x,	(1.0-c)*x*y - s*z,	(1-c)*x*z + s*y,
		(1.0-c)*x*y + s*z,	c + (1.0-c)*y*y,    (1.0-c)*y*z - s*x,
		(1.0-c)*z*x - s*y,	(1.0-c)*z*y + s*x,	c + (1.0-c)*z*z
		);
}

cml::matrix3 cml::PlaneProject_mat3( const matrix3& mat )
{
	vector3 rot_vec = log_mat3(mat);
	double theta2 = rot_vec.length() / 2;
	if(std::abs(theta2) < 1e-6) return roty_mat(0.0);

	rot_vec = rot_vec.normalize();
	double theta = pi()-2*std::atan2(std::cos(theta2),std::sin(theta2)*rot_vec[1]);
	
	return roty_mat(theta);
}

cml::transf cml::PlaneProject_transf( const transf& f )
{
	matrix3 m3 = PlaneProject_mat3(mat3(f));
	vector3 v3 = trans(f);
	v3[1] = 0;
	return make_transf(m3,v3);
}

cml::vector3 cml::between_vector( const vector3 &a, const vector3 &b )
{
	cml::vector3 crossV = cml::cross(a,b);
	double len_crossV= cml::length(crossV);
	if (len_crossV != 0.0) {
		return atan2(double(len_crossV), double(cml::dot(a,b))) * crossV / len_crossV;
	}
	else
		return crossV;
}

void cml::ConvexHull( std::vector<vector2>& points )
{
	struct Comp {
		Comp(vector2 &base) {
			m_base = base;
		}				

		bool operator() (const vector2& a, const vector2& b) const {
			vector2 da = a - m_base;
			vector2 db = b - m_base;

			double cosa = da[0] / da.length();
			double cosb = db[0] / db.length();

			//if(da[1] < 1e-15) da[1] = 1e-15;
			//if(db[1] < 1e-15) db[1] = 1e-15;

			return cosa > cosb;
		}
		vector2 m_base;
	};

	std::vector<vector2> vc;
	double min_y = points[0][1];
	int min_i = 0;

	for(size_t i=1; i<points.size(); ++i) {
		if(points[i][1] < min_y) {
			min_y = points[i][1];
			min_i = i;
		}
	}
	std::swap(points[0], points[min_i]);
	std::sort(points.begin()+1, points.end(), Comp(points[0]) );

	vc.push_back(points[0]);
	vc.push_back(points[1]);

	for(size_t i=2; i<points.size(); ++i) {
		while(vc.size() >= 2) {
			vector2 fst = vc[vc.size()-2];
			vector2 snd = vc[vc.size()-1];

			//if(true) {
			if(GetCcw(fst,snd,points[i]) <= 0) {
				vc.pop_back();
			}
			else break;
			/*

			double eval = cross_vec2(points[i]-snd, snd - fst);
			if(eval >= 0) vc.pop_back();
			else break;
			break;
			*/
		}
		vc.push_back(points[i]);
	}

	points = std::move(vc);
}

void cml::ConvexHull2( std::vector<vector2>& points )
{
	struct Comp {
		Comp(vector2 &base) {
			m_base = base;
		}				

		bool operator() (const vector2& a, const vector2& b) const {
			vector2 da = a - m_base;
			vector2 db = b - m_base;

			double cosa = -da[0] / da.length();
			double cosb = -db[0] / db.length();

			//if(da[1] < 1e-15) da[1] = 1e-15;
			//if(db[1] < 1e-15) db[1] = 1e-15;

			return cosa > cosb;
		}
		vector2 m_base;
	};

	std::vector<vector2> vc;
	double max_y = points[0][1];
	int max_i = 0;

	for(size_t i=1; i<points.size(); ++i) {
		if(points[i][1] > max_y) {
			max_y = points[i][1];
			max_i = i;
		}
	}
	std::swap(points[0], points[max_i]);
	std::sort(points.begin()+1, points.end(), Comp(points[0]) );

	vc.push_back(points[0]);
	vc.push_back(points[1]);

	for(size_t i=2; i<points.size(); ++i) {
		while(vc.size() >= 2) {
			vector2 fst = vc[vc.size()-2];
			vector2 snd = vc[vc.size()-1];

			//if(true) {
			if(GetCcw(fst,snd,points[i]) <= 0) {
				vc.pop_back();
			}
			else break;
			/*

			double eval = cross_vec2(points[i]-snd, snd - fst);
			if(eval >= 0) vc.pop_back();
			else break;
			break;
			*/
		}
		vc.push_back(points[i]);
	}

	points = std::move(vc);
}

void cml::ConvexHull3( std::vector<vector2>& points )
{
	struct Comp {
		Comp(vector2 &base) {
			m_base = base;
		}				

		bool operator() (const vector2& a, const vector2& b) const {
			vector2 da = a - m_base;
			vector2 db = b - m_base;

			double cosa = -da[1] / da.length();
			double cosb = -db[1] / db.length();

			//if(da[1] < 1e-15) da[1] = 1e-15;
			//if(db[1] < 1e-15) db[1] = 1e-15;

			return cosa > cosb;
		}
		vector2 m_base;
	};

	std::vector<vector2> vc;
	double min_x = points[0][0];
	int min_i = 0;

	for(size_t i=1; i<points.size(); ++i) {
		if(points[i][0] < min_x) {
			min_x = points[i][0];
			min_i = i;
		}
	}
	std::swap(points[0], points[min_i]);
	std::sort(points.begin()+1, points.end(), Comp(points[0]) );

	vc.push_back(points[0]);
	vc.push_back(points[1]);

	for(size_t i=2; i<points.size(); ++i) {
		while(vc.size() >= 2) {
			vector2 fst = vc[vc.size()-2];
			vector2 snd = vc[vc.size()-1];

			//if(true) {
			if(GetCcw(fst,snd,points[i]) <= 0) {
				vc.pop_back();
			}
			else break;
			/*

			double eval = cross_vec2(points[i]-snd, snd - fst);
			if(eval >= 0) vc.pop_back();
			else break;
			break;
			*/
		}
		vc.push_back(points[i]);
	}

	points = std::move(vc);
}

void cml::ConvexHullwithIndex( const std::vector<vector2>& points, std::vector<int> &index )
{
	std::vector<int> pp;
	for (int i = 0; i < (int)points.size(); ++i) pp.push_back(i);

	struct Comp {
		Comp(const std::vector<vector2>& points_, int i) : points(points_)
		{
			m_base = points_[i];
		}				

		bool operator() (const int& a, const int& b) const {

			vector2 da = points[a] - m_base;
			vector2 db = points[b] - m_base;

			if(da[1] < 1e-15) da[1] = 1e-15;
			if(db[1] < 1e-15) db[1] = 1e-15;

			return da[0]/da[1] > db[0]/db[1];
		}
		vector2 m_base;
		const std::vector<vector2>& points;
	};

	std::vector<int> vc;
	double min_y = points[0][1];
	int min_i = 0;

	for(int i=1; i<points.size(); ++i) {
		if(points[i][1] < min_y) {
			min_y = points[i][1];
			min_i = i;
		}
	}

	std::swap(pp[0], pp[min_i]);



	std::sort(pp.begin()+1, pp.end(), 	Comp(points, pp[0]) );

	vc.push_back(pp[0]);
	vc.push_back(pp[1]);

	for(int i=2; i<pp.size(); ++i) {
		while(vc.size() >= 2) {
			vector2 fst = points[vc[vc.size()-2]];
			vector2 snd = points[vc[vc.size()-1]];

			double eval = cross_vec2(points[pp[i]]-snd, snd - fst);
			//if(eval > 0) vc.pop_back();
			//else 
			break;
		}
		vc.push_back(pp[i]);
	}

	index = vc;
}

bool cml::InterSect( const vector2 &a0, const vector2 &b0, const vector2& a1, const vector2& b1, vector2 *out )
{
	vector2 v0 = b0-a0;
	vector2 v1 = b1-a1;

	matrix22d_c mat;
	mat(0,0) = v0[0];
	mat(0,1) = -v1[0];
	mat(1,0) = v0[1];
	mat(1,1) = -v1[1];

	if(std::abs( determinant(mat) ) < 1e-12) return false;
	mat.inverse();

	vector2 a_(a1[0] - a0[0], a1[1] - a0[1]);

	vector2 t = mat * a_;
	if(t[0] < 0 || t[0] > 1 || t[1] < 0 || t[1] > 1) return false;

	(*out) = a0 + v0 * t[0];
	return true;
}

cml::vector4 cml::pos4( const vector3& v )
{
	return vector4(v[0], v[1], v[2], 1);
}

cml::vector4 cml::vec4( const vector3& v )
{
	return vector4(v[0], v[1], v[2], 0);
}

int cml::GetCcw( const vector2& p1, const vector2& p2, const vector2& p3 )
{
  vector2 d2 = p2 - p1;
  vector2 d3 = p3 - p1;

//  return d2[0] * d3[1] - d2[1] * d3[0];

  if (d2[0]*d3[1] > d2[1]*d3[0]) return 1;
  if (d2[0]*d3[1] < d2[1]*d3[0]) return -1;
  if (d2[0] == 0 && d2[1] == 0) return 0;
  if (( d2[0]*d3[0] < 0) || (d2[1]*d3[1] < 0) ) return -1;
  if (d2.length_squared() < d3.length_squared()) return 1;
  return 0;
}

/*
double cml::ACOS( double x )
{
  if (x > 1.0) return 0;
  else if (x < -1.0) return pi();
  else return std::acos(x);
}
*/

double cml::blend_weight( double x )
{
	return 0.5 * std::cos(pi() * x) + 0.5;
}

double cml::SpeedFunc( double t )
{
	double t2 = t * t;
	double t3 = t2 * t;
	return - 2 * t3 + 3 * t2;
}

double cml::angle_distance( const matrix3 &mat1, const matrix3 &mat2 )
{
  const vector3 ref_vec(1.,0.,0.);
  vector3 mov1 = mat1 * ref_vec;
  vector3 mov2 = mat2 * ref_vec;
  return between_vector(mov1, mov2).length();
}

// return a random integer in the range `[0,' `n)'
int nrand(int n)
{
	rand(); //rand()의 처음 나오는 값이 srand(x)의 x에 비례해서 변하는 문제가 있어서 두번째 값부터 쓰기 위해서 추가.

	if (n <= 0 || n > RAND_MAX) {
		std::cout << "nrand error" << std::endl;
		// throw domain_error("Argument to nrand is out of range");
	}

	const int bucket_size = RAND_MAX / n;
	int r;

	do r = rand() / bucket_size;
	while (r >= n);

	return r;
}

int random(int lowest, int highest)
{
 	int range = (highest-lowest)+1; 
	return lowest + nrand(range);
}

double random_float(double lowest, double hightest, double unit)
{
	int i = 1. / unit;
	return static_cast<double>(random(lowest*i, hightest*i)) / static_cast<double>(i);
}

void get_diff2s(std::vector<cml::vector2d> &diff2s)
{
	diff2s.clear();
	diff2s.push_back(cml::vector2d(0, 0)  ); // 0
	diff2s.push_back(cml::vector2d(0, 1)  );
	diff2s.push_back(cml::vector2d(1, 0)  );
	diff2s.push_back(cml::vector2d(0, -1) );
	diff2s.push_back(cml::vector2d(-1, 0) );	
	diff2s.push_back(cml::vector2d(-1, 1) );	
	diff2s.push_back(cml::vector2d(1, 1)  );		
	diff2s.push_back(cml::vector2d(1, -1) );		
	diff2s.push_back(cml::vector2d(-1, -1));		
	diff2s.push_back(cml::vector2d(-2, 0) );	
	diff2s.push_back(cml::vector2d(2, 0)  );
	diff2s.push_back(cml::vector2d(0, -2) );
	diff2s.push_back(cml::vector2d(0, 2)  );
	diff2s.push_back(cml::vector2d(2, 1)  );
	diff2s.push_back(cml::vector2d(2, -1) );
	diff2s.push_back(cml::vector2d(1, 2)  );
	diff2s.push_back(cml::vector2d(-2, -1));
	diff2s.push_back(cml::vector2d(-2, 1) );
	diff2s.push_back(cml::vector2d(1, -2) );
	diff2s.push_back(cml::vector2d(-1, -2));
	diff2s.push_back(cml::vector2d(-1, 2) );
	diff2s.push_back(cml::vector2d(-2, -2));
	diff2s.push_back(cml::vector2d(-2, 2) );
	diff2s.push_back(cml::vector2d(2, -2) );
	diff2s.push_back(cml::vector2d(2, 2)  );
	diff2s.push_back(cml::vector2d(0, 3));
	diff2s.push_back(cml::vector2d(0, -3));
	diff2s.push_back(cml::vector2d(-3, 0));
	diff2s.push_back(cml::vector2d(3, 0));	
	diff2s.push_back(cml::vector2d(1, -3));	
	diff2s.push_back(cml::vector2d(-1, -3));		
	diff2s.push_back(cml::vector2d(-3, 1));		
	diff2s.push_back(cml::vector2d(1, 3));		
	diff2s.push_back(cml::vector2d(-3, -1));	
	diff2s.push_back(cml::vector2d(-1, 3));
	diff2s.push_back(cml::vector2d(3, -1));
	diff2s.push_back(cml::vector2d(3, 1)); //36
	diff2s.push_back(cml::vector2d(-3, 2)  );
	diff2s.push_back(cml::vector2d(3, 2)   );
	diff2s.push_back(cml::vector2d(3, -2)  );
	diff2s.push_back(cml::vector2d(2, 3)   );
	diff2s.push_back(cml::vector2d(-2, 3)  );	
	diff2s.push_back(cml::vector2d(2, -3)  );	
	diff2s.push_back(cml::vector2d(-3, -2) );
	diff2s.push_back(cml::vector2d(-2, -3) );	
	diff2s.push_back(cml::vector2d(4, 0)   );	
	diff2s.push_back(cml::vector2d(-4, 0)  );	
	diff2s.push_back(cml::vector2d(0, -4)  );
	diff2s.push_back(cml::vector2d(0, 4)   );
	diff2s.push_back(cml::vector2d(4, -1)  );
	diff2s.push_back(cml::vector2d(-4, 1)  );
	diff2s.push_back(cml::vector2d(-1, -4) );
	diff2s.push_back(cml::vector2d(-4, -1) );
	diff2s.push_back(cml::vector2d(4, 1)   );
	diff2s.push_back(cml::vector2d(-1, 4)  );
	diff2s.push_back(cml::vector2d(1, 4)   );
	diff2s.push_back(cml::vector2d(1, -4)  );
	diff2s.push_back(cml::vector2d(3, 3)   );
	diff2s.push_back(cml::vector2d(3, -3)  );
	diff2s.push_back(cml::vector2d(-3, -3) );
	diff2s.push_back(cml::vector2d(-3, 3)  );
	diff2s.push_back(cml::vector2d(-2, -4) );
	diff2s.push_back(cml::vector2d(-4, 2)  ); //49
	diff2s.push_back(cml::vector2d(-4, -2) );
	diff2s.push_back(cml::vector2d(2, 4)   );
	diff2s.push_back(cml::vector2d(2, -4)  );
	diff2s.push_back(cml::vector2d(4, -2)  );	
	diff2s.push_back(cml::vector2d(-2, 4)  );	
	diff2s.push_back(cml::vector2d(4, 2)   );
	diff2s.push_back(cml::vector2d(0, 5)   );	
	diff2s.push_back(cml::vector2d(-3, 4)  );	
	diff2s.push_back(cml::vector2d(0, -5)  );	
	diff2s.push_back(cml::vector2d(-3, -4) );
	diff2s.push_back(cml::vector2d(3, -4)  );
	diff2s.push_back(cml::vector2d(-5, 0)  );
	diff2s.push_back(cml::vector2d(-4, 3)  );
	diff2s.push_back(cml::vector2d(-4, -3) );
	diff2s.push_back(cml::vector2d(3, 4)   );
	diff2s.push_back(cml::vector2d(4, -3)  );
	diff2s.push_back(cml::vector2d(4, 3)   );
	diff2s.push_back(cml::vector2d(5, 0)   );
	diff2s.push_back(cml::vector2d(-1, -5) );
	diff2s.push_back(cml::vector2d(1, -5)  );
	diff2s.push_back(cml::vector2d(1, 5)   );
	diff2s.push_back(cml::vector2d(-5, -1) );
	diff2s.push_back(cml::vector2d(-1, 5)  );
	diff2s.push_back(cml::vector2d(-5, 1)  );
	diff2s.push_back(cml::vector2d(5, -1)  );
	diff2s.push_back(cml::vector2d(5, 1)   );
	diff2s.push_back(cml::vector2d(2, 5)   );
	diff2s.push_back(cml::vector2d(-2, 5)  );
	diff2s.push_back(cml::vector2d(5, -2)  );	
	diff2s.push_back(cml::vector2d(-2, -5) );	
	diff2s.push_back(cml::vector2d(-5, 2)  );
	diff2s.push_back(cml::vector2d(2, -5)  );	
	diff2s.push_back(cml::vector2d(-5, -2) );	
	diff2s.push_back(cml::vector2d(5, 2)   );	
	diff2s.push_back(cml::vector2d(-4, 4)  );
	diff2s.push_back(cml::vector2d(4, -4)  );
	diff2s.push_back(cml::vector2d(-4, -4) );
	diff2s.push_back(cml::vector2d(4, 4)   ); //100
}

double gaussian_function( const double x, const double m, const double sigma, const double amp /*= 1.0*/ )
{
  return amp * std::exp(-1.0 * std::pow((x - m) / sigma, 2.0) / 2.0);
}

