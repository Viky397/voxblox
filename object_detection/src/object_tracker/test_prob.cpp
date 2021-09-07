#include "iostream"
#include <math.h>
#include <boost/bind.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/uniform.hpp>

using namespace std;

class Object1 {
 public:
    ~Object1() {}

    double a = 2.0;
    double b = 2.0;
    double mu = 0.01;
    double sig = 0.1;
};

void object_update_probability(
	    Object1& obj,
		double tau,
		double x
		){
	double s_sq = 1.0 / (1.0 / (pow(obj.sig, 2)) + 1.0 / (pow(tau, 2)));
	double m = s_sq * (obj.mu / (pow(obj.sig, 2)) + x / (pow(tau, 2)));
	// cout << "ssq: " << s_sq << '\n' << "m: " << m << '\n' << endl;

	boost::math::normal_distribution<double> norm_dist(obj.mu, (obj.sig));
	boost::math::uniform_distribution<double> uniform_dist(0.0, 1.0);

	double C1 = (obj.a / (obj.a + obj.b)) * boost::math::pdf(norm_dist, x);
	double C2 = (obj.b / (obj.a + obj.b)) * boost::math::pdf(uniform_dist, x);

	double C_norm = C1 + C2;
	C1 /= C_norm;
	C2 /= C_norm;

	// cout << "C1: " << C1 << '\n' << "C2: " << C2 << '\n' << endl;

	double mu_prime = C1 * m + C2 * obj.mu;
	obj.sig = sqrt(C1 * (s_sq + pow(m, 2)) + C2 * (pow(obj.sig, 2) + pow(obj.mu, 2)) - pow(mu_prime, 2));

	double f = C1 * (obj.a + 1.0) / (obj.a + obj.b + 1.0) + C2 * obj.a / (obj.a + obj.b + 1.0);
	double e = C1 * (obj.a + 1.0) * (obj.a + 2.0) / ((obj.a + obj.b + 1.0) * (obj.a + obj.b + 2.0))
			  + C2 * obj.a * (obj.a + 1.0) / ((obj.a + obj.b + 1.0) * (obj.a + obj.b + 2.0));

	obj.mu = mu_prime;

	obj.a = (e - f) / (f - e / f);
	obj.b = obj.a * (1.0 - f) / f ;

	cout << "mu: " << obj.mu << '\n' << "sig: " << obj.sig << '\n' << endl;

	cout << "a: " << obj.a << '\n' << "b: " << obj.b << '\n' << endl;

    cout << "Static Probability: " << obj.a / (obj.a + obj.b) << "\n" << endl;
}

int main(int argc, char *argv[])

{   Object1 obj;

    double tau  = 0.05;
    double x = 0.02;
    int N = 10;

    while(N--){

    	object_update_probability(obj, tau, x);

    }

}
