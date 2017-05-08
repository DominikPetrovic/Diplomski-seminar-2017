//funckije za rad s matricama za aglib V 1.0
//Mutka 28.01.2012.

#ifndef __AlglibCotnigus_h
#define __AlglibCotnigus_h



#include "AlglibIncludeAll.h"

using namespace std;
using namespace alglib;

class AlglibCognitusC
{
public:
	double EUCLID3D(double A, double B, double C);
	void IspisiMatricu2D(real_2d_array Matrica,int dim1, int dim2);
	void IspisiMatricu1D(real_1d_array Matrica);
	real_2d_array Multiply(real_2d_array Matrix1,real_2d_array Matrix2);
	double ScalarProduct(real_1d_array vec1, real_1d_array vec2);
	real_2d_array covariance_matrix(real_2d_array data1,real_2d_array data2,int M,int n);
	real_1d_array ProductScalarVector(double value, real_1d_array a);
	real_1d_array CrossProduct(real_1d_array a, real_1d_array b);
	real_1d_array Normalize(real_1d_array a);
	real_1d_array Summ(real_1d_array a, real_1d_array b);
	real_1d_array Diff(real_1d_array a, real_1d_array b);
	real_2d_array Transpose(real_2d_array Matrix);

};


#endif
