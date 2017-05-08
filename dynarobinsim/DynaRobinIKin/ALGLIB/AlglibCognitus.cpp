#include "AlglibCognitus.h"

void AlglibCognitusC::IspisiMatricu2D(real_2d_array Matrica,int dim1, int dim2)
{
	for(int i=0;i<dim1;i++)//0 do n-1
	{
		for(int j=0;j<dim2;j++)//0 1 2
		{
			printf("%lf ",Matrica(i,j));
		}
		printf("\r\n"); 
	}
}

void AlglibCognitusC::IspisiMatricu1D(real_1d_array Matrica)
{
	
	for(int i=0;i<=2;i++)//0 do n-1
	{
		printf("%lf ",Matrica(i));
		
	}
}

double AlglibCognitusC::EUCLID3D(double A, double B, double C)
{
	return sqrt(A*A+B*B+C*C);
}

//kovarijance matrix, alglib

real_2d_array AlglibCognitusC::covariance_matrix(real_2d_array data1,real_2d_array data2,int M,int n)
{
	real_2d_array covar_matr;covar_matr.setlength(M,M);
	for(int j1=0;j1<M;j1++)
	{
		for(int j2=0;j2<M;j2++)
		{
			covar_matr(j1,j2)=0;
		}
	}


	for(int N1=0;N1<n;N1++)
	{
		for(int j1=0;j1<M;j1++)
		{
			for(int j2=0;j2<M;j2++)
			{
				covar_matr(j1,j2)=covar_matr(j1,j2)+data1(N1,j1)*data2(N1,j2);

			}

		}
	}
	
	return covar_matr;
}
real_2d_array AlglibCognitusC::Transpose(real_2d_array Matrix)
{
	double ROW=Matrix.rows();
	double COL=Matrix.cols();

	real_2d_array newMatrix;
	newMatrix.setlength(COL,ROW);

	for(int i=0;i<ROW;i++)
	{
		for(int j=0;j<COL;j++)
		{
			newMatrix[j][i]=Matrix[i][j];
		}
	}
	return newMatrix;
}

//3d vektori
double AlglibCognitusC::ScalarProduct(real_1d_array vec1, real_1d_array vec2)
{
		return vec1[0]*vec2[0] +vec1[1]*vec2[1] +vec1[2]*vec2[2] ;
		
}
real_1d_array AlglibCognitusC::ProductScalarVector(double value, real_1d_array a)
{
		real_1d_array rez;rez.setlength(3);
		rez[0] = a[0]*value;
		rez[1] = a[1]*value;
		rez[2] = a[2]*value;
		return rez;
	
}
real_1d_array AlglibCognitusC::CrossProduct(real_1d_array a, real_1d_array b)
{
		real_1d_array cross;cross.setlength(3);
		cross[0] = a[1]*b[2]-a[2]*b[1];
		cross[1] = a[2]*b[0]-a[0]*b[2];
		cross[2] = a[0]*b[1]-a[1]*b[0];
		return cross;
	
}
real_1d_array AlglibCognitusC::Normalize(real_1d_array a)
{
		real_1d_array rez;rez.setlength(3);
		double norm = EUCLID3D(a[0],a[1],a[2]);

		rez[0] =a[0]/norm;
		rez[1] =a[1]/norm;
		rez[2] =a[2]/norm;
		return rez;
	
}

real_1d_array AlglibCognitusC::Summ(real_1d_array a, real_1d_array b)
{
		real_1d_array sum;sum.setlength(3);
		sum[0] =a[0]+b[0];
		sum[1] =a[1]+b[1];
		sum[2] =a[2]+b[2];
		return sum;
	
}
real_1d_array AlglibCognitusC::Diff(real_1d_array a, real_1d_array b)
{
		real_1d_array diff;diff.setlength(3);
		diff[0] =a[0]-b[0];
		diff[1] =a[1]-b[1];
		diff[2] =a[2]-b[2];
		return diff;
	
}




real_2d_array AlglibCognitusC::Multiply(real_2d_array Matrix1,real_2d_array Matrix2)
{
	long row;
	long column;
	long column2;
	

	double ROWMAX=Matrix1.rows();
	double COLMAX=Matrix1.cols();
	double COLMAX2=Matrix2.cols();

	real_2d_array newMatrix;
	newMatrix.setlength(ROWMAX,COLMAX2);

	for(int i=0;i<ROWMAX;i++)
	{
		for(int j=0;j<COLMAX2;j++)
		{
			newMatrix[i][j]=0;
		}
	}

	if( Matrix1.cols()!=Matrix2.rows())//ERROR
	{
		return newMatrix;
	}
	else
	{

		for(int row=0;row<ROWMAX;row++)
		{
			for(int column=0;column<COLMAX2;column++)
			{
				for(int column2=0;column2<COLMAX;column2++)
				{
					newMatrix[row][column] = newMatrix[row][column]+Matrix1[row][column2]*Matrix2[column2][column];
				}
			}
		}

	}

	return newMatrix;

}