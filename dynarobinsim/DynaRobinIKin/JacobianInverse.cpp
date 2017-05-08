
#include "stdafx.h"
#include "ALGLIB/AlglibIncludeAll.h"
#include "ALGLIB/AlglibCognitus.h"
#include "include/JacobianInverse.h"
#include <QDebug>

bool JakobijanInver(vector<float>* FL,vector<float>* FR, vector<float>* BL, vector<float>* BR, vector<float> ksi)
{
	///jako bitno, parametri robota
    double l1=0.14;
    double l2=0.05;
    double l3=0.07;
	double l4=0;
    double l5=0.1;
    double l6=0.12;

	//definiranje zglobova
	double qFR1=-FR->at(0)*3.14/180;
    double qFR2=FR->at(1)*3.14/180;
	double qFR3=FR->at(2)*3.14/180;

	double qFL1=FL->at(0)*3.14/180;
	double qFL2=-FL->at(1)*3.14/180;
	double qFL3=-FL->at(2)*3.14/180;

	double qBL1=BL->at(0)*3.14/180;
	double qBL2=-BL->at(1)*3.14/180;
	double qBL3=-BL->at(2)*3.14/180;

	double qBR1=-BR->at(0)*3.14/180;
	double qBR2=BR->at(1)*3.14/180;
	double qBR3=BR->at(2)*3.14/180;
	
	real_2d_array P;P.setlength(12,6);

	P(0,0) = sin(qFR2 + qFR3);
	P(0,1) = cos(qFR2 + qFR3)* sin(qFR1);
	P(0,2) = -cos(qFR1)* cos(qFR2 + qFR3);
	P(0,3) = (l3 + l2* cos(qFR1)) * cos(qFR2 + qFR3);
	P(0,4) = cos(qFR1)* (l1* cos(qFR2 + qFR3) - l5* sin(qFR3)) - (l4 *cos(qFR1) +  l3* sin(qFR1)) *sin(qFR2 + qFR3);
	P(0,5)= cos(qFR2 + qFR3) * sin(qFR1)* (l1 + l5 *sin(qFR2)) + (l2 + l3* cos(qFR1) - (l4 + l5 *cos(qFR2)) *sin(qFR1)) *sin(qFR2 + qFR3);

	P(1,0) = cos(qFR2 + qFR3);
	P(1,1) = -sin(qFR1) *sin(qFR2 + qFR3);
	P(1,2) = cos(qFR1)* sin(qFR2 + qFR3);
	P(1,3) = -(l3 + l2 *cos(qFR1)) *sin(qFR2 + qFR3);
	P(1,4) = -l3 *cos(qFR2 + qFR3)* sin(qFR1) - cos(qFR1) *(l6 + l5* cos(qFR3) + l4* cos(qFR2 + qFR3) + l1 *sin(qFR2 + qFR3));
	P(1,5) = cos(qFR2 + qFR3)* (l2 + l3 *cos(qFR1) - l4 *sin(qFR1)) - sin(qFR1)* (l6 + l5 *cos(qFR3) + l1* sin(qFR2 + qFR3));
	
	P(2,0)=0;
	P(2,1)=-cos(qFR1);
	P(2,2)=-sin(qFR1);
	P(2,3) = -l4 - l5 *cos(qFR2) - l6 *cos(qFR2 + qFR3) + l2 *sin(qFR1);
	P(2,4) = sin(qFR1)* (l1 + l5 *sin(qFR2) + l6* sin(qFR2 + qFR3));
	P(2,5) = -cos(qFR1) *(l1 + l5* sin(qFR2) + l6 * sin(qFR2 + qFR3));

	P(3,0)=-sin(qFL2 + qFL3);
	P(3,1)=cos(qFL2 + qFL3)* sin(qFL1);
	P(3,2)=-cos(qFL1)* cos(qFL2 + qFL3);
	P(3,3)=-(l3 + l2 *cos(qFL1))* cos(qFL2 + qFL3);
	P(3,4)=-l3 *sin(qFL1)* sin(qFL2 + qFL3) +  cos(qFL1)* (l1* cos(qFL2 + qFL3) + l5 *sin(qFL3) + l4* sin(qFL2 + qFL3));
	P(3,5)=l1* cos(qFL2 + qFL3) *sin(qFL1) + l5* sin(qFL1)* sin(qFL3) + (l2 + l3* cos(qFL1) + l4 *sin(qFL1))* sin(qFL2 + qFL3);

	P(4,0) = -cos(qFL2 + qFL3);
	P(4,1) =-sin(qFL1)* sin(qFL2 + qFL3);
	P(4,2) =cos(qFL1)* sin(qFL2 + qFL3);
	P(4,3) =(l3 + l2 *cos(qFL1)) *sin(qFL2 + qFL3);
	P(4,4) =-l3 *cos(qFL2 + qFL3) *sin(qFL1) +  cos(qFL1)* (l6 + l5 *cos(qFL3) + l4 *cos(qFL2 + qFL3) - l1* sin(qFL2 + qFL3));
	P(4,5) = cos(qFL2 + qFL3)* (l2 + l3 *cos(qFL1) + l4 *sin(qFL1)) +  sin(qFL1) *(l6 + l5 *cos(qFL3) - l1 *sin(qFL2 + qFL3));

	P(5,0) = 0;
	P(5,1) =cos(qFL1);
	P(5,2) =sin(qFL1);
	P(5,3) =l4 + l5 *cos(qFL2) + l6* cos(qFL2 + qFL3) + l2 *sin(qFL1);
	P(5,4) =sin(qFL1) *(-l1 + l5 *sin(qFL2) + l6 *sin(qFL2 + qFL3));
	P(5,5) =cos(qFL1)* (l1 - l5 *sin(qFL2) - l6 *sin(qFL2 + qFL3));

	P(6,0)=-sin(qBL2 + qBL3);
	P(6,1) = cos(qBL2 + qBL3)* sin(qBL1);
	P(6,2) =-cos(qBL1)* cos(qBL2 + qBL3);
	P(6,3) =-(l3 + l2* cos(qBL1))* cos(qBL2 + qBL3);
	P(6,4) =-l3 *sin(qBL1) *sin(qBL2 + qBL3) +  cos(qBL1)* (-l1* cos(qBL2 + qBL3) + l5 *sin(qBL3) + l4 *sin(qBL2 + qBL3));
	P(6,5) =-l1 *cos(qBL2 + qBL3)* sin(qBL1) +  l5* sin(qBL1)* sin(qBL3) + (l2 + l3 *cos(qBL1) + l4 *sin(qBL1)) * sin( qBL2 + qBL3);


	P(7,0)=-cos(qBL2 + qBL3);
	P(7,1)=-sin(qBL1) *sin(qBL2 + qBL3);
	P(7,2)=cos(qBL1) *sin(qBL2 + qBL3);
	P(7,3)=(l3 + l2 *cos(qBL1))* sin(qBL2 + qBL3);
	P(7,4)=-l3 *cos(qBL2 + qBL3) *sin(qBL1) +  cos(qBL1)* (l6 + l5 *cos(qBL3) + l4 *cos(qBL2 + qBL3) +     l1* sin(qBL2 + qBL3));
	P(7,5)=cos(qBL2 + qBL3) *(l2 + l3* cos(qBL1) + l4* sin(qBL1)) +  sin(qBL1) *(l6 + l5 *cos(qBL3) + l1 *sin(qBL2 + qBL3));


	P(8,0)=0;
	P(8,1)=cos(qBL1);
	P(8,2)=sin(qBL1);
	P(8,3)=l4 + l5* cos(qBL2) + l6 *cos(qBL2 + qBL3) + l2* sin(qBL1);
	P(8,4)=sin(qBL1) *(l1 + l5 *sin(qBL2) + l6 *sin(qBL2 + qBL3));
	P(8,5)=-cos(qBL1) *(l1 + l5 *sin(qBL2) + l6 *sin(qBL2 + qBL3));


	P(9,0)=sin(qBR2 + qBR3);
	P(9,1)=cos(qBR2 + qBR3)* sin(qBR1);
	P(9,2)=-cos(qBR1) *cos(qBR2 + qBR3);
	P(9,3)=(l3 + l2 *cos(qBR1))* cos(qBR2 + qBR3);
	P(9,4)=-l3* sin(qBR1) *sin(qBR2 + qBR3) -  cos(qBR1)* (l1 *cos(qBR2 + qBR3) + l5 *sin(qBR3) + l4 *sin(qBR2 + qBR3));
	P(9,5)=-sin(qBR1)* (l1 *cos(qBR2 + qBR3) + l5* sin(qBR3)) + (l2 + l3* cos(qBR1) -l4 *sin(qBR1))* sin(qBR2 + qBR3);

	P(10,0)= cos(qBR2 + qBR3);
	P(10,1)=-sin(qBR1)* sin(qBR2 + qBR3);
	P(10,2)=cos(qBR1)* sin(qBR2 + qBR3);
	P(10,3)=-(l3 + l2 *cos(qBR1))* sin(qBR2 + qBR3);
	P(10,4)=-l3* cos(qBR2 + qBR3)* sin(qBR1) -  cos(qBR1)* (l6 + l5 *cos(qBR3) + l4 *cos(qBR2 + qBR3) - l1* sin(qBR2 + qBR3));
	P(10,5)=cos(qBR2 + qBR3) *(l2 + l3* cos(qBR1) - l4 *sin(qBR1)) -  sin(qBR1) *(l6 + l5* cos(qBR3) - l1* sin(qBR2 + qBR3));
	
	P(11,0)=0;
	P(11,1)=-cos(qBR1);
	P(11,2)=-sin(qBR1);
	P(11,3)=-l4 - l5 *cos(qBR2) - l6 *cos(qBR2 + qBR3) + l2 * sin(qBR1);
	P(11,4)=sin(qBR1)* (-l1 + l5 *sin(qBR2) + l6 *sin(qBR2 + qBR3));
	P(11,5)=cos(qBR1)* (l1 - l5* sin(qBR2) - l6* sin(qBR2 + qBR3));


	real_2d_array Q;Q.setlength(12,12);

    Q(0,0) = l3 * cos(qFR2 + qFR3);
         Q(0,1) = l5 * sin(qFR3);
         Q(0,2)=0;
         Q(0,3)=0;
         Q(0,4)=0;
         Q(0,5)=0;
         Q(0,6)=0;
         Q(0,7)=0;
         Q(0,8)=0;
         Q(0,9)=0;
         Q(0,10)=0;
         Q(0,11)=0;

         Q(1,0) = -l3 *sin(qFR2 + qFR3);
         Q(1,1) = l6 + l5* cos(qFR3);
         Q(1,2)=l6;
         Q(1,3)=0;
         Q(1,4)=0;
         Q(1,5)=0;
         Q(1,6)=0;
         Q(1,7)=0;
         Q(1,8)=0;
         Q(1,9)=0;
         Q(1,10)=0;
         Q(1,11)=0;

         Q(2,0) = -l4 - l5 *cos(qFR2) - l6* cos(qFR2 + qFR3);
         Q(2,1) = 0;
         Q(2,2)=0;
         Q(2,3)=0;
         Q(2,4)=0;
         Q(2,5)=0;
         Q(2,6)=0;
         Q(2,7)=0;
         Q(2,8)=0;
         Q(2,9)=0;
         Q(2,10)=0;
         Q(2,11)=0;

         Q(3,0) = 0;
         Q(3,1) = 0;
         Q(3,2) = 0;
         Q(3,3) = -l3 * cos(qFL2 + qFL3);
         Q(3,4)=l5 * sin(qFL3);
         Q(3,5)=0;
         Q(3,6)=0;
         Q(3,7)=0;
         Q(3,8)=0;
         Q(3,9)=0;
         Q(3,10)=0;
         Q(3,11)=0;

         Q(4,0) = 0;
         Q(4,1) = 0;
         Q(4,2) = 0;
         Q(4,3) = l3* sin(qFL2 + qFL3);
         Q(4,4)= l6 + l5 *cos(qFL3);
         Q(4,5)= l6;
         Q(4,6)=0;
         Q(4,7)=0;
         Q(4,8)=0;
         Q(4,9)=0;
         Q(4,10)=0;
         Q(4,11)=0;

         Q(5,0) = 0;
         Q(5,1) = 0;
         Q(5,2) = 0;
         Q(5,3) = l4 + l5* cos(qFL2) + l6 * cos(qFL2 + qFL3);
         Q(5,4)= 0;
         Q(5,5)= 0;
         Q(5,6)=0;
         Q(5,7)=0;
         Q(5,8)=0;
         Q(5,9)=0;
         Q(5,10)=0;
         Q(5,11)=0;

         Q(6,0) = 0;
         Q(6,1) = 0;
         Q(6,2) = 0;
         Q(6,3) = 0;
         Q(6,4)= 0;
         Q(6,5)= 0;
         Q(6,6)=-l3* cos(qBL2 + qBL3);
         Q(6,7)= l5 * sin(qBL3);
         Q(6,8)=0;
         Q(6,9)=0;
         Q(6,10)=0;
         Q(6,11)=0;

         Q(7,0) = 0;
         Q(7,1) = 0;
         Q(7,2) = 0;
         Q(7,3) = 0;
         Q(7,4)= 0;
         Q(7,5)= 0;
         Q(7,6)=l3 *sin(qBL2 + qBL3);
         Q(7,7)= l6 + l5 *cos(qBL3);
         Q(7,8)=l6;
         Q(7,9)=0;
         Q(7,10)=0;
         Q(7,11)=0;

         Q(8,0) = 0;
         Q(8,1) = 0;
         Q(8,2) = 0;
         Q(8,3) = 0;
         Q(8,4)= 0;
         Q(8,5)= 0;
         Q(8,6)=l4 + l5* cos(qBL2) + l6* cos(qBL2 + qBL3);
         Q(8,7)= 0;
         Q(8,8)=0;
         Q(8,9)=0;
         Q(8,10)=0;
         Q(8,11)=0;

         Q(9,0) = 0;
         Q(9,1) = 0;
         Q(9,2) = 0;
         Q(9,3) = 0;
         Q(9,4)= 0;
         Q(9,5)= 0;
         Q(9,6)=0;
         Q(9,7)= 0;
         Q(9,8)=0;
         Q(9,9)=l3 *cos(qBR2 + qBR3);
         Q(9,10)=l5* sin(qBR3);
         Q(9,11)=0;

         Q(10,0) = 0;
         Q(10,1) = 0;
         Q(10,2) = 0;
         Q(10,3) = 0;
         Q(10,4)= 0;
         Q(10,5)= 0;
         Q(10,6)=0;
         Q(10,7)= 0;
         Q(10,8)=0;
         Q(10,9)=-l3 *sin(qBR2 + qBR3);
         Q(10,10)=l6 + l5* cos(qBR3);
         Q(10,11)=l6;

         Q(11,0) = 0;
         Q(11,1) = 0;
         Q(11,2) = 0;
         Q(11,3) = 0;
         Q(11,4)= 0;
         Q(11,5)= 0;
         Q(11,6)=0;
         Q(11,7)= 0;
         Q(11,8)=0;
         Q(11,9)=-l4 - l5 * cos(qBR2) - l6* cos(qBR2 + qBR3);
         Q(11,10)=0;
         Q(11,11)=0;

    for(int i=0;i<=11;i++)
    {
        for(int j=0;j<=11;j++)
        {
            Q(i,j)*=-1;
        }
    }

    //agc->IspisiMatricu2D(Q,12,12);



    AlglibCognitusC * agc = new AlglibCognitusC();
    //stvaranje jakobijana
    ae_int_t info;
    matinvreport rep;
    rmatrixinverse(Q,info,rep);


    //real_2d_array pseudoQ = agc->Transpose(Q);
    real_2d_array FINAL = agc->Multiply(Q,P);


	//popunjavanje brzinske matrice
	real_2d_array ksiVec;ksiVec.setlength(6,1);
    ksiVec(0,0) = ksi.at(0);
    ksiVec(1,0) = ksi.at(1);
    ksiVec(2,0) = ksi.at(2);
    ksiVec(3,0) = ksi.at(3);
    ksiVec(4,0) = ksi.at(4);
    ksiVec(5,0) = ksi.at(5);

	real_2d_array deltaQ = agc->Multiply(FINAL,ksiVec);
	
	///vracanje nazad
	FR->at(0) += -deltaQ(0,0)*180/3.14;
    FR->at(1) += deltaQ(1,0)*180/3.14;
	FR->at(2) += deltaQ(2,0)*180/3.14;

	FL->at(0) += deltaQ(3,0)*180/3.14;
	FL->at(1) += -deltaQ(4,0)*180/3.14;
	FL->at(2) += -deltaQ(5,0)*180/3.14;

	BL->at(0) += deltaQ(6,0)*180/3.14;
	BL->at(1) += -deltaQ(7,0)*180/3.14;
	BL->at(2) += -deltaQ(8,0)*180/3.14;

	BR->at(0)+= -deltaQ(9,0)*180/3.14;
	BR->at(1)+= deltaQ(10,0)*180/3.14;
	BR->at(2)+= deltaQ(11,0)*180/3.14;
	
	agc->IspisiMatricu2D(deltaQ,12,1);

	return true;

}

/*
int _tmain(int argc, _TCHAR* argv[])
{

	vector<float> FL;for(int i=0;i<4;i++) FL.push_back(0);
	vector<float> FR;for(int i=0;i<4;i++) FR.push_back(0);
	vector<float> BL;for(int i=0;i<4;i++) BL.push_back(0);
	vector<float> BR;for(int i=0;i<4;i++) BR.push_back(0);

	///ovo je vektor delti, dX,dY,dZ,dPhi,dTheta,dPsi
	vector<float> ksi;
	ksi.push_back(0);
	ksi.push_back(0);
	ksi.push_back(-1);
	ksi.push_back(0);
	ksi.push_back(0);
	ksi.push_back(0);

	JakobijanInver(&FL,&FR,&BL,&BR,ksi);
	getchar();

	



	return 0;
}
*/
