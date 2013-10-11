#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include "nrutil.h"


#define NR_END 1
#define FREE_ARG char*


//----------------------- for my_qp
double **_nrTEMP1_34x34, **_nrTEMP2_34x34, **_nrTEMP3_34x34;
double **_A_56x56, **_Aa_27x27, **_A0_54x27, **_N_27x27, **_EYE_34;
int _izrov_54x1[55], _iposv_54x1[55], _l1_55x1[56], _l3_54x1[55], _ia_54x1[55];
double _ba_27x1[28], _b0_54x1[55], _d_27x1[28], _grad_27x1[28], _lambda_27x1[28], _Ax_54x1[55], _temp_54x1[55], _temp2_54x1[55]; 
//-----------------------

//----------------------- for inv
int _indxc[35],_indxr[35],_ipiv[35];
//-----------------------

//----------------------- for svdcmp
double _rv1[35];
//-----------------------

//----------------------- for pinv_svd
double **_nrTEMP4_34x34, **_nrTEMP5_34x34, _s_33x1[34];
//-----------------------


extern float** _EYE_33;


//---------------------------------------

void nrerror(char error_text[])
/* Numerical Recipes standard error handler */
{
	printf("\nNumerical Recipes run-time error...\n");
	printf("%s\n",error_text);
}

//---------------------------------------

float *vector(unsigned int nl, unsigned int nh)
/* allocate a float vector with subscript range v[nl..nh] */
{
	float *v;

	v=(float *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(float)));
	if (!v) nrerror("allocation failure in vector()");
	return v-nl+NR_END;
}

//---------------------------------------

int *ivector(unsigned int nl, unsigned int nh)
/* allocate an int vector with subscript range v[nl..nh] */
{
	int *v;

	v=(int *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(int)));
	if (!v) nrerror("allocation failure in ivector()");
	return v-nl+NR_END;
}

//---------------------------------------

unsigned char *cvector(long nl, long nh)
/* allocate an unsigned char vector with subscript range v[nl..nh] */
{
	unsigned char *v;

	v=(unsigned char *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(unsigned char)));
	if (!v) nrerror("allocation failure in cvector()");
	return v-nl+NR_END;
}

//---------------------------------------

unsigned long *lvector(long nl, long nh)
/* allocate an unsigned long vector with subscript range v[nl..nh] */
{
	unsigned long *v;

	v=(unsigned long *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(long)));
	if (!v) nrerror("allocation failure in lvector()");
	return v-nl+NR_END;
}

//---------------------------------------

double *dvector(long nl, long nh)
/* allocate a double vector with subscript range v[nl..nh] */
{
	double *v;

	v=(double *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(double)));
	if (!v) nrerror("allocation failure in dvector()");
	return v-nl+NR_END;
}

//---------------------------------------

float **matrix(unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch)
/* allocate a float matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	unsigned int i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	float **m;

	/* allocate pointers to rows */
	m=(float **) malloc((size_t)((nrow+NR_END)*sizeof(float*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl]=(float *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

//---------------------------------------

double **dmatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	double **m;

	/* allocate pointers to rows */
	m=(double **) malloc((size_t)((nrow+NR_END)*sizeof(double*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl]=(double *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(double)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

//---------------------------------------

int **imatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a int matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	int **m;

	/* allocate pointers to rows */
	m=(int **) malloc((size_t)((nrow+NR_END)*sizeof(int*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;


	/* allocate rows and set pointers to them */
	m[nrl]=(int *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(int)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

//---------------------------------------

float **submatrix(float **a, long oldrl, long oldrh, long oldcl, long oldch,
	long newrl, long newcl)
/* point a submatrix [newrl..][newcl..] to a[oldrl..oldrh][oldcl..oldch] */
{
	long i,j,nrow=oldrh-oldrl+1,ncol=oldcl-newcl;
	float **m;

	/* allocate array of pointers to rows */
	m=(float **) malloc((size_t) ((nrow+NR_END)*sizeof(float*)));
	if (!m) nrerror("allocation failure in submatrix()");
	m += NR_END;
	m -= newrl;

	/* set pointers to rows */
	for(i=oldrl,j=newrl;i<=oldrh;i++,j++) m[j]=a[i]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

//---------------------------------------

float **convert_matrix(float *a, long nrl, long nrh, long ncl, long nch)
/* allocate a float matrix m[nrl..nrh][ncl..nch] that points to the matrix
declared in the standard C manner as a[nrow][ncol], where nrow=nrh-nrl+1
and ncol=nch-ncl+1. The routine should be called with the address
&a[0][0] as the first argument. */
{
	long i,j,nrow=nrh-nrl+1,ncol=nch-ncl+1;
	float **m;

	/* allocate pointers to rows */
	m=(float **) malloc((size_t) ((nrow+NR_END)*sizeof(float*)));
	if (!m) nrerror("allocation failure in convert_matrix()");
	m += NR_END;
	m -= nrl;

	/* set pointers to rows */
	m[nrl]=a-ncl;
	for(i=1,j=nrl+1;i<nrow;i++,j++) m[j]=m[j-1]+ncol;
	/* return pointer to array of pointers to rows */
	return m;
}

//---------------------------------------

float ***f3tensor(long nrl, long nrh, long ncl, long nch, long ndl, long ndh)
/* allocate a float 3tensor with range t[nrl..nrh][ncl..nch][ndl..ndh] */
{
	long i,j,nrow=nrh-nrl+1,ncol=nch-ncl+1,ndep=ndh-ndl+1;
	float ***t;

	/* allocate pointers to pointers to rows */
	t=(float ***) malloc((size_t)((nrow+NR_END)*sizeof(float**)));
	if (!t) nrerror("allocation failure 1 in f3tensor()");
	t += NR_END;
	t -= nrl;

	/* allocate pointers to rows and set pointers to them */
	t[nrl]=(float **) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float*)));
	if (!t[nrl]) nrerror("allocation failure 2 in f3tensor()");
	t[nrl] += NR_END;
	t[nrl] -= ncl;

	/* allocate rows and set pointers to them */
	t[nrl][ncl]=(float *) malloc((size_t)((nrow*ncol*ndep+NR_END)*sizeof(float)));
	if (!t[nrl][ncl]) nrerror("allocation failure 3 in f3tensor()");
	t[nrl][ncl] += NR_END;
	t[nrl][ncl] -= ndl;

	for(j=ncl+1;j<=nch;j++) t[nrl][j]=t[nrl][j-1]+ndep;
	for(i=nrl+1;i<=nrh;i++) {
		t[i]=t[i-1]+ncol;
		t[i][ncl]=t[i-1][ncl]+ncol*ndep;
		for(j=ncl+1;j<=nch;j++) t[i][j]=t[i][j-1]+ndep;
	}

	/* return pointer to array of pointers to rows */
	return t;
}

//---------------------------------------

void free_vector(float *v, unsigned int nl, unsigned int nh)
/* free a float vector allocated with vector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_ivector(int *v, unsigned int nl, unsigned int nh)
/* free an int vector allocated with ivector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_cvector(unsigned char *v, long nl, long nh)
/* free an unsigned char vector allocated with cvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_lvector(unsigned long *v, long nl, long nh)
/* free an unsigned long vector allocated with lvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_dvector(double *v, long nl, long nh)
/* free a double vector allocated with dvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_matrix(float **m, unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch)
/* free a float matrix allocated by matrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

//---------------------------------------

void free_dmatrix(double **m, long nrl, long nrh, long ncl, long nch)
/* free a double matrix allocated by dmatrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

//---------------------------------------

void free_imatrix(int **m, long nrl, long nrh, long ncl, long nch)
/* free an int matrix allocated by imatrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

//---------------------------------------

void free_submatrix(float **b, long nrl, long nrh, long ncl, long nch)
/* free a submatrix allocated by submatrix() */
{
	free((FREE_ARG) (b+nrl-NR_END));
}

//---------------------------------------

void free_convert_matrix(float **b, long nrl, long nrh, long ncl, long nch)
/* free a matrix allocated by convert_matrix() */
{
	free((FREE_ARG) (b+nrl-NR_END));
}

//---------------------------------------

void free_f3tensor(float ***t, long nrl, long nrh, long ncl, long nch,
	long ndl, long ndh)
/* free a float f3tensor allocated by f3tensor() */
{
	free((FREE_ARG) (t[nrl][ncl]+ndl-NR_END));
	free((FREE_ARG) (t[nrl]+ncl-NR_END));
	free((FREE_ARG) (t+nrl-NR_END));
}

//---------------------------------------

void simp1(double **a, int mm, int ll[], int nll, int iabf, int *kp, double *bmax)
{
	int k;
	double test;

	if (nll <= 0)
		*bmax=0.0;
	else {
		*kp=ll[1];
		*bmax=a[mm+1][*kp+1];
		for (k=2;k<=nll;k++) {
			if (iabf == 0)
				test=a[mm+1][ll[k]+1]-(*bmax);
			else
				test = (double)(fabs(a[mm+1][ll[k]+1])-fabs(*bmax));
			if (test > 0.) {
				*bmax=a[mm+1][ll[k]+1];
				*kp=ll[k];
			}
		}
	}
}

//---------------------------------------

void simp2(double **a, int m, int n, int *ip, int kp)
{
	int k,i;
	double qp,q0,q,q1;

	*ip=0;
	for (i=1;i<=m;i++)
		if (a[i+1][kp+1] < -EPS) break;
	if (i>m) return;
	q1 = -a[i+1][1]/a[i+1][kp+1];
	*ip=i;
	for (i=*ip+1;i<=m;i++) {
		if (a[i+1][kp+1] < -EPS) {
			q = -a[i+1][1]/a[i+1][kp+1];
			if (q < q1) {
				*ip=i;
				q1=q;
			} else if (q == q1) {
				for (k=1;k<=n;k++) {
					qp = -a[*ip+1][k+1]/a[*ip+1][kp+1];
					q0 = -a[i+1][k+1]/a[i+1][kp+1];
					if (q0 != qp) break;
				}
				if (q0 < qp) *ip=i;
			}
		}
	}
}

//---------------------------------------

void simp3(double **a, int i1, int k1, int ip, int kp)
{
	int kk,ii;
	double piv;

	piv=1.0f/a[ip+1][kp+1];
	for (ii=1;ii<=i1+1;ii++)
		if (ii-1 != ip) {
			a[ii][kp+1] *= piv;
			for (kk=1;kk<=k1+1;kk++)
				if (kk-1 != kp)
					a[ii][kk] -= a[ip+1][kk]*a[ii][kp+1];
		}
	for (kk=1;kk<=k1+1;kk++)
		if (kk-1 != kp) a[ip+1][kk] *= -piv;
	a[ip+1][kp+1]=piv;
}




/////////////////////////////////////////////////////// added by Inhyeok

//---------------------------------------
int mult_mv(const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_nx1, float *result_mx1)	// matrix x vector
{
	unsigned int i, j;
	float sum = 0.;

	for(i=1; i<=m; i++)
	{
		sum = 0.;		
		for(j=1; j<=n; j++)
			sum += matrix_mxn[i][j]*vector_nx1[j];
		
		result_mx1[i] = sum;
	}
	return 0;
}


//---------------------------------------
int mult_mv_(const double **matrix_mxn, unsigned int m, unsigned int n, const double *vector_nx1, double *result_mx1)	// matrix x vector
{
	unsigned int i, j;
	double sum = 0.;

	for(i=1; i<=m; i++)
	{
		sum = 0.;		
		for(j=1; j<=n; j++)
			sum += matrix_mxn[i][j]*vector_nx1[j];
		
		result_mx1[i] = sum;
	}
	return 0;
}

//---------------------------------------

int mult_mm(const float **matrix_m1xn1, unsigned int m1, unsigned int n1, const float *matrix_n1xn2[], unsigned int n2, float *result_m1xn2[]) // matrix x matrix
{
	unsigned int i,j,k;
	float sum = 0.;

	for(i=1; i<=m1; i++)
	{				
		for(j=1; j<=n2; j++)
		{
			sum = 0.;
			for(k=1; k<=n1; k++)
				sum += matrix_m1xn1[i][k]*matrix_n1xn2[k][j];
			
			result_m1xn2[i][j] = sum;
		}
	}
	return 0;
}

//---------------------------------------

int mult_mm_(const double **matrix_m1xn1, unsigned int m1, unsigned int n1, const double *matrix_n1xn2[], unsigned int n2, double *result_m1xn2[]) // matrix x matrix
{
	unsigned int i,j,k;
	double sum = 0.;

	for(i=1; i<=m1; i++)
	{				
		for(j=1; j<=n2; j++)
		{
			sum = 0.;
			for(k=1; k<=n1; k++)
				sum += matrix_m1xn1[i][k]*matrix_n1xn2[k][j];
			
			result_m1xn2[i][j] = sum;
		}
	}
	return 0;
}

//---------------------------------------

float dot(const float *vector1_nx1, unsigned int n, const float *vector2_nx1) // vector dot vector
{
	unsigned int i;
	float sum = 0.;

	for(i=1; i<=n; i++)
		sum += vector1_nx1[i]*vector2_nx1[i];

	return sum;
}

//---------------------------------------

int mult_vvt(float scalar, const float *vector_mx1, unsigned int m, const float *vector_nx1, unsigned int n, float **result_mxn) // vector * vector transpose
{
	unsigned int i, j;

	if(scalar != 1.f)
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_mxn[i][j] = vector_mx1[i]*vector_nx1[j];
	}
	else
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_mxn[i][j] = scalar*vector_mx1[i]*vector_nx1[j];		
	}
	

	return 0;

}

//---------------------------------------

int mult_vtm(float scalar, const float *vector_mx1, unsigned int m, const float **matrix_mxn, unsigned int n, float *result_nx1) // scalar * vector transpose * matrix
{
	unsigned int i, j;
	float sum;

	if(scalar == 1.f)
	{
		for(i=1; i<=n; i++)
		{
			sum = 0.;
			for(j=1; j<=m; j++)
				sum += vector_mx1[j]*matrix_mxn[j][i];
			
			result_nx1[i] = sum;
		}
	}
	else
	{
		for(i=1; i<=n; i++)
		{
			sum = 0.;
			for(j=1; j<=m; j++)
				sum += vector_mx1[j]*matrix_mxn[j][i];
			
			result_nx1[i] = scalar*sum;
		}		
	}
	return 0;
}

//---------------------------------------

int trans(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, float **result_nxm) // scalar * matrix transpose
{
	unsigned int i,j;

	if(scalar == 1.f)
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_nxm[j][i] = matrix_mxn[i][j];	
	}
	else
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_nxm[j][i] = scalar*matrix_mxn[i][j];		
	}

	return 0;
}

//---------------------------------------

int trans_(double scalar, const double **matrix_mxn, unsigned int m, unsigned int n, double **result_nxm) // scalar * matrix transpose
{
	unsigned int i,j;

	if(scalar == 1.)
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_nxm[j][i] = matrix_mxn[i][j];	
	}
	else
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_nxm[j][i] = scalar*matrix_mxn[i][j];		
	}

	return 0;
}

//---------------------------------------

int trans2(float scalar, float **matrix_mxn, unsigned int m, unsigned int n) // scalar * matrix transpose
{
	float temp;
	unsigned int i,j;
	unsigned int k = (unsigned int)IMAX(m,n);

	if(scalar == 1.f)
	{
		for(i=1; i<=k; i++)
		{
			for(j=1; j<i; j++)
			{
				temp = matrix_mxn[i][j];
				matrix_mxn[i][j] = matrix_mxn[j][i];
				matrix_mxn[j][i] = temp;
			}
		}
	}
	else
	{
		for(i=1; i<=k; i++)
		{
			for(j=1; j<i; j++)
			{
				temp = matrix_mxn[i][j];
				matrix_mxn[i][j] = scalar*matrix_mxn[j][i];
				matrix_mxn[j][i] = scalar*temp;
			}
		}		
	}

	return 0;
}


int trans2_(double scalar, double **matrix_mxn, unsigned int m, unsigned int n) // scalar * matrix transpose
{
	double temp;
	unsigned int i,j;
	unsigned int k = (unsigned int)IMAX(m,n);

	if(scalar == 1.)
	{
		for(i=1; i<=k; i++)
		{
			for(j=1; j<i; j++)
			{
				temp = matrix_mxn[i][j];
				matrix_mxn[i][j] = matrix_mxn[j][i];
				matrix_mxn[j][i] = temp;
			}
		}
	}
	else
	{
		for(i=1; i<=k; i++)
		{
			for(j=1; j<i; j++)
			{
				temp = matrix_mxn[i][j];
				matrix_mxn[i][j] = scalar*matrix_mxn[j][i];
				matrix_mxn[j][i] = scalar*temp;
			}
		}		
	}

	return 0;
}

//---------------------------------------

int sum_mm(const float **matrix1_mxn, unsigned int m, unsigned int n, const float **matrix2_mxn, float **result_mxn) // matrix + matrix
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = matrix1_mxn[i][j] + matrix2_mxn[i][j];

	return 0;
}

//---------------------------------------

int sum_smsm(float scalar1, const float **matrix1_mxn, unsigned int m, unsigned int n, float scalar2, const float **matrix2_mxn, float **result_mxn) // scalar1*matrix + scalar2*matrix
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = scalar1*matrix1_mxn[i][j] + scalar2*matrix2_mxn[i][j];

	return 0;
}

//---------------------------------------

int sum_smsm_(double scalar1, const double **matrix1_mxn, unsigned int m, unsigned int n, double scalar2, const double **matrix2_mxn, double **result_mxn) // scalar1*matrix + scalar2*matrix
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = scalar1*matrix1_mxn[i][j] + scalar2*matrix2_mxn[i][j];

	return 0;
}

//---------------------------------------

int diff_mm(const float **matrix1_mxn, unsigned int m, unsigned int n, const float **matrix2_mxn, float **result_mxn) // matrix - matrix
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = matrix1_mxn[i][j] - matrix2_mxn[i][j];

	return 0;
}

//---------------------------------------

int diff_mm_(const double **matrix1_mxn, unsigned int m, unsigned int n, const double **matrix2_mxn, double **result_mxn) // matrix - matrix
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = matrix1_mxn[i][j] - matrix2_mxn[i][j];

	return 0;
}

//---------------------------------------

int sum_vv(const float *vector1_mx1, unsigned int m, const float *vector2_mx1, float *result_mx1) // vector + vector
{
	unsigned int i;

	for(i=1; i<=m; i++)
		result_mx1[i] = vector1_mx1[i] + vector2_mx1[i];

	return 0;
}

//---------------------------------------

int diff_vv(const float *vector1_mx1, unsigned int m, const float *vector2_mx1, float *result_mx1) // vector - vector
{
	unsigned int i;

	for(i=1; i<=m; i++)
		result_mx1[i] = vector1_mx1[i] - vector2_mx1[i];

	return 0;
}

//---------------------------------------

int cross(float scalar, const float vector1_3x1[4], const float vector2_3x1[4], float result_3x1[4])	// scalar * vector cross product
{
	if(scalar == 1.f)
	{			
		result_3x1[1] = (vector1_3x1[2]*vector2_3x1[3] - vector1_3x1[3]*vector2_3x1[2]);
		result_3x1[2] = (vector1_3x1[3]*vector2_3x1[1] - vector1_3x1[1]*vector2_3x1[3]);
		result_3x1[3] = (vector1_3x1[1]*vector2_3x1[2] - vector1_3x1[2]*vector2_3x1[1]);
	}
	else
	{
		result_3x1[1] = scalar*(vector1_3x1[2]*vector2_3x1[3] - vector1_3x1[3]*vector2_3x1[2]);
		result_3x1[2] = scalar*(vector1_3x1[3]*vector2_3x1[1] - vector1_3x1[1]*vector2_3x1[3]);
		result_3x1[3] = scalar*(vector1_3x1[1]*vector2_3x1[2] - vector1_3x1[2]*vector2_3x1[1]);
	}

	return 0;
}

//---------------------------------------

int mult_sv(const float *vector_nx1, unsigned int n, float scalar, float *result_nx1)		// scalar * vector
{
	unsigned int i;

	for(i=1; i<=n; i++)
		result_nx1[i] = scalar*vector_nx1[i];

	return 0;
}

//---------------------------------------

int mult_sm(const float **matrix_mxn, unsigned int m, unsigned int n, float scalar, float **result_mxn) // scalar * matrix
{
	unsigned int i,j;

	for(i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = scalar*matrix_mxn[i][j];

	return 0;
}

//---------------------------------------

int subs_v(const float *vector_nx1, unsigned int n, float *result_nx1)		// result_nx1 = vector_nx1
{
	unsigned int i;
	
	for(i=1; i<=n; i++)
		result_nx1[i] = vector_nx1[i];

	return 0;
}

//---------------------------------------

int subs_sv(float scalar, const float *vector_nx1, unsigned int n, float *result_nx1)		// result_nx1 = scalar*vector_nx1
{
	unsigned int i;
	
	for(i=1; i<=n; i++)
		result_nx1[i] = scalar*vector_nx1[i];

	return 0;
}

//---------------------------------------

int subs_sv_(double scalar, const double *vector_nx1, unsigned int n, double *result_nx1)		// result_nx1 = scalar*vector_nx1
{
	unsigned int i;
	
	for(i=1; i<=n; i++)
		result_nx1[i] = scalar*vector_nx1[i];

	return 0;
}

//---------------------------------------

int subs_m(const float **matrix_mxn, unsigned int m, unsigned int n, float **result_mxn) // result_mxn = matrix_mxn
{

	unsigned int i,j;

	for(i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = matrix_mxn[i][j];

	return 0;
}

//---------------------------------------

int sum_svsv(float scalar1, const float *vector1_nx1, unsigned int n, float scalar2, const float *vector2_nx1, float *result_nx1) // s1*v1 + s2*v2
{
	unsigned int i;

	for(i=1; i<=n; i++)
		result_nx1[i] = scalar1*vector1_nx1[i] + scalar2*vector2_nx1[i];

	return 0;
}

//---------------------------------------

int sum_svsv_(double scalar1, const double *vector1_nx1, unsigned int n, double scalar2, const double *vector2_nx1, double *result_nx1) // s1*v1 + s2*v2
{
	unsigned int i;

	for(i=1; i<=n; i++)
		result_nx1[i] = scalar1*vector1_nx1[i] + scalar2*vector2_nx1[i];

	return 0;
}

//---------------------------------------

int mult_smv(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_nx1, float *result_mx1)	// scalar * matrix * vector
{
	unsigned int i, j;
	float sum = 0.;

	for(i=1; i<=m; i++)
	{
		sum = 0.;		
		for(j=1; j<=n; j++)
			sum += matrix_mxn[i][j]*vector_nx1[j];
		
		result_mx1[i] = scalar*sum;
	}
	return 0;
}

//---------------------------------------

int mult_smv_(double scalar, const double **matrix_mxn, unsigned int m, unsigned int n, const double *vector_nx1, double *result_mx1)	// scalar * matrix * vector
{
	unsigned int i, j;
	double sum = 0.;

	for(i=1; i<=m; i++)
	{
		sum = 0.;		
		for(j=1; j<=n; j++)
			sum += matrix_mxn[i][j]*vector_nx1[j];
		
		result_mx1[i] = scalar*sum;
	}
	return 0;
}

//---------------------------------------

int mult_smm(float scalar, const float **matrix_m1xn1, unsigned int m1, unsigned int n1, const float **matrix_n1xn2, unsigned int n2, float **result_m1xn2) // scalar*matrix*matrix
{
	unsigned int i,j,k;
	float sum = 0.;

	for(i=1; i<=m1; i++)
	{				
		for(j=1; j<=n2; j++)
		{
			sum = 0.;
			for(k=1; k<=n1; k++)
				sum += matrix_m1xn1[i][k]*matrix_n1xn2[k][j];
			
			result_m1xn2[i][j] = scalar*sum;
		}
	}
	return 0;
}

//---------------------------------------

int accu_vv(float scalar, const float *vector_n1x1, unsigned int n1, const float *vector_n2x1, unsigned int n2, float *result_n1n2x1) // accumulate two vectors
{
	unsigned int i;

	if(scalar == 1.f)
	{
		for(i=1; i<=n1; i++)
			result_n1n2x1[i] = vector_n1x1[i];

		for(i=1; i<=n2; i++)
			result_n1n2x1[n1+i] = vector_n2x1[i];
	}
	else
	{
		for(i=1; i<=n1; i++)
			result_n1n2x1[i] = scalar*vector_n1x1[i];

		for(i=1; i<=n2; i++)
			result_n1n2x1[n1+i] = scalar*vector_n2x1[i];
	}

	return 0;
}

//---------------------------------------

int aug_vv(float scalar, const float *vector1_nx1, unsigned int n, const float *vector2_nx1, float **result_nx2) // augment a vector
{
	unsigned int i;

	if(scalar == 1.f)
	{
		for(i=1; i<=n; i++)
		{
			result_nx2[i][1] = vector1_nx1[i];
			result_nx2[i][2] = vector2_nx1[i];
		}
	}
	else
	{
		for(i=1; i<=n; i++)
		{
			result_nx2[i][1] = scalar*vector1_nx1[i];
			result_nx2[i][2] = scalar*vector2_nx1[i];
		}
	}

	

	return 0;
}

//---------------------------------------

int accu_mm(float scalar, const float **matrix_m1xn, unsigned int m1, unsigned int n, const float **matrix_m2xn, unsigned int m2, float **result_m1m2xn) // accumulate two matrices
{
	unsigned int i, j;

	if(scalar == 1.f)
	{		
		for(j=1; j<=n; j++)
		{
			for(i=1; i<=m1; i++)		
				result_m1m2xn[i][j] = matrix_m1xn[i][j];

			for(i=1; i<=m2; i++)
				result_m1m2xn[m1+i][j] = matrix_m2xn[i][j];
		}
	}
	else
	{		
		for(j=1; j<=n; j++)
		{
			for(i=1; i<=m1; i++)		
				result_m1m2xn[i][j] = scalar*matrix_m1xn[i][j];

			for(i=1; i<=m2; i++)
				result_m1m2xn[m1+i][j] = scalar*matrix_m2xn[i][j];
		}
	}

	return 0;
}

//---------------------------------------

int aug_mm(float scalar, const float **matrix_mxn1, unsigned int m, unsigned int n1, const float **matrix_mxn2, unsigned int n2, float **result_mxn1n2) // augment a matrix
{
	unsigned int i, j;

	if(scalar == 1.f)
	{
		for(i=1; i<=m; i++)
		{
			for(j=1; j<=n1; j++)
				result_mxn1n2[i][j] = matrix_mxn1[i][j];

			for(j=1; j<=n2; j++)
				result_mxn1n2[i][j+n1] = matrix_mxn2[i][j];
		}
	}
	else
	{
		for(i=1; i<=m; i++)
		{
			for(j=1; j<=n1; j++)
				result_mxn1n2[i][j] = scalar*matrix_mxn1[i][j];

			for(j=1; j<=n2; j++)
				result_mxn1n2[i][j+n1] = scalar*matrix_mxn2[i][j];
		}
	}


	return 0;
}

//---------------------------------------

int aug_mv(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_mx1, float **result_mxn1) // augment a matrix
{
	unsigned int i,j;

	if(scalar == 1.f)
	{
		for(i=1; i<=m; i++)
		{
			for(j=1; j<=n; j++)
				result_mxn1[i][j] = matrix_mxn[i][j];

			result_mxn1[i][n+1] = vector_mx1[i];
		}
	}
	else
	{
		for(i=1; i<=m; i++)
		{
			for(j=1; j<=n; j++)
				result_mxn1[i][j] = scalar*matrix_mxn[i][j];

			result_mxn1[i][n+1] = scalar*vector_mx1[i];
		}
	}

	
	return 0;
}

//---------------------------------------

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int gaussj_mod(float **A, int n, float *X)	// n<=29
{	
	int i,icol,irow,j,k,l,ll;
	float big,dum,pivinv,temp;
//	int *indxc,*indxr,*ipiv;

//	indxc=ivector(1,n);
//	indxr=ivector(1,n);
//	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) 
		_ipiv[j]=0;
	
	for (i=1;i<=n;i++)
	{
		big=0.;
		for (j=1;j<=n;j++)
		{
			if (_ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (_ipiv[k] == 0)
					{
						if (fabs(A[j][k]) >= big)
						{
							big=(float)fabs(A[j][k]);
							irow=j;	
							icol=k;
						}
					}
				}
			}
		}
		++(_ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++)
				SWAP(A[irow][l],A[icol][l])
			SWAP(X[irow],X[icol])
		}
		_indxr[i]=irow;
		_indxc[i]=icol;
		if (A[icol][icol] == 0.0) 
			return 1;
		pivinv=1.f/A[icol][icol];
		A[icol][icol]=1.f;
		for (l=1;l<=n;l++) 
			A[icol][l] *= pivinv;
		X[icol] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=A[ll][icol];
				A[ll][icol]=0.0;
				for (l=1;l<=n;l++) 
					A[ll][l] -= A[icol][l]*dum;
				X[ll] -= X[icol]*dum;
			}
	}

	for (l=n;l>=1;l--) {
		if (_indxr[l] != _indxc[l])
			for (k=1;k<=n;k++)
				SWAP(A[k][_indxr[l]],A[k][_indxc[l]]);
	}
//	free_ivector(ipiv,1,n);
//	free_ivector(indxr,1,n);
//	free_ivector(indxc,1,n);

	return 0;
}
#undef SWAP

//---------------------------------------

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int inv(float **Ai, int n)
{
//	int *indxc,*indxr,*ipiv;
	int i,icol,irow,j,k,l,ll;
	float big,dum,pivinv,temp;
	
//	indxc=ivector(1,n);
//	indxr=ivector(1,n);
//	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) 
		_ipiv[j]=0;
	
	for (i=1;i<=n;i++)
	{
		big=0.;
		for (j=1;j<=n;j++)
		{
			if (_ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (_ipiv[k] == 0)
					{
						if (fabs(Ai[j][k]) >= big)
						{
							big=(float)fabs(Ai[j][k]);
							irow=j;	
							icol=k;
						}
					}
				}
			}
		}
		++(_ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++)
				SWAP(Ai[irow][l],Ai[icol][l])
		}
		_indxr[i]=irow;
		_indxc[i]=icol;
		if (fabs(Ai[icol][icol]) <= EPS) 
			return 1;
		pivinv=1.f/Ai[icol][icol];
		Ai[icol][icol]=1.f;
		for (l=1;l<=n;l++) 
			Ai[icol][l] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=Ai[ll][icol];
				Ai[ll][icol]=0.0;
				for (l=1;l<=n;l++) 
					Ai[ll][l] -= Ai[icol][l]*dum;
			}
	}

	for (l=n;l>=1;l--) {
		if (_indxr[l] != _indxc[l])
			for (k=1;k<=n;k++)
				SWAP(Ai[k][_indxr[l]],Ai[k][_indxc[l]]);
	}
//	free_ivector(ipiv,1,n);
//	free_ivector(indxr,1,n);
//	free_ivector(indxc,1,n);
	
	return 0;
}
#undef SWAP

//---------------------------------------

//---------------------------------------

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int inv_(double **Ai, int n)
{
//	int *indxc,*indxr,*ipiv;
	int i,icol,irow,j,k,l,ll;
	double big,dum,pivinv,temp;
	
//	indxc=ivector(1,n);
//	indxr=ivector(1,n);
//	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) 
		_ipiv[j]=0;
	
	for (i=1;i<=n;i++)
	{
		big=0.;
		for (j=1;j<=n;j++)
		{
			if (_ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (_ipiv[k] == 0)
					{
						if (fabs(Ai[j][k]) >= big)
						{
							big=fabs(Ai[j][k]);
							irow=j;	
							icol=k;
						}
					}
				}
			}
		}
		++(_ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++)
				SWAP(Ai[irow][l],Ai[icol][l])
		}
		_indxr[i]=irow;
		_indxc[i]=icol;
		if (Ai[icol][icol] == 0.0) 
			return 1;
		pivinv=1.f/Ai[icol][icol];
		Ai[icol][icol]=1.f;
		for (l=1;l<=n;l++) 
			Ai[icol][l] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=Ai[ll][icol];
				Ai[ll][icol]=0.0;
				for (l=1;l<=n;l++) 
					Ai[ll][l] -= Ai[icol][l]*dum;
			}
	}

	for (l=n;l>=1;l--) {
		if (_indxr[l] != _indxc[l])
			for (k=1;k<=n;k++)
				SWAP(Ai[k][_indxr[l]],Ai[k][_indxc[l]]);
	}
//	free_ivector(ipiv,1,n);
//	free_ivector(indxr,1,n);
//	free_ivector(indxc,1,n);
	
	return 0;
}
#undef SWAP

//---------------------------------------

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int inv2(const float **A, int n, float **Ai)
{
//	int *indxc,*indxr,*ipiv;
	int i,icol,irow,j,k,l,ll;
	float big,dum,pivinv,temp;

	subs_m(A,n,n, Ai);

//	indxc=ivector(1,n);
//	indxr=ivector(1,n);
//	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) 
		_ipiv[j]=0;
	
	for (i=1;i<=n;i++)
	{
		big=0.;
		for (j=1;j<=n;j++)
		{
			if (_ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (_ipiv[k] == 0)
					{
						if (fabs(Ai[j][k]) >= big)
						{
							big=(float)fabs(Ai[j][k]);
							irow=j;	
							icol=k;
						}
					}
				}
			}
		}
		++(_ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++)
				SWAP(Ai[irow][l],Ai[icol][l])
		}
		_indxr[i]=irow;
		_indxc[i]=icol;
		if (Ai[icol][icol] == 0.0) 
			return 1;
		pivinv=1.f/Ai[icol][icol];
		Ai[icol][icol]=1.f;
		for (l=1;l<=n;l++) 
			Ai[icol][l] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=Ai[ll][icol];
				Ai[ll][icol]=0.0;
				for (l=1;l<=n;l++) 
					Ai[ll][l] -= Ai[icol][l]*dum;
			}
	}

	for (l=n;l>=1;l--) {
		if (_indxr[l] != _indxc[l])
			for (k=1;k<=n;k++)
				SWAP(Ai[k][_indxr[l]],Ai[k][_indxc[l]]);
	}
//	free_ivector(ipiv,1,n);
//	free_ivector(indxr,1,n);
//	free_ivector(indxc,1,n);
	
	return 0;
}
#undef SWAP

//---------------------------------------

int svdcmp(float **a, int m, int n, float w[], float **v)
{	
	int flag,i,its,j,jj,k,l,nm;
	float anorm,c,f,g,h,s,scale,x,y,z;
//	float *rv1;

//	rv1=vector(1,n);
	g=scale=anorm=0.0;
	for (i=1;i<=n;i++) 
	{
		l=i+1;
		_rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i <= m) 
		{
			for (k=i;k<=m;k++) 
				scale += (float)fabs(a[k][i]);
			if (scale) 
			{
				for (k=i;k<=m;k++) 
				{
					a[k][i] /= scale;
					s += a[k][i]*a[k][i];
				}
				f=a[i][i];
				g = -(float)SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][i]=f-g;
				for (j=l;j<=n;j++) 
				{
					for (s=0.0,k=i;k<=m;k++) 
						s += a[k][i]*a[k][j];
					f=s/h;
					for (k=i;k<=m;k++) 
						a[k][j] += f*a[k][i];
				}
				for (k=i;k<=m;k++) 
					a[k][i] *= scale;
			}
		}
		w[i]=scale *g;
		g=s=scale=0.0;
		if (i <= m && i != n) 
		{
			for (k=l;k<=n;k++) 
				scale += (float)fabs(a[i][k]);
			if (scale) 
			{
				for (k=l;k<=n;k++) 
				{
					a[i][k] /= scale;
					s += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g = -(float)SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][l]=f-g;
				for (k=l;k<=n;k++) 
					_rv1[k]=a[i][k]/h;
				for (j=l;j<=m;j++) 
				{
					for (s=0.0,k=l;k<=n;k++) 
						s += a[j][k]*a[i][k];
					for (k=l;k<=n;k++) 
						a[j][k] += s*(float)_rv1[k];
				}
				for (k=l;k<=n;k++) 
					a[i][k] *= scale;
			}
		}
		anorm = (float)FMAX(anorm,(float)(fabs(w[i])+fabs(_rv1[i])));
	}
	for (i=n;i>=1;i--) 
	{
		if (i < n) 
		{
			if (g) 
			{
				for (j=l;j<=n;j++)
					v[j][i]=(a[i][j]/a[i][l])/g;
				for (j=l;j<=n;j++) 
				{
					for (s=0.0,k=l;k<=n;k++) 
						s += a[i][k]*v[k][j];
					for (k=l;k<=n;k++) 
						v[k][j] += s*v[k][i];
				}
			}
			for (j=l;j<=n;j++) 
				v[i][j]=v[j][i]=0.0;
		}
		v[i][i]=1.0;
		g=(float)_rv1[i];
		l=i;
	}
	for (i=IMIN(m,n);i>=1;i--) 
	{
		l=i+1;
		g=w[i];
		for (j=l;j<=n;j++) 
			a[i][j]=0.0;
		if (g) {
			g=1.f/g;
			for (j=l;j<=n;j++) 
			{
				for (s=0.0,k=l;k<=m;k++) 
					s += a[k][i]*a[k][j];
				f=(s/a[i][i])*g;
				for (k=i;k<=m;k++) 
					a[k][j] += f*a[k][i];
			}
			for (j=i;j<=m;j++) 
				a[j][i] *= g;
		} 
		else 
			for (j=i;j<=m;j++) a[j][i]=0.0;
		++a[i][i];
	}
	for (k=n;k>=1;k--) 
	{
		for (its=1;its<=30;its++) 
		{
			flag=1;
			for (l=k;l>=1;l--) 
			{
				nm=l-1;
				//if ((float)(fabs(_rv1[l])+anorm) == anorm) 
				if (fabs(_rv1[l]) <= EPS*0.5f) 
				{
					flag=0;
					break;
				}
				//if ((float)(fabs(w[nm])+anorm) == anorm) 
				if (fabs(w[nm])<= EPS*0.5f) 
					break;
			}
			if (flag) 
			{
				c=0.0;
				s=1.0;
				for (i=l;i<=k;i++) 
				{
					f=s*(float)_rv1[i];
					_rv1[i]=c*_rv1[i];
					//if ((float)(fabs(f)+anorm) == anorm) 
					if (fabs(f)<= EPS*0.5f) 
						break;
					g=w[i];
					h=pythag(f,g);
					w[i]=h;
					h=1.f/h;
					c=g*h;
					s = -f*h;
					for (j=1;j<=m;j++) 
					{
						y=a[j][nm];
						z=a[j][i];
						a[j][nm]=y*c+z*s;
						a[j][i]=z*c-y*s;
					}
				}
			}
			z=w[k];
			if (l == k) 
			{
				if (z < 0.0) 
				{
					w[k] = -z;
					for (j=1;j<=n;j++) 
						v[j][k] = -v[j][k];
				}
				break;
			}
			if (its == 30) 
			{
				nrerror("no convergence in 30 svdcmp iterations");
				return -1;
			}
			x=w[l];
			nm=k-1;
			y=w[nm];
			g=(float)_rv1[nm];
			h=(float)_rv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.f*h*y);
			g=pythag(f,1.f);
			f=((x-z)*(x+z)+h*((y/(f+(float)SIGN(g,f)))-h))/x;
			c=s=1.f;
			for (j=l;j<=nm;j++) 
			{
				i=j+1;
				g=(float)_rv1[i];
				y=w[i];
				h=s*g;
				g=c*g;
				z=pythag(f,h);
				_rv1[j]=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g = g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=1;jj<=n;jj++) 
				{
					x=v[jj][j];
					z=v[jj][i];
					v[jj][j]=x*c+z*s;
					v[jj][i]=z*c-x*s;
				}
				z=pythag(f,h);
				w[j]=z;
				if (z) 
				{
					z=1.f/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=1;jj<=m;jj++) 
				{
					y=a[jj][j];
					z=a[jj][i];
					a[jj][j]=y*c+z*s;
					a[jj][i]=z*c-y*s;
				}
			}
			_rv1[l]=0.0;
			_rv1[k]=f;
			w[k]=x;
		}
	}
	
//	free_vector(rv1,1,n);
	return 0;
}


int svdcmp_(double **a, int m, int n, double w[], double **v)
{	
	int flag,i,its,j,jj,k,l,nm;
	double anorm,c,f,g,h,s,scale,x,y,z;
//	float *rv1;

//	rv1=vector(1,n);
	g=scale=anorm=0.0;
	for (i=1;i<=n;i++) 
	{
		l=i+1;
		_rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i <= m) 
		{
			for (k=i;k<=m;k++) 
				scale += fabs(a[k][i]);
			if (scale) 
			{
				for (k=i;k<=m;k++) 
				{
					a[k][i] /= scale;
					s += a[k][i]*a[k][i];
				}
				f=a[i][i];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][i]=f-g;
				for (j=l;j<=n;j++) 
				{
					for (s=0.0,k=i;k<=m;k++) 
						s += a[k][i]*a[k][j];
					f=s/h;
					for (k=i;k<=m;k++) 
						a[k][j] += f*a[k][i];
				}
				for (k=i;k<=m;k++) 
					a[k][i] *= scale;
			}
		}
		w[i]=scale *g;
		g=s=scale=0.0;
		if (i <= m && i != n) 
		{
			for (k=l;k<=n;k++) 
				scale += fabs(a[i][k]);
			if (scale) 
			{
				for (k=l;k<=n;k++) 
				{
					a[i][k] /= scale;
					s += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][l]=f-g;
				for (k=l;k<=n;k++) 
					_rv1[k]=a[i][k]/h;
				for (j=l;j<=m;j++) 
				{
					for (s=0.0,k=l;k<=n;k++) 
						s += a[j][k]*a[i][k];
					for (k=l;k<=n;k++) 
						a[j][k] += s*_rv1[k];
				}
				for (k=l;k<=n;k++) 
					a[i][k] *= scale;
			}
		}
		anorm = DMAX(anorm,(fabs(w[i])+fabs(_rv1[i])));
	}
	for (i=n;i>=1;i--) 
	{
		if (i < n) 
		{
			if (g) 
			{
				for (j=l;j<=n;j++)
					v[j][i]=(a[i][j]/a[i][l])/g;
				for (j=l;j<=n;j++) 
				{
					for (s=0.0,k=l;k<=n;k++) 
						s += a[i][k]*v[k][j];
					for (k=l;k<=n;k++) 
						v[k][j] += s*v[k][i];
				}
			}
			for (j=l;j<=n;j++) 
				v[i][j]=v[j][i]=0.0;
		}
		v[i][i]=1.0;
		g=_rv1[i];
		l=i;
	}
	for (i=IMIN(m,n);i>=1;i--) 
	{
		l=i+1;
		g=w[i];
		for (j=l;j<=n;j++) 
			a[i][j]=0.0;
		if (g) {
			g=1.f/g;
			for (j=l;j<=n;j++) 
			{
				for (s=0.0,k=l;k<=m;k++) 
					s += a[k][i]*a[k][j];
				f=(s/a[i][i])*g;
				for (k=i;k<=m;k++) 
					a[k][j] += f*a[k][i];
			}
			for (j=i;j<=m;j++) 
				a[j][i] *= g;
		} 
		else 
			for (j=i;j<=m;j++) a[j][i]=0.0;
		++a[i][i];
	}
	for (k=n;k>=1;k--) 
	{
		for (its=1;its<=30;its++) 
		{
			flag=1;
			for (l=k;l>=1;l--) 
			{
				nm=l-1;
				//if ((float)(fabs(_rv1[l])+anorm) == anorm) 
				if (fabs(_rv1[l]) <= EPS*0.5f) 
				{
					flag=0;
					break;
				}
				//if ((float)(fabs(w[nm])+anorm) == anorm) 
				if (fabs(w[nm])<= EPS*0.5f) 
					break;
			}
			if (flag) 
			{
				c=0.0;
				s=1.0;
				for (i=l;i<=k;i++) 
				{
					f=s*_rv1[i];
					_rv1[i]=c*_rv1[i];
					//if ((float)(fabs(f)+anorm) == anorm) 
					if (fabs(f)<= EPS*0.5f) 
						break;
					g=w[i];
					h=pythag_(f,g);
					w[i]=h;
					h=1.f/h;
					c=g*h;
					s = -f*h;
					for (j=1;j<=m;j++) 
					{
						y=a[j][nm];
						z=a[j][i];
						a[j][nm]=y*c+z*s;
						a[j][i]=z*c-y*s;
					}
				}
			}
			z=w[k];
			if (l == k) 
			{
				if (z < 0.0) 
				{
					w[k] = -z;
					for (j=1;j<=n;j++) 
						v[j][k] = -v[j][k];
				}
				break;
			}
			if (its == 30) 
			{
				nrerror("no convergence in 30 svdcmp iterations");
				return -1;
			}
			x=w[l];
			nm=k-1;
			y=w[nm];
			g=_rv1[nm];
			h=_rv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.f*h*y);
			g=pythag_(f,1.f);
			f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
			c=s=1.f;
			for (j=l;j<=nm;j++) 
			{
				i=j+1;
				g=_rv1[i];
				y=w[i];
				h=s*g;
				g=c*g;
				z=pythag_(f,h);
				_rv1[j]=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g = g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=1;jj<=n;jj++) 
				{
					x=v[jj][j];
					z=v[jj][i];
					v[jj][j]=x*c+z*s;
					v[jj][i]=z*c-x*s;
				}
				z=pythag_(f,h);
				w[j]=z;
				if (z) 
				{
					z=1.f/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=1;jj<=m;jj++) 
				{
					y=a[jj][j];
					z=a[jj][i];
					a[jj][j]=y*c+z*s;
					a[jj][i]=z*c-y*s;
				}
			}
			_rv1[l]=0.0;
			_rv1[k]=f;
			w[k]=x;
		}
	}
	
//	free_vector(rv1,1,n);
	return 0;
}

//---------------------------------------

float pythag(float a, float b)
{
	float absa,absb;
	absa = (float)fabs(a);
	absb = (float)fabs(b);
	if (absa > absb) 
		return (float)(absa*sqrt(1.0+SQR(absb/absa)));
	else 
		return (float)(absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}

double pythag_(double a, double b)
{
	double absa,absb;
	absa = fabs(a);
	absb = fabs(b);
	if (absa > absb) 
		return (absa*sqrt(1.0+DSQR(absb/absa)));
	else 
		return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+DSQR(absa/absb)));
}

//---------------------------------------

int pinv(const float **A, int m, int n, float **Ai)
{
	int temp;
	int i,j;

	trans(1.f, A, m,n, (float**)_nrTEMP1_34x34);
	mult_mm(A,m,n, (const float**)_nrTEMP1_34x34,m, (float**)_nrTEMP2_34x34);
	temp = inv((float**)_nrTEMP2_34x34, m);
	mult_mm((const float**)_nrTEMP1_34x34,n,m, (const float**)_nrTEMP2_34x34,m, Ai);

	if(temp == 1)	// singularity
	{
		printf("\n singularity in pinv()\n");
		for(i=1; i<=m; i++)
		{
			for(j=1; j<=n; j++)
				printf("%f ", A[i][j]);
			printf("\n");
		}

		return -1;
	}
	return 0;
}


int pinv_SR(const float **A, int m, int n, float lambda, float **Ai)
{
	int temp;
	int i,j;

	if(m>33 || n>33)
	{
		printf("\n error - matrix dimension is out of scope!");
		return -2;
	}

	for(i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			_nrTEMP4_34x34[i][j] = A[i][j];


	trans_(1.f, (const double**)_nrTEMP4_34x34, m,n, _nrTEMP1_34x34);
	mult_mm_((const double**)_nrTEMP4_34x34,m,n, (const double**)_nrTEMP1_34x34,m, _nrTEMP2_34x34);
	sum_smsm_(1.f, (const double**)_nrTEMP2_34x34,m,m, lambda, (const double**)_EYE_34, _nrTEMP2_34x34);
	temp = inv_(_nrTEMP2_34x34, m);
	mult_mm_((const double**)_nrTEMP1_34x34,n,m, (const double**)_nrTEMP2_34x34,m, _nrTEMP4_34x34);

	if(temp != 0)	// singularity
	{
		printf("\n singularity in pinv_SR()\n");
		/*for(i=1; i<=m; i++)
		{
			for(j=1; j<=n; j++)
				printf("%f ", A[i][j]);
			printf("\n");
		}
		*/
		return -1;
	}
	else
	{
		for(i=1; i<=n; i++)
			for(j=1; j<=m; j++)
				Ai[i][j] = (float)_nrTEMP4_34x34[i][j];
	}

	return 0;
}

//---------------------------------------

int pinv_svd(const float **A, int m, int n, float **Ai)
{
	/*
	int i,j;
	double f_temp = 10.*EPS;

	for(i=1;i<=m;i++)
		for(j=1;j<=n;j++)
			_nrTEMP1_34x34[i][j] = A[i][j];

	trans_(1.f, (const double**)_nrTEMP1_34x34, m,n, _nrTEMP4_34x34);
	mult_mm_((const double**)_nrTEMP1_34x34,m,n, (const double**)_nrTEMP4_34x34,m, _nrTEMP5_34x34);
	for(i=1;i<=m;i++)
		for(j=1;j<=m;j++)
			_nrTEMP1_34x34[i][j] = _nrTEMP5_34x34[i][j];

	svdcmp_(_nrTEMP5_34x34,m,m, _s_33x1, _nrTEMP2_34x34);

	for(i=1;i<=m;i++)
	{
		if(_s_33x1[i] >= f_temp)
		{
			for(j=1;j<=m;j++)
				_nrTEMP5_34x34[j][i] /= _s_33x1[i];
		}
		else
		{
			for(j=1;j<=m;j++)
				_nrTEMP5_34x34[j][i] = 0.;
		}
	}

	trans2_(1.f,_nrTEMP5_34x34,m,m);
	mult_mm_((const double**)_nrTEMP2_34x34,m,m, (const double**)_nrTEMP5_34x34,m, _nrTEMP1_34x34);
	mult_mm_((const double**)_nrTEMP4_34x34,n,m, (const double**)_nrTEMP1_34x34,m, _nrTEMP5_34x34);

	for(i=1;i<=n;i++)
		for(j=1;j<=m;j++)
			Ai[i][j] = (float)_nrTEMP5_34x34[i][j];
	
	return 0;
	*/
	
	
	int i,j;
	double f_temp = 10.*EPS;

	for(i=1;i<=m;i++)
		for(j=1;j<=n;j++)
			_nrTEMP4_34x34[i][j] = A[i][j];

	if(svdcmp_(_nrTEMP4_34x34,m,n,_s_33x1,_nrTEMP5_34x34) == -1)
		return -1;

	for(i=1; i<=n; i++)
	{
		if(_s_33x1[i]>=f_temp)
		{
			for(j=1;j<=n;j++)
				_nrTEMP5_34x34[j][i] /= _s_33x1[i];
		}
		else
		{
			for(j=1;j<=n;j++)
			{
				_nrTEMP5_34x34[j][i] = 0.;			
				_nrTEMP4_34x34[j][i] = 0.;
			}
		}
	}


	trans2_(1.f, _nrTEMP4_34x34,m,n);

	mult_mm_((const double**)_nrTEMP5_34x34,n,n, (const double**)_nrTEMP4_34x34,m, _nrTEMP1_34x34);

	for(i=1;i<=n;i++)
		for(j=1;j<=m;j++)
			Ai[i][j] = (float)_nrTEMP1_34x34[i][j];

	return 0;
}

//---------------------------------------

#define SWAP(a,b) temp=(a);(a)=(b);(b)=temp;
#define SWAPi(a,b) tempi=(a);(a)=(b);(b)=tempi;
// example 
// arr_4x1[] = {0, 4, 3, 2, 1}
// index_4x1[] = {0, 1, 2, 3, 4}
//
// nrselect(1,4, arr_4x1, index_4x1)
// arr_4x1 : {0, 1, 2, 3, 4}
// index_4x1 : {0, 4, 3, 2, 1}
// returned value : 1

float nrselect(unsigned int k, unsigned int n, float arr[], unsigned int index[])	
{
	unsigned int i,ir,j,l,mid;
	float a,temp;
	unsigned int ind, tempi;

	l=1;
	ir=n;
	for (;;) {
		if (ir <= l+1) {
			if (ir == l+1 && arr[ir] < arr[l]) {
				SWAP(arr[l],arr[ir])
				SWAPi(index[l],index[ir])
			}
			return arr[k];
		} else {
			mid=(l+ir) >> 1;
			SWAP(arr[mid],arr[l+1])
			SWAPi(index[mid], index[l+1])
			if (arr[l] > arr[ir]) {
				SWAP(arr[l],arr[ir])
				SWAPi(index[l],index[ir])
			}
			if (arr[l+1] > arr[ir]) {
				SWAP(arr[l+1],arr[ir])
				SWAPi(index[l+1],index[ir])
			}
			if (arr[l] > arr[l+1]) {
				SWAP(arr[l],arr[l+1])
				SWAPi(index[l],index[l+1])
			}
			i=l+1;
			j=ir;
			a=arr[l+1];
			ind = index[l+1];
			for (;;) {
				do i++; while (arr[i] < a);
				do j--; while (arr[j] > a);
				if (j < i) break;
				SWAP(arr[i],arr[j])
				SWAPi(index[i],index[j])
			}
			arr[l+1]=arr[j];
			index[l+1] = index[j];
			arr[j]=a;
			index[j] = ind;
			if (j >= k) ir=j-1;
			if (j <= k) l=i;
		}
	}
}
#undef SWAP
#undef SWAPi

//---------------------------------------

float norm_v(const float* vector_nx1, unsigned int n)
{
	unsigned int i;
	float sum = 0.;

	for(i=1; i<=n; i++)
		sum += vector_nx1[i]*vector_nx1[i];

	return((float)sqrt(sum));
}

//---------------------------------------

double norm_v_(const double* vector_nx1, unsigned int n)
{
	unsigned int i;
	double sum = 0.;

	for(i=1; i<=n; i++)
		sum += vector_nx1[i]*vector_nx1[i];

	return((double)sqrt(sum));
}

//---------------------------------------

int findminmax(float data[], unsigned int n, float *result_max, float *result_min, unsigned int *i_max, unsigned int *i_min)
{
	unsigned int i, n_max, n_min;
	float max = -1.e9;
	float min = 1.e9;

	for(i=0; i<n; i++)
	{
		if(max < data[i])
		{
			max = data[i];
			n_max = i;
		}		
		
		if(min > data[i])
		{
			min = data[i];
			n_min = i;
		}
	}

	*result_max = max;
	*i_max = n_max;

	*result_min = min;
	*i_min = n_min;

	return 0;
}

//---------------------------------------

int findmax(float data[], unsigned int n, float *result_max, unsigned int *i_max)
{
	unsigned int i, n_max;
	float max = -1.e9;

	for(i=0; i<n; i++)
	{
		if(max < data[i])
		{
			max = data[i];
			n_max = i;
		}				
	}

	*result_max = max;
	*i_max = n_max;

	return 0;
}

//---------------------------------------

int findmin(float data[], unsigned int n, float *result_min, unsigned int *i_min)
{
	unsigned int i, n_min;
	float min = 1.e9;

	for(i=0; i<n; i++)
	{
		if(min > data[i])
		{
			min = data[i];
			n_min = i;
		}				
	}

	*result_min = min;
	*i_min = n_min;

	return 0;
}

//---------------------------------------

int InitGlobalNRutilVariables(void)
{
	int i,j;

	_A_56x56 = dmatrix(1,56,1,56);
	_Aa_27x27 = dmatrix(1,27,1,27);
	_A0_54x27 = dmatrix(1,54,1,27);
	_N_27x27 = dmatrix(1,27,1,27);
	_EYE_34 = dmatrix(1,34,1,34);
	_nrTEMP1_34x34 = dmatrix(1,34,1,34);
	_nrTEMP2_34x34 = dmatrix(1,34,1,34);
	_nrTEMP3_34x34 = dmatrix(1,34,1,34);
	_nrTEMP4_34x34 = dmatrix(1,34,1,34);
	_nrTEMP5_34x34 = dmatrix(1,34,1,34);
	

	for(i=1; i<=34; i++)
		for(j=1; j<=34; j++)
		{
			if(i == j)
				_EYE_34[i][j] = 1.;
			else
				_EYE_34[i][j] = 0.;	
		}

	return 0;
}

//---------------------------------------

int FreeGlobalNRutilVariables(void)
{
	free_dmatrix(_A_56x56,1,56,1,56);
	free_dmatrix(_Aa_27x27,1,27,1,27);
	free_dmatrix(_A0_54x27,1,54,1,27);
	free_dmatrix(_N_27x27,1,27,1,27);
	free_dmatrix(_EYE_34,1,34,1,34);
	free_dmatrix(_nrTEMP1_34x34, 1,34,1,34);
	free_dmatrix(_nrTEMP2_34x34, 1,34,1,34);
	free_dmatrix(_nrTEMP3_34x34, 1,34,1,34);
	free_dmatrix(_nrTEMP4_34x34, 1,34,1,34);
	free_dmatrix(_nrTEMP5_34x34, 1,34,1,34);

	return 0;
}

//---------------------------------------

#define my_min(a,b) (((a)<(b)) ? (a) : (b))
int my_qp(const float **A_mxn, int m, int n, float bu_mx1[], float bl_mx1[], int n_opt, float result_nx1[])   // Quadratic Programming, finding x such that min 1/2*|x(1:n_opt)|^2 subject to bl<=Ax<=bu
{																										      //  m<=25, n<=25																									
	int n2 = 2*n;
	int m1 = 0;
	int m2 = 0;
	int m3 = 0;
	int m3_temp = 0;
	int m_, na, leaving;

	int i,j,k, i_temp;
	double f_temp;
	
	int nl1, kp, ip, kh, is;	
	double q1, bmax;

	double alpha;

	for(i=1; i<=m; i++)
	{
		if(fabs(bu_mx1[i]-bl_mx1[i]) < EPS)
			m3++;			
	}

	m_ = 2*m-m3;

	for(i=1;i<=m;i++)
	{
		if(fabs(bu_mx1[i]-bl_mx1[i]) < EPS)
		{
			m3_temp++;
			if(bu_mx1[i]<0.)
			{
				_A_56x56[m_-m3_temp][1] = -bu_mx1[i];
				for(j=1;j<=n;j++)
				{
					_A_56x56[m_-m3_temp+2][j+1] = A_mxn[i][j];
					_A_56x56[m_-m3_temp+2][j+1+n] = -A_mxn[i][j];
					_A0_54x27[m_-m3_temp+1][j] = A_mxn[i][j];					
				}				
			}
			else
			{
				_A_56x56[m_-m3_temp][1] = bu_mx1[i];
				for(j=1;j<=n;j++)
				{
					_A_56x56[m_-m3_temp+2][j+1] = -A_mxn[i][j];
					_A_56x56[m_-m3_temp+2][j+1+n] = A_mxn[i][j];
					_A0_54x27[m_-m3_temp+1][j] = A_mxn[i][j];
				
				}
			}
			_b0_54x1[m_-m3_temp+1] = bu_mx1[i];			
		}
		else
		{
			if(bu_mx1[i] >= 0.)
			{
				m1++;
				_A_56x56[1+m1][1] = bu_mx1[i];
				for(j=1;j<=n;j++)
				{
					_A_56x56[1+m1][j+1] = -A_mxn[i][j];
					_A_56x56[1+m1][j+1+n] = A_mxn[i][j];
					_A0_54x27[m1][j] = A_mxn[i][j];					
				}
				_b0_54x1[m1] = bu_mx1[i];
			}
			else
			{
				m2++;
				_A_56x56[m_-m3-m2+2][1] = -bu_mx1[i];
				for(j=1;j<=n;j++)
				{
					_A_56x56[m_-m3-m2+2][j+1] = A_mxn[i][j];
					_A_56x56[m_-m3-m2+2][j+1+n] = -A_mxn[i][j];
					_A0_54x27[m_-m3-m2+1][j] = A_mxn[i][j];					
				}
				_b0_54x1[m_-m3-m2+1] = bu_mx1[i];
			}

			if(bl_mx1[i] >= 0.)
			{
				m2++;
				_A_56x56[m_-m3-m2+2][1] = bl_mx1[i];
				for(j=1;j<=n;j++)
				{
					_A_56x56[m_-m3-m2+2][j+1] = -A_mxn[i][j];
					_A_56x56[m_-m3-m2+2][j+1+n] = A_mxn[i][j];
					_A0_54x27[m_-m3-m2+1][j] = -A_mxn[i][j];					
				}
				_b0_54x1[m_-m3-m2+1] = -bl_mx1[i];				
			}
			else
			{
				m1++;
				_A_56x56[1+m1][1] = -bl_mx1[i];
				for(j=1;j<=n;j++)
				{
					_A_56x56[1+m1][j+1] = A_mxn[i][j];
					_A_56x56[1+m1][j+1+n] = -A_mxn[i][j];
					_A0_54x27[m1][j] = -A_mxn[i][j];					
				}
				_b0_54x1[m1] = -bl_mx1[i];				
			}
		}
	}

	if((m2+m3) > 0)
	{
		for(i=1;i<=n2;i++)
		{
			_l1_55x1[i] = i;
			_izrov_54x1[i] = i;
		}

		for(i=1;i<=m_;i++)
			_iposv_54x1[i] = n2+i;
		nl1 = n2;

		for(i=1;i<=m2;i++)
			_l3_54x1[i] = 1;

		for (i=1;i<=(n2+1);i++) 
		{
			q1=0.f;
			for (j=m1+1;j<=m_;j++) 
				q1 += _A_56x56[j+1][i];
			_A_56x56[m_+2][i] = -q1;
		}

		for (;;) 
		{
			simp1(_A_56x56, m_+1, _l1_55x1, nl1, 0, &kp, &bmax);
			if (bmax <= EPS && _A_56x56[m_+2][1] < -EPS)
			{
				printf("\n simplex phase 0 problem, 1");
				printf("\n m %d, n %d, m_ %d, nl1 %d",m,n,m_,nl1);
				for(i=1;i<=48;i++)
				{
					printf("\n");
					for(j=1;j<=35;j++)
						printf("%f ",_A_56x56[i][j]);
				}
				printf("\n _l1_55x1\n");
				for(i=1;i<=35;i++)
				{
					printf("%f ",_l1_55x1[i]);
				}
				return (-1);   // no solutions satisfy constraints given
			}
			else if (bmax <= EPS && _A_56x56[m_+2][1] <= EPS) 
			{
				for (ip=m1+m2+1; ip<=m_; ip++) 
				{
					if (_iposv_54x1[ip] == (ip+n2)) 
					{
						simp1(_A_56x56, ip, _l1_55x1, nl1, 1, &kp, &bmax);
						if (bmax > EPS)
							goto one;
					}
				}
				for (i=m1+1; i<=m1+m2; i++)
				{
					if (_l3_54x1[i-m1] == 1)
					{
						for (k=1; k<=n2+1; k++)
							_A_56x56[i+1][k] = -_A_56x56[i+1][k];
					}
				}
				break;
			}
			simp2(_A_56x56, m_, n2, &ip, kp);
			if (ip == 0) 
			{
				printf("\n simplex phase 0 problem, 2");
				return (-1);		// no solutions satisfy constraints given
			}

	one:	simp3(_A_56x56, m_+1, n2, ip, kp);
			if (_iposv_54x1[ip] >= (n2+m1+m2+1)) 
			{
				for (k=1; k<=nl1; k++)
					if (_l1_55x1[k] == kp) 
						break;
				--nl1;
				for (is=k; is<=nl1; is++) 
					_l1_55x1[is] = _l1_55x1[is+1];
			} 
			else 
			{
				kh = _iposv_54x1[ip]-m1-n2;
				if (kh >= 1 && _l3_54x1[kh]) 
				{
					_l3_54x1[kh]=0;
					_A_56x56[m_+2][kp+1]++;
					for (i=1; i<=m_+2; i++)
						_A_56x56[i][kp+1] = -_A_56x56[i][kp+1];
				}
			}
			is = _izrov_54x1[kp];
			_izrov_54x1[kp] = _iposv_54x1[ip];
			_iposv_54x1[ip] = is;
		}

		for(i=1;i<=m_;i++)
		{
			_temp_54x1[i] = 0.;
			if(_iposv_54x1[i]<=n2)
				_temp_54x1[_iposv_54x1[i]] = _A_56x56[i+1][1];			
		}

		for(i=1; i<=n; i++)
			_temp2_54x1[i] = _temp_54x1[i] - _temp_54x1[i+n];
	}
	else
	{
		for(i=1;i<=n; i++)
			result_nx1[i] = 0.f;
		return 0;
	}  //---------------------------------- a feasible solution has been obtained

	mult_mv_((const double**)_A0_54x27, m_, n, _temp2_54x1, _Ax_54x1);

	na = 0;  // number of active constraints
	for(i=1; i<=m_; i++)
	{
		_ia_54x1[i] = 0;

		f_temp = (double)fabs(_Ax_54x1[i]-_b0_54x1[i]);
		i_temp = (int)(_logb(_Ax_54x1[i])*0.3f);
		if(i_temp >= 1)
		{
			if ( f_temp < EPS*(float)pow(10.f,(float)i_temp)) 
			{
				na++;
				for(j=1;j<=n;j++)
					_Aa_27x27[na][j] = _A0_54x27[i][j];
				_ba_27x1[na] = _b0_54x1[i];
				_ia_54x1[na] = i;
			}
		}
		else
		{
			if ( f_temp < EPS) 
			{
				na++;
				for(j=1;j<=n;j++)
					_Aa_27x27[na][j] = _A0_54x27[i][j];
				_ba_27x1[na] = _b0_54x1[i];
				_ia_54x1[na] = i;
			}
		}
	}

	for(i=1; i<=n_opt; i++)
		_grad_27x1[i] = _temp2_54x1[i];
	for(i=n_opt; i<=n; i++)
		_grad_27x1[i] = 0.;

	if (na == 0)
		subs_sv_(-1.f, _grad_27x1,n, _d_27x1);
	else
	{
		if(na == m3)
		{
			for(i=1; i<=n; i++)
				result_nx1[i] = (float)_temp2_54x1[i];
			return 0;
		}

		trans_(1.f, (const double**)_Aa_27x27, na, n, _nrTEMP1_34x34);
		mult_mm_((const double**)_Aa_27x27, na, n, (const double**)_nrTEMP1_34x34,na, _nrTEMP2_34x34);
		inv_(_nrTEMP2_34x34,na);
		mult_mm_((const double**)_nrTEMP1_34x34,n,na,(const double**)_nrTEMP2_34x34,na, _nrTEMP3_34x34);
		mult_mm_((const double**)_nrTEMP3_34x34,n,na, (const double**)_Aa_27x27,n, _nrTEMP1_34x34);
		diff_mm_((const double**)_EYE_34,n,n, (const double**)_nrTEMP1_34x34, _N_27x27);       // null space projection matrix N
		mult_smv_(-1., (const double**)_N_27x27, n,n, _grad_27x1, _d_27x1);  // feasible direction d
	}
	
	for(k=1; k<20; k++)  // if k>=20, no convergent solution
	{
		while(2)
		{
			alpha = 1.; // basic step length
	        if(norm_v_(_d_27x1,n) > EPS*3.f)
			{
				mult_mv_((const double**)_A0_54x27,m_,n,_d_27x1, _temp_54x1);
				j=1;
				for(i=1; i<=m_-m3; i++)
				{
					if(i != _ia_54x1[j])
					{
						if((_temp_54x1[i] > 0.) && (_b0_54x1[i]>_Ax_54x1[i]))
							alpha = my_min(alpha, (_b0_54x1[i]-_Ax_54x1[i])/_temp_54x1[i]);											
					}
					else
						j++;
                }
				subs_sv_(1., _temp2_54x1,n,_temp_54x1);
				sum_svsv_(1.f,_temp_54x1,n, alpha,_d_27x1, _temp2_54x1);            
				mult_mv_((const double**)_A0_54x27, m_, n, _temp2_54x1, _Ax_54x1);
				
				na=0;
				for(i=1; i<=m_; i++)
				{
					_ia_54x1[i] = 0;
					f_temp = (double)fabs(_Ax_54x1[i]-_b0_54x1[i]);
					i_temp = (int)(_logb(_Ax_54x1[i])*0.3f);
					if(i_temp >= 1)
					{
						if (f_temp < EPS*(float)pow(10.f, (float)i_temp)) 
						{
							na++;
							for(j=1;j<=n;j++)
								_Aa_27x27[na][j] = _A0_54x27[i][j];
							_ba_27x1[na] = _b0_54x1[i];
							_ia_54x1[na] = i;
						}
					}
					else
					{
						if (f_temp < EPS) 
						{
							na++;
							for(j=1;j<=n;j++)
								_Aa_27x27[na][j] = _A0_54x27[i][j];
							_ba_27x1[na] = _b0_54x1[i];
							_ia_54x1[na] = i;
						}
					}
				}
				
				for(i=1; i<=n_opt; i++)
					_grad_27x1[i] = _temp2_54x1[i];
				for(i=1+n_opt; i<=n; i++)
					_grad_27x1[i] = 0.;
				
				if(na==0)
					subs_sv_(-1.f, _grad_27x1,n, _d_27x1);					
				else
				{
					trans_(1.f, (const double**)_Aa_27x27, na, n, _nrTEMP1_34x34);
					mult_mm_((const double**)_Aa_27x27, na, n, (const double**)_nrTEMP1_34x34,na, _nrTEMP2_34x34);
					inv_(_nrTEMP2_34x34,na);
					mult_mm_((const double**)_nrTEMP1_34x34,n,na,(const double**)_nrTEMP2_34x34,na, _nrTEMP3_34x34);
					mult_mm_((const double**)_nrTEMP3_34x34,n,na, (const double**)_Aa_27x27,n, _nrTEMP1_34x34);
					diff_mm_((const double**)_EYE_34,n,n, (const double**)_nrTEMP1_34x34, _N_27x27);       // null space projection matrix N
					mult_smv_(-1.f, (const double**)_N_27x27, n,n, _grad_27x1, _d_27x1);  // feasible direction d
				}
			}
			else
				break;
        } // while(2)

		if(na > m3)
		{
			leaving = 0;
			trans_(1., (const double**)_Aa_27x27, na, n, _nrTEMP1_34x34);
			mult_mm_((const double**)_Aa_27x27, na, n, (const double**)_nrTEMP1_34x34,na, _nrTEMP2_34x34);
			inv_(_nrTEMP2_34x34,na);
			mult_mm_((const double**)_nrTEMP2_34x34,na,na, (const double**)_Aa_27x27,n, _nrTEMP3_34x34);
			mult_smv_(-1.f, (const double**)_nrTEMP3_34x34,na,n,_grad_27x1, _lambda_27x1);  // Lagrange multipliers
			
			for(i=1; i<=na-m3; i++)
			{
				if(_lambda_27x1[i] < -EPS*3.f)
				{
					if(leaving == 0)
						leaving = i;
					else if(_lambda_27x1[i] < _lambda_27x1[leaving])
						leaving = i;
				}           
            }   
			
			if(leaving == 0)
			{
				for(i=1; i<=n; i++)
					result_nx1[i] = (float)_temp2_54x1[i];
				return 0;  // optimal solution, _temp2_54x1
			}
			else
			{
				for(i=leaving; i<=na-1; i++)
				{
					for(j=1; j<=n; j++)
					{
						_Aa_27x27[i][j] = _Aa_27x27[i+1][j];
					}
					_ba_27x1[i] = _ba_27x1[i+1];
					_ia_54x1[i] = _ia_54x1[i+1];					
				}
				na--;
				for(i=1; i<=n_opt; i++)
					_grad_27x1[i] = _temp2_54x1[i];
				
				trans_(1.f, (const double**)_Aa_27x27, na, n, _nrTEMP1_34x34);
				mult_mm_((const double**)_Aa_27x27, na, n, (const double**)_nrTEMP1_34x34,na, _nrTEMP2_34x34);
				inv_(_nrTEMP2_34x34,na);
				mult_mm_((const double**)_nrTEMP1_34x34,n,na, (const double**)_nrTEMP2_34x34,na, _nrTEMP3_34x34);
				mult_mm_((const double**)_nrTEMP3_34x34,n,na, (const double**)_Aa_27x27,n, _nrTEMP1_34x34);
				diff_mm_((const double**)_EYE_34,n,n, (const double**)_nrTEMP1_34x34, _N_27x27);       // null space projection matrix N
				mult_smv_(-1.f, (const double**)_N_27x27, n,n, _grad_27x1, _d_27x1);  // feasible direction d
			}
		}
		else
		{
			for(i=1; i<=n; i++)
					result_nx1[i] = (float)_temp2_54x1[i];
			return 0;
		}
	}
	return -2; // feasible solution but not optimal
}
#undef my_min

//---------------------------------------

int my_round(float x)
{
	return ((x>=0.) ? (int)(x+0.5f) : (int)(x-0.5f));
}



float bounding(float input, float bu, float bl)
{
	if(input > bu)
		return bu;
	else if(input < bl)
		return bl;
	else
		return input;
}