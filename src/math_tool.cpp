# include <stddef.h>
# include <stdlib.h>
# include <string.h>
# include <math.h>
# include <stdio.h>

double max(double *x, int size)
{
    int i;
    double max = x[0];
    for (i = 0; i< size; i++)
    {
        if (x[i] > max)
        {
            max = x[i];
        }
    }
    return max;
}

void cross(const double b_a[3], const double b_b[3], double b_c[3])
{
    b_c[0] = b_a[1] * b_b[2] - b_a[2] * b_b[1];
    b_c[1] = b_a[2] * b_b[0] - b_a[0] * b_b[2];
    b_c[2] = b_a[0] * b_b[1] - b_a[1] * b_b[0];
}

double dot(const double b_a[3], const double b_b[3])
{
    return (b_a[0] * b_b[0] + b_a[1] * b_b[1]) + b_a[2] * b_b[2];
}

double b_norm(const double x[3])
{
    double y;
    double scale;
    double absxk;
    double t;
    scale = 3.3121686421112381E-170;
    absxk = fabs(x[0]);
    if (absxk > scale)
    {
        y = 1.0;
        scale = absxk;
    }
    else
    {
        t = absxk / scale;
        y = t * t;
    }

    absxk = fabs(x[1]);
    if (absxk > scale)
    {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
    }
    else
    {
        t = absxk / scale;
        y += t * t;
    }

    absxk = fabs(x[2]);
    if (absxk > scale)
    {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
    }
    else
    {
        t = absxk / scale;
        y += t * t;
    }

    return scale * sqrt(y);
}

double norm(double * vec, int num)
{
    int i;
    double sum;
    sum = 0.0;
    for (i = 0; i < num; i++)
    {
        sum += vec[i] * vec[i];
    }
    sum = sqrt(sum);
    return sum;
}

void b_sign(double *x)
{
    if (*x < 0.0)
    {
        *x = -1.0;
    }
    else if (*x > 0.0)
    {
        *x = 1.0;
    }
    else
    {
        if (*x == 0.0)
        {
            *x = 0.0;
        }
    }
}

double sum(const double x_data[])
{
    return (x_data[0] + x_data[1]) + x_data[2];
}

void b_sqrt(double *x)
{
    *x = sqrt(*x);
}

// ���?
void mldivide(const double A[16], double B[16])
{
    double b_A[16];
    signed char ipiv[4];
    int j;
    int b_b;
    int jj;
    int iy;
    int jp1j;
    int n;
    int jA;
    double smax;
    int ix;
    int i28;
    double s;
    int ijA;
    memcpy(&b_A[0], &A[0], sizeof(double) << 4);
    ipiv[0] = 1;
    ipiv[1] = 2;
    ipiv[2] = 3;
    ipiv[3] = 4;
    for (j = 0; j < 3; j++)
    {
        b_b = j * 5;
        jj = j * 5;
        jp1j = b_b + 2;
        n = 4 - j;
        iy = 0;
        ix = b_b;
        smax = fabs(b_A[b_b]);
        for (jA = 2; jA <= n; jA++)
        {
            ix++;
            s = fabs(b_A[ix]);
            if (s > smax)
            {
                iy = jA - 1;
                smax = s;
            }
        }

        if (b_A[jj + iy] != 0.0)
        {
            if (iy != 0)
            {
                iy += j;
                ipiv[j] = (signed char)(iy + 1);
                smax = b_A[j];
                b_A[j] = b_A[iy];
                b_A[iy] = smax;
                ix = j + 4;
                iy += 4;
                smax = b_A[ix];
                b_A[ix] = b_A[iy];
                b_A[iy] = smax;
                ix += 4;
                iy += 4;
                smax = b_A[ix];
                b_A[ix] = b_A[iy];
                b_A[iy] = smax;
                ix += 4;
                iy += 4;
                smax = b_A[ix];
                b_A[ix] = b_A[iy];
                b_A[iy] = smax;
            }

            i28 = jj - j;
            for (jA = jp1j; jA <= i28 + 4; jA++)
            {
                b_A[jA - 1] /= b_A[jj];
            }
        }

        n = 2 - j;
        iy = b_b + 4;
        jA = jj + 5;
        for (b_b = 0; b_b <= n; b_b++)
        {
            smax = b_A[iy];
            if (b_A[iy] != 0.0)
            {
                ix = jj + 1;
                i28 = jA + 1;
                jp1j = (jA - j) + 3;
                for (ijA = i28; ijA <= jp1j; ijA++)
                {
                    b_A[ijA - 1] += b_A[ix] * -smax;
                    ix++;
                }
            }

            iy += 4;
            jA += 4;
        }

        if (ipiv[j] != j + 1)
        {
            iy = ipiv[j] - 1;
            smax = B[j];
            B[j] = B[iy];
            B[iy] = smax;
            smax = B[j + 4];
            B[j + 4] = B[iy + 4];
            B[iy + 4] = smax;
            smax = B[j + 8];
            B[j + 8] = B[iy + 8];
            B[iy + 8] = smax;
            smax = B[j + 12];
            B[j + 12] = B[iy + 12];
            B[iy + 12] = smax;
        }
    }

    for (j = 0; j < 4; j++)
    {
        iy = j << 2;
        if (B[iy] != 0.0)
        {
            for (jA = 2; jA < 5; jA++)
            {
                i28 = (jA + iy) - 1;
                B[i28] -= B[iy] * b_A[jA - 1];
            }
        }

        if (B[1 + iy] != 0.0)
        {
            for (jA = 3; jA < 5; jA++)
            {
                B[(jA + iy) - 1] -= B[1 + iy] * b_A[jA + 3];
            }
        }

        if (B[2 + iy] != 0.0)
        {
            for (jA = 4; jA < 5; jA++)
            {
                B[iy + 3] -= B[2 + iy] * b_A[11];
            }
        }
    }

    for (j = 0; j < 4; j++)
    {
        iy = j << 2;
        smax = B[3 + iy];
        if (smax != 0.0)
        {
            B[3 + iy] = smax / b_A[15];
            for (jA = 0; jA < 3; jA++)
            {
                i28 = jA + iy;
                B[i28] -= B[3 + iy] * b_A[jA + 12];
            }
        }

        smax = B[2 + iy];
        if (smax != 0.0)
        {
            B[2 + iy] = smax / b_A[10];
            for (jA = 0; jA < 2; jA++)
            {
                B[jA + iy] -= B[2 + iy] * b_A[jA + 8];
            }
        }

        smax = B[1 + iy];
        if (smax != 0.0)
        {
            B[1 + iy] = smax / b_A[5];
            for (jA = 0; jA < 1; jA++)
            {
                B[iy] -= B[1 + iy] * b_A[4];
            }
        }

        if (B[iy] != 0.0)
        {
            B[iy] /= b_A[0];
        }
    }
}


void mldivide_rot(const double A[9], const double B[9], double Y[9])
{
  double b_A[9];
  int r1;
  int r2;
  int r3;
  double maxval;
  double a21;
  int rtemp;
  double d0;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  memcpy(&b_A[0], &A[0], 9U * sizeof(double));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(A[0]);
  a21 = fabs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[3 + r2] -= b_A[r2] * b_A[3 + r1];
  b_A[3 + r3] -= b_A[r3] * b_A[3 + r1];
  b_A[6 + r2] -= b_A[r2] * b_A[6 + r1];
  b_A[6 + r3] -= b_A[r3] * b_A[6 + r1];
  if (fabs(b_A[3 + r3]) > fabs(b_A[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[3 + r3] /= b_A[3 + r2];
  b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + r2];
  maxval = B[r2] - B[r1] * b_A[r2];
  a21 = b_A[3 + r3];
  d0 = b_A[6 + r3];
  d1 = ((B[r3] - B[r1] * b_A[r3]) - maxval * a21) / d0;
  Y[2] = d1;
  d2 = b_A[6 + r1];
  d3 = b_A[6 + r2];
  maxval -= d1 * d3;
  d4 = b_A[3 + r2];
  maxval /= d4;
  Y[1] = maxval;
  d5 = b_A[3 + r1];
  Y[0] = ((B[r1] - d1 * d2) - maxval * d5) / b_A[r1];
  d6 = B[r1 + 3];
  maxval = B[r2 + 3] - d6 * b_A[r2];
  d1 = ((B[r3 + 3] - d6 * b_A[r3]) - maxval * a21) / d0;
  Y[5] = d1;
  d6 -= d1 * d2;
  maxval -= d1 * d3;
  maxval /= d4;
  Y[4] = maxval;
  d6 -= maxval * d5;
  d6 /= b_A[r1];
  Y[3] = d6;
  d6 = B[r1 + 6];
  maxval = B[r2 + 6] - d6 * b_A[r2];
  d1 = ((B[r3 + 6] - d6 * b_A[r3]) - maxval * a21) / d0;
  Y[8] = d1;
  d6 -= d1 * d2;
  maxval -= d1 * d3;
  maxval /= d4;
  Y[7] = maxval;
  d6 -= maxval * d5;
  d6 /= b_A[r1];
  Y[6] = d6;
}

/*
*   ����----------����˷���matlab������ʽ����
*   ����----------left double���� �����?
                  xl��yl���������������?
                  right double���� �Ҿ���
                  xr��yr �Ҿ�������������
*   ���?----------result ����������ڷ����?ָ�룬����ߴ����?��ǰȷ��
*/
void matrixMultiply(double *left, int xl, int yl, double *right, int xr, int yr, double *result)
{
    int i = 0;
    int j = 0;
    int k = 0;
    int T01_tmp = 0;

    if (yl != xr)
    {
        printf("MATH ERROR: matrix mutiply faild!\n");
        return;
    }

    for (i = 0; i < xl; i++)
    {
        for (j = 0; j < yr; j++)
        {
            result[i + j * xl] = 0.0;
            for (k = 0; k < yl; k++)
            {
                result[i + j * xl] += *(left + i + k * xl) * (*(right + j * xr + k));
            }
        }
    }
}

void slerp(float starting[4], float ending[4], float result[4], float t )
{
    float cosa = starting[0]*ending[0] + starting[1]*ending[1] + starting[2]*ending[2] + starting[3]*ending[3];
    
    // If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
    // the shorter path. Fix by reversing one quaternion.
    if ( cosa < 0.0f ) 
    {
        ending[0] = -ending[0];
        ending[1] = -ending[1];
        ending[2] = -ending[2];
        ending[3] = -ending[3];
        cosa = -cosa;
    }
    
    float k0, k1;
    
    // If the inputs are too close for comfort, linearly interpolate
    if ( cosa > 0.9995f ) 
    {
        k0 = 1.0f - t;
        k1 = t;
    }
    else 
    {
        float sina = sqrt( 1.0f - cosa*cosa );
        float a = atan2( sina, cosa );
        k0 = sin((1.0f - t)*a)  / sina;
        k1 = sin(t*a) / sina;
    }
    result[0] = starting[0]*k0 + ending[0]*k1;
    result[1] = starting[1]*k0 + ending[1]*k1;
    result[2] = starting[2]*k0 + ending[2]*k1;
    result[3] = starting[3]*k0 + ending[3]*k1;
}

void printf_d(double * data, int size)
{
    int i;
    for (i = 0; i < size; i ++)
    {
        printf(",%f" + !i, data[i]);
    }
    printf("\n");
}

void quatU2Axis(const double qin[4], double qout[4])
{
    double theta = 2.0*acos(qin[0]);
    double std2 = sin(theta/2);
    if (std2 == 0.0)
    {
        qout[0] = 0.0;
        qout[1] = 0.0;
        qout[2] = 0.0;
        qout[3] = 0.0;
    }else
    {
        qout[0] = theta;
        qout[1] = qin[1] / std2;
        qout[2] = qin[2] / std2;
        qout[3] = qin[3] / std2;
    }
    
}

void matrixTrans(double * in, int x , int y, double * out)
{
    int i,j;
    for (i = 0; i<x; i++)
    {
        for (j = 0; j< y; j++)
        {
            out[i*y+j] = in[j*x +i];
        }
    }
}

void schmdit(double in[9], double out[9])
{
    int i,j;
    double beta1[3];
    double beta2[3];
    double beta3[3];

    // 正交化
    for (i = 0; i<3; i++)
    {
        beta1[i] = in[i];
    }
    double d1 = dot(*(double(*)[3]) &in[3] ,beta1)/dot(beta1, beta1);
    for (i = 0; i<3; i++)
    {
        beta2[i] = in[i + 3] - d1* beta1[i];
    }

    d1 = dot(*(double(*)[3]) &in[6] ,beta1)/dot(beta1, beta1);
    double d2 = dot(*(double(*)[3]) &in[6] , beta2)/dot(beta2, beta2);
    for (i = 0; i<3; i++)
    {
        beta3[i] = in[i + 6] - d1 * beta1[i] - d2 * beta2[i];
    }

    // 单位化
    for ( i = 0; i<3; i++)
    {
        out[i] = beta1[i] / b_norm(beta1);
    }
    for ( i = 0; i<3; i++)
    {
        out[i + 3] = beta2[i] / b_norm(beta2);
    }
    for ( i = 0; i<3; i++)
    {
        out[i + 6] = beta3[i] / b_norm(beta3);
    }

}

int matrixInverse(double* pSourceR, int iNum, double* pDestR)
{
	int i, j, k, i0, j0;
	double dMax, dTem;

	int* iRow = (int*)malloc(sizeof(int) * iNum);
	int* iCol = (int*)malloc(sizeof(int) * iNum);
	double* dTem1 = (double*)malloc(sizeof(double) * iNum);
	double* dTem2 = (double*)malloc(sizeof(double) * iNum);

    matrixTrans(pSourceR, iNum, iNum, pDestR);

	// 复制数组, 在缓冲区之间拷贝字符
	// memcpy(pDestR, pSourceR, (iNum * iNum) * sizeof(double));

	for (k = 0; k < iNum; k++)
	{
		dMax = 0.0;

		// 首先，全选主元；从第k行k列开始的右下角子阵中选取绝对值最大的元素
		for (i = k; i < iNum; i++)
		{
			for (j = k; j < iNum; j++)
			{
				if (fabs(pDestR[i*iNum+j]) > fabs(dMax))
				{
					dMax = pDestR[i*iNum+j];
					i0 = i;
					j0 = j;
				}
			}
		}

		// 若绝对值最大的元素为零，则为奇异矩阵，求逆失败
		if (fabs(dMax) < 0.000000000001)
		{
			free(iRow);  free(iCol);  free(dTem1);  free(dTem2);
			return 0;
		}

		// 将绝对值最大的元素所在行与k行互换
		if (i0 != k)
		{
			for (j = 0; j < iNum; j++)
			{
				dTem = pDestR[i0*iNum+j];
				pDestR[i0*iNum+j] = pDestR[k*iNum+j];
				pDestR[k*iNum+j] = dTem;
			}
		}

		// 将绝对值最大的元素所在列与k列互换，即可交换到主元素位置上
		if (j0 != k)
		{
			for (i = 0; i < iNum; i++)
			{
				dTem = pDestR[i*iNum+j0];
				pDestR[i*iNum+j0] = pDestR[i*iNum+k];
				pDestR[i*iNum+k] = dTem;
			}
		}

		// 记录该元素所在的行号和列号
		iRow[k] = i0;
		iCol[k] = j0;

		// 然后求解
		for (j = 0; j < iNum; j++)
		{
			if (j == k)
			{
				dTem1[j] = 1 / dMax;
				dTem2[j] = 1.0;
			}
			else
			{
				dTem1[j] =  - pDestR[k*iNum+j] / dMax;
				dTem2[j] = pDestR[j*iNum+k];
			}
			pDestR[k*iNum+j] = 0.0;
			pDestR[j*iNum+k] = 0.0;
		}
		for (i = 0; i < iNum; i++)
			for (j = 0; j < iNum; j++)
				pDestR[i*iNum+j] += dTem2[i] * dTem1[j];
	}

	// 最后，根据在全选主元过程中所记录的行、列交换的信息进行恢复
	for (k = iNum-1; k >= 0; k--)
	{
		i0 = iRow[k];
		j0 = iCol[k];
		if (i0 != k)
		{
			for (i = 0; i < iNum; i++)
			{
				dTem = pDestR[i*iNum+i0];
				pDestR[i*iNum+i0] = pDestR[i*iNum+k];
				pDestR[i*iNum+k] = dTem;
			}
		}
		if (j0 != k)
		{
			for (j = 0; j < iNum; j++)
			{
				dTem = pDestR[j0*iNum+j];
				pDestR[j0*iNum+j] = pDestR[k*iNum+j];
				pDestR[k*iNum+j] = dTem;
			}
		}
	}

    matrixTrans(pDestR, iNum, iNum, pSourceR);
    memcpy( pDestR, pSourceR, sizeof(pDestR));

	free(iRow);  free(iCol);  free(dTem1);  free(dTem2);
	return 1;
}

int CM_Atoi(char* source,int& integer)
{
	int offset1,offset2,num;
	int signedflag;//+为1 -为0
 
	if(source == NULL || *source == 0 ||&integer == NULL)
	{
		return 0;
	}
 
	offset1 = 0;
	offset2 = 0;
	num = 0;
 
	while(*source > 0 && *source <= 32)//去除首部空格 \r \n \t \r 等异常字符
	{
		source++;
		offset1++;
	}
 
	signedflag = 1;//默认为+
	if(*source == '+')
	{
		signedflag = 1;
		source++;
		offset1++;
	}
	else if(*source == '-')
	{
		signedflag = 0;
		source++;
		offset1++;
	}
 
	while(*source != '\0' && *source >= '0' && *source <= '9')
	{
		num = *source- '0' + num*10;
		source++;
		offset2++;
	}
 
	if(signedflag == 0)
	{
		num = -num;
	}
 
	if(offset2)
	{
		integer = num;
		return offset1+offset2;
	}
	else
	{
		return 0;
	}
}

long long pow1(int x,int y)
{
	long long num = 1;
	int i;
 
	for(i = 0; i < y; i++)
	{
		num = num*x;
	}
 
	return num;
}

int CM_Atof(char* source,double& doubleing)
{
	int offset1,offset2,n;
	double num;
	int signedflag;//+为1 -为0
 
	if(source == NULL || *source == 0 || &doubleing == NULL)
	{
		return 0;
	}
 
	offset1 = 0;
	offset2 = 0;
	num = 0.0;
 
	while(*source > 0 && *source <= 32)//去除首部空格 \r \n \t \r 等异常字符
	{
		source++;
		offset1++;
	}
 
	signedflag = 1;//默认为+
	if(*source == '+')
	{
		signedflag = 1;
		source++;
		offset1++;
	}
	else if(*source == '-')
	{
		signedflag = 0;
		source++;
		offset1++;
	}
 
 
	//整数部分
	while(*source != '\0' && *source >= '0' && *source <= '9')
	{
		num = *source- '0' + num*10.0;
		source++;
		offset2++;
	}
 
	if(offset2 != 0 && *source == '.')
	{
		source++;
		offset2++;
 
		//小数部分
		n = 0;
		while(*source != '\0' && *source >= '0' && *source <= '9')
		{
			num = (*source- '0')*(1.0/pow1(10,++n)) + num;
			source++;
			offset2++;
		}
	}
 
	if(signedflag == 0)
	{
		num = -num;
	}
 
	if(offset2)
	{
		doubleing = num;
		return offset1+offset2;
	}
	else
	{
		return 0;
	}
}