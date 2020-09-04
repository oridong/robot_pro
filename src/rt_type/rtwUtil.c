#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "tmwtypes.h"
# include "rt_nonfinite.h"
# include "rt_defines.h"
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
double rt_atan2d_snf(double u0, double u1)
{
    double y;
    int b_u0;
    int b_u1;
    if (rtIsNaN(u0) || rtIsNaN(u1))
    {
        y = rtNaN;
    }
    else if (rtIsInf(u0) && rtIsInf(u1))
    {
        if (u0 > 0.0)
        {
            b_u0 = 1;
        }
        else
        {
            b_u0 = -1;
        }

        if (u1 > 0.0)
        {
            b_u1 = 1;
        }
        else
        {
            b_u1 = -1;
        }

        y = atan2(b_u0, b_u1);
    }
    else if (u1 == 0.0)
    {
        if (u0 > 0.0)
        {
            y = RT_PI / 2.0;
        }
        else if (u0 < 0.0)
        {
            y = -(RT_PI / 2.0);
        }
        else
        {
            y = 0.0;
        }
    }
    else
    {
        y = atan2(u0, u1);
    }

    return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
double rt_powd_snf(double u0, double u1)
{
    double y;
    double d7;
    double d8;
    if (rtIsNaN(u0) || rtIsNaN(u1))
    {
        y = rtNaN;
    }
    else
    {
        d7 = fabs(u0);
        d8 = fabs(u1);
        if (rtIsInf(u1))
        {
            if (d7 == 1.0)
            {
                y = 1.0;
            }
            else if (d7 > 1.0)
            {
                if (u1 > 0.0)
                {
                    y = rtInf;
                }
                else
                {
                    y = 0.0;
                }
            }
            else if (u1 > 0.0)
            {
                y = 0.0;
            }
            else
            {
                y = rtInf;
            }
        }
        else if (d8 == 0.0)
        {
            y = 1.0;
        }
        else if (d8 == 1.0)
        {
            if (u1 > 0.0)
            {
                y = u0;
            }
            else
            {
                y = 1.0 / u0;
            }
        }
        else if (u1 == 2.0)
        {
            y = u0 * u0;
        }
        else if ((u1 == 0.5) && (u0 >= 0.0))
        {
            y = sqrt(u0);
        }
        else if ((u0 < 0.0) && (u1 > floor(u1)))
        {
            y = rtNaN;
        }
        else
        {
            y = pow(u0, u1);
        }
    }

    return y;
}