/**
 * @file math_util.c
 */

#include "math_util.h"


// ====================================================================================================
// Private function definitions/implementations
// ====================================================================================================

static float 
generate_gauss(float mu, float sigma) 
{
    float U1, U2, W, scalar;
    static float X1, X2;
    static int call = 0;

    if (call == 1) 
    {
        call = !call;
        return (mu + sigma * (float)X2);
    }

    // Compute the uniform norm
    do 
    {
        U1 = -1 + ((float)rand() / RAND_MAX) * 2;
        U2 = -1 + ((float)rand() / RAND_MAX) * 2;
        W = pow(U1, 2) + pow(U2, 2);
    } while (W >= 1 || W == 0);

    scalar = sqrt((-2 * log(W)) / W);
    X1 = U1 * scalar;
    X2 = U2 * scalar;

    call = !call;

    return (mu + sigma * (float)X1);
}

// ====================================================================================================
// Linear algebra routines that do not depend on the matrix datatype
// ====================================================================================================
float
dot(float* p_srca, float* p_srcb, uint16_t length)
{
    float sum = 0;  // Reset;

    // Multiply each row
    for (int i = 0; i < length; ++i)
        sum += (*(p_srca++)) * (*(p_srcb++));
    return sum;
}


void
eye(float* p_dst, uint16_t row, uint16_t column)
{
    // Reset first
    memset(p_dst, 0, row * column * sizeof(float));

    for (int i = 0; i < row; i++)
    {
        *p_dst = 1.0;
        p_dst += row + 1;
    }
}


void
diag(float* p_src, float* p_dst, int row_d, int column_d)
{
    // Reset the matrix array
    memset(p_dst, 0, row_d * column_d * sizeof(float));

    for (int i = 0; i < row_d; i++) {
        for (int j = 0; j < column_d; j++) {
            if (j == i) {
                *p_dst = p_src[i];
                p_dst += column_d + 1;
            }
        }
    }
}


void
zeros(float* p_dst, int row, int column)
{
    memset(p_dst, 0, row * column * sizeof(float));
}


void
ones(float* p_dst, int row, int column)
{
    memset(p_dst, 1, row * column * sizeof(float));
}


void
randn(float* p_dst, uint16_t length, float mu, float sigma)
{
    srand(time(NULL));
    for (uint16_t i = 0; i < length; i++)
        p_dst[i] = generate_gauss(mu, sigma);
}

float
norm(float* p_src, int row, int column)
{
    uint16_t size = row*column;

    float sum = 0;

    for (uint16_t i = 0; i < size; ++i)
    {
        sum += p_src[i]*p_src[i];
    }

    return sqrtf(sum);
}

void
scale(float* p_src, uint16_t length, float scalar, float* p_dst)
{
    for (int i = 0; i < length; ++i)
    {
        p_dst[i] = p_src[i]*scalar;
    }
}

// ====================================================================================================
// Miscellaneous
// ====================================================================================================
void
copy(float* p_src, float* p_dst, int row, int column)
{
    memcpy(p_dst, p_src, column * row * sizeof(float));
}

void 
print(float* p_src, uint16_t row, uint16_t column)
{
    for (uint16_t i = 0; i < row; i++) 
    {
        for (uint16_t j = 0; j < column; j++)
            printf("%0.18f\t", *(p_src++));
        printf("\n");
    }
    printf("\n");
}


float 
saturation(float input, float lower_limit, float upper_limit) 
{
    if (input > upper_limit)
        return upper_limit;
    else if (input < lower_limit)
        return lower_limit;
    else 
        return input; // No action
}


float 
sign(float number) 
{
    if (number > 0) 
        return 1;
    else if (number < 0) 
        return -1;
    else 
        return 0;
}


float
mean(float* p_src, uint16_t length)
{
    float s = 0;

    for (uint16_t i = 0; i < length; i++)
        s += p_src[i];
    return s / ((float)length);
}


float
std_dev(float* p_src, uint16_t length)
{
    float mu = mean(p_src, length);
    float sigma = 0;
    
    for (uint16_t i = 0; i < length; i++)
        sigma += (p_src[i] - mu) * (p_src[i] - mu);
    return sqrtf(sigma / ((float)length));
}

bool
is_equal(float* p_a, float* p_b, uint16_t length)
{
    for (uint16_t i = 0; i < length; ++i)
    {
        // skip if both are NaN
        if (isnan(p_a[i]) && isnan(p_b[i]))
        {
            continue;
        }

        if (!is_equal_margin(p_a[i], p_b[i]))
        {
            return false;
        }
    }

    return true;
}

void
zero_patch(float* p_a, uint16_t length)
{
    for (uint16_t i = 0; i < length; ++i)
    {
        // skip if both are NaN
        if (isnan(p_a[i]) || isinf(p_a[i]))
        {
            p_a[i] = 0.0;
        }
    }
}
