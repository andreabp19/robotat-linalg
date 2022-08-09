/**
 * @file matf32_check.c
 */

#include <stdint.h>
#include <stdbool.h>

#include "matf32/matf32_check.h"


// ====================================================================================================
// Matrix datatype-based checks
// ====================================================================================================


bool
matf32_check_triangular_upper(const matf32_t* const p_mat)
{
#ifdef MATH_MATRIX_CHECK
    if (p_mat->num_rows != p_mat->num_cols)
    {
        return false;
    }
#endif

    float* p_data_src = p_mat->p_data;

    for (uint16_t i = 1; i < p_mat->num_rows; ++i)
    {
        for (uint16_t j = 0; j < i; ++j)
        {
            if (0 != p_data_src[i*p_mat->num_rows + j])
            {
                return false;
            }
        }
    }

    return true;
}

bool
matf32_check_triangular_lower(const matf32_t* const p_mat)
{
#ifdef MATH_MATRIX_CHECK
    if (p_mat->num_rows != p_mat->num_cols)
    {
        return false;
    }
#endif

    float* p_data_src = p_mat->p_data;

    for (uint16_t i = 0; i < p_mat->num_rows-1; ++i)
    {
        for (uint16_t j = p_mat->num_cols-1; j > i; --j)
        {
            if (0 != p_data_src[i*p_mat->num_rows + j])
            {
                return false;
            }
        }
    }

    return true;
}

bool
matf32_is_equal(const matf32_t* const p_mat_a, const matf32_t* const p_mat_b)
{
#ifdef MATH_MATRIX_CHECK
    if ((p_mat_a->num_rows != p_mat_b->num_rows) || (p_mat_a->num_cols != p_mat_b->num_cols))
    {
        return false;
    }
#endif
    return is_equal(p_mat_a->p_data, p_mat_b->p_data, p_mat_a->num_rows * p_mat_a->num_cols);
}


bool
matf32_check_symmetric(const matf32_t* const p_mat)
{
#ifdef MATH_MATRIX_CHECK
    if (matf32_check_square_matrix(p_mat))
    {
        return false;
    }
#endif

    float* p_data_source = p_mat->p_data;

    uint16_t size = p_mat->num_cols*p_mat->num_rows -1;

    for (uint16_t i = 0; i < size; ++i)
    {
        // change != with a comparation with error (abs(a-b) < err)
        if (p_data_source[i] != p_data_source[size-i])
        {
            return false;
        }
    }

    return true;
}


bool
matf32_check_hessenberg_upper(const matf32_t* const p_mat)
{
#ifdef MATH_MATRIX_CHECK
    if (p_mat->num_rows != p_mat->num_cols)
    {
        return false;
    }
#endif

    float* p_data_src = p_mat->p_data;

    for (uint16_t i = 2; i < p_mat->num_rows; ++i)
    {
        for (uint16_t j = 0; j < i-1; ++j)
        {
            if (0 != p_data_src[i*p_mat->num_rows + j])
            {
                return false;
            }
        }
    }

    return true;
}


bool
matf32_check_hessenberg_lower(const matf32_t* const p_mat)
{
#ifdef MATH_MATRIX_CHECK
    if (p_mat->num_rows != p_mat->num_cols)
    {
        return false;
    }
#endif

    float* p_data_src = p_mat->p_data;

    for (uint16_t i = 0; i < p_mat->num_rows-2; ++i)
    {
        for (uint16_t j = p_mat->num_cols-1; j > i+1; --j)
        {
            if (0 != p_data_src[i*p_mat->num_rows + j])
            {
                //return false;
            }
        }
    }

    return true;
}