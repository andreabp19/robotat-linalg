/**
 * @file robotat_linalg.c
 */
#include "matf32_def.h"


void
matf32_init(matf32_t* const instance, uint16_t num_rows, uint16_t num_cols, float* p_data)
{
    instance->num_rows = num_rows;
    instance->num_cols = num_cols;
    instance->p_data = p_data;
}


void
matf32_print(const matf32_t* p_src)
{
    float* p_data_src = p_src->p_data;

    for (uint16_t i = 0; i < p_src->num_rows; i++)
    {
        for (uint16_t j = 0; j < p_src->num_cols; j++)
        {
            printf("%0.18f\t", *(p_data_src++));
        }
        printf("\n");
    }
    printf("\n");
}


void
err_status_print(err_status_t err)
{
    switch(err)
    {
        case MATH_SUCCESS:
            printf("MATH_SUCCESS\n");
            break;
            
        case MATH_ARGUMENT_ERROR:
            printf("MATH_ARGUMENT_ERROR\n");
            break;
            
        case MATH_LENGTH_ERROR:
            printf("MATH_LENGTH_ERROR\n");
            break;
            
        case MATH_SIZE_MISMATCH:
            printf("MATH_SIZE_MISMATCH\n");
            break;
            
        case MATH_NANINF:
            printf("MATH_NANINF\n");
            break;
            
        case MATH_SINGULAR:
            printf("MATH_SINGULAR\n");
            break;
            
        case MATH_TEST_FAILURE:
            printf("MATH_TEST_FAILURE\n");
            break;
            
        case MATH_DECOMPOSITION_FAILURE:
            printf("MATH_DECOMPOSITION_FAILURE\n");
            break;
            
    }
}

// ====================================================================================================
// Sub-matrix operattions
// ====================================================================================================

err_status_t
matf32_submatrix_copy(const matf32_t* const p_src, matf32_t* const p_dst,
                      const uint16_t src_row, const uint16_t src_col,
                      const uint16_t dst_row, const uint16_t dst_col,
                      const uint16_t rows,    const uint16_t cols)
{
#ifdef MATH_MATRIX_CHECK
    if (    (p_src->num_rows < (src_row+rows) )
         || (p_src->num_cols < (src_col+cols) )
         || (p_dst->num_rows < (dst_row+rows) )
         || (p_dst->num_cols < (dst_col+cols) ))
    {
        return MATH_SIZE_MISMATCH;
    }
#endif

    // A(i,j)
    for (uint16_t i = 0; i < rows; ++i)
    {
        for (uint16_t j = 0; j < cols; ++j)
        {
            p_dst->p_data[(dst_row+i)*p_dst->num_cols + (dst_col+j)] = p_src->p_data[(src_row+i)*p_src->num_cols + (src_col+j)];
        }
    }

    return MATH_SUCCESS;
}


// ====================================================================================================
// Special matrix initializations
// ====================================================================================================


void
matf32_eye(matf32_t* const p_dst)
{
    eye(p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_diag(float* p_src, matf32_t* const p_dst)
{
    diag(p_src, p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_zeros(matf32_t* const p_dst)
{
    zeros(p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_ones(matf32_t* const p_dst)
{
    ones(p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_randn(matf32_t* const p_dst, float mu, float sigma)
{
    randn(p_dst->p_data, p_dst->num_rows * p_dst->num_cols, mu, sigma);
}
