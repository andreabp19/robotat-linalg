/**
 * @file robotat_linalg.h
 *
 * Adapted from CControl (https://github.com/DanielMartensson/CControl) with the following changes:
 *
 * 1. Removed all but linear algebra and (some) optimization routines.
 * 2. Changed all variable length arrays for fixed size to increase portability (as VLAs are compiler
 *    dependent extensions since C11 and generally a bad idea in embedded).
 * 3. Merged all function implementations into a single source file, this introduces some clutter
 *    but allows a more manageable memory footprint by defining static, auxiliary, fixed-size float
 *    arrays (originally some function calls like inv needed a huge, impractical stack size when using
 *    both variable-length and fixed-length arrays created inside the routines).
 * 4. Defined a matrix data structure to add code readability, decrease redundancy in constantly passing
 *    matrix dimensions as parameters and add compatibility with ARM's CMSIS DSP libraries (this will
 *    allow us to define wrappers for ARM's HW accelerated routines). This also adds a layer of safety
 *    when doing linear algebra operations. It's even recommended to use single row or column matrices
 *    instead of arrays to gain these dimension checks even though it has a speed penalty (check next point).
 * 5. Added a size mismatch check similar to ARM's CMSIS DSP matrix libraries (this can be disabled to
 *    reduce overhead by undefining the MATH_MATRIX_CHECK macro).
 
 *   Created on: 5 oct. 2019
 *      @author: Daniel Martensson
 *  Modified on: 1 aug. 2021
 *           By: Miguel Zea (mezea@uvg.edu.gt)
 *  Modified on: 20 may 2022
 *           By: Daniel Pineda (bar18714@uv.edu.gt)
 *
 * TODO: Update above description.
 */

#ifndef ROBOTAT_LINALG_H_
#define ROBOTAT_LINALG_H_

 /**
  * Dependencies.
  */

#include <string.h>	                    // For memcpy, memset etc.
#include <stdio.h>                      // For printf.
#include <stdlib.h>                     // Standard library.
#include <stdint.h>	                    // For uint8_t, uint16_t and uint16_t.
#include <math.h>	                    // For sqrtf.
#include <float.h>	                    // Required for FLT_EPSILON.
#include <stdbool.h>                    // For bool datatype.
#include <time.h>                       // For srand, clock.

#include "matf32.h"
#include "linsolve.h"
#include "quadprog.h"




//// ====================================================================================================
//// Miscellaneous
//// ====================================================================================================
//void
//cut(float A[], uint16_t row, uint16_t column, float B[], uint16_t start_row, uint16_t stop_row, uint16_t start_column, uint16_t stop_column);
//
//void
//insert(float A[], float B[], uint16_t row_a, uint16_t column_a, uint16_t column_b, uint16_t startRow_b, uint16_t startColumn_b);
///**
//  * Linear algebra.
//  */
//void
//svd_jacobi_one_sided(float A[], uint16_t row, uint8_t max_iterations, float U[], float S[], float V[]);
//
//void
//dlyap(float A[], float P[], float Q[], uint16_t row);
//
//uint8_t
//svd_golub_reinsch(float A[], uint16_t row, uint16_t column, float U[], float S[], float V[]);
//

//float
//det(float A[], uint16_t row);
//
//uint8_t
//linsolve_lup(float A[], float x[], float b[], uint16_t row);
//
//void
//pinv(float A[], uint16_t row, uint16_t column);
//
//void
//hankel(float V[], float H[], uint16_t row_v, uint16_t column_v, uint16_t row_h, uint16_t column_h, uint16_t shift);
//
//void
//balance(float A[], uint16_t row);
//
//void
//eig(float A[], float wr[], float wi[], uint16_t row);
//
//void
//eig_sym(float A[], uint16_t row, float d[]);
//
//void
//sum(float A[], uint16_t row, uint16_t column, uint8_t l);
//
//float
//norm(float A[], uint16_t row, uint16_t column, uint8_t l);
//
//void
//expm(float A[], uint16_t row);
//
//void
//nonlinsolve(void (*nonlinear_equation_system)(float[], float[], float[]), float b[], float x[], uint8_t elements, float alpha, float max_value, float min_value, bool random_guess_active);
//
//void
//linsolve_gauss(float* A, float* x, float* b, uint16_t row, uint16_t column, float alpha);
//
//
//
///**
//  * Optimization.
//  */
//  /** @TODO: implement convex quadprog and general gradient descent w/o constraints. */
//void
//linprog(float c[], float A[], float b[], float x[], uint8_t row_a, uint8_t column_a, uint8_t max_or_min, uint8_t iteration_limit);

#endif /* ROBOTAT_LINALG_H_ */
