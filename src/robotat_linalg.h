/**
 * @defgroup Robotat-Linalg
 * @{
 *
 * Robotat Linalg main file to import matf32, linsolve and quadprog libraries.
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
 *    reduce overhead by undefining the MATH_MATRIX_CHECK macro)
 * 6. Organized the matf32 routines in the following files: matf32_check, matf32_def,
 *    matf32_math and math_util, while linsolve and quadprog have a single file. Included routines for
 *    matrix operations and factorizations, matrix check functions, linear solver methods and methods for
 *    convex quadratic programs with equality and inequality restricctions.
 * 7. Combined matf32_check, matf32_def, matf32_math and math_util into a single matf32
 *    pair of files, to facilitate use and organization. Also, moved matrix factorizations from linsolve 
 *    to matf32, for subject coherence and to be able to use them in other applications without creating
 *    dependence from matf32 to linsolve.
 * 8. Validated the previously developed routines in matf32, linsolve and quadprog. Made significant changes
 *    in the algorithms of the following: matf32_lu and linsolve lu (added partial pivoting for stability),
 *    matf32_cholesky (fixed issues), ones (fixed issues), quadprog_sqp (fixed issues and adapted to solve
 *    problems with both equality and inequality restriccions). Also renamed quadprog_qp as quadprog_qp_linsolve
 *    and modified it to allow to choose which linsolve method to be used.
 * 9. Added the following routines to their respective libraries: matf32_pinv, matf32_cond, matf32_check_symposdef,
 *    matf32_one_sided_jacobi, matf32_jacobi_svd, linsolve_qr, linsolve_cholesky, linsolve_svd, quadprog_qp_ldlt,
 *    quadprog_qp_nullspace.
 * 10.Added the robotat_control and robotat_robotics libraries.
 * 
 * 
 *  @date Created on: 5 oct. 2019
 *            By: Daniel Martensson (original author) \n
 *  Modified on: 1 aug. 2021
 *            By: Miguel Zea (mezea@uvg.edu.gt) \n
 *  Modified on: 20 may. 2022
 *            By: Daniel Pineda (bar18714@uvg.edu.gt) \n
 *  Modified on: 26 nov. 2025
 *            By: Andrea Pineda (bar20575@uvg.edu.gt) \n
 */

#ifndef ROBOTAT_LINALG_H_
#define ROBOTAT_LINALG_H_

// C Libraries
#include <string.h>	                    // For memcpy, memset etc.
#include <stdio.h>                      // For printf.
#include <stdlib.h>                     // Standard library.
#include <stdint.h>	                    // For uint8_t, uint16_t and uint16_t.
#include <math.h>	                      // For sqrtf.
#include <float.h>	                    // Required for FLT_EPSILON.
#include <stdbool.h>                    // For bool datatype.
#include <time.h>                       // For srand, clock.

// Robotat Linalg Libraries
#include "matf32.h"                     // Matrix Linear Algebra Library
#include "linsolve.h"                   // Linear Solver Library
#include "quadprog.h"                   // Convex Quadratic Programming Library

#endif /* ROBOTAT_LINALG_H_ */

/**
 * @}
 */
