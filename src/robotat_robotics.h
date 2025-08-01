
/**
 * @file robotat_robotics.h
 * @author Andrea Pineda
 * @date created 19 Jul. 2025
 *
 * Robotics algorithms
*/

#ifndef ROBOTAT_ROBOTICS_H_
#define ROBOTAT_ROBOTICS_H_

/**
 * Dependencies
 */

#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>               // For sin() and cos()
#include "matf32.h"
#include "linsolve.h"

#ifdef __cplusplus
extern "C" {
#endif

// ====================================================================================================
// Data structures, enums and type definitions
// ====================================================================================================

// ----- TODO -----

// PENDING TO ADD IN THE .h
// funciones para generar secuencias de rotación eje-angulo, angulos de euler-cuaterniones, viceversa, todas las operaciones con cuaterniones.
// cuidado con las singularidades en todas estas operaciones.
// combinar o multiplicar transformaciones para comprobar si se están realizando bien entre fuente-destino
// Documentación:
//  - documentacion en github, doxygen, explicar cómo usar y dejar unos ejemplos de cómo usar la librería - digerible para estudiantes y publicación
//  - esto aparte de mi tesis - qué hice y cómo lo hice, etc.
//  - añadir carpeta de demos para mostrar ejemplos explicados de cómo usar las librerías y funciones.
// leer acerca de "uniones" en lenguaje c para cuaterniones: operar tanto a nivel de matrices como de elementos específicos.
// Precheckear dimensiones de la matriz de transformacion homogenea en el init.
// Comparar resultado y tiempo de operación de las funciones con equivalentes de Robotics Toolbox en Matlab (exceptuando prints, inits, etc. de esos solo ver respuesta y medir tiempo por referencia).
// Want to add a rob_status_t, equivalent to the err_status_t in matf32 but for the robotics library. This will replace the void in the function type of the operations.



/**
 *  ADDED IN THE .h BUT NOT FINISHED IN THE .c
 *  - inversa de T
 *  - inicializar cuaternion
 *  - imprimir cuaternion
 *  - rob_ishomog = revisar si T es una matriz 4x4 o 4x4xN (incluir dentro de rob_frame_init para verificar dimensiones)
 *  - rob_isrot = revisar si R es una matriz 3x3 o 3x3xN.
 *  - rob_isvec = revisar si un vector tiene 3 elementos (columna o fila) = para revisar dimensiones del vector de coordenadas
 *  */ 



// FULLY DONE :D
/**
 * - funciones de robotica: rob_funcion, ctr_funcion
 * - Definir matriz de transformacion homogenea como un matf32_t. 
 * - rob_frame_t para definir la matriz homogenea, init: pedir R, p, (todos matf32)
 * - el tipo de dato de la matriz homogenea debe incluir cual es la matriz de referencia y la matriz objetivo de la transformacion.
 *      - opcion 1: un char - letras mayusculas en tablas ascii (65-90), añadir 2 chars para el typedef del frame_t para identificarlos, igual para los vectores.
 *      - opcion 2: usar enumerates - como los de err_status_t - Frame_A, Frame_B (no puede ser solo una letra) - no tan limitado en cuanto a etiquetas.
 *      - Solución: OPCION 2, USAR ENUMERATES PARA QUE SEA CONSISTENTE CON EL USO DE ENUMERATES EN OTRAS FUNCIONES.
 * - funcion para imprimir los datos de los marcos: rob_print_frame() indicar si se quiere usar como matriz de rotacion.
 * - rob_rotx, rob_roty, rob_rotz, 
 * - rob_transl.
 * - completar init de un punto de referencia, ejemplo A_p o B_p, para poder hacer después la función apply transform que corresponde a: A_p = A_T_B * B_p;
 * - funcion apply_transform: input de un vector y una matriz. mas eficiente multiplicar los valores directamente, sin funciones de matf32.
 * - funciones para generar matrices de rotacion para angulos de euler. Por ejemplo XYZ, ZYZ, etc. 
 */



/**
 * Additional notes:
 * - Added deg2rad() and rad2deg() functions for degrees to rad conversion and viceversa.
 */


/**
 * @brief   Enumerates list for matrix tags.
 */
typedef enum
{
    FRAME_A,  
    FRAME_B,
    FRAME_C,
    FRAME_O   // Global reference frame
} rob_frame_tags_t;

/**
 * @brief   Enumerates list of tags for possible rotation sequences to implement.
 */
typedef enum
{
    XYX,
    XZX,
    YXY,
    YZY,
    ZXZ,
    ZYZ,
    XYZ,
    XZY,
    YZX,
    YXZ,
    ZXY,
    ZYX
} rob_euler_angles_t;

/**
 * @brief   Homogeneneous transformation matrix construct for reference frames' pose calcultions
 */
typedef struct 
{   
    matf32_t* p_T;                /** Points to homogeneous transformation matrix */
    matf32_t* p_R;                /** Points to rotation matrix */
    matf32_t* p_v;                /** Points to coordinates vector */
    rob_frame_tags_t ref_tag;     /** Current reference frame for the transformation */
    rob_frame_tags_t dst_tag;     /** Destination reference frame for the transformation */
    bool angle_units;             /** Angle unit tags, between RADIANS or DEGREES */
} rob_frame_t;

typedef struct
{
    const matf32_t* p_v;        /** Coordinates vector for a given point with respect to the frame */
    rob_frame_tags_t ref_tag;   /** Enumerate tags for frame reference and destination */
} rob_point_t;

/**
 * @brief   Quaternion struct
 */
typedef struct
{
    const matf32_t* p_R;    /** Real number s of the quaternion*/
    const matf32_t* p_t;    /** Imaginary number i of the quaternion*/
    const matf32_t* p_j;    /** Imaginary number j of the quaternion*/
    const matf32_t* p_k;    /** Imaginary number k of the quaternion*/
} rob_quat_t;

// ====================================================================================================
// Function prototypes
// ====================================================================================================
// ====================================================================================================
// Reference frame init and print functions
// ====================================================================================================

/**
 * @brief   Constructor for the reference frame data structure, defines 4x4 dimensiones, sets the rotation matrix,
 * coordinates vector and creates the homogeneous transformation matrix.
 * 
 * @param[in]   p_T             Transformation matrix for the reference frame.
 * @param[in]   p_R             Rotation matrix.
 * @param[in]   p_v             Coordinates vector.
 * @param[in]   ref_tag         Enumerate tag for the current reference frame for the transformation.
 * @param[in]   dst_tag         Enumerate tag for the destination frame for the transformation.
 * @param[in]   angle_units     Enumerate tag for the units of the angles.
 * 
 * @return None
 */
void
rob_frame_init(rob_frame_t* p_F, matf32_t* p_T, matf32_t* p_R, matf32_t* p_v, rob_frame_tags_t ref_tag, rob_frame_tags_t dst_tag, bool angle_units);

/**
 * @brief   Constructor for the reference frame data structure, defines 4x4 dimensiones, sets the rotation matrix,
 * coordinates vector and creates the homogeneous transformation matrix.
 * 
 * @param[in]   p_p         Points to the point struct to use.
 * @param[in]   p_data      Points to data float to use
 * @param[in]   ref_tag     Enum tag to indiate the associated reference frame
 * 
 * @return None
 */
void
rob_refpoint_init(rob_point_t* const p_p, matf32_t* p_v, float* p_data, rob_frame_tags_t ref_tag);

/**
 * @brief   Prints the data and configurations of a reference frame.
 * 
 * @param[in]   p_F     Pointer to reference matrix.
 * 
 * @return None
 */
void
rob_frame_print(rob_frame_t* p_F);

/**
 * @brief   Prints a point's data and configurations to use with it.
 * 
 * @param[in]   p_p     Pointer to a given point.
 * 
 * @return None
 */
void
rob_refpoint_print(rob_point_t* const p_p);

/**
 * @brief   Prints text corresponding to the tags in rob_angle_unit_tags_t.
 */
void
rob_angle_units_print(bool angle_units);

/**
 * @brief   Prints text corresponding to the tags in rob_frame_tags_t.
 * 
 * @param[in]   frame_tags  Frame tags enum
 * 
 * @return None
 */
void
rob_frame_tags_print(rob_frame_tags_t tags);

// ====================================================================================================
// Quaternion functions
// ====================================================================================================

// TODO

// ====================================================================================================
// Homogeneous transform functions
// ====================================================================================================

// TODO: Make an equivalent of err_status_t for this functions. Something like: rob_status_t, and then change the function return type from void to rob_status_t.

/**
 * @brief   Set coordinates vector in the homogeneous transformation matrix
 * 
 * @param[in,out]   p_F     Points to the reference frame struct
 * @param[in]       p_v     Points to the coordinates vector to be implemented
 * 
 * @return  err_status_t
 */
void
rob_transl(rob_frame_t* p_F, matf32_t* p_v);

/**
 * @brief   Applies rotation in the x axis.
 * 
 * @param[in,out]   p_F         Pointer to reference frame struct.
 * @param[in]       p_theta     Points to the value of degrees by which to rotate the frame.
 * 
 * @return err_status_t
 */
void
rob_rotx(rob_frame_t* p_F, float* p_theta);

/**
 * @brief   Applies rotation in the y axis.
 * 
 * @param[in,out]   p_F         Pointer to reference frame struct.
 * @param[in]       p_theta     Angle for the rotation (either radians or degrees, which must be indicated when initiation the frame)
 * 
 * @return None
 */
void
rob_roty(rob_frame_t* p_F, float* p_theta);

/**
 * @brief   Applies rotation in the z axis.
 * 
 * @param[in,out]   p_F         Pointer to reference frame struct.
 * @param[in]       p_theta     Angle for the rotation (either radians or degrees, which must be indicated when initiation the frame)
 * 
 * @return None
 */
void
rob_rotz(rob_frame_t* p_F, float* p_theta);

/**
 * @brief   Applies the homogeneous transformation to calculate the pose.
 * 
 * @param[in]   p_F     Pointer to reference frame struct to work with.
 * @param[in]   p_srcp  Coordinates vector with reference to original frame.
 * @param[out]  p_dstp  Coordinates vector to the new frame of reference.
 * 
 * @return None
 */
void
rob_apply_transform(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp);

/**
 * @brief   Applies inverse of the homogeneous transformation of the pose.
 * 
 * @param[in,out]   p_F     Pointer to reference frame struct.
 * 
 * @return None
 */
void
rob_inv_transform(rob_frame_t* const p_F, const matf32_t* const p_p);

/**
 * @brief   Apply transforms based on Euler angles sequences.
 * 
 * @param[in]   p_F         Points to reference frame struct to work with.  
 * @param[in]   p_srcp      Pointss to reference point for the frame to transform from.
 * @param[out]  p_dstp      Points to reference point with respect to the new frame.
 * @param[in]   euler_tag   Tag to indicate the rotation sequence to apply, for example: XYZ, ZYZ, etc.
 * 
 * @return None
 */
void
rob_apply_euler_angles(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp, rob_euler_angles_t euler_tag, float* p_theta);

// ====================================================================================================
// Utility functions
// ====================================================================================================

/**
 * @brief   Converts degrees to radians.
 * 
 * @return float
 */
float
deg2rad(float* p_theta);

/**
 * @brief   Converts radians to degrees.
 * 
 * @return float
 */
float
rad2deg(float* p_theta);

/**
 * @brief    Check if a matrix is 3x3 to be a rotation matrix
 * 
 * @param[in]   p_R     Points to matrix to be checked.
 * 
 * @return None
 */
void
rob_isrot(matf32_t* p_R);

/**
 * @brief   Check if a matrix is 4x4 to be an homogeneous transformation matrix
 * 
 * @param[in]   p_T     Points to matrix to be checked.
 * 
 * @return None
 */
void
rob_ishomog(matf32_t* p_T);

/**
 * @brief   Check if a vector has 3 elements to be used as coordinates vector in the homogeneous transformation matrix
 * 
 * @param[in]   p_v     Points to vector to be checked (matrix, 1x3 or 3x1);
 * 
 * @return None
 */
void
rob_isvec(matf32_t* p_v);

#ifdef __cplusplus
}
#endif

#endif // ROBOTAT_ROBOTICS_H_