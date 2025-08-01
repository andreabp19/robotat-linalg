#include "robotat_control.h"


static float m1data[MAX_MAT_SIZE];
static matf32_t m1;
static float m2data[MAX_MAT_SIZE];
static matf32_t m2;
static float m3data[MAX_MAT_SIZE];
static matf32_t m3;
static float m4data[MAX_MAT_SIZE];
static matf32_t m4;
static float m5data[MAX_MAT_SIZE];
static matf32_t m5;

// ====================================================================================================
// Public function definitions
// ====================================================================================================
// PID Control
// ====================================================================================================
void
ctr_pid_init(ctr_pid_t* const pid, float kp, float ki, float kd, ctr_discretizations_t pid_alg, bool set_i_limits, ...)
{
	va_list ap;

	pid->e_k_1 = 0;
	pid->u_k_1 = 0;
	// If unspecified, don't saturate the integrator.
	pid->i_min = FLT_MIN + 1;
	pid->i_max = FLT_MAX - 1;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->pid_alg = pid_alg;

	va_start(ap, set_i_limits);

	if (pid_alg == PURE_DISCRETE)
	{
		pid->dt = -1;	// Discrete but unspecified sample time
		pid->tau = -1;  // Use ideal differentiator
		if (set_i_limits)
		{
			pid->i_min = (float) va_arg(ap, float);
			pid->i_max = (float) va_arg(ap, float);
		}
	}
	else
	{
		pid->dt = (float) va_arg(ap, float);
		pid->tau = (float) va_arg(ap, float);
		if (set_i_limits)
		{
			pid->i_min = (float) va_arg(ap, float);
			pid->i_max = (float) va_arg(ap, float);
		}
	}
	va_end(ap);
}


float
ctr_pid_update(ctr_pid_t* const pid, float r_k, float y_k)
{
	float u_k = 0;
	float e_k;
	float E_k;

	e_k = r_k - y_k;

	switch (pid->pid_alg)
	{
	case PURE_DISCRETE:
		E_k = saturation(pid->e_k_1 + e_k, pid->i_min, pid->i_max);
		u_k = pid->kp * e_k + pid->ki * E_k + pid->kd * (e_k - pid->e_k_1);
		break;

	case FWD_EULER:
		E_k = saturation(pid->dt * pid->e_k_1 + pid->u_k_1, pid->i_min, pid->i_max);
		u_k = pid->kp * e_k + pid->ki * E_k +
			pid->kd * (pid->tau * e_k - pid->tau * pid->e_k_1 - (pid->dt * pid->tau - 1) * pid->u_k_1);
		break;

	case BWD_EULER:
		E_k = saturation(pid->dt * e_k + pid->u_k_1, pid->i_min, pid->i_max);
		u_k = pid->kp * e_k + pid->ki * E_k +
			(pid->kd / (pid->dt * pid->tau + 1)) * (pid->tau * e_k - pid->tau * pid->e_k_1 + pid->u_k_1);
		break;

	case TUSTIN:
		E_k = saturation((pid->dt / 2) * (e_k + pid->e_k_1) + pid->u_k_1, pid->i_min, pid->i_max);
		u_k = pid->kp * e_k + pid->ki * E_k +
			((pid->kd * 2 * pid->tau) / (pid->dt * pid->tau + 2)) *
			(e_k - pid->e_k_1) - ((pid->dt * pid->tau - 2) / (pid->dt * pid->tau + 2)) * pid->u_k_1;
		break;

		// TODO: Implement ZOH discretization for the PID controller. 
	case ZOH:

		break;

	default:
		break;
	}

	pid->e_k_1 = e_k;
	pid->u_k_1 = u_k;

	return u_k;
}


// ====================================================================================================
// State space representation
// ====================================================================================================
err_status_t
ctr_ss_lti(matf32_t* A, matf32_t* B, matf32_t* C, matf32_t* D, float sample_time, ctr_sys_lti_t* const sys) 
{
	// Check if the dimensions of all matrices are consistent
	if ((!matf32_size_check(A, A->num_rows, A->num_rows)) || (A->num_cols != B->num_rows) || (A->num_rows != C->num_cols))
		return MATH_SIZE_MISMATCH;

	sys->A = A;
	sys->B = B;
	sys->C = C;
	sys->D = D;
	if (sample_time <= FLT_EPSILON)
	{
		sys->dt = 0;
		sys->is_continuous = true;
	}
	else
	{
		sys->dt = sample_time;
		sys->is_continuous = false;
	}
	sys->state_dim = A->num_rows;
	sys->input_dim = B->num_cols;
	sys->output_dim = C->num_rows;

	return MATH_SUCCESS;
}


err_status_t
ctr_c2d(ctr_sys_lti_t* const sys, float sample_time, ctr_discretizations_t method)
{	
	// Check if system is already discrete time
	if (!sys->is_continuous)
		return MATH_ARGUMENT_ERROR;

	matf32_t* I = &m1;
	matf32_init(I, sys->state_dim, sys->state_dim, m1data);
	matf32_eye(I);

	sys->is_continuous = false;
	sys->dt = sample_time;

	switch (method)
	{
	case PURE_DISCRETE:
		break;

	case FWD_EULER:
		matf32_scale(sys->A, sample_time, sys->A);
		matf32_add(sys->A, I, sys->A);
		matf32_scale(sys->B, sample_time, sys->B);
		break;

	case BWD_EULER:

		break;

	case TUSTIN:

		break;


	case ZOH:

		break;

	default:
		break;
	}

	return MATH_SUCCESS;
}


err_status_t
ctr_sys_lti_init(ctr_sys_lti_t* const sys, matf32_t* const state, matf32_t* const A, matf32_t* const B, matf32_t* const C, matf32_t* const D, float sample_time)
{
	const uint16_t state_dim = state->num_rows; 

#ifdef MATH_MATRIX_CHECK
	if (state->num_cols != 1) return MATH_SIZE_MISMATCH;
	
	if ((matf32_size_check(A, state_dim, state_dim)) 
		&& (B->num_rows == state_dim) && (C->num_cols == state_dim) 
		&& (B->num_cols == D->num_cols) && (C->num_rows == D->num_rows));
	else return MATH_SIZE_MISMATCH;
#endif
	sys->state = state;
	sys->state_dim = state_dim;
	sys->input_dim = B->num_cols;
	sys->output_dim = C->num_rows;
	sys->A = A;
	sys->B = B;
	sys->C = C;
	sys->D = D;

	if (sample_time <= FLT_EPSILON)
	{
		sys->dt = 0;
		sys->is_continuous = true;
	}
	else
	{
		sys->dt = sample_time;
		sys->is_continuous = false;
	}
	return MATH_SUCCESS;
}


// NOTE: change matf32_size_check to matf32_is_correct_size for readability, maybe add matf32_is_colvector
err_status_t
ctr_sys_nonlin_init(ctr_sys_nonlin_t* const sys, matf32_t* const state, uint16_t input_dim, uint16_t output_dim, err_status_t(*dynamics)(matf32_t* const, const matf32_t*, const matf32_t*), err_status_t(*outputs)(matf32_t* const, const matf32_t*, const matf32_t*), float sample_time)
{
	const uint16_t state_dim = state->num_rows;

#ifdef MATH_MATRIX_CHECK
	if (state->num_cols != 1) return MATH_SIZE_MISMATCH;

	matf32_t* const test_input = &m1;
	matf32_t* const test_xdot = &m2;
	matf32_t* const test_output = &m3;

	err_status_t error;

	matf32_zeros(state);
	matf32_init(test_input, input_dim, 1, m1data);
	matf32_init(test_xdot, state_dim, 1, m2data);
	matf32_init(test_output, output_dim, 1, m3data);

	error = dynamics(test_xdot, state, test_input);
	if(error != MATH_SUCCESS) return error;

	error = outputs(test_output, state, test_input);
	if (error != MATH_SUCCESS) return error;
#endif
	sys->state = state;
	sys->state_dim = state_dim;
	sys->input_dim = input_dim;
	sys->output_dim = output_dim;
	sys->dynamics = dynamics;
	sys->outputs = outputs;

	if (sample_time <= FLT_EPSILON)
	{
		sys->dt = 0;
		sys->is_continuous = true;
	}
	else
	{
		sys->dt = sample_time;
		sys->is_continuous = false;
	}
	return MATH_SUCCESS;
}


err_status_t
ctr_linloc(ctr_sys_nonlin_t* const src_sys, ctr_sys_lti_t* const dst_sys, const matf32_t* const xss, const matf32_t* const uss, float delta)
{
#ifdef MATH_MATRIX_CHECK
	if ((src_sys->state_dim == dst_sys->state_dim) && (src_sys->input_dim == dst_sys->input_dim) && (src_sys->output_dim == dst_sys->output_dim));
	else return MATH_SIZE_MISMATCH;

	if (matf32_size_check(xss, src_sys->state_dim, 1) && matf32_size_check(uss, src_sys->output_dim, 1));
	else return MATH_SIZE_MISMATCH;
#endif

	matf32_t* const fss = &m1;
	matf32_t* const hss = &m2;
	matf32_t* const dx = &m3;
	matf32_t* const df = &m4;
	matf32_t* const dh = &m5;
	const uint16_t state_dim = src_sys->state_dim;
	const uint16_t input_dim = src_sys->input_dim;
	const uint16_t output_dim = src_sys->output_dim;
	float xss_i; // xss[i, 1]
	float uss_i; // uss[i, 1]

	matf32_init(fss, state_dim, 1, m1data);
	matf32_init(hss, output_dim, 1, m2data);
	matf32_init(dx, state_dim, 1, m3data);
	matf32_init(df, state_dim, 1, m4data);
	matf32_init(dh, output_dim, 1, m5data);

	// Get the derivative and output at steady state (does not necessarily have to be an equilibrium point)
	src_sys->dynamics(fss, xss, uss); // xdot_ss = fss = f(xss, uss) 
	src_sys->outputs(hss, xss, uss); // yss = hss = h(xss, uss)

	// Transpose the resulting LTI system's matrices to use column linear indexing
	matf32_trans(dst_sys->A, dst_sys->A);
	matf32_trans(dst_sys->B, dst_sys->B);
	matf32_trans(dst_sys->C, dst_sys->C);
	matf32_trans(dst_sys->D, dst_sys->D);

	// Get the Jacobians with respect to the state vector
	matf32_copy(xss, dx);
	for (uint16_t i = 1; i <= state_dim; i++)
	{
		matf32_get(dx, i, 1, &xss_i);
		matf32_set(dx, i, 1, xss_i + delta);
		
		src_sys->dynamics(df, dx, uss);
		matf32_sub(df, df, fss);
		matf32_scale(df, 1 / delta, df);
		memcpy(dst_sys->A->p_data + (i - 1) * state_dim, df->p_data, state_dim * sizeof(float));

		src_sys->outputs(dh, dx, uss);
		matf32_sub(dh, dh, hss);
		matf32_scale(dh, 1 / delta, dh);
		memcpy(dst_sys->C->p_data + (i - 1) * output_dim, df->p_data, output_dim * sizeof(float));

		matf32_set(dx, i, 1, xss_i);
	}

	// Get the Jacobians with respecto to the input vector
	matf32_t* const du = &m3;
	matf32_reshape(du, output_dim, 1);

	matf32_copy(uss, du);
	for (uint16_t i = 1; i <= input_dim; i++)
	{
		matf32_get(du, i, 1, &uss_i);
		matf32_set(du, i, 1, uss_i + delta);

		src_sys->dynamics(df, xss, du);
		matf32_sub(df, df, fss);
		matf32_scale(df, 1 / delta, df);
		memcpy(dst_sys->B->p_data + (i - 1) * state_dim, df->p_data, state_dim * sizeof(float));

		src_sys->outputs(dh, xss, du);
		matf32_sub(dh, dh, hss);
		matf32_scale(dh, 1 / delta, dh);
		memcpy(dst_sys->D->p_data + (i - 1) * output_dim, df->p_data, output_dim * sizeof(float));

		matf32_set(du, i, 1, uss_i);
	}

	// Return to row linear indexing
	matf32_trans(dst_sys->A, dst_sys->A);
	matf32_trans(dst_sys->B, dst_sys->B);
	matf32_trans(dst_sys->C, dst_sys->C);
	matf32_trans(dst_sys->D, dst_sys->D);

	return MATH_SUCCESS;
}


// ====================================================================================================
// Linear state space controllers
// ====================================================================================================
err_status_t
ctr_linear_state_feedback(matf32_t* const u, const matf32_t* K, const matf32_t* x, const matf32_t* xss, const matf32_t* uss)
{
#ifdef MATH_MATRIX_CHECK
	if (matf32_is_same_size(x, xss) && matf32_size_check(K, uss->num_rows, x->num_rows) &&
		(x->num_cols == 1) && (xss->num_cols == 1) && (u->num_cols == 1) && (uss->num_cols == 1));
	else return MATH_SIZE_MISMATCH;
#endif
	// Matrices to store intermediate results
	matf32_t* const z = &m1;
	matf32_t* const Kz = &m2;
	matf32_init(z, x->num_rows, 1, m1data); // z: dim(x) x 1
	matf32_init(Kz, u->num_rows, 1, m2data); // Kz: dim(u) x 1

	// Update the linear state feedback controller u = -K * (x - xss) + uss = K * (xss - x) + uss
	matf32_sub(xss, x, z); // z = xss - x
	matf32_mul(K, z, Kz); // Kz
	matf32_add(Kz, uss, u); // u = Kz + uss

	return MATH_SUCCESS;
}


// ====================================================================================================
// Linear time-varying, discrete time Kalman filter
// ====================================================================================================
err_status_t
ctr_kalman_init(ctr_kalman_t* const kf, ctr_sys_lti_t* const sys, matf32_t* F, matf32_t* Qw, matf32_t* Qv, matf32_t* const xhat, matf32_t* const P)
{
	// Mandatory size checking (change to use new size checking routines)
	if ( (sys->A->num_rows == P->num_rows) && (sys->A->num_cols == P->num_cols) 
		&& (Qw->num_rows == sys->input_dim) && (Qv->num_rows == sys->output_dim) );
	else return MATH_SIZE_MISMATCH;

	// Check if the dynamics are discrete-time
	if (sys->is_continuous)
		return MATH_ARGUMENT_ERROR;

	// Initialize the kalman filter data structure (assumes that the initial condition and covariance matrix are set by the user)
	kf->sys = sys;
	kf->F = F;
	kf->Qw = Qw;
	kf->Qv = Qv;
	kf->xhat = xhat;
	kf->P = P;

	return MATH_SUCCESS;
}


err_status_t
ctr_kalman_predict(ctr_kalman_t* const kf, const matf32_t* inputs)
{
	// Check if the inputs vector has the correct size
	if ((inputs->num_rows != kf->sys->input_dim) || (inputs->num_cols != 1))
		return MATH_SIZE_MISMATCH;

	// State, input, output and noise dimensions
	const float dim_xhat = kf->sys->state_dim;
	const float dim_u = kf->sys->input_dim;
	const float dim_y = kf->sys->output_dim;
	const float dim_w = kf->Qw->num_rows;
	const float dim_v = kf->Qv->num_rows;

	// Use the dynamics to get the a-priori estimate
	matf32_t* const Ax = &m1;
	matf32_t* const Bu = &m2;
	matf32_init(Ax, dim_xhat, 1, m1data); // Ax: dim(xhat) x 1
	matf32_init(Bu, dim_xhat, 1, m2data); // Bu: dim(xhat) x 1
	
	matf32_mul(kf->sys->A, kf->xhat, Ax); // A[k] * xhat[k-1|k-1]
	matf32_mul(kf->sys->B, inputs, Bu); // B[k] * u[k]
	matf32_add(Ax, Bu, kf->xhat); // xhat[k|k-1] = A[k] * xhat[k-1|k-1] + B[k] * u[k] 
	
	// Update the covariance matrix using the dynamics and process noise covariance
	matf32_t* const At = &m1;
	matf32_t* const Ft = &m2;
	matf32_reshape(At, dim_xhat, dim_xhat); // A^T: dim(xhat) x dim(xhat)
	matf32_reshape(Ft, dim_w, dim_xhat); // F^T: dim(w) x dim(xhat)
	
	matf32_trans(kf->sys->A, At); // A^T
	matf32_trans(kf->F, Ft); // F^T

	const matf32_t* const APAt[] = {kf->sys->A, kf->P, At};
	matf32_arr_mul(APAt, 3, kf->P); // A[k] * P[k-1|k-1]) * A[k]'

	matf32_t* const tmpmat = &m1;
	matf32_reshape(tmpmat, dim_xhat, dim_xhat); 
	const matf32_t* const FQwFt[] = {kf->F, kf->Qw, Ft};
	matf32_arr_mul(FQwFt, 3, tmpmat); // F[k] * Qw[k-1]) * F[k]'

	matf32_add(kf->P, tmpmat, kf->P); // P[k|k-1] = A[k] * P[k-1|k-1] * A[k]' + F[k] * Qw[k-1] * F[k]'

	return MATH_SUCCESS;
}


err_status_t
ctr_kalman_correct(ctr_kalman_t* const kf, const matf32_t* measurements)
{
	// Check if the measurements vector has the correct size
	if ((measurements->num_rows != kf->sys->output_dim) || (measurements->num_cols != 1))
		return MATH_SIZE_MISMATCH;

	// State, input, output and noise dimensions
	const float dim_xhat = kf->sys->state_dim;
	const float dim_u = kf->sys->input_dim;
	const float dim_y = kf->sys->output_dim;
	const float dim_w = kf->Qw->num_rows;
	const float dim_v = kf->Qv->num_rows;
	
	// Get the innovation covariance matrix and its inverse
	matf32_t* const Ct = &m1;
	matf32_t* const S = &m2;
	matf32_t* const Si = &m3;
	matf32_init(Ct, dim_xhat, dim_y, m1data); // S: dim(xhat) x dim(y)
	matf32_init(S, dim_y, dim_y, m2data); // C^T: dim(y) x dim(y)
	matf32_init(Si, dim_y, dim_y, m3data); // S^-1: dim(y) x dim(y)

	matf32_trans(kf->sys->C, Ct); // C^T
	const matf32_t* const CPCt[] = { kf->sys->C, kf->P, Ct };
	matf32_arr_mul(CPCt, 3, S); // C[k] * P[k|k-1] * C[k]'
	matf32_add(S, kf->Qv, S); // S[k] = C[k] * P[k|k-1] * C[k]' + Qv[k]
	err_status_t status = matf32_inv(S, Si); // S[k]^-1
	
	if (status != MATH_SUCCESS)
		return status;

	// Get the Kalman gain
	matf32_t* const L = &m2;
	matf32_reshape(L, dim_xhat, dim_y); // L: dim(xhat) x dim(y)

	const matf32_t* const PCtSi[] = { kf->P, Ct, Si };
	matf32_arr_mul(PCtSi, 3, L); // L[k] = P[k|k-1] * C[k]' * S[k]^-1

	// Update the estimate covariance matrix
	matf32_t* const I = &m1;
	matf32_t* const I_LC = &m3;
	matf32_reshape(I, dim_xhat, dim_xhat); // I: dim(xhat) x dim(xhat)
	matf32_reshape(I_LC, dim_xhat, dim_xhat); // I - LC: dim(xhat) x dim(xhat)

	matf32_eye(I);
	matf32_mul(L, kf->sys->C, I_LC); // L[k] * C[k]
	matf32_sub(I, I_LC, I_LC); // I - L[k] * C[k]

	matf32_t* const Pkk = &m1;
	matf32_reshape(Pkk, dim_xhat, dim_xhat); // P: dim(xhat) x dim(xhat)

	matf32_mul(I_LC, kf->P, Pkk); // (I - L[k] * C[k]) * P[k|k-1]
	matf32_copy(Pkk, kf->P); // P[k|k] = (I - L[k] * C[k]) * P[k|k-1]

	// Update the state estimate
	matf32_t* const xhatkk = &m1;
	matf32_reshape(xhatkk, dim_xhat, 1); // xhat: dim(xhat) x 1

	matf32_mul(I_LC, kf->xhat, xhatkk); // (I - L[k] * C[k]) * x[k|k-1]
	matf32_copy(xhatkk, kf->xhat); // x[k|k] - L[k] * y[k] = (I - L[k] * C[k]) * x[k|k-1]
	
	matf32_t* const Ly = &m1;
	matf32_reshape(Ly, dim_xhat, 1); // L*y: dim(xhat) x 1

	matf32_mul(L, measurements, Ly); // L[k] * y[k]
	matf32_add(kf->xhat, Ly, kf->xhat); // x[k|k] = (I - L[k] * C[k]) * x[k|k-1] + L[k] * y[k]

	return MATH_SUCCESS;
}


//void
//kalman_predict(kalman_info_t* const kf, float* const inputs)
//{
//	matf32_t* tmpmat1 = &m1;
//	matf32_t* tmpmat2 = &m2;
//	matf32_t* tmpmat3 = &m3;
//	float dim_xhat = kf->sys->state_dim;
//	float dim_u = kf->sys->input_dim;
//	float dim_y = kf->sys->output_dim;
//	float dim_w = kf->Qw->num_rows;
//	float dim_v = kf->Qv->num_rows;
//
//	// Temp 'vectors' to store partial results
//	matf32_init(tmpmat1, dim_xhat, 1, m1data); // tmpmat1: dim(xhat) x 1
//	matf32_init(tmpmat2, dim_u, 1, inputs); // tmpmat2: dim(u) x 1
//	matf32_init(tmpmat3, dim_xhat, 1, m3data); // tmpmat1: dim(xhat) x 1
//
//	// Predict the prior using the linear dynamics
//	matf32_mul(kf->sys->A, kf->xhat, tmpmat1); // tmpmat1 = A[k]*xhat[k-1|k-1]
//	matf32_mul(kf->sys->B, tmpmat2, tmpmat3); // tmpmat3 = B[k]*u[k], tmpmat2 = u[k]
//	matf32_add(tmpmat1, tmpmat3, kf->xhat); // xhat[k|k-1] = A[k]*xhat[k-1|k-1] + B[k]*u[k] 
//
//	// Update the covariance matrix using the dynamics and process noise covariance
//	matf32_reshape(tmpmat1, dim_xhat, dim_w); // tmpmat1: dim(xhat) x dim(w)
//	matf32_reshape(tmpmat2, dim_w, dim_xhat); // tmpmat2: dim(w) x dim(xhat)
//	tmpmat2->p_data = &m2data;
//	matf32_reshape(tmpmat3, dim_xhat, dim_xhat); // tmpmat3: dim(xhat) x dim(xhat)
//	
//	matf32_mul(kf->F, kf->Qw, tmpmat1); // tmpmat1 = F[k]*Qw[k-1]
//	matf32_trans(kf->F, tmpmat2); // tmpmat2 = F[k]' 
//	matf32_mul(tmpmat1, tmpmat2, tmpmat3); // tempmat3 = (F[k]*Qw[k-1]) * F[k]' 
//
//	matf32_reshape(tmpmat1, dim_xhat, dim_xhat); // tmpmat1: dim(xhat) x dim(xhat)
//	matf32_reshape(tmpmat2, dim_xhat, dim_xhat); // tmpmat2: dim(xhat) x dim(xhat)
//	
//	matf32_mul(kf->sys->A, kf->P, tmpmat1); // tmpmat1 = A[k]*P[k-1|k-1]
//	matf32_trans(kf->sys->A, tmpmat2); // tmpmat2 = A[k]'
//	matf32_mul(tmpmat1, tmpmat2, kf->P); // kf->P = (A[k]*P[k-1|k-1]) * A[k]'
//
//	// P[k|k-1] = A[k]*P[k-1|k-1] + F[k]*Qw[k-1]*F[k]' = kf->P + tmpmat3  
//	matf32_add(kf->P, tmpmat3, kf->P);
//}


//err_status_t
//kalman_correct(kalman_info_t* const kf, float* const measurements)
//{
//	err_status_t status;
//	matf32_t* tmpmat1 = &m1;
//	matf32_t* tmpmat2 = &m2;
//	matf32_t* tmpmat3 = &m3;
//	float dim_xhat = kf->sys->state_dim;
//	float dim_u = kf->sys->input_dim;
//	float dim_y = kf->sys->output_dim;
//	float dim_w = kf->Qw->num_rows;
//	float dim_v = kf->Qv->num_rows;
//
//	// Temp matrices to store partial results
//	matf32_init(tmpmat1, dim_xhat, dim_y, m1data); // tmpmat1: dim(xhat) x dim(y)
//	matf32_init(tmpmat2, dim_xhat, dim_y, m2data); // tmpmat2: dim(xhat) x dim(y)
//	matf32_init(tmpmat3, dim_y, dim_y, m3data); // tmpmat3: dim(y) x dim(y)
//
//	// Innovation covariance
//	matf32_trans(kf->sys->C, tmpmat1); // tmpmat1 = C[k]'
//	matf32_mul(kf->P, tmpmat1, tmpmat2); // tmpmat2 = P[k|k-1]*C[k]'
//	matf32_mul(kf->sys->C, tmpmat2, tmpmat3); // tmpmat3 = C[k]*P[k|k-1]*C[k]'
//	matf32_add(tmpmat3, kf->Qv, tmpmat3); // S[k] = tmpmat3 += Qv[k]
//
//	// Kalman gain
//	status = matf32_inv(tmpmat3, tmpmat3); // S[k]^-1 = tmpmat3^-1 = (C[k]*P[k|k-1]*C[k]' + Qv[k])^-1 
//	// If matrix inversion fails, return from kalman update 
//	if (status != MATH_SUCCESS)
//		return status;
//
//	matf32_reshape(tmpmat1, dim_xhat, dim_u); // tmpmat1: dim(xhat) x dim(u)
//	matf32_mul(tmpmat2, tmpmat3, tmpmat1); // tmpmat1 = L[k] = P[k|k-1]*C[k]'*S[k]^-1 = tmpmat2 * tmpmat3 
//
//	// Update the estimates using the measurements
//	matf32_reshape(tmpmat2, dim_xhat, dim_xhat); // tmpmat2: dim(xhat) x dim(xhat)
//	matf32_reshape(tmpmat3, dim_xhat, dim_xhat); // tmpmat3: dim(xhat) x dim(xhat)
//
//	matf32_eye(tmpmat2); // tmpmat2 = I
//	matf32_mul(tmpmat1, kf->sys->C, tmpmat3); // tmpmat3 = L[k]*C[k]
//	matf32_sub(tmpmat2, tmpmat3, tmpmat2); // tmpmat2 = I - L[k]*C[k] = tmpmat2 - tmpmat3
//
//	// Error covariance matrix
//	matf32_mul(tmpmat2, kf->P, tmpmat3); // tmpmat3 = (I - L[k]*C[k])*P[k|k-1] = tmpmat2 * kf->P
//	matf32_copy(tmpmat3, kf->P); // P[k-1|k-1] = tmpmat3 
//
//	matf32_reshape(tmpmat3, dim_xhat, 1); // tmpmat1: dim(xhat) x 1
//
//	// State estimate
//	matf32_mul(tmpmat2, kf->xhat, tmpmat3); // tmpmat3 = (I - L[k]*C[k])*x[k|k-1] = tmpmat2 * kf->xhat
//	matf32_copy(tmpmat3, kf->xhat); // x[k|k] = tmpmat3 + ...
//
//	matf32_reshape(tmpmat2, dim_y, 1); // tmpmat1: dim(y) x 1
//	tmpmat2->p_data = measurements; // tmpmat2 = measurements vector as matrix
//	matf32_mul(tmpmat1, tmpmat2, tmpmat3); // tmpmat3 = L[k]*y[k] = tmpmat1 * tmpmat2
//	matf32_add(kf->xhat, tmpmat3, kf->xhat); // x[k|k] = (I - L[k]*C[k])*x[k|k-1] + L[k]*y[k] = kf->xhat + tmpmat3
//
//	return MATH_SUCCESS;
//}
