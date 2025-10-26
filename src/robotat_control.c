
/**
 * @author Miguel Zea
 * 
 * Last modified: 26 Oct 2025
 * 		By: Andrea Pineda
 * 
 */

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
static float m6data[MAX_MAT_SIZE];
static matf32_t m6;

// ====================================================================================================
// Public function definitions
// ====================================================================================================
// ====================================================================================================
// 1. PID Control
// ====================================================================================================

// Tested => Works
void
ctr_pid_init(ctr_pid_t* const pid, float kp, float ki, float kd, ctr_discretizations_t pid_alg, bool set_i_limits, ...)
{
	va_list ap;

	pid->e_k = 0;
	pid->e_k_1 = 0;
	pid->e_k_2 = 0;
	pid->u_k_1 = 0;
	pid->u_k_2 = 0;
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
			pid->i_min = (float) va_arg(ap, double);
			pid->i_max = (float) va_arg(ap, double);
		}
	}
	else
	{
		pid->dt = (float) va_arg(ap, double);
		pid->tau = (float) va_arg(ap, double);
		if (set_i_limits)
		{
			pid->i_min = (float) va_arg(ap, double);
			pid->i_max = (float) va_arg(ap, double);
		}
	}
	va_end(ap);
}

// TODO: Change to my version of the equations that I used in matlab
float
ctr_pid_update(ctr_pid_t* const pid, float r_k, float y_k)
{
	float e_k;
	float e_k_1 = pid->e_k_1;
	float e_k_2 = pid->e_k_2;
	float u_k = 0;
	float u_k_1 = pid->u_k_1;
	float u_k_2 = pid->u_k_2;
	
	float E_k;
	float a0;

	float kp = pid->kp;
	float ki = pid->ki;
	float kd = pid->kd;
	float T = pid->dt;
	float tau = pid->tau;

	e_k = r_k - y_k;

	switch (pid->pid_alg)
	{
	case PURE_DISCRETE:
		// Original equations
		//E_k = saturation(pid->e_k_1 + e_k, pid->i_min, pid->i_max);
		//u_k = pid->kp * e_k + pid->ki * E_k + pid->kd * (e_k - pid->e_k_1);
		
		u_k = kp*e_k + ki*(e_k_1 + e_k) + kd*(e_k - e_k_1);
		
		break;

	case FWD_EULER:
		//E_k = saturation(pid->dt * pid->e_k_1 + pid->u_k_1, pid->i_min, pid->i_max);
		//u_k = pid->kp * e_k + pid->ki * E_k +
		//	pid->kd * (pid->tau * e_k - pid->tau * pid->e_k_1 - (pid->dt * pid->tau - 1) * pid->u_k_1);
		
		a0 = tau;
		u_k = (kp*tau + ki*tau + kd)*e_k
			+ (kp*(T - 2*tau) + ki*(T - tau) - 2*kd)*e_k_1
			+ (kp*(tau - T) + kd)*e_k_2
			- (T - 2*tau)*u_k_1
			- (tau - T)*u_k_2;
		u_k = u_k / a0;
		
		break;

	case BWD_EULER:
		//E_k = saturation(pid->dt * e_k + pid->u_k_1, pid->i_min, pid->i_max);
		//u_k = pid->kp * e_k + pid->ki * E_k +
		//	(pid->kd / (pid->dt * pid->tau + 1)) * (pid->tau * e_k - pid->tau * pid->e_k_1 + pid->u_k_1);
		
		a0 = T + tau;
            u_k = (kp*(T + tau) + ki*(T + tau) + kd)*e_k
                + (-kp*(2*tau + T) - ki*tau - 2*kd)*e_k_1
                + (kp*tau + kd)*e_k_2
                + (2*tau + T)*u_k_1
                - (tau)*u_k_2;
            u_k = u_k / a0;
		
		break;

	case TUSTIN:
		//E_k = saturation((pid->dt / 2) * (e_k + pid->e_k_1) + pid->u_k_1, pid->i_min, pid->i_max);
		//u_k = pid->kp * e_k + pid->ki * E_k +
		//	((pid->kd * 2 * pid->tau) / (pid->dt * pid->tau + 2)) *
		//	(e_k - pid->e_k_1) - ((pid->dt * pid->tau - 2) / (pid->dt * pid->tau + 2)) * pid->u_k_1;
		
		a0 = 2*tau + T;
		u_k = (kp*(2*tau + T) + 0.5*ki*(2*tau + T) + 2*kd)*e_k
			+ (-kp*4*tau + 0.5*ki*(-2*tau + T) - 4*kd)*e_k_1
			+ (kp*(2*tau - T) + 2*kd)*e_k_2
			+ (4*tau)*u_k_1
			- (2*tau - T)*u_k_2;
		u_k = u_k / a0;
		break;

		// TODO: Implement ZOH discretization for the PID controller. 
	case ZOH:

		break;

	default:
		break;
	}

	pid->e_k_2 = pid->e_k_1;
	pid->e_k_1 = e_k;
	pid->u_k_2 = pid->u_k_1;
	pid->u_k_1 = u_k;

	return u_k;
}



// ====================================================================================================
// 2. State space representation
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 2.1. LTI State Space
// ----------------------------------------------------------------------------------------------------

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


// Tested = works
err_status_t
ctr_c2d(ctr_sys_lti_t* const sys, float sample_time, ctr_discretizations_t method)
{	
	// Check if system is already discrete time
	if (!sys->is_continuous)
		return MATH_ARGUMENT_ERROR;

	matf32_t* I = &m1;
	matf32_init(I, sys->state_dim, sys->state_dim, m1data);
	matf32_eye(I);

	matf32_t* I_Asample_time = &m2;
	matf32_init(I_Asample_time, sys->state_dim, sys->state_dim, m2data);

	matf32_t* temp_B = &m3;
	matf32_init(temp_B, sys->B->num_rows, sys->B->num_cols, m3data);

	matf32_t* temp_A = &m4;
	matf32_init(temp_A, sys->A->num_rows, sys->A->num_cols, m4data);

	sys->is_continuous = false;
	sys->dt = sample_time;

	switch (method)
	{
	case PURE_DISCRETE:
		break;

	case FWD_EULER:

		// A_d = I + A*sample_time
		matf32_scale(sys->A, sample_time, temp_A);
		matf32_add(temp_A, I, sys->A);

		// B_d = B*sample_time
		matf32_scale(sys->B, sample_time, sys->B);
		break;

	case BWD_EULER:

		// A_d = (I - A*sample_time)^-1
		matf32_scale(sys->A, sample_time, sys->A);
		matf32_sub(I, sys->A, sys->A);
		matf32_inv(sys->A, sys->A);

		// B_d = A_d * (B*sample_time)
		matf32_scale(sys->B, sample_time, temp_B);
		matf32_mul(sys->A, temp_B, sys->B);

		break;

	case TUSTIN:

		// I_Asample_time = (I - A*sample_time/2)^-1
		matf32_scale(sys->A, sample_time/2, sys->A);
		matf32_sub(I, sys->A, I_Asample_time);
		matf32_inv(I_Asample_time, I_Asample_time);

		// A_d = I_Asample_time * (I + A*sample_time/2)
		matf32_scale(sys->A, sample_time/2, sys->A);
		matf32_add(I, sys->A, temp_A);
		matf32_mul(I_Asample_time, temp_A, sys->A);

		// B_d = I_Asample_time * (B*sample_time/2)
		matf32_scale(sys->B, sample_time/2, temp_B);
		matf32_mul(I_Asample_time, temp_B, sys->B);

		break;

	case ZOH:
		// TODO: Includes integration, which is not implemented yet.
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

// ----------------------------------------------------------------------------------------------------
// 2.2. Non-linear State Space
// ----------------------------------------------------------------------------------------------------

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

// comparar con loclin_fast en matlab = works
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
		// dx = xss_i + delta
		matf32_get(dx, i, 1, &xss_i);
		matf32_set(dx, i, 1, xss_i + delta);
		
		// A = df(x,u)/dx
		src_sys->dynamics(df, dx, uss); // df = f(xe+dx, ue)
		matf32_sub(df, fss, df); // fss = df - df = 0? Correct seems to be: df = df-fss
		matf32_scale(df, 1 / delta, df); // df = df/delta
		memcpy(dst_sys->A->p_data + (i - 1) * state_dim, df->p_data, state_dim * sizeof(float));

		// C = dh(x,u)/dx
		src_sys->outputs(dh, dx, uss); // dh = h(xe+dx, ue)
		matf32_sub(dh, hss, dh); // hss = dh - dh = 0? Correct seems to be: dh = dh-hss
		matf32_scale(dh, 1 / delta, dh); // dh = dh/delta
		memcpy(dst_sys->C->p_data + (i - 1) * output_dim, dh->p_data, output_dim * sizeof(float)); // Minor error fixed: was using df->p_data instead of dh->p_data

		matf32_set(dx, i, 1, xss_i); // Reset dx to xss_i
	}

	// Get the Jacobians with respect to the input vector
	matf32_t* const du = &m3;
	matf32_reshape(du, output_dim, 1);

	matf32_copy(uss, du);
	for (uint16_t i = 1; i <= input_dim; i++)
	{
		// du = uss_i + delta
		matf32_get(du, i, 1, &uss_i);
		matf32_set(du, i, 1, uss_i + delta);

		// B = df(x,u)/du
		src_sys->dynamics(df, xss, du); // df = f(xe, ue+du)
		matf32_sub(df, fss, df); // fss = df - df = 0? Correct seems to be: df = df-fss
		matf32_scale(df, 1 / delta, df); // df = df/delta
		memcpy(dst_sys->B->p_data + (i - 1) * state_dim, df->p_data, state_dim * sizeof(float));

		// D = dh(x,u)/du
		src_sys->outputs(dh, xss, du); // dh = h(xe, ue+du)
		matf32_sub(dh, hss, dh); // hss = dh - dh = 0? Correct seems to be: dh = dh-hss
		matf32_scale(dh, 1 / delta, dh); // dh = dh/delta
		memcpy(dst_sys->D->p_data + (i - 1) * output_dim, dh->p_data, output_dim * sizeof(float));

		matf32_set(du, i, 1, uss_i); // Reset du to xss_i
	}

	// Return to row linear indexing
	matf32_trans(dst_sys->A, dst_sys->A);
	matf32_trans(dst_sys->B, dst_sys->B);
	matf32_trans(dst_sys->C, dst_sys->C);
	matf32_trans(dst_sys->D, dst_sys->D);

	return MATH_SUCCESS;
}


// ----------------------------------------------------------------------------------------------------
// 2.3. Linear State Space Controllers
// ----------------------------------------------------------------------------------------------------

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


// Tested = Works
err_status_t
ctr_sys_nonlin_simulate(ctr_sys_nonlin_t* sys, const matf32_t* const x_k, matf32_t* const x_k_1, const matf32_t* const u_k, float delta, ctr_discretizations_t method)
{
	matf32_t* fss = &m1;
	matf32_init(fss, x_k->num_rows, x_k->num_cols, m1data);

	matf32_t* k1 = &m2;
	matf32_init(k1, fss->num_rows, fss->num_cols, m2data);

	matf32_t* k2 = &m3;
	matf32_init(k2, fss->num_rows, fss->num_cols, m3data);

	matf32_t* k3 = &m4;
	matf32_init(k3, fss->num_rows, fss->num_cols, m4data);

	matf32_t* k4 = &m5;
	matf32_init(k4, fss->num_rows, fss->num_cols, m5data);

	float temp_data[MAX_MAT_SIZE];
	matf32_t temp;
	matf32_init(&temp, fss->num_rows, fss->num_cols, temp_data);

	switch (method)
	{
		case FWD_EULER:

			/**
			 * Forward Euler
			 * x_k_1 = x_k + f(x_k)*delta
			 */

			sys->dynamics(fss, x_k, u_k);	// f(x_k, u_k)
			matf32_scale(fss, delta, fss); 	// f(x_k, u_k)*delta
			matf32_add(x_k, fss, x_k_1); 	// x_k_1 = x_k + f(x_k, u_k)*delta

			break;

		case RK4:

			/**
			 * Runge-Kutta-4
			 * x_k_1 = x_k + (delta/6)(k1 + 2k2 + 2k3 + k4)
			 * 		 = x_k + (delta/6)(k1 + 2(k2 + k3) + k4)
			 * 
			 * k1 = f(x_k)
			 * k2 = f(x_k + (delta/2)*k1)
			 * k3 = f(x_k + (delta/2)*k2)
			 * k4 = f(x_k + delta*k3)
			 */

			// k1
			sys->dynamics(k1, x_k, u_k); 		// k1 = f(x_k, u_k)

			// k2
			matf32_scale(k1, delta/2, k2); 		// (delta/2)*k1
			matf32_add(x_k, k2, &temp); 		// temp = x_k + (delta/2)*k1
			sys->dynamics(k2, &temp, u_k);		// k2 = f(temp, u_k) = f(x_k + (delta/2)*k1, u_k)
			
			// k3
			matf32_scale(k2, delta/2, k3);		// (delta/2)*k2
			matf32_add(x_k, k3, &temp);			// temp = x_k + (delta/2)*k2
			sys->dynamics(k3, &temp, u_k);		// k3 = f(temp, u_k) = f(x_k + (delta/2)*k2, u_k)

			// k4
			matf32_scale(k3, delta, k4);		// delta*k3
			matf32_add(x_k, k4, &temp);			// temp = x_k + delta*k3
			sys->dynamics(k4, &temp, u_k);		// k4 = f(temp, u_k) = f(x_k + delta*k3, u_k)

			// x_k_1
			matf32_add(k2, k3, k3);				// k2 + k3
			matf32_scale(k3, 2, &temp);			// temp = 2*(k2 + k3) = 2k2 + 2k3
			matf32_add(&temp, k4, k4);			// k4 = 2k2 + 2k3 + k4
			matf32_add(k4, k1, &temp);			// temp = k1 + 2k2 + 2k3 + k4
			matf32_scale(&temp, delta/6, k1);	// k1 = (delta/6)(k1 + 2k2 + 2k3 + k4)
			matf32_add(x_k, k1, x_k_1);			// x_k_1 = x_k + (delta/6)(k1 + 2k2 + 2k3 + k4)

			break;
	}

	return MATH_SUCCESS;
}


// ====================================================================================================
// 3. Kalman Filter (Linear time-varying, discrete time)
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

	const matf32_t* APAt[] = {kf->sys->A, kf->P, At};
	matf32_arr_mul(APAt, 3, kf->P); // A[k] * P[k-1|k-1]) * A[k]'

	matf32_t* const tmpmat = &m1;
	matf32_reshape(tmpmat, dim_xhat, dim_xhat); 
	const matf32_t* FQwFt[] = {kf->F, kf->Qw, Ft};
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
	const matf32_t* CPCt[] = { kf->sys->C, kf->P, Ct };
	matf32_arr_mul(CPCt, 3, S); // C[k] * P[k|k-1] * C[k]'
	matf32_add(S, kf->Qv, S); // S[k] = C[k] * P[k|k-1] * C[k]' + Qv[k]
	err_status_t status = matf32_inv(S, Si); // S[k]^-1
	
	if (status != MATH_SUCCESS)
		return status;

	// Get the Kalman gain
	matf32_t* const L = &m2;
	matf32_reshape(L, dim_xhat, dim_y); // L: dim(xhat) x dim(y)

	const matf32_t* PCtSi[] = { kf->P, Ct, Si };
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


// Other version of kalman functions (don't touch)

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


// ====================================================================================================
// 4. Model Predictive Control (MPC)
// ====================================================================================================

err_status_t
ctr_mpc_unconstrained_lti_init(ctr_mpc_lti_shooting_t* mpc, quadprog_t* qp, ctr_sys_lti_t* sys, matf32_t* const u_k,
	matf32_t* const x_k, matf32_t* const Ain, matf32_t* const bin, float** const mpc_Q, float** const mpc_R, float** const mpc_C, float** const mpc_M, float N)
{
	mpc->sys = sys; // Set LTI system for the MPC, but don't initialize it yet
	mpc->qp = qp; // Set QP for the MPC, but don't initialize it yet

	// MPC Internal Matrices, arrays and floats
	mpc->u_k = u_k;
	mpc->x_k = x_k;
	mpc->N = N;
	mpc->mpc_Q = mpc_Q;
	mpc->mpc_R = mpc_R;
	mpc->mpc_C = mpc_C;
	mpc->mpc_M = mpc_M;
	mpc->Ain = Ain;
	mpc->bin = bin;
}


err_status_t
ctr_mpc_set_M(ctr_mpc_lti_shooting_t* mpc)
{
	uint16_t N = mpc->N; // Horizon length

	matf32_t* const A = mpc->sys->A;

	float** const mpc_M = mpc->mpc_M;

	float temp_data[MAX_MAT_SIZE];
	matf32_t temp;

	for (uint16_t k = 0; k < N; ++k)
	{
		uint16_t exp = k + 1;
		matf32_init(&temp, A->num_rows, A->num_cols, mpc_M[k]);
		matf32_exp(A, &temp, exp);
	}

	for (uint16_t k = 0; k < N; ++k)
	{
		printf("M%i:\n", k+1);
		matf32_init(&temp, A->num_rows, A->num_cols, mpc_M[k]);
		matf32_print(&temp);
	}

	return MATH_SUCCESS;
}


err_status_t
ctr_mpc_set_C(ctr_mpc_lti_shooting_t* mpc)
{
	uint16_t N = mpc->N; // Horizon length

	matf32_t* const A = mpc->sys->A;
	matf32_t* const B = mpc->sys->B;

	float** const mpc_C = mpc->mpc_C;

	matf32_t* Ak = &m1; // To save A^i
	matf32_init(Ak, A->num_rows, A->num_cols, m1data);
	matf32_zeros(Ak);

	matf32_t temp;

	// Assign values with row-major indexing of C
	for (uint16_t i = 0; i < N; ++i) // Rows
	{
		for (uint16_t j = 0; j < N; ++j) // Columns
		{
			matf32_init(&temp, A->num_rows, B->num_cols, mpc_C[i*N + j]);

			if (i > j) // Set A^i * B in the lower triangle, with increasing exponents towards the lower rows 
			{
				/*
					The exponent starts in 1 just below the diagonal.
					Corresponds to: (1,0) => A^1 * B
									(2,0) => A^2 * B
									(2,1) => A^1 * B
									...
					So the difference between the indices == exponent to be used
				*/

				uint16_t exp = (i+1) - (j+1); // i,j are zero-indexed so add 1 to fit the algebraic notation above

				matf32_exp(A, Ak, exp); // A^exp
				matf32_mul(Ak, B, &temp); // A^exp * B
			}

			if (i == j) // Set data of B in the diagonals
			{
				matf32_copy(B, &temp);
			}

			if (j > i) // Set a zero matrix in the upper triangle
			{
				matf32_zeros(&temp);
			}
		}
	}
}


err_status_t
ctr_mpc_set_Q(ctr_mpc_lti_shooting_t* mpc, matf32_t* const Q, matf32_t* const S)
{
	uint16_t N = mpc->N; // Horizon length

	float** const mpc_Q = mpc->mpc_Q;

	matf32_t temp;

	// Row-major indexing for assigning the values of Q
	for (uint16_t i = 0; i < N; ++i) // Rows
	{
		for (uint16_t j = 0; j < N; ++j) // Columns
		{
			matf32_init(&temp, Q->num_rows, Q->num_cols, mpc_Q[i*N + j]);

			if (i == j)
			{
				if (i+1 == N) // Assign S instead of Q for the last diagonal value
				{
					matf32_copy(S, &temp);
				}
				else // Assign Q in all other diagonals
				{
					matf32_copy(Q, &temp);
				}
			}
			else
			{
				matf32_zeros(&temp);
			}
		}
	}

	return MATH_SUCCESS;
}


err_status_t
ctr_mpc_set_R(ctr_mpc_lti_shooting_t* mpc, matf32_t* const R)
{
	uint16_t N = mpc->N; // Horizon length

	float** const mpc_R = mpc->mpc_R;

	matf32_t temp;

	// Row-major indexing for assigning the values of Q
	for (uint16_t i = 0; i < N; ++i) // Rows
	{
		for (uint16_t j = 0; j < N; ++j) // Columns
		{
			matf32_init(&temp, R->num_rows, R->num_cols, mpc_R[i*N + j]);

			if (i == j)
			{
				matf32_copy(R, &temp);
			}
			else
			{
				matf32_zeros(&temp);
			}
		}
	}

	return MATH_SUCCESS;
}


err_status_t
ctr_mpc_set_qpQ(ctr_mpc_lti_shooting_t* mpc, matf32_t* const qp_Q)
{
	uint16_t N = mpc->N; // Horizon length
	uint16_t n = mpc->sys->A->num_rows;
	uint16_t m = mpc->sys->B->num_cols;

	float** const mpc_Q = mpc->mpc_Q;
	float** const mpc_R = mpc->mpc_R;
	float** const mpc_C = mpc->mpc_C;
	float** const mpc_M = mpc->mpc_M;

	float CtQ_data[N*n*N*m];

	matf32_t* tmpmat1 = &m1;
	matf32_t* tmpmat2 = &m2;
	matf32_t* tmpmat3 = &m3;
	matf32_t* tmpmat4 = &m4;
	matf32_t* CtQ = &m5;
	
	// Generate Qqp = 2*((C' * Q * C) + R) (C=convolution matrix, Q=penalization matrix)

	matf32_init(tmpmat1, m, n, m1data);
	matf32_init(tmpmat2, n, n, m2data);
	matf32_init(tmpmat3, m, n, m3data);
	matf32_init(tmpmat4, m, n, m4data); // To sum the results for the element i,j
	matf32_init(CtQ, N*m, N*n, CtQ_data);

	// C' * Q
	uint16_t ij = 0;
	uint16_t current_col = 0;
	for (uint16_t i = 0; i < N; ++i) // Rows
	{
		for (uint16_t j = 0; j < N; ++j) // Columns
		{
			matf32_zeros(tmpmat4);
			for (uint16_t k = 0; k < N; ++k) // Elements in the respective row,col
			{
				tmpmat1->p_data = mpc_C[k*N + current_col]; // C'(k,j)
				tmpmat2->p_data = mpc_Q[k*N + j]; // Q(k,j)

				matf32_mul(tmpmat1, tmpmat2, tmpmat3); // C'(k,j) * Q(k,j)
				matf32_add(tmpmat4, tmpmat3, tmpmat4); // Add C'(i,k) * Q(i,k) to previous result
			}

			// Assign to the results matrix, element-by-element because the dimensions won't match the for loops counts
			for (uint16_t l = 0; l < tmpmat4->num_rows*tmpmat4->num_cols; ++l)
			{
				CtQ_data[ij] = tmpmat4->p_data[l];
				ij += m; // Increase element count
			}
		}
		current_col += 1;
	}

	matf32_init(tmpmat1, m, n, m1data);
	matf32_zeros(tmpmat1);
	matf32_init(tmpmat2, n, m, m2data);
	matf32_zeros(tmpmat2);
	matf32_init(tmpmat3, m, m, m3data);
	matf32_zeros(tmpmat3);
	matf32_init(tmpmat4, m, m, m4data);
	matf32_zeros(tmpmat4);

	// C' * Q * C
	ij = 0;
	current_col = 0;
	for (uint16_t i = 0; i < N; ++i) // Rows
	{
		for (uint16_t j = 0; j < N; ++j) // Columns
		{
			matf32_zeros(tmpmat4);
			for (uint16_t k = 0; k < N; ++k) // Elements in the respective row,col
			{
				matf32_submatrix_copy(CtQ, tmpmat1, i, current_col, 0, 0, m, n); // CtQ(i,current_col)
				tmpmat2->p_data = mpc_C[k*N + j]; // C(k,j)

				matf32_mul(tmpmat1, tmpmat2, tmpmat3); // CtQ(k,j) * C(k,j)
				matf32_add(tmpmat4, tmpmat3, tmpmat4); // Add CtQ(k,j) * C(k,j) to previous result

				current_col += n;
			}
			current_col = 0;

			// Assign to the results matrix, element-by-element because the dimensions won't match the for loops counts
			for (uint16_t l = 0; l < tmpmat4->num_rows*tmpmat4->num_cols; ++l)
			{
				qp_Q->p_data[ij] = tmpmat4->p_data[l];
				ij += m; // Increase element count
			}
		}
	}

	matf32_init(tmpmat1, m, m, m1data);
	matf32_zeros(tmpmat1);

	// (C' * Q * C) + R
	ij = 0;
	for (uint16_t i = 0; i < N; ++i) // Rows
	{
		for (uint16_t j = 0; j < N; ++j) // Columns
		{
			tmpmat1->p_data = mpc_R[i*N + j]; // R(i,j)
			
			for (uint16_t l = 0; l < tmpmat1->num_rows*tmpmat1->num_cols; ++l)
			{
				qp_Q->p_data[ij] += tmpmat1->p_data[l];
				ij += m; // Increase element count
			}
		}
	}

	// 2*((C' * Q * C) + R)
	matf32_scale(qp_Q, 2.0, qp_Q);
}


err_status_t
ctr_mpc_set_qpc(ctr_mpc_lti_shooting_t* mpc, matf32_t* const qp_c)
{
	uint16_t N = mpc->N; // Horizon length
	uint16_t n = mpc->sys->A->num_rows;
	uint16_t m = mpc->sys->B->num_cols;

	matf32_t* const x_k = mpc->x_k;

	float** const mpc_Q = mpc->mpc_Q;
	float** const mpc_C = mpc->mpc_C;
	float** const mpc_M = mpc->mpc_M;

	float CtQ_data[N*n*N*m];
	float F_data[N*m*n];

	matf32_t* tmpmat1 = &m1;
	matf32_t* tmpmat2 = &m2;
	matf32_t* tmpmat3 = &m3;
	matf32_t* tmpmat4 = &m4;
	matf32_t* CtQ = &m5;
	matf32_t* F = &m6;

	matf32_init(tmpmat1, m, n, m1data);
	matf32_init(tmpmat2, n, n, m2data);
	matf32_init(tmpmat3, m, n, m3data);
	matf32_init(tmpmat4, m, n, m4data); // To sum the results for the element i,j
	matf32_init(CtQ, N*m, N*n, CtQ_data);
	matf32_init(F, N*m, n, F_data);

	// C' * Q
	uint16_t ij = 0;
	uint16_t current_col = 0;
	for (uint16_t i = 0; i < N; ++i) // Rows
	{
		for (uint16_t j = 0; j < N; ++j) // Columns
		{
			matf32_zeros(tmpmat4);
			for (uint16_t k = 0; k < N; ++k) // Elements in the respective row,col
			{
				tmpmat1->p_data = mpc_C[k*N + current_col]; // C'(k,j)
				tmpmat2->p_data = mpc_Q[k*N + j]; // Q(k,j)

				matf32_mul(tmpmat1, tmpmat2, tmpmat3); // C'(k,j) * Q(k,j)
				matf32_add(tmpmat4, tmpmat3, tmpmat4); // Add C'(i,k) * Q(i,k) to previous result
			}

			// Assign to the results matrix, element-by-element because the dimensions won't match the for loops counts
			for (uint16_t l = 0; l < tmpmat4->num_rows*tmpmat4->num_cols; ++l)
			{
				CtQ_data[ij] = tmpmat4->p_data[l];
				ij += m; // Increase element count
			}
		}
		current_col += 1;
	}

	matf32_init(tmpmat1, m, n, m1data);
	matf32_zeros(tmpmat1);
	matf32_init(tmpmat2, n, n, m2data);
	matf32_zeros(tmpmat2);
	matf32_init(tmpmat3, m, n, m3data);
	matf32_zeros(tmpmat3);
	matf32_init(tmpmat4, m, n, m4data);
	matf32_zeros(tmpmat4);

	// C' * Q * M
	ij = 0;
	current_col = 0;
	uint16_t current_M = 0;
	for (uint16_t i = 0; i < N; ++i) // Rows
	{
		matf32_zeros(tmpmat4);
		for (uint16_t k = 0; k < N; ++k) // Elements in the respective row,col
		{
			matf32_submatrix_copy(CtQ, tmpmat1, i, current_col, 0, 0, m, n); // CtQ(i,current_col)
			tmpmat2->p_data = mpc_M[k]; // M(k,j)

			matf32_mul(tmpmat1, tmpmat2, tmpmat3); // CtQ(k,j) * C(k,j)
			matf32_add(tmpmat4, tmpmat3, tmpmat4); // Add CtQ(k,j) * C(k,j) to previous result

			current_col += n;
		}
		current_col = 0;
		current_M += 1;

		// Assign to the results matrix, element-by-element because the dimensions won't match the for loops counts
		for (uint16_t l = 0; l < tmpmat4->num_rows*tmpmat4->num_cols; ++l)
		{
			F_data[ij] = tmpmat4->p_data[l];
			ij += m; // Increase element count
		}
	}

	// c = 2*F*x_k
	matf32_mul(F, x_k, qp_c); // qp_c = F*x_k
	matf32_scale(qp_c, 2.0, qp_c); // qp_c = 2*F*x_k
}


err_status_t
ctr_mpc_set_constraints(ctr_mpc_lti_shooting_t* mpc, float ub, float lb, bool state_constraints)
{
	matf32_t* const A = mpc->sys->A;
	matf32_t* const B = mpc->sys->B;

	matf32_t* const u_k = mpc->u_k;

	matf32_t* const Ain = mpc->Ain;
	matf32_t* const bin = mpc->bin;

	uint16_t N = mpc->N; // Horizon length

	// Define dimensions for Ain and bin
	matf32_init(Ain, 2*N, 2*N, Ain->p_data);
	matf32_init(bin, 2*N, 1, bin->p_data);

	matf32_t* tmpmat1 = &m1;

	uint16_t row = 0;
	uint16_t diag = 0;

	if (state_constraints)
	{
		/* 	
			Ain = [A, 0 ... 0; 0, A ... 0; 0 0 -A... 0] -> A in the diagonals so that Ain is 2N x 2N
			bin = [lb - B*u_k; ub - B*u_k] -> Each vector is N x 1
		*/
		matf32_init(tmpmat1, A->num_rows, A->num_cols, m1data);
		matf32_zeros(tmpmat1);
		matf32_copy(A, tmpmat1); // tmpmat1 = A
		matf32_scale(tmpmat1, -1.0, tmpmat1); // tmpmat1 = -A

		// Assign first half of diagonals in Ain to be A
		for (uint16_t i = 0; i < N/2; ++i)
		{
			matf32_submatrix_copy(A, Ain, 0, 0, diag, diag, A->num_rows, A->num_cols);
			diag += A->num_rows;
		}

		// Assign second half of diagonals in Ain to be -A
		for (uint16_t i = 0; i < N/2; ++i)
		{
			matf32_submatrix_copy(tmpmat1, Ain, 0, 0, diag, diag, A->num_rows, A->num_cols);
			diag += A->num_rows;
		}

		// Calculate state constraints

		matf32_t* Bu_k = &m2;
		matf32_init(Bu_k, B->num_rows, u_k->num_cols, m2data);
		matf32_zeros(Bu_k);
		matf32_scale(B, u_k->p_data[0], Bu_k); // Bu_k = B*u_k
		printf("B*u_k:\n");
		matf32_print(Bu_k);
		printf("u_k[0]: %.9f\n", u_k->p_data[0]);

		matf32_t* x_ub = &m3;
		matf32_init(x_ub, B->num_rows, u_k->num_cols, m3data);
		matf32_t* x_lb = &m4;
		matf32_init(x_lb, B->num_rows, u_k->num_cols, m4data);

		matf32_set_col(x_ub, 0, ub);
		matf32_set_col(x_lb, 0, lb);

		matf32_sub(x_ub, Bu_k, x_ub);
		matf32_sub(x_lb, Bu_k, x_lb);

		printf("x_ub = ub:\n");
		matf32_print(x_ub);
		printf("x_lb = lb:\n");
		matf32_print(x_lb);

		row = 0;
		for (uint16_t i = 0; i < N/2; ++i)
		{
			matf32_submatrix_copy(x_ub, bin, 0, 0, row, 0, x_ub->num_rows, x_ub->num_cols);
			row += x_ub->num_rows;
		}

		for (uint16_t i = 0; i < N/2; ++i)
		{
			matf32_submatrix_copy(x_lb, bin, 0, 0, row, 0, x_lb->num_rows, x_lb->num_cols);
			row += x_lb->num_rows;
		}
	}
	else // Only constraints for the input vector u_k
	{
		/*
			Ain = [I, 0; 0, I] -> I in the diagonals so that Ain is 2N x 2N
			bin = [[ub]; [lb]] -> Each vector is N x 1
		*/

		matf32_init(tmpmat1, N, N, m1data);
		matf32_zeros(tmpmat1);
		matf32_eye(tmpmat1); // tmpmat1 = I
		matf32_submatrix_copy(tmpmat1, Ain, 0, 0, 0, 0, tmpmat1->num_rows, tmpmat1->num_cols);

		for (uint16_t i = N+1; i <= Ain->num_rows; ++i)
		{
			matf32_set(Ain, i, i, -1.0); // Set second half of Ain as -I
		}

		for (uint16_t i = 0; i < bin->num_rows/2; ++i)
		{
			matf32_set(bin, i+1, 1, ub); // Set upper bounds in the first half of bin
		}

		for (uint16_t i = N; i < bin->num_rows; ++i)
		{
			matf32_set(bin, i+1, 1, lb); // Set lower bounds in the second half of bin
		}
	}

	printf("Ain:\n");
	matf32_print(Ain);
	printf("bin:\n");
	matf32_print(bin);

	return MATH_SUCCESS;
}

err_status_t
ctr_mpc_update(ctr_mpc_lti_shooting_t* mpc, matf32_t* const qp_Q, matf32_t* const qp_c, matf32_t* const x_k, matf32_t* const u_k)
{
	matf32_t* const A = mpc->sys->A;
	matf32_t* const B = mpc->sys->B;
	matf32_t* const Ain = mpc->Ain;
	matf32_t* const bin = mpc->bin;

	// 1. Solve the QP: linsolve for unconstrained, quadprog_sqp (active-set) for inequality constrained
	if (Ain != NULL && bin != NULL)
	{
		quadprog_init(mpc->qp, qp_Q, qp_c, NULL, NULL, Ain, bin, u_k);
		quadprog_sqp(mpc->qp, u_k);
	}
	else
	{
		matf32_scale(qp_c, -1.0, qp_c);
		linsolve(qp_Q, qp_c, u_k); // Solve qp_Q * u_k = -qp_c
	}

	// 2. Update the value of x_k based on the new u_k
	matf32_t* Ax_k = &m1;
	matf32_init(Ax_k, A->num_rows, x_k->num_cols, m1data);
	matf32_zeros(Ax_k);
	matf32_mul(A, x_k, Ax_k); // A*x_k

	matf32_t* Bu_k = &m2;
	matf32_init(Bu_k, B->num_rows, u_k->num_cols, m2data);
	matf32_zeros(Bu_k);
	matf32_scale(B, u_k->p_data[0], Bu_k); // B*u_k(1), use only the first element in u_k

	matf32_add(Ax_k, Bu_k, x_k); // x_k = A*x_k + B*u_k(1)

	return MATH_SUCCESS;
}



// ====================================================================================================
// 5. Utility functions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 5.1. Printing functions
// ----------------------------------------------------------------------------------------------------

void 
ctr_pid_print(ctr_pid_t* const p_pid)
{
	printf("\n-------------------------\n");
	printf("PID\n");
	printf("-------------------------\n");

	printf("kp     : %.9f\n", p_pid->kp);
	printf("ki     : %.9f\n", p_pid->ki);
	printf("kd     : %.9f\n", p_pid->kd);
	printf("e_k_1  : %.9f\n", p_pid->e_k_1);
	printf("u_k_1  : %.9f\n", p_pid->u_k_1);
	printf("i_min  : %.9f\n", p_pid->i_min);
	printf("i_max  : %.9f\n", p_pid->i_max);
	printf("tau    : %.9f\n", p_pid->tau);
	printf("dt     : %.9f\n", p_pid->dt);
	
	printf("pid_alg: ");
	ctr_discretizations_print(p_pid->pid_alg);

	printf("-------------------------\n\n");
}

void
ctr_discretizations_print(ctr_discretizations_t pid_alg)
{
	switch (pid_alg)
	{
		case PURE_DISCRETE:
			printf("PURE_DISCRETE\n");
			break;

		case FWD_EULER:
			printf("FWD_EULER\n");
			break;

		case BWD_EULER:
			printf("BWD_EULER\n");
			break;

		case TUSTIN:
			printf("TUSTIN\n");
			break;

		case ZOH:
			printf("ZOH\n");
			break;

		case RK4:
			printf("RK4\n");
			break;
	}
}

void 
ctr_sys_lti_print(ctr_sys_lti_t* p_sys_lti)
{
	printf("\n-------------------------\n");
	printf("LTI State Space System\n");
	printf("-------------------------\n");

	matf32_print(p_sys_lti->state);
	
	printf("dt: %.9f\n", p_sys_lti->dt);
	printf("state_dim: %.9f\n", (float)p_sys_lti->state_dim);
	printf("input_dim: %.9f\n", (float)p_sys_lti->input_dim);
	printf("output_dim: %.9f\n", (float)p_sys_lti->output_dim);
	printf("is_continuous: %i\n\n", p_sys_lti->is_continuous);

	printf("A:\n");
	matf32_print(p_sys_lti->A);
	printf("B:\n");
	matf32_print(p_sys_lti->B);
	printf("C:\n");
	matf32_print(p_sys_lti->C);
	printf("D:\n");
	matf32_print(p_sys_lti->D);

	printf("-------------------------\n\n");
}

// Add a print for the nonlinear system	

void
ctr_kalman_print(ctr_kalman_t* p_kalman)
{
	printf("\n---------------------------------------------------------------------------\n");
	printf("Kalman Filter\n");
	printf("---------------------------------------------------------------------------\n");

	ctr_sys_lti_print(p_kalman->sys);

	printf("F:\n");
	matf32_print(p_kalman->F);
	printf("Qw:\n");
	matf32_print(p_kalman->Qw);
	printf("Qv:\n");
	matf32_print(p_kalman->Qv);
	printf("xhat:\n");
	matf32_print(p_kalman->xhat);
	printf("P:\n");
	matf32_print(p_kalman->P);

	printf("---------------------------------------------------------------------------\n\n");
}


//
