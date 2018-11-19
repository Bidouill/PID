#ifndef _PID_H
#define _PID_H

#include <stdint.h>

#define PID_DERIVATE_FILTER_MAX_SIZE 5

/*
 * Structures definition
 * ---------------------
 */

typedef struct{
    float output_max;     								// output saturation
    float prev_out;										// previous output value
    float output_var_max; 								// output variation saturation
    float prev_var;										// previous variation value
    float output_var_2nd_order_max; 					// output variation of variation saturation
}PID_filter_t;

typedef struct{
    float KP;     										// Proportional gain
    float KI;       									// Integrative gain
    float KD;       									// Derivative gain
    float I_limit;										// Integral saturation level
    float integral;										// integral accumulated value
	uint8_t derivate_nb_samples;						// number of samples for derivate filter
	uint8_t prev_index; 								// index in circular buffer for derivate filter (below)
	float prev_error[PID_DERIVATE_FILTER_MAX_SIZE]; 	// previous error (circular buf)
    float consign;										// PID consign
    float input;										// PID input

	void (*set_output)(void *, float);					// pointer on output function
	void *output_channel;
	float (*get_input)(void *);						    // pointer on input function
    void * input_channel;
    void (*output_filter)(void *, float *);				// pointer on filter function
	void *filter_param;
}PID_struct_t;

/*
 * PID Functions Prototypes
 */
void PID_do_process(PID_struct_t *xPID);
void PID_set_coefficient(PID_struct_t *xPID,float KP,float KI,float KD,float I_limit,uint8_t derivate_nb_samples);
void PID_reset(PID_struct_t *xPID);
PID_struct_t* PID_init(void);
void PID_config_output(PID_struct_t *xPID, void (*set_out)(void *, float), void *out_channel);
void PID_config_input(PID_struct_t *xPID, float (*get_in)(void *), void *in_channel);
void PID_config_filter(PID_struct_t *xPID, void (*out_filter)(void *, float *), void *filter_param);
void PID_set_consign(PID_struct_t *xPID, float consign);
PID_filter_t* PID_filter_init(void);
void PID_set_filter(PID_filter_t *xfilter, float out_max, float var_max, float var2_max);
void PID_rectangle_filter(PID_filter_t *pFilter, float *output);
void PID_trapezium_filter(PID_filter_t *pFilter, float *output);
void PID_S_curve_filter(PID_filter_t *pFilter, float *output);
#endif /* ! _PID_H */
