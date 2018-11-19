/* -----------------------------------------------------------------------------
 * pid
 * I-Grebot PID library
 * -----------------------------------------------------------------------------
 * File        : pid.c
 * Language    : C
 * Author      : Sebastien Brulais
 * Creation    : 2013-06-07
 * -----------------------------------------------------------------------------
 * Description :
 *   This is a PID control library that can be use for speed, position and more
 * -----------------------------------------------------------------------------
 * Dependency :
 * stdint.h
 * stdlib.h
 * math.h
 * -----------------------------------------------------------------------------
 * Library usage :
 * - User has to declare a PID process structure and configure it.
 * - Input and output function has to be set using PID_config_input and
 *   PID_config_output. It could be direct control of a motor or QEI peripheral
 *   but it also could be a sub-function such as another PID.
 * - Then the PID_Process() function should be called at constant rate, for 
 *   instance by a Timer
 * - Update reference position of the motor with PID_set_consign()
 * - Note that all the variables are "float"
 *
 * See ./example for more informations
 *
 * -----------------------------------------------------------------------------
 * Versionning informations
 * Repository: http://svn2.assembla.com/svn/paranoid_android/
 * -----------------------------------------------------------------------------
 * $Rev:$
 * $LastChangedBy:$
 * $LastChangedDate:$
 * -----------------------------------------------------------------------------
 * Version     Comment                                   Author       Date
 * 2.0		   Rewriting to make it generic				 Pierrick B. 2017-08-14
 * 1.2         Adding PID Process + Testing              Pierrick B. 2014-01-04
 * 1.1	       Separation speed/position update		 	 Pierrick B. 2013-12-11
 * 1.0         Initial release                           Seb B.      2013-06-07
 * -----------------------------------------------------------------------------
 */

#include "pid.h"
#include <stdlib.h>
#include <string.h>

/**
 *  signed maxmimum : both signs are tested
 */
#define S_MAX(to_saturate, value_max)    \
do {                                     \
   if (to_saturate > value_max)          \
     to_saturate = value_max;            \
   else if (to_saturate < -value_max)    \
     to_saturate = -value_max;           \
 } while(0)

static inline void
safe_set_output(void (*f)(void *, float), void * param, float value)
{
	void (*f_tmp)(void *, float);
	void * param_tmp;
	f_tmp = f;
	param_tmp = param;

	if (f_tmp)
	{
		f_tmp(param_tmp, value);
	}
}

static inline float
safe_get_input(float (*f)(void *), void * param)
{
	float (*f_tmp)(void *);
	void * param_tmp;
	f_tmp = f;
	param_tmp = param;

	if (f_tmp)
	{
		return f_tmp(param_tmp);
	}

	return 0;
}

static inline void
safe_filter_output(void (*f)(void *, float *), void * param, float * value)
{
	void (*f_tmp)(void *, float *);
	void * param_tmp;
	f_tmp = f;
	param_tmp = param;

	if (f_tmp)
	{
		f_tmp(param_tmp, value);
	}
}

void PID_do_process(PID_struct_t *PID){
    float output, error, derivate;
	uint8_t derivate_index;
	/* get input and calculate error */
    PID->input = (float)safe_get_input(PID->get_input,PID->input_channel);
    error = PID->consign - PID->input;

    /* Compute Proportional correction */
	output = error*PID->KP;

	/* Add Integral correction */
    if(PID->KI)
    {
        PID->integral += error;
        S_MAX(PID->integral,PID->I_limit);
        output += error*PID->KI;
    }

    /* Add Derivate correction */
    if(PID->KD)
    {
    	derivate_index = PID->prev_index + 1;
    	if (derivate_index >= PID->derivate_nb_samples)
    		derivate_index = 0;
    	derivate = error - PID->prev_error[derivate_index];
    	output += derivate*PID->KD/PID->derivate_nb_samples;
    }

    if(PID->output_filter)
    	safe_filter_output(PID->output_filter, PID->filter_param, &output);

    /* backup of current value (for the next calcul of derivate value) */
    PID->prev_error[PID->prev_index] = error ;
    PID->prev_index = derivate_index;

    safe_set_output(PID->set_output, PID->output_channel, output);
 }

void PID_set_coefficient(PID_struct_t *PID,float KP,float KI,float KD,float I_limit,uint8_t derivate_nb_samples){
    /* Set coefficients */
    PID->KP = KP;
    PID->KI = KI;
    PID->KD = KD;
    PID->I_limit = I_limit;
}

void PID_reset(PID_struct_t *xPID){
	memset(xPID, 0, sizeof(PID_struct_t));
    xPID->derivate_nb_samples = (uint8_t) 1;
	xPID->KP = (float) 1;
}

PID_struct_t* PID_init(void){
	/* Create the structure and set up */
	PID_struct_t *xPID;
    xPID =(PID_struct_t *) malloc(sizeof(PID_struct_t));
    PID_reset(xPID);
    return xPID;
}

void PID_config_output(PID_struct_t *xPID, void (*set_out)(void *, float), void *out_channel){
    xPID->set_output = set_out;
    xPID->output_channel = out_channel;
}

void PID_config_input(PID_struct_t *xPID, float (*get_in)(void *), void *in_channel)
{
    xPID->get_input = get_in;
    xPID->input_channel = in_channel;
}

void PID_config_filter(PID_struct_t *xPID, void (*out_filter)(void *, float *), void *filter_param)
{
	xPID->output_filter = out_filter;
	xPID->filter_param = filter_param;
}

void PID_set_consign(PID_struct_t *pPID, float consign) {
    pPID->consign = consign;
}

PID_filter_t* PID_filter_init(void)
{
	/* Create the structure and set up */
	PID_filter_t *xFilter = (PID_filter_t *) malloc(sizeof(PID_filter_t));
	memset(xFilter, 0, sizeof(PID_filter_t));
    return xFilter;
}

void PID_set_filter(PID_filter_t *xfilter, float out_max, float var_max, float var2_max)
{
	xfilter->output_max = out_max;
	xfilter->output_var_max = var_max;
	xfilter->output_var_2nd_order_max = var2_max;
}

void PID_rectangle_filter(PID_filter_t *pFilter, float *output)
{
	S_MAX(*output, pFilter->output_max);					// saturate if output over max
	pFilter->prev_out = *output;							// backup output
}

void PID_trapezium_filter(PID_filter_t *pFilter, float *output)
{
	float var = *output - pFilter->prev_out;				// compute output variation
	S_MAX(var, pFilter->output_var_max);					// saturate variation if over max
    *output = pFilter->prev_out + var;						// compute new output
	S_MAX(*output, pFilter->output_max);					// saturate if output over max
    pFilter->prev_var = *output - pFilter->prev_out;		// recompute and backup variation
	pFilter->prev_out = *output;							// backup output
}

void PID_S_curve_filter(PID_filter_t *pFilter, float *output)
{
	float var = (*output - pFilter->prev_out);				// compute output variation
	float var2 = var - pFilter->prev_var;					// compute output variation of variation
	float target = *output;
	S_MAX(var2, pFilter->output_var_2nd_order_max);			// saturate variation of variation if over max
	var = pFilter->prev_var + var2;							// compute new variation
	S_MAX(var, pFilter->output_var_max);					// saturate variation if over max
    *output = pFilter->prev_out + var;						// compute new output
    S_MAX(*output, pFilter->output_max);					// saturate if output over max
    var = *output - pFilter->prev_out;						// recompute and backup variation
    var2 = var - pFilter->prev_var;
    S_MAX(target, pFilter->output_max);
    if((target-(*output)) <= (var*var)/(2*var2))
    {
    	if(var > 0)											// defining the sign of variation
    	{
    		/* if positive then converging to 0 from top */
    		var = pFilter->prev_var - pFilter->output_var_2nd_order_max;
    		if(var < 0)
    		{
    			var = 0;
    			*output = target;
    		}
    		else
    			*output = pFilter->prev_out + var;
    	}
    	else
    	{
    		/* if negative then converging to 0 from bottom */
    		var = pFilter->prev_var + pFilter->output_var_2nd_order_max;	// compute new variation
    		if(var > 0)
    		{
    			var = 0;
    			*output = target;
    		}
    		else
    			*output = pFilter->prev_out + var;
    	}
    }
    pFilter->prev_var = var;								// backup variation
    pFilter->prev_out = *output;							// backup output
}
