/** ============================================================================
 *  @file    fir_filter.cc
 *  @author  Matthew Pan (matthew.pan@queensu.ca)
 *  @date    June 30, 2023 
 *  @version 1.0 
 *  
 *  @brief Source file containing functions to implement an FIR filter
 *
 *  @section Source file to implement a finite impulse response (FIR) filter. 
 *  Recall that the output of a FIR filter is the weight sum of the N most 
 *  recent input values.
 * 
 *  y[n] = b_0*x[n] + b_1*x[n-1] + ... + b_N*x[n-N]
 * 
 *  where: 
 *  	- x[n] is the input signal
 * 		- y[n] is the output signal
 * 		- N is the filter order
 *  	- b_i is the value of the inpulse reponse at the ith instant for 0<=i<=N 
 * 		  of an Nth order filter 
 *  
 * ===========================================================================*/

#include "kinematics/fir_filter.h"

/// Function to initialize filter using an array of coeffs
FirFilter::FirFilter(int taps, double coeffs[]) {
	taps_ = taps; 
	coeffs_ = new double[taps_];
    std::copy(&coeffs[0], &coeffs[taps-1], coeffs_);
	std::fill(input_buffer_, input_buffer_ + BUFFER_SIZE, 0);
	std::fill(output_buffer_, output_buffer_ + BUFFER_SIZE, 0);
	offset_ = 0;
}

/// Function to initialize filter using a single repeated coeff
FirFilter::FirFilter(int taps, double coeff) {
	taps_ = taps; 
	coeffs_ = new double[taps_];
	for (int i = 0; i < taps; i++) 
		coeffs_[i] = coeff; 	
	ResetFilter();
}

FirFilter::~FirFilter() {
	delete[] coeffs_;
}

/// Function to set filter to different number of taps and coeffs
void FirFilter::SetFilter(int taps, double coeffs[]) {
	if(taps_ < taps) {
		double *old = coeffs_;
		coeffs_ = new double[taps];
		delete[] old;
	}
	taps_ = taps; 
	std::copy(&coeffs[0], &coeffs[taps-1], coeffs_);
}

/// Function to set filter to different number of taps and coeff
void FirFilter::SetFilter(int taps, double coeff) {
	if(taps_ < taps) {
		double *old = coeffs_;
		coeffs_ = new double[taps];
		delete[] old;
	}
	taps_ = taps; 
	for (int i = 0; i < taps; i++) 
		coeffs_[i] = coeff; 	
}

/// Resets buffers
void FirFilter::ResetFilter() {    
	std::fill(input_buffer_, input_buffer_ + BUFFER_SIZE, 0);
	std::fill(output_buffer_, output_buffer_ + BUFFER_SIZE, 0);
	offset_ = 0;
}

/// Function to filter incoming data using an input buffer
double FirFilter::Filter(double in)
{
	double *coeff     = coeffs_;
	double *coeff_end = coeffs_ + taps_;
	double *in_buf_val = input_buffer_ + offset_;
	double *out_buf_val = output_buffer_ + offset_;

	*in_buf_val = in;
	double output = 0;
		
	while (in_buf_val >= input_buffer_ && coeff < coeff_end)
		output += *in_buf_val-- * *coeff++;

	// to handle wraparound
	in_buf_val = input_buffer_ + BUFFER_SIZE-1;
	while(coeff < coeff_end)
		output += *in_buf_val-- * *coeff++;
	
	*out_buf_val = output;
	
	if(++offset_ >= BUFFER_SIZE)
		offset_ = 0;
	
	return output;
}
