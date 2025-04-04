/*==========================================================
 * filter.cpp
 *
 * Source file containing functions to implement an IIR filter
 * 
 *           b0 + b1 * z^-1 + b2 * z^-2 + ... + bn * z^-n
 *  H(z) =  -----------------------------------------------
 *           1  + a1 * z^-1 + a2 * z^-2 + ... + an * z^-n
 * 
 * Created by: Vinay Chawda
 *             vinay.chawda@disneyresearch.com
 *
 * Date created: 03/18/2016
 *
 * Last modified: 03/18/2016
 *========================================================*/

#include "kinematics/iir_filter.h"

IirFilter::IirFilter(double *b, double *a) {
    // Function to initialize the filter
    // Filter coefficients b and a are expected to be pre-allocated arrays of length FILTER_ORDER+1 
    for(int i = 0; i < FILTER_ORDER+1; i++) {
        b_[i] = b[i];
        a_[i] = a[i];
        input_buffer_[i] = 0;
        output_buffer_[i] = 0;
    }
}

IirFilter::IirFilter() {
    // Function to initialize the filter
    // Filter coefficients b and a are expected to be pre-allocated arrays of length FILTER_ORDER+1 
    for(int i = 0; i < FILTER_ORDER+1; i++) {
        b_[i] = 0;
        a_[i] = 0;
        input_buffer_[i] = 0;
        output_buffer_[i] = 0;
    }
}

IirFilter::~IirFilter() {
    
}

void IirFilter::ResetFilter() {
    // Function to reset a filter
    for(int i = 0; i < FILTER_ORDER+1; i++) {
        input_buffer_[i] = 0;
        output_buffer_[i] = 0;
    }
}

void IirFilter::SetFilter(double *b, double *a) {
    // Function to set the filter
    // Filter coefficients b and a are expected to be pre-allocated arrays of length FILTER_ORDER+1 
    for(int i = 0; i < FILTER_ORDER+1; i++) {
        b_[i] = b[i];
        a_[i] = a[i];
        input_buffer_[i] = 0;
        output_buffer_[i] = 0;
    }
}

double IirFilter::Filter(double in)  {
    // Function to implement the IIR filter
    
    for(int i = FILTER_ORDER; i > 0 ; i--) {
        input_buffer_[i] = input_buffer_[i-1];
        output_buffer_[i] = output_buffer_[i-1];
    }
    input_buffer_[0] = in;
    output_buffer_[0] = b_[0] * input_buffer_[0];

    for(int i = 1; i < FILTER_ORDER + 1; i++) {
        output_buffer_[0] += b_[i] * input_buffer_[i] - a_[i] * output_buffer_[i];
    }

    return(output_buffer_[0]);
}