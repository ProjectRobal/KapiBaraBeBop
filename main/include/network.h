#pragma once

#include <stdint.h>
#include <stddef.h>

#define RAND_FLOAT_MAX UINT32_MAX/100.f


typedef void(*activation_func)(float*,size_t);

typedef struct neuron
{
    size_t size;
    float bias;
    float* input_weights;
} neuron_t;

typedef struct layer
{
    size_t size;
    neuron_t* neurons;
    activation_func filter;
    // pointer for next layer
    struct layer* next;
}layer_t;


typedef struct network
{
    layer_t* layers;
}network_t;


// gauss distribution

float gauss(float mu, float stdev);

// activation functions

void linear(float* x,size_t size);

void relu(float* x,size_t size);

void sigmoid(float* x,size_t size);

// generate neuron with random weights and bias
neuron_t new_neuron(size_t size);

void free_neuron(neuron_t* neuron);

// create a new layer with neurons with random weights, amount of size
layer_t new_layer(size_t size,size_t input_size,activation_func activation);

void free_layer(layer_t* layer);

void free_network(layer_t* layer);

void append_layer(network_t* network,layer_t* layer);

void fire_network(layer_t* networks,float *inputs,float *output);

float fire_neuron(neuron_t* neuron,float* inputs);

void fire_layer(layer_t* layer,float* inputs,float* outputs);

