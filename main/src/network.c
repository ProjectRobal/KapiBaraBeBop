#include "network.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <esp_random.h>
#include <esp_dsp.h>


float gauss (float mu, float stdev)
{
  float U1, U2, W, mult;
  static float X1, X2;
  static int call = 0;
 
  if (call == 1)
    {
      call = !call;
      return (mu + stdev * (float) X2);
    }
 
  do
    {
      U1 = -1 + ((float) esp_random () / RAND_MAX) * 2;
      U2 = -1 + ((float) esp_random () / RAND_MAX) * 2;
      W = pow (U1, 2) + pow (U2, 2);
    }
  while (W >= 1 || W == 0);
 
  mult = sqrt ((-2 * log (W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;
 
  call = !call;
 
  return (mu + stdev * (float) X1);
}


void linear(float* x,size_t size)
{

}

void relu(float* x,size_t size)
{
    for(size_t i=0;i<size;++i)
    {
        x[i]=x[i]*(x[i]>=0.f);
    }
}

void sigmoid(float* x,size_t size)
{
    for(size_t i=0;i<size;++i)
    {
        x[i]=1.f/(1.f + exp(-x[i]));
    }
}


// generate neuron with random weights and bias
neuron_t new_neuron(size_t size)
{
    neuron_t neuron;
    neuron.size=size;

    neuron.input_weights=malloc(sizeof(float)*size);

    for(size_t i=0;i<size;++i)
    {
        neuron.input_weights[i]=gauss(0.f,0.01);
    }

    neuron.bias=gauss(0.f,0.01);

    return neuron;
}

void free_neuron(neuron_t* neuron)
{
    free(neuron->input_weights);
}

// create a new layer with neurons with random weights, amount of size
layer_t new_layer(size_t size,size_t input_size,activation_func activation)
{
    layer_t layer;
    layer.size=size;

    layer.neurons=(neuron_t*)calloc(size,sizeof(neuron_t));

    for(size_t i=0;i<size;++i)
    {
        layer.neurons[i] = new_neuron(input_size);
    }
    
    layer.filter=activation;

    layer.next=NULL;

    return layer;
}

void free_layer(layer_t* layer)
{
    for(size_t i=0;i<layer->size;++i)
    {
        free_neuron(&layer->neurons[i]);
    }

    free(layer->neurons);
}

void free_network(layer_t* layer)
{
    while(layer != NULL)
    {
        free_layer(layer);

        layer=layer->next;
    }
}

void append_layer(network_t* network,layer_t* layer)
{
    if(network->layers == NULL)
    {
        network->layers=layer;

        return;
    }

    layer_t* ptr=network->layers;

    while(ptr->next != NULL)
    {
        ptr=ptr->next;
    }

    ptr->next=layer;

}

float fire_neuron(neuron_t* neuron,float* inputs)
{
    float result=0;

    dsps_dotprod_f32(neuron->input_weights,inputs,&result,neuron->size);

    return result+neuron->bias;
}

void fire_layer(layer_t* layer,float* inputs,float* outputs)
{
    for(size_t i=0;i<layer->size;++i)
    {
        outputs[i] = fire_neuron(&layer->neurons[i],inputs);
    }

    (*layer->filter)(outputs,layer->size);
}

// that should be optimised
void fire_network(layer_t* networks,float *inputs,float *output)
{
    float *last_output=inputs;

    layer_t* last_netowrk=NULL;

    do
    {
        if(last_output != inputs){ free(last_output); }

        float *_output=malloc(networks->size*sizeof(float));

        fire_layer(networks,last_output,_output);

        last_output=_output;

        last_netowrk = networks;
        networks = networks->next;

    }while(networks != NULL);


    memcpy(output,last_output,sizeof(float)*last_netowrk->size);

}