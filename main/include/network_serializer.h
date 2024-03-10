#pragma once

#include "network.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

size_t neuron_size(neuron_t* neuron)
{
    return sizeof(neuron->size)+sizeof(neuron->bias)+(sizeof(float)*neuron->size);
}

uint8_t * serialize_neuron(neuron_t* neuron)
{
    uint8_t* output=malloc(neuron_size(neuron));

    memmove(output,(uint8_t*)&neuron->size,sizeof(neuron->size));
    memmove(output+sizeof(neuron->size),(uint8_t*)&neuron->bias,sizeof(neuron->bias));

    memmove(output+sizeof(neuron->size)+sizeof(neuron->bias),(uint8_t*)neuron->input_weights,sizeof(float)*neuron->size);

    return output;
}

size_t layer_size(layer_t* layer)
{
    return sizeof(layer->size);
}

uint8_t* serialize_layer(layer_t* layer)
{
    uint8_t* output=malloc(layer_size(layer));

    memmove(output,(uint8_t*)&layer->size,sizeof(layer->size));

    return output;
}

void save_layer_to_file(FILE* file,layer_t* layer)
{
    // write header
    fwrite("#21",1,3,file);

    uint8_t* buffer=serialize_layer(layer);

    fwrite(buffer,1,layer_size(layer),file);

    free(buffer);

    for(size_t i=0;i<layer->size;++i)
    {
        buffer=serialize_neuron(&layer->neurons[i]);

        fwrite(buffer,1,neuron_size(&layer->neurons[i]),file);

        free(buffer);
    }


}

bool load_layer_from_file(FILE* file,layer_t* layer)
{
    char header[3]={0};

    fread(header,1,3,file);

    if(strncmp(header,"#21",3)!=0)
    {
        ESP_LOGD("NETWORK","Broken header in layer file!");
        return false;
    }

    char buffer[sizeof(layer->size)];

    fread(buffer,1,sizeof(layer->size),file);

    memmove((uint8_t*)&layer->size,buffer,sizeof(size_t));

    layer->neurons=(neuron_t*)calloc(layer->size,sizeof(neuron_t));

    // deserialize neurons

    char size_buffer[sizeof(size_t)];
    char bias_buffer[sizeof(float)];

    for(size_t i=0;i<layer->size;++i)
    {
        neuron_t neuron;

        fread(size_buffer,1,sizeof(size_t),file);
        fread(bias_buffer,1,sizeof(float),file);

        memmove((uint8_t*)&neuron.size,size_buffer,sizeof(size_t));
        memmove((uint8_t*)&neuron.bias,bias_buffer,sizeof(float));

        neuron.input_weights=(float*)calloc(neuron.size,sizeof(float));

        fread((uint8_t*)neuron.input_weights,sizeof(float),neuron.size,file);

        layer->neurons[i]=neuron;
    }

    return true;
}