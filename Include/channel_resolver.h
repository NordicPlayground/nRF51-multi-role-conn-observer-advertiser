#ifndef CHANNEL_RESOLVER_H__
#define CHANNEL_RESOLVER_H__

#include <stdint.h>

typedef enum
{
    ADV_CHANNEL_37 = 37,
    ADV_CHANNEL_38 = 38,
    ADV_CHANNEL_39 = 39
} adv_channel_t;

// Defines the order to cycle through adv channels.
// n_channels can be 1, 2, or 3. 
// If n_channels is 2, channel3 is ignored.
// If n_channels is 1, channel2 and channel3 are ignored.
typedef struct _hop_sequence
{
    adv_channel_t channels[3];        
    uint8_t n_channels;
} adv_channel_hop_sequence_t;

uint8_t channel_resolver_get_channel(void);
uint8_t channel_resolver_get_previous_channel(void);
uint8_t channel_resolver_get_next_channel(void);
uint8_t channel_resolver_get_frequency(uint8_t channel);
void channel_resolver_set_hop_sequence(adv_channel_hop_sequence_t* new_hop_sequence);
adv_channel_hop_sequence_t channel_resolver_get_hop_sequence(void);


#endif
