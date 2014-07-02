/* Copyright (c) 2014, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   * Neither the name of Nordic Semiconductor ASA nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "channel_resolver.h"
#include "nrf_gpio.h"

static adv_channel_hop_sequence_t hop_sequence = {{ADV_CHANNEL_37, ADV_CHANNEL_38, ADV_CHANNEL_39}, 3};

/* Returns the logical channel corresponding to the current radio frequency */
uint8_t channel_resolver_get_channel(void)
{
    uint8_t freq = NRF_RADIO->FREQUENCY;
    
    uint8_t channel;
    /* Special cases for the advertise channels */
    if(freq == 2)
        channel = 37;
    else if(freq == 26)
        channel = 38;
    else if(freq == 80)
        channel = 39;
    /* Data channels */
    else 
        channel = (freq / 2 ) - (freq < 26 ? 2 : 3); // Spec Vol. 6, Part B, 1.4.1
    
    return channel;
}

int get_index_in_hop_sequence(adv_channel_t channel)
{
    int i;
    for (i = 0; (i < hop_sequence.n_channels) && (channel != hop_sequence.channels[i]); i++)
    {}
    return i;
}

uint8_t channel_resolver_get_previous_channel(void)
{
    adv_channel_t current_channel = (adv_channel_t)channel_resolver_get_channel();
    int current_index = get_index_in_hop_sequence(current_channel);
    int prev_index = (current_index + hop_sequence.n_channels - 1) % hop_sequence.n_channels;
    return (uint8_t)hop_sequence.channels[prev_index];
}

uint8_t channel_resolver_get_next_channel(void)
{
    adv_channel_t current_channel = (adv_channel_t)channel_resolver_get_channel();
    int current_index = get_index_in_hop_sequence(current_channel);
    int next_index = (current_index + 1) % hop_sequence.n_channels;
    return (uint8_t)hop_sequence.channels[next_index];
}

/* Does NOT work for ADV channels */
uint8_t channel_resolver_get_frequency(uint8_t channel)
{
	uint8_t freq;
	
	/* Special cases for the advertise channels */
	if(channel == 37)
		freq = 2;
	else if(channel == 38)
		freq = 26;
	else if(channel == 39)
		freq = 80;
	else
		freq = channel + (channel < 11 ? 2 : 3) * 2; // Spec Vol. 6, Part B, 1.4.1

	return freq;
}

void channel_resolver_set_hop_sequence(adv_channel_hop_sequence_t* new_hop_sequence)
{
    hop_sequence = *new_hop_sequence;
}

adv_channel_hop_sequence_t channel_resolver_get_hop_sequence(void)
{
    return hop_sequence;
}
