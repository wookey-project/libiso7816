#ifndef ENTRY_H_
#define ENTRY_H_

#include "libc/types.h"


extern volatile bool Frama_C_entropy_source_b __attribute__((unused)) __attribute__((FRAMA_C_MODEL));
extern volatile uint8_t Frama_C_entropy_source_8 __attribute__((unused)) __attribute__((FRAMA_C_MODEL));
extern volatile uint16_t Frama_C_entropy_source_16 __attribute__((unused)) __attribute__((FRAMA_C_MODEL));
extern volatile uint32_t Frama_C_entropy_source_32 __attribute__((unused)) __attribute__((FRAMA_C_MODEL));


/*@ requires order: 0 <= min <= max <= 1;
    assigns \nothing;
    ensures result_bounded: min <= \result <= max ;
 */
bool Frama_C_interval_b(bool min, bool max);



/*@ requires order: 0 <= min <= max <= 255;
    assigns \nothing;
    ensures result_bounded: min <= \result <= max ;
 */
uint8_t Frama_C_interval_8(uint8_t min, uint8_t max);

/*@ requires order: 0 <= min <= max <= 65535;
    assigns \nothing;
    ensures result_bounded: min <= \result <= max ;
 */
uint16_t Frama_C_interval_16(uint16_t min, uint16_t max);

/*@ requires order: 0 <= min <= max <= 4294967295;
    assigns \nothing;
    ensures result_bounded: min <= \result <= max ;
 */
uint32_t Frama_C_interval_32(uint32_t min, uint32_t max);


//@ assigns Frama_C_entropy_source_b \from Frama_C_entropy_source_b;
void Frama_C_update_entropy_b(void);


//@ assigns Frama_C_entropy_source_8 \from Frama_C_entropy_source_8;
void Frama_C_update_entropy_8(void);


//@ assigns Frama_C_entropy_source_16 \from Frama_C_entropy_source_16;
void Frama_C_update_entropy_16(void);

//@ assigns Frama_C_entropy_source_32 \from Frama_C_entropy_source_32;
void Frama_C_update_entropy_32(void);


#endif/*!ENTRY_H_*/
