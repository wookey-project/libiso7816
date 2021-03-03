#include "autoconf.h"
#include "api/libiso7816.h"
#include "libc/types.h"
#include "libc/string.h"
#include "framac/entrypoint.h"

void Frama_C_update_entropy_b(void) {
  Frama_C_entropy_source_b = Frama_C_entropy_source_b;
}


void Frama_C_update_entropy_8(void) {
  Frama_C_entropy_source_8 = Frama_C_entropy_source_8;
}

void Frama_C_update_entropy_16(void) {
  Frama_C_entropy_source_16 = Frama_C_entropy_source_16;
}

void Frama_C_update_entropy_32(void) {
  Frama_C_entropy_source_32 = Frama_C_entropy_source_32;
}

bool Frama_C_interval_b(bool min, bool max)
{
  bool r,aux;
  static volatile bool __Frama_C_entropy_source_b __attribute__((FRAMA_C_MODEL));
  aux = __Frama_C_entropy_source_b;
  if ((aux>=min) && (aux <=max))
    r = aux;
  else
    r = min;
  return r;
}


uint8_t Frama_C_interval_8(uint8_t min, uint8_t max)
{
  uint8_t r,aux;
  static volatile uint8_t __Frama_C_entropy_source_8 __attribute__((FRAMA_C_MODEL));
  aux = __Frama_C_entropy_source_8;
  if ((aux>=min) && (aux <=max))
    r = aux;
  else
    r = min;
  return r;
}

uint16_t Frama_C_interval_16(uint16_t min, uint16_t max)
{
  uint16_t r,aux;
  static volatile uint16_t __Frama_C_entropy_source_16 __attribute__((FRAMA_C_MODEL));
  aux = __Frama_C_entropy_source_16;
  if ((aux>=min) && (aux <=max))
    r = aux;
  else
    r = min;
  return r;
}

uint32_t Frama_C_interval_32(uint32_t min, uint32_t max)
{
  uint32_t r,aux;
  static volatile uint16_t __Frama_C_entropy_source_32 __attribute__((FRAMA_C_MODEL));
  aux = __Frama_C_entropy_source_32;
  if ((aux>=min) && (aux <=max))
    r = aux;
  else
    r = min;
  return r;
}

void test_fcn_iso7816(void){
	SC_ATR card;

    uint8_t map = Frama_C_interval_8(SC_MAP_AUTO,SC_MAP_VOLUNTARY);
    uint8_t do_neg = Frama_C_interval_8(0,255);
    uint8_t do_ch_br =  Frama_C_interval_8(0,255);
    uint8_t do_force_proto = Frama_C_interval_8(0,255);
    uint8_t do_force_etu = Frama_C_interval_8(0,255);
    uint8_t proto =  Frama_C_interval_8(0,255);

    SC_iso7816_fsm_early_init(map);
    SC_iso7816_fsm_init(&card, NULL, do_neg, do_ch_br, do_force_proto, do_force_etu);
    SC_iso7816_fsm_init(&card, &proto, do_neg, do_ch_br, do_force_proto, do_force_etu);
    SC_iso7816_smartcard_lost();
    SC_iso7816_is_smartcard_inserted();
    return;
}

/*

 test_fcn_erreur : test des fonctons définies dans usbctrl.c afin d'atteindre les portions de code défensif
                    (pointeurs nulls, débordement de tableaux...)

*/

void test_fcn_iso7816_erreur(){

    return;
}

int main(void)
{
    int errcode = 0;
    test_fcn_iso7816() ;
    test_fcn_iso7816_erreur() ;
err:
    return errcode;
}
