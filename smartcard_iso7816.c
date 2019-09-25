#include "libc/stdio.h"
#include "libc/nostd.h"
#include "libc/string.h"
#include "libc/syscall.h"
#include "libdrviso7816.h"
#include "api/libiso7816.h"

static volatile uint32_t SC_current_sc_frequency = SMARTCARD_DEFAULT_CLK_FREQ;
/* Get the current smartcard configured frequency */
static uint32_t SC_get_sc_clock_freq(void){
        return SC_current_sc_frequency;
}

static volatile uint32_t SC_current_sc_etu = SMARTCARD_DEFAULT_ETU;
/* Get the current smartcard configured ETU */
static uint32_t SC_get_sc_etu(void){
        return SC_current_sc_etu;
}

/* Fixed delay of a given number of clock cycles */
static void SC_delay_sc_clock_cycles(uint32_t sc_clock_cycles_timeout){
        uint64_t t, start_tick, curr_tick;
	uint32_t clock_freq = SC_get_sc_clock_freq();

	if(sc_clock_cycles_timeout == 0){
		return;
	}

        /* The timeout is in smartcard clock cycles, which can be converted to MCU clock time
         * using a simple conversion. The clock time is expressed in microseconds.
         */
	if(clock_freq == 0){
		/* Avoid division by zero */
		return;
	}
        t = (sc_clock_cycles_timeout * 1000000ULL) / clock_freq;
        start_tick = platform_get_microseconds_ticks();
        /* Now wait */
        curr_tick = start_tick;
        while((curr_tick - start_tick) <= t){
                curr_tick = platform_get_microseconds_ticks();
        }

        return;
}

/* Fixed delay of a given number of ETUs */
static void SC_delay_etu(uint32_t etu_timeout){

        SC_delay_sc_clock_cycles(etu_timeout * SC_get_sc_etu());

        return;
}

typedef enum {
	SC_TS_DIRECT_CONVENTION = 0,
	SC_TS_INVERSE_CONVENTION = 1,
} Sc_convention;
static volatile uint8_t SC_convention = SC_TS_DIRECT_CONVENTION;

static uint8_t SC_is_inverse_conv(void){
	if(SC_convention == SC_TS_DIRECT_CONVENTION){
		return 0;
	}
	else{
		return 1;
	}
}

static int SC_set_direct_conv(void){
	/* Set the inverse convention */
	SC_convention = SC_TS_DIRECT_CONVENTION;

	return platform_SC_set_direct_conv();
}

static int SC_set_inverse_conv(void){
	/* Set the inverse convention */
	SC_convention = SC_TS_INVERSE_CONVENTION;

	return platform_SC_set_inverse_conv();
}

static uint8_t SC_inverse_conv(uint8_t c){
	unsigned int i;
	uint8_t res = 0;

	for(i = 0; i < 8; i++){
		res |= (((~c) >> i) & 0x1) << (7-i);
	}

	return res;
}

/* Get a byte with a timeout. Timeout of 0 means forever. */
static int SC_getc_timeout(uint8_t *c, uint32_t etu_timeout){
        uint64_t t, start_tick, curr_tick;
	uint32_t clock_freq = SC_get_sc_clock_freq();

        /* The timeout is in ETU times, which can be converted to sys ticks
         * using the baud rate. The sys ticks are expressed in microseconds.
         */
	if(clock_freq == 0){
		/* Avoid division by zero */
		return -1;
	}
        t = (etu_timeout * SC_get_sc_etu() * 1000000ULL) / clock_freq;
        start_tick = platform_get_microseconds_ticks();

        while(platform_SC_getc(c, t, 0)){
                if(etu_timeout != 0){
                        curr_tick = platform_get_microseconds_ticks();
                        /* Check the timeout and return if time is elapsed */
                        if((curr_tick - start_tick) > t){
				/* Reset our getc state machine in the lower layers ... */
				platform_SC_getc(c, t, 1);
                                return -1;
                        }
                }
        }

	if(SC_is_inverse_conv()){
		*c = SC_inverse_conv(*c);
	}

        return 0;
}



static int SC_putc_timeout(uint8_t c, uint32_t etu_timeout){
        uint64_t t, start_tick, curr_tick;
	uint32_t clock_freq = SC_get_sc_clock_freq();

	if(SC_is_inverse_conv()){
		c = SC_inverse_conv(c);
	}

	/* The timeout is in ETU times, which can be converted to sys ticks
         * using the baud rate. The sys ticks are expressed in microseconds.
         */
	if(clock_freq == 0){
		/* Avoid division by zero */
		return -1;
	}
        t = (etu_timeout * SC_get_sc_etu() * 1000000ULL) / clock_freq;
        start_tick = platform_get_microseconds_ticks();

        while(platform_SC_putc(c, t, 0)){
                if(etu_timeout != 0){
                        curr_tick = platform_get_microseconds_ticks();
                        /* Check the timeout and return if time is elapsed */
                        if((curr_tick - start_tick) > t){
				/* Reset our putc state machine in the lower layers ... */
				platform_SC_putc(c, t, 1);
                                return -1;
                        }
                }
        }

	/* Wait for the character extra guard time after the stop bit */
	SC_delay_etu(CGT_character_guard_time);

	return 0;
}

static int SC_adapt_clocks(uint32_t etu, uint32_t frequency){
	uint32_t tmp_etu = etu;
	uint32_t tmp_frequency = frequency;

	if(platform_SC_adapt_clocks(&tmp_etu, &tmp_frequency)){
		return -1;
	}
	else{
		SC_current_sc_frequency = tmp_frequency;
		SC_current_sc_etu = tmp_etu;

		return 0;
	}
}

/* Get the ATR from the card */
int SC_get_ATR(SC_ATR *atr){
	unsigned int i;
	uint8_t curr_mask, checksum, do_check_checksum = 0;

	if(atr == NULL){
		goto err;
	}

	/* Default values per standard */
	atr->D_i_curr = 1;
	atr->F_i_curr = SMARTCARD_DEFAULT_ETU;
	atr->f_max_curr = SMARTCARD_DEFAULT_CLK_FREQ;
	atr->T_protocol_curr = 0;
	atr->ifsc = 0;
	for(i = 0; i < SETUP_LENGTH; i++){
		atr->t_mask[i] = 0;
	}
	atr->h_num = 0;
	/* Wait for the ATR reception with a timeout
	 * of 20160 ETU (20160 bauds).
	 */
	/******************************/
	if(SC_getc_timeout(&(atr->ts), ATR_ETU_TIMEOUT)){
		goto err;
	}
	/* First byte of the ATR has been received */
	if(atr->ts == 0x3b){
		if(SC_set_direct_conv()){
			goto err;
		}
	}
	else if(atr->ts == 0x03){
		if(SC_set_inverse_conv()){
			goto err;
		}
		atr->ts = SC_inverse_conv(atr->ts);
	}
	else{
		goto err;
	}
	/* Get the format byte T0 */
	if(SC_getc_timeout(&(atr->t0), WT_wait_time)){
		goto err;
	}
	checksum = atr->t0;
	/* Get the interface bytes */
	curr_mask = (atr->t0) >> 4;
	for(i = 0; i < SETUP_LENGTH; i++){
		if(curr_mask & 0x1){
		        if(SC_getc_timeout(&(atr->ta[i]), WT_wait_time)){
		                goto err;
       			}
			atr->t_mask[0] |= (0x1 << i);
			checksum ^= atr->ta[i];
		}
		if(curr_mask & 0x2){
		        if(SC_getc_timeout(&(atr->tb[i]), WT_wait_time)){
		                goto err;
       			}
			atr->t_mask[1] |= (0x1 << i);
			checksum ^= atr->tb[i];
		}
		if(curr_mask & 0x4){
		        if(SC_getc_timeout(&(atr->tc[i]), WT_wait_time)){
		                goto err;
       			}
			atr->t_mask[2] |= (0x1 << i);
			checksum ^= atr->tc[i];
		}
		if(curr_mask & 0x8){
		        if(SC_getc_timeout(&(atr->td[i]), WT_wait_time)){
		                goto err;
       			}
			atr->t_mask[3] |= (0x1 << i);
			checksum ^= atr->td[i];
			curr_mask = atr->td[i] >> 4;
			if((atr->td[i] & 0x0f) != 0){
				do_check_checksum = 1;
			}
		}
		else{
			break;
		}
	}
	/* Get the historical bytes */
	atr->h_num = atr->t0 & 0x0f;
	for(i = 0; i < atr->h_num; i++){
	        if(SC_getc_timeout(&(atr->h[i]), WT_wait_time)){
	                goto err;
		}
		checksum ^= atr->h[i];
	}
	atr->tck = 0;
	atr->tck_present = 0;
	/* The checksum TCK is present if and only if any of the TDi encode a protocol != 0 */
	if(do_check_checksum){
		atr->tck_present = 1;
		/* Get the checksum */
		if(SC_getc_timeout(&(atr->tck), WT_wait_time)){
			goto err;
		}
		/* Check the checksum */
		if(checksum != atr->tck){
			log_printf("Smartcard ATR checksum error ...\n");
			goto err;
		}
	}

	return 0;
err:
	return -1;
}

/* Print the ATR on the console */
void SC_iso7816_print_ATR(SC_ATR *atr)
{
	unsigned int i;

	if(atr == NULL){
		return;
	}

	log_printf("===== ATR ============\n");
	log_printf("TS = %x, T0 = %x\n", atr->ts, atr->t0);
	for(i = 0; i < 4; i++){
		if(atr->t_mask[0] & (0x1 << i)){
			log_printf("TA[%d] = %x\n", i, atr->ta[i]);
		}
	}
	for(i = 0; i < 4; i++){
		if(atr->t_mask[1] & (0x1 << i)){
			log_printf("TB[%d] = %x\n", i, atr->tb[i]);
		}
	}
	for(i = 0; i < 4; i++){
		if(atr->t_mask[2] & (0x1 << i)){
			log_printf("TC[%d] = %x\n", i, atr->tc[i]);
		}
	}
	for(i = 0; i < 4; i++){
		if(atr->t_mask[3] & (0x1 << i)){
			log_printf("TD[%d] = %x\n", i, atr->td[i]);
		}
	}
	for(i = 0; i < (atr->h_num & 0x0f); i++){
		log_printf("H[%d] = %x\n", i, atr->h[i]);
	}
	if(atr->tck_present){
		log_printf("TCK = %x\n", atr->tck);
	}
        log_printf("F=%d, D=%d, fmax=%d\n", atr->F_i_curr, atr->D_i_curr, atr->f_max_curr);

	return;
}

/* Interval in ETU between the last received bye from the card and the next byte sent by the terminal */
uint32_t GT_receive_send_interval_etu = 0;
/* [RB] FIXME: for now, the PTS/PSS negotation is a work in
 * progress and has not been exhaustively tested/debugged.
 */
/* PTS (Protocol Type Selection) negotiation.
 * Returns the protocol that has been negotiated.
 */
static int SC_negotiate_PTS(SC_ATR *atr, uint8_t *T_protocol, uint8_t do_negotiate_pts, uint8_t do_change_baud_rate, uint8_t do_force_protocol, uint32_t do_force_etu){
	uint8_t ta1, td1;
	uint32_t etu_curr = 0;
	uint8_t negotiable_mode = 1;
	uint8_t extra_guard_time = 1;
	uint8_t asked_ta1 = 0, asked_tc1 = 0;
	uint8_t pck;
	uint8_t c;

	if((atr == NULL) || (T_protocol == NULL)){
		goto err;
	}

	/* Check TA2 to see if we cannot negotiate */
	if(atr->t_mask[0] & (0x1 << 1)){
		/* The card supports specific mode */
		log_printf("[Smartcard] TA2 present, implies specific mode\n");
		negotiable_mode = 0;
	}
	else{
		/* The card supports negotiable mode */
		log_printf("[Smartcard] TA2 absent, implies negotiable mode\n");
		negotiable_mode = 1;
	}

	/* Do we have TA1? If not, assume it is 0x11 (Fi = 372 and Di = 1) */
	if(atr->t_mask[0] & 0x1){
		ta1 = atr->ta[0];
		asked_ta1 = 1;
	}
	else{
		ta1 = 0x11;
	}

	/* T=0 is the default protocol if nothing is specified by the card */
	*T_protocol = 0;
	/* Do we have TD1? If yes, this is the preferred protocol. Use it, unless we have been overriden with
	 * a specific choice.
	 */
	if((atr->t_mask[3] & 0x1) && (do_force_protocol == 0)){
		td1 = atr->td[0];
		*T_protocol = td1 & 0x0f;
		log_printf("[Smartcard] ATR prefered protocol T=%d\n", *T_protocol);
		if((*T_protocol != 0) && (*T_protocol != 1)){
			/* We do not support T=15 or protocols other than T=0/T=1 */
			log_printf("[Smartcard] Asking for unsupported protocol T=%d\n", *T_protocol);
			goto err;
		}
	}
	if(do_force_protocol){
		if(do_force_protocol > 2){
			log_printf("[Smartcard] Asking for unsupported *user forced* protocol T=%d\n", (do_force_protocol - 1));
			goto err;
		}
		*T_protocol = (do_force_protocol - 1);
	}
	atr->T_protocol_curr = *T_protocol;

	if(do_change_baud_rate){
		/* Get asked Di, Fi and fmax */
		atr->D_i_curr = D_i[ta1 & 0x0f];
		atr->F_i_curr = F_i[ta1 >> 4];
		atr->f_max_curr = f_max[ta1 >> 4];

		/* Is the using forcing a specific ETU? */
		if(do_force_etu){
			if(atr->D_i_curr != 0){
				log_printf("[Smartcard] Current maximum Fi=%d, Di=%d, ETU=%d, user asked for %d ...\n", atr->F_i_curr, atr->D_i_curr, atr->F_i_curr / atr->D_i_curr, do_force_etu);
			}
			else{
				log_printf("[Smartcard] Current maximum Fi=%d, Di=%d, user asked for %d ...\n", atr->F_i_curr, atr->D_i_curr, do_force_etu);

			}
			/* Find a suitable ETU with Fi and the possible values of Di */
			unsigned int i = ta1 & 0x0f;
			uint32_t etu_tmp = 0;
			while(1){
				if(i == 0){
					log_printf("[Smartcard] Error: forcing ETU to %d failed ...\n", do_force_etu);
					goto err;
				}
				if(D_i[i] != 0){
					if((D_i[i] & 0xffff) == 0){
						/* Di is not a fraction */
						etu_tmp = atr->F_i_curr * (D_i[i] >> 16);
					}
					else{
						/* Di is a fraction */
						etu_tmp = atr->F_i_curr / D_i[i];
					}
					if(etu_tmp < do_force_etu){
						i--;
					}
					else{
						break;
					}
				}
				else{
					i--;
				}
			}
			atr->D_i_curr = D_i[i];
			log_printf("[Smartcard] Trying Fi=%d, Di=%d, ETU=%d as the best choice compared to asked %d\n", atr->F_i_curr, atr->D_i_curr, etu_tmp, do_force_etu);
		}

		/* If the card is asking for a forbidden value, return */
		if((atr->D_i_curr == 0) || (atr->F_i_curr == 0) || (atr->f_max_curr == 0)){
			log_printf("[Smartcard] PSS/PTS error (asking for impossible Fi = %d and/or Di = %d)\n", atr->F_i_curr, atr->D_i_curr);
			goto err;
		}
		/* Compute our new ETU */
		if((atr->D_i_curr & 0xffff) == 0){
			/* Di is a fraction */
			etu_curr = atr->F_i_curr * (atr->D_i_curr >> 16);
		}
		else{
			/* Di is not a fraction */
			etu_curr = atr->F_i_curr / atr->D_i_curr;
		}
		/* Adapt our baudrate and clock frequency to what the card is asking,
		 */
		/* In specific mode, we change the Baudrate *before* sending the PPS */
		if(negotiable_mode == 0){
			log_printf("[Smartcard] Switching to ETU = %d (Di = %d, Fi = %d), guard time = %d, max_frequency = %d, Protocol T=%d\n", etu_curr, atr->D_i_curr, atr->F_i_curr, extra_guard_time, atr->f_max_curr, *T_protocol);
			if(SC_adapt_clocks(etu_curr, atr->f_max_curr)){
				goto err;
			}
		}
	}

	/* Adapt the Guard Time per TC1 if necessary */
	if(atr->t_mask[2] & 0x1){
		/* We have a TC1 representing the prefered guard time */
		extra_guard_time = atr->tc[0];
		/* Handle special value 0xff */
		if(extra_guard_time == 0xff){
			if(*T_protocol == 0){
				extra_guard_time = 0;
			}
			else if(*T_protocol == 1){
				/* TC1 = 0xff means that we must reduce our standard 2 ETU guard time
				 * to 1 ETU for 'agressive' performance mode. Our underlying physical layer
				 * might not handle such a parameter, hence for now we return an error.
				 */
                                if(platform_smartcard_set_1ETU_guardtime()){
                                        log_printf("[Smartcard] PSS/PTS error: TC1=0xff for T=1 means unsupported ETU=1 ...\n");
                                        goto err;
                                }
				extra_guard_time = 0;
			}
			else{
				goto err;
			}
		}
		asked_tc1 = 1;
		/* Add the extra guard time to our CGT */
		CGT_character_guard_time += extra_guard_time;
	}

	if(do_negotiate_pts){
		/* New baudrate to negotiate */
		uint8_t new_baud_rate = 0;
		/* Default TA1 in case where we do not want to neogtiate the baud rate */
		uint8_t default_ta1 = 0x11;
		/****** Send the PTS to the card ******/
		pck = 0;
		/* Send PTSS */
		if(SC_putc_timeout(0xff, WT_wait_time)){
			log_printf("[Smartcard] PSS/PTS error (PSS byte 0xff)\n");
			goto err;
		}
		pck ^= 0xff;
		/* Send PTS0 telling that we will apply new Fi/Di (PTS1) */
		if(SC_putc_timeout((asked_ta1 << 4) | (*T_protocol), WT_wait_time)){
			log_printf("[Smartcard] PSS/PTS error (byte PTS0)\n");
			goto err;
		}
		pck ^= (asked_ta1 << 4) | (*T_protocol);
		/* Send PTS1 if necessary */
		if(asked_ta1){
			if(do_change_baud_rate){
				unsigned int i;
				for(i = 0; i < sizeof(F_i) / sizeof(uint32_t); i++){
					if(atr->F_i_curr == F_i[i]){
						new_baud_rate = (i << 4);
						break;
					}
				}
				for(i = 0; i < sizeof(D_i) / sizeof(uint32_t); i++){
					if(atr->D_i_curr == D_i[i]){
						new_baud_rate ^= (i & 0x0f);
						break;
					}
				}
				if(SC_putc_timeout(new_baud_rate, WT_wait_time)){
					log_printf("[Smartcard] PSS/PTS error (byte PTS1)\n");
					goto err;
				}
				pck ^= new_baud_rate;
			}
			else{
				if(SC_putc_timeout(default_ta1, WT_wait_time)){
					log_printf("[Smartcard] PSS/PTS error (byte PTS1)\n");
					goto err;
				}
				pck ^= default_ta1;
			}
		}
		/* Send the checksum */
		if(SC_putc_timeout(pck, WT_wait_time)){
			log_printf("[Smartcard] PSS/PTS error (byte PTS checksum)\n");
			goto err;
		}
		/* Now check that the card agrees to our PTS */
		/* Check for PSS = 0xff */
		if(SC_getc_timeout(&c, WT_wait_time)){
			log_printf("[Smartcard] PSS/PTS error (0xff ACK error in receive)\n");
			goto err;
		}
		if(c != 0xff){
			log_printf("[Smartcard] PSS/PTS error (0xff ACK error, values differ)\n");
			goto err;
		}
		/* Check for PTS0 */
		if(SC_getc_timeout(&c, WT_wait_time)){
			log_printf("[Smartcard] PSS/PTS error (PTS0 ACK error in receive)\n");
			goto err;
		}
		if(c != ((asked_ta1 << 4) | (*T_protocol))){
			log_printf("[Smartcard] PSS/PTS error (PTS0 ACK error, values differ)\n");
			goto err;
		}
		/* Optionally check for PTS1 */
		if(asked_ta1){
			if(SC_getc_timeout(&c, WT_wait_time)){
				log_printf("[Smartcard] PSS/PTS error (PTS1 ACK error in receive)\n");
				goto err;
			}
			if(do_change_baud_rate){
				if(c != new_baud_rate){
					log_printf("[Smartcard] PSS/PTS error (PTS1 ACK error, values differ)\n");
					goto err;
				}
			}
			else{
				if(c != default_ta1){
					log_printf("[Smartcard] PSS/PTS error (PTS1 ACK error, values differ)\n");
					goto err;
				}
			}
		}
		/* Check for the PCK checksum */
		if(SC_getc_timeout(&c, WT_wait_time)){
			log_printf("[Smartcard] PSS/PTS error (PTS checksum ACK error in receive)\n");
			goto err;
		}
		if(c != pck){
			log_printf("[Smartcard] PSS/PTS error (PTS checksum ACK error, values differ)\n");
			goto err;
		}
		log_printf("[Smartcard] PSS/PTS success: PTS sent and confirmed by the card!\n");
	}

	if(do_change_baud_rate){
		/* Adapt the Work Waiting Time for T=0 */
		uint8_t wi = 10; /* default value */
		if(atr->t_mask[2] & (0x1 << 1)){
			/* Handle the specific T=0 ATR TC2 byte here, specifying the Work Waiting Time.
			 * NOTE: TC2 = 0x00 is reserved for future use, so we do not update wi in this case.
			 */
			if(atr->tc[1] != 0){
				wi = atr->tc[1];
			}
		}
		/* Work Waiting Time = 960 x D x WI */
		WT_wait_time = 960 * atr->D_i_curr * wi;
		/* NOTE: since our lower layer only triggers a successful reception *after*
		 * a full byte reception, it is hard to detect the leading edge of the next full byte since
		 * it is being received. Hence, in order for our CWT to fully work, we instead use the more
		 * conservative "two bytes detection" and add to the standard duration 14 ETU since our current
		 * layer will detect the detection only after the byte has been fully receive.
		 * (14 ETU = 12 ETU for reception + guard time + 2 ETU for conservative value)
		*/
		 WT_wait_time = WT_wait_time + 14;
		/* Set the USART clocks to the new settings if it is not already done yet.
		 */
		if(negotiable_mode == 1){
			log_printf("[Smartcard] Switching to ETU = %d (Di = %d, Fi = %d), guard time = %d, max_frequency = %d, Protocol T=%d\n", etu_curr, atr->D_i_curr, atr->F_i_curr, extra_guard_time, atr->f_max_curr, *T_protocol);
			if(SC_adapt_clocks(etu_curr, atr->f_max_curr)){
				goto err;
			}
		}
	}

	/* Some specific stuff in ISO7816 */
	/* In T=0, When using D = 64, the interface device shall ensure a delay of at least 16 etu between the leading edge of
	 * the last received character and the leading edge of the character transmitted for initiating a command. */
	if((atr->D_i_curr == 64) && (*T_protocol == 0)){
		GT_receive_send_interval_etu = 16;
	}
	else{
		GT_receive_send_interval_etu = 0;
	}

	return 0;

err:
	return -1;
}

/* Get the response of an APDU from the card in T=0.
 * The response is a "raw" one, and the response fragmentation is handled
 * in our upper T=0 functions layer.
 * pull_type == 0 => we wait for a procedure byte (ACK, NULL byte, SW1)
 * pull_type == 1 => we wait to get data + SW1/SW2 with no possible NULL bytes
 * pull_type == 2 => we wait to get data + SW1/SW2 with a possible NULL byte
 *
 * wait_all_bytes == 1 => we wait for all the bytes from the card regardless of
 * wait_all_bytes == 0 => we stop waiting when we have reached the Le bytes in the current APDU
 */
static int SC_pull_RESP_T0(SC_T0_APDU_cmd *apdu, SC_T0_APDU_resp *resp, unsigned char pull_type, unsigned char wait_all_bytes){
	int ret;
	uint8_t c;
	uint8_t ack;
	uint8_t ack_one_byte;

	if((apdu == NULL) || (resp == NULL)){
		goto err;
	}

	ack = apdu->ins;
	ack_one_byte = apdu->ins ^ 0xff;

	if(pull_type != 0){
		goto GET_RESP_BYTES;
	}
	/* First, we wait for the data with a timeout of
	 * around some ETUs.
	 */
WAIT_AGAIN:
	if(resp->le >= (SHORT_APDU_LE_MAX + 2)){
		/* Overflow ... Return an error */
		goto err;
	}
	if(SC_getc_timeout(&(resp->data[resp->le]), WT_wait_time)){
		goto err;
	}
	/* Check the received procedure byte */
	if(resp->data[resp->le] == 0x60){
		/* We have received a NULL byte, this is a lose way in T=0 of
		 * telling us to wait ...
		 */
		goto WAIT_AGAIN;
	}
	if((resp->data[resp->le] == ack) && ((resp->data[resp->le] & 0xf0) != 0x60) && ((resp->data[resp->le] & 0xf0) != 0x90)){
		/* This is an ACK, go and send or receive all the data */
		return 1;
	}
	if((resp->data[resp->le] == ack_one_byte) && ((resp->data[resp->le] & 0xf0) != 0x60) && ((resp->data[resp->le] & 0xf0) != 0x90)){
		/* This is an ACK to send or receive one byte */
		if(apdu->lc != 0){
			/* Send one byte (return to the upper layer to deal with this) */
			return 2;
		}
		else{
			/* Receive one byte and got back to wait for a procedure byte, the other bytes should follow ... */
			ret = SC_getc_timeout(&(resp->data[resp->le]), WT_wait_time);
			if(ret){
				/* Timeout reached without any byte, get out */
				goto END;
			}
			resp->le++;
			goto WAIT_AGAIN;
		}
	}
	/* Then, we get the data and SW1/SW2
	 */
	resp->le++;
GET_RESP_BYTES:
	while(1){
		ret = SC_getc_timeout(&c, WT_wait_time);
		if(ret){
			/* Timeout reached without any byte, get out */
			goto END;
		}
		else{
			if(resp->le >= (SHORT_APDU_LE_MAX + 2)){
				/* Overflow ... Return an error */
				goto err;
			}
			/* A byte has been received, continue */
			if((c == 0x60) && (pull_type == 2) && (resp->le == 0)){
				/* If this a WAIT NULL byte and we could expect a procedure byte, we
				 * drop it and go back to waiting other bytes.
				 * NB: the WAIT NULL byte can only be the first one received.
				 */
				goto GET_RESP_BYTES;
			}
			resp->data[resp->le++] = c;
		}
		/* Do we wait for unexpected bytes or not? */
		if(wait_all_bytes == 0){
                        /* If we did not have any procedure byte and end here, this means that the card directly sends us SW1/SW2 */
                        if(pull_type == 0){
                                if(resp->le >= 2){
                                        goto END;
                                }
                        }
			/* Check how many bytes we have compared to the expected Le
			 * and early finish if we already have the expected number of bytes.
			 */
			if(apdu->send_le != 0){
				if((apdu->le != 0) && (resp->le >= (apdu->le + 2))){
					goto END;
				}
			}
			else{
				/* No Le expected, exit when we have SW1/SW2 */
				if(resp->le >= 2){
					goto END;
				}
			}
		}
	}
END:
	if(resp->le < 2){
		/* We should have received at least two bytes (SW1 and SW2).
		 * If this is not the case, this is an erroneous answer.
		 */
		goto err;
	}
	/* Split the status bytes from the received data */
	resp->le -= 2;
	/* Sanity checks on sizes */
	if(resp->le > SHORT_APDU_LE_MAX){
		goto err;
	}
	resp->sw1 = resp->data[resp->le];
	resp->sw2 = resp->data[resp->le+1];

	return 0;

err:
	if(resp != NULL){
		resp->le = 0;
	}
	return -1;
}

#define SC_T0_WAIT_ALL_BYTES 	0

/* Push TPDU in T=0 and pull the response from the card */
static int SC_push_pull_APDU_T0(SC_T0_APDU_cmd *apdu, SC_T0_APDU_resp *resp){
	unsigned int i;
	int ret;
	uint16_t curr_send_byte;

	if((apdu == NULL) || (resp == NULL)){
		goto err;
	}
	/* Sanity checks on the lengths */
	if(apdu->le > SHORT_APDU_LE_MAX){
		/* Note: apdu->lc is on an uint8_t, so no need to check */
		/* Return an error: in T=0,
		 * extended APDUs are handled at the upper layer level with the
		 * ENVELOPE command.
		 */
		goto err;
	}
	/* Sanity check on Le and Lc: we should not have
	 * a case 4 APDU here since its specific fragmentation
	 * is handled in the upper layer ... See page 36 in ISO7816-3:2006.
	 * As a result, we return an error here if we have a case 4 APDU.
	 */
	if((apdu->send_le != 0) && (apdu->lc != 0)){
		log_printf("[Smartcard] T=0 case 4 APDU not fragmented ...\n");
		goto err;
	}

	/* Initialize our response length */
	resp->le = 0;

	/* See ISO7816-3:2006 10.2:
	 * When using D = 64, the interface device shall ensure a delay of at least 16 etu between the leading edge of
	 * the last received character and the leading edge of the character transmitted for initiating a command.
	 */
	SC_delay_etu(GT_receive_send_interval_etu);

	/* Push the header */
	/* Send the CLA */
	if(SC_putc_timeout(apdu->cla, WT_wait_time)){
		goto err;
	}
	/* Send the INS */
	if(SC_putc_timeout(apdu->ins, WT_wait_time)){
		goto err;
	}
	/* Send P1 */
	if(SC_putc_timeout(apdu->p1, WT_wait_time)){
		goto err;
	}
	/* Send P2 */
	if(SC_putc_timeout(apdu->p2, WT_wait_time)){
		goto err;
	}
	/* Push P3 */
	if(apdu->lc != 0){
		/* Send Lc in one byte */
		if(SC_putc_timeout(apdu->lc, WT_wait_time)){
       			goto err;
       	 	}
	}
        else{
                /* Case 1 APDU or case 2? */
                if(apdu->send_le){
                        /* Case 2 APDU: send Le in one byte */
                        if(SC_putc_timeout((apdu->le) & 0xff, WT_wait_time)){
                                goto err;
                        }
                }
                else{
                        /* We send a 0 as P3 in a case 1 APDU */
                        if(SC_putc_timeout(0x00, WT_wait_time)){
                                goto err;
                        }
                }
        }
	curr_send_byte = 0;
GET_PROCEDURE_BYTE:
	/* Get the procedure byte(s) and possibly the answer from the card */
	ret = SC_pull_RESP_T0(apdu, resp, 0, SC_T0_WAIT_ALL_BYTES);
	if(ret == -1){
		goto err;
	}
	else if((ret == 1) || (ret == 2)){
		if((ret == 2) && (curr_send_byte < apdu->lc)){
			/* Send only one byte and go to waiting a procedure byte */
			if(SC_putc_timeout(apdu->data[curr_send_byte], WT_wait_time)){
				goto err;
			}
			curr_send_byte++;
			goto GET_PROCEDURE_BYTE;
		}
		if((ret == 1) && (curr_send_byte < apdu->lc)){
			/* We had an ACK to send all our data */
			if(apdu->lc != 0){
				/* Send the remaining data in one block as asked by the smartcard */
				for(i = curr_send_byte; i < apdu->lc; i++){
					if(SC_putc_timeout(apdu->data[i], WT_wait_time)){
 						goto err;
			        	}
				}
			}
		}
		if(apdu->send_le){
			/* Get our response from the card with NO possible WAIT byte since
			 * we wait data from the card.
			 */
			ret = SC_pull_RESP_T0(apdu, resp, 1, SC_T0_WAIT_ALL_BYTES);
		}
		else{
			/* Get our response from the card with a possible WAIT byte since
			 * we do not wait data from the card.
			 */
			ret = SC_pull_RESP_T0(apdu, resp, 2, SC_T0_WAIT_ALL_BYTES);
		}
		if(ret != 0){
			goto err;
		}
	}
	else if(ret == 0){
		/* This is an answer from the card */
		return 0;
	}
	else{
		/* Unexpected case, this is an error */
		goto err;
	}

	return 0;
err:

	return -1;
}

/* This primitive sends an APDU in T=0 and handles the request/response fragmentation
 * by using ENVELOPE and GET_RESPONSE instructions whenever necessary (extended APDUs,
 * case 4 APDUs, ...).
 */
static int SC_send_APDU_T0(SC_APDU_cmd *apdu, SC_APDU_resp *resp){
	SC_T0_APDU_cmd curr_apdu = { .cla = 0, .ins = 0, .p1 = 0, .p2 = 0, .lc = 0, .data = { 0 }, .le = 0, .send_le = 0 };
	SC_T0_APDU_resp curr_resp = { .data = { 0 }, .le = 0, .sw1 = 0, .sw2 = 0 };
	/* Special case 4 APDU split with a GET_RESPONSE */
	unsigned char case4_getresponse = 0;

	if((apdu == NULL) || (resp == NULL)){
		goto err;
	}
	/* Cleanup our lower layer */
	platform_SC_flush();

	if(apdu->lc > SHORT_APDU_LC_MAX){
		/* If we have to send an extended APDU, we have to use the ENVELOPE command
		 * and split it.
		 */
		unsigned int encapsulated_apdu_len;
		unsigned int num_t0_apdus;
		unsigned int i;

		/* Sanity checks on our lengths */
		if((apdu->lc > APDU_MAX_BUFF_LEN) || (apdu->le > APDU_MAX_BUFF_LEN)){
			goto err;
		}

		/* Get the number of T=0 APDUs we will have to send with ENVELOPE commands */
		encapsulated_apdu_len = SC_APDU_get_encapsulated_apdu_size(apdu, NULL, NULL);
		num_t0_apdus = (encapsulated_apdu_len / SHORT_APDU_LC_MAX) + 1;
		if(((encapsulated_apdu_len % SHORT_APDU_LC_MAX) == 0) && (encapsulated_apdu_len != 0)){
			num_t0_apdus--;
		}
		/* Send fragmented T=0 APDUs */
		for(i = 0; i < num_t0_apdus; i++){
			int ret;
			/* Fill in the buffer of our local T0 APDU */
			curr_apdu.lc = SC_APDU_prepare_buffer(apdu, curr_apdu.data, i, SHORT_APDU_LC_MAX, &ret);
			if((curr_apdu.lc == 0) || (ret != 0)){
				/* Error */
				goto err;
			}
			curr_apdu.cla = apdu->cla;
			curr_apdu.p1 = curr_apdu.p2 = 0;
			curr_apdu.ins = INS_ENVELOPE;
			curr_apdu.le = 0;
			curr_apdu.send_le = 0;
			curr_resp.sw1 = curr_resp.sw2 = curr_resp.le = 0;
			if(SC_push_pull_APDU_T0(&curr_apdu, &curr_resp)){
				goto err;
			}
			/* Check that the response is 9000 except for the last envelope */
			if(((curr_resp.sw1 != 0x90) && (curr_resp.sw2 != 0x00)) && (i != (num_t0_apdus-1))){
				/* This is an error (either the card does not support the
				 * ENVELOPE instruction, or this is another error). Anyways,
				 * return the error as is to the upper layer.
				 */
				resp->le = curr_resp.le;
				resp->sw1 = curr_resp.sw1;
				resp->sw2 = curr_resp.sw2;
				memcpy(resp->data, curr_resp.data, curr_resp.le);
				return 0;
			}
		}
		/* From here, we continue to getting the answer from the card */
	}
	else{
		/* We have to send a short APDU. Copy the data in our working buffer. */
		curr_apdu.cla = apdu->cla;
		curr_apdu.ins = apdu->ins;
		curr_apdu.p1  = apdu->p1;
		curr_apdu.p2  = apdu->p2;
		curr_apdu.lc  = apdu->lc;
		if(apdu->send_le != 0){
			if(apdu->lc != 0){
				/* There is a special case for case 4 APDUs.
			 	 * See page 36 in ISO7816-3:2006: we have to send a GET_RESPONSE
				 * to send our Le and get the actual response from the card.
				 */
				curr_apdu.le = 0;
				curr_apdu.send_le = 0;
			}
			else{
				if(apdu->le > SHORT_APDU_LE_MAX){
					/* If it is a case 2E.2 APDU, we send P3 = 0 as described in the ISO7816 standard */
					curr_apdu.le = 0;
					curr_apdu.send_le = 1;
				}
				else{
					curr_apdu.le = apdu->le;
					curr_apdu.send_le = 1;
				}
			}
		}
		memcpy(curr_apdu.data, apdu->data, apdu->lc);
		curr_resp.sw1 = curr_resp.sw2 = curr_resp.le = 0;
		if(SC_push_pull_APDU_T0(&curr_apdu, &curr_resp)){
			goto err;
		}
	}
	/* Handle the case 4 APDU using the GET_RESPONSE method */
	if((apdu->send_le != 0) && (apdu->lc != 0)){
		/* See page 36 in  ISO7816-3:2006 for the different cases */
		if((curr_resp.sw1 == 0x90) && (curr_resp.sw2 == 0x00)){
			case4_getresponse = 1;
			curr_resp.sw2 = apdu->le;
		}
		if(curr_resp.sw1 == 0x61){
			case4_getresponse = 1;
			/* Keep the SW2 as next Le to ask in the GET_RESPONSE */
		}
		/* Else: map TPDU response without any change */
	}
	/* Get the response, possibly split across multiple responses */
	if(((curr_resp.sw1 == 0x61) && (apdu->send_le != 0) && (apdu->le > SHORT_APDU_LE_MAX)) || (case4_getresponse == 1)){
		resp->le = 0;
		/* Zeroize our case 4 state */
		case4_getresponse = 0;
		while(1){
			/* We have data to get with an ISO7816 GET_RESPONSE */
			curr_apdu.cla = apdu->cla;
			curr_apdu.ins = INS_GET_RESPONSE;
			curr_apdu.p1  = 0;
			curr_apdu.p2  = 0;
			curr_apdu.lc  = 0;
			curr_apdu.le  = curr_resp.sw2;
			curr_apdu.send_le = 1;
			curr_resp.sw1 = curr_resp.sw2 = curr_resp.le = 0;
			if(SC_push_pull_APDU_T0(&curr_apdu, &curr_resp)){
				goto err;
			}
			/* Copy the data from the response */
			resp->sw1 = curr_resp.sw1;
			resp->sw2 = curr_resp.sw2;
			if((curr_resp.sw1 == 0x61) || ((curr_resp.sw1 == 0x90) && (curr_resp.sw2 == 0x00))){
				/* We still agregate fragmented answers */
				if((resp->le + curr_resp.le) > APDU_MAX_BUFF_LEN){
					/* We have an overflow, this is an error */
					goto err;
				}
				memcpy(&(resp->data[resp->le]), curr_resp.data, curr_resp.le);
				resp->le += curr_resp.le;
				if((curr_resp.sw1 == 0x90) && (curr_resp.sw2 == 0x00)){
					/* This is the last packet without error, get out! */
					break;
				}
			}
			else{
				/* Sanity check */
				if(curr_resp.le > APDU_MAX_BUFF_LEN){
					goto err;
				}
				/* We have an error, copy the last response data */
				resp->le = curr_resp.le;
				memcpy(resp->data, curr_resp.data, curr_resp.le);
				break;
			}
		}
	}
	else{
		/* Response is not fragmented: copy it in our upper layer APDU response */
		resp->le = curr_resp.le;
		resp->sw1 = curr_resp.sw1;
		resp->sw2 = curr_resp.sw2;
		memcpy(resp->data, curr_resp.data, curr_resp.le);
	}

	return 0;
err:
	/* We have an error, clean up stuff */
	memset(&curr_apdu, 0, sizeof(curr_apdu));
	memset(&curr_resp, 0, sizeof(curr_resp));
	if(resp != NULL){
		memset(resp, 0, sizeof(SC_APDU_resp));
	}
	return -1;
}

/***************** T=1 case ********************************/
/* Compute the checksum (LRC) of a TPDU */
static uint8_t SC_TPDU_T1_lrc(SC_TPDU *tpdu){
	unsigned int i;
	uint8_t lrc = 0;

	if(tpdu == NULL){
		return 0;
	}

	lrc ^= tpdu->nad;
	lrc ^= tpdu->pcb;
	lrc ^= tpdu->len;
	for(i = 0; i < tpdu->len; i++){
		lrc ^= tpdu->data[i];
	}

	return lrc;
}


/* Compute the checksum (CRC-16) of a TPDU */
/* [RB] TODO: check the CRC-16 algorithm ... */
#define CRC_BLOCK(in, crc, poly) do {			\
	unsigned int j;					\
	uint32_t data;					\
	data = (in);					\
	for(j = 0; j < 8; j++){				\
		if(((crc) & 0x0001) ^ (data & 0x0001)){	\
			(crc) = ((crc) >> 1) ^ (poly);	\
		}					\
		else{					\
			(crc) >>= 1;			\
		}					\
		data >>= 1;				\
	}						\
} while(0);

static uint16_t SC_TPDU_T1_crc(SC_TPDU *tpdu){
	unsigned int i;
	uint32_t poly = 0x8408; /* CCIT polynomial x16 + x12 + x5 + 1 */
	uint32_t crc  = 0xffff;

	if(tpdu == NULL){
		return 0;
	}

	CRC_BLOCK(tpdu->nad, crc, poly);
	CRC_BLOCK(tpdu->pcb, crc, poly);
	CRC_BLOCK(tpdu->len, crc, poly);
	for(i = 0; i < tpdu->len; i++){
		CRC_BLOCK(tpdu->data[i], crc, poly);
	}

	crc = ~crc;
	crc = (crc << 8) | ((crc >> 8) & 0xff);

	return (uint16_t)crc;
}

/* Compute the checksum of a TPDU */
static void SC_TPDU_T1_checksum_compute(SC_TPDU *tpdu, SC_ATR *atr){

	if((tpdu == NULL) || (atr == NULL)){
		return;
	}

	/* The method used for the checksum depends on ATR byte (LRC or CRC). Default is LRC.
	 * TCi (i>2) contains this information.
	 */
	tpdu->edc_type = EDC_TYPE_LRC;
	if((atr->t_mask[2] & (0x1 << 2)) && (atr->tc[2] & 0x1)){
		tpdu->edc_type = EDC_TYPE_CRC;
	}

	if(tpdu->edc_type == EDC_TYPE_LRC){
		/* LRC is the xor of all the bytes of the TPDU */
		tpdu->edc_lrc = SC_TPDU_T1_lrc(tpdu);
		return;
	}
	else{
		/* CRC type */
		tpdu->edc_crc = SC_TPDU_T1_crc(tpdu);
	}

	return;
}

/* Check the checksum of a TPDU */
static int SC_TPDU_T1_checksum_check(SC_TPDU *tpdu){

	if(tpdu == NULL){
		return 0;
	}

	if(tpdu->edc_type == EDC_TYPE_LRC){
		/* LRC is the xor of all the bytes of the TPDU */
		if(tpdu->edc_lrc != SC_TPDU_T1_lrc(tpdu)){
			return 0;
		}
	}
	else{
		/* CRC type */
		if(tpdu->edc_crc != SC_TPDU_T1_crc(tpdu)){
			return 0;
		}
	}

	return 1;
}


/* Push a TPDU on the line */
static int SC_push_TPDU_T1(SC_TPDU *tpdu){
	unsigned int i;

	if(tpdu == NULL){
		goto err;
	}

	/* Sanity check on the length (254 bytes max as specified in the standard) */
	if(tpdu->len > TPDU_T1_DATA_MAXLEN){
		goto err;
	}
	/* Send the NAD */
	if(SC_putc_timeout(tpdu->nad, CWT_character_wait_time)){
		goto err;
	}
	/* Send the other bytes with the CWT timeout */
	/* Send PCB */
	if(SC_putc_timeout(tpdu->pcb, CWT_character_wait_time)){
		goto err;
	}
	/* Send the length */
	if(SC_putc_timeout(tpdu->len, CWT_character_wait_time)){
		goto err;
	}
	/* Send the information field if it is present */
	if(tpdu->data != NULL){
		for(i = 0; i < tpdu->len; i++){
			if(SC_putc_timeout(tpdu->data[i], CWT_character_wait_time)){
				goto err;
			}
		}
	}
	/* Send the epilogue */
	if(tpdu->edc_type == EDC_TYPE_LRC){
		if(SC_putc_timeout(tpdu->edc_lrc, CWT_character_wait_time)){
			goto err;
		}
	}
	else if(tpdu->edc_type == EDC_TYPE_CRC){
		if(SC_putc_timeout((tpdu->edc_crc >> 8) & 0xff, CWT_character_wait_time)){
			goto err;
		}
		if(SC_putc_timeout(tpdu->edc_crc & 0xff, CWT_character_wait_time)){
			goto err;
		}
	}
	else{
		goto err;
	}

	return 0;
err:
	return -1;
}

/* Pull a TPDU from the line */
static int SC_pull_TPDU_T1(SC_TPDU *tpdu, uint32_t resp_timeout){
	unsigned int i;

	if(tpdu == NULL){
		goto err;
	}

	/* Get the NAD with the response timeout (usually BWT) */
	if(SC_getc_timeout(&(tpdu->nad), resp_timeout)){
		goto err;
	}
	/* Get the other bytes with the CWT timeout ... */
	/* Get the PCB */
	if(SC_getc_timeout(&(tpdu->pcb), CWT_character_wait_time)){
		goto err;
	}
	/* Get the length */
	if(SC_getc_timeout(&(tpdu->len), CWT_character_wait_time)){
		goto err;
	}
	/* Sanity check on the length (254 bytes max as specified in the standard) */
	if(tpdu->len > TPDU_T1_DATA_MAXLEN){
		goto err;
	}
	/* Get the data */
	if(tpdu->data != NULL){
		for(i = 0; i < tpdu->len; i++){
			if(SC_getc_timeout(&(tpdu->data[i]), CWT_character_wait_time)){
				goto err;
			}
		}
	}
	/* Get the checksum */
	if(tpdu->edc_type == EDC_TYPE_LRC){
		if(SC_getc_timeout(&(tpdu->edc_lrc), CWT_character_wait_time)){
			goto err;
		}
	}
	else if(tpdu->edc_type == EDC_TYPE_CRC){
		uint8_t crc1, crc2;
		if(SC_getc_timeout(&crc1, CWT_character_wait_time)){
			goto err;
		}
		if(SC_getc_timeout(&crc2, CWT_character_wait_time)){
			goto err;
		}
		tpdu->edc_crc = (crc1 << 8) & crc2;
	}
	else{
		goto err;
	}

	return 0;
err:
	return -1;
}

/*** T=1 helpers for block types and error handling ***/
static inline int SC_TPDU_T1_is_IBLOCK(SC_TPDU *tpdu){
	if(tpdu == NULL){
		return 0;
	}
	return ((tpdu->pcb & PCB_IBLOCK_MSK) == PCB_IBLOCK);
}

static inline int SC_TPDU_T1_is_RBLOCK(SC_TPDU *tpdu){
	if(tpdu == NULL){
		return 0;
	}
	return ((tpdu->pcb & PCB_RBLOCK_MSK) == PCB_RBLOCK);
}

static inline int SC_TPDU_T1_is_SBLOCK(SC_TPDU *tpdu){
	if(tpdu == NULL){
		return 0;
	}
	return ((tpdu->pcb & PCB_SBLOCK_MSK) == PCB_SBLOCK);
}

static inline void SC_TPDU_T1_set_IBLOCK(SC_TPDU *tpdu){
	if(tpdu == NULL){
		return;
	}
	tpdu->pcb |= PCB_IBLOCK;
	return;
}

static inline void SC_TPDU_T1_set_RBLOCK(SC_TPDU *tpdu){
	if(tpdu == NULL){
		return;
	}
	tpdu->pcb |= PCB_RBLOCK;
	return;
}

static inline void SC_TPDU_T1_set_SBLOCK(SC_TPDU *tpdu){
	if(tpdu == NULL){
		return;
	}
	tpdu->pcb |= PCB_SBLOCK;
	return;
}

static uint8_t SC_TPDU_T1_get_sequence(SC_TPDU *tpdu){
	if(tpdu == NULL){
		return 0xff;
	}
	if(SC_TPDU_T1_is_IBLOCK(tpdu)){
		return ((tpdu->pcb & PCB_ISEQ_NUM_MASK) >> PCB_ISEQ_NUM_POS);
	}
	else if(SC_TPDU_T1_is_RBLOCK(tpdu)){
		return ((tpdu->pcb & PCB_RSEQ_NUM_MASK) >> PCB_RSEQ_NUM_POS);
	}
	else{
		/* Should not happen */
		return 0xff;
	}
}

static inline int SC_TPDU_T1_is_sequence_ok(SC_TPDU *tpdu, uint8_t sequence_number){
	return (SC_TPDU_T1_get_sequence(tpdu) == sequence_number);
}


static int SC_TPDU_T1_set_sequence(SC_TPDU *tpdu, uint8_t sequence_number){
	if(tpdu == NULL){
		return -1;
	}
	if(SC_TPDU_T1_is_IBLOCK(tpdu)){
		tpdu->pcb |= (sequence_number << PCB_ISEQ_NUM_POS);
		return 0;
	}
	else if(SC_TPDU_T1_is_RBLOCK(tpdu)){
		tpdu->pcb |= (sequence_number << PCB_RSEQ_NUM_POS);
		return 0;
	}
	else{
		return -1;
	}
}

static inline uint8_t SC_TPDU_T1_RBLOCK_get_error(SC_TPDU *tpdu){
	if(tpdu == NULL){
		return 0xff;
	}
	/* Return an error if this is not an RBLOCK */
	if(!SC_TPDU_T1_is_RBLOCK(tpdu)){
		return 0xff;
	}
	return (tpdu->pcb & PCB_ERR_MASK);
}

/* Send an error frame with the given error and the given frame sequence */
static int SC_TPDU_T1_send_error(uint8_t pcb_err, uint8_t sequence_number, SC_ATR *atr){
	SC_TPDU err_tpdu;

	if(atr == NULL){
		return -1;
	}

	err_tpdu.nad = 0;

	/* PCB is an RBLOCK */
	err_tpdu.pcb = 0;
	SC_TPDU_T1_set_RBLOCK(&err_tpdu);
	SC_TPDU_T1_set_sequence(&err_tpdu, sequence_number);
	/* Put the error field */
	err_tpdu.pcb |= pcb_err;
	/* No data */
	err_tpdu.len  = 0;
	err_tpdu.data = NULL;
	/* Compute the checksum */
	SC_TPDU_T1_checksum_compute(&err_tpdu, atr);

	/* Send the error on the line */
	SC_push_TPDU_T1(&err_tpdu);

	return 0;
}

/* Get the type of an SBLOCK */
static uint8_t SC_TPDU_T1_SBLOCK_get_type(SC_TPDU *tpdu){
	if(tpdu == NULL){
		return 0xff;
	}

	/* Return an error if this is not an SBLOCK */
	if(!SC_TPDU_T1_is_SBLOCK(tpdu)){
		return 0xff;
	}

	return (tpdu->pcb & SBLOCK_TYPE_MSK);
}

/* Get the waiting time asked by an SBLOCK */
static int SC_TPDU_T1_SBLOCK_get_waiting_time(SC_TPDU *tpdu, uint8_t *waiting_time){
	if((tpdu == NULL) || (waiting_time == NULL)){
		goto err;
	}

	*waiting_time = 0;
	/* Sanity check: is this an SBLOCK? */
	if(!SC_TPDU_T1_is_SBLOCK(tpdu)){
		goto err;
	}
	/* The waiting time should be encoded in a one byte data field
	 * as a multiple of the BWT (Block Waiting Time).
	 */
	if((tpdu->len != 1) || (tpdu->data == NULL)){
		goto err;
	}
	*waiting_time = tpdu->data[0];

	return 0;
err:
	return -1;
}

/* Get the new IFS asked by an SBLOCK */
static int SC_TPDU_T1_SBLOCK_get_new_ifs(SC_TPDU *tpdu, uint8_t *new_ifs){
	if((tpdu == NULL) || (new_ifs == NULL)){
		goto err;
	}

	*new_ifs = 0;
	/* Sanity check: is this an SBLOCK? */
	if(!SC_TPDU_T1_is_SBLOCK(tpdu)){
		goto err;
	}
	/* The new IFS should be encoded in a one byte data field.
	 */
	if((tpdu->len != 1) || (tpdu->data == NULL)){
		goto err;
	}
	*new_ifs = tpdu->data[0];

	return 0;
err:
	return -1;
}


/* Send an SBLOCK */
static int SC_TPDU_T1_send_sblock(uint8_t sblock_type, uint8_t *data, uint8_t size, SC_ATR *atr){
	SC_TPDU s_tpdu;

	if((data == NULL) || (atr == NULL)){
		return -1;
	}

	s_tpdu.nad = 0;

	/* PCB is an SBLOCK */
	s_tpdu.pcb = 0;
	SC_TPDU_T1_set_SBLOCK(&s_tpdu);
	/* Set the SBLOCK type */
	s_tpdu.pcb |= sblock_type;
	/* Is there data? No data */
	if(size > TPDU_T1_DATA_MAXLEN){
		/* Sanity check */
		return 0;
	}
	s_tpdu.len  = size;
	s_tpdu.data  = data;
	/* Compute the checksum */
	SC_TPDU_T1_checksum_compute(&s_tpdu, atr);

	/* Send the SBLOCK on the line */
	SC_push_TPDU_T1(&s_tpdu);

	return 0;
}

/* Send APDU in T=1 and get the response
 * [RB] FIXME: for now, this is a basic yet straightforward way of handling T=1. Some
 * error/corner cases are not implemented yet! However, this should work
 * for the basic interactions with cards we need.
 */
static volatile uint8_t last_send_sequence = 0;
static volatile uint8_t last_received_sequence = 0;
static int SC_send_APDU_T1(SC_APDU_cmd *apdu, SC_APDU_resp *resp, SC_ATR *atr){
	/* Create an IBLOCK in order to send our APDU */
	SC_TPDU tpdu_send;
	SC_TPDU tpdu_rcv;
	unsigned int i, num_iblocks, bwi, cwi;
	/* Internal working buffers.
	 * Note: we can work with only one buffer for both send and receive, but
	 * this is cleaner to split those two for debug purposes. Additionally, the
	 * buffers size is 254 bytes maximum imposed by the standard, which is reasonable
	 * on our STM32F4 platform.
	 */
	uint8_t buffer_send[TPDU_T1_DATA_MAXLEN] = { 0 };
	uint8_t buffer_recv[TPDU_T1_DATA_MAXLEN] = { 0 };
	unsigned int encapsulated_apdu_len;
	unsigned int received_size;
	uint8_t bwt_factor = 1; /* BWT factor for waiting time extension */

	if((apdu == NULL) || (resp == NULL) || (atr == NULL)){
		goto err;
	}
	/* Cleanup our lower layer */
	platform_SC_flush();

	/* Modify the global value of current IFSC if it has not been done yet */
	if(atr->ifsc == 0){
		/* Is the ATR telling us we can change the IFSC in TAi? */
		atr->ifsc = 32; /* Default is 32 bytes as specified by the standard */
		if(atr->t_mask[0] & (0x1 << 2)){
			/* Sanity check */
			if(atr->ta[2] <= 0xfe){
				atr->ifsc = atr->ta[2];
			}
		}
		if((atr->ifsc == 0) || (atr->ifsc == 255)){
			log_printf("[Smartcard T=1] Bad value for IFSC in TAi = %d\n", atr->ifsc);
			goto err;
		}
	}

	/* Compute the length we will have to send */
	encapsulated_apdu_len = SC_APDU_get_encapsulated_apdu_size(apdu, NULL, NULL);

	/* How much IBLOCKS do we need? */
	if(atr->ifsc == 0){
		/* Avoid division by zero */
		goto err;
	}
	num_iblocks = (encapsulated_apdu_len / atr->ifsc) + 1;
	if(((encapsulated_apdu_len % atr->ifsc) == 0) && (encapsulated_apdu_len != 0)){
		num_iblocks--;
	}

	/* Get the max waiting times from TBi, useful for our reception primitives */
	bwi = 4; /* Default value */
	cwi = 13; /* Default value */
	if(atr->t_mask[1] & (0x1 << 2)){
		cwi = atr->tb[2] & 0x0f;
		bwi = (atr->tb[2] >> 4) & 0x0f;
	}
	/* Update the CWT and BWT according to the dedicated formulas */
	/* NOTE: since our lower layer only triggers a successful reception *after*
	 * a full byte reception, it is hard to detect the leading edge of the next full byte since
	 * it is being received. Hence, in order for our CWT to fully work, we instead use the more
	 * conservative "two bytes detection" and add to the standard duration 14 ETU since our current
	 * layer will detect the detection only after the byte has been fully received.
	 * (14 ETU = 12 ETU for reception + guard time + 2 ETU for conservative value)
	 */
	/* CWT = 11 * etu + 2**cwi etu */
	CWT_character_wait_time = 11 + (0x1 << cwi);
	CWT_character_wait_time = CWT_character_wait_time + 14;
	/* BWT = 11 * etu + 2**bwi * 960 * 372/f s,
	 * with 372/f s = (D/F) * 372 new ETU
	 */
	if(atr->F_i_curr == 0){
		/* Avoid division by zero */
		goto err;
	}
	BWT_block_wait_time = 11 + (((0x1 << bwi) * 960 * 372 * atr->D_i_curr) / atr->F_i_curr);
	BWT_block_wait_time = BWT_block_wait_time + 14;

	/* Sanity zeroize */
	memset(&tpdu_rcv, 0, sizeof(tpdu_rcv));
	memset(&tpdu_send, 0, sizeof(tpdu_send));
	/* NAD is always zero in our case (no multi-slaves) */
	tpdu_send.nad = 0;
	/* Send all the IBLOCKS */
	for(i = 0; i < num_iblocks; i++){
		int ret;
		tpdu_send.pcb = 0;
		/* PCB is an IBLOCK */
		SC_TPDU_T1_set_IBLOCK(&tpdu_send);
		if(i != (num_iblocks-1)){
			/* Blocks are chained except for the last one */
			tpdu_send.pcb |= PCB_M_CHAIN;
		}
		/* Set the sequence number */
		SC_TPDU_T1_set_sequence(&tpdu_send, last_send_sequence);
		/* Flip the sequence number for the next block to send */
		last_send_sequence = (last_send_sequence + 1) % 2;
		/* Compute the length to send and prepare the buffer */
		tpdu_send.len = SC_APDU_prepare_buffer(apdu, buffer_send, i, atr->ifsc, &ret);
		if(ret){
			goto err;
		}

		/* Adapt the data pointer */
		tpdu_send.data = buffer_send;

		/* Compute the checksum */
		SC_TPDU_T1_checksum_compute(&tpdu_send, atr);
SEND_TPDU_AGAIN_CMD:
		/* Reset the BWT factor to 1 */
		bwt_factor = 1;
		/* Send the TPDU */
		if(SC_push_TPDU_T1(&tpdu_send)){
			goto err;
		}
RECEIVE_TPDU_AGAIN_CMD:
		/* Get the ACK from the card */
		tpdu_rcv.data = buffer_recv;
		tpdu_rcv.len = 0;
		tpdu_rcv.edc_type = tpdu_send.edc_type;
		if(!SC_pull_TPDU_T1(&tpdu_rcv, bwt_factor * BWT_block_wait_time)){
			/* If the checksum of the received block is wrong, send an error R block and receive again */
			if(!SC_TPDU_T1_checksum_check(&tpdu_rcv)){
				/* Wait a bit and send a parity error */
				SC_delay_etu(BGT_block_guard_time); /* Wait for the standardized Block Guard Time (22 ETU by default) */
				SC_TPDU_T1_send_error(PCB_ERR_EDC, SC_TPDU_T1_get_sequence(&tpdu_rcv), atr);
				goto RECEIVE_TPDU_AGAIN_CMD;
			}
			/* If we have an error, send again */
			if(SC_TPDU_T1_is_RBLOCK(&tpdu_rcv) && (SC_TPDU_T1_RBLOCK_get_error(&tpdu_rcv) != PCB_ERR_NOERR)){
				/* Check the sequence number */
				if(SC_TPDU_T1_is_sequence_ok(&tpdu_rcv, SC_TPDU_T1_get_sequence(&tpdu_send))){
					/* Genuine error, send again */
					SC_delay_etu(BGT_block_guard_time); /* Wait for the standardized Block Guard Time (22 ETU by default) */
					goto SEND_TPDU_AGAIN_CMD;
				}
				/* Unexpected error */
				log_printf("[Smartcard T=1] Unexpected case: received error block with bad sequence number ...\n");
				goto err;
			}
			/* Check that this is the ACK we are waiting for */
			if(i != (num_iblocks - 1)){
				/* This is not the last block, we should receive a R type block with a last transmitted I Block sequence + 1 */
				if(!SC_TPDU_T1_is_RBLOCK(&tpdu_rcv) || !SC_TPDU_T1_is_sequence_ok(&tpdu_rcv, (SC_TPDU_T1_get_sequence(&tpdu_send) + 1) % 2)){
					/* This is not what we expect */
					log_printf("[Smartcard T=1] Unexpected case: received other block than expected RBLOCK, or bad sequence number ...\n");
					goto err;
				}
			}
			else{
				/* This is the last block, we should receive at least one I type block with a last I Block received sequence + 1 value
				 */
				if(!SC_TPDU_T1_is_IBLOCK(&tpdu_rcv) || !SC_TPDU_T1_is_sequence_ok(&tpdu_rcv, last_received_sequence)){
					/* If we have an error, send again */
					if(SC_TPDU_T1_is_RBLOCK(&tpdu_rcv) && (SC_TPDU_T1_RBLOCK_get_error(&tpdu_rcv) != PCB_ERR_NOERR)){
						/* Check the sequence number */
						if(SC_TPDU_T1_is_sequence_ok(&tpdu_rcv, SC_TPDU_T1_get_sequence(&tpdu_send))){
							/* Genuine error, send again */
							SC_delay_etu(BGT_block_guard_time); /* Wait for the standardized Block Guard Time (22 ETU by default) */
							goto SEND_TPDU_AGAIN_CMD;
						}
						/* Unexpected error */
						log_printf("[Smartcard T=1] Unexpected case: received error block with bad sequence number ...\n");
						goto err;
					}
					/* If this is something else, fallback to our error case ... */
					if(SC_TPDU_T1_is_SBLOCK(&tpdu_rcv)){
						/* If this is an SBLOCK we should not receive, this is an error ... */
						if((SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_RESYNC_REQ) || (SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_WAITING_RESP)){
							log_printf("[Smartcard T=1] Unexpected SBLOCK reveived from smartcard (SBLOCK_RESYNC_REQ or SBLOCK_WAITING_RESP)\n");
							goto err;
						}
						/* If this is a Request Waiting Time extension, answer and go back to waiting our response ... */
						if(SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_WAITING_REQ){
							/* Get the expected waiting time in number of BWT */
							if(SC_TPDU_T1_SBLOCK_get_waiting_time(&tpdu_rcv, &bwt_factor)){
								goto err;
							}
							/* Acknowledge the waiting time */
							SC_delay_etu(BGT_block_guard_time); /* Wait for the standardized Block Guard Time (22 ETU by default) */
							SC_TPDU_T1_send_sblock(SBLOCK_WAITING_RESP, tpdu_rcv.data, tpdu_rcv.len, atr);
							goto RECEIVE_TPDU_AGAIN_CMD;
						}
						if(SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_CHANGE_IFS_REQ){
							/* Get the new IFSC */
							uint8_t new_ifsc;
							if(SC_TPDU_T1_SBLOCK_get_new_ifs(&tpdu_rcv, &new_ifsc)){
								goto err;
							}
							if((new_ifsc == 0) || (new_ifsc == 255)){
								log_printf("[Smartcard T=1] Bad value for IFSC asked with SBLOCK_CHANGE_IFS_REQ = %d\n", new_ifsc);
								goto err;
							}
							else{
								/* Record the new current IFSC in our ATR context: this will be actibe
								 * from now on for the next transactions.
								 */
								atr->ifsc = new_ifsc;
								/* Acknowledge the new IFSC */
								SC_delay_etu(BGT_block_guard_time); /* Wait for the standardized Block Guard Time (22 ETU by default) */
								SC_TPDU_T1_send_sblock(SBLOCK_CHANGE_IFS_RESP, tpdu_rcv.data, tpdu_rcv.len, atr);
								/* Go back to waiting the IBlock */
								goto RECEIVE_TPDU_AGAIN_CMD;
							}
						}
						/* Else, fallback to error since SBLOCKS are not fully implemented */
						log_printf("[Smartcard T=1] S blocks automaton not fully implemented yet!\n");
						goto err;
					}
					log_printf("[Smartcard T=1] Unexpected case: received other block than expected IBLOCK, or bad sequence number ...\n");
					goto err;
				}
				/* Now get out and receive other necessary I blocks */
			}
		}
		else{
			/* Error pulling the response ... */
			log_printf("[Smartcard T=1] TPDU response reception error 1 ...\n");
			goto err;
		}
	}
	/* If we are here, we have received at least one IBlock. We have to check
	 * if more blocks have to be received.
	 */
	/* Reset the BWT factor to 1 */
	bwt_factor = 1;
	received_size = 0;
	while(1){
		/* Flip the last reception sequence counter */
		last_received_sequence = (last_received_sequence + 1) % 2;
		/* Copy the data batch in the response buffer */
		for(i = 0; i < tpdu_rcv.len; i++){
			/* Sanity check */
			if(received_size >= (APDU_MAX_BUFF_LEN + 2)){
				/* We have an overflow, this should not happen ... */
				goto err;
			}
			resp->data[received_size++] = tpdu_rcv.data[i];
		}
		/* More IBlocks are to be received */
		if((tpdu_rcv.pcb & PCB_M_MSK) == PCB_M_CHAIN){
			SC_delay_etu(BGT_block_guard_time); /* Wait for the standardized Block Guard Time (22 ETU by default) */
			/* Send an ACK with the received IBlock sequence + 1 */
			SC_TPDU_T1_send_error(PCB_ERR_NOERR, (SC_TPDU_T1_get_sequence(&tpdu_rcv)+1)%2, atr);
RECEIVE_TPDU_AGAIN_RESP:
			/* Receive the new block */
			if(SC_pull_TPDU_T1(&tpdu_rcv, bwt_factor * BWT_block_wait_time)){
				log_printf("[Smartcard T=1] TPDU response reception error 2 ...\n");
				goto err;
			}
			/* If the checksum of the received block is wrong, send an error R block and receive again */
			if(!SC_TPDU_T1_checksum_check(&tpdu_rcv)){
				/* Wait a bit and send a parity error */
				SC_delay_etu(BGT_block_guard_time); /* Wait for the standardized Block Guard Time (22 ETU by default) */
				SC_TPDU_T1_send_error(PCB_ERR_EDC, SC_TPDU_T1_get_sequence(&tpdu_rcv), atr);
				goto RECEIVE_TPDU_AGAIN_RESP;
			}
			/* If this is not an IBlock, check if this is an SBLOCK and perform the appropriate action.
			 * [RB] TODO: handle the *full* resync automaton here instead of aborting ...
			 */
			if(!SC_TPDU_T1_is_IBLOCK(&tpdu_rcv)){
				if(SC_TPDU_T1_is_SBLOCK(&tpdu_rcv)){
					/* If this is an SBLOCK we should not receive, this is an error ... */
					if((SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_RESYNC_REQ) || (SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_WAITING_RESP)){
						log_printf("[Smartcard T=1] Unexpected SBLOCK received from smartcard (SBLOCK_RESYNC_REQ or SBLOCK_WAITING_RESP)\n");
						goto err;
					}
					/* If this is a Request Waiting Time extension, answer and go back to waiting our response ... */
					if(SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_WAITING_REQ){
						/* Get the expected waiting time in number of BWT */
						if(SC_TPDU_T1_SBLOCK_get_waiting_time(&tpdu_rcv, &bwt_factor)){
							goto err;
						}
						/* Acknowledge the waiting time */
						SC_delay_etu(BGT_block_guard_time); /* Wait for the standardized Block Guard Time (22 ETU by default) */
						SC_TPDU_T1_send_sblock(SBLOCK_WAITING_RESP, tpdu_rcv.data, tpdu_rcv.len, atr);
						goto RECEIVE_TPDU_AGAIN_RESP;
					}
					if(SC_TPDU_T1_SBLOCK_get_type(&tpdu_rcv) == SBLOCK_CHANGE_IFS_REQ){
						/* The smartcard is negotiating a new IFSC, modify it */
						/* Get the new IFSC */
						uint8_t new_ifsc;
						if(SC_TPDU_T1_SBLOCK_get_new_ifs(&tpdu_rcv, &new_ifsc)){
							goto err;
						}
						if((new_ifsc == 0) || (new_ifsc == 255)){
							log_printf("[Smartcard T=1] Bad value for IFSC asked with SBLOCK_CHANGE_IFS_REQ = %d\n", new_ifsc);
							goto err;
						}
						atr->ifsc = new_ifsc;
						/* Acknowledge the new IFSC */
						SC_delay_etu(BGT_block_guard_time); /* Wait for the standardized Block Guard Time (22 ETU by default) */
						SC_TPDU_T1_send_sblock(SBLOCK_CHANGE_IFS_RESP, tpdu_rcv.data, tpdu_rcv.len, atr);
					}
					/* Else, fallback to error since SBLOCKS are not fully implemented */
					log_printf("[Smartcard T=1] S blocks automaton not fully implemented yet!\n");
					goto err;
				}
				else{
					log_printf("[Smartcard T=1] TPDU response reception error: expected IBLOCK but got something else ...\n");
					goto err;
				}
			}
			/* Reset the BWT factor to 1 after each I-Block received */
			bwt_factor = 1;
		}
		else{
			/* We are finished, nothing more to get. Get out. */
			break;
		}
	}
	/* We have received everything, format the response */
	if(received_size < 2){
		/* We have received less than 2 bytes, this is an error ... */
		goto err;
	}
	resp->le = received_size - 2;
	/* Sanity checks on sizes */
	if((resp->le > ((uint32_t)0x1 << 16)) || (resp->le > APDU_MAX_BUFF_LEN)){
		goto err;
	}
	resp->sw1 = resp->data[resp->le];
	resp->sw2 = resp->data[resp->le+1];
	resp->data[resp->le] = resp->data[resp->le+1] = 0;

	return 0;
err:
	/* We have an error, clean up stuff */
	memset(&tpdu_send, 0, sizeof(SC_TPDU));
	memset(&tpdu_rcv, 0, sizeof(SC_TPDU));
	if(resp != NULL){
		memset(resp, 0, sizeof(SC_APDU_resp));
	}

	return -1;
}

/**************** ISO7816 Printing functions *************/

/* Print a T=1 TPDU on the console */
void SC_print_TPDU(SC_TPDU *tpdu)
{
	unsigned int i;

	if(tpdu == NULL){
		return;
	}

	log_printf("===== TPDU ============\n");
	log_printf("NAD = %x, PCB = %x, LEN = %x\n", tpdu->nad, tpdu->pcb, tpdu->len);
	if(tpdu->len > TPDU_T1_DATA_MAXLEN){
		log_printf("Len error: too big ...\n");
		return;
	}
	log_printf("TPDU TYPE = ");
	if(SC_TPDU_T1_is_IBLOCK(tpdu)){
		log_printf("I-Type\n");
	}
	else if(SC_TPDU_T1_is_RBLOCK(tpdu)){
		log_printf("R-Type\n");
	}
	else if(SC_TPDU_T1_is_SBLOCK(tpdu)){
		log_printf("S-Type\n");
	}
	else{
		log_printf("UNKNOWN\n");
	}
	log_printf("APDU: ");
	if(tpdu->data != NULL){
		for(i = 0; i < tpdu->len; i++){
			log_printf("%x ", tpdu->data[i]);
		}
	}
	if(tpdu->len != 0){
		log_printf("\n");
	}
	if(tpdu->edc_type == EDC_TYPE_LRC){
		log_printf("EDC (LRC) = %x\n", tpdu->edc_lrc);
	}
	else{
		log_printf("EDC (CRC) = %x\n", tpdu->edc_crc);
	}

	return;
}


/* Abstract Send APDU/Receive response function */
int SC_iso7816_send_APDU(SC_APDU_cmd *apdu, SC_APDU_resp *resp, SC_ATR *atr, uint8_t T_protocol){
	if((apdu == NULL) || (resp == NULL) || (atr == NULL)){
		goto err;
	}
	switch(T_protocol){
		case 0:
			return SC_send_APDU_T0(apdu, resp);
			break;
		case 1:
			return SC_send_APDU_T1(apdu, resp, atr);
			break;
		default:
			log_printf("[Smartcard] Unsupported asked protocol T=%d in SC_iso7816_send_APDU\n", T_protocol);
			goto err;
	}

	return 0;
err:
	return -1;
}

/* Abstract wait card timeout */
int SC_iso7816_wait_card_timeout(SC_ATR *atr __attribute__((unused)), uint8_t T_protocol){
	if(atr == NULL){
		goto err;
	}

	switch(T_protocol){
		case 0:
			SC_delay_etu(WT_wait_time);
			break;
		case 1:
			SC_delay_etu(BWT_block_wait_time);
			break;
		default:
			log_printf("[Smartcard] Unsupported asked protocol T=%d in SC_iso7816_wait_card_timeout\n", T_protocol);
			goto err;
	}

	return 0;
err:
	return -1;
}

/*** T=0/T=1 Finite State Machine until Idle command *************/

/*
 *  Vcc ____|°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
 *
 *  CLK _______|XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
 *
 *  RST ________________________|°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
 *
 *  I/O _XXXXXXXXXXX|°°°°°°°°°°°°°°°°°°°°°°°°°°|_______|XXXXXXXXX
 *
 *
 */

static int SC_reinit_iso7816(void){
	/* Reinitialize the ISO7816-3 wait times */
        CGT_character_guard_time = CGT_DEFAULT; /* 12 ETU in total, N = 0 by default */
        WT_wait_time = WT_DEFAULT; /* 9600 ETU by default */
        BGT_block_guard_time = BGT_DEFAULT; /* 22 ETU by default */
        CWT_character_wait_time = CWT_DEFAULT; /* Default value of CWT is CGT */
        BWT_block_wait_time = BWT_DEFAULT; /* Default value of BWT is BGT */

        SC_convention = SC_TS_DIRECT_CONVENTION;

        /* (Re)initialize our global variables */
        last_send_sequence = last_received_sequence = 0;

        /* (Re)Initialize the hardware blocks */
	platform_smartcard_reinit();
        if(platform_smartcard_init()){
                goto err;
        }

        platform_SC_reinit_iso7816();

        /* Reinitialize the clocks */
        SC_adapt_clocks(SMARTCARD_DEFAULT_ETU, SMARTCARD_DEFAULT_CLK_FREQ);

        return 0;

err:
        return -1;
}

static volatile bool map_voluntary;

int SC_iso7816_fsm_early_init(sc_iso7816_map_mode_t map_mode)
{
    uint8_t ret;
    switch (map_mode) {
        case SC_7816_MAP_AUTO:
            map_voluntary = false;
            if((ret = platform_smartcard_early_init(DRV7816_MAP_AUTO))){
                return ret;
            }
            break;
        case SC_7816_MAP_VOLUNTARY:
            map_voluntary = true;
            if((ret = platform_smartcard_early_init(DRV7816_MAP_VOLUNTARY))){
                return ret;
            }
            break;
        default:
            log_printf("invalid map mode\n");
            break;
    }
   return 0;
 }


int SC_iso7816_fsm_map(void)
{
    if (map_voluntary) {
        return platform_smartcard_map();
    }
    return 0;

}

int SC_iso7816_fsm_unmap(void)
{
    if (map_voluntary) {
        return platform_smartcard_unmap();
    }
    return 0;
}

void SC_iso7816_smartcard_lost(void)
{
  platform_smartcard_lost();
}

uint8_t SC_iso7816_is_smartcard_inserted(void)
{
  return (platform_is_smartcard_inserted());
}

void SC_iso7816_register_user_handler_action(void (*action)(void)){
  platform_smartcard_register_user_handler_action(action);
  return;
}

int SC_iso7816_fsm_init(SC_ATR *atr, uint8_t *T_protocol, uint8_t do_negiotiate_pts, uint8_t do_change_baud_rate, uint8_t do_force_protocol, uint32_t do_force_etu){
	int ret;

	SC_current_state = SC_READER_IDLE;

	if((atr == NULL) || (T_protocol == NULL)){
		return -1;
	}

	switch (SC_current_state) {
		case SC_READER_IDLE:{
SC_READER_IDLE_LABEL:
			/* RST is set low */
			platform_set_smartcard_rst(0);
			/* Vcc is set low */
			platform_set_smartcard_vcc(0);

			log_printf("Waiting for card insertion\n");
			platform_SC_reinit_smartcard_contact();
			/* We are waiting for a card insertion to make the transition
			 */
			while(!platform_is_smartcard_inserted())
				continue;
			log_printf("Card detected ... Go!\n");

			SC_current_state = SC_READER_IDLE;

			/* (Re)initialize our waiting times and other variables  */
			if(SC_reinit_iso7816()){
				return -1;
			}
			/* A card has been inserted, go! */
			goto SC_POWER_CARD_LABEL;
			break;
		}
		case SC_POWER_CARD:{
SC_POWER_CARD_LABEL:
			SC_current_state = SC_POWER_CARD;
			/***** Cold reset sequence *******/
			/* Lower the Vcc and RST pins */
			platform_set_smartcard_vcc(0);
			platform_set_smartcard_rst(0);
			/* Raise the Vcc pin */
			platform_set_smartcard_vcc(1);
			/* Wait for 40000 clock cycles to raise RST */
			SC_delay_sc_clock_cycles(SC_RST_TIMEOUT);
			/* Raise the RST pin */
			platform_set_smartcard_rst(1);
			/* Wait for the ATR */
			ret = SC_get_ATR(atr);
			if(ret){
				/* If we don't have an ATR, go back to reader idle state */
				goto SC_READER_IDLE_LABEL;
			}
			else{
				/* If we have an ATR, move on! */
				goto SC_PROTOCOL_NEG_LABEL;
			}
			break;
		}
		case SC_PROTOCOL_NEG:{
SC_PROTOCOL_NEG_LABEL:
			SC_current_state = SC_PROTOCOL_NEG;
			/* Negotiate PTS */
			ret = SC_negotiate_PTS(atr, T_protocol, do_negiotiate_pts, do_change_baud_rate, do_force_protocol, do_force_etu);
			if(ret){
				/* If negotiation has failed, return an error */
				/* TODO: force conservative values */
				return -2;
			}
			else{
				/* If we have successfully negotiated, we can send APDUs */
				SC_current_state = SC_IDLE_CMD;
				return 0;
			}
			break;
		}
		default:
			log_printf("Smartcard unhandled state %d in T=0 initialization FSM", SC_current_state);
			return -1;
	}

	return 0;
}
