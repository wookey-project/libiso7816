#ifndef __SMARTCARD_ISO7816_H__
#define __SMARTCARD_ISO7816_H__

#include "libsmartcard.h"

/*** ISO7816-3 constants */
static const uint32_t F_i[16]   = { 372, 372, 558, 744, 1116, 1488, 1860, 0, 0, 512, 768, 1024, 1536, 2048, 0, 0 };
static const uint32_t f_max[16] = { 4000000, 5000000, 6000000, 8000000, 12000000, 16000000, 20000000, 0, 0, 5000000, 7500000, 10000000, 15000000, 20000000, 0, 0 };
static const uint32_t D_i[16]   = { 0, 1, 2, 4, 8, 16, 32, 0, 12, 20, ((uint32_t)2 << 16), ((uint32_t)4 << 16), ((uint32_t)8 << 16), ((uint32_t)16 << 16), ((uint32_t)32 << 16), ((uint32_t)64 << 16) };

/********** APDU (T=0) ************************/
/*** Short APDU Lc max length = 255 ***********/
#define SHORT_APDU_LC_MAX	255
/*** Short APDU Le max length = 256 ***********/
#define SHORT_APDU_LE_MAX	256

/* A T=0 APDU command, limited to short APDU */
typedef struct
{
        uint8_t cla;  /* Command class */
        uint8_t ins;  /* Instruction */
        uint8_t p1;   /* Parameter 1 */
        uint8_t p2;   /* Parameter 2 */
        uint8_t lc;  /* Length of data field, Lc encoded on 8 bits since it is <= 255 */
        uint8_t data[SHORT_APDU_LC_MAX];  /* Data field */
        uint16_t le;   /* Expected return length, encoded on 16 bits since it is <= 256 (so we must encode the last value) */
        uint8_t send_le;
} SC_T0_APDU_cmd;

/* A T=0 APDU response, limited to short APDU */
typedef struct
{
        uint8_t data[SHORT_APDU_LE_MAX + 2]; /* Data field + 2 bytes for temporaty SW1/SW2 storage */
        uint16_t le; /* Actual return length. It is on an uint16_t because we increment it when receiving (this avoids integer overflows). */
        uint8_t sw1; /* Status Word 1 */
        uint8_t sw2; /* Status Word 2 */
} SC_T0_APDU_resp;


/********** T=1 TPDU block ********************/
typedef struct
{
        /* Prologue field (NAD, PCB, LEN) */
        uint8_t nad;
        uint8_t pcb;
        uint8_t len;
        /* Information field (APDU) is handled by the lower layer */
        uint8_t *data;
        /* Epilogue field (EDC) */
        union {
                uint8_t edc_lrc;
                uint16_t edc_crc;
        };
        uint8_t edc_type; /* Type of used EDC */
} SC_TPDU;

#define TPDU_T1_DATA_MAXLEN	254

#define PCB_M_POS		5
#define PCB_M_MSK		(0x1 << PCB_M_POS)
#define PCB_M_NO_CHAIN		(0x0 << PCB_M_POS)
#define PCB_M_CHAIN		(0x1 << PCB_M_POS)

#define PCB_IBLOCK_POS		7
#define PCB_IBLOCK_MSK		(0x1 << PCB_IBLOCK_POS)
#define PCB_IBLOCK		(0x0 << PCB_IBLOCK_POS)

#define PCB_RBLOCK_POS		6
#define PCB_RBLOCK_MSK		(0x3 << PCB_RBLOCK_POS)
#define PCB_RBLOCK		(0x2 << PCB_RBLOCK_POS)

#define PCB_SBLOCK_POS		6
#define PCB_SBLOCK_MSK		(0x3 << PCB_SBLOCK_POS)
#define PCB_SBLOCK		(0x3 << PCB_SBLOCK_POS)

#define PCB_ISEQ_NUM_POS 	6
#define PCB_ISEQ_NUM_MASK	(0x1 << PCB_ISEQ_NUM_POS)
#define PCB_ISEQ_NUM0		(0x0 << PCB_ISEQ_NUM_POS)
#define PCB_ISEQ_NUM1		(0x1 << PCB_ISEQ_NUM_POS)

#define PCB_RSEQ_NUM_POS 	4
#define PCB_RSEQ_NUM_MASK	(0x1 << PCB_RSEQ_NUM_POS)
#define PCB_RSEQ_NUM0		(0x0 << PCB_RSEQ_NUM_POS)
#define PCB_RSEQ_NUM1		(0x1 << PCB_RSEQ_NUM_POS)

#define PCB_ERR_POS		0
#define PCB_ERR_MASK		(0x3 << PCB_ERR_POS)
#define PCB_ERR_NOERR		(0x0 << PCB_ERR_POS)
#define PCB_ERR_EDC		(0x1 << PCB_ERR_POS)
#define PCB_ERR_OTHER		(0x2 << PCB_ERR_POS)

/* SBLOCK types */
#define SBLOCK_TYPE_MSK		0x3f
#define SBLOCK_RESYNC_REQ	0x00
#define SBLOCK_RESYNC_RESP	0x20
#define SBLOCK_CHANGE_IFS_REQ	0x01
#define SBLOCK_CHANGE_IFS_RESP	0x21
#define SBLOCK_ABORT_REQ	0x02
#define SBLOCK_ABORT_RESP	0x22
#define SBLOCK_WAITING_REQ	0x03
#define SBLOCK_WAITING_RESP	0x23
#define SBLOCK_VPP_ERR_RESP	0x24

#define	EDC_TYPE_LRC		0
#define	EDC_TYPE_CRC		1


/* The ISO7816 automaton states from a reader's point of view */
typedef enum {
        SC_READER_IDLE = 0,
        SC_POWER_CARD = 1,
        SC_PROTOCOL_NEG = 2,
        SC_IDLE_CMD = 3,
        SC_WAIT_RESP = 4,
        SC_WAIT_CMD_COMP = 5,
} SC_state;

/* Global variable holding the current state of the FSM */
static SC_state SC_current_state __attribute__((used)) = SC_READER_IDLE;

/***** ATR related constants ******/
/********************************************************/
/* ATR should appear in the 40000 clock cycles, i.e. ~110 ETU
 * with 372 clock cycles per ETU.
 */
#define ATR_ETU_TIMEOUT		110

/* Default ATR frequency and ETU */
#define SMARTCARD_DEFAULT_CLK_FREQ      3500000
#define SMARTCARD_DEFAULT_ETU           372

/**** Guard times and waiting times related constants ***/
/* Here we define all the wait times we need to communicate with the
 * card, and ther default value before the ATR.
 */

/*
 * The CGT is the minimum delay between the leading
 * edges of the two consecutive characters in the same
 * direction of transmission.
 * This is relevant for T=0 and T=1.
 */
#define CGT_DEFAULT	0
static unsigned int CGT_character_guard_time __attribute__((used)) = CGT_DEFAULT; /* 12 ETU in total, N = 0 by default */

/*
 * The WT is the maximum delay allowed between two
 * consecutive characters transmitted by the card or an
 * interfacing device. Only relevant for T=0.
 */
/* NOTE: since our lower layer only triggers a successful reception *after*
 * a full byte reception, it is hard to detect the leading edge of the next full byte since
 * it is being received. Hence, in order for our CWT to fully work, we instead use the more
 * conservative "two bytes detection" and add to the standard duration 14 ETU since our current
 * layer will detect the detection only after the byte has been fully receive.
 * (14 ETU = 12 ETU for reception + guard time + 2 ETU for conservative value)
 */
#define WT_DEFAULT	((9600) + 14)
static unsigned int WT_wait_time __attribute__((used)) = WT_DEFAULT; /* 9600 ETU by default, (960 x D x WI) ETU with WI=10 by default */

/* Block guard time, only relevant for T=1 */
#define BGT_DEFAULT	22
static unsigned int BGT_block_guard_time __attribute__((used)) = BGT_DEFAULT; /* 22 ETU by default */

/* The Character Wait Time, maximum time between two characters leading edge in a block.
 * Only relevant for T=1.
 */
/* NOTE: since our lower layer only triggers a successful reception *after*
 * a full byte reception, it is hard to detect the leading edge of the next full byte since
 * it is being received. Hence, in order for our CWT to fully work, we instead use the more
 * conservative "two bytes detection" and add to the standard duration 14 ETU since our current
 * layer will detect the detection only after the byte has been fully receive.
 * (14 ETU = 12 ETU for reception + guard time + 2 ETU for conservative value)
 */
#define CWT_DEFAULT	((11 + (0x1 << 13)) + 14)
static unsigned int CWT_character_wait_time __attribute__((used)) = CWT_DEFAULT; /* Default value of CWT (CWI is 13 by default) */

/* The Block Wait Time, maximum time between two blocks in the same direction
 * from the card to the terminal. Only relevant for T=1.
 */
/* NOTE: since our lower layer only triggers a successful reception *after*
 * a full byte reception, it is hard to detect the leading edge of the next full byte since
 * it is being received. Hence, in order for our CWT to fully work, we instead use the more
 * conservative "two bytes detection" and add to the standard duration 14 ETU since our current
 * layer will detect the detection only after the byte has been fully receive.
 * (14 ETU = 12 ETU for reception + guard time + 2 ETU for conservative value)
 */
#define BWT_DEFAULT	((11 + ((0x1 << 4) * 960)) + 14)
static unsigned int BWT_block_wait_time __attribute__((used)) = BWT_DEFAULT; /* Default value of BWT (BWI is 4 by default) */

/* Number of clock cycles to wait before raising the
 * RST pin.
 */
#define SC_RST_TIMEOUT		400000

/* Some ISO7816-4 useful instructions */
#define INS_GET_RESPONSE	0xc0
#define INS_ENVELOPE		0xc2

typedef enum {
    SC_7816_MAP_AUTO,
    SC_7816_MAP_VOLUNTARY
} sc_iso7816_map_mode_t;



void SC_iso7816_print_ATR(SC_ATR *atr);
int SC_iso7816_fsm_init(SC_ATR *atr, uint8_t *T_protocol, uint8_t do_negotiate_pts, uint8_t do_change_baudrate, uint8_t do_force_protocol, uint32_t do_force_etu);
int SC_iso7816_send_APDU(SC_APDU_cmd *apdu, SC_APDU_resp *resp, SC_ATR *atr, uint8_t T_protocol);
int SC_iso7816_fsm_early_init(sc_iso7816_map_mode_t map_mode);
void SC_iso7816_smartcard_lost(void);
uint8_t SC_iso7816_is_smartcard_inserted(void);
int SC_iso7816_wait_card_timeout(SC_ATR *atr, uint8_t T_protocol);
void SC_iso7816_register_user_handler_action(void (*action)(void));

int SC_iso7816_fsm_map(void);
int SC_iso7816_fsm_unmap(void);

#endif /* __SMARTCARD_ISO7816_H__ */
