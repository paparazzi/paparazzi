/*
 * Copyright (C) 2017 Michal Podhradsky <mpodhradsky@galois.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file datalink/gec/gec.h
 *
 * Galois embedded crypto implementation
 *
 */
#ifndef SPPRZ_GEC_H
#define SPPRZ_GEC_H

#include "std.h"
#include "mcu_periph/rng.h"
#include "generated/keys_uav.h"
#include "../ext/hacl-c/Hacl_Ed25519.h"
#include "../ext/hacl-c/Hacl_Curve25519.h"
#include "../ext/hacl-c/Hacl_SHA2_512.h"
#include "../ext/hacl-c/Hacl_Chacha20Poly1305.h"
#include "../ext/hacl-c/kremlib.h"

#define PPRZ_MSG_TYPE_PLAINTEXT 0xaa
#define PPRZ_MSG_TYPE_ENCRYPTED 0x55
// index of encrypted/payload byte
#define PPRZ_GEC_IDX 0
// index of the beginning of the counter
#define PPRZ_CNTR_IDX 1
// index of the beginning of the authenticated bytes
#define PPRZ_AUTH_IDX 5

// legth of the message signature
#define PPRZ_SIGN_LEN 64
// lenght of a hash for key derivation
#define PPRZ_HASH_LEN 64
// length of the encryption keys
#define PPRZ_KEY_LEN 32
// length of the message authentication tag
#define PPRZ_MAC_LEN 16
// length of the message nonce
#define PPRZ_NONCE_LEN 12
// length of the counter
#define PPRZ_COUNTER_LEN 4
// length of the crypto overhead (4 bytes of counter + 16 bytes of tag)
#define PPRZ_CRYPTO_OVERHEAD 20
// basepoint value for the scalar curve multiplication
#define PPRZ_CURVE_BASEPOINT 9

typedef unsigned char ed25519_signature[64];

struct gec_privkey {
  uint8_t priv[PPRZ_KEY_LEN];
  uint8_t pub[PPRZ_KEY_LEN]; bool ready;
};

struct gec_pubkey {
  uint8_t pub[PPRZ_KEY_LEN]; bool ready;
};

struct gec_sym_key {
  uint8_t key[PPRZ_KEY_LEN];
  uint8_t nonce[PPRZ_NONCE_LEN];
  uint32_t counter; bool ready;
};

typedef enum {
  INIT, WAIT_MSG1, WAIT_MSG2, WAIT_MSG3, CRYPTO_OK,
} stage_t;

typedef enum {
  INITIATOR, RESPONDER, CLIENT, INVALID_PARTY
} party_t;

typedef enum {
  P_AE, P_BE, SIG,
} gec_sts_msg_type_t;

typedef enum {
  ERROR_NONE,
  // RESPONDER ERRORS
  MSG1_TIMEOUT_ERROR,
  MSG1_ENCRYPT_ERROR,
  MSG3_TIMEOUT_ERROR,
  MSG3_DECRYPT_ERROR,
  MSG3_SIGNVERIFY_ERROR,
  // INITIATOR ERRORS
  MSG2_TIMEOUT_ERROR,
  MSG2_DECRYPT_ERROR,
  MSG2_SIGNVERIFY_ERROR,
  MSG3_ENCRYPT_ERROR,
  // BOTH PARTIES
  UNEXPECTED_MSG_TYPE_ERROR,
  UNEXPECTED_MSG_DATA_ERROR,
  UNEXPECTED_MSG_ERROR,
} sts_error_t;

// Intermediate data structure containing information relating to the stage of
// the STS protocol.
struct gec_sts_ctx {
  struct gec_pubkey their_public_key;
  struct gec_privkey my_private_key;
  struct gec_pubkey their_public_ephemeral;
  struct gec_privkey my_private_ephemeral;
  struct gec_sym_key rx_sym_key;
  struct gec_sym_key tx_sym_key;
  stage_t protocol_stage;
  party_t party;
  sts_error_t last_error;
  uint32_t rx_counter_err;
  uint32_t encrypt_err;
  uint32_t decrypt_err;
};

void gec_sts_init(struct gec_sts_ctx *sts);
void gec_clear_sts(struct gec_sts_ctx *sts);
void gec_generate_ephemeral_keys(struct gec_privkey *sk);
void gec_derive_key_material(struct gec_sts_ctx *sts, uint8_t *z);

void gec_counter_to_bytes(uint32_t n, uint8_t *bytes);
uint32_t gec_bytes_to_counter(uint8_t *bytes);

#endif /* SPPRZ_GEC_H */
