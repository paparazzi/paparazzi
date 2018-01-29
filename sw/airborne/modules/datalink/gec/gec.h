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
 * Galois embedded crypto iplementation
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


#define PPRZ_SIGN_LEN 64
#define PPRZ_KEY_LEN 32
#define PPRZ_NONCE_LEN 12
#define PPRZ_MAC_LEN 16

typedef unsigned char ed25519_signature[64];

struct gec_privkey
{
  uint8_t priv[PPRZ_KEY_LEN];
  uint8_t pub[PPRZ_KEY_LEN];
};

struct gec_pubkey
{
  uint8_t pub[PPRZ_KEY_LEN];
};

struct gec_sym_key
{
  uint8_t key[PPRZ_KEY_LEN];
  uint8_t nonce[PPRZ_NONCE_LEN];
  uint32_t ctr;
};

typedef enum
{
  INIT, WAIT_MSG1, WAIT_MSG2, WAIT_MSG3, CRYPTO_OK,
} stage_t;

typedef enum
{
  INITIATOR, RESPONDER, CLIENT, INVALID_PARTY
} party_t;

typedef enum
{
  P_AE, P_BE, SIG,
} gec_sts_msg_type_t;

typedef enum
{
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
  UNEXPECTED_STS_STAGE_ERROR,
  UNEXPECTED_MSG_ERROR
} sts_error_t;

// Intermediate data structure containing information relating to the stage of
// the STS protocol.
struct gec_sts_ctx
{
  struct gec_pubkey theirPublicKey;
  struct gec_privkey myPrivateKey;
  struct gec_pubkey theirPublicKeyEphemeral;
  struct gec_privkey myPrivateKeyEphemeral;
  struct gec_sym_key theirSymmetricKey;
  struct gec_sym_key mySymmetricKey;
  stage_t protocol_stage;
  party_t party;
  sts_error_t last_error;
  uint32_t counter_err;
  uint32_t encrypt_err;
  uint32_t decrypt_err;
};

void gec_sts_init(struct gec_sts_ctx * sts);

void clear_sts(struct gec_sts_ctx * sts);

void generate_ephemeral_keys(struct gec_privkey *sk);

void derive_key_material(struct gec_sts_ctx *sts, uint8_t* z);

uint32_t gec_encrypt(struct gec_sym_key *k, uint8_t *ciphertext,
    uint8_t *plaintext, uint8_t len, uint8_t *mac);
uint32_t gec_decrypt(struct gec_sym_key *k, uint8_t *plaintext,
    uint8_t *ciphertext, uint8_t len, uint8_t *mac);

#endif /* SPPRZ_GEC_H */
