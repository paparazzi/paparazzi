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
 * @file datalink/gec/gec.c
 *
 * Galois embedded crypto iplementation
 *
 */
#include "modules/datalink/gec/gec.h"

void gec_sts_init(struct gec_sts_ctx * sts)
{
  clear_sts(sts);

  uint8_t theirPublicKey[PPRZ_KEY_LEN] = GCS_PUBLIC;
  memcpy(&sts->theirPublicKey, theirPublicKey, PPRZ_KEY_LEN);

  uint8_t myPublicKey[PPRZ_KEY_LEN] = UAV_PUBLIC;
  memcpy(&sts->myPrivateKey.pub, myPublicKey, PPRZ_KEY_LEN);

  uint8_t myPrivateKey[PPRZ_KEY_LEN] = UAV_PRIVATE;
  memcpy(&sts->myPrivateKey.priv, myPrivateKey, PPRZ_KEY_LEN);

}

void clear_sts(struct gec_sts_ctx * sts)
{
  memset(&sts->theirPublicKeyEphemeral, 0, sizeof(struct gec_pubkey));
  memset(&sts->myPrivateKeyEphemeral, 0, sizeof(struct gec_privkey));
  memset(&sts->theirSymmetricKey, 0, sizeof(struct gec_sym_key));
  memset(&sts->mySymmetricKey, 0, sizeof(struct gec_sym_key));
  sts->protocol_stage = WAIT_MSG1;
  sts->party = RESPONDER;
  sts->last_error = ERROR_NONE;
}

/**
 * Generate private and public key pairs for future use.
 */
void generate_ephemeral_keys(struct gec_privkey *sk)
{
  for (uint16_t i = 0; i < (PPRZ_KEY_LEN / sizeof(uint32_t)); i++) {
    uint32_t tmp = rng_wait_and_get();
    sk->priv[i] = (uint8_t) tmp;
    sk->priv[i + 1] = (uint8_t) (tmp >> 8);
    sk->priv[i + 2] = (uint8_t) (tmp >> 16);
    sk->priv[i + 3] = (uint8_t) (tmp >> 24);
  }
  uint8_t basepoint[32] = {0};
  basepoint[0] = 9; // default basepoint
  Hacl_Curve25519_crypto_scalarmult(sk->pub, sk->priv, basepoint);
}


/**
 * Derive key material for both sender and receiver
 */
void derive_key_material(struct gec_sts_ctx *ctx, uint8_t* z) {
  uint8_t tmp[PPRZ_KEY_LEN*2] = {0};
  uint8_t input[PPRZ_KEY_LEN+1] = {0};

  // Ka|| Sa = kdf(z,0)
  memcpy(input, z, PPRZ_KEY_LEN);
  input[PPRZ_KEY_LEN] = 0;
  Hacl_SHA2_512_hash(tmp, input, sizeof(input));
  memcpy(ctx->theirSymmetricKey.key, tmp, PPRZ_KEY_LEN); // K_a
  memcpy(ctx->theirSymmetricKey.nonce, &tmp[PPRZ_KEY_LEN], PPRZ_NONCE_LEN); // S_a

  // Kb|| Sb = kdf(z,1)
  input[PPRZ_KEY_LEN] = 1;
  Hacl_SHA2_512_hash(tmp, input, sizeof(input));
  memcpy(ctx->mySymmetricKey.key, tmp, PPRZ_KEY_LEN);  // K_b
  memcpy(ctx->mySymmetricKey.nonce, &tmp[PPRZ_KEY_LEN], PPRZ_NONCE_LEN);  // S_b
}


/**
 * Decrypt a message, no AAD for now
 * if res == 0 everything OK
 */
uint32_t gec_decrypt(struct gec_sym_key *k, uint8_t *plaintext, uint8_t *ciphertext, uint8_t len, uint8_t *mac){
  uint32_t res = Hacl_Chacha20Poly1305_aead_decrypt(plaintext,
                                               ciphertext,
                                               len,
                                               mac,
                                               NULL,
                                               0,
                                               k->key,
                                               k->nonce);
  return res;
}

/**
 * Encrypt a message, no AAD for now
 * if res == 0 everything OK
 */
uint32_t gec_encrypt(struct gec_sym_key *k, uint8_t *ciphertext, uint8_t *plaintext, uint8_t len, uint8_t *mac) {
  uint32_t res = Hacl_Chacha20Poly1305_aead_encrypt(ciphertext,  // ciphertext
                                               mac,  // mac
                                               plaintext,  // plaintext
                                               len,  // plaintext len
                                               NULL,  // aad
                                               0,  // aad len
                                               k->key,  // key
                                               k->nonce);  // nonce
  return res;
}


