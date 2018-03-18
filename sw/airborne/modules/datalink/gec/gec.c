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

#if defined(__linux__) || defined(__CYGWIN__)
#include <arpa/inet.h>
#endif

void gec_sts_init(struct gec_sts_ctx *sts)
{
  // reset all keys
  gec_clear_sts(sts);

  // load their public key
  uint8_t theirPublicKey[PPRZ_KEY_LEN] = GCS_PUBLIC;
  memcpy(&sts->their_public_key, theirPublicKey, PPRZ_KEY_LEN);
  sts->their_public_key.ready = true;

  // load my private key
  uint8_t myPublicKey[PPRZ_KEY_LEN] = UAV_PUBLIC;
  memcpy(&sts->my_private_key.pub, myPublicKey, PPRZ_KEY_LEN);
  uint8_t myPrivateKey[PPRZ_KEY_LEN] = UAV_PRIVATE;
  memcpy(&sts->my_private_key.priv, myPrivateKey, PPRZ_KEY_LEN);
  sts->my_private_key.ready = true;

  // generate ephemeral key
  gec_generate_ephemeral_keys(&sts->my_private_ephemeral);
}

void gec_clear_sts(struct gec_sts_ctx *sts)
{
  memset(&sts->their_public_ephemeral, 0, sizeof(struct gec_pubkey));
  sts->their_public_ephemeral.ready = false;
  memset(&sts->my_private_ephemeral, 0, sizeof(struct gec_privkey));
  sts->my_private_ephemeral.ready = false;
  memset(&sts->rx_sym_key, 0, sizeof(struct gec_sym_key));
  sts->rx_sym_key.ready = false;
  memset(&sts->tx_sym_key, 0, sizeof(struct gec_sym_key));
  sts->tx_sym_key.ready = false;

  sts->protocol_stage = WAIT_MSG1;
  sts->party = RESPONDER;
  sts->last_error = ERROR_NONE;
}

/**
 * Generate private and public key pairs for future use.
 */
void gec_generate_ephemeral_keys(struct gec_privkey *sk)
{
  for (uint16_t i = 0; i < PPRZ_KEY_LEN; i += sizeof(uint32_t)) {
    uint32_t tmp = rng_wait_and_get();
    sk->priv[i] = (uint8_t) tmp;
    sk->priv[i + 1] = (uint8_t)(tmp >> 8);
    sk->priv[i + 2] = (uint8_t)(tmp >> 16);
    sk->priv[i + 3] = (uint8_t)(tmp >> 24);
  }
  uint8_t basepoint[32] = { 0 };
  basepoint[0] = 9;  // default basepoint
  Hacl_Curve25519_crypto_scalarmult(sk->pub, sk->priv, basepoint);
  sk->ready = true;
}

/**
 * Derive key material for both sender and receiver
 */
void gec_derive_key_material(struct gec_sts_ctx *ctx, uint8_t *z)
{
  uint8_t tmp[PPRZ_KEY_LEN * 2] = { 0 };
  uint8_t input[PPRZ_KEY_LEN + 1] = { 0 };

  // Ka|| Sa = kdf(z,0)
  memcpy(input, z, PPRZ_KEY_LEN);
  input[PPRZ_KEY_LEN] = 0;
  Hacl_SHA2_512_hash(tmp, input, sizeof(input));
  memcpy(ctx->rx_sym_key.key, tmp, PPRZ_KEY_LEN);  // K_a
  memcpy(ctx->rx_sym_key.nonce, &tmp[PPRZ_KEY_LEN], PPRZ_NONCE_LEN);  // S_a
  ctx->rx_sym_key.counter = 0;
  ctx->rx_sym_key.ready = true;

  // Kb|| Sb = kdf(z,1)
  input[PPRZ_KEY_LEN] = 1;
  Hacl_SHA2_512_hash(tmp, input, sizeof(input));
  memcpy(ctx->tx_sym_key.key, tmp, PPRZ_KEY_LEN);  // K_b
  memcpy(ctx->tx_sym_key.nonce, &tmp[PPRZ_KEY_LEN], PPRZ_NONCE_LEN);  // S_b
  ctx->tx_sym_key.counter = 0;
  ctx->tx_sym_key.ready = true;
}

/**
 * Convert from network byte order (big endian) to the machine byte order
 */
uint32_t gec_bytes_to_counter(uint8_t *bytes)
{
  // assume big endian
  uint32_t x = (bytes[3] << 24) + (bytes[2] << 16) + (bytes[1] << 8) + bytes[0];
  // now check the appropriate endiannes
  /* ... for Linux */
#if defined(__linux__) || defined(__CYGWIN__)
  return ntohl(x);
  /* ... generic big-endian fallback code */
#elif defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  // the byte order is the same as for network
  return x;
  /* ... generic little-endian fallback code */
#elif defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  // do a byte swap
  return htobe32(x);
  /* ... couldn't determine endian-ness of the target platform */
#else
#pragma message "Please define __BYTE_ORDER__!"
#endif /* defined(__linux__) || ... */
}

/**
 * Convert counter to bytes in network byte order
 */
void gec_counter_to_bytes(uint32_t n, uint8_t *bytes)
{
  //first account for appropriate endiannes
  /* ... for Linux */
#if defined(__linux__) || defined(__CYGWIN__)
  n = htonl(n);
  /* ... generic big-endian fallback code */
#elif defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  // the byte order is the same as for network
  // do nothing
  /* ... generic little-endian fallback code */
#elif defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  // do a byte swap
  n = htobe32(n);
  /* ... couldn't determine endian-ness of the target platform */
#else
#pragma message "Error: Please define __BYTE_ORDER__!"
#endif /* defined(__linux__) || ... */

  bytes[3] = (n >> 24) & 0xFF;
  bytes[2] = (n >> 16) & 0xFF;
  bytes[1] = (n >> 8) & 0xFF;
  bytes[0] = n & 0xFF;
}
