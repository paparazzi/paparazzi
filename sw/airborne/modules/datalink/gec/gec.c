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

#include "mcu_periph/rng.h"
#include "generated/keys_uav.h"
#include "../ext/hacl-c/Hacl_Ed25519.h"
#include "../ext/hacl-c/Hacl_Curve25519.h"
#include "../ext/hacl-c/Hacl_SHA2_512.h"
#include "../ext/hacl-c/Hacl_Chacha20Poly1305.h"


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
