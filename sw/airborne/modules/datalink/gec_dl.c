/*
 * Copyright (C) 2017 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
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
 *
 */

/** \file modules/datalink/spprz_dl.c
 *  \brief Datalink using Galois Embedded Crypto
 */

#include "gec_dl.h"
#include "modules/datalink/gec/gec.h"
#include "subsystems/datalink/datalink.h"
#include "pprzlink/messages.h"
#include <string.h> // for memset()

#if PPRZLINK_DEFAULT_VER != 2
#error "Secure link is only for Pprzlink v 2.0"
#endif

struct gec_transport gec_tp;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_secure_link_info(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SECURE_LINK_STATUS(trans, dev, AC_ID,
    (uint8_t*)&gec_tp.sts.protocol_stage,
    (uint8_t*)&gec_tp.sts.last_error,
    &gec_tp.sts.counter_err,
    &gec_tp.sts.decrypt_err,
    &gec_tp.sts.encrypt_err);
}

#endif


static struct gec_transport * get_trans(struct pprzlink_msg *msg)
{
  return (struct gec_transport *)(msg->trans->impl);
}

static inline void insert_byte(struct gec_transport *t, const uint8_t byte) {
  t->tx_msg[t->tx_msg_idx] = byte;
  t->tx_msg_idx++;
}


static void put_bytes(struct pprzlink_msg *msg,
                      long fd __attribute__((unused)),
                      enum TransportDataType type __attribute__((unused)),
                      enum TransportDataFormat format __attribute__((unused)),
                      const void *bytes, uint16_t len)
{
  const uint8_t *b = (const uint8_t *) bytes;
  int i;
  for (i = 0; i < len; i++) {
    insert_byte(get_trans(msg), b[i]);
  }
}


static void put_named_byte(struct pprzlink_msg *msg,
                           long fd __attribute__((unused)),
                           enum TransportDataType type __attribute__((unused)),
                           enum TransportDataFormat format __attribute__((unused)),
                           uint8_t byte, const char *name __attribute__((unused)))
{
  insert_byte(get_trans(msg), byte);
}


/**
 * TODO
 */
static uint8_t size_of(struct pprzlink_msg *msg, uint8_t len)
{
  // message length: payload + crypto overhead
  return len + get_trans(msg)->pprz_tp.trans_tx.size_of(msg, len);
  // + ???; TODO depends on crypto header size
}


/**
 * TODO
 */
static void start_message(struct pprzlink_msg *msg,
                          long fd __attribute__((unused)),
                          uint8_t payload_len)
{
  PPRZ_MUTEX_LOCK(get_trans(msg)->mtx_tx); // lock mutex
  memset(get_trans(msg)->tx_msg, _FD, TRANSPORT_PAYLOAD_LEN); // erase message data
  get_trans(msg)->tx_msg_idx = 0; // reset index
  // TODO add crypto header to buffer if needed
}

/**
 * TODO
 */
static void end_message(struct pprzlink_msg *msg, long fd)
{
  // TODO apply crypto on buffer here, or a part of it

  // if valid
    // encapsulated encrypted data with pprz
    get_trans(msg)->pprz_tp.trans_tx.start_message(msg, fd, get_trans(msg)->tx_msg_idx);
    for (int i = 0; i < get_trans(msg)->tx_msg_idx; i++) {
      // add byte one by one for now
      get_trans(msg)->pprz_tp.trans_tx.put_bytes(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR,
          &(get_trans(msg)->tx_msg[i]), 1);
      // TODO: replace with one put_bytes() call
    }
    get_trans(msg)->pprz_tp.trans_tx.end_message(msg, fd);

  // unlock mutex
  PPRZ_MUTEX_UNLOCK(get_trans(msg)->mtx_tx);
}


/**
 * TODO
 */
static void overrun(struct pprzlink_msg *msg)
{
  get_trans(msg)->pprz_tp.trans_tx.overrun(msg);
}

static void count_bytes(struct pprzlink_msg *msg, uint8_t bytes)
{
  get_trans(msg)->pprz_tp.trans_tx.count_bytes(msg, bytes);
}

static int check_available_space(struct pprzlink_msg *msg, long *fd, uint16_t bytes)
{
  return get_trans(msg)->pprz_tp.trans_tx.check_available_space(msg, fd, bytes);
}


// Init pprz transport structure
static void gec_transport_init(struct gec_transport *t)
{
  t->trans_rx.msg_received = false;
  t->trans_tx.size_of = (size_of_t) size_of;
  t->trans_tx.check_available_space = (check_available_space_t) check_available_space;
  t->trans_tx.put_bytes = (put_bytes_t) put_bytes;
  t->trans_tx.put_named_byte = (put_named_byte_t) put_named_byte;
  t->trans_tx.start_message = (start_message_t) start_message;
  t->trans_tx.end_message = (end_message_t) end_message;
  t->trans_tx.overrun = (overrun_t) overrun;
  t->trans_tx.count_bytes = (count_bytes_t) count_bytes;
  t->trans_tx.impl = (void *)(t);
  PPRZ_MUTEX_INIT(t->mtx_tx); // init mutex, check if correct pointer
}






void gec_process_sts_msg(struct link_device *dev, struct gec_transport *trans, uint8_t *buf)
{
  uint8_t sender_id = SenderIdOfPprzMsg(buf);
  uint8_t msg_id = IdOfPprzMsg(buf);

  if (sender_id != 0) {
    // process only messages from GCS
    // log an error
    return;
  }

  uint8_t msg_type = DL_KEY_EXCHANGE_GCS_msg_type(buf);

  switch (msg_id) {
    case DL_KEY_EXCHANGE_GCS:
      switch (trans->sts.protocol_stage) {
        case WAIT_MSG1:
          if (msg_type == P_AE) {
            respond_sts(dev, trans, buf);
          } else {
            // log an error
            trans->sts.last_error = UNEXPECTED_MSG_TYPE_ERROR;
          }
          break;
        case WAIT_MSG3:
          if (msg_type == SIG) {
            finish_sts(dev, trans, buf);
          } else {
            // log an error
            trans->sts.last_error = UNEXPECTED_MSG_TYPE_ERROR;
          }
          break;
        default:
          // INIT, WAIT_MSG2, CRYPTO_OK
          // do nothing or log an error
          trans->sts.last_error = UNEXPECTED_STS_STAGE_ERROR;
          break;
      }
      break;
    default:
      // process only KEY_EXCHANGE for now
      // log an error
      trans->sts.last_error = UNEXPECTED_MSG_ERROR;
      break;
  }
}


void gec_dl_init(void)
{
  // init pprz transport
  pprz_transport_init(&gec_tp.pprz_tp);

  // init crypto transport
  gec_transport_init(&gec_tp);

  // initialize keys
  gec_sts_init(&gec_tp.sts);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SECURE_LINK_STATUS, send_secure_link_info);
#endif
}


void gec_dl_event(void)
{
  pprz_check_and_parse(&DOWNLINK_DEVICE.device, &gec_tp.pprz_tp, gec_tp.trans_rx.payload,
                      (bool*) &gec_tp.trans_rx.msg_received);
  if (gec_tp.trans_rx.msg_received) {
    if (gec_tp.sts.protocol_stage != CRYPTO_OK) {
      // message is in plaintext, treat is as such
      // TODO: check for decryption
      // don't pass through
      gec_process_sts_msg(&DOWNLINK_DEVICE.device, &gec_tp, gec_tp.trans_rx.payload);

    } else {
      // attempt decryption, pass data if all good
      // process decrypted message if needed
      // store in dl_buffer (see macro in datalink.h)
      DatalinkFillDlBuffer(gec_tp.trans_rx.payload, TRANSPORT_PAYLOAD_LEN);

      // pass to datalink
      DlCheckAndParse(&DOWNLINK_DEVICE.device, &gec_tp.trans_tx, dl_buffer, &dl_msg_available);
    }
    // reset flag
    gec_tp.trans_rx.msg_received = false;
  }
}


/**
 * message1 = [ Pae (32 bytes) ]
 * 2. B generates ephemeral curve25519 key pair (Pbe, Qbe).
 * 3. B computes the shared secret: z = scalar_multiplication(Qbe, Pae)
 * 4. B uses the key derivation function kdf(z,1) to compute Kb || Sb,
 * kdf(z,0) to compute Ka || Sa, and kdf(z,2) to compute Kclient || Sclient.
 * 5. B computes the ed25519 signature: sig = signQb(Pbe || Pae)
 * 6. B computes and sends the message Pbe || Ekey=Kb,IV=Sb||zero(sig)
 *
 * Sends: message2 = [ Pbe (32 bytes) | Encrypted Signature (64 bytes) ] + MAC (16bytes)
 */
void respond_sts(struct link_device *dev, struct gec_transport *trans, uint8_t *buf) {
  // TODO: check message length

  // copy P_ae over
  memcpy(trans->sts.theirPublicKeyEphemeral.pub, DL_KEY_EXCHANGE_GCS_msg_data(buf), sizeof(struct gec_pubkey));

  // 2. B generates ephemeral curve25519 key pair (Pbe, Qbe).
  generate_ephemeral_keys(&trans->sts.myPrivateKeyEphemeral);

  // 3. B computes the shared secret: z = scalar_multiplication(Qbe, Pae)
  uint8_t z[32] = {0};
  Hacl_Curve25519_crypto_scalarmult(z, trans->sts.myPrivateKeyEphemeral.priv, trans->sts.theirPublicKeyEphemeral.pub);

  // 4. B uses the key derivation function kdf(z,1) to compute Kb || Sb,
  // kdf(z,0) to compute Ka || Sa, and kdf(z,2) to compute Kclient || Sclient.
  derive_key_material(&trans->sts, z);

  // 5. B computes the ed25519 signature: sig = signQb(Pbe || Pae)
  uint8_t sig[PPRZ_SIGN_LEN] = {0};
  uint8_t pbe_concat_p_ae[PPRZ_KEY_LEN*2] = {0};
  memcpy(pbe_concat_p_ae, trans->sts.myPrivateKeyEphemeral.pub, PPRZ_KEY_LEN);
  memcpy(&pbe_concat_p_ae[PPRZ_KEY_LEN], trans->sts.theirPublicKeyEphemeral.pub, PPRZ_KEY_LEN);
  Hacl_Ed25519_sign(sig, trans->sts.myPrivateKey.priv, pbe_concat_p_ae, PPRZ_KEY_LEN*2);

  // 6. B computes and sends the message Pbe || Ekey=Kb,IV=Sb||zero(sig)
  uint8_t msg_data[PPRZ_KEY_LEN + PPRZ_SIGN_LEN + PPRZ_MAC_LEN] = {0};
  memcpy(msg_data, &trans->sts.myPrivateKeyEphemeral.pub, PPRZ_KEY_LEN);
  if (gec_encrypt(&trans->sts.mySymmetricKey, &msg_data[PPRZ_KEY_LEN], sig, PPRZ_SIGN_LEN, &msg_data[PPRZ_KEY_LEN + PPRZ_SIGN_LEN])) {
    // log error here and return
    trans->sts.last_error = MSG1_ENCRYPT_ERROR;
    return;
  }
  // all good, send message and increment status
  uint8_t msg_type = P_BE;
  uint8_t nb_msg_data = PPRZ_KEY_LEN + PPRZ_SIGN_LEN + PPRZ_MAC_LEN;


  pprz_msg_send_KEY_EXCHANGE_UAV(&trans->trans_tx, dev, AC_ID, &msg_type, nb_msg_data, msg_data);

  // update protocol stage
  trans->sts.protocol_stage = WAIT_MSG3;
  // TODO: add timeout
}



/**
 * Finalize STS exchange
 *
 * message3 = [ Encrypted Signature (64 bytes) ] + MAC(16 bytes) = Ekey=Ka,IV=Sa||zero(sig)
 * where sig = signQa(Pae || Pbe)
 * 13. B decrypts the message and verifies the signature.
 */
void finish_sts(struct link_device *dev __attribute__((__unused__)),
                struct gec_transport *trans,
                uint8_t *buf){
  // TODO: check message length

  // decrypt
  uint8_t sign[PPRZ_SIGN_LEN] = {0};
  if (gec_decrypt(&trans->sts.theirSymmetricKey, sign, DL_KEY_EXCHANGE_GCS_msg_data(buf),
      PPRZ_SIGN_LEN, DL_KEY_EXCHANGE_GCS_msg_data(buf)+PPRZ_SIGN_LEN)) {
    // log error and return
    // MSG3 decrypt error
    trans->sts.last_error = MSG3_DECRYPT_ERROR;
    return;
  }

  // verify
  uint8_t pbe_concat_p_ae[PPRZ_KEY_LEN*2] = {0};
  memcpy(pbe_concat_p_ae, trans->sts.myPrivateKeyEphemeral.pub, PPRZ_KEY_LEN);
  memcpy(&pbe_concat_p_ae[PPRZ_KEY_LEN], trans->sts.theirPublicKeyEphemeral.pub, PPRZ_KEY_LEN);
  // returns true if verified properly
  if (!Hacl_Ed25519_verify(trans->sts.theirPublicKey.pub, pbe_concat_p_ae, PPRZ_SIGN_LEN, sign)) {
    // log error and return
    // MSG3 sign verify error
    trans->sts.last_error = MSG3_SIGNVERIFY_ERROR;
    return;
  }

 /*
  // update the transport
  // tx key => my symm key
  memcpy(trans->tx_key, sts.mySymmetricKey.key, PPRZ_KEY_LEN);
  // tx nonce (12 bytes, first 4 bytes will be overwritten by the counter)
  memcpy(trans->tx_nonce, sts.mySymmetricKey.nonce, PPRZ_NONCE_LEN);

  // rx key => their symm key
  memcpy(trans->rx_key, sts.theirSymmetricKey.key, PPRZ_KEY_LEN);
  // rx nonce (12 bytes, first 4 bytes will be overwritten by the counter)
  memcpy(trans->rx_nonce, sts.theirSymmetricKey.nonce, PPRZ_NONCE_LEN);
*/
  // if everything went OK, proceed to CRYPTO_OK status
  trans->sts.protocol_stage = CRYPTO_OK;
}
