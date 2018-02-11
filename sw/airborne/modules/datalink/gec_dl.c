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

#ifdef GEC_STATUS_LED
#include "led.h" // for LED indication
#endif

#if PPRZLINK_DEFAULT_VER != 2
#error "Secure link is only for Pprzlink v 2.0"
#endif

void gec_encapsulate_and_send_msg(struct pprzlink_msg *msg, long fd);
static void gec_transport_init(struct gec_transport *t);

struct gec_transport gec_tp;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_secure_link_info(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SECURE_LINK_STATUS(trans, dev, AC_ID,
                                   (uint8_t *)&gec_tp.sts.protocol_stage,
                                   (uint8_t *)&gec_tp.sts.last_error,
                                   &gec_tp.sts.rx_counter_err,
                                   &gec_tp.sts.decrypt_err,
                                   &gec_tp.sts.encrypt_err);
}
#endif

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

/**
 * Helper function to get the relevant transport struct
 */
static struct gec_transport *get_trans(struct pprzlink_msg *msg)
{
  return (struct gec_transport *)(msg->trans->impl);
}

/**
 * Simply insert byte to the message buffer
 */
static inline void insert_byte(struct gec_transport *t, const uint8_t byte)
{
  t->tx_msg[t->tx_msg_idx] = byte;
  t->tx_msg_idx++;
}

// add a message id to the whitelist
void gec_add_to_whitelist(struct gec_whitelist *whitelist, uint8_t id)
{
  if (whitelist->init && (whitelist->idx < WHITELIST_LEN)) {
    whitelist->whitelist[whitelist->idx] = id;
    whitelist->idx++;

  } else {
    memset(whitelist, 0, WHITELIST_LEN);  // erase the whitelist
    whitelist->init = true;
    whitelist->idx = 0;
  }
}

// return true if given message id is in the whitelist
bool gec_is_in_the_whitelist(struct gec_whitelist *whitelist, uint8_t id)
{
  if (whitelist->init) {
    for (uint8_t i = 0; i < whitelist->idx; i++) {
      if (whitelist->whitelist[i] == id) {
        return true;
      }
    }
  }
  return false;
}

/**
 * Simply put bytes in the message buffer
 */
static void put_bytes(struct pprzlink_msg *msg, long fd __attribute__((unused)),
                      enum TransportDataType type __attribute__((unused)),
                      enum TransportDataFormat format __attribute__((unused)), const void *bytes,
                      uint16_t len)
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
                           enum TransportDataFormat format __attribute__((unused)), uint8_t byte,
                           const char *name __attribute__((unused)))
{
  insert_byte(get_trans(msg), byte);
}

/**
 * Return the length of a regular pprzlink message (PPRZ_STX .. CHECKSUM)
 * and add CRYPTO_OVERHEAD(20) + crypto byte(1)
 */
static uint8_t size_of(struct pprzlink_msg *msg, uint8_t len)
{
  return PPRZ_CRYPTO_OVERHEAD + 1
         + get_trans(msg)->pprz_tp.trans_tx.size_of(msg, len);
}

/**
 * Just reset the message data
 */
static void start_message(struct pprzlink_msg *msg,
                          long fd __attribute__((unused)),
                          uint8_t payload_len __attribute__((unused)))
{
  PPRZ_MUTEX_LOCK(get_trans(msg)->mtx_tx);  // lock mutex
  memset(get_trans(msg)->tx_msg, _FD, TRANSPORT_PAYLOAD_LEN);  // erase message data
  get_trans(msg)->tx_msg_idx = 0;  // reset index
}

/**
 * Report to the underlying transport
 */
static void overrun(struct pprzlink_msg *msg)
{
  get_trans(msg)->pprz_tp.trans_tx.overrun(msg);
}

/**
 * Call the underlying transport
 */
static void count_bytes(struct pprzlink_msg *msg, uint8_t bytes)
{
  get_trans(msg)->pprz_tp.trans_tx.count_bytes(msg, bytes);
}

/**
 * Check that there is enought space in the buffer.
 * Two cases:
 * if CRYPTO_OK:
 *  return the check_available_space from the underlying transport
 * else:
 *  return zero, to avoid race conditions over the message buffer (no messages are being
 *  sent before the key exchange)
 */
static int check_available_space(struct pprzlink_msg *msg, long *fd,
                                 uint16_t bytes)
{
  if (get_trans(msg)->sts.protocol_stage == CRYPTO_OK) {
    return get_trans(msg)->pprz_tp.trans_tx.check_available_space(msg, fd,
           bytes);
  }
  return 0;
}

void gec_dl_init(void)
{
#ifdef GEC_STATUS_LED
  LED_OFF(GEC_STATUS_LED);
#endif

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

/**
 * Get bytes from the message buffer (SENDER_ID .. MSG_PAYLOAD)
 * Add crypto byte
 * Attempt encryption
 * If succesfull, add counter, auth, ciphertext and tag (CRYPTO_BYTE .. TAG)
 * and wrap in pprz transport (PPRZ_STX .. CHECKSUM)
 */
static void end_message(struct pprzlink_msg *msg, long fd)
{
  switch (gec_tp.sts.protocol_stage) {
    case CRYPTO_OK:
      if (gec_encrypt_message(gec_tp.tx_msg, &gec_tp.tx_msg_idx)) {
        gec_encapsulate_and_send_msg(msg, fd);
      }
      break;
    default:
      // shouldn't be here as sending messages is not allowed until after the key exchange
      break;
  }
  // unlock mutex
  PPRZ_MUTEX_UNLOCK(get_trans(msg)->mtx_tx);
}

// Init pprz transport structure
static void gec_transport_init(struct gec_transport *t)
{
  t->trans_rx.msg_received = false;
  t->trans_tx.size_of = (size_of_t) size_of;
  t->trans_tx.check_available_space =
    (check_available_space_t) check_available_space;
  t->trans_tx.put_bytes = (put_bytes_t) put_bytes;
  t->trans_tx.put_named_byte = (put_named_byte_t) put_named_byte;
  t->trans_tx.start_message = (start_message_t) start_message;
  t->trans_tx.end_message = (end_message_t) end_message;
  t->trans_tx.overrun = (overrun_t) overrun;
  t->trans_tx.count_bytes = (count_bytes_t) count_bytes;
  t->trans_tx.impl = (void *)(t);
  PPRZ_MUTEX_INIT(t->mtx_tx);  // init mutex, check if correct pointer

  // add whitelist messages
  gec_add_to_whitelist(&(t->whitelist), KEY_EXCHANGE_MSG_ID_UAV);
  gec_add_to_whitelist(&(t->whitelist), KEY_EXCHANGE_MSG_ID_GCS);
}

/**
 * Encapsulate message data (CRYPTO_BYTE .. MSG_PAYLOAD .. TAG) with
 * pprzlink header and checksum (PPRZ_STX .. CHECKSUM).
 * and send message
 */
void gec_encapsulate_and_send_msg(struct pprzlink_msg *msg, long fd)
{
  get_trans(msg)->pprz_tp.trans_tx.start_message(msg, fd,
      get_trans(msg)->tx_msg_idx);
  get_trans(msg)->pprz_tp.trans_tx.put_bytes(msg, _FD, DL_TYPE_UINT8,
      DL_FORMAT_SCALAR, get_trans(msg)->tx_msg, get_trans(msg)->tx_msg_idx);
  get_trans(msg)->pprz_tp.trans_tx.end_message(msg, fd);
}

/**
 * Attempts message encryption
 * Adds crypto_byte, counter and tag
 * Assumes that the first two bytes of 'payload' are source_ID and dest_ID
 * and these two bytes will be authenticated
 * Returns encrypted pprzlink 2.0 message (crypto_byte .. tag) in the same
 * buffer as was the incoming message
 */
bool gec_encrypt_message(uint8_t *buf, uint8_t *payload_len)
{
  // check payload length
  if (*payload_len <= PPRZ_V2_MSG_ID) {
    return false;
  }
  // check if the key is ready
  if (!gec_tp.sts.tx_sym_key.ready) {
    return false;
  }

  // increment counter
  uint32_t counter = gec_tp.sts.tx_sym_key.counter + 1;
  // convert to bytes
  uint8_t counter_as_bytes[4];
  gec_counter_to_bytes(counter, counter_as_bytes);
  // update nonce
  memcpy(gec_tp.sts.tx_sym_key.nonce, counter_as_bytes, sizeof(uint32_t));

  // update intermediate fields
  uint8_t auth[PPRZ_AUTH_LEN];
  memcpy(auth, buf, PPRZ_AUTH_LEN);
  uint8_t tag[PPRZ_MAC_LEN];
  uint8_t ciphertext[TRANSPORT_PAYLOAD_LEN - PPRZ_CRYPTO_OVERHEAD];
  uint32_t mlen = *payload_len - PPRZ_AUTH_LEN;  // at least CLASS_BYTE and MSG_ID

  uint32_t res = Hacl_Chacha20Poly1305_aead_encrypt(ciphertext, tag,
                 &buf[PPRZ_AUTH_LEN], mlen, auth,
                 PPRZ_AUTH_LEN, gec_tp.sts.tx_sym_key.key, gec_tp.sts.tx_sym_key.nonce);

  if (res == 0) {
    memset(buf, 0, TRANSPORT_PAYLOAD_LEN);  // reset  whole buffer
    *payload_len = 0;
    buf[PPRZ_GEC_IDX] = PPRZ_MSG_TYPE_ENCRYPTED;  // CRYPTO_BYTE
    *payload_len += 1;
    memcpy(&buf[PPRZ_CNTR_IDX], counter_as_bytes, sizeof(uint32_t));  // counter
    *payload_len += sizeof(uint32_t);
    memcpy(&buf[PPRZ_AUTH_IDX], auth, PPRZ_AUTH_LEN);  // auth data
    *payload_len += PPRZ_AUTH_LEN;
    memcpy(&buf[PPRZ_CIPH_IDX], ciphertext, mlen);  // ciphertext
    *payload_len += mlen;
    memcpy(&buf[PPRZ_CIPH_IDX + mlen], tag, PPRZ_MAC_LEN);  // tag
    *payload_len += PPRZ_MAC_LEN;
    gec_tp.sts.tx_sym_key.counter = counter;  // update counter
    return true;
  } else {
    gec_tp.sts.encrypt_err++;
    return false;
  }

}

/**
 * Attemp message decryption
 * If a message is unencrypted, pass it through only if the MSG_ID is in the whitelist
 * Returns Pprzlink 2.0 message bytes (source_ID .. msg payload)
 *
 * Input: expects (CRYPTO_BYTE .. MSG_DATA .. optional TAG)
 * Output: returns stripped message (SENDER_ID .. MSG_PAYLOAD)
 */
bool gec_decrypt_message(uint8_t *buf, volatile uint8_t *payload_len)
{
  switch (buf[PPRZ_GEC_IDX]) {
    case PPRZ_MSG_TYPE_PLAINTEXT:
      // handle plaintext message
      if (*payload_len < PPRZ_PLAINTEXT_MSG_MIN_LEN) {
        return false;
      }
      // check if the message is in the whitelist
      if (gec_is_in_the_whitelist(&gec_tp.whitelist,
                                  buf[PPRZ_PLAINTEXT_MSG_ID_IDX])) {
        // return the buffer minus the crypto byte
        memmove(buf, &buf[1], *payload_len - 1);
        return true;
      }
      return false;
      break;
    case PPRZ_MSG_TYPE_ENCRYPTED:
      // attempt decryption
      if (*payload_len < PPRZ_ENCRYPTED_MSG_MIN_LEN) {
        return false;
      }
      if (!gec_tp.sts.rx_sym_key.ready) {
        // the rx key is not ready yet
        return false;
      }
      // first check the message counter
      uint32_t counter = gec_bytes_to_counter(&buf[PPRZ_CNTR_IDX]);
      // check against the saved counter
      if (counter <= gec_tp.sts.rx_sym_key.counter) {
        gec_tp.sts.rx_counter_err++;
        return false;
      }
      // update nonce with 4 counter bytes
      memcpy(gec_tp.sts.rx_sym_key.nonce, &buf[PPRZ_CNTR_IDX],
             sizeof(uint32_t));
      // update intermediate fields
      uint8_t auth[PPRZ_AUTH_LEN];
      memcpy(auth, &buf[PPRZ_AUTH_IDX], PPRZ_AUTH_LEN);
      uint8_t tag[PPRZ_MAC_LEN];
      memcpy(tag, &buf[*payload_len - PPRZ_MAC_LEN], PPRZ_MAC_LEN);
      uint8_t ciphertext[TRANSPORT_PAYLOAD_LEN - PPRZ_CRYPTO_OVERHEAD];
      // payload - AUTH_DATA(2) - TAG(16) - CRYPTO_BYTE(1) - COUNTER(4) = payload_len - 23
      uint32_t mlen = *payload_len - PPRZ_CRYPTO_OVERHEAD - 1 - PPRZ_AUTH_LEN;  // don't forget to count for CRYPTO_BYTE
      memcpy(ciphertext, &buf[PPRZ_CIPH_IDX], mlen);
      uint8_t plaintext[TRANSPORT_PAYLOAD_LEN - PPRZ_CRYPTO_OVERHEAD];

      // try decryption
      uint32_t res = Hacl_Chacha20Poly1305_aead_decrypt(plaintext, ciphertext,
                     mlen, tag, auth,
                     PPRZ_AUTH_LEN, gec_tp.sts.rx_sym_key.key,
                     gec_tp.sts.rx_sym_key.nonce);

      // res == 0 means all good
      if (res == 0) {
        memset(buf, 0, TRANSPORT_PAYLOAD_LEN);  // reset  whole buffer
        *payload_len = 0;
        memcpy(buf, auth, PPRZ_AUTH_LEN);  // copy auth data
        *payload_len += PPRZ_AUTH_LEN;
        memcpy(&buf[PPRZ_AUTH_LEN], plaintext, mlen);  // copy plaintext
        *payload_len += mlen;
        gec_tp.sts.rx_sym_key.counter = counter;  // update counter
        return true;
      } else {
        gec_tp.sts.decrypt_err++;
        return false;
      }
      break;
    default:
      break;
  }
  return false;
}

/**
 * Parse incoming message bytes (PPRZ_STX..CHCKSUM B) and returns a new decrypted message if it is available.
 * While the status != Crypto_OK no message is returned, and all logic is handled internally.
 * Returned message has format of
 * Pprzlink 2.0
 * ```ignore
 * payload[0] source SENDER_ID
 * payload[1] destination ID
 * payload[2] class/component
 * payload[3] MSG_ID
 * payload[4-end] MSG_PAYLOAD
 * ```
 * and can be directly processed by a parser
 *
 * NOTE: the KEY_EXCHANGE messages are whitelisted and thus pass through the "decryption" even if they are
 * in plaintext and after the key exchange is established. This is not a problem, because we don't act on them.
 * It would be good to limit "sending" them from other processes, but for now lets assume nobody will want to send
 * this message.
 *
 * In the future, the KEY_EXCHANGE messages will be handled in `firmware_parse_datalink_msgs()` and act upon accordingly
 * (i.e. the key renegotiation will be possible). For now, I am keeping this note here as a reminder.
 */
void gec_dl_event(void)
{
  // strip PPRZ_STX, MSG_LEN, and CHEKSUM, return (CRYPTO_BYTE..MSG_PAYLOAD)
  pprz_check_and_parse(&DOWNLINK_DEVICE.device, &gec_tp.pprz_tp,
                       gec_tp.pprz_tp.trans_rx.payload, (bool *) &gec_tp.trans_rx.msg_received);
  // we have (CRYPTO_BYTE .. MSG_PAYLOAD) in
  if (gec_tp.trans_rx.msg_received) {  // self.rx.parse_byte(b)

    switch (gec_tp.sts.protocol_stage) {
      case CRYPTO_OK:
        // decrypt message
        // if successfull return the message (sender_ID .. MSG_payload)
        if (gec_decrypt_message(gec_tp.pprz_tp.trans_rx.payload,
                                &gec_tp.pprz_tp.trans_rx.payload_len)) {
          // copy the buffer over
          // NOTE/TODO: the real payload_len is at least one byte shorter
          // but we can copy whole buffer anyway since we don't overflow
          DatalinkFillDlBuffer(gec_tp.pprz_tp.trans_rx.payload,
                               gec_tp.pprz_tp.trans_rx.payload_len);
          // pass to datalink
          DlCheckAndParse(&DOWNLINK_DEVICE.device, &gec_tp.trans_tx, dl_buffer,
                          &dl_msg_available);
        }
        break;
      case WAIT_MSG1:
        // decrypt message (checks against the whitelist and removes the crypto byte)
        // returns (SENDER_ID .. MSG_PAYLOAD
        if (gec_decrypt_message(gec_tp.pprz_tp.trans_rx.payload,
                                &gec_tp.pprz_tp.trans_rx.payload_len)) {
          gec_process_msg1(gec_tp.pprz_tp.trans_rx.payload);
        } else {
        }
        break;
      case WAIT_MSG3:
        // decrypt message (checks against the whitelist and removes the crypto byte)
        // returns (SENDER_ID .. MSG_PAYLOAD
        if (gec_decrypt_message(gec_tp.pprz_tp.trans_rx.payload,
                                &gec_tp.pprz_tp.trans_rx.payload_len)) {
          if (gec_process_msg3(gec_tp.pprz_tp.trans_rx.payload)) {
            // if OK, we are ready for an ongoing communication
            gec_tp.sts.protocol_stage = CRYPTO_OK;
#ifdef GEC_STATUS_LED
            LED_ON(GEC_STATUS_LED);
#endif
          }
        }
        break;
      default:
        // shouldn't be here unless something is really wrong
        break;
    }
    // reset flag
    gec_tp.trans_rx.msg_received = false;
  }
}

/**
 * NOTE: for RESPONDER party only
 *  Process incoming message (expected MSG1)
 *  if the right (KEY_EXCHANGE) message received with the right data (P_AE)
 *  and the right P_AE.len=PPRZ_KEY_LEN, the internal state of Sts gets updated
 *  (key derivation etc), and msg2 is prepared to be sent.
 *  Input: decrypted message (source_ID .. msg payload)
 *  If all good, it sends response message
 */
void gec_process_msg1(uint8_t *buf)
{
  if (SenderIdOfPprzMsg(buf) != 0) {
    // process only messages from GCS
    // log an error
    return;
  }

  // check message ID
  if (IdOfPprzMsg(buf) != DL_KEY_EXCHANGE_GCS) {
    gec_tp.sts.last_error = UNEXPECTED_MSG_ERROR;
    return;
  }

  // check message type
  if (DL_KEY_EXCHANGE_GCS_msg_type(buf) != P_AE) {
    gec_tp.sts.last_error = UNEXPECTED_MSG_TYPE_ERROR;
    return;
  }

  // check data length
  if (DL_KEY_EXCHANGE_GCS_msg_data_length(buf) != PPRZ_KEY_LEN) {
    gec_tp.sts.last_error = UNEXPECTED_MSG_DATA_ERROR;
    return;
  }

  // copy P_ae over
  memcpy(&gec_tp.sts.their_public_ephemeral.pub,
         DL_KEY_EXCHANGE_GCS_msg_data(buf), sizeof(struct gec_pubkey));
  gec_tp.sts.their_public_ephemeral.ready = true;

  // 2. B generates ephemeral curve25519 key pair (Pbe, Qbe).
  if (!gec_tp.sts.my_private_ephemeral.ready) {
    gec_generate_ephemeral_keys(&gec_tp.sts.my_private_ephemeral);
  }

  // 3. B computes the shared secret: z = scalar_multiplication(Qbe, Pae)
  uint8_t z[32] = { 0 };
  Hacl_Curve25519_crypto_scalarmult(z, gec_tp.sts.my_private_ephemeral.priv,
                                    gec_tp.sts.their_public_ephemeral.pub);

  // 4. B uses the key derivation function kdf(z,1) to compute Kb || Sb,
  // kdf(z,0) to compute Ka || Sa, and kdf(z,2) to compute Kclient || Sclient.
  gec_derive_key_material(&gec_tp.sts, z);

  // 5. B computes the ed25519 signature: sig = signQb(Pbe || Pae)
  uint8_t sig[PPRZ_SIGN_LEN] = { 0 };
  uint8_t pbe_concat_p_ae[PPRZ_KEY_LEN * 2] = { 0 };
  memcpy(pbe_concat_p_ae, &gec_tp.sts.my_private_ephemeral.pub, PPRZ_KEY_LEN);
  memcpy(&pbe_concat_p_ae[PPRZ_KEY_LEN], &gec_tp.sts.their_public_ephemeral.pub,
         PPRZ_KEY_LEN);
  Hacl_Ed25519_sign(sig, gec_tp.sts.my_private_key.priv, pbe_concat_p_ae,
                    PPRZ_KEY_LEN * 2);

  // 6. B computes and sends the message Pbe || Ekey=Kb,IV=Sb||zero(sig)
  uint8_t msg_data[PPRZ_KEY_LEN + PPRZ_SIGN_LEN + PPRZ_MAC_LEN] = { 0 };
  memcpy(msg_data, &gec_tp.sts.my_private_ephemeral.pub, PPRZ_KEY_LEN);

  if (Hacl_Chacha20Poly1305_aead_encrypt(&msg_data[PPRZ_KEY_LEN],
                                         &msg_data[PPRZ_KEY_LEN + PPRZ_SIGN_LEN], sig, PPRZ_SIGN_LEN, NULL, 0,
                                         gec_tp.sts.tx_sym_key.key, gec_tp.sts.tx_sym_key.nonce)) {
    gec_tp.sts.last_error = MSG1_ENCRYPT_ERROR;
    return;
  }

  // now we have to manually construct the message
  // CRYPTO BYTE
  // source_id
  // sender_id
  // class id
  // msg id
  // msg_type
  // msg_data
  gec_tp.tx_msg[PPRZ_GEC_IDX] = PPRZ_MSG_TYPE_PLAINTEXT;
  gec_tp.tx_msg_idx = 1;
  gec_tp.tx_msg[gec_tp.tx_msg_idx] = AC_ID;
  gec_tp.tx_msg_idx++;
  gec_tp.tx_msg[gec_tp.tx_msg_idx] = 0;
  gec_tp.tx_msg_idx++;
  gec_tp.tx_msg[gec_tp.tx_msg_idx] = 1;  // telemetry
  gec_tp.tx_msg_idx++;
  gec_tp.tx_msg[gec_tp.tx_msg_idx] = 239;
  gec_tp.tx_msg_idx++;
  gec_tp.tx_msg[gec_tp.tx_msg_idx] = P_BE;
  gec_tp.tx_msg_idx++;
  gec_tp.tx_msg[gec_tp.tx_msg_idx] = sizeof(msg_data);
  gec_tp.tx_msg_idx++;
  memcpy(&gec_tp.tx_msg[gec_tp.tx_msg_idx], msg_data, sizeof(msg_data));
  gec_tp.tx_msg_idx += sizeof(msg_data);

  struct pprzlink_msg msg2;
  msg2.trans = &gec_tp.pprz_tp.trans_tx;
  msg2.dev = &DOWNLINK_DEVICE.device;

  gec_encapsulate_and_send_msg(&msg2, 0);

  /*
   * Note that ideally we would use the following function call, but the problem is that
   * 'end_message()` cannot currently distinguish between encrypted and plaintext message to be sent
   * (i.e. I don't know the message ID). The second problem I am using the same buffer here as for
   * constructing a message, so a corruption of the buffer is possible.
   *
   * Since I am doing this only once, the method above is fine for the time being.
   *
   uint8_t msg_status = P_BE;
   pprz_msg_send_KEY_EXCHANGE_UAV(&gec_tp.pprz_tp.trans_tx, &DOWNLINK_DEVICE.device, AC_ID,
   &msg_status, sizeof(msg_data), msg_data);
   */

  // increment status
  gec_tp.sts.protocol_stage = WAIT_MSG3;
}

/**
 * Process incoming message (expected MSG3)
 *  if the right (KEY_EXCHANGE) message received with the right data (SIG)
 *  and the right SIG.len=PPRZ_SIGN_LEN, and the signature is verified, Ok() is returned.
 *  Input: decrypted message (source_ID .. msg payload)
 *  Returns true if this party is ready for communication
 */
bool gec_process_msg3(uint8_t *buf)
{
  if (SenderIdOfPprzMsg(buf) != 0) {
    // process only messages from GCS
    // log an error
    return false;
  }

// check message ID
  if (IdOfPprzMsg(buf) != DL_KEY_EXCHANGE_GCS) {
    gec_tp.sts.last_error = UNEXPECTED_MSG_ERROR;
    return false;
  }

// check message type
  if (DL_KEY_EXCHANGE_GCS_msg_type(buf) != SIG) {
    gec_tp.sts.last_error = UNEXPECTED_MSG_TYPE_ERROR;
    return false;
  }

// check data length
  if (DL_KEY_EXCHANGE_GCS_msg_data_length(buf)
      != (PPRZ_SIGN_LEN + PPRZ_MAC_LEN)) {
    gec_tp.sts.last_error = UNEXPECTED_MSG_DATA_ERROR;
    return false;
  }

  // 13. B decrypts the message and verifies the signature.
  uint8_t sign[PPRZ_SIGN_LEN] = { 0 };
  if (Hacl_Chacha20Poly1305_aead_decrypt(sign,
                                         DL_KEY_EXCHANGE_GCS_msg_data(buf),
                                         PPRZ_SIGN_LEN, DL_KEY_EXCHANGE_GCS_msg_data(buf) + PPRZ_SIGN_LEN, NULL, 0,
                                         gec_tp.sts.rx_sym_key.key, gec_tp.sts.rx_sym_key.nonce)) {
    gec_tp.sts.last_error = MSG3_DECRYPT_ERROR;
    return false;
  }

  // verify
  uint8_t p_ae_concat_p_be[PPRZ_KEY_LEN * 2] = { 0 };
  memcpy(p_ae_concat_p_be, &gec_tp.sts.their_public_ephemeral.pub,
         PPRZ_KEY_LEN);
  memcpy(&p_ae_concat_p_be[PPRZ_KEY_LEN], &gec_tp.sts.my_private_ephemeral.pub,
         PPRZ_KEY_LEN);
  // returns true if verified properly
  if (!Hacl_Ed25519_verify(gec_tp.sts.their_public_key.pub, p_ae_concat_p_be,
                           PPRZ_SIGN_LEN, sign)) {
    // log error and return
    gec_tp.sts.last_error = MSG3_SIGNVERIFY_ERROR;
    return false;
  }

  // all ok
  return true;
}
