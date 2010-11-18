#ifndef SPISTREAM_PROTOCOL_H__
#define SPISTREAM_PROTOCOL_H__

#include <string.h>

#ifndef SPISTREAM_MAX_MESSAGE_LENGTH
#define SPISTREAM_MAX_MESSAGE_LENGTH 720
#endif

#ifndef SPISTREAM_MAX_RX_MESSAGE_LENGTH
#define SPISTREAM_MAX_RX_MESSAGE_LENGTH SPISTREAM_MAX_MESSAGE_LENGTH
#endif
#ifndef SPISTREAM_RX_BUFFER_SIZE
#define SPISTREAM_RX_BUFFER_SIZE SPISTREAM_MAX_RX_MESSAGE_LENGTH
#endif

#ifndef SPISTREAM_MAX_TX_MESSAGE_LENGTH
#define SPISTREAM_MAX_TX_MESSAGE_LENGTH SPISTREAM_MAX_MESSAGE_LENGTH
#endif
#ifndef SPISTREAM_MAX_TX_PARALLEL_TRANSACTIONS
#define SPISTREAM_MAX_TX_PARALLEL_TRANSACTIONS 4
#endif
#ifndef SPISTREAM_TX_MAX_BUFFER_PACKAGES
#define SPISTREAM_TX_MAX_BUFFER_PACKAGES ( \
		(SPISTREAM_MAX_TX_MESSAGE_LENGTH / \
			SPISTREAM_PACKAGE_SIZE) * \
		SPISTREAM_MAX_TX_PARALLEL_TRANSACTIONS)
#endif

#define SPISTREAM_INVALID_MESSAGE_ID 0

struct spistream_state_t {
    uint8_t tx_message_cnt;     // message cnt of next message to be sent
    uint8_t rx_package_cntd;
};

struct spistream_message_range_t {
	uint8_t index;
	uint8_t size;
};

struct spistream_buffers_t {
		uint16_t rx_num_packages; // number of packages in buffer

		uint16_t tx_insert;       // next index for package insertion
		uint16_t tx_read;         // next index to read package from
		uint16_t tx_num_packages; // number of packages in buffer

// RX stores data as array
    uint8_t  rx[SPISTREAM_RX_BUFFER_SIZE];
// TX stores packages
    struct AutopilotMessagePTStream tx[SPISTREAM_TX_MAX_BUFFER_PACKAGES];
};

typedef void (*spistream_message_rx_handler_t)(uint8_t msg_id, uint8_t * data, uint16_t num_bytes);
typedef void (*spistream_message_tx_handler_t)(uint8_t msg_id);
struct spistream_config_t {
// Handler to call for processing received message
    spistream_message_rx_handler_t message_rx_handler;
// Handler to call after message transmission
    spistream_message_tx_handler_t message_tx_handler;
};

enum spistream_flag { SPISTREAM_NO_WAIT=0, SPISTREAM_WAIT_FOR_READ };

/* Function declarations */

static inline void spistream_init(spistream_message_rx_handler_t message_rx_handler,
                                  spistream_message_tx_handler_t message_tx_handler);
static inline void spistream_read_pkg(struct AutopilotMessagePTStream * pkg_in);
static inline void spistream_write_pkg(struct AutopilotMessagePTStream * pkg_out);
static inline uint8_t spistream_send_msg(uint8_t * data, uint16_t num_bytes, enum spistream_flag);

/* Definitions */

static struct spistream_state_t   spistream_state;
static struct spistream_buffers_t spistream_buffers;
static struct spistream_config_t  spistream;

static inline void spistream_init(spistream_message_rx_handler_t message_rx_handler,
                                  spistream_message_tx_handler_t message_tx_handler)
{
    memset(&spistream_buffers, 0, sizeof(struct spistream_buffers_t));
    memset(&spistream_state,   0, sizeof(struct spistream_state_t));
    spistream.message_rx_handler = message_rx_handler;
    spistream.message_tx_handler = message_tx_handler;
    spistream_buffers.rx_num_packages = 0;
    spistream_state.rx_package_cntd   = 0;
    spistream_buffers.tx_num_packages = 0;
}

/**
 * Read a single package into internal RX buffer.
 * Converts data from package domain to byte array.
 * After receiving a full message, the registered spistream.message_rx_handler
 * function is called.
 * Called on every SPI event.
 */
static inline void spistream_read_pkg(struct AutopilotMessagePTStream * pkg_in)
{
		uint8_t package_cntd;

    if(pkg_in->message_cnt == SPISTREAM_INVALID_MESSAGE_ID) {
        return;
    }

		// In the last package of every message, the package_cntd is expected to be
		// negative or 0. It indicates the number of zero-bytes that are padded to
		// the end of the message to fill a package.
		if(pkg_in->package_cntd <= 0) { package_cntd = 1; }
		else { package_cntd = pkg_in->package_cntd; }

		if(pkg_in->package_cntd >= spistream_buffers.rx_num_packages) {
			spistream_buffers.rx_num_packages = pkg_in->package_cntd;
		}

    if(spistream_state.rx_package_cntd == 0) { // Beginning of new message
        // Message length is first value of package countdown:
        spistream_buffers.rx_num_packages = package_cntd;
				spistream_state.rx_package_cntd   = package_cntd;
    }
    memcpy(spistream_buffers.rx +
						((spistream_buffers.rx_num_packages - package_cntd) *
						 SPISTREAM_PACKAGE_SIZE),
           pkg_in->pkg_data,
           SPISTREAM_PACKAGE_SIZE);

    if(pkg_in->package_cntd <= 0) {
      // Message is ready, pass to handler:
			spistream.message_rx_handler(pkg_in->message_cnt,
																	 (uint8_t *)(spistream_buffers.rx),
																	 (spistream_buffers.rx_num_packages *
																		SPISTREAM_PACKAGE_SIZE) +
																	 pkg_in->package_cntd);
			spistream_state.rx_package_cntd = 0;
   }
}

/**
 * Fill given SPI package with next package from TX buffer.
 * Called on every SPI event.
 */
static inline void spistream_write_pkg(struct AutopilotMessagePTStream * pkg_out)
{
    if(spistream_buffers.tx_num_packages == 0) {
			memset(pkg_out, 0, sizeof(struct AutopilotMessagePTStream));
      pkg_out->message_cnt = SPISTREAM_INVALID_MESSAGE_ID;
      return;
    }

		memcpy(pkg_out,
				   spistream_buffers.tx + spistream_buffers.tx_read,
				   sizeof(struct AutopilotMessagePTStream));
		if(pkg_out->package_cntd <= 0) {
			spistream.message_tx_handler(pkg_out->message_cnt);
		}

		spistream_buffers.tx_read++;
		if(spistream_buffers.tx_read >= SPISTREAM_TX_MAX_BUFFER_PACKAGES) {
			spistream_buffers.tx_read = 0;
		}

		spistream_buffers.tx_num_packages--;
}

/**
 * Enqueue given message in TX buffer.
 * This function is directly wrapped by spistream_send_message
 * at the moment.
 */
static inline uint8_t spistream_enqueue_msg(uint8_t * data,
																						uint16_t num_bytes,
																						enum spistream_flag wait_for_read)
{
	uint16_t pkg_idx, num_packages, num_padding;
	uint16_t idx;
	// Enough space in buffer?

	if(wait_for_read == SPISTREAM_NO_WAIT ||
		 spistream_buffers.tx_num_packages+1 < SPISTREAM_TX_MAX_BUFFER_PACKAGES)
	{
		spistream_state.tx_message_cnt++;
		// Message id 0 is reserved for invalid packages:
		if(spistream_state.tx_message_cnt == SPISTREAM_INVALID_MESSAGE_ID) {
			spistream_state.tx_message_cnt = 1;
		}
		// How many packages we need for this message:
		num_packages = (num_bytes / SPISTREAM_PACKAGE_SIZE);
		if(num_bytes % SPISTREAM_PACKAGE_SIZE != 0) {
			num_packages++;
		}
		// How many zero-bytes we will have at the end of the last package:
		if(num_bytes > SPISTREAM_PACKAGE_SIZE) {
			num_padding = (num_packages * SPISTREAM_PACKAGE_SIZE) - num_bytes;
		}
		else {
			num_padding = SPISTREAM_PACKAGE_SIZE - num_bytes;
		}

		pkg_idx = spistream_buffers.tx_insert;

		// Convert data to packages and add them to TX buffer:
		for(idx = 0; num_packages > 0; idx++) {
			if(idx < num_bytes) {
				spistream_buffers.tx[pkg_idx].pkg_data[idx % SPISTREAM_PACKAGE_SIZE] = data[idx];
			}
			else { // padding
				spistream_buffers.tx[pkg_idx].pkg_data[idx % SPISTREAM_PACKAGE_SIZE] = 0;
			}
			// Last byte in current package:
			if((idx % SPISTREAM_PACKAGE_SIZE) == SPISTREAM_PACKAGE_SIZE-1) {

			// Finish configuration of current package
				// Last package uses field package_cntd to indicate the number
				// of padding bytes it contains, as negative number:
				if(num_packages == 1) {
					spistream_buffers.tx[pkg_idx].package_cntd = -num_padding;
				}
				else {
					spistream_buffers.tx[pkg_idx].package_cntd = num_packages;
				}
				spistream_buffers.tx[pkg_idx].message_cnt = spistream_state.tx_message_cnt;

			// Prepare next package:
				num_packages--;
				// Increment insert pointer with ring buffer overflow:
				spistream_buffers.tx_insert++;
				if(spistream_buffers.tx_insert >= SPISTREAM_TX_MAX_BUFFER_PACKAGES) {
					spistream_buffers.tx_insert = 0;
				}
				// Continue with next package:
				pkg_idx = spistream_buffers.tx_insert;
				spistream_buffers.tx_num_packages++;
			}
		}
#if 0
printf("Enqueue finished. Buffer: \n");
		for(pkg_idx = 0; pkg_idx < spistream_buffers.tx_num_packages; pkg_idx++) {
			printf("Package %2d | %3d |: ", pkg_idx, spistream_buffers.tx[pkg_idx].package_cntd);
			for(idx = 0; idx < SPISTREAM_PACKAGE_SIZE; idx++) {
				printf("%3d ", spistream_buffers.tx[pkg_idx].pkg_data[idx]);
			}
			printf("\n");
		}
#endif

		return 1;
	}
	return 0;
}

static inline void spistream_dequeue_msg(uint8_t message_id) {
}

/**
 * Used from userland: Send num_bytes bytes from buffer over spistream.
 * Flags are:
 * - SPISTREAM_WAIT_FOR_READ:  Reject packages when TX buffer is full
 *   and return 0, otherwise enqueue message and return 1.
 * - SPISTREAM_NO_WAIT: Overwrite data if TX buffer is full, enqueue
 *   message and always return 1.
 */
static inline uint8_t spistream_send_msg(uint8_t * data,
																			   uint16_t num_bytes,
																				 enum spistream_flag wait_for_read)
{
    return spistream_enqueue_msg(data, num_bytes, wait_for_read);
}

#endif /* SPISTREAM_PROTOCOL_H__ */

