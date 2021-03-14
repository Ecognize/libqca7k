/*
* Copyright 2021 Ecognize.me OÜ
*
* Licensed under the EUPL, Version 1.2 or – as soon they
* will be approved by the European Commission - subsequent
* versions of the EUPL (the "Licence");
* You may not use this work except in compliance with the
* Licence.
* You may obtain a copy of the Licence at:
*
* https://joinup.ec.europa.eu/software/page/eupl
*
* Unless required by applicable law or agreed to in
* writing, software distributed under the Licence is
* distributed on an "AS IS" basis,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
* express or implied.
* See the Licence for the specific language governing
* permissions and limitations under the Licence.
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Register definitions from in-tech smart charging GmbH (I2SE) documenation for PLC stamp mini 2
 * NOTE: Possibly a subset of what is actually available on the chip */
/** Buffer size setup before data transfer (W) */
const uint32_t QCA7K_REG_BFR_SIZE       = 0x0100;
/** Write buffer space available (R) */ 
const uint32_t QCA7K_REG_WRBUF_SPC_AVA  = 0x0200;
/** Read buffer data available (R) */
const uint32_t QCA7K_REG_RDBUF_BYTE_AVA = 0x0300;
/** Settings register (R/W)
 * Most of the settings not known, do a read-modify-write cycle to work with it */
const uint32_t QCA7K_REG_SPI_CONFIG     = 0x0400;
/** Reason for hardware interrupt (R/W)
 * Write to confirm the interrupt */
const uint32_t QCA7K_REG_INTR_CAUSE     = 0x0C00;
/** Interrup reasons setup mask (R/W) */
const uint32_t QCA7K_REG_INTR_ENABLE    = 0x0D00;
/** Signature command to verify connectivity and endianness (R) */
const uint32_t QCA7K_REG_SIGNATURE      = 0x1A00;

/* Settings (not exhaustive) */
/** Reset the device */
const uint32_t QCA7K_SLAVE_RESET_BIT    = 1 << 6;

/* Interrupt reasons */
/** Device performed a startup */
const uint32_t QCA7K_INT_CPU_ON         = 1 << 6;
/** Write buffer error */
const uint32_t QCA7K_INT_WRBUF_ERR      = 1 << 2;
/** Read buffer error */
const uint32_t QCA7K_INT_RDBUF_ERR      = 1 << 1;
/** Data available to read */
const uint32_t QCA7K_INT_PKT_AVLBL      = 1 << 0;

/** Signature value */
const uint32_t QCA7K_SIGNATURE          = 0xAA55;

/** Maximum frame size */
const size_t QCA7K_FRAME_MAX            = 1522;
/** Minimum frame size (will be padded) */
const size_t QCA7K_FRAME_MIN            = 60;

/** Start of Frame (repeated 4 times)  */
const uint8_t QCA7K_SOF                 = 0xAA;
/** Padding bytes */
const uint8_t QCA7K_RESERVED            = 0x00;
/** End of Frame (repeated 2 times) */
const uint8_t QCA7K_EOF                 = 0x55;

/* Error and state codes */
typedef enum
{
    /** Everything is fine */
    QCA7K_OK = 0,
    /** Signature mismatch */
    QCA7K_BAD_SIGNATURE,
    /** Frame too big */
    QCA7K_FRAME_OVERFLOW,
    /** Not enough write buffer space, retry later */
    QCA7K_WRITE_BUFFER_INSUFFICIENT,
    /** NULL pointer supplied for receiving a frame */
    QCA7K_NULL_RECV_BUFFER,
    /** Nothing in the read buffer */
    QCA7K_EMPTY_READ_BUFFER,
    /** The state machine got confused, report this error to me */
    QCA7K_INTERNAL_ERROR,
    /** Waiting for SOF */
    QCA7K_READING_SOF,
    /** Waiting for frame length */
    QCA7K_READING_FL,
    /** Waiting for padding */
    QCA7K_READING_RESERVED,
    /** We have frame bytes to read */
    QCA7K_READING_FRAME,
    /** Reading End of Frame */
    QCA7K_READING_EOF,
} qca7k_state_t;

/* High level interface */
/** Enable all interrupts */
void qca7k_interrupts_enable_all();

/** Enable specific interrupts
 * @param mask  interrrupt mask
 */
void qca7k_interrupts_enable(uint16_t mask);

/** Disable all interrupts */
void qca7k_interrupts_disable_all();

/** Disable specific interrupts
 * @param mask  interrrupt mask
 */
void qca7k_interrupts_disable(uint16_t mask);

/* Interrupt reason mask
 * NOTE: does the full interrupt sequence by disabling all interrupts, getting and confirming the reason
 * NOTE: re-enable interrupts on your own after handling and interrupt
 */
uint16_t qca7k_interrupt_reasons();

/** Request device signature in host byte order */
uint16_t qca7k_signature();

/** Recommended startup sequence
 * Checks the signature and enables all interrupts
 * Recommended to be executed after handling the QCA7K_INT_CPU_ON interrupt
 * NOTE: make sure SPI is up and running by this point
 * @return      QCA7K_OK on success, error code otherwise
 */
qca7k_state_t qca7k_startup();

/** Reset the device */
void qca7k_reset();

/** Send a frame
 * @param data  data to transmit
 * @param size  length of data
 * @return      QCA7K_OK on success, error code otherwise
 */
qca7k_state_t qca7k_send(uint8_t* data, size_t size);

/** Receive a frame
 * The operation may not finish in a single run, keep running it with the same storage pointer on interrupt
 * If run with a different pointer mid-reading, the current packet will be discarded
 * NOTE: this function is not reentrant, make sure it's only called from one place
 * @param data  pointer to storage, must have at least QCA7K_FRAME_MAX bytes allocated
 * @return      QCA7K_OK if full frame is received, error or state code otherwise
 */
qca7k_state_t qca7k_recv(uint8_t* data);

/* Shims the user is expected to provide */
/** Write a byte over SPI */
void qca7k_spi_write(uint8_t);

/** Read a byte from SPI */
uint8_t qca7k_spi_read();

/** Begin an SPI transaction (pull CS low) */
void qca7k_spi_begin();

/** End an SPI transaction (release CS) */
void qca7k_spi_end();

/* Low level interface, you probably don't need to use it */
/** Write a command header
 * @param rw    read (true) or write (false)
 * @param in    internal (true) or external (false)
 * @param reg   register, overriden to 0 for external commands
 */
void qca7k_write_command(bool rw, bool in, uint16_t reg);

/** Read a register value
 * @return      register value in host byte order (16 bit)
 */
uint16_t qca7k_read_register();

/** Write a register value
 * @param val   register value in host byte order (16 bit)
 */
void qca7k_write_register(uint16_t val);

/** Get current interrupt mask
 * @return      mask of enabled interrupts
 */
uint16_t qca7k_interrupts_get();

/** Set and overwrite current interrupt mask
 * @param mask  mask of interrupts to set
 */
void qca7k_interrupts_set(uint16_t mask);

#ifdef __cplusplus
}
#endif
