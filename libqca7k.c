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

#include "libqca7k.h"

static volatile qca7k_state_t _g_state = QCA7K_READING_SOF;
static volatile uint8_t* _g_recv_buf_origin = NULL, * _g_recv_buf_ptr = NULL;
/** How many bytes are left to read in current state */
static volatile size_t _g_state_bytes_left = 4;
/** What is the byte we are expecting */
static volatile uint8_t _g_expected_byte = QCA7K_SOF;
/** Frame length buffer */
static volatile uint16_t _g_fl = 0;

/** Repeats the byte to form a symmetric uint16_t */
static inline uint16_t __u16(uint8_t v)
{
    return ((uint16_t)v) << 8 | (uint16_t)v;
}

void qca7k_interrupts_enable_all()
{
    qca7k_interrupts_set(QCA7K_INT_CPU_ON | QCA7K_INT_WRBUF_ERR | QCA7K_INT_RDBUF_ERR | QCA7K_INT_PKT_AVLBL);
}

void qca7k_interrupts_enable(uint16_t mask)
{
    qca7k_interrupts_set(qca7k_interrupts_get() | mask);
}

void qca7k_interrupts_disable_all()
{
    qca7k_interrupts_set(0x0000);
}

void qca7k_interrupts_disable(uint16_t mask)
{
    qca7k_interrupts_set(qca7k_interrupts_get() & ~mask);
}

uint16_t qca7k_interrupt_reasons()
{
    qca7k_interrupts_disable_all();

    qca7k_spi_begin();
    qca7k_write_command(true, true, QCA7K_REG_INTR_CAUSE);
    uint16_t reasons = qca7k_read_register();
    qca7k_spi_end();

    /* Confirming by rewriting the same value */
    qca7k_spi_begin();
    qca7k_write_command(false, true, QCA7K_REG_INTR_CAUSE);
    qca7k_write_register(reasons);
    qca7k_spi_end();

    return reasons;
}

uint16_t qca7k_signature()
{
    qca7k_spi_begin();
    qca7k_write_command(true, true, QCA7K_REG_SIGNATURE);
    uint16_t res = qca7k_read_register();
    qca7k_spi_end();

    return res;
}

qca7k_state_t qca7k_startup()
{
    /* The documentation recommends to first request a signature without checking and then re-do it */
    (void)qca7k_signature();

    if (qca7k_signature() != QCA7K_SIGNATURE)
        return QCA7K_BAD_SIGNATURE;

    qca7k_interrupts_enable_all();
    return QCA7K_OK;
}

void qca7k_reset()
{
    /* Reset is the only known bit of the config register, so no point in making a wider API */
    qca7k_spi_begin();
    qca7k_write_command(true, true, QCA7K_REG_SPI_CONFIG);
    uint16_t config = qca7k_read_register();
    qca7k_spi_end();

    qca7k_spi_begin();
    qca7k_write_command(false, true, QCA7K_REG_SPI_CONFIG);
    qca7k_write_register(config | QCA7K_SLAVE_RESET_BIT);
    qca7k_spi_end();
}

qca7k_state_t qca7k_send(uint8_t* data, size_t size)
{
    /* Straight up overflow */
    if (size > QCA7K_FRAME_MAX)
        return QCA7K_FRAME_OVERFLOW;

    /* Enlarge to minimum size if needed */
    size_t size_to_write = size < QCA7K_FRAME_MIN ? QCA7K_FRAME_MIN : size;

    /* Calculate the size needs and compare with available space */
    size_t size_needed = 4 + 2 + 2 + size_to_write + 2;
    qca7k_spi_begin();
    qca7k_write_command(true, true, QCA7K_REG_WRBUF_SPC_AVA);
    uint16_t write_available = qca7k_read_register();
    qca7k_spi_end();

    if (write_available < size_needed)
        return QCA7K_WRITE_BUFFER_INSUFFICIENT;

    /* Inform the size of the external write operation */
    qca7k_spi_begin();
    qca7k_write_command(false, true, QCA7K_REG_BFR_SIZE);
    qca7k_write_register((uint16_t)size_needed);
    qca7k_spi_end();

    /* Write actual data as external write */
    qca7k_spi_begin();
    qca7k_write_command(false, false, 0x0000);

    /* Start of Frame (double) */
    qca7k_write_register(__u16(QCA7K_SOF));
    qca7k_write_register(__u16(QCA7K_SOF));

    /* Frame length
     * NOTE: Little endian! */
    union
    {
        uint16_t val;
        uint8_t bytes[2];
    } res;
    res.val = size_to_write;
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
    uint16_t t = res.bytes[0];
    res.bytes[0] = res.bytes[1];
    res.bytes[1] = t;
#endif

    /* Reserved */
    qca7k_write_register(__u16(QCA7K_RESERVED));

    /* Frame data */
    for (size_t i = 0; i < size_to_write; i++)
        qca7k_spi_write(i < size ? data[i] : 0x00);

    /* End of frame */
    qca7k_write_register(__u16(QCA7K_EOF));
    qca7k_spi_end();

    return QCA7K_OK;
}

/** Set the state back to the "waiting for SOF" state */
static inline void qca7k_reset_state_machine(volatile uint8_t * data)
{
    _g_recv_buf_origin = data;
    _g_recv_buf_ptr = data;
    _g_state_bytes_left = 4;
    _g_expected_byte = QCA7K_SOF;
    _g_state = QCA7K_READING_SOF;
    _g_fl = 0;
}

qca7k_state_t qca7k_recv(uint8_t* data)
{
    /* Check for NULL not to confuse our logic */
    if (!data)
        return QCA7K_NULL_RECV_BUFFER;

    /* Fix the state if the last one was the end of the frame or internal error
     * Check that buffer pointer is the same or uninialized */
    if (!_g_recv_buf_origin || data != _g_recv_buf_origin || _g_state == QCA7K_OK || _g_state == QCA7K_INTERNAL_ERROR)
        qca7k_reset_state_machine(data);

    /* Check how many bytes are available for reading */
    qca7k_spi_begin();
    qca7k_write_command(true, true, QCA7K_REG_RDBUF_BYTE_AVA);
    uint16_t bytes_available = qca7k_read_register();
    qca7k_spi_end();
    if (!bytes_available)
        return QCA7K_EMPTY_READ_BUFFER;

    /* Scan the read buffer */
    qca7k_spi_begin();
    qca7k_write_command(true, false, 0x0000);
    bool retry_loop = false;
    for (size_t i = 0; i < bytes_available; i++)
    {
        /* Read a byte if we're not in retry mode */
        static uint8_t v = 0x00;
        if (!retry_loop)
            v = qca7k_spi_read();
        else 
            retry_loop = false;

        /* We assume this is never run twice in parallel so we can expect that we are in control of all the globals */
        switch (_g_state)
        {
            /* In 3 modes we are waiting for the same characters to pop up and just counting */
            case QCA7K_READING_SOF:
            case QCA7K_READING_RESERVED:
            case QCA7K_READING_EOF:
                if (_g_expected_byte != v)
                {
                    qca7k_reset_state_machine(_g_recv_buf_origin);
                    /* Re-trying the same character if it wasn't SOF mode */
                    if (_g_state != QCA7K_READING_SOF)
                    {
                        retry_loop = true;
                        i --;
                    }
                    continue;
                }
                break;

            /* In FL mode, compose the value
            * NOTE: Little Endian */
            case QCA7K_READING_FL:
                _g_fl <<= 8;
                _g_fl |= v;
                break;

            /* In frame reading mode just save data */
            case QCA7K_READING_FRAME:
                *_g_recv_buf_ptr ++ = v;
                break;

            /* This should never happen, but if it does, let's try to clean up everything */
            default:
                qca7k_reset_state_machine(NULL);
                _g_state = QCA7K_INTERNAL_ERROR;
                goto done;
        }

        /* If we made this far, the byte was accepted, check if we are at the end of the stage */
        _g_state_bytes_left --;
        if (!_g_state_bytes_left)
        {
            switch (_g_state)
            {
                case QCA7K_READING_SOF:
                    _g_state = QCA7K_READING_FL;
                    _g_state_bytes_left = 2;
                    break;

                case QCA7K_READING_FL:
                    _g_state = QCA7K_READING_RESERVED;
                    _g_state_bytes_left = 2;
                    _g_expected_byte = QCA7K_RESERVED;
                    break;

                case QCA7K_READING_RESERVED:
                    _g_state = QCA7K_READING_FRAME;
                    _g_recv_buf_ptr = _g_recv_buf_origin;
                    _g_state_bytes_left = _g_fl;
                    break;

                case QCA7K_READING_FRAME:
                    _g_state = QCA7K_READING_EOF;
                    _g_state_bytes_left = 2;
                    _g_expected_byte = QCA7K_EOF;
                    break;

                /* TODO: what happens if we don't read the full buffer? */
                case QCA7K_READING_EOF:
                    qca7k_reset_state_machine(_g_recv_buf_origin);
                    _g_state = QCA7K_OK;
                    goto done;
                    break;

                /* Will not happen but let's keep the compiler happy */
                default:
                    break;
            }
        }
    }

done:
    qca7k_spi_end();

    return _g_state;
}

void qca7k_write_command(bool rw, bool in, uint16_t reg)
{
    uint16_t res = in ? ( (reg << 2) >> 2 ) : 0x0000;
    res |= ((uint16_t) rw) << 15 | ((uint16_t) in) << 14;
    qca7k_write_register(res);
}

void qca7k_write_register(uint16_t val)
{
    union
    {
        uint16_t val;
        uint8_t bytes[2];
    } res;
    res.val = val;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    uint16_t t = res.bytes[0];
    res.bytes[0] = res.bytes[1];
    res.bytes[1] = t;
#endif
    qca7k_spi_write(res.bytes[0]);
    qca7k_spi_write(res.bytes[1]);
}

uint16_t qca7k_read_register()
{
    uint16_t res = (uint16_t)qca7k_spi_read();
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    res |= ((uint16_t)qca7k_spi_read()) << 8;
#else
    res <<= 8;
    res |= (uint16_t)qca7k_spi_read();
#endif
    return res;
}

uint16_t qca7k_interrupts_get()
{
    qca7k_spi_begin();
    qca7k_write_command(true, true, QCA7K_REG_INTR_ENABLE);
    uint16_t res= qca7k_read_register();
    qca7k_spi_end();

    return res;
}

void qca7k_interrupts_set(uint16_t mask)
{
    qca7k_spi_begin();
    qca7k_write_command(false, true, QCA7K_REG_INTR_ENABLE);
    qca7k_write_register(mask);
    qca7k_spi_end();
}
