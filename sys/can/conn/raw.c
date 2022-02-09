/*
 * Copyright (C) 2016 OTA keys S.A.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_can_conn
 * @{
 * @file
 * @brief       Implementation of raw CAN connection
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @}
 */

#include <assert.h>
#include <errno.h>
#include <string.h>

#include "can/conn/raw.h"
#include "can/can.h"
#include "can/raw.h"
#include "timex.h"

#ifdef MODULE_CONN_CAN_RAW_MULTI
#include "utlist.h"
#endif

#define ENABLE_DEBUG 0
#include "debug.h"

#include "xtimer.h"

#define _TIMEOUT_TX_MSG_TYPE    (0x8000)
#define _TIMEOUT_RX_MSG_TYPE    (0x8001)
#define _CLOSE_CONN_MSG_TYPE    (0x8002)
#define _TIMEOUT_MSG_VALUE      (0xABCDEFAB)

#ifndef CONN_CAN_RAW_TIMEOUT_TX_CONF
#define CONN_CAN_RAW_TIMEOUT_TX_CONF (1 * US_PER_SEC)
#endif

static inline int try_put_msg(conn_can_raw_t *conn, msg_t *msg)
{
#ifdef MODULE_CONN_CAN_RAW_MULTI
    return mbox_try_put(&conn->master->mbox, msg);
#else
    return mbox_try_put(&conn->mbox, msg);
#endif
}

static inline void put_msg(conn_can_raw_t *conn, msg_t *msg)
{
#ifdef MODULE_CONN_CAN_RAW_MULTI
    mbox_put(&conn->master->mbox, msg);
#else
    mbox_put(&conn->mbox, msg);
#endif
}

static inline int try_get_msg(conn_can_raw_t *conn, msg_t *msg)
{
#ifdef MODULE_CONN_CAN_RAW_MULTI
    return mbox_try_get(&conn->master->mbox, msg);
#else
    return mbox_try_get(&conn->mbox, msg);
#endif
}

static inline void get_msg(conn_can_raw_t *conn, msg_t *msg)
{
#ifdef MODULE_CONN_CAN_RAW_MULTI
    mbox_get(&conn->master->mbox, msg);
#else
    mbox_get(&conn->mbox, msg);
#endif
}

int conn_can_raw_create(conn_can_raw_t *conn, struct can_filter *filter, size_t count,
                        int ifnum, int flags)
{
    assert(conn != NULL);
    if (ifnum < 0 || ifnum >= CAN_DLL_NUMOF) {
        memset(conn, 0, sizeof (*conn));
        conn->ifnum = -1;
        return -ENODEV;
    }

    DEBUG("conn_can_raw_create: create conn=%p, ifnum=%d flags=%d\n", (void *)conn, ifnum, flags);

#ifdef MODULE_CONN_CAN_RAW_MULTI
    DEBUG("conn_can_raw_create: conn=%p, conn->master=%p, ifnum=%d\n",
          (void *)conn, (void *)conn->master, ifnum);

    if (conn->master == conn || conn->master == NULL) {
        conn->master = conn;
        conn->master->next = NULL;
        mutex_init(&conn->master->lock);
        mutex_lock(&conn->master->lock);
        DEBUG("conn_can_raw_create: init master conn\n");
        mbox_init(&conn->master->mbox, conn->master->mbox_queue, CONN_CAN_RAW_MBOX_SIZE);
        mutex_unlock(&conn->master->lock);
    }
#else
    mbox_init(&conn->mbox, conn->mbox_queue, CONN_CAN_RAW_MBOX_SIZE);
#endif

    conn->flags = flags;
    conn->count = 0;
    conn->ifnum = ifnum;

    if (flags & CONN_CAN_RECVONLY) {
        can_opt_t opt;
        opt.opt = CANOPT_STATE;
        canopt_state_t state = CANOPT_STATE_LISTEN_ONLY;
        opt.data = &state;
        opt.data_len = sizeof(state);
        int ret = raw_can_set_can_opt(ifnum, &opt);
        if (ret < 0) {
            return ret;
        }
    }

    return conn_can_raw_set_filter(conn, filter, count);
}

int conn_can_raw_set_filter(conn_can_raw_t *conn, struct can_filter *filter, size_t count)
{
    assert(conn != NULL);
    assert(filter != NULL || count == 0);

    DEBUG("conn_can_raw_set_filter: conn=%p, filter=%p, count=%u\n",
          (void *)conn, (void *)filter, (unsigned)count);
    DEBUG("conn_can_raw_set_filter: conn->filter=%p, conn->count=%u\n",
          (void *)conn->filter, (unsigned)conn->count);

    /* unset previous filters */
    if (conn->count) {
        for (size_t i = 0; i < conn->count; i++) {
            DEBUG("conn_can_raw_set_filter: unsetting filter=0x%" PRIx32 ", mask=0x%" PRIx32 "\n",
                 conn->filter[i].can_id, conn->filter[i].can_mask);
#ifdef MODULE_CONN_CAN_RAW_MULTI
            assert(conn->master != NULL);
            raw_can_unsubscribe_rx_mbox(conn->ifnum, &conn->filter[i], &conn->master->mbox, conn);
#else
            raw_can_unsubscribe_rx_mbox(conn->ifnum, &conn->filter[i], &conn->mbox, conn);
#endif
        }
    }

    for (size_t i = 0; i < count; i++) {
        DEBUG("conn_can_raw_set_filter: setting filter=0x%" PRIx32 ", mask=0x%" PRIx32 "\n",
              filter[i].can_id, filter[i].can_mask);
#ifdef MODULE_CONN_CAN_RAW_MULTI
        assert(conn->master != NULL);
        int ret = raw_can_subscribe_rx_mbox(conn->ifnum, &filter[i], &conn->master->mbox, conn);
#else
        int ret = raw_can_subscribe_rx_mbox(conn->ifnum, &filter[i], &conn->mbox, conn);
#endif
        if (ret < 0) {
            DEBUG("conn_can_raw_set_filter: error setting filters %d\n", ret);
            for (size_t j = 0; j < i; j++) {
                DEBUG("conn_can_raw_set_filter: unsetting filter=0x%" PRIx32 ", mask=0x%" PRIx32 "\n",
                      filter[j].can_id, filter[j].can_mask);
#ifdef MODULE_CONN_CAN_RAW_MULTI
                assert(conn->master != NULL);
                raw_can_unsubscribe_rx_mbox(conn->ifnum, &filter[j], &conn->master->mbox, conn);
#else
                raw_can_unsubscribe_rx_mbox(conn->ifnum, &filter[j], &conn->mbox, conn);
#endif
            }
            return ret;
        }
    }

    conn->filter = filter;
    conn->count = count;

#ifdef MODULE_CONN_CAN_RAW_MULTI
    if (conn != conn->master) {
        mutex_lock(&conn->master->lock);
        LL_APPEND(conn->master->next, (conn_can_raw_slave_t *)conn);
        mutex_unlock(&conn->master->lock);
    }
#endif

    return 0;
}

static void _tx_conf_timeout(void *arg)
{
    conn_can_raw_t *conn = arg;
    msg_t msg;

    msg.type = _TIMEOUT_TX_MSG_TYPE;
    msg.content.value = _TIMEOUT_MSG_VALUE;

    try_put_msg(conn, &msg);
}

int conn_can_raw_send(conn_can_raw_t *conn, const struct can_frame *frame, int flags)
{
    assert(conn != NULL);

    if (conn->ifnum < 0 || conn->ifnum >= CAN_DLL_NUMOF) {
        return -ENODEV;
    }

    assert((conn->flags & CONN_CAN_RECVONLY) == 0);
    assert(frame != NULL);

    int ret = 0;
    int handle;

    DEBUG("conn_can_raw_send: conn=%p, frame=%p, flags=%d\n",
          (void *)conn, (void *)frame, flags);

    if (flags & CONN_CAN_DONTWAIT) {
        handle = ret = raw_can_send(conn->ifnum, frame, 0);
        if (ret >= 0) {
            ret = 0;
        }
    }
    else {
        xtimer_t timer;
        timer.callback = _tx_conf_timeout;
        timer.arg = conn;
        xtimer_set(&timer, CONN_CAN_RAW_TIMEOUT_TX_CONF);

        handle = raw_can_send_mbox(conn->ifnum, frame, &conn->mbox);
        if (handle < 0) {
            xtimer_remove(&timer);
            return handle;
        }

        msg_t msg;
        int timeout = 5;
        while (1) {
            get_msg(conn, &msg);
            xtimer_remove(&timer);
            switch (msg.type) {
            case CAN_MSG_TX_ERROR:
                return -EIO;
            case CAN_MSG_TX_CONFIRMATION:
                if ((int)msg.content.value == handle) {
                    DEBUG("conn_can_raw_send: frame sent correctly\n");
                    return 0;
                }
                else {
                    raw_can_abort(conn->ifnum, handle);
                    return -EINTR;
                }
                break;
            case _TIMEOUT_TX_MSG_TYPE:
                DEBUG("conn_can_raw_send: timeout\n");
                raw_can_abort(conn->ifnum, handle);
                return -ETIMEDOUT;
                break;
            default:
                DEBUG("conn_can_raw_send: unexpected msg=%x, requeing\n", msg.type);
                put_msg(conn, &msg);
                if (!timeout--) {
                    return -EINTR;
                }
                xtimer_set(&timer, CONN_CAN_RAW_TIMEOUT_TX_CONF);
                break;
            }
        }
    }

    return ret;
}

static void _rx_timeout(void *arg)
{
    conn_can_raw_t *conn = arg;
    msg_t msg;

    msg.type = _TIMEOUT_RX_MSG_TYPE;
    msg.content.value = _TIMEOUT_MSG_VALUE;

    try_put_msg(conn, &msg);
}

int conn_can_raw_recv(conn_can_raw_t *conn, struct can_frame *frame, uint32_t timeout)
{
    assert(conn != NULL);

    if (conn->ifnum < 0 || conn->ifnum >= CAN_DLL_NUMOF) {
        return -ENODEV;
    }

    assert(frame != NULL);

    int ret;
#ifdef MODULE_CONN_CAN_RAW_MULTI
    if (conn->rx)
    {
        memcpy(frame, conn->rx->data.iov_base, conn->rx->data.iov_len);
        ret = conn->rx->data.iov_len;
        raw_can_free_frame(conn->rx);
        return ret;
    }
#endif

    xtimer_t timer;

    if (timeout != 0) {
        timer.callback = _rx_timeout;
        timer.arg = conn;
        xtimer_set(&timer, timeout);
    }

    msg_t msg;
    can_rx_data_t *rx;

    while (1)
    {
        get_msg(conn, &msg);
        switch (msg.type) {
        case CAN_MSG_RX_INDICATION:
            DEBUG("conn_can_raw_recv: CAN_MSG_RX_INDICATION\n");
            rx = msg.content.ptr;
#ifdef MODULE_CONN_CAN_RAW_MULTI
            if (rx->arg != conn)
            {
                // if conn_can_raw_recv is used without preceding select(), assume the connection
                // receiving exclusively, therefore drop frames designated for other connections
                raw_can_free_frame(rx);
                break;
            }
#endif
            if (timeout != 0) {
                xtimer_remove(&timer);
            }
            memcpy(frame, rx->data.iov_base, rx->data.iov_len);
            ret = rx->data.iov_len;
            raw_can_free_frame(rx);
            return ret;
        case _TIMEOUT_RX_MSG_TYPE:
            if (msg.content.value == _TIMEOUT_MSG_VALUE) {
                ret = -ETIMEDOUT;
            }
            else {
                ret = -EINTR;
            }
            return ret;
        case _CLOSE_CONN_MSG_TYPE:
#ifdef MODULE_CONN_CAN_RAW_MULTI
            if ((msg.content.ptr == conn) || (msg.content.ptr == conn->master)) {
#else
            if (msg.content.ptr == conn) {
#endif
                if (timeout != 0) {
                    xtimer_remove(&timer);
                }
                ret = -ECONNABORTED;
            }
            else {
                ret = -EINTR;
            }
            return ret;
        default:
            put_msg(conn, &msg);
            ret = -EINTR;
            break;
        }
    }

    return ret;
}

int conn_can_raw_close(conn_can_raw_t *conn)
{
    assert(conn != NULL);

    if (conn->ifnum < 0 || conn->ifnum >= CAN_DLL_NUMOF) {
        return -ENODEV;
    }

    DEBUG("conn_can_raw_close: conn=%p\n", (void *)conn);

    if (conn->count) {
        for (size_t i = 0; i < conn->count; i++) {
            DEBUG("conn_can_raw_close: unsetting filter=0x%" PRIx32 ", mask=0x%" PRIx32 "\n",
                 conn->filter[i].can_id, conn->filter[i].can_mask);
#ifdef MODULE_CONN_CAN_RAW_MULTI
            assert(conn->master != NULL);
            raw_can_unsubscribe_rx_mbox(conn->ifnum, &conn->filter[i], &conn->master->mbox, conn);
#else
            raw_can_unsubscribe_rx_mbox(conn->ifnum, &conn->filter[i], &conn->mbox, conn);
#endif
        }
        conn->count = 0;
        msg_t msg;
        while (try_get_msg(conn, &msg)) {
            if (msg.type == CAN_MSG_RX_INDICATION) {
                DEBUG("conn_can_raw_close: incoming msg pending, freeing\n");
                raw_can_free_frame(msg.content.ptr);
            }
        }
        msg.type = _CLOSE_CONN_MSG_TYPE;
        msg.content.ptr = conn;
        try_put_msg(conn, &msg);
    }

    return 0;
}

#ifdef MODULE_CONN_CAN_RAW_MULTI
int conn_can_raw_select(conn_can_raw_slave_t **conn, conn_can_raw_t *master, uint32_t timeout)
{
    assert(master != NULL);
    assert(conn != NULL);

    int ret;

    xtimer_t timer;
    if (timeout != 0) {
        timer.callback = _rx_timeout;
        timer.arg = master;
        xtimer_set(&timer, timeout);
    }

    msg_t msg;
    can_rx_data_t *rx;

    mbox_get(&master->mbox, &msg);

    if (timeout != 0) {
        xtimer_remove(&timer);
    }
    switch (msg.type) {
    case CAN_MSG_RX_INDICATION:
        DEBUG("conn_can_raw_select: CAN_MSG_RX_INDICATION\n");
        rx = msg.content.ptr;
        *conn = rx->arg;
        (*conn)->rx = rx;
        ret = 0;
        break;
    case _TIMEOUT_RX_MSG_TYPE:
        DEBUG("conn_can_raw_select: _TIMEOUT_MSG_VALUE\n");
        if (msg.content.value == _TIMEOUT_MSG_VALUE) {
            ret = -ETIMEDOUT;
        }
        else {
            ret = -EINTR;
        }
        *conn = NULL;
        break;
    default:
        DEBUG("conn_can_raw_select: %d\n", msg.type);
        *conn = NULL;
        ret = -EINTR;
        break;
    }

    return ret;
}
#endif
