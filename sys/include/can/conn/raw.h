/*
 * Copyright (C) 2016 OTA keys S.A.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_can_conn Connection
 * @ingroup     sys_can
 * @brief       conn interface for CAN stack
 *
 * This is the user interface to send and receive raw CAN frames or ISO-TP datagrams
 *
 * @{
 *
 * @file
 * @brief       Definitions of generic CAN interface
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 *
 */

#ifndef CAN_CONN_RAW_H
#define CAN_CONN_RAW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can/can.h"
#include "can/raw.h"
#include "mbox.h"

#ifndef CONN_CAN_RAW_MBOX_SIZE
/**
 * @brief Mailbox size of a conn_can_raw_t
 */
#define CONN_CAN_RAW_MBOX_SIZE (16)
#endif

/**
 * @name flags values
 * @{
 */
#define CONN_CAN_DONTWAIT     (1)     /**< Do not wait for Tx confirmation when sending */
#define CONN_CAN_RECVONLY     (2)     /**< Do not send anything on the bus */
/** @} */

/**
 * @brief   RAW CAN connection
 */
#if defined(MODULE_CONN_CAN_RAW_MULTI) || defined(DOXYGEN)
/**
 * @brief RAW connection
 *
 * When conn_can_raw_multi module is used, this is a 'master' connection
 * which can be used to send and receive with multiple connections within
 * a single thread.
 *
 * If conn_can_raw_multi is not used, this is a simple RAW connection
 */
typedef struct conn_can_raw_master conn_can_raw_t;

/**
 * @brief RAW salve connection
 *
 * This is a slave connection which exists only when conn_can_raw_multi
 * module is used.
 */
typedef struct conn_can_raw_slave {
    struct conn_can_raw_slave *next;     /**< Next slave in the list */
    struct conn_can_raw_master *master;  /**< Master connection holding the mailbox */
    int ifnum;                           /**< Interface number of the can device */
    int flags;                           /**< Config flags for that conn object */
    size_t count;                        /**< number of filters set */
    struct can_filter *filter;           /**< list of filter */
    can_rx_data_t *rx;                   /**< Buffered rx data */
} conn_can_raw_slave_t;

/**
 * @brief RAW master connection
 */
struct conn_can_raw_master {
    /* slave fields */
    struct conn_can_raw_slave *next;     /**< First slave in the list */
    struct conn_can_raw_master *master;  /**< Master connection */
    int ifnum;                           /**< Interface number of the can device */
    int flags;                           /**< Config flags for that conn object */
    size_t count;                        /**< number of filters set */
    struct can_filter *filter;           /**< list of filter */
    can_rx_data_t *rx;                   /**< Buffered rx data */
    /* slave fields end */
    mutex_t lock;                        /**< Master lock */
    mbox_t mbox;                         /**< mailbox for the connection list */
    /** Connection list message queue */
    msg_t mbox_queue[CONN_CAN_RAW_MBOX_SIZE];
};

/**
 * @brief Initialize a slave connection
 *
 * This initializes a slave connection.
 *
 * This must be called on slave connections when conn_can_raw_multi is used.
 * Does not exist otherwise.
 *
 * @param[in]    master     the master connection
 * @param[inout] slave      the slave connection to initialize
 */
static inline void conn_can_raw_init_slave(conn_can_raw_t *master, conn_can_raw_slave_t *slave)
{
    slave->next = NULL;
    slave->master = master;
    slave->rx = NULL;
}
#else
typedef struct conn_can_raw {
    int ifnum;                 /**< Interface number of the can device */
    int flags;                 /**< Config flags for that conn object */
    size_t count;              /**< number of filters set */
    struct can_filter *filter; /**< list of filter */
    mbox_t mbox;               /**< mbox */
    /**
     * message queue
     */
    msg_t mbox_queue[CONN_CAN_RAW_MBOX_SIZE];
} conn_can_raw_t;
#endif

/**
 * @brief  Create can connection socket
 *
 * @param[inout] conn       CAN connection
 * @param[in] filter        list of filters to set
 * @param[in] count         number of filters in @p filter
 * @param[in] ifnum         can device Interface
 * @param[in] flags         conn flags to set (CONN_CAN_RECVONLY)
 *
 * @post   @p filter must remain allocated until @p conn is closed
 *
 * @return 0 if socket was successfully connected
 * @return any other negative number in case of an error
 */
int conn_can_raw_create(conn_can_raw_t *conn, struct can_filter *filter, size_t count,
                        int ifnum, int flags);

/**
 * @brief  Close can connection socket
 *
 * @param[in] conn          CAN connection
 *
 * @return 0 if conn is closed correctly
 * @return any other negative number in case of an error.
 */
int conn_can_raw_close(conn_can_raw_t *conn);

/**
 *  @brief  Generic can receive
 *
 * @param[in] conn          CAN connection
 * @param[out] frame        CAN frame to receive
 * @param[in] timeout       timeout in us, 0 for infinite
 *
 * @return the number of bytes received
 * @return any other negative number in case of an error
 */
int conn_can_raw_recv(conn_can_raw_t *conn, struct can_frame *frame, uint32_t timeout);

/**
 * @brief  Generic can send
 *
 * @param[in] conn          CAN connection
 * @param[in] frame         frame to send
 * @param[in] flags         make function blocked or not
 *                          (CONN_CAN_DONTWAIT to ignore tx confirmation)
 *
 * @return the number of bytes sent
 * @return any other negative number in case of an error
 */
int conn_can_raw_send(conn_can_raw_t *conn, const struct can_frame *frame, int flags);

/**
 * @brief  Set raw CAN filters
 *
 * If filters were already set for this connection, it first unsets the previous filters
 * and sets the new ones.
 *
 * @param[in] conn          CAN connection
 * @param[in] filter        list of filters to set
 * @param[in] count         number of filters in @p filter
 *
 * @pre    previously set filters must be allocated until the end of the call
 * @post   @p filter must remain allocated until @p conn is closed or
 * conn_can_raw_set_filter() is called
 *
 * @return 0 if can filters were successfully set
 * @return any other negative number in case of an error
 */
int conn_can_raw_set_filter(conn_can_raw_t *conn, struct can_filter *filter, size_t count);

#if defined(MODULE_CONN_CAN_RAW_MULTI) || defined(DOXYGEN)
/**
 * @brief Wait for reception from multiple connections
 *
 * @param[out] conn        RAW connection which received data
 * @param[in] master       the master connection
 * @param[in] timeout      timeout in us, 0 for infinite wait
 *
 * @return 0 if OK, < 0 if error
 */
int conn_can_raw_select(conn_can_raw_slave_t **conn, conn_can_raw_t *master, uint32_t timeout);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CAN_CONN_RAW_H */
/** @} */
