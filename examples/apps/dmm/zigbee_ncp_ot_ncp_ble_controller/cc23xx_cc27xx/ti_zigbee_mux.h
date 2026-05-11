#ifndef TI_ZIGBEE_MUX_H
#define TI_ZIGBEE_MUX_H
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Initialise the Zigbee NCP serial driver on MUX NLI 2.
 *
 * Must be called before MuxTask_create().  Registers ZigbeeMux_rxNotify as
 * the NLI_ZB receive callback, which passes every decoded byte to the
 * zb_osif_set_uart_byte_received_cb() handler once the Zigbee library has
 * registered it.
 */
void ZigbeeMux_init(void);

#ifdef __cplusplus
}
#endif
#endif /* TI_ZIGBEE_MUX_H */
