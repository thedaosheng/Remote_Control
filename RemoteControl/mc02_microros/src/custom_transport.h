/**
 * @file custom_transport.h
 * @brief micro-ROS 自定义 USB CDC 传输层头文件
 */

#ifndef CUSTOM_TRANSPORT_H
#define CUSTOM_TRANSPORT_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#ifdef __cplusplus
extern "C" {
#endif

bool custom_transport_open(struct uxrCustomTransport *transport);
bool custom_transport_close(struct uxrCustomTransport *transport);
size_t custom_transport_write(struct uxrCustomTransport *transport,
                               const uint8_t *buf, size_t len, uint8_t *err);
size_t custom_transport_read(struct uxrCustomTransport *transport,
                              uint8_t *buf, size_t len, int timeout, uint8_t *err);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_TRANSPORT_H */
