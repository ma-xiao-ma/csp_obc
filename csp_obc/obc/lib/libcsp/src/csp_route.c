/*
Cubesat Space Protocol - A small network-layer protocol designed for Cubesats
Copyright (C) 2012 GomSpace ApS (http://www.gomspace.com)
Copyright (C) 2012 AAUSAT3 Project (http://aausat3.space.aau.dk) 

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>

/* CSP includes */
#include <csp/csp.h>
#include <csp/csp_crc32.h>
#include <csp/csp_endian.h>

#include <csp/arch/csp_thread.h>
#include <csp/arch/csp_queue.h>

#include "crypto/csp_hmac.h"
#include "crypto/csp_xtea.h"

#include "csp_port.h"
#include "csp_conn.h"
#include "csp_io.h"
#include "csp_promisc.h"
#include "csp_qfifo.h"
#include "csp_dedup.h"
#include "transport/csp_transport.h"

/**
 * Check supported packet options
 * @param interface pointer to incoming interface
 * @param packet pointer to packet
 * @return CSP_ERR_NONE is all options are supported, CSP_ERR_NOTSUP if not
 */
static int csp_route_check_options(csp_iface_t *interface, csp_packet_t *packet)
{
#ifndef CSP_USE_XTEA
	/* Drop XTEA packets */
	if (packet->id.flags & CSP_FXTEA) {
		csp_log_error("Received XTEA encrypted packet, but CSP was compiled without XTEA support. Discarding packet");
		interface->autherr++;
		return CSP_ERR_NOTSUP;
	}
#endif

#ifndef CSP_USE_HMAC
	/* Drop HMAC packets */
	if (packet->id.flags & CSP_FHMAC) {
		csp_log_error("Received packet with HMAC, but CSP was compiled without HMAC support. Discarding packet");
		interface->autherr++;
		return CSP_ERR_NOTSUP;
	}
#endif

#ifndef CSP_USE_RDP
	/* Drop RDP packets */
	if (packet->id.flags & CSP_FRDP) {
		csp_log_error("Received RDP packet, but CSP was compiled without RDP support. Discarding packet");
		interface->rx_error++;
		return CSP_ERR_NOTSUP;
	}
#endif
	return CSP_ERR_NONE;
}

/**
 * Helper function to decrypt, check auth and CRC32
 * @param security_opts either socket_opts or conn_opts
 * @param interface pointer to incoming interface
 * @param packet pointer to packet
 * @return -1 Missing feature, -2 XTEA error, -3 CRC error, -4 HMAC error, 0 = OK.
 */
static int csp_route_security_check(uint32_t security_opts, csp_iface_t * interface, csp_packet_t * packet) {

#ifdef CSP_USE_XTEA
	/* XTEA encrypted packet */
	if (packet->id.flags & CSP_FXTEA) {
		/* Read nonce */
		uint32_t nonce;
		memcpy(&nonce, &packet->data[packet->length - sizeof(nonce)], sizeof(nonce));
		nonce = csp_ntoh32(nonce);
		packet->length -= sizeof(nonce);

		/* Create initialization vector */
		uint32_t iv[2] = {nonce, 1};

		/* Decrypt data */
		if (csp_xtea_decrypt(packet->data, packet->length, iv) != 0) {
			/* Decryption failed */
			csp_log_error("Decryption failed! Discarding packet");
			interface->autherr++;
			return CSP_ERR_XTEA;
		}
	} else if (security_opts & CSP_SO_XTEAREQ) {
		csp_log_warn("Received packet without XTEA encryption. Discarding packet");
		interface->autherr++;
		return CSP_ERR_XTEA;
	}
#endif

	/* CRC32 verified packet */
	if (packet->id.flags & CSP_FCRC32) {
#ifdef CSP_USE_CRC32
		if (packet->length < 4)
			csp_log_error("Too short packet for CRC32, %u", packet->length);
		/* Verify CRC32 (does not include header for backwards compatability with csp1.x) */
		if (csp_crc32_verify(packet, false) != 0) {
			/* Checksum failed */
			csp_log_error("CRC32 verification error! Discarding packet");
			interface->rx_error++;
			return CSP_ERR_CRC32;
		}
	} else if (security_opts & CSP_SO_CRC32REQ) {
		csp_log_warn("Received packet without CRC32. Accepting packet");
#else
		/* Strip CRC32 field and accept the packet */
		csp_log_warn("Received packet with CRC32, but CSP was compiled without CRC32 support. Accepting packet");
		packet->length -= sizeof(uint32_t);
#endif
	}

#ifdef CSP_USE_HMAC
	/* HMAC authenticated packet */
	if (packet->id.flags & CSP_FHMAC) {
		/* Verify HMAC (does not include header for backwards compatability with csp1.x) */
		if (csp_hmac_verify(packet, false) != 0) {
			/* HMAC failed */
			csp_log_error("HMAC verification error! Discarding packet");
			interface->autherr++;
			return CSP_ERR_HMAC;
		}
	} else if (security_opts & CSP_SO_HMACREQ) {
		csp_log_warn("Received packet without HMAC. Discarding packet");
		interface->autherr++;
		return CSP_ERR_HMAC;
	}
#endif

#ifdef CSP_USE_RDP
	/* RDP packet */
	if (!(packet->id.flags & CSP_FRDP)) {
		if (security_opts & CSP_SO_RDPREQ) {
			csp_log_warn("Received packet without RDP header. Discarding packet");
			interface->rx_error++;
			return CSP_ERR_INVAL;
		}
	}
#endif

	return CSP_ERR_NONE;

}

int csp_route_work(uint32_t timeout) {

	csp_qfifo_t input;
	csp_packet_t * packet;
	csp_conn_t * conn;
	csp_socket_t * socket;

#ifdef CSP_USE_RDP
	/* 检查连接超时 (目前仅用在RDP中) */
	csp_conn_check_timeouts();
#endif

	/* 从路由器FIFO中获取一个packet */
	if (csp_qfifo_read(&input) != CSP_ERR_NONE)
		return -1;

	packet = input.packet;

	csp_log_packet("INP: S %u, D %u, Dp %u, Sp %u, Pr %u, Fl 0x%02X, Sz %"PRIu16" VIA: %s",
			packet->id.src, packet->id.dst, packet->id.dport,
			packet->id.sport, packet->id.pri, packet->id.flags, packet->length, input.interface->name);

	/* Here there be promiscuous mode */
#ifdef CSP_USE_PROMISC
	csp_promisc_add(packet);
#endif

#ifdef CSP_USE_DEDUP
	/* Check for duplicates */
	if (csp_dedup_is_duplicate(packet)) {
		/* Discard packet */
		csp_log_packet("Duplicate packet discarded");
		csp_buffer_free(packet);
		return 0;
	}
#endif

	/* 若packet不是本节点的，则将此packet路由到其他节点 */
	if ((packet->id.dst != csp_get_address()) && (packet->id.dst != CSP_BROADCAST_ADDR)) {

		/* 通过ID中的目的地址寻找发送接口设备 */
		csp_iface_t * dstif = csp_rtable_find_iface(packet->id.dst);

		/* 如果消息解析为输入接口，则不要将其循环回去 */
		if ((dstif == NULL) || ((dstif == input.interface) && (input.interface->split_horizon_off == 0))) {
			csp_buffer_free(packet);
			return 0;
		}

		/* 直接将包通过发送接口发送出去 */
		if (csp_send_direct(packet->id, packet, dstif, 0) != CSP_ERR_NONE) {
			csp_log_warn("Router failed to send");
			csp_buffer_free(packet);
		}

		/* Next message, please */
		return 0;
	}

	/* 丢弃具有不受支持的选项的数据包 */
	if (csp_route_check_options(input.interface, packet) != CSP_ERR_NONE) {
		csp_buffer_free(packet);
		return 0;
	}

	/* 消息是给本节点的，通过传入包的目的端口查找套接字 */
	socket = csp_port_get_socket(packet->id.dport);

	/* 如果查到的socket选项是无连接类型的，则直接将packet放入socket队列中 */
	if (socket && (socket->opts & CSP_SO_CONN_LESS)) {
		if (csp_route_security_check(socket->opts, input.interface, packet) < 0) {
			csp_buffer_free(packet);
			return 0;
		}
		if (csp_queue_enqueue(socket->socket, &packet, 0) != CSP_QUEUE_OK) {
			csp_log_error("Conn-less socket queue full");
			csp_buffer_free(packet);
			return 0;
		}
		return 0;
	}

	/* 查找已经存在的连接*/
	conn = csp_conn_find(packet->id.ext, CSP_ID_CONN_MASK);

	/* 如没有查找到已存在的连接 */
	if (conn == NULL) {

		/* Reject packet if no matching socket is found */
		if (!socket) {
			csp_buffer_free(packet);
			return 0;
		}

		/* Run security check on incoming packet */
		if (csp_route_security_check(socket->opts, input.interface, packet) < 0) {
			csp_buffer_free(packet);
			return 0;
		}

		/* 新的传入连接被接受 */
		csp_id_t idout;
		idout.pri   = packet->id.pri;
		idout.src   = csp_get_address();

		idout.dst   = packet->id.src;
		idout.dport = packet->id.sport;
		idout.sport = packet->id.dport;
		idout.flags = packet->id.flags;

		/* 创建新连接 */
		conn = csp_conn_new(packet->id, idout);

		if (!conn) {
			csp_log_error("No more connections available");
			csp_buffer_free(packet);
			return 0;
		}

		/* 存储socket队列和选项*/
		conn->socket = socket->socket;
		conn->opts = socket->opts;

	/* Packet to existing connection */
	} else {

		/* Run security check on incoming packet */
		if (csp_route_security_check(conn->opts, input.interface, packet) < 0) {
			csp_buffer_free(packet);
			return 0;
		}

	}

#ifdef CSP_USE_RDP
	/* 将数据包传递给RDP模块 */
	if (packet->id.flags & CSP_FRDP) {
		csp_rdp_new_packet(conn, packet);
		return 0;
	}
#endif

	/* 将数据包传递给RDP模块  */
	csp_udp_new_packet(conn, packet);
	return 0;
}

CSP_DEFINE_TASK(csp_task_router) {

	/* Here there be routing */
	while (1) {
		csp_route_work(FIFO_TIMEOUT);
	}

}

int csp_route_start_task(unsigned int task_stack_size, unsigned int priority) {

	static csp_thread_handle_t handle_router;
	int ret = csp_thread_create(csp_task_router, "RTE", task_stack_size, NULL, priority, &handle_router);

	if (ret != 0) {
		csp_log_error("Failed to start router task");
		return CSP_ERR_NOMEM;
	}

	return CSP_ERR_NONE;

}