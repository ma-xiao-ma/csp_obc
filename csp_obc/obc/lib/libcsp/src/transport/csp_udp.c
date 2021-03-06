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

#include <csp/csp.h>
#include <csp/arch/csp_queue.h>
#include "../csp_port.h"
#include "../csp_conn.h"

void csp_udp_new_packet(csp_conn_t * conn, csp_packet_t * packet) {

	/* 将packet根据id中的优先级 放入连接相应的接收队列当中 */
	if (csp_conn_enqueue_packet(conn, packet) < 0) {
		csp_log_error("Connection buffer queue full!");
		csp_buffer_free(packet);
		return;
	}

	/* 再将将新连接放入socket中 */
	if (conn->socket != NULL) {
		if (csp_queue_enqueue(conn->socket, &conn, 0) != CSP_QUEUE_OK) {
			csp_log_warn("Warning socket connection queue full");
			csp_close(conn);
			return;
		}

		/* 将socket指针设为空指针，防止此连接再次被放入 */
		conn->socket = NULL;
	}

}

