/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (C) 2009-2019 Michael Singer <michael@a-singer.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _LIBUSB_VHCI_H
#define _LIBUSB_VHCI_H 1

#include <pthread.h>
#include <stdint.h>

#ifdef __cplusplus
#include <errno.h>
#include <string>
#include <exception>
#include <stdexcept>
#include <vector>
#include <list>
#include <queue>
#endif

#include <linux/usb-vhci.h>

#define USB_VHCI_DEVICE_FILE "/dev/usb-vhci"

#ifdef _LIB_USB_VHCI_NOTHROW
#undef _LIB_USB_VHCI_NOTHROW
#endif
#ifdef __cplusplus
extern "C" {
#define _LIB_USB_VHCI_NOTHROW 
#else
#define _LIB_USB_VHCI_NOTHROW
#endif

#define USB_VHCI_STATUS_SUCCESS                0x00000000
#define USB_VHCI_STATUS_PENDING                0x10000001
#define USB_VHCI_STATUS_SHORT_PACKET           0x10000002
#define USB_VHCI_STATUS_ERROR                  0x7ff00000
#define USB_VHCI_STATUS_CANCELED               0x30000001
#define USB_VHCI_STATUS_TIMEDOUT               0x30000002
#define USB_VHCI_STATUS_DEVICE_DISABLED        0x71000001
#define USB_VHCI_STATUS_DEVICE_DISCONNECTED    0x71000002
#define USB_VHCI_STATUS_BIT_STUFF              0x72000001
#define USB_VHCI_STATUS_CRC                    0x72000002
#define USB_VHCI_STATUS_NO_RESPONSE            0x72000003
#define USB_VHCI_STATUS_BABBLE                 0x72000004
#define USB_VHCI_STATUS_STALL                  0x74000001
#define USB_VHCI_STATUS_BUFFER_OVERRUN         0x72100001
#define USB_VHCI_STATUS_BUFFER_UNDERRUN        0x72100002
#define USB_VHCI_STATUS_ALL_ISO_PACKETS_FAILED 0x78000001

struct usb_vhci_iso_packet
{
	uint32_t offset;
	int32_t packet_length, packet_actual;
	int32_t status;
};

struct usb_vhci_urb
{
	uint64_t handle;
	uint8_t *buffer;
	struct usb_vhci_iso_packet *iso_packets;
	int32_t buffer_length, buffer_actual;
	int32_t packet_count, error_count;
	int32_t status, interval;
	uint16_t flags;
	uint16_t wValue, wIndex, wLength;
	uint8_t bmRequestType, bRequest;
	uint8_t devadr, epadr;
	uint8_t type;
};

struct usb_vhci_port_stat
{
	uint16_t status, change;
#define USB_VHCI_PORT_STAT_CONNECTION    0x0001
#define USB_VHCI_PORT_STAT_ENABLE        0x0002
#define USB_VHCI_PORT_STAT_SUSPEND       0x0004
#define USB_VHCI_PORT_STAT_OVERCURRENT   0x0008
#define USB_VHCI_PORT_STAT_RESET         0x0010
#define USB_VHCI_PORT_STAT_POWER         0x0100
#define USB_VHCI_PORT_STAT_LOW_SPEED     0x0200
#define USB_VHCI_PORT_STAT_HIGH_SPEED    0x0400
#define USB_VHCI_PORT_STAT_C_CONNECTION  0x0001
#define USB_VHCI_PORT_STAT_C_ENABLE      0x0002
#define USB_VHCI_PORT_STAT_C_SUSPEND     0x0004
#define USB_VHCI_PORT_STAT_C_OVERCURRENT 0x0008
#define USB_VHCI_PORT_STAT_C_RESET       0x0010
	uint8_t index, flags;
};
#define USB_VHCI_DATA_RATE_FULL 0
#define USB_VHCI_DATA_RATE_LOW  1
#define USB_VHCI_DATA_RATE_HIGH 2

struct usb_vhci_work
{
	union
	{
		uint64_t handle;
		struct usb_vhci_urb urb;
		struct usb_vhci_port_stat port_stat;
	} work;

	int type;
};

int usb_vhci_open(uint8_t port_count,
                  int32_t *id,
                  int32_t *usb_busnum,
                  char    **bus_id) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_close(int fd) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_fetch_work(int fd, struct usb_vhci_work *work) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_fetch_work_timeout(int fd, struct usb_vhci_work *work, int16_t timeout) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_fetch_data(int fd, const struct usb_vhci_urb *urb) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_giveback(int fd, const struct usb_vhci_urb *urb) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_port_connect(int fd, uint8_t port, uint8_t data_rate) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_port_disconnect(int fd, uint8_t port) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_port_disable(int fd, uint8_t port) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_port_resumed(int fd, uint8_t port) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_port_overcurrent(int fd, uint8_t port, uint8_t set) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_port_reset_done(int fd, uint8_t port, uint8_t enable) _LIB_USB_VHCI_NOTHROW;

// helper function for detecting relevant port stat changes issued by the kernel
uint8_t usb_vhci_port_stat_triggers(const struct usb_vhci_port_stat *stat,
                                    const struct usb_vhci_port_stat *prev) _LIB_USB_VHCI_NOTHROW;

// for converting status codes
int usb_vhci_to_errno(int32_t status, uint8_t iso_urb) _LIB_USB_VHCI_NOTHROW;
int32_t usb_vhci_from_errno(int errno, uint8_t iso_urb) _LIB_USB_VHCI_NOTHROW;
int usb_vhci_to_iso_packets_errno(int32_t status) _LIB_USB_VHCI_NOTHROW;
int32_t usb_vhci_from_iso_packets_errno(int errno) _LIB_USB_VHCI_NOTHROW;

#ifdef __cplusplus
} // extern "C"
#endif

#define usb_vhci_is_out(epadr)    !((epadr) & 0x80)
#define usb_vhci_is_in(epadr)     !!((epadr) & 0x80)
#define usb_vhci_is_iso(type)     ((type) == USB_VHCI_URB_TYPE_ISO)
#define usb_vhci_is_int(type)     ((type) == USB_VHCI_URB_TYPE_INT)
#define usb_vhci_is_control(type) ((type) == USB_VHCI_URB_TYPE_CONTROL)
#define usb_vhci_is_bulk(type)    ((type) == USB_VHCI_URB_TYPE_BULK)

#define USB_VHCI_PORT_STAT_TRIGGER_DISABLE   0x01
#define USB_VHCI_PORT_STAT_TRIGGER_SUSPEND   0x02
#define USB_VHCI_PORT_STAT_TRIGGER_RESUMING  0x04
#define USB_VHCI_PORT_STAT_TRIGGER_RESET     0x08
#define USB_VHCI_PORT_STAT_TRIGGER_POWER_ON  0x10
#define USB_VHCI_PORT_STAT_TRIGGER_POWER_OFF 0x20

#define URB_RQ_GET_STATUS         0
#define URB_RQ_CLEAR_FEATURE      1
#define URB_RQ_SET_FEATURE        3
#define URB_RQ_SET_ADDRESS        5
#define URB_RQ_GET_DESCRIPTOR     6
#define URB_RQ_SET_DESCRIPTOR     7
#define URB_RQ_GET_CONFIGURATION  8
#define URB_RQ_SET_CONFIGURATION  9
#define URB_RQ_GET_INTERFACE     10
#define URB_RQ_SET_INTERFACE     11
#define URB_RQ_SYNCH_FRAME       12

#ifdef __cplusplus
namespace usb
{
	enum urb_type
	{
		urb_type_isochronous = USB_VHCI_URB_TYPE_ISO,
		urb_type_interrupt   = USB_VHCI_URB_TYPE_INT,
		urb_type_control     = USB_VHCI_URB_TYPE_CONTROL,
		urb_type_bulk        = USB_VHCI_URB_TYPE_BULK
	};

	enum data_rate
	{
		data_rate_full = USB_VHCI_DATA_RATE_FULL,
		data_rate_low  = USB_VHCI_DATA_RATE_LOW,
		data_rate_high = USB_VHCI_DATA_RATE_HIGH
	};

	class urb
	{
	private:
		usb_vhci_urb _urb;

		void _cpy(const usb_vhci_urb& u) ;
		void _chk() ;

	public:
		urb(const urb&) ;
		urb(uint64_t handle,
		    urb_type type,
		    int32_t buffer_length,
		    uint8_t* buffer,
		    bool own_buffer,
		    int32_t iso_packet_count,
		    usb_vhci_iso_packet* iso_packets,
		    bool own_iso_packets,
		    int32_t buffer_actual,
		    int32_t status,
		    int32_t error_count,
		    uint16_t flags,
		    uint16_t interval,
		    uint8_t devadr,
		    uint8_t epadr,
		    uint8_t bmRequestType,
		    uint8_t bRequest,
		    uint16_t wValue,
		    uint16_t wIndex,
		    uint16_t wLength) ;
		urb(const usb_vhci_urb& urb) ;
		urb(const usb_vhci_urb& urb, bool own) ;
		virtual ~urb() ;
		urb& operator=(const urb&) ;

		const usb_vhci_urb* get_internal() const  { return &_urb; }
		uint64_t get_handle() const  { return _urb.handle; }
		uint8_t* get_buffer() const  { return _urb.buffer; }
		uint32_t get_iso_packet_offset(int32_t index) const  { return _urb.iso_packets[index].offset; }
		int32_t get_iso_packet_length(int32_t index) const  { return _urb.iso_packets[index].packet_length; }
		int32_t get_iso_packet_actual(int32_t index) const  { return _urb.iso_packets[index].packet_actual; }
		int32_t get_iso_packet_status(int32_t index) const  { return _urb.iso_packets[index].status; }
		uint8_t* get_iso_packet_buffer(int32_t index) const  { return _urb.buffer + _urb.iso_packets[index].offset; }
		int32_t get_buffer_length() const  { return _urb.buffer_length; }
		int32_t get_buffer_actual() const  { return _urb.buffer_actual; }
		int32_t get_iso_packet_count() const  { return _urb.packet_count; }
		int32_t get_iso_error_count() const  { return _urb.error_count; }
		int32_t get_status() const  { return _urb.status; }
		int32_t get_interval() const  { return _urb.interval; }
		uint16_t get_flags() const  { return _urb.flags; }
		uint16_t get_wValue() const  { return _urb.wValue; }
		uint16_t get_wIndex() const  { return _urb.wIndex; }
		uint16_t get_wLength() const  { return _urb.wLength; }
		uint8_t get_bmRequestType() const  { return _urb.bmRequestType; }
		uint8_t get_bRequest() const  { return _urb.bRequest; }
		uint8_t get_device_address() const  { return _urb.devadr; }
		uint8_t get_endpoint_address() const  { return _urb.epadr; }
		uint8_t get_endpoint_number() const  { return _urb.epadr & 0x07; }
		urb_type get_type() const ; }
		bool is_in() const  { return _urb.epadr & 0x80; }
		bool is_out() const ; }
		bool is_isochronous() const  == urb_type_isochronous; }
		bool is_interrupt() const  == urb_type_interrupt; }
		bool is_control() const  == urb_type_control; }
		bool is_bulk() const  == urb_type_bulk; }
		void set_status(int32_t value)  { _urb.status = value; }
		void ack() ; }
		void stall() ; }
		void set_buffer_actual(int32_t value)  { _urb.buffer_actual = value; }
		void set_iso_error_count(int32_t value)  { _urb.error_count = value; }
		void set_iso_status(int32_t index, int32_t value)  { _urb.iso_packets[index].status = value; }
		void ack_iso(int32_t index) ; }
		void stall_iso(int32_t index) ; }
		void set_iso_packet_actual(int32_t index, int32_t value)  { _urb.iso_packets[index].packet_actual = value; }
		bool is_short_not_ok() const  { return _urb.flags & USB_VHCI_URB_FLAGS_SHORT_NOT_OK; }
		bool is_zero_packet() const  { return _urb.flags & USB_VHCI_URB_FLAGS_ZERO_PACKET; }
		void set_iso_results() ;
	};

	namespace vhci
	{
		class lock
		{
		private:
			pthread_mutex_t& mutex;

			lock& operator=(const lock&) ;
			lock(const lock&) ;

		public:
			explicit lock(volatile pthread_mutex_t& m) 
			{
				pthread_mutex_lock(&mutex);
			}

			~lock() 
			{
				pthread_mutex_unlock(&mutex);
			}
		};

		class port_stat
		{
		private:
			uint16_t status;
			uint16_t change;
			uint8_t flags;

		public:
			port_stat()  { }
			port_stat(uint16_t status, uint16_t change, uint8_t flags)  :
				status(status),
				change(change),
				flags(flags) { }
			virtual ~port_stat() ;
			uint16_t get_status() const  { return status; }
			uint16_t get_change() const  { return change; }
			uint8_t get_flags()   const  { return flags; }
			void set_status(uint16_t value)  { status = value; }
			void set_change(uint16_t value)  { change = value; }
			void set_flags(uint8_t value)    { flags = value; }
			bool get_resuming() const  { return flags & USB_VHCI_PORT_STAT_FLAG_RESUMING; }
			void set_resuming(bool value) 
			{ flags = (flags & ~USB_VHCI_PORT_STAT_FLAG_RESUMING) | (value ? USB_VHCI_PORT_STAT_FLAG_RESUMING : 0); }
			bool get_connection()  const  { return status & USB_VHCI_PORT_STAT_CONNECTION; }
			bool get_enable()      const  { return status & USB_VHCI_PORT_STAT_ENABLE; }
			bool get_suspend()     const  { return status & USB_VHCI_PORT_STAT_SUSPEND; }
			bool get_overcurrent() const  { return status & USB_VHCI_PORT_STAT_OVERCURRENT; }
			bool get_reset()       const  { return status & USB_VHCI_PORT_STAT_RESET; }
			bool get_power()       const  { return status & USB_VHCI_PORT_STAT_POWER; }
			bool get_low_speed()   const  { return status & USB_VHCI_PORT_STAT_LOW_SPEED; }
			bool get_high_speed()  const  { return status & USB_VHCI_PORT_STAT_HIGH_SPEED; }
			void set_connection(bool value) 
			{ status = (status & ~USB_VHCI_PORT_STAT_CONNECTION) |  (value ? USB_VHCI_PORT_STAT_CONNECTION : 0); }
			void set_enable(bool value) 
			{ status = (status & ~USB_VHCI_PORT_STAT_ENABLE) |      (value ? USB_VHCI_PORT_STAT_ENABLE : 0); }
			void set_suspend(bool value) 
			{ status = (status & ~USB_VHCI_PORT_STAT_SUSPEND) |     (value ? USB_VHCI_PORT_STAT_SUSPEND : 0); }
			void set_overcurrent(bool value) 
			{ status = (status & ~USB_VHCI_PORT_STAT_OVERCURRENT) | (value ? USB_VHCI_PORT_STAT_OVERCURRENT : 0); }
			void set_reset(bool value) 
			{ status = (status & ~USB_VHCI_PORT_STAT_RESET) |       (value ? USB_VHCI_PORT_STAT_RESET : 0); }
			void set_power(bool value) 
			{ status = (status & ~USB_VHCI_PORT_STAT_POWER) |       (value ? USB_VHCI_PORT_STAT_POWER: 0); }
			void set_low_speed(bool value) 
			{ status = (status & ~USB_VHCI_PORT_STAT_LOW_SPEED) |   (value ? USB_VHCI_PORT_STAT_LOW_SPEED : 0); }
			void set_high_speed(bool value) 
			{ status = (status & ~USB_VHCI_PORT_STAT_HIGH_SPEED) |  (value ? USB_VHCI_PORT_STAT_HIGH_SPEED : 0); }
			bool get_connection_changed()  const  { return change & USB_VHCI_PORT_STAT_C_CONNECTION; }
			bool get_enable_changed()      const  { return change & USB_VHCI_PORT_STAT_C_ENABLE; }
			bool get_suspend_changed()     const  { return change & USB_VHCI_PORT_STAT_C_SUSPEND; }
			bool get_overcurrent_changed() const  { return change & USB_VHCI_PORT_STAT_C_OVERCURRENT; }
			bool get_reset_changed()       const  { return change & USB_VHCI_PORT_STAT_C_RESET; }
			void set_connection_changed(bool value) 
			{ change = (change & ~USB_VHCI_PORT_STAT_C_CONNECTION) |  (value ? USB_VHCI_PORT_STAT_C_CONNECTION : 0); }
			void set_enable_changed(bool value) 
			{ change = (change & ~USB_VHCI_PORT_STAT_C_ENABLE) |      (value ? USB_VHCI_PORT_STAT_C_ENABLE : 0); }
			void set_suspend_changed(bool value) 
			{ change = (change & ~USB_VHCI_PORT_STAT_C_SUSPEND) |     (value ? USB_VHCI_PORT_STAT_C_SUSPEND : 0); }
			void set_overcurrent_changed(bool value) 
			{ change = (change & ~USB_VHCI_PORT_STAT_C_OVERCURRENT) | (value ? USB_VHCI_PORT_STAT_C_OVERCURRENT : 0); }
			void set_reset_changed(bool value) 
			{ change = (change & ~USB_VHCI_PORT_STAT_C_RESET) |       (value ? USB_VHCI_PORT_STAT_C_RESET : 0); }
		};

		class work
		{
		private:
			uint8_t port;
			bool canceled;

		protected:
			work(uint8_t port) ;

		public:
			virtual ~work() ;
			uint8_t get_port() const  { return port; }
			bool is_canceled() const  { return canceled; }
			void cancel() ;
		};

		class process_urb_work : public work
		{
		private:
			usb::urb* urb;

		public:
			process_urb_work(uint8_t port, usb::urb* urb) ;
			process_urb_work(const process_urb_work&) ;
			process_urb_work& operator=(const process_urb_work&) ;
			virtual ~process_urb_work() ;
			usb::urb* get_urb() const  { return urb; }
		};

		class cancel_urb_work : public work
		{
		private:
			uint64_t handle;

		public:
			cancel_urb_work(uint8_t port, uint64_t handle) ;
			uint64_t get_handle() const  { return handle; }
		};

		class port_stat_work : public work
		{
		private:
			port_stat stat;
			uint8_t trigger_flags;

		public:
			port_stat_work(uint8_t port, const port_stat& stat) ;
			port_stat_work(uint8_t port, const port_stat& stat, const port_stat& prev) ;
			const port_stat& get_port_stat() const  { return stat; }
			uint8_t get_trigger_flags()     const  { return trigger_flags; }
			bool triggers_disable()  const  { return trigger_flags & USB_VHCI_PORT_STAT_TRIGGER_DISABLE; }
			bool triggers_suspend()  const  { return trigger_flags & USB_VHCI_PORT_STAT_TRIGGER_SUSPEND; }
			bool triggers_resuming() const  { return trigger_flags & USB_VHCI_PORT_STAT_TRIGGER_RESUMING; }
			bool triggers_reset()    const  { return trigger_flags & USB_VHCI_PORT_STAT_TRIGGER_RESET; }
			bool triggers_power_on()  const  { return trigger_flags & USB_VHCI_PORT_STAT_TRIGGER_POWER_ON; }
			bool triggers_power_off() const  { return trigger_flags & USB_VHCI_PORT_STAT_TRIGGER_POWER_OFF; }
		};

		class hcd
		{
		public:
			class callback
			{
			private:
				void (*func)(void*, hcd&) ;
				void* arg;

			public:
				callback(void (*func)(void*, hcd&) 
				{
					if(!func) throw std::invalid_argument("func");
				}

				bool operator==(const callback& other) const 
				{
					return func == other.func && arg == other.arg;
				}

				bool operator!=(const callback& other) const ; }
				void (*get_func() const  { return func; }
				void* get_arg() const  { return arg; }
				void call(hcd& from) const ; }
			};

		private:
			std::vector<callback> work_enqueued_callbacks;

			pthread_t bg_thread;
			volatile bool thread_shutdown;
			pthread_mutex_t thread_sync;

			uint8_t port_count;
			pthread_mutex_t _lock;
			std::deque<work*> inbox;
			std::list<work*> processing;

			hcd(const hcd&) ;
			hcd& operator=(const hcd&) ;

			static void* bg_thread_start(void* _this) ;

		protected:
			explicit hcd(uint8_t ports) ;
			virtual void bg_work() volatile  = 0;
			virtual uint8_t address_from_port(uint8_t port) const  = 0;
			virtual uint8_t port_from_address(uint8_t address) const  = 0;
			virtual void canceling_work(work* w, bool in_progress) ;
			virtual void finishing_work(work* w) ;
			virtual void on_work_enqueued() ;
			void enqueue_work(work* w) ;
			void init_bg_thread() volatile ;
			void join_bg_thread() volatile ;
			pthread_mutex_t& get_lock() volatile ; }
			bool is_thread_shutdown() const volatile  { return thread_shutdown; }

		public:
			virtual ~hcd() ;

			void add_work_enqueued_callback(callback c) volatile ;
			void remove_work_enqueued_callback(callback c) volatile ;
			virtual const port_stat& get_port_stat(uint8_t port) volatile  = 0;
			virtual void port_connect(uint8_t port, usb::data_rate rate) volatile  = 0;
			virtual void port_disconnect(uint8_t port) volatile  = 0;
			virtual void port_disable(uint8_t port) volatile  = 0;
			virtual void port_resumed(uint8_t port) volatile  = 0;
			virtual void port_overcurrent(uint8_t port, bool set) volatile  = 0;
			virtual void port_reset_done(uint8_t port, bool enable = true) volatile  = 0;
			uint8_t get_port_count() const volatile  { return port_count; }
			bool next_work(work** w) volatile ;
			void finish_work(work* w) volatile ;
			bool cancel_process_urb_work(uint64_t handle) volatile ;
		};

		class local_hcd : public hcd
		{
		private:
			struct _port_info
			{
				uint8_t adr;
				port_stat stat;
				_port_info()  { }
				_port_info(uint8_t adr, const port_stat& stat)  { }
			};

			int fd;
			int32_t id, usb_bus_num;
			std::string bus_id;
			_port_info* port_info;

			local_hcd(const local_hcd&) ;
			local_hcd& operator=(const local_hcd&) ;

		protected:
			virtual uint8_t address_from_port(uint8_t port) const ;
			virtual uint8_t port_from_address(uint8_t address) const ;
			virtual void canceling_work(work* w, bool in_progress) ;
			virtual void finishing_work(work* w) ;

		public:
			explicit local_hcd(uint8_t ports) ;
			virtual ~local_hcd() ;

			int32_t get_vhci_id() volatile  { return id; }
			const std::string& get_bus_id() volatile ; }
			int32_t get_usb_bus_num() volatile  { return usb_bus_num; }
			virtual void bg_work() volatile ;
			virtual const port_stat& get_port_stat(uint8_t port) volatile ;
			virtual void port_connect(uint8_t port, usb::data_rate rate) volatile ;
			virtual void port_disconnect(uint8_t port) volatile ;
			virtual void port_disable(uint8_t port) volatile ;
			virtual void port_resumed(uint8_t port) volatile ;
			virtual void port_overcurrent(uint8_t port, bool set) volatile ;
			virtual void port_reset_done(uint8_t port, bool enable = true) volatile ;
		};
	}
}
#endif // __cplusplus

#endif // _LIBUSB_VHCI_H

