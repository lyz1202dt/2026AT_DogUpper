#include "usb_cdc.hpp"
#include <libusb-1.0/libusb.h>

CDCDevice::CDCDevice() {
    _disconnected     = true;
    _handling_events  = true;
    _need_reconnected = false;
    interfaces_num    = 0;
    libusb_init(&ctx);
}

CDCDevice::~CDCDevice() {
    _handling_events = false;
    close();
    libusb_exit(ctx);
}

bool CDCDevice::open(uint16_t vid, uint16_t pid, int interfaces_num) {
    last_vid             = vid;
    last_pid             = pid;
    this->interfaces_num = interfaces_num;

    handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
    if (!handle)
        return false;
    if (libusb_kernel_driver_active(handle, interfaces_num))
        libusb_detach_kernel_driver(handle, interfaces_num);
    libusb_claim_interface(handle, interfaces_num); // 获取通道1

    // 分配异步传输结构体
    recv_transfer = libusb_alloc_transfer(0);

    // 填写异步传输结构体参数
    libusb_fill_bulk_transfer(
        recv_transfer, handle, EP_IN, reinterpret_cast<uint8_t*>(cdc_rx_buffer),
        sizeof(cdc_rx_buffer),
        [](libusb_transfer* transfer) -> void {
            auto self = static_cast<CDCDevice*>(transfer->user_data);

            if (!self->_handling_events) {
                libusb_cancel_transfer(transfer);
                return;
            }
            if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
                self->_disconnected = true;
                return;
            }

            if (self->cdc_recv_cb)
                self->cdc_recv_cb(transfer->buffer, transfer->actual_length);

            int rc = libusb_submit_transfer(transfer);
            if (rc != 0)
                self->_disconnected = true;
        },
        this, 0);

    // 如果平台支持热插拔
    if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        int rc = libusb_hotplug_register_callback(
            ctx,
            static_cast<libusb_hotplug_event>(
                LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT | LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED),
            LIBUSB_HOTPLUG_NO_FLAGS, vid, pid, LIBUSB_HOTPLUG_MATCH_ANY,
            [](libusb_context* ctx, libusb_device* device, libusb_hotplug_event event,
               void* user_data) -> int {
                static_cast<CDCDevice*>(user_data)->on_hotplug(event);
                return 0;
            },
            this, &hotplug_handle);

        if (rc != LIBUSB_SUCCESS) {}
    }

    // 提交异步接收请求
    if (libusb_submit_transfer(recv_transfer) != 0) {
        
    }

    _disconnected     = false;
    _handling_events  = true;
    _need_reconnected = false;
    return true;
}

void CDCDevice::close() {
    if (recv_transfer) {
        libusb_cancel_transfer(recv_transfer);
        libusb_free_transfer(recv_transfer);
        recv_transfer = nullptr;
    }
    if (handle) {
        libusb_release_interface(handle, interfaces_num);
        libusb_close(handle);
        handle = nullptr;
    }
}

int CDCDevice::send(const void* data, int size, unsigned int time_out) {
    int actual_size;
    int rc = libusb_bulk_transfer(handle, EP_OUT, (uint8_t*)data, size, &actual_size, time_out);
    if (rc != 0)
        return -1;
    else
        return actual_size;
}

void CDCDevice::process_once() {
    timeval tv = {.tv_sec = 0, .tv_usec = 0};
    libusb_handle_events_timeout_completed(ctx, &tv, nullptr);

    if (_disconnected) {
        close();
    }
    if (_need_reconnected) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        open(last_vid, last_pid, interfaces_num);
    }
}

void CDCDevice::regeiser_recv_cb(std::function<void(uint8_t* data, int size)> recv_cb) {
    cdc_recv_cb = std::move(recv_cb);
}

void CDCDevice::on_hotplug(libusb_hotplug_event event) {
    if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        _disconnected = true;
    } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        _need_reconnected = true;
    }
}
