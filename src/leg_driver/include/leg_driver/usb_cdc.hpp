#ifndef __USB_CDC_HPP__
#define __USB_CDC_HPP__


#include <atomic>
#include <libusb-1.0/libusb.h>
#include <memory>
#include <functional>
#include <thread>
#include <memory>


class CDCDevice{
public:

    static constexpr uint8_t EP_OUT        = 0x01;
    static constexpr uint8_t EP_IN         = 0x81;


    CDCDevice();
    ~CDCDevice();
    bool open(uint16_t vid,uint16_t pid,int interfaces_num);
    void close();
    int send(const void* data,int size,unsigned int time_out);    //异步发送数据包
    void regeiser_recv_cb(std::function<void(uint8_t* data,int size)> recv_cb);    //注册数据包接收回调
    void process_once();    //处理一次事件
private:
    void on_hotplug(libusb_hotplug_event event);  //热插拔中
    
    uint16_t last_vid,last_pid;
    int interfaces_num;
    std::function<void(uint8_t* data,int size)> cdc_recv_cb;
    std::atomic_bool _disconnected;
    std::atomic_bool _need_reconnected;
    std::atomic_bool _handling_events;
    std::byte cdc_rx_buffer[64];
    libusb_transfer* recv_transfer; 
    libusb_context *ctx;
    libusb_device_handle *handle;
    libusb_hotplug_callback_handle hotplug_handle;
};


#endif