#ifndef __PACKAGECOMM_HPP__
#define __PACKAGECOMM_HPP__

#include "usb_cdc.hpp"
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>


#pragma pack(1)

struct CDC_Trans_t{
    uint32_t pack_id;
    uint8_t data[];
};

#pragma pack()


class PackageComm{
public:
    PackageComm();
    ~PackageComm();
    bool async_send(uint8_t *data,int size);    //异步发送
    bool register_recv_cb(std::function<void(uint8_t* data,int size)> recv_cb);    //注册异步接收函数

    template<typename T>
    bool async_send_struct(const T& pack);      //自动识别结构体并发送其常引用

private:
    void on_recv_cdc(uint8_t* data,int size);
    
    bool cdc_thread_running;
    std::unique_ptr<std::thread> cdc_device_handle_thread;
    std::unique_ptr<CDCDevice> cdc_device;
    std::function<void(uint8_t *data,int size)> pack_recv_cb;
    uint8_t recv_buffer[512];
    uint32_t recv_buffer_index=0;
    uint32_t last_recv_pack_id;
};

#endif
