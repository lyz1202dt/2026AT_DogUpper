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
    bool async_send(const uint8_t *data,int size);    //异步发送
    bool register_recv_cb(std::function<void(const uint8_t* data,int size)> recv_cb);    //注册异步接收函数

    template <typename T>
    bool async_send_struct(const T& pack) {
    static_assert(std::is_standard_layout<T>::value, "结构体不是标准构型");
    static_assert(std::is_trivial<T>::value, "数据包必须是可复制的");

    constexpr int pack_size = sizeof(T);
    
    async_send(reinterpret_cast<const uint8_t*>(&pack), pack_size);
    return true;
    }
    
private:
    void on_recv_cdc(const uint8_t* data,int size);
    
    bool cdc_thread_running;
    std::unique_ptr<std::thread> cdc_device_handle_thread;
    std::unique_ptr<CDCDevice> cdc_device;
    std::function<void(const uint8_t *data,int size)> pack_recv_cb;
    uint8_t recv_buffer[512];
    uint32_t recv_buffer_index=0;
    uint32_t last_recv_pack_id;
};

#endif
