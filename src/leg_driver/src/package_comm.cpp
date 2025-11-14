#include "package_comm.hpp"
#include "usb_cdc.hpp"
#include <cstring>
#include <memory>

PackageComm::PackageComm() {
    recv_buffer_index=0;
    last_recv_pack_id=0;
    cdc_thread_running       = true;
    cdc_device               = std::make_unique<CDCDevice>();
    cdc_device_handle_thread = std::make_unique<std::thread>([this]() {
        while (cdc_thread_running)
            cdc_device->process_once();
    });
    cdc_device->open(0x0483, 0x0001, 1);
    cdc_device->regeiser_recv_cb([this](uint8_t *data,int size){on_recv_cdc(data, size);});
    
}

PackageComm::~PackageComm() {
    cdc_thread_running = false;
    if (cdc_device_handle_thread->joinable())
        cdc_device_handle_thread->join();
    cdc_device->close();
}

bool PackageComm::async_send(uint8_t* data, int size) {
    uint8_t temp[64];
    int pack_index = size / (64 - (int)sizeof(uint32_t)); // 计算需要发的包的数量
    for (int i = pack_index; i >= 0; i--)                 // 填写发送缓冲区
    {
        CDC_Trans_t* trans = reinterpret_cast<CDC_Trans_t*>(temp);
        memcpy(trans->data,data,size);
        trans->pack_id = i;
        if (i == 0) {
            cdc_device->send(data, size % (64 - (int)sizeof(uint32_t)), 10);
        }
        else {
            cdc_device->send(data, 64, 10);
        }
    }
    return true;
}

void PackageComm::on_recv_cdc(uint8_t* data, int size){
    CDC_Trans_t *trans = reinterpret_cast<CDC_Trans_t*>(data);
        if (trans->pack_id) // 包ID不为0
        {
            if (last_recv_pack_id <= trans->pack_id) // 如果当前包的ID大于等于之前包的ID，说明此时是全新的传输任务（上一个包被异常中断），将本次接收的数据包复制到用户缓冲区新位置
            {
                recv_buffer_index = 0;
            }
            if (recv_buffer_index + size - sizeof(uint32_t) < 512)
            {
                memcpy(recv_buffer + recv_buffer_index, data + sizeof(uint32_t), size - sizeof(uint32_t)); // 将接收的数据包拷贝到用户缓冲区
                recv_buffer_index = recv_buffer_index + size - sizeof(uint32_t);
            }
        }
        else // 包ID为0，说明本次包传输任务正确完成
        {
            memcpy(recv_buffer + recv_buffer_index, data + sizeof(uint32_t), size - sizeof(uint32_t));
            recv_buffer_index = recv_buffer_index + size - sizeof(uint32_t);
            pack_recv_cb(recv_buffer,(int)recv_buffer_index+1);
            recv_buffer_index = 0;
        }
        last_recv_pack_id = trans->pack_id;
}

bool PackageComm::register_recv_cb(std::function<void(uint8_t* data, int size)> recv_cb) {
    pack_recv_cb=std::move(recv_cb);
    return true;
}

template <typename T>
bool PackageComm::async_send_struct(const T& pack) {
    static_assert(std::is_standard_layout<T>::value, "结构体不是标准构型");
    static_assert(std::is_trivial<T>::value, "数据包必须是可复制的");

    constexpr int pack_size = sizeof(T);

    async_send(static_cast<uint8_t*>(&pack), pack_size);
    return true;
}
