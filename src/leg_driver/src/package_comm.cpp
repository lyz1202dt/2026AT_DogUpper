#include "package_comm.hpp"
#include "usb_cdc.hpp"
#include <cstring>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

PackageComm::PackageComm() {
    recv_buffer_index=0;
    last_recv_pack_id=0;
    cdc_thread_running       = true;
    cdc_device               = std::make_unique<CDCDevice>();
    cdc_device_handle_thread = std::make_unique<std::thread>([this]() {
        while (cdc_thread_running)
            cdc_device->process_once();
    });
    bool ret=cdc_device->open(0x0483, 0x5740, 1);
    cdc_device->regeiser_recv_cb([this](uint8_t *data,int size){on_recv_cdc(data, size);RCLCPP_INFO(rclcpp::get_logger("packageComm"),"接收到CDC数据包");});

    if(ret)
        RCLCPP_INFO(rclcpp::get_logger("packageComm"),"PackageComm类初始化完成，打开USB设备成功");
    else
        RCLCPP_INFO(rclcpp::get_logger("packageComm"),"PackageComm类初始化完成，打开USB设备失败");
}

PackageComm::~PackageComm() {
    cdc_thread_running = false;
    if (cdc_device_handle_thread->joinable())
        cdc_device_handle_thread->join();
    cdc_device->close();
}

bool PackageComm::async_send(const uint8_t* data, int size) {
    uint8_t temp[64];
    int pack_index = size / (64 - (int)sizeof(uint32_t)); // 计算需要发的包的数量
    if(size % (64 - sizeof(uint32_t))==0)	//如果可以整包发完，那么不再计算空包
		pack_index=pack_index-1;
    
    int j=0;
    for (int i = pack_index; i >= 0; i--)                 // 填写发送缓冲区
    {
        CDC_Trans_t* trans = reinterpret_cast<CDC_Trans_t*>(temp);
        std::memcpy(trans->data,data,size>60?60:size);
        trans->pack_id = i;
        if (i == 0) {
            cdc_device->send(temp, size, 10);
        }
        else {
            cdc_device->send(temp, 64, 10);
            size=size-60;
        }
        RCLCPP_INFO(rclcpp::get_logger("packageComm"),"分包发送%d/%d",j,pack_index);
        j++;
    }
    return true;
}

void PackageComm::on_recv_cdc(const uint8_t* data, int size){
    const CDC_Trans_t *trans = reinterpret_cast<const CDC_Trans_t*>(data);
    RCLCPP_INFO(rclcpp::get_logger("packageComm"),"包ID:%d,包长度%d",trans->pack_id,size);
        if (trans->pack_id) // 包ID不为0
        {
            if (last_recv_pack_id <= trans->pack_id) // 如果当前包的ID大于等于之前包的ID，说明此时是全新的传输任务，将本次接收的数据包复制到用户缓冲区新位置
            {
                RCLCPP_INFO(rclcpp::get_logger("packageComm"),"缓冲区索引清零 上一个ID %d，本次ID %d",last_recv_pack_id,trans->pack_id);
                if(size)    //有时候会有空包触发，加一个这个，如果有空包，那么不处理
                    recv_buffer_index = 0;
            }
            if (size&&(recv_buffer_index + size - sizeof(uint32_t) < 512))
            {
                std::memcpy(recv_buffer + recv_buffer_index, trans->data, size - sizeof(uint32_t)); // 将接收的数据包拷贝到用户缓冲区
                recv_buffer_index = recv_buffer_index + size - sizeof(uint32_t);
            }
        }
        else // 包ID为0，说明本次包传输任务已经完成
        {
            RCLCPP_INFO(rclcpp::get_logger("packageComm"),"关键字节序(1)%d (60)%d (100)%d ",recv_buffer[0],recv_buffer[59],recv_buffer[99]);
            std::memcpy(recv_buffer + recv_buffer_index, trans->data, size - sizeof(uint32_t));
            recv_buffer_index = recv_buffer_index + size - sizeof(uint32_t);
            pack_recv_cb(recv_buffer,(recv_buffer_index+1));
            recv_buffer_index = 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("packageComm"),"缓冲区索引%d",recv_buffer_index);
        last_recv_pack_id = trans->pack_id;
}

bool PackageComm::register_recv_cb(std::function<void(const uint8_t* data, int size)> recv_cb) {
    pack_recv_cb=std::move(recv_cb);
    return true;
}
