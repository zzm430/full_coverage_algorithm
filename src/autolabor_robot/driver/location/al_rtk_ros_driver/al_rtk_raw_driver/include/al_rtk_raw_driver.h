//
// Created by Apache on 2022/3/20.
//

#ifndef AL_RTK_RAW_DRIVER_AL_RTK_RAW_DRIVER_H
#define AL_RTK_RAW_DRIVER_AL_RTK_RAW_DRIVER_H

#include <event.h>
#include <iostream>
#include <string>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "base64.h"
#include "glog/logging.h"
#include <netdb.h>

#define TMP_BUFF_SIZE 2048

namespace al_rtk {


    static const int SERIAL_BUFF_SIZE = 20480;
    static const int SOCKET_BUFF_SIZE=20480;
    static const int SOCKET_RETRY_SECONDS=5; // 串口重连间隔事件 秒
    static const int SOCKET_MONITOR_INTERVEL=15; //NTRIP 服务有效数据超时时间 秒

    class RTKDriver {

        typedef void(*callback_fun)(std::string);

    public:

        RTKDriver();

        ~RTKDriver();


        int start_rtk(std::string device, int baudrate, std::string domain, std::string account, std::string passsword,
                      std::string log_dir, bool logtostdout); //启动RTK接收机

        void exit_rtk(); //关闭RTK接收机

        void register_gpgga_callback(callback_fun callback_fun); //注册GPGGA信息回调函数

        static bool is_gpgga(std::string data); //判断内容是否是GPGGA包

        static bool is_gpgga_empty(std::string gpgga_str); //判断空GPPGA包

    private:

        std::vector<std::string> serial_recv_lines; //串口接收内容

        std::string ntrip_domain; //ntrip 服务域名

        std::string ntrip_account; //ntrip 账号

        std::string ntrip_password; //ntrip 密码

        struct event_base *base; //事件管理器

        struct event *ev_socket_validate;

        struct event *ev_socket_validate_timeout; //socket 验证账号密码超时

        struct event *ev_socket_live_monitor;

        static void connect_socket(int dont_use,short event, void * args); //连接 ntrip socket

        static void on_socket_connect(int dont_use,short event, void * args);//socket 连接成功事件

        static void on_socket_validate(int dont_use,short event, void * args); //socket 验证事件

        static void on_socket_validate_timeout(int dont_use,short event, void * args); //socket 验证超时

        static void on_socket_validate_passed(int dont_use,short event, void * args); //验证通过事件

        static void on_socket_validate_failed(int dont_use,short event, void * args); //验证失败事件

        static void on_socket_disconnect(int dont_use,short event, void * args);  //socket断开事件

        static void on_socket_recieved(int fd, short event, void *arg); //接收数据事件

        static void on_socket_live_monitor(int dont_use,short event, void * args);// socket 监控定时事件

        struct event *ev_serial_live_monitor;

        struct event *ev_serial_receive;

        struct event *ev_socket_receive;

        static void on_serial_disconnect(int dont_use,short event, void * args); //串口断开事件

        static void connect_serial(int dont_use, short event, void *arg); //连接串口

        static void init_rtk_device(int dont_use, short event, void *arg); //初始化RTK接收机

        static void on_serial_received(int dont_use, short event, void *arg); //串口收到数据事件

        static void on_serial_live_monitor(int dont_use,short event, void * args);// 串口定时事件

        static void sighandler(int dont_use, short event, void *arg);

        struct event ev_sigint;

        bool socket_live; //socket 收到有效数据标志

        bool serial_live; //串口收到有效数据标志

        bool socket_ready; //socket 可用标志

        bool serial_ready;//串口 可用标志

        bool parser_line(char *global_buffer, int *global_buffer_size, char *append_buffer, int append_buffer_size,
                         std::vector<std::string> *parsed_lines); //按照换行符回车符解析数据


        callback_fun gpgga_callback_fun = nullptr; //回调函数

        int serial_fd;

        int sock_fd;

        std::string serial_device; //串口设备

        int serial_baudrate; //波特率

        std::string last_gpgga_str; //最新有效GPPGA数据

        char serial_buffer[SERIAL_BUFF_SIZE];

        char sock_buffer[SOCKET_BUFF_SIZE];

        int serial_buffer_length; //当前串口缓冲区有效数据长度

        bool bcd_validate(std::string line);//奇偶校验

        void on_recieved_gpgga(std::string gpgga_str); //收到GPGGA数据事件

        void init_log(std::string log_dir,int stderrthreshold);//初始化日志

        int get_baud(int baud); //转化波特率

        int dns_query(std::string domain, struct sockaddr_in *addr); //域名解析请求

        bool write_socket(std::string content); //写socket

        void write_serial(std::string content); //写串口

    };


}


#endif //AL_RTK_RAW_DRIVER_AL_RTK_RAW_DRIVER_H
