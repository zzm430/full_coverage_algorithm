
#include <sstream>
#include <vector>
#include "../include/al_rtk_raw_driver.h"

namespace al_rtk {


    RTKDriver::RTKDriver()
    {

        std::cout<<" RTK Driver init" <<std::endl;

        this->base = event_base_new(); // libevent事件管理器初始化

//        struct event *sig_event_sigterm= evsignal_new(this->base,SIGTERM,sighandler,this);
//
//        event_add(sig_event_sigterm,NULL);


    }

    RTKDriver::~RTKDriver()
    {

        exit_rtk();

    }


    void RTKDriver:: sighandler(int dont_use, short event, void *arg)
    {

        RTKDriver * self=(RTKDriver*) arg;

        self->exit_rtk();

    }

/// 收到有效GPGGA数据事件
/// \param gpgga_str 收到的GPGGA字符串
    void RTKDriver::on_recieved_gpgga(std::string gpgga_str) {

        last_gpgga_str = gpgga_str;

        if (gpgga_callback_fun != nullptr) {

            (*gpgga_callback_fun)(last_gpgga_str);

        }

    }
/// 判断gppga是否为空，需事先用is_gppga判断是GPPGA包否则有out-of-range异常
/// \param gpgga_str
/// \return
    bool RTKDriver::is_gpgga_empty(std::string gpgga_str) {

        std::stringstream ss(gpgga_str);

        std::vector<std::string> items;

        while (ss.good()) {

            std::string substr;

            getline(ss, substr, ',');

            items.push_back(substr);

        }


        return items.at(2).length() == 0;

    }
/// 奇偶校验
/// \param line
/// \return
    bool RTKDriver::bcd_validate(std::string line)
    {
        if (line.length()<4)
        {
            return false;
        }

        if (line.rfind("$",0)!=0)
        {
            return false;
        }

        if(0 != line.compare(line.length() - 2, 2, "\r\n"))
        {
            return false;
        }

        int indexOfStar=line.find_last_of('*');

        unsigned int bcdCal = 0x00;
        for (int i = 1; i < indexOfStar; i++) {
            bcdCal ^= line.c_str()[i];
        }
        int bcd = (int) std::strtol(line.substr(line.length() - 4, 2).c_str(), NULL, 16);

        return bcd == bcdCal;

    }
    /// 判断字符串是否是gpgga包
    /// \param data
    /// \return
    bool RTKDriver::is_gpgga(std::string data) {

       return data.rfind("$GPGGA,", 0) == 0;

    }
    /// 定时事件，用于监控串口有效数据，若太久未收到串口数据则触发重连
    /// \param dont_use 不要使用这个参数
    /// \param event  事件类型，未用到
    /// \param arg 当前对象指针
    void RTKDriver::on_serial_live_monitor(int dont_use,short event, void * arg)
    {

        RTKDriver * self=(RTKDriver*) arg;

        if (self->serial_live==false)
        {

            LOG(ERROR)<<"monitor of gpgga-live-check was time-out. reconnect rtk device." <<std::endl;

            if (self->ev_serial_live_monitor!=NULL) {
                event_del(self->ev_serial_live_monitor);
            }
            event_base_once(self->base, -1, EV_TIMEOUT, on_serial_disconnect, self,NULL);
        }

        self->serial_live=false;

    }
    /// 串口收到数据事件，也接收各种串口异常
    /// \param dont_use
    /// \param event
    /// \param arg
    void RTKDriver::on_serial_received(int dont_use, short event, void *arg) {

        RTKDriver * self=(RTKDriver*) arg;

        char buffer[TMP_BUFF_SIZE];

        int bytes_rev = read(self->serial_fd, &buffer, TMP_BUFF_SIZE);

        if (bytes_rev <= 0) {

            LOG(ERROR) << "Error on read serial port"<< self->serial_device <<" error num " << std::to_string(errno) << " msg:" << strerror(errno)
                       << std::endl;

            event_base_once(self->base, -1, EV_TIMEOUT, on_serial_disconnect, self,NULL);

        }


        if (bytes_rev > 0) {

            self->serial_recv_lines.clear();

            self->parser_line(self->serial_buffer, &self->serial_buffer_length,
                              buffer, bytes_rev, &self->serial_recv_lines);

            for(auto str:self->serial_recv_lines)
            {

                if (is_gpgga(str))
                {
                    self->serial_live=true;

                    if (is_gpgga_empty(str)==false)
                    {
                        self->last_gpgga_str=str;

                        self->on_recieved_gpgga(str);

                    }



                }

            }

        }


    }

    bool RTKDriver::write_socket(std::string content)
    {
        if (write(this->sock_fd, const_cast<char *>(content.c_str()), content.length()) < 0) {
            std::string error_msg ="Error on write socket  error num " + std::to_string(errno) + " msg:" + strerror(errno) +
                                   "\r\n";

            LOG(ERROR)<<error_msg;

            event_base_once(this->base, -1, EV_TIMEOUT, on_socket_disconnect, this,NULL);

            return false;
        }

        return true;
    }

    void RTKDriver::write_serial(std::string content) {

        sleep(1);

        if (write(this->serial_fd, const_cast<char *>(content.c_str()), content.length()) < 0) {
            std::string error_msg ="Error on write serial port " +this->serial_device +"  error num " + std::to_string(errno) + " msg:" + strerror(errno) +
                    "\r\n";

            LOG(ERROR)<<error_msg;



            event_base_once(this->base, -1, EV_TIMEOUT, on_serial_disconnect, this,NULL);
        }

    }

/// 初始化RTK接收机
/// \param dont_use
/// \param event
/// \param arg

    void RTKDriver::init_rtk_device(int dont_use, short event, void *arg) {

        RTKDriver * self=(RTKDriver*) arg;

        LOG(INFO)<< "rtk device initializing." <<std::endl;

        self->write_serial("mode rover\r\n"); //设置模式为rover

        self->write_serial("gpgga com1 1\r\n"); //打开 com1 gpgga输出

        self->write_serial("gpgga com2 1\r\n");

        self->write_serial("gpgga com3 1\r\n");


        if (self->ev_serial_receive == NULL) {

            self->ev_serial_receive = event_new(self->base, self->serial_fd, EV_READ | EV_PERSIST, on_serial_received,
                                                self);
        }
        event_add(self->ev_serial_receive, NULL);


        if (self->ev_serial_live_monitor==NULL) {

            self->ev_serial_live_monitor = event_new(self->base, -1, EV_TIMEOUT | EV_PERSIST, on_serial_live_monitor,
                                                     self);
        }

        struct timeval serial_monitor_intervel = {SOCKET_MONITOR_INTERVEL, 10};

        event_add(self->ev_serial_live_monitor, &serial_monitor_intervel);

        self->serial_ready=true;

        LOG(INFO)<< "rtk device initialized." <<std::endl;
    }

/// ntrip服务验证账号通过事件
/// \param fd
/// \param event
/// \param args
    void RTKDriver::on_socket_validate_passed(int fd,short event, void * args) {

        RTKDriver *self = (RTKDriver *) args;

        LOG(INFO) << "ntrip service validate passed." << std::endl;

        self->socket_ready=true;


        if (self->serial_ready && self->last_gpgga_str!="") {

            bool success=self->write_socket(self->last_gpgga_str);

            LOG(INFO)<<"send first gpgga to ntrip service :"<<self->last_gpgga_str<< "  "<<std::to_string(success) <<std::endl;

        }

        if (self->ev_socket_receive == NULL) {

            self->ev_socket_receive = event_new(self->base, self->sock_fd, EV_READ | EV_PERSIST, on_socket_recieved,
                                                self);
        }

        event_add(self->ev_socket_receive, NULL);

        if (self->ev_socket_live_monitor==NULL) {

            self->ev_socket_live_monitor = event_new(self->base, -1, EV_TIMEOUT | EV_PERSIST, on_socket_live_monitor,
                                                     self);
        }
        struct timeval socket_monitor_intervel = {SOCKET_MONITOR_INTERVEL, 10};

        event_add(self->ev_socket_live_monitor, &socket_monitor_intervel);


    }
/// ntrip账号验证失败事件
/// \param fd
/// \param event
/// \param args
    void RTKDriver::on_socket_validate_failed(int fd,short event, void * args)
    {

        RTKDriver * self=(RTKDriver*) args;
        LOG(ERROR)<< "ntrip service validate failed, please check account and password is correct." <<std::endl;
        struct timeval time_out={SOCKET_RETRY_SECONDS,0};
        event_base_once(self->base, -1, EV_TIMEOUT, on_socket_disconnect, self,&time_out); //触发断开连接事件

    }
/// 开始验证nrtip账号事件
/// \param dont_use
/// \param event
/// \param args
    void RTKDriver::on_socket_validate(int dont_use,short event, void * args)
    {
        RTKDriver * self=(RTKDriver*) args;

        LOG(INFO)<< "ntrip service validate msg received." <<std::endl;

        if (self->ev_socket_validate_timeout!=NULL)
        {
            event_del(self->ev_socket_validate_timeout); //注销掉超时事件
        }

        int num_bytes = read(self->sock_fd, self->sock_buffer, SOCKET_BUFF_SIZE);

        if (num_bytes <=0) {

            LOG(ERROR) << "Error on read from socket error : " << strerror(errno)
                       << std::endl;

            struct timeval time_out={SOCKET_RETRY_SECONDS,0};

            event_base_once(self->base, -1, EV_TIMEOUT, on_socket_disconnect, self,&time_out); //触发断开连接事件

            return;
        }

        std::string response_str(self->sock_buffer);

        if (response_str.find("ICY 200 OK") == response_str.npos) {

            event_base_once(self->base, -1, EV_TIMEOUT, on_socket_validate_failed, self,NULL); //触发验证失败事件

        } else {

            event_base_once(self->base, -1, EV_TIMEOUT, on_socket_validate_passed, self,NULL); //触发验证通过事件

        }


    }
/// ntrip账号验证超时事件，即nrtip服务未返回数据
/// \param fd
/// \param event
/// \param args
    void RTKDriver::on_socket_validate_timeout(int fd,short event, void * args)
    {
        LOG(ERROR)<< "ntrip service validate timeout." <<std::endl;

        RTKDriver * self=(RTKDriver*) args;

        if (self->ev_socket_validate!=NULL)
        {
            event_del(self->ev_socket_validate);
        }


        event_base_once(self->base, -1, EV_TIMEOUT, on_socket_disconnect, self,NULL); //触发断开连接事件

    }


    void RTKDriver::on_socket_disconnect(int dont_use,short event, void * args)
    {
        LOG(ERROR)<< "ntrip service socket disconnect." <<std::endl;

        RTKDriver * self=(RTKDriver*) args;

        self->socket_ready=false;

        close(self->sock_fd);
        if (self->ev_socket_live_monitor!=NULL)
        {
            event_del(self->ev_socket_live_monitor);
        }
        if (self->ev_socket_receive != NULL) {
            event_del(self->ev_socket_receive);
        }
        struct timeval delay={SOCKET_RETRY_SECONDS,0};

        event_base_once(self->base, -1, EV_TIMEOUT, connect_socket, self,&delay); //延迟触发重连事件


    }
     void RTKDriver::on_socket_connect(int dont_use,short event, void * args)
     {
         RTKDriver * self=(RTKDriver*) args;

         LOG(INFO)<< "ntrip service socket connected." <<std::endl;

         std::string encode_str = base64_encode(
                 const_cast<char *>(std::string(self->ntrip_account + ":" + self->ntrip_password).c_str()));

         std::string authorStr("GET /AUTO HTTP/1.0\r\n"
                               "User-Agent: Autolabor RTK 1.0 \r\n"
                               "Accept: */*\r\n"
                               "Connection: close\r\n"
                               "Authorization: Basic " + encode_str + "\r\n\r\n");



       if (self->write_socket(authorStr))
       {


             if (self->ev_socket_validate==NULL)
             {

                 self->ev_socket_validate=event_new(self->base, self->sock_fd, EV_READ, on_socket_validate, self); //注册验证事件

             }

             event_add(self->ev_socket_validate,NULL);

             struct timeval time_out={15,0};

             if (self->ev_socket_validate_timeout==NULL) {

                 self->ev_socket_validate_timeout = event_new(self->base, -1, EV_TIMEOUT, on_socket_validate_timeout,
                                                              self);//注册验证超时事件
             }
             event_add(self->ev_socket_validate_timeout,&time_out);


         }



     }
     /// socket有效数据定时监控事件，超过一定事件未收到ntrip服务即重连
     /// \param dont_use
     /// \param event
     /// \param args
     void RTKDriver::on_socket_live_monitor(int dont_use,short event, void * args)
    {
        RTKDriver * self=(RTKDriver*) args;


        if (self->socket_live==false)
        {

            LOG(ERROR)<<"monitor of ntrip service  was time-out. reconnect ntrip service." <<std::endl;

            event_base_once(self->base, -1, EV_TIMEOUT, on_socket_disconnect, self,NULL); //触发断开连接事件
        }

        self->socket_live=false;


    }
    /// socket接收到数据事件
    /// \param dont_use 不要使用
    /// \param event 不重要
    /// \param arg 当前对象指针
    void RTKDriver::on_socket_recieved(int dont_use, short event, void *arg) {

        RTKDriver * self=(RTKDriver*) arg;

        int num_bytes = read(self->sock_fd, self->sock_buffer, SOCKET_BUFF_SIZE);

        if (num_bytes <=0) {

            LOG(ERROR) << "Error on read socket error  :" << strerror(errno)
                       << std::endl;

            if (self->ev_socket_receive != NULL)
            {
                event_del(self->ev_socket_receive);
            }


            event_base_once(self->base, -1, EV_TIMEOUT, on_socket_disconnect, self,NULL); //触发断开连接事件


        }
        if (num_bytes > 0) {

            self->socket_live=true; //重置收到有效数据标志

            if (self->serial_ready) { //如果串口已经准备好，则向其写入收到的差分数据

                write(self->serial_fd,self->sock_buffer,num_bytes);

            }

        }


    }

/// DNS Client， 请求域名返回IP
/// \param domain
/// \param addr
/// \return
    int RTKDriver::dns_query(std::string domain, struct sockaddr_in * addr)
    {
        struct addrinfo hints;
        struct addrinfo *res, * cur;
        int ret;

        memset(&hints,0,sizeof (struct addrinfo));

        hints.ai_family=AF_INET;
        hints.ai_flags=AI_PASSIVE;
        hints.ai_protocol=0;
        hints.ai_socktype=SOCK_DGRAM;

        ret= getaddrinfo(domain.c_str(),NULL,&hints,&res);
        if (ret!=0)
        {
            LOG(ERROR)<<"ntrips domain query error :" << gai_strerror(ret);
            return ret;

        }
            memcpy(addr,res->ai_addr,sizeof(struct sockaddr_in));
            freeaddrinfo(res);

        return ret;

    }

    void RTKDriver::connect_socket(int dont_use,short event, void * args)
    {

        RTKDriver * self=(RTKDriver*) args;

        self->sock_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

        struct sockaddr_in serv_addr;

        memset(&serv_addr, 0, sizeof(serv_addr));

        if (self->dns_query(self->ntrip_domain,&serv_addr)!=0)
        {
            event_base_once(self->base, -1, EV_TIMEOUT, on_socket_disconnect, self,NULL);
            return;

        }

        serv_addr.sin_family = AF_INET;  //使用IPv4地址

        serv_addr.sin_port = htons(8002); // WGS84 特定端口

        if (connect(self->sock_fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) == 0) {

            event_base_once(self->base, -1, EV_TIMEOUT, on_socket_connect, self,NULL); //触发连接成功事件


        }else
        {

            event_base_once(self->base, -1, EV_TIMEOUT, on_socket_disconnect, self,NULL); //触发断开连接事件

        }




    }




    void RTKDriver::on_serial_disconnect(int dont_use,short event, void * args)
    {
        RTKDriver * self=(RTKDriver*) args;

        LOG(ERROR)<< "serial disconnect" <<std::endl;

        if (self->serial_fd>0) {

            close(self->serial_fd);

        }

        if (self->ev_serial_receive != NULL)
        {
            event_del(self->ev_serial_receive);
        }

        self->last_gpgga_str="";

        self->serial_ready=false;

        struct timeval delay={SOCKET_RETRY_SECONDS,0};

        LOG(INFO)<< "reconnect serial port after "<<std::to_string(SOCKET_RETRY_SECONDS) <<" seconds." <<std::endl;

        event_base_once(self->base, -1, EV_TIMEOUT, connect_serial, self,&delay); //延迟触发重连事件


    }
    /// 开始连接串口（RTK接收机）
    /// \param dont_use
    /// \param event
    /// \param args
    void RTKDriver::connect_serial(int dont_use, short event, void *args){

        RTKDriver * self=(RTKDriver*) args;

        LOG(INFO)<< "connect to serial port of rtk device." <<std::endl;

        self->serial_fd = open(const_cast<char *>(self->serial_device.c_str()), O_RDWR);

        if (self->serial_fd<0)
        {

            LOG(ERROR) << "Error on open serial port "<< self->serial_device.c_str() <<" : " << strerror(errno)
                       << std::endl;

            event_base_once(self->base, -1, EV_TIMEOUT, on_serial_disconnect, self,NULL);

            return ;

        }

        LOG(INFO)<< "serial port of rtk device connected." <<std::endl;


        self->serial_buffer_length = 0;

        memset(self->serial_buffer, '\0', sizeof(serial_buffer));

        struct termios tty;

        if (tcgetattr(self->serial_fd, &tty) != 0) {
            LOG(ERROR) << "Error on open serial port  error num " << std::to_string(errno) << " msg:" << strerror(errno)
                       << std::endl;

            event_base_once(self->base, -1, EV_TIMEOUT, on_serial_disconnect, self,NULL);

            return ;

        }

        tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
        tty.c_cflag |= CS8; // 8 bits per byte (most common)
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON; // Disable ICANON
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                         ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
        // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

        tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 1;

        // Set in/out baud rate to be 9600
        cfsetispeed(&tty, self->get_baud(self->serial_baudrate));

        cfsetospeed(&tty, self->get_baud(self->serial_baudrate));

        tcflush(self->serial_fd,TCIOFLUSH);

        // Save tty settings, also checking for error
        if (tcsetattr(self->serial_fd, TCSANOW, &tty) != 0) {

            LOG(ERROR) << "Error set serial port attribute error num " << std::to_string(errno) << " msg:"
                       << strerror(errno) << std::endl;


            event_base_once(self->base, -1, EV_TIMEOUT, on_serial_disconnect, self,NULL);

            return ;

        }

        sleep(2);

        init_rtk_device(-1,-1,self);




    }

    void RTKDriver::init_log(std::string log_dir,int stderrthreshold) {

        FLAGS_log_dir = log_dir; //当前位置为日志输出目录

        FLAGS_logbufsecs = 1; //刷新logbuf时间 秒

        FLAGS_colorlogtostderr=true;

        FLAGS_colorlogtostdout=true;

        FLAGS_stderrthreshold=stderrthreshold; //输出日志级别 INFO

        google::InitGoogleLogging("/al_rtk_driver"); //日志输出文件名

        LOG(INFO) << "start rtk driver" << std::endl;

    }

    int RTKDriver::get_baud(int baud) {

        switch (baud) {
            case 9600:
                return B9600;
            case 19200:
                return B19200;
            case 38400:
                return B38400;
            case 57600:
                return B57600;
            case 115200:
                return B115200;
            case 230400:
                return B230400;
            default:
                return -1;
        }
    }


    void RTKDriver::register_gpgga_callback(callback_fun callback_fun) {

        gpgga_callback_fun = callback_fun;

    }


/// 开启RTK接收机
/// \param device 串口设备地址
/// \param baudrate 波特率
/// \param domain 差分服务域名 （千寻 rtk.ntrip.qxwz.com）
/// \param account 差分服务账号
/// \param passsword 差分服务密码
/// \return
    int RTKDriver::start_rtk(std::string device, int baudrate, std::string domain,std::string account, std::string passsword,std::string log_dir,bool logtostdout) {

        init_log(log_dir,logtostdout);

        this->serial_device = device;

        this->serial_baudrate = baudrate;

        this->ntrip_domain =domain;

        this->ntrip_account = account;

        this->ntrip_password = passsword;

        connect_socket(-1,-1,this); //开始连接ntrip服务

        connect_serial(-1,-1,this); //开始连接rtk接收机串口

        event_base_loop(this->base,EVLOOP_NO_EXIT_ON_EMPTY);

        return 0;
    }


    void RTKDriver::exit_rtk() {

        if (this->serial_fd>0) {

            close(this->serial_fd);
        }


        if (this->sock_fd>0)
        {
            close(sock_fd);
        }

        event_base_loopexit(this->base,NULL);

    }
    /// 将串口收到数据按行解析
    /// \param global_buffer
    /// \param global_buffer_size
    /// \param append_buffer
    /// \param append_buffer_size
    /// \param parsed_lines
    /// \return

    bool RTKDriver::parser_line(char *global_buffer, int *global_buffer_size, char *append_buffer, int append_buffer_size,
                     std::vector<std::string> *parsed_lines) {

        bool found_line = false;

        if ((*global_buffer_size + append_buffer_size) < TMP_BUFF_SIZE) {
            memcpy(global_buffer + *global_buffer_size, append_buffer, append_buffer_size);
            *global_buffer_size += append_buffer_size;
        }


        int i = 0;

        do {

            if (i + 1 >= *global_buffer_size) {

                break;
            }
            if (global_buffer[i] == '\r' && global_buffer[i + 1] == '\n') {
                int len = i + 2;
                std::string parsed_line=std::string(global_buffer, len);

                if (bcd_validate(parsed_line)) //BCD校验通过后，放入结果容器
                {
                    parsed_lines->push_back(parsed_line);

                }

                int left = *global_buffer_size - len;

                char temp[left];

                memcpy(temp, global_buffer + len, left);

                memset(global_buffer, '\0', TMP_BUFF_SIZE);

                memcpy(global_buffer, temp, left);

                *global_buffer_size = left;

                i = 0;

                found_line = true;

            } else {

                i += 1;
            }


        } while (true);

        return found_line;
    }

}