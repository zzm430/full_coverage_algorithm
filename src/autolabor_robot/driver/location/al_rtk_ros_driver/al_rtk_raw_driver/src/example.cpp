//
// Created by zyh on 2022/3/24.
//

#include "example.h"
#include <iostream>

struct event_base *base;

void test_call(std::string gpgga_str) {

    std::cout << "recieve gpgga str " << gpgga_str << std::endl;
}


int main() {


    rtk = new al_rtk::RTKDriver();

    rtk->register_gpgga_callback(test_call);

    rtk->start_rtk("/dev/cu.wchusbserial1420", 115200,"rtk.ntrip.qxwz.com","", "","/home/autolabor");

    return 0;

}