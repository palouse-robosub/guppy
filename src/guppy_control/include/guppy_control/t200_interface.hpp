#ifndef GUPPY_T200_INTERFACE_H
#define GUPPY_T200_INTERFACE_H

#include <Eigen/Core>
#include <string>
#include <vector>

#include <iostream>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace t200_interface {

/* A hardware interface for a controller to interact with the T200 thrusters
 * over CAN */
class T200Interface {
    /* an ordered vector of the CAN id for each motor */
    std::vector<unsigned int> can_ids;

    /* the CAN socket descriptor */
    int sock_;

    /* the can interface (like can0 or vcan0) */
    std::string can_interface_;

    /* are thrusters enabled */
    bool enabled_;

    /*
        @brief sets up the CAN interface and socket
        @return whether or not the operation was successful
    */
    bool setup_can();

    /*
        @brief sends a double value over CAN to a specific CAN id
        @param id the CAN id
        @param value the double value to send
        @return whether or not the operation was successful
    */
    bool send_to_can(unsigned int id, float value);

  public:
    /*
        @brief construct a new T200 thruster interface (one for multiple
       thrusters)
        @param can_interface the can interface to connect to, like can0 or vcan0
        @param can_ids an ordered vector of thruster CAN ids
    */
    T200Interface(
        std::string can_interface, std::vector<unsigned int> can_ids
    ) : can_ids(can_ids), can_interface_(can_interface) {
        setup_can();
    };

    /* closes socket */
    ~T200Interface() {
        if (sock_ >= 0)
            close(sock_);
    }

    /*
        @brief write a vector of throttles to CAN
        @param throttles an Eigen::VectorXd of throttle values in -1/1 format
        @return whether or not the operation was sucessful for all values
    */
    bool write(Eigen::VectorXd throttles);

    /*
        @brief start up the interface and initialize all thrusters with 0
       throttle 100x
        @return whether or not the operation was sucessful for all values
    */
    bool initialize();

    /*
        @brief shut down the interface and halt all thrusters with 0 throttle
       100x
        @return whether or not the operation was sucessful for all values
    */
    bool shutdown();

    /*
        @brief sets thrusters to be enabled/disabled
    */
    void set_enabled(bool enabled);
};

}    // namespace t200_interface

#endif
