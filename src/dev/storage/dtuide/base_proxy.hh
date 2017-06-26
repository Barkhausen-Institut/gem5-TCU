#ifndef __DEV_STORAGE_DTU_IDE_BASE_PROXY_HH__
#define __DEV_STORAGE_DTU_IDE_BASE_PROXY_HH__

#include <cstddef>
#include <list>

#include "dev/storage/dtuide/pci_connector.hh"
#include "mem/dtu/connector/base.hh"
#include "mem/dtu/regfile.hh"
#include "mem/dtu/dtu.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/BaseProxy.hh"
#include "sim/system.hh"

/**
 * Proxy class to process and forward requests to the PCI host, the ide
 * controller and to trigger events in the DTU, such as sending packets to
 * other endpoints. */
class BaseProxy : public MemObject
{
public:
    class InterruptSM;

private:

    /* Constant to specify the id of the endpoint to send packets, so on the
     * PE associated with this proxy. */
    static const unsigned int EP_SEND;
    static const unsigned int EP_REPLY;

    static const unsigned int PREP_PACKET_ADDR;
    static const unsigned int RESPOND_ADDR;
    static const unsigned int ACK_ADDR;

    /* ID to specify this proxy as the sender in packets */
    MasterID masterId;
    unsigned int id;

    /* Variable to specify if the device should process requests in atomic
     * mode. */
    const bool atomic;
    Addr reg_base;
    /// Stores the Packet for later retry
    PacketPtr retryPkt;
    PacketPtr tempPacket;
    bool intAnswered;

  protected:

    /**
     * This is the function responsible for walking through the steps of the
     * interrupt state machine and if implemented, also for the dma state
     * machine. It has to be scheduled through a event handler to be called at
     * some tick later in time, most probably the immediate next tick.
     */
    void tick();

    /**
     * Function responsible for sending range changes to master components.
     */
    void init();

    void sendMemResponse();

    /**
     * Function called after a tick has completed.
     */
    void completeRequest(PacketPtr pkt);

    void handleIntResp(PacketPtr pkt);

    /* Class for the port accessed by the DTU and triggering actions in the
       proxy */
    class DtuSidePort : public SlavePort
    {
        private:
            BaseProxy& baseproxy;
            AddrRangeList addrRangeList;
        public:
            DtuSidePort(const std::string& _name, BaseProxy* _baseproxy,
                    std::vector<AddrRange> _ranges)
                : SlavePort(_name, _baseproxy), baseproxy(*_baseproxy),
                  addrRangeList(_ranges.begin(), _ranges.end())
                {}
        protected:
            /**
             * This function is called if this port receives a request from
             * the corresponding master port.
             */
            bool recvTimingReq(PacketPtr pkt) override;

            /**
             * Function to retry response to packets if sending them failed.
             */
            void recvRespRetry() override;

            /**
             * Is called if the master port tries to send a packet atomically.
             * Not supported.
             */
            Tick recvAtomic(PacketPtr ptr) override;

            /**
             * Receive packet in a functional manner.
             */
            void recvFunctional(PacketPtr ptr) override;

            AddrRangeList getAddrRanges() const override;
    };

    /* Device side port to trigger actions at the pci host and the ide
     * controller. */
    class DeviceSidePort : public MasterPort
    {
      private:
        BaseProxy& baseproxy;
      public:
        DeviceSidePort(const std::string& _name, BaseProxy* _baseproxy)
            : MasterPort(_name, _baseproxy), baseproxy(*_baseproxy)
        { }
      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;
    };

    /* Class to send messages to the DTU, effectively triggering interrupts at
     * the target ep. */
    class InterruptPort : public MasterPort
    {
      private:
        BaseProxy& baseproxy;
      public:
        InterruptPort(const std::string& name, BaseProxy* proxy)
           : MasterPort(name, proxy), baseproxy(*proxy)
        { }

      protected:
        bool recvTimingResp(PacketPtr pkt) override;

        void recvReqRetry() override;
    };

    /**
     * Create a packet.
     */
    PacketPtr createPacket(Addr paddr, size_t size, MemCmd cmd);

    /**
     * Create a packet.
     */
    PacketPtr createPacket(Addr paddr, const void *data, size_t size,
        MemCmd cmd);

    /**
     * Create a packet for dtu registers.
     */
    PacketPtr createDtuRegPkt(Addr reg, RegFile::reg_t value, MemCmd cmd);

    /**
     * Create a packet for dtu command registers to prepare execution of
     * commands by the dtu.
     */
    PacketPtr createDtuCmdPkt(Dtu::Command::Opcode cmd, unsigned epid,
        uint64_t data, uint64_t size, uint64_t off);

    /**
     * Forward packet via the dtu port as a response
     */
    bool forwardFromDevice(PacketPtr pkt);

    /**
     * Forward a packet as a request via the dev port as a responsem to either
     * the pci host or the dma device.
     */
    bool forwardToDevice(PacketPtr pkt);

    /**
     * Delete the specified packet from memory.
     */
    void freePacket(PacketPtr pkt);

    /**
     * Send the packet via the dtu mastering port to read or write from dtu
     * registers
     */
    bool sendToDtu(PacketPtr pkt);

    /**
     * Looping through the list of unsent packets to resend them to the
     * device.
     */
    void recvRetry();

    /**
     * Looping through the list of unsent packets targeted as a response to
     * resent them via the
     * dtu port.
     */
    void respRetry();

    /**
     * Calculate register address.
     */
    Addr getRegAddr(CmdReg reg);

    /**
     * Calculate register address.
     */
    Addr getRegAddr(DtuReg reg);

    /**
     * Get the register address of the respective register on the specified
     * EP.
     */
    Addr getRegAddr(unsigned reg, unsigned epid);

    /**
     * Port to send packages directed at the device or the pci host.
     */
    DeviceSidePort dev_port;

    /**
     * Port to receive request packages
     */
    DtuSidePort dtu_port;

    /**
     * Port to master the dtu to write messages to the dtu and therefore
     * controlling it.
     */
    InterruptPort int_port;

    std::list<PacketPtr> *respPacketList;

    std::list<PacketPtr> *recvPacketList;

    System *system;

    EventWrapper<BaseProxy, &BaseProxy::tick> tickEvent;

    EventWrapper<BaseProxy, &BaseProxy::sendMemResponse> memRespEvent;

    InterruptSM * interruptSM;

    PCIConnector * con;

  public:

    BaseProxy(const BaseProxyParams *params);

    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;
    BaseSlavePort& getSlavePort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    /**
     * Post an interrupt with the specified line being specified.
     */
    void postPciInt(int line);

    /**
     * Clear the interrupt
     */
    void clearPciInt(int line);

    /**
     * Post an console interrupt. Currently not implemented.
     */
    void postConsoleInt();

    /**
     * Clear the console interrupt. Currently not implemented.
     */
    void clearConsoleInt();

    /**
     * Set the pci connector to transmit interrupts.
     */
    void setConnector(PCIConnector * con);

    /**
     * Routine being called by the dtu if interrupt is to be send to the
     * device or the VPE has res-
     * ponded to the former interrupt that was directed at the VPE.
     */
    void wakeup();

    /**
     * Reset the state of the base proxy.
     */
    void reset();

    /**
     * Propagates a connector interrupt from the dtu via the connector to the
     * device.
     */
    void interruptFromConnector();

    /* State machine to control the steps needed to transmit an interrupt to
     * the target ep. */
    class InterruptSM
    {
      public:

        enum State
        {
            IDLE,
            SEND,
            SEND_COMMAND,
            WAIT,
            FETCH,
            READ_ADDR,
            ACK,
        };

        void setCurPacket(PacketPtr pkt);

        void setInterruptPending();

        void clearInterrupt();

        explicit InterruptSM(BaseProxy *proxy)
            : proxy(proxy), state(IDLE), replyAddr(), interruptSize(),
              interruptPending(false), pio_mode(true) {}

        void start(Addr size)
        {
            interruptPending = false;
            interruptSize = size;
            state = IDLE;
        }

        PacketPtr tick();

        bool handleMemResp(PacketPtr pkt);

        std::string getStateName();

        bool isIdle();

        State getState();

       private:

        PacketPtr cur_pkt;
        BaseProxy * proxy;
        State state;
        Addr replyAddr;
        Addr interruptSize;
        bool interruptPending;
        bool pio_mode;
    };
};

#endif // __CPU_DTU_ACCEL_HH__
