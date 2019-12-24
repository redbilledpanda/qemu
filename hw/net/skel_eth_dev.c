/*
 * Skeletal Ethernet Device Emulation
 *
 * Copyright (c) 2016 Nagaraj Krishnamurthy
 * Written by Nagaraj Krishnamurthy
 *
 * This code is licensed under the GNU GPL v2
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 *
 * Email: nagaraj.kmurthy@gmail.com
 */

#include "hw/sysbus.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "hw/ptimer.h"
#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/hw.h"
#include "qemu/main-loop.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "qemu/typedefs.h"
#include "qemu/log.h"

#define TYPE_SKEL_ETH_DEV       "skel_eth_dev"

#define  MY_PHY_ADDR            7

#define ETH_CTRL		0x0
#define MAC_ADDR_HIGH		0x4
#define MAC_ADDR_LOW		0x8
#define MGMT_FRM		0xC
#define IE1		        0x10
#define INT_STS1		0x14
#define INT_CLR1		0x18

#define TX_DESC_BASE_LOW	0x1C
#define TX_DESC_BASE_HIGH	0x20
#define TX_DESC_FIFO_SIZE	0x24
#define TX_DESC_FIFO_HEAD	0x28
#define TX_DESC_FIFO_TAIL	0x2C

#define TX_DONE_DESC_BASE_LOW	0x30
#define TX_DONE_DESC_BASE_HIGH	0x34
#define TX_DONE_DESC_FIFO_SIZE	0x38
#define TX_DONE_DESC_FIFO_HEAD	0x3C
#define TX_DONE_DESC_FIFO_TAIL	0x40

#define RX_EMPTY_DESC_BASE_LOW	0x44
#define RX_EMPTY_DESC_BASE_HIGH	0x48
#define RX_EMPTY_DESC_FIFO_SIZE	0x4C
#define RX_EMPTY_DESC_FIFO_HEAD	0x50
#define RX_EMPTY_DESC_FIFO_TAIL	0x54

#define RX_DONE_DESC_BASE_LOW	0x58
#define RX_DONE_DESC_BASE_HIGH	0x5C
#define RX_DONE_DESC_FIFO_SIZE	0x60
#define RX_DONE_DESC_FIFO_HEAD	0x64
#define RX_DONE_DESC_FIFO_TAIL	0x68


#define ETH_CTRL_SPEED_SHIFT		8
#define ETH_CTRL_SPEED_MASK		(1<<8)
#define ETH_CTRL_DRT_SHIFT 		7
#define ETH_CTRL_DRT_MASK 		(1<<7)
#define ETH_CTRL_LOOP_SHIFT		6
#define ETH_CTRL_LOOP_MASK		(1<<6)
#define ETH_CTRL_FDEN_SHIFT		5
#define ETH_CTRL_FDEN_MASK		(1<<5)
#define ETH_CTRL_MODE_SHIFT		3
#define ETH_CTRL_RXE_SHIFT		2
#define ETH_CTRL_RXE_MASK		(1<<2)
#define ETH_CTRL_TXE_SHIFT		1
#define ETH_CTRL_TXE_MASK		(1<<1)
#define ETH_CTRL_ETH_EN_SHIFT		0
#define ETH_CTRL_ETH_EN_MASK		(1<<0)


#define MGMT_FRM_SP_SHIFT		30
#define MGMT_FRM_OP_SHIFT		28
#define MGMT_FRM_OP_MASK		0x30000000
#define MGMT_FRM_PHYAD_SHIFT		23
#define MGMT_FRM_PHYAD_MASK		0x0F800000
#define MGMT_FRM_REGAD_SHIFT		18
#define MGMT_FRM_TA_SHIFT		16
#define MGMT_FRM_DATA_SHIFT		0


#define IE1_MII_SHIFT	        	7
#define IE1_MII_MASK		        (1<<7)
#define IE1_LSCD_SHIFT	        	6
#define IE1_LSCD_MASK		        (1<<6)
#define IE1_TIMEOUT_ERROR_SHIFT		5
#define IE1_TIMEOUT_ERROR_MASK		(1<<5)
#define IE1_RX_BUS_ERROR_SHIFT		4
#define IE1_RX_BUS_ERROR_MASK		(1<<4)
#define IE1_TX_BUS_ERROR_SHIFT		3
#define IE1_TX_BUS_ERROR_MASK		(1<<3)
#define IE1_RX_QUEUE_EMPTY_SHIFT	2
#define IE1_RX_QUEUE_EMPTY_MASK		(1<<2)
#define IE1_RX_QUEUE_SHIFT		1
#define IE1_RX_QUEUE_MASK		(1<<1)
#define IE1_TX_Q_CMPLT_SHIFT		0
#define IE1_TX_Q_CMPLT_MASK		(1<<0)

#define INT_STS1_MII_SHIFT		7
#define INT_STS1_MII_MASK		(1<<7)
#define INT_STS1_LSCD_SHIFT		6
#define INT_STS1_LSCD_MASK		(1<<6)
#define INT_STS1_TIMEOUT_ERROR_SHIFT	5
#define INT_STS1_TIMEOUT_ERROR_MASK	(1<<5)
#define INT_STS1_RX_BUS_ERROR_SHIFT	4
#define INT_STS1_RX_BUS_ERROR_MASK	(1<<4)
#define INT_STS1_TX_BUS_ERROR_SHIFT	3
#define INT_STS1_TX_BUS_ERROR_MASK	(1<<3)
#define INT_STS1_RX_QUEUE_EMPTY_SHIFT	2
#define INT_STS1_RX_QUEUE_EMPTY_MASK	(1<<2)
#define INT_STS1_RX_QUEUE_SHIFT		1
#define INT_STS1_RX_QUEUE_MASK		(1<<1)
#define INT_STS1_TX_Q_CMPLT_SHIFT	0
#define INT_STS1_TX_Q_CMPLT_MASK	(1<<0)

#define INT_CLR1_MII_SHIFT		7
#define INT_CLR1_MII_MASK		(1<<7)
#define INT_CLR1_LSCD_SHIFT		6
#define INT_CLR1_LSCD_MASK		(1<<6)
#define INT_CLR1_TIMEOUT_ERROR_SHIFT	5
#define INT_CLR1_TIMEOUT_ERROR_MASK	(1<<5)
#define INT_CLR1_RX_BUS_ERROR_SHIFT	4
#define INT_CLR1_RX_BUS_ERROR_MASK	(1<<4)
#define INT_CLR1_TX_BUS_ERROR_SHIFT	3
#define INT_CLR1_TX_BUS_ERROR_MASK	(1<<3)
#define INT_CLR1_RX_QUEUE_EMPTY_SHIFT	2
#define INT_CLR1_RX_QUEUE_EMPTY_MASK	(1<<2)
#define INT_CLR1_RX_QUEUE_SHIFT		1
#define INT_CLR1_RX_QUEUE_MASK		(1<<1)
#define INT_CLR1_TX_Q_CMPLT_SHIFT	0
#define INT_CLR1_TX_Q_CMPLT_MASK	(1<<0)



typedef union
{
    struct
    {
        unsigned int                    tx_buf_address;
        unsigned int                    buffer_length:16;
        unsigned int                    rsvd:16;
    }u;
    unsigned int val_l;
    unsigned int val_h;
}tx_desc_t;

typedef union
{
    struct
    {
        unsigned int                    send_done:1;
        unsigned int                    index:10;
        unsigned int                    rsvd:21;
    }u;
    unsigned int val;
}tx_done_desc_t;


typedef union
{
    struct
    {
        unsigned int                    rx_buf_address;
        unsigned int                    buffer_length:16;
        unsigned int                    rsvd:16;
    }u;
    unsigned int val;
}rx_empty_desc_t;

typedef union
{
    struct
    {
        unsigned int                    receive_done:1;
        unsigned int                    index:10;
        unsigned int                    rsvd:9;
        unsigned int                    cur_buf_length:12;
    }u;
    unsigned int val_l;
    unsigned int val_h;
}rx_done_desc_t;



typedef struct {
    SysBusDevice busdev;
    NICState *nic;
    NICConf conf;
    qemu_irq irq;
    MemoryRegion mmio;
    ptimer_state *timer;

    uint32_t eth_ctrl;
    uint32_t mac_addr_high;
    uint32_t mac_addr_low;
    uint32_t mgmt_frm;
    uint32_t ie1;
    uint32_t int_sts1;
    uint32_t int_clr1;
    uint32_t tx_desc_base_low;
    uint32_t tx_desc_base_high;
    uint32_t tx_desc_fifo_size;
    uint32_t tx_desc_fifo_head;
    uint32_t tx_desc_fifo_tail;
    uint32_t tx_done_desc_base_low;
    uint32_t tx_done_desc_base_high;
    uint32_t tx_done_desc_fifo_size;
    uint32_t tx_done_desc_fifo_head;
    uint32_t tx_done_desc_fifo_tail;

    uint32_t rx_empty_desc_base_low;
    uint32_t rx_empty_desc_base_high;
    uint32_t rx_empty_desc_fifo_size;
    uint32_t rx_empty_desc_fifo_head;
    uint32_t rx_empty_desc_fifo_tail;

    uint32_t rx_done_desc_base_low;
    uint32_t rx_done_desc_base_high;
    uint32_t rx_done_desc_fifo_size;
    uint32_t rx_done_desc_fifo_head;
    uint32_t rx_done_desc_fifo_tail;


    //These are PHY registers
    uint32_t phy_status;
    uint32_t phy_control;
    uint32_t phy_advertise;
    uint32_t phy_int;
    uint32_t phy_int_mask;

    uint32_t mac_mii_data;
    unsigned char scatter_gather_buf[1650];

} skel_eth_device_state;


#define DEBUG_SKE_ETH_DEVICE

#ifdef DEBUG_SKE_ETH_DEVICE
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "skel_eth_device: " fmt , ## __VA_ARGS__); } while (0)
#define BADF(fmt, ...) \
do { hw_error("skel_eth_device: error: " fmt , ## __VA_ARGS__);} while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#define BADF(fmt, ...) \
do { fprintf(stderr, "skel_eth_device: error: " fmt , ## __VA_ARGS__);} while (0)
#endif

void skel_eth_device_init(NICInfo *nd, uint32_t base, qemu_irq irq);

/*
--------------------------------------------------------------------------------
Phy Control register definition
--------------------------------------------------------------------------------
0.15 Reset 
        1 = PHY reset 0 = normal operation R/W SC
0.14 Loopback 
        1 = enable loopback mode 0 = disable loopback mode R/W
0.13 Speed Selection (LSB) 0.6:0.13 
        11= Reserved 10= 1000 Mb/s 01= 100 Mb/s 00= 10 Mb/s R/W
0.12 Auto-Negotiation Enable 
        1 = enable Auto-Negotiation process 
        0 = disable Auto-Negotiation process R/W
0.11 Power Down 
        1 = power down 0 = normal operationb R/W
0.10 Isolate 
        1 = electrically Isolate PHY from MII or GMII 
        0 = normal operationb R/W
0.9 Restart Auto-Negotiation 
        1 = restart Auto-Negotiation process 
        0 = normal operation R/W SC
0.8 Duplex Mode 
        1 = full duplex 
        0 = half duplex R/W
0.7 Collision Test 
        1 = enable COL signal test 
        0 = disable COL signal test R/W
0.6 Speed Selection (MSB) 0.6:0.13 
        11= Reserved 
        10= 1000 Mb/s 
        01= 100 Mb/s 
        00= 10 Mb/s R/W
0.5 Unidirectional enable 
        When bit 0.12 is one or bit 0.8 is zero, this bit is ignored. 
        When bit 0.12 is zero and bit 0.8 is one: 
                1 = Enable transmit from media independent interface 
                                regardless of whether the PHY has determined 
                                that a valid link has been established
                0 = Enable transmit from media independent interface 
                                only when the PHY has determined that a valid 
                                link has been established R/W
0.4:0 Reserved Write as 0, ignore on read

--------------------------------------------------------------------------------
Phy Status register definition
--------------------------------------------------------------------------------
1.15 100BASE-T4 
        1 = PHY able to perform 100BASE-T4 
        0 = PHY not able to perform 100BASE-T4 RO
1.14 100BASE-X Full Duplex 
        1 = PHY able to perform full duplex 100BASE-X 
        0 = PHY not able to perform full duplex 100BASE-X RO
1.13 100BASE-X Half Duplex 
        1 = PHY able to perform half duplex 100BASE-X 
        0 = PHY not able to perform half duplex 100BASE-X RO
1.12 10 Mb/s Full Duplex 
        1 = PHY able to operate at 10 Mb/s in full duplex mode 
        0 = PHY not able to operate at 10 Mb/s in full duplex mode RO
1.11 10 Mb/s Half Duplex 
        1 = PHY able to operate at 10 Mb/s in half duplex mode 
        0 = PHY not able to operate at 10 Mb/s in half duplex mode RO
1.10 100BASE-T2 Full Duplex 
        1 = PHY able to perform full duplex 100BASE-T2 
        0 = PHY not able to perform full duplex 100BASE-T2 RO
1.9 100BASE-T2 Half Duplex 
        1 = PHY able to perform half duplex 100BASE-T2 
        0 = PHY not able to perform half duplex 100BASE-T2 RO
1.8 Extended Status 
        1 = Extended status information in Register 15 
        0 = No extended status information in Register 15 RO
1.7 Unidirectional ability
        1 = PHY able to transmit from media independent interface regardless 
                of whether the PHY has determined that a valid link has 
                been established
        0 = PHY able to transmit from media independent interface only when 
                the PHY has determined that a valid link has been established 
        RO
1.6 MF Preamble Suppression
        1 = PHY will accept management frames with preamble suppressed.
        0 = PHY will not accept management frames with preamble suppressed.  RO
1.5 Auto-Negotiation Complete
        1 = Auto-Negotiation process completed
        0 = Auto-Negotiation process not completed RO
1.4 Remote Fault
        1 = remote fault condition detected
        0 = no remote fault condition detected RO/ LH
1.3 Auto-Negotiation Ability
        1 = PHY is able to perform Auto-Negotiation
        0 = PHY is not able to perform Auto-Negotiation RO
1.2 Link Status
        1 = link is up
        0 = link is down RO/ LL
1.1 Jabber Detect
        1 = jabber condition detected
        0 = no jabber condition detected RO/ LH
1.0 Extended Capability
        1 = extended register capabilities
        0 = basic register set capabilities only RO
--------------------------------------------------------------------------------
*/

static void phy_update_link(skel_eth_device_state *s);
static void skel_eth_device_update(skel_eth_device_state *s);

static void phy_reset(skel_eth_device_state *s)
{
    s->phy_status = 0x7809;
    s->phy_control = 0x3000; //Speed 100Mbit, autonegotiation on
    s->phy_advertise = 0x01e1;
    s->phy_int_mask = 0;
    s->phy_int = 0;
    phy_update_link(s);
}


static void phy_update_irq(skel_eth_device_state *s)
{
    skel_eth_device_update(s);
}

static void phy_update_link(skel_eth_device_state *s)
{
    DPRINTF("%s\n", __FUNCTION__);
    /* Autonegotiation status mirrors link status.  */
    if (qemu_get_queue(s->nic)->link_down) {
        s->phy_status &= ~0x0024;
    } else {
        s->phy_status |= 0x0024;
    }
    phy_update_irq(s);
}

static void skel_eth_device_set_link(NetClientState *nc)
{
    DPRINTF("%s\n", __FUNCTION__);
    phy_update_link(qemu_get_nic_opaque(nc));
}

static void skel_eth_device_update(skel_eth_device_state *s)
{
    int level;

    level = (s->int_sts1 & s->ie1) != 0;
    //if(level)
    {
        //DPRINTF("%s Raising interrupt %x\n", __FUNCTION__, s->int_sts1);
        qemu_set_irq(s->irq, level);
    }
}

static void skel_eth_device_mac_changed(skel_eth_device_state *s)
{
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}


static void skel_eth_device_reset(DeviceState *d)
{

    skel_eth_device_state *s = 
        OBJECT_CHECK(skel_eth_device_state, d, TYPE_SKEL_ETH_DEV);

    DPRINTF("%s\n", __FUNCTION__);

    s->eth_ctrl = 0x8;
    s->mac_addr_high = 0;
    s->mac_addr_low = 0;
    //s->mgmt_frm = 0xA0020000;
    s->mgmt_frm = 0;
    s->ie1 = 0;
    s->int_sts1 = 0;
    s->int_clr1 = 0;
    s->tx_desc_base_low = 0;
    s->tx_desc_base_high = 0;
    s->tx_desc_fifo_size = 0;
    s->tx_desc_fifo_head = 0;
    s->tx_desc_fifo_tail = 0;
    s->tx_done_desc_base_low = 0;
    s->tx_done_desc_base_high = 0;
    s->tx_done_desc_fifo_size = 0;
    s->tx_done_desc_fifo_head = 0;
    s->tx_done_desc_fifo_tail = 0;
    s->rx_empty_desc_base_low = 0;
    s->rx_empty_desc_base_high = 0;
    s->rx_empty_desc_fifo_size = 0;
    s->rx_empty_desc_fifo_head = 0;
    s->rx_empty_desc_fifo_tail = 0;
    s->rx_done_desc_base_low = 0;
    s->rx_done_desc_base_high = 0;
    s->rx_done_desc_fifo_size = 0;
    s->rx_done_desc_fifo_head = 0;
    s->rx_done_desc_fifo_tail = 0;

    phy_reset(s);
}

static int skel_eth_device_can_receive(NetClientState *nc)
{
    skel_eth_device_state *s = qemu_get_nic_opaque(nc);

    //Rx descriptor fifo underflow 
    if(s->rx_empty_desc_fifo_tail == s->rx_empty_desc_fifo_head){
        fprintf(stderr, "%s Error, Rx Descriptor underflow\n", __FUNCTION__);
        return 0;
    }
    return 1;
}

static ssize_t skel_eth_device_receive(NetClientState *nc, const uint8_t *buf,
        size_t size)
{
    skel_eth_device_state *s = qemu_get_nic_opaque(nc);
    rx_empty_desc_t* rx_empty_desc;
    rx_done_desc_t* rx_done_desc;
    rx_empty_desc_t rx_empty_desc_local;
    rx_done_desc_t rx_done_desc_local;

    DPRINTF("%s packet of size:%zu\n", __FUNCTION__, size);

    //Rx descriptor fifo underflow 
    if(s->rx_empty_desc_fifo_tail == s->rx_empty_desc_fifo_head){
        fprintf(stderr, "%s Error, Rx Descriptor underflow\n", __FUNCTION__);
        // We will just ignore this packet
        return size;
    }

    // Get the current rx_empty_desc
    rx_empty_desc = (rx_empty_desc_t*)(s->rx_empty_desc_base_low + 
            (s->rx_empty_desc_fifo_tail * sizeof(rx_empty_desc_t)));
    cpu_physical_memory_read((hwaddr)rx_empty_desc, &rx_empty_desc_local, 
            sizeof(rx_empty_desc_t));


    // Copy the payload to the buffer pointed to by the rx descriptor
    if(size > rx_empty_desc_local.u.buffer_length)
    {
        fprintf(stderr, 
                "%s Error, Rx Descriptor size(%d) is less than packet size(%zu)\n", 
                __FUNCTION__, rx_empty_desc_local.u.buffer_length, size);
        return -1;
    }
    cpu_physical_memory_write(rx_empty_desc_local.u.rx_buf_address, buf, size);

    // Update rx_done_desc descriptor
    memset(&rx_done_desc_local, 0, sizeof(rx_done_desc_t));
    rx_done_desc_local.u.receive_done=1;

    //index points to the current rx_empty_desc
    rx_done_desc_local.u.index = s->rx_empty_desc_fifo_tail; 
    rx_done_desc_local.u.cur_buf_length = size; 

    // Get the pointer to current rx_done_desc
    rx_done_desc = (rx_done_desc_t*)(s->rx_done_desc_base_low + 
            (s->rx_done_desc_fifo_head * sizeof(rx_done_desc_t)));
    cpu_physical_memory_write((hwaddr)rx_done_desc, &rx_done_desc_local, 
            sizeof(rx_done_desc_t));

    s->rx_empty_desc_fifo_tail+=1;
    if(s->rx_empty_desc_fifo_tail >= s->rx_empty_desc_fifo_size) {
        s->rx_empty_desc_fifo_tail=0;
    }

    // Advance the rx_done_desc_fifo_head
    s->rx_done_desc_fifo_head+=1;
    if(s->rx_done_desc_fifo_head >= s->rx_empty_desc_fifo_size) {
        s->rx_done_desc_fifo_head=0;
    }

    // Assert interrupt complete interrupt
    s->int_sts1 |= 2; //Rx-Queue Complete interrupt
    skel_eth_device_update(s);

    return size;
}

static void do_tx_packet(skel_eth_device_state *s)
{
    uint8_t buf[1700];
    tx_desc_t* tx_desc;
    tx_done_desc_t* tx_done_desc;
    tx_desc_t tx_desc_local;
    tx_done_desc_t tx_done_desc_local;


    DPRINTF("skel_eth_device %s\n", __FUNCTION__);

    while(s->tx_desc_fifo_head != s->tx_desc_fifo_tail)
    {
        // Get the current tx_desc
        tx_desc = (tx_desc_t*)(s->tx_desc_base_low + 
                (s->tx_desc_fifo_tail * sizeof(tx_desc_t)));

        cpu_physical_memory_read((hwaddr)tx_desc, &tx_desc_local, sizeof(tx_desc_t));

        {
            DPRINTF("Sending packet tx_desc: pkt addr:%x, size:%d\n",
                    tx_desc_local.u.tx_buf_address,
                    tx_desc_local.u.buffer_length);

            cpu_physical_memory_read(tx_desc_local.u.tx_buf_address, buf,
                    tx_desc_local.u.buffer_length);
            qemu_send_packet(qemu_get_queue(s->nic), (const uint8_t*)(buf),
                    tx_desc_local.u.buffer_length);

            // Move this descriptor to done
            memset(&tx_done_desc_local, 0, sizeof(tx_done_desc_t));
            tx_done_desc_local.u.send_done=1;
            //index points to the current tx_desc
            tx_done_desc_local.u.index = s->tx_desc_fifo_tail; 

            // Get the pointer to current tx_done_desc
            tx_done_desc = (tx_done_desc_t*)(s->tx_done_desc_base_low + 
                    (s->tx_done_desc_fifo_head * sizeof(tx_done_desc_t)));

            // Copy the updated tx_done_desc to the actual location 
            cpu_physical_memory_write((hwaddr)tx_done_desc, &tx_done_desc_local, 
                    sizeof(tx_done_desc_t));

            // Advance the tx_desc_fifo_tail
            s->tx_desc_fifo_tail+=1;
            if(s->tx_desc_fifo_tail >= s->tx_desc_fifo_size) {
                s->tx_desc_fifo_tail=0;
            }

            // Advance the tx_done_desc_fifo_head
            s->tx_done_desc_fifo_head+=1;
            if(s->tx_done_desc_fifo_head >= s->tx_desc_fifo_size) {
                s->tx_done_desc_fifo_head=0;
            }

        }
    }
    // Assert Transmit complete interrupt
    s->int_sts1 |= 1; 
    skel_eth_device_update(s);
}

static uint32_t do_phy_read(skel_eth_device_state *s, int reg)
{
    uint32_t val;

    switch (reg) {
        case 0: /* Basic Control */
            return s->phy_control;
        case 1: /* Basic Status */
            return s->phy_status;
        case 2: /* ID1 */
            return MY_PHY_ADDR;
        case 3: /* ID2 */
            return 0xc0d1;
        case 4: /* Auto-neg advertisement */
            return s->phy_advertise;
        case 5: /* Auto-neg Link Partner Ability */
            return 0x0f71;
        case 6: /* Auto-neg Expansion */
            return 1;
        case 29: /* Interrupt source.  */
            val = s->phy_int;
            s->phy_int = 0;
            phy_update_irq(s);
            return val;
        case 30: /* Interrupt mask */
            return s->phy_int_mask;
        default:
            BADF("PHY read reg %d\n", reg);
            return 0;
    }
}

static void do_phy_write(skel_eth_device_state *s, int reg, uint32_t val)
{
    switch (reg) {
        case 0: /* Basic Control */
            if (val & 0x8000) {
                phy_reset(s);
                break;
            }
            s->phy_control = val & 0x7980;
            /* Complete autonegotiation immediately.  */
            if (val & 0x1000) {
                s->phy_status |= 0x0020;
            }
            break;
        case 4: /* Auto-neg advertisement */
            s->phy_advertise = (val & 0x2d7f) | 0x80;
            break;
            /* TODO 17, 18, 27, 31 */
        case 30: /* Interrupt mask */
            s->phy_int_mask = val & 0xff;
            phy_update_irq(s);
            break;
        default:
            BADF("PHY write reg %d = 0x%04x\n", reg, val);
    }
}


static void skel_eth_device_tick(void *opaque)
{
    skel_eth_device_state *s = (skel_eth_device_state *)opaque;
    skel_eth_device_update(s);
}

static void skel_eth_device_writel(void *opaque, hwaddr offset,
        uint64_t val, unsigned size)
{
    skel_eth_device_state *s = (skel_eth_device_state *)opaque;
    DeviceState *dev = DEVICE(&s->busdev);

    offset &= 0xff;
    //fprintf(stderr, "Write reg 0x%02x = 0x%08x\n", (int)offset, val);

    switch (offset) {
        case ETH_CTRL:
            if (((s->eth_ctrl & 1) != (val&1)) && ((val&1) == 1))
            {
                skel_eth_device_reset(dev);
            }
            s->eth_ctrl=val;
            break;

        case MAC_ADDR_HIGH:
            s->mac_addr_high=val;
            s->conf.macaddr.a[4] = val & 0xff;
            s->conf.macaddr.a[5] = (val >> 8) & 0xff;
            skel_eth_device_mac_changed(s);
            break;

        case MAC_ADDR_LOW:
            s->mac_addr_low=val;
            s->conf.macaddr.a[0] = val & 0xff;
            s->conf.macaddr.a[1] = (val >> 8) & 0xff;
            s->conf.macaddr.a[2] = (val >> 16) & 0xff;
            s->conf.macaddr.a[3] = (val >> 24) & 0xff;
            skel_eth_device_mac_changed(s);
            break;

        case MGMT_FRM:
            //Read/Write PHY register here
            s->mgmt_frm=val;
            if(((val & MGMT_FRM_PHYAD_MASK) >> MGMT_FRM_PHYAD_SHIFT) 
                    == MY_PHY_ADDR)
            {
                switch((val & MGMT_FRM_OP_MASK) >> MGMT_FRM_OP_SHIFT) 
                {
                    case 1: //write operation
                        do_phy_write(s, (val >> MGMT_FRM_REGAD_SHIFT) & 0x1f, 
                                val & 0xFFFF);
                        break;
                    case 2: //read operation
                        s->mac_mii_data = do_phy_read(s, 
                                (val >> MGMT_FRM_REGAD_SHIFT) & 0x1f);
                        break;
                    default:
                        s->mac_mii_data =0;
                        fprintf(stderr, "%s Invalid MGMT_FRM Operation\n", 
                                __FUNCTION__);
                        break;
                }
            }
            else 
            {
                s->mac_mii_data =0;
            }
            break;

        case IE1:
            s->ie1=val;
            if(s->int_sts1 & s->ie1) {
                skel_eth_device_update(s);
            }
            break;

        case INT_STS1:
            s->int_sts1=val;
            break;

        case INT_CLR1:
            s->int_sts1 &= ~(val);
            skel_eth_device_update(s);
            break;

        case TX_DESC_BASE_LOW:
            s->tx_desc_base_low=val;
            break;

        case TX_DESC_BASE_HIGH:
            s->tx_desc_base_high=val;
            break;

        case TX_DESC_FIFO_SIZE:
            s->tx_desc_fifo_size=val;
            break;

        case TX_DESC_FIFO_HEAD:
            s->tx_desc_fifo_head=val;
            if(s->tx_desc_fifo_head != s->tx_desc_fifo_tail)
            {
                do_tx_packet(s);
            }
            break;

        case TX_DESC_FIFO_TAIL:
            s->tx_desc_fifo_tail=val;
            break;

        case TX_DONE_DESC_BASE_LOW:
            s->tx_done_desc_base_low=val;
            break;

        case TX_DONE_DESC_BASE_HIGH:
            s->tx_done_desc_base_high=val;
            break;

        case TX_DONE_DESC_FIFO_SIZE:
            s->tx_done_desc_fifo_size=val;
            break;

        case TX_DONE_DESC_FIFO_HEAD:
            s->tx_done_desc_fifo_head=val;
            break;

        case TX_DONE_DESC_FIFO_TAIL:
            s->tx_done_desc_fifo_tail=val;
            break;

        case RX_EMPTY_DESC_BASE_LOW:
            s->rx_empty_desc_base_low=val;
            break;

        case RX_EMPTY_DESC_BASE_HIGH:
            s->rx_empty_desc_base_high=val;
            break;

        case RX_EMPTY_DESC_FIFO_SIZE:
            s->rx_empty_desc_fifo_size=val;
            break;

        case RX_EMPTY_DESC_FIFO_HEAD:
            s->rx_empty_desc_fifo_head=val;
            break;

        case RX_EMPTY_DESC_FIFO_TAIL:
            s->rx_empty_desc_fifo_tail=val;
            break;

        case RX_DONE_DESC_BASE_LOW:
            s->rx_done_desc_base_low=val;
            break;

        case RX_DONE_DESC_BASE_HIGH:
            s->rx_done_desc_base_high=val;
            break;

        case RX_DONE_DESC_FIFO_SIZE:
            s->rx_done_desc_fifo_size=val;
            break;

        case RX_DONE_DESC_FIFO_HEAD:
            s->rx_done_desc_fifo_head=val;
            break;

        case RX_DONE_DESC_FIFO_TAIL:
            s->rx_done_desc_fifo_tail=val;
            break;

        default:
            hw_error("skel_eth_device_write: Bad reg 0x%x = %x\n", 
                    (int)offset, (int)val);
            break;
    }
}


static uint64_t skel_eth_device_readl(void *opaque, hwaddr offset,
        unsigned size)
{

    skel_eth_device_state *s = (skel_eth_device_state *)opaque;
    //fprintf(stderr, "Read reg 0x%02x\n", (int)offset);

    switch (offset) {
        case ETH_CTRL:
            return s->eth_ctrl;
        case MAC_ADDR_HIGH:
            return s->mac_addr_high;
        case MAC_ADDR_LOW:
            return s->mac_addr_low;
        case MGMT_FRM:
            //Read/Write PHY register here
            return s->mac_mii_data ;
        case IE1:
            return s->ie1;
        case INT_STS1:
            return s->int_sts1;
        case INT_CLR1:
            return s->int_clr1;
        case TX_DESC_BASE_LOW:
            return s->tx_desc_base_low;
        case TX_DESC_BASE_HIGH:
            return s->tx_desc_base_high;
        case TX_DESC_FIFO_SIZE:
            return s->tx_desc_fifo_size;
        case TX_DESC_FIFO_HEAD:
            return s->tx_desc_fifo_head;
        case TX_DESC_FIFO_TAIL:
            return s->tx_desc_fifo_tail;
        case TX_DONE_DESC_BASE_LOW:
            return s->tx_done_desc_base_low;
        case TX_DONE_DESC_BASE_HIGH:
            return s->tx_done_desc_base_high;
        case TX_DONE_DESC_FIFO_SIZE:
            return s->tx_done_desc_fifo_size;
        case TX_DONE_DESC_FIFO_HEAD:
            return s->tx_done_desc_fifo_head;
        case TX_DONE_DESC_FIFO_TAIL:
            return s->tx_done_desc_fifo_tail;

        case RX_EMPTY_DESC_BASE_LOW:
            return s->rx_empty_desc_base_low;
        case RX_EMPTY_DESC_BASE_HIGH:
            return s->rx_empty_desc_base_high;
        case RX_EMPTY_DESC_FIFO_SIZE:
            return s->rx_empty_desc_fifo_size;
        case RX_EMPTY_DESC_FIFO_HEAD:
            return s->rx_empty_desc_fifo_head;
        case RX_EMPTY_DESC_FIFO_TAIL:
            return s->rx_empty_desc_fifo_tail;
        case RX_DONE_DESC_BASE_LOW:
            return s->rx_done_desc_base_low;
        case RX_DONE_DESC_BASE_HIGH:
            return s->rx_done_desc_base_high;
        case RX_DONE_DESC_FIFO_SIZE:
            return s->rx_done_desc_fifo_size;
        case RX_DONE_DESC_FIFO_HEAD:
            return s->rx_done_desc_fifo_head;
        case RX_DONE_DESC_FIFO_TAIL:
            return s->rx_done_desc_fifo_tail;
    }
    hw_error("skel_eth_device_read: Bad reg 0x%x\n", (int)offset);
    return 0;
}

static const MemoryRegionOps skel_eth_device_mem_ops = {
    .read = skel_eth_device_readl,
    .write = skel_eth_device_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void skel_eth_device_cleanup(NetClientState *nc)
{
    fprintf(stderr, "%s\n", __FUNCTION__);
    skel_eth_device_state *s = qemu_get_nic_opaque(nc);

    s->nic = NULL;
}

static NetClientInfo net_skel_eth_device_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = skel_eth_device_can_receive,
    .receive = skel_eth_device_receive,
    .cleanup = skel_eth_device_cleanup,
    .link_status_changed = skel_eth_device_set_link,
};

static void skel_eth_device_init1(DeviceState *dev, Error **errp)
{
    skel_eth_device_state *s = OBJECT_CHECK(skel_eth_device_state, dev, TYPE_SKEL_ETH_DEV);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    //QEMUBH *bh;
    const MemoryRegionOps *mem_ops = &skel_eth_device_mem_ops;

    DPRINTF("%s\n", __FUNCTION__);
    qemu_log_mask(LOG_UNIMP, "%s\n", __FUNCTION__);

    memory_region_init_io(&s->mmio, OBJECT(dev), mem_ops, s, "skel_eth_device-mmio", 0x100);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);

    s->nic = qemu_new_nic(&net_skel_eth_device_info, &s->conf,
            object_get_typename(OBJECT(dev)), dev->id, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
    DPRINTF("%s DONE!\n", __FUNCTION__);
    //bh = qemu_bh_new(skel_eth_device_tick, s);
    s->timer = ptimer_init(skel_eth_device_tick, s, PTIMER_POLICY_DEFAULT);
    ptimer_set_freq(s->timer, 10000);
    ptimer_set_limit(s->timer, 0xffff, 1);

    //return 0;
}

static Property skel_eth_device_properties[] = {
    DEFINE_NIC_PROPERTIES(skel_eth_device_state, conf),
    DEFINE_PROP_END_OF_LIST(),
};


static void skel_eth_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    //SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    DPRINTF("%s\n", __FUNCTION__);

    dc->realize = skel_eth_device_init1;
    dc->reset = skel_eth_device_reset;
    dc->props = skel_eth_device_properties;
}

static const TypeInfo skel_eth_device_info = {
    .name          = TYPE_SKEL_ETH_DEV,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(skel_eth_device_state),
    .class_init    = skel_eth_device_class_init,
};

static void skel_eth_device_register_types(void)
{
    DPRINTF("%s\n", __FUNCTION__);
    type_register_static(&skel_eth_device_info);
}

/* Legacy helper function.  Should go away when machine config files are
   implemented.  */
void skel_eth_device_init(NICInfo *nd, uint32_t base, qemu_irq irq)
{
    DeviceState *dev;
    SysBusDevice *s;
    DPRINTF("%s\n", __FUNCTION__);

    qemu_check_nic_model(nd, TYPE_SKEL_ETH_DEV);
    dev = qdev_create(NULL, TYPE_SKEL_ETH_DEV);
    qdev_set_nic_properties(dev, nd);
    qdev_init_nofail(dev);
    s = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(s, 0, base);
    sysbus_connect_irq(s, 0, irq);
}

type_init(skel_eth_device_register_types)
