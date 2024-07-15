//! This file contains the structs that are used to access device registers and configuration values to write to registers, taken from the datasheet.
//! 
//! The registers are divided into multiple structs because we need to separate out the 
//! receive and transmit queue registers and store them separately for virtualization. 
//! 
//! The 7 structs which cover the registers of the entire memory-mapped region are:
//! * `IntelIxgbeRegisters1`
//! * `IntelIxgbeRxRegisters1`
//! * `IntelIxgbeRegisters2`
//! * `IntelIxgbeTxRegisters`
//! * `IntelIxgbeMacRegisters`
//! * `IntelIxgbeRxRegisters2`
//! * `IntelIxgbeRegisters3`
//! 
//! Some of the type classifiers may be more restrictive than specified in the datasheet.
//! For example, setting RW fields to RO or keeping registers private.
//! This simply indicates that the extra functions are currently not used in the driver, 
//! and so we haven't implemented the necessary checks for safe access.


use volatile::{Volatile, ReadOnly, WriteOnly};
use zerocopy::FromBytes;
use bit_field::BitField;
use num_enum::TryFromPrimitive;
use crate::hal::{
    *
};

/// The layout in memory of the first set of general registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeRegisters1 {
    /// Device Control Register
    ctrl:                               Volatile<u32>,          // 0x0
    _padding0:                          [u8; 4],                // 0x4 - 0x7
    
    /// Device Status Register
    status:                             ReadOnly<u32>,          // 0x8
    _padding1:                          [u8; 12],               // 0xC - 0x17

    /// Extended Device Control Register
    ctrl_ext:                           Volatile<u32>,          // 0x18
    _padding2:                          [u8; 12],               // 0x1C - 0x27

    /// I2C Control
    i2cctl:                             Volatile<u32>,          // 0x28
    _padding3:                          [u8; 2004],             // 0x2C - 0x7FF

    /// Extended Interrupt Cause Register
    eicr:                           Volatile<u32>,          // 0x800
    _padding4:                          [u8; 4],                // 0x804 - 0x807

    /// Extended Interrupt Cause Set Register
    eics:                               WriteOnly<u32>,         // 0x808
    _padding5:                          [u8; 4],                // 0x80C - 0x80F

    /// Extended Interrupt Auto Clear Register
    eiac:                               Volatile<u32>,          // 0x810; 
    _padding6:                          [u8; 12],               // 0x814 - 0x81F 
    
    /// Extended Interrupt Throttle Registers
    eitr:                               [Volatile<u32>; 24],    // 0x820 - 0x87F; 

    /// Extended Interrupt Mask Set/ Read Register
    eims:                               Volatile<u32>,          // 0x880; 
    _padding7:                          [u8; 4],                // 0x884 - 0x887

    /// Extended Interrupt Mask Clear Register
    eimc:                               WriteOnly<u32>,         // 0x888; 
    _padding8:                          [u8; 4],                // 0x88C - 0x88F     

    /// Extended Interrupt Auto Mask Enable Register
    eiam:                               Volatile<u32>,          // 0x890; 
    _padding9:                          [u8; 4],                // 0x894 - 0x897

    /// General Purpose Interrupt Enable
    gpie:                               Volatile<u32>,          // 0x898; 
    _padding10:                         [u8; 100],              // 0x89C - 0x8FF

    /// Interrupt Vector Allocation Registers
    ivar:                               [Volatile<u32>; 64],    // 0x900 - 0x9FF  
    _padding11:                         [u8; 1536],             // 0xA00 - 0xFFF

} // 1 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeRegisters1>(), 4096);


impl IntelIxgbeRegisters1 {
    pub fn ctrl_reset(&mut self) {
        let val = self.ctrl.read();
        self.ctrl.write(val | CTRL_RST | CTRL_LRST); // TinyNF didn't do LRST
    }

    // Prevents DPDK bug 23
    // We can make this more generic in the future by creating a U31 that allows any bit in a u32 to be set except for the MSB
    pub fn eimc_disable_interrupts(&mut self) {
        self.eimc.write(DISABLE_INTERRUPTS);
    }

    pub fn ctrl_ext_no_snoop_disable(&mut self) {
        self.ctrl_ext.write(self.ctrl_ext.read() | CTRL_EXT_NO_SNOOP_DIS);
    }
}

/// The layout in memory of the first set of receive queue registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeRxRegisters1 {
    /// First set of Rx Registers for 64 Rx Queues
    pub rx_regs1:                       [RegistersRx; 64],      // 0x1000 - 0x1FFF

} // 1 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeRxRegisters1>(), 4096);

/// The layout in memory of the second set of general registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeRegisters2 {
    _padding1:                          [u8; 3840],             // 0x2000 - 0x2EFF
    
    /// Receive DMA Control Register
    rdrxctl:                            Volatile<u32>,          // 0x2F00;
    _padding2:                          [u8; 252],              // 0x2F04 - 0x2FFF

    /// Receive Control Register
    rxctrl:                             Volatile<u32>,          // 0x3000;
    _padding3:                          [u8; 508],              // 0x3004 - 0x31FF

    /// Flow Control Transmit Timer Value
    pub fcttv:                          [Volatile<u32>;4],      // 0x3200 - 0x320F
    _padding4:                          [u8; 16],               // 0x3210 - 0x321F

    /// Flow Control Receive Threshold Low
    fcrtl:                              [Volatile<u32>;8],      // 0x3220 - 0x323F
    _padding5:                          [u8; 32],               // 0x3240 - 0x325F 

    /// Flow Control Receive Threshold High
    fcrth:                              [Volatile<u32>;8],      // 0x3260 - 0x327F
    _padding6:                          [u8; 32],               // 0x3280 - 0x329F

    /// Flow Control Refresh Threshold Value
    fcrtv:                              Volatile<u32>,          // 0x32A0;
    _padding7:                          [u8; 2396],             // 0x32A4 - 0x3CFF

    ///Receive Packet Buffer Size
    rxpbsize:                           [Volatile<u32>;8],      // 0x3C00   
    _padding8:                          [u8; 224],              // 0x3C20 - 0x3CFF        

    /// Flow Control Configuration
    fccfg:                              Volatile<u32>,          // 0x3D00;
    _padding9:                          [u8; 880],              // 0x3D04 - 0x4073

    /// Good Packets Received Count
    pub gprc:                           ReadOnly<u32>,          // 0x4074
    _padding10:                         [u8; 8],                // 0x4078 - 0x407F

    /// Good Packets Transmitted Count
    pub gptc:                           ReadOnly<u32>,          // 0x4080
    _padding11:                         [u8; 4],                // 0x4084 - 0x4087 

    /// Good Octets Received Count Low
    pub gorcl:                          ReadOnly<u32>,          // 0x4088

    /// Good Octets Received Count High
    pub gorch:                          ReadOnly<u32>,          // 0x408C
    
    /// Good Octets Transmitted Count Low
    pub gotcl:                          ReadOnly<u32>,          // 0x4090

    /// Good Octets Transmitted Count High
    pub gotch:                          ReadOnly<u32>,          // 0x4094
    _padding12:                         [u8; 424],              // 0x4098 - 0x423F

    /// MAC Core Control 0 Register 
    hlreg0:                             Volatile<u32>,          // 0x4240;
    _padding13:                         [u8; 80],               // 0x4244 - 0x4293

    /// MAC Flow Control Register
    mflcn:                              Volatile<u32>,          // 0x4294;
    _padding13a:                        [u8; 8],                // 0x4298 - 0x429F

    /// Auto-Negotiation Control Register
    autoc:                              Volatile<u32>,          // 0x42A0;

    /// Link Status Register
    pub links:                          ReadOnly<u32>,          // 0x42A4;

    /// Auto-Negotiation Control 2 Register    
    autoc2:                             Volatile<u32>,          // 0x42A8;
    _padding14:                         [u8; 120],              // 0x42AC - 0x4323

    /// Link Status Register 2
    links2:                             ReadOnly<u32>,          // 0x4324
    _padding15:                         [u8; 1496],             // 0x4328 - 0x48FF

    /// DCB Transmit Descriptor Plane Control and Status
    rttdcs:                             Volatile<u32>,          // 0x4900;

    /// DCB Transmit Descriptor Plane Queue Select
    rttdqsel:                          Volatile<u32>,           // 0x4904;  

    /// DCB Transmit Descriptor Plane T1 Config
    rttdt1c:                            Volatile<u32>,          // 0x4908 
    _padding16:                         [u8; 68],               // 0x490C - 0x494F

    txpbthresh:                         [Volatile<u32>; 8],     // 0x4950;
    _padding16a:                        [u8; 272],              // 0x4970 - 0x4A7F

    /// DMA Tx Control
    dmatxctl:                           Volatile<u32>,          // 0x4A80;
    _padding17:                         [u8; 4],                // 0x4A84 - 0x4A87
    
    /// DMA Tx TCP Flags Control Low
    dtxtcpflgl:                         Volatile<u32>,          // 0x4A88;
    
    /// DMA Tx TCP Flags Control High
    dtxtcpflgh:                         Volatile<u32>,          // 0x4A8C;
    _padding18:                         [u8; 1392],             // 0x4A90 - 0x4FFF

    /// Receive Checksum Control
    rxcsum:                             Volatile<u32>,          // 0x5000
    _padding19:                         [u8; 124],              // 0x5004 - 0x507F

    /// Filter Control Register
    fctrl:                              Volatile<u32>,          // 0x5080;
    _padding20:                         [u8; 164],              // 0x5084 - 0x5127

    /// EType Queue Filter
    etqf:                               [Volatile<u32>;8],      // 0x5128 - 0x5147;
    _padding21:                         [u8; 3768],             // 0x5148 - 0x5FFF
} // 4 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeRegisters2>(), 4 * 4096);

pub enum RxPBSizeReg1_7 {
    Size0KiB = 0,
}

#[derive(Debug, TryFromPrimitive)]
#[repr(u32)]
pub enum RxPBReg {
    R1 = 1,
    R2 = 2,
    R3 = 3,
    R4 = 4,
    R5 = 5,
    R6 = 6,
    R7 = 7,
}

bitflags! {
    pub struct FilterCtrlFlags: u32 {
        const STORE_BAD_PACKETS                 = 1 << 1;
        const MULTICAST_PROMISCUOUS_ENABLE      = 1 << 8;
        const UNICAST_PROMISCUOUS_ENABLE        = 1 << 9;
        const BROADCAST_ACCEPT_MODE             = 1 << 10;
    }
}

// Ensure that we never expose reserved bits 0, [7:2], [31:11] as part of the `FilterCtrlFlags` interface.
const_assert_eq!(FilterCtrlFlags::all().bits() & 0xFFFF_F8FD, 0);

pub struct FCTRLSet(bool);
pub struct RXCTRLDisabled(bool);

/// Enable CRC strip by HW
pub const HLREG0_CRC_STRIP:             u32 = 1 << 1;
/// Enable CRC strip by HW
pub const RDRXCTL_CRC_STRIP:            u32 = 1;
/// These 5 bits have to be cleared by software
pub const RDRXCTL_RSCFRSTSIZE:          u32 = 0x1F << 17;

impl IntelIxgbeRegisters2 {
    // Any function that writes to rdrxcrtl must make sure these bits are set
    const RDRXCTL_BASE_VAL: u32 = (1 << 26) | (1 << 25);

    // pub fn rdrxctl_read(&self) -> u32 {
    //     self.rdrxctl.read()
    // }

    // According to the datasheet, these two registers must match
    // prevents DPDK Bug 22
    pub fn crc_strip(&mut self) {
        let val = self.rdrxctl.read();
        self.rdrxctl.write(val | Self::RDRXCTL_BASE_VAL | RDRXCTL_CRC_STRIP);

        self.hlreg0.write(self.hlreg0.read() | HLREG0_CRC_STRIP);
    }

    /// Sets values of bits opposite to what the HW has set them to
    /// RSCFRSTSIZE [21:17] should be set to 0
    /// RSCACKC [25] should be set to 1
    /// FCOE_WRFIX [26] should be set to 1
    pub fn rdrxctl_set_reserved_bits(&mut self) {
        self.rdrxctl.write((self.rdrxctl.read() | Self::RDRXCTL_BASE_VAL) & !RDRXCTL_RSCFRSTSIZE);
    }

    pub fn rdrxctl_dma_init_done(&self) -> bool {
        const DMAIDONE_BIT: u32 = 1 << 3;
        self.rdrxctl.read() & DMAIDONE_BIT == DMAIDONE_BIT
    }

   
    pub fn rxctrl_rx_enable(&mut self, _fctrl_set: FCTRLSet) {
        let val = self.rxctrl.read();
        self.rxctrl.write(val | RECEIVE_ENABLE); 
    }

    pub fn rxctrl_rx_disable(&mut self) -> RXCTRLDisabled {
        let val = self.rxctrl.read();
        self.rxctrl.write(val & !RECEIVE_ENABLE); 
        RXCTRLDisabled(true)
    }

    pub fn fcrtl_clear(&mut self) {
        for fcrtl in self.fcrtl.iter_mut() {
            fcrtl.write(0);
        }
    } 
    
    // pub fn fcrth_clear(&mut self) {
    //     for fcrth in self.fcrth.iter_mut() {
    //         fcrth.write(0);
    //     }
    // } 

    /// Sets the Receive Threshold High (RTH) bits [18:5] for reg 0
    pub fn fcrth0_set_rth(&mut self, rth: u16) {
        self.fcrth[0].write((rth as u32  & 0x3FFF) << 5);
    }

    pub fn fcrtv_clear(&mut self) {
        self.fcrtv.write(0);
    }

    pub fn fccfg_clear(&mut self) {
        self.fccfg.write(0);
    }

    pub fn fccfg_enable_transmit_flow_control(&mut self) {
        self.fccfg.write(self.fccfg.read() | (1 << 3));
    }

    pub fn rxpbsize_reg1_7_set_buffer_size(&mut self, reg_idx: RxPBReg, size: RxPBSizeReg1_7) {
        self.rxpbsize[reg_idx as usize].write((size as u32) << 10);
    }

    // pub fn hlreg0_crc_en(&mut self) {
    //     self.hlreg0.write(self.hlreg0.read() | HLREG0_TXCRCEN);
    // }

    // pub fn hlreg0_tx_pad_en(&mut self) {
    //     self.hlreg0.write(self.hlreg0.read() | HLREG0_TXPADEN);
    // }

    // Resolves DPDK Bug 21
    pub fn fctrl_write(&mut self, val: FilterCtrlFlags, _rx_disabled: RXCTRLDisabled) -> FCTRLSet {
        self.fctrl.write(val.bits());
        FCTRLSet(true)
    }

    pub fn rttdcs_set_arbdis(&mut self) {
        self.rttdcs.write(self.rttdcs.read() | RTTDCS_ARBDIS);
    }

    pub fn rttdcs_clear_arbdis(&mut self) {
        self.rttdcs.write(self.rttdcs.read() & !RTTDCS_ARBDIS);
    }

    // pub fn dmatxctl_disable_tx(&mut self) {
    //     self.dmatxctl.write(self.dmatxctl.read() & !TE); 
    // }

    pub fn dmatxctl_enable_tx(&mut self) {
        self.dmatxctl.write(self.dmatxctl.read() | TE); 
    }

    // pub fn rxcsum_enable_rss_writeback(&mut self) {
    //     self.rxcsum.write(RXCSUM_PCSD);
    // }

    pub fn mflcn_enable_receive_flow_control(&mut self) {
        self.mflcn.write(self.mflcn.read() | (1 << 3));
    }

    pub fn rttdqsel_set_queue_id(&mut self, queue: u8) {
        self.rttdqsel.write((queue & 0x7F) as u32);
    }

    pub fn rttdt1c_write(&mut self, credit_refill: u16) {
        self.rttdt1c.write(credit_refill as u32 & 0x3FFF);
    }

    pub fn txpbthresh0_write(&mut self, thresh: u16) {
        self.txpbthresh[0].write(thresh as u32 & 0x3FF);
    }
}

/// The layout in memory of the transmit queue registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub(crate) struct IntelIxgbeTxRegisters {
    /// Set of registers for 128 transmit descriptor queues
    pub tx_regs:                        [RegistersTx; 128],     // 0x6000 - 0x7FFF
} // 2 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeTxRegisters>(), 2 * 4096);

/// The layout in memory of the set of registers containing the MAC address of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeMacRegisters {
    _padding1:                          [u8; 256],              // 0x8000 - 0x80FF
    /// DMA Tx TCP Max Allow Size Requests
    dtxmxszrq:                          Volatile<u32>,          // 0X8100
    _padding2:                          [u8; 8444],             // 0x8104 - 0xA1FF
    
    /// Receive Address Low
    pub ral:                            ReadOnly<u32>,          // 0xA200;
    
    /// Receive Address High
    rah:                                ReadOnly<u32>,          // 0xA204;
    _padding3:                          [u8; 10744],            // 0xA208 - 0xCBFF

    /// Transmit Packet Buffer Size
    txpbsize:                           [Volatile<u32>;8],      // 0xCC00
    _padding4:                          [u8; 992],              // 0xCC20 - 0xCFFF
} // 5 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeMacRegisters>(), 5 * 4096);

pub enum TxPBSize {
    Size0KiB = 0,
    /// For DCB and VT disabled, set TXPBSIZE.SIZE to 160KB
    Size160KiB = 0xA0,
}

#[derive(Debug, TryFromPrimitive)]
#[repr(u32)]
pub enum TxPBReg {
    R0 = 0,
    R1 = 1,
    R2 = 2,
    R3 = 3,
    R4 = 4,
    R5 = 5,
    R6 = 6,
    R7 = 7,
}

impl IntelIxgbeMacRegisters {
    pub fn rah(&self) -> u16 {
        self.rah.read() as u16
    }

    pub fn dtxmxszrq_allow_max_byte_requests(&mut self) {
        const DTXMXSZRQ_MAX_BYTES: u32 = 0xFFF;
        self.dtxmxszrq.write(DTXMXSZRQ_MAX_BYTES);
    }

    pub fn txpbsize_write(&mut self, reg_idx: TxPBReg, val: TxPBSize) {
        self.txpbsize[reg_idx as usize].write((val as u32) << 10);
    }
}

/// The layout in memory of the second set of receive queue registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeRxRegisters2 {
    /// Second set of Rx Registers for 64 Rx Queues
    pub rx_regs2:                       [RegistersRx; 64],      // 0xD000 - 0xDFFF, for 64 queues
} // 1 4KiB page

const_assert_eq!(core::mem::size_of::<IntelIxgbeRxRegisters2>(), 4096);

/// The layout in memory of the third set of general registers of the 82599 device.
#[derive(FromBytes)]
#[repr(C)]
pub struct IntelIxgbeRegisters3 {
    /// Source Address Queue Filter
    pub saqf:                           [Volatile<u32>;128],    // 0xE000 - 0xE1FF
    
    /// Destination Address Queue Filter
    pub daqf:                           [Volatile<u32>;128],    // 0xE200 - 0xE3FF
    
    /// Source Destination Port Queue Filter
    pub sdpqf:                          [Volatile<u32>;128],    // 0xE400 - 0xE5FF
    
    /// Five Tuple Queue Filter
    ftqf:                               [Volatile<u32>;128],    // 0xE600 - 0xE7FF
    
    /// L3 L4 Tuples Immediate Interrupt Rx 
    l34timir:                           [Volatile<u32>;128],    // 0xE800 - 0xE9FF

    _padding1:                          [u8; 256],              // 0xEA00 - 0xEAFF

    /// Redirection Table
    reta:                               [Volatile<u32>;32],     // 0xEB00 - 0xEB7F

    /// RSS Random Key Register
    pub rssrk:                          [Volatile<u32>;10],     // 0xEB80 - 0xEBA7
    _padding2:                          [u8; 88],               // 0xEBA8 - 0xEBFF

    /// EType Queue Select
    etqs:                               [Volatile<u32>;8],      // 0xEC00 - 0xEC1F;
    _padding3:                          [u8; 96],               // 0xEC20 - 0xEC7F

    /// Multiple Receive Queues Command Register
    mrqc:                               Volatile<u32>,          // 0xEC80;
    _padding4:                          [u8; 1916],             // 0xEC84 - 0xF3FF

    pub pfuta:                              [Volatile<u32>;128],    // 0xF400 - 00xF5FF
    _padding4a:                         [u8; 2576],             // 0xF600 - 0x1000F

    /// EEPROM/ Flash Control Register
    eec:                                Volatile<u32>,          // 0x10010

    /// EEPROM Read Register
    eerd:                               Volatile<u32>,          // 0x10014;
    _padding5:                          [u8; 296],              // 0x10018 - 0x1013F

    /// Software Semaphore Register
    swsm:                               Volatile<u32>,          // 0x10140
    _padding6a:                         [u8; 4],               // 0x10144 - 0x10147

    fwsm:                               Volatile<u32>,          // 0x10148
    _padding6b:                         [u8; 20],               // 0x10144 - 0x1015F

    /// Software Firmware Synchronization
    sw_fw_sync:                         Volatile<u32>,          // 0x10160 
    _padding7:                          [u8; 3852],             // 0x10164 - 0x1106F

    /// DCA Requester ID Information Register
    dca_id:                             ReadOnly<u32>,          // 0x11070

    /// DCA Control Register
    dca_ctrl:                           Volatile<u32>,          // 0x11074
    _padding8:                          [u8; 61320],            // 0x11078 - 0x1FFFF

} // 18 4KiB page (total NIC mem = 128 KB)

const_assert_eq!(core::mem::size_of::<IntelIxgbeRegisters3>(), 18 * 4096);

pub struct FWValid(bool);

impl IntelIxgbeRegisters3 {
    pub fn fwsm_fw_valid(&self) -> Option<FWValid> {
        if self.fwsm.read().get_bit(15) {
            Some(FWValid(true))
        } else {
            None
        }
    }

    // Returns bit 3:1 if the firmware mode is valid
    // Prevents DPDK bug 26
    pub fn fswm_fw_mode(&self, _valid: FWValid) -> u32 {
        (self.fwsm.read() >> 1) & 0x7
    }

    pub fn eec_auto_read(&self) -> bool {
        self.eec.read().get_bit(EEC_AUTO_RD as u8)
    }
}

// check that the sum of all the register structs is equal to the memory of the ixgbe device (128 KiB).
const_assert_eq!(core::mem::size_of::<IntelIxgbeRegisters1>() + core::mem::size_of::<IntelIxgbeRxRegisters1>() +
    core::mem::size_of::<IntelIxgbeRegisters2>() + core::mem::size_of::<IntelIxgbeTxRegisters>() + 
    core::mem::size_of::<IntelIxgbeMacRegisters>() + core::mem::size_of::<IntelIxgbeRxRegisters2>() +
    core::mem::size_of::<IntelIxgbeRegisters3>(), 0x20000);



// CTRL commands
pub const CTRL_LRST:                    u32 = 1<<3; 
pub const CTRL_RST:                     u32 = 1<<26;

// EEPROM Commands
/// Bit which indicates that auto-read by hardware from EEPROM is done
pub const EEC_AUTO_RD:                  u32 = 9;

// Link Commands
pub const LINKS_SPEED_MASK:             u32 = 0x3 << 28;

/// DCB Arbiters Disable
pub const RTTDCS_ARBDIS:                u32 = 1 << 6;

/// For DCB and VT disabled, set TXPBSIZE.SIZE to 160KB
// pub const TXPBSIZE_160KB:                u32 = 0xA0 << 10;
// /// For DCB and VT disabled, set RXPBSIZE.SIZE to 512KB
// pub const RXPBSIZE_512KB:                u32 = 512; //0x200;
// pub const RXPBSIZE_128KB:                u32 = 128;// 0x00020000; // from ixy.rs

// RCTL commands
// pub const BSIZEPACKET_8K:               u32 = 8;
// pub const BSIZEHEADER_256B:             u32 = 4;
// pub const BSIZEHEADER_0B:               u32 = 0;
// pub const DESCTYPE_LEG:                 u32 = 0;
// pub const DESCTYPE_ADV_1BUFFER:         u32 = 1;
// pub const DESCTYPE_ADV_HS:              u32 = 2;
pub const RX_Q_ENABLE:                  u32 = 1 << 25;
// pub const STORE_BAD_PACKETS:            u32 = 1 << 1;
// pub const MULTICAST_PROMISCUOUS_ENABLE: u32 = 1 << 8;
// pub const UNICAST_PROMISCUOUS_ENABLE:   u32 = 1 << 9;
// pub const BROADCAST_ACCEPT_MODE:        u32 = 1 << 10;
pub const RECEIVE_ENABLE:               u32 = 1;

pub const DCA_RXCTRL_CLEAR_BIT_12:      u32 = 1 << 12;
pub const CTRL_EXT_NO_SNOOP_DIS:        u32 = 1 << 16;

// RSS commands
// pub const RXCSUM_PCSD:                  u32 = 1 << 13; 

// // DCA commands
// pub const RX_DESC_DCA_ENABLE:           u32 = 1 << 5;
// pub const RX_HEADER_DCA_ENABLE:         u32 = 1 << 6;
// pub const RX_PAYLOAD_DCA_ENABLE:        u32 = 1 << 7;
// pub const RX_DESC_R_RELAX_ORDER_EN:     u32 = 1 << 9;
// pub const RX_DATA_W_RELAX_ORDER_EN:     u32 = 1 << 13;
// pub const RX_SP_HEAD_RELAX_ORDER_EN:    u32 = 1 << 15;
// pub const DCA_CPUID_SHIFT:              u32 = 24;
// pub const DCA_CTRL_ENABLE:              u32 = 0;
// pub const DCA_MODE_1:                   u32 = 0 << 1;  
// pub const DCA_MODE_2:                   u32 = 1 << 1;

// // 5-tuple Queue Filter commands
// pub const SPDQF_SOURCE_SHIFT:           u32 = 0;
// pub const SPDQF_DEST_SHIFT:             u32 = 16;
// pub const FTQF_PROTOCOL:                u32 = 3;
// pub const FTQF_PROTOCOL_TCP:            u32 = 0;
// pub const FTQF_PROTOCOL_UDP:            u32 = 1;
// pub const FTQF_PROTOCOL_SCTP:           u32 = 2;
// pub const FTQF_PRIORITY:                u32 = 7;
// pub const FTQF_PRIORITY_SHIFT:          u32 = 2;
// pub const FTQF_SOURCE_ADDRESS_MASK:     u32 = 1 << 25;
// pub const FTQF_DEST_ADDRESS_MASK:       u32 = 1 << 26;
// pub const FTQF_SOURCE_PORT_MASK:        u32 = 1 << 27;
// pub const FTQF_DEST_PORT_MASK:          u32 = 1 << 28;
// pub const FTQF_PROTOCOL_MASK:           u32 = 1 << 29;
// pub const FTQF_POOL_MASK:               u32 = 1 << 30;
// pub const L34TIMIR_LLI_ENABLE:          u32 = 1 << 20;

/// Enable a transmit queue
pub const TX_Q_ENABLE:                  u32 = 1 << 25;
/// Transmit Enable
pub const TE:                           u32  = 1;           

// /// Tx descriptor pre-fetch threshold (value taken from DPDK)
// pub const TXDCTL_PTHRESH:               u32 = 36; 
// /// Tx descriptor host threshold (value taken from DPDK)
// pub const TXDCTL_HTHRESH:               u32 = 8 << 8; 
// /// Tx descriptor write-back threshold (value taken from DPDK)
// pub const TXDCTL_WTHRESH:               u32 = 4 << 16; 

// Interrupt Register Commands 
pub const DISABLE_INTERRUPTS:           u32 = 0x7FFFFFFF; 

const_assert_eq!(core::mem::size_of::<RegistersTx>(), 64);

pub struct TDHSet(bool);
pub struct TDLENSet(bool);

// Tells what the value of the RS bit should be in the 8-bit DCMD field of the transmit descriptor.
// The inner vlaue will be ORed with the remaining flags for the DCMD field
pub struct ReportStatusBit(u8);

impl ReportStatusBit {
    fn one() -> ReportStatusBit {
        ReportStatusBit(1 << 3)
    }

    fn zero() -> ReportStatusBit {
        ReportStatusBit(0 << 3)
    }

    pub fn value(&self) -> u8 {
        self.0
    }
}


impl RegistersTx {

    pub fn txdctl_read(&self) -> u32 {
        self.txdctl.read()
    }

    // the queue can only be enabled after the TDH register is set
    pub fn txdctl_txq_enable(&mut self, _tdh_set: TDHSet) {
        let val = self.txdctl.read();
        self.txdctl.write(val | TX_Q_ENABLE); 
    }

    pub fn txdctl_write_wthresh(&mut self, wthresh: U7) -> ReportStatusBit {
        let val = self.txdctl.read() & !0x7F_0000;
        self.txdctl.write(val | ((wthresh.bits() as u32) << 16));

        if wthresh.bits() > 0 {
            ReportStatusBit::zero()
        } else { // if wthresh is set to zero then we should set the RS bit in the descriptor
            ReportStatusBit::one()
        }
    }

    // Upholds the invariant that hthresh > 0 when pthresh is set
    pub fn txdctl_write_pthresh_hthresh(&mut self, pthresh: U7, hthresh: HThresh) {
        let val = self.txdctl.read() & !0x7F7F;
        self.txdctl.write(val | (pthresh.bits() as u32) | ((hthresh.bits() as u32) << 8));
    }

    /// Assume we used the advanced tx descriptors, otherwise create an enum for descriptor types
    pub fn tdlen_write(&mut self, num_descs: NumDesc) -> TDLENSet{
        self.tdlen.write((num_descs as u32) * 16);
        TDLENSet(true)
    }

    // gate access so that the upper 16 bits are always set to 0
    // Prevents DPDK bug 25
    // The two linear types enforce the order of operations tdlen -> tdh -> txq enable
    pub fn tdh_write(&mut self, val: u16, _tdlen_written: TDLENSet) -> TDHSet {
        self.tdh.write(val as u32);
        TDHSet(true)
    }

    pub fn tdwba_set_and_enable(&mut self, addr: u64) {
        self.tdwbah.write((addr >> 32) as u32);
        self.tdwbal.write(addr as u32 | 0x1);
    }

    // pub fn tdwbal_read(&self) -> u32 {
    //     self.tdwbal.read()
    // }

    pub fn dca_txctrl_disable_relaxed_ordering_head_wb(&mut self){
        const TX_DESC_WBRO_EN_BIT: u32 = 1 << 11;

        let val = self.dca_txctrl.read();
        self.dca_txctrl.write(val & !TX_DESC_WBRO_EN_BIT);
    }
}

const_assert_eq!(core::mem::size_of::<RegistersRx>(), 64);

impl RegistersRx {
    /// Assume we used the advanced rx descriptors, otherwise create an enum for descriptor types
    pub fn rdlen_write(&mut self, num_descs: NumDesc) {
        self.rdlen.write((num_descs as u32) * 16 as u32)
    }

    // // gate access so that the upper 16 bits are always set to 0
    // pub fn rdh_write(&mut self, val: u16) {
    //     self.rdh.write(val as u32);
    // }

    pub fn dca_rxctrl_clear_bit_12(&mut self) {
        let val = self.dca_rxctrl.read();
        self.dca_rxctrl.write(val & !DCA_RXCTRL_CLEAR_BIT_12);
    }
    
    pub fn srrctl_write(&mut self, desc_type: DescType, receive_buffer_size: RxBufferSizeKiB) {
        self.srrctl.write(((desc_type as u32) << 25) | (receive_buffer_size as u32));
    }

    pub fn srrctl_drop_enable(&mut self) {
        const DROP_ENABLE: u32 = 1 << 28;
        
        let val = self.srrctl.read() | DROP_ENABLE;
        self.srrctl.write(val);
    }

    pub fn rxdctl_read(&self) -> u32 {
        self.rxdctl.read()
    }

    pub fn rxdctl_rxq_enable(&mut self) {
        let val = self.rxdctl.read();
        self.rxdctl.write(val | RX_Q_ENABLE); 
    }
}


/// Set of registers associated with one transmit descriptor queue.
#[derive(FromBytes)]
#[repr(C)]
pub(crate) struct RegistersTx {
    /// Transmit Descriptor Base Address Low
    pub tdbal:                          Volatile<u32>,        // 0x6000

    /// Transmit Descriptor Base Address High
    pub tdbah:                          Volatile<u32>,        // 0x6004
    
    /// Transmit Descriptor Length    
    tdlen:                              Volatile<u32>,        // 0x6008

    /// Tx DCA Control Register
    dca_txctrl:                         Volatile<u32>,          // 0x600C

    /// Transmit Descriptor Head
    tdh:                                Volatile<u32>,          // 0x6010
    _padding0:                          [u8; 4],                // 0x6014 - 0x6017

    /// Transmit Descriptor Tail
    tdt:                            Volatile<u32>,          // 0x6018
    _padding1:                          [u8; 12],               // 0x601C - 0x6027

    /// Transmit Descriptor Control
    txdctl:                             Volatile<u32>,          // 0x6028
    _padding2:                          [u8; 12],               // 0x602C - 0x6037

    /// Transmit Descriptor Completion Write Back Address Low
    tdwbal:                             Volatile<u32>,          // 0x6038

    /// Transmit Descriptor Completion Write Back Address High
    tdwbah:                             Volatile<u32>,          // 0x603C
} // 64B

impl RegistersTx {
    // gate access so that the upper 16 bits are always set to 0
    #[inline(always)]
    pub fn tdt_write(&mut self, val: u16) {
        self.tdt.write(val as u32);
    }

    // #[inline(always)]
    // pub fn tdt_read(&mut self) -> u32 {
    //     self.tdt.read()
    // }
}

/// Set of registers associated with one receive descriptor queue.
#[derive(FromBytes)]
#[repr(C)]
pub struct RegistersRx {
    /// Receive Descriptor Base Address Low
    pub rdbal:                          Volatile<u32>,        // 0x1000

    /// Recive Descriptor Base Address High
    pub rdbah:                          Volatile<u32>,        // 0x1004

    /// Recive Descriptor Length
    rdlen:                              Volatile<u32>,        // 0x1008

    /// Rx DCA Control Register
    dca_rxctrl:                         Volatile<u32>,          // 0x100C

    /// Recive Descriptor Head
    rdh:                                Volatile<u32>,          // 0x1010

    /// Split Receive Control Registers
    srrctl:                             Volatile<u32>,          // 0x1014 //specify descriptor type

    /// Receive Descriptor Tail
    rdt:                                Volatile<u32>,          // 0x1018
    _padding1:                          [u8;12],                // 0x101C - 0x1027

    /// Receive Descriptor Control
    rxdctl:                             Volatile<u32>,          // 0x1028
    _padding2:                          [u8;20],                // 0x102C - 0x103F                                            
} // 64B

impl RegistersRx {
    // gate access so that the upper 16 bits are always set to 0
    #[inline(always)]
    pub fn rdt_write(&mut self, val: u16) {
        self.rdt.write(val as u32);
    }

    // #[inline(always)]
    // pub fn rdt_read(&mut self) -> u32 {
    //     self.rdt.read()
    // }
}