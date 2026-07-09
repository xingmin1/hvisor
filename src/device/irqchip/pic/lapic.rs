// Copyright (c) 2025 Syswonder
// hvisor is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//     http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND, EITHER
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT, MERCHANTABILITY OR
// FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.
//
// Syswonder Website:
//      https://www.syswonder.org
//
// Authors:
//  Solicey <lzoi_lth@163.com>

use crate::{
    arch::{
        cpu::{this_apic_id, this_cpu_id},
        idt::IdtVector,
        ipi,
        msr::Msr::{self, *},
    },
    cpu_data::this_cpu_data,
    device::irqchip::pic::pop_vector,
    error::HvResult,
};
use bit_field::BitField;
use core::{ops::Range, u32};
use x2apic::lapic::{LocalApic, LocalApicBuilder, TimerDivide, TimerMode};

const GUEST_LAPIC_BASE: u64 = 0xfee0_0000;
const APIC_BASE_BSP: u64 = 1 << 8;
const APIC_BASE_X2APIC_ENABLE: u64 = 1 << 10;
const APIC_BASE_GLOBAL_ENABLE: u64 = 1 << 11;
const SIVR_SOFTWARE_ENABLE: u32 = 1 << 8;
const LVT_MASKED: u32 = 1 << 16;

pub struct VirtLocalApic {
    pub phys_lapic: LocalApic,
    pub virt_timer_vector: u8,
    virt_apic_base_bits: u64,
    virt_lvt_timer_bits: u32,
    virt_sivr_bits: u32,
    virt_timer_initial_count: u32,
    virt_timer_divide_config: u32,
    virt_tsc_deadline: u64,
}

impl VirtLocalApic {
    pub fn new() -> Self {
        Self {
            phys_lapic: Self::new_phys_lapic(
                IdtVector::APIC_TIMER_VECTOR as _,
                IdtVector::APIC_ERROR_VECTOR as _,
                IdtVector::APIC_SPURIOUS_VECTOR as _,
            ),
            virt_timer_vector: IdtVector::APIC_TIMER_VECTOR as _,
            virt_apic_base_bits: GUEST_LAPIC_BASE
                | APIC_BASE_X2APIC_ENABLE
                | APIC_BASE_GLOBAL_ENABLE,
            virt_lvt_timer_bits: LVT_MASKED,
            virt_sivr_bits: 0,
            virt_timer_initial_count: 0,
            virt_timer_divide_config: TimerDivide::Div256 as u32,
            virt_tsc_deadline: 0,
        }
    }

    fn new_phys_lapic(timer: usize, error: usize, spurious: usize) -> LocalApic {
        let mut lapic = LocalApicBuilder::new()
            .timer_vector(timer)
            .error_vector(error)
            .spurious_vector(spurious)
            .build()
            .unwrap();
        unsafe {
            lapic.enable();
            lapic.disable_timer();
        }
        lapic
    }

    fn timer_divide(value: u32) -> TimerDivide {
        match value & 0b1011 {
            0b0000 => TimerDivide::Div2,
            0b0001 => TimerDivide::Div4,
            0b0010 => TimerDivide::Div8,
            0b0011 => TimerDivide::Div16,
            0b1000 => TimerDivide::Div32,
            0b1001 => TimerDivide::Div64,
            0b1010 => TimerDivide::Div128,
            _ => TimerDivide::Div256,
        }
    }

    fn timer_mode(&self) -> TimerMode {
        match self.virt_lvt_timer_bits.get_bits(17..19) {
            0 => TimerMode::OneShot,
            1 => TimerMode::Periodic,
            _ => TimerMode::TscDeadline,
        }
    }

    fn is_tsc_deadline_mode(&self) -> bool {
        self.virt_lvt_timer_bits.get_bits(17..19) >= 2
    }

    fn apic_base_enabled(&self) -> bool {
        self.virt_apic_base_bits & APIC_BASE_GLOBAL_ENABLE != 0
    }

    fn software_enabled(&self) -> bool {
        self.virt_sivr_bits & SIVR_SOFTWARE_ENABLE != 0
    }

    fn timer_masked(&self) -> bool {
        self.virt_lvt_timer_bits & LVT_MASKED != 0
    }

    fn timer_enabled(&self) -> bool {
        self.apic_base_enabled() && self.software_enabled() && !self.timer_masked()
    }

    fn sync_timer_config(&mut self) {
        unsafe {
            self.phys_lapic.set_timer_mode(self.timer_mode());
            self.phys_lapic
                .set_timer_divide(Self::timer_divide(self.virt_timer_divide_config));
        }
    }

    fn sync_timer_gate(&mut self) {
        unsafe {
            if self.timer_enabled() {
                self.phys_lapic.enable_timer();
            } else {
                self.phys_lapic.disable_timer();
            }

            if self.timer_enabled() && self.is_tsc_deadline_mode() {
                IA32_TSC_DEADLINE.write(self.virt_tsc_deadline);
            } else {
                IA32_TSC_DEADLINE.write(0);
            }
        }
    }

    fn reload_timer_initial(&mut self) {
        unsafe {
            self.phys_lapic
                .set_timer_initial(self.virt_timer_initial_count);
        }
        self.sync_timer_gate();
    }

    /// 重置虚拟 LAPIC 和宿主 LAPIC timer 状态，避免跨 zone 残留中断。
    pub fn reset(&mut self) {
        unsafe {
            IA32_TSC_DEADLINE.write(0);
            self.phys_lapic.disable_timer();
            self.phys_lapic.end_of_interrupt();
        }

        *self = Self::new();
        self.sync_timer_config();
        self.sync_timer_gate();
    }

    /// 返回 guest 可见的 LAPIC base MSR 值，并按当前 vCPU 合成 BSP 位。
    pub fn apic_base(&self, is_bsp: bool) -> u64 {
        let bsp = if is_bsp { APIC_BASE_BSP } else { 0 };
        (self.virt_apic_base_bits & !APIC_BASE_BSP) | bsp
    }

    /// 更新 guest LAPIC base 状态，并同步宿主 LAPIC timer 使能状态。
    pub fn write_apic_base(&mut self, value: u64) {
        self.virt_apic_base_bits = GUEST_LAPIC_BASE
            | if value & APIC_BASE_GLOBAL_ENABLE != 0 {
                APIC_BASE_GLOBAL_ENABLE | APIC_BASE_X2APIC_ENABLE
            } else {
                0
            };

        if !self.apic_base_enabled() {
            self.virt_tsc_deadline = 0;
            unsafe { IA32_TSC_DEADLINE.write(0) };
        }

        self.sync_timer_gate();
    }

    /// 判断 guest LAPIC 是否处于全局使能且软件使能状态。
    pub fn is_enabled(&self) -> bool {
        self.apic_base_enabled() && self.software_enabled()
    }

    /// 处理宿主 LAPIC timer 到期，并返回应注入给 guest 的 timer 向量。
    pub fn handle_timer_irq(&mut self) -> u8 {
        if self.is_tsc_deadline_mode() {
            self.virt_tsc_deadline = 0;
            unsafe { IA32_TSC_DEADLINE.write(0) };
        }
        self.virt_timer_vector
    }

    pub const fn msr_range() -> Range<u32> {
        0x800..0x840
    }

    pub fn phys_local_apic<'a>() -> &'a mut LocalApic {
        &mut this_cpu_data().arch_cpu.virt_lapic.phys_lapic
    }

    pub fn rdmsr(&mut self, msr: Msr) -> HvResult<u64> {
        match msr {
            IA32_X2APIC_APICID => {
                // info!("apicid: {:x}", this_cpu_id());
                Ok(this_apic_id() as u64)
            }
            IA32_X2APIC_VERSION => Ok(0x0005_0014),
            IA32_X2APIC_LDR => Ok(this_apic_id() as u64), // logical apic id
            IA32_X2APIC_ISR0 | IA32_X2APIC_ISR1 | IA32_X2APIC_ISR2 | IA32_X2APIC_ISR3
            | IA32_X2APIC_ISR4 | IA32_X2APIC_ISR5 | IA32_X2APIC_ISR6 | IA32_X2APIC_ISR7 => {
                // info!("isr!");
                Ok(0)
            }
            IA32_X2APIC_IRR0 | IA32_X2APIC_IRR1 | IA32_X2APIC_IRR2 | IA32_X2APIC_IRR3
            | IA32_X2APIC_IRR4 | IA32_X2APIC_IRR5 | IA32_X2APIC_IRR6 | IA32_X2APIC_IRR7 => {
                // info!("irr!");
                Ok(0)
            }
            IA32_X2APIC_LVT_TIMER => Ok(self.virt_lvt_timer_bits as _),
            IA32_TSC_DEADLINE => Ok(self.virt_tsc_deadline),
            IA32_X2APIC_SIVR => Ok(self.virt_sivr_bits as _),
            IA32_X2APIC_ESR => Ok(0),
            IA32_X2APIC_INIT_COUNT => Ok(self.virt_timer_initial_count as _),
            IA32_X2APIC_CUR_COUNT if self.timer_enabled() => unsafe {
                Ok(self.phys_lapic.timer_current() as _)
            },
            IA32_X2APIC_CUR_COUNT => Ok(0),
            IA32_X2APIC_DIV_CONF => Ok(self.virt_timer_divide_config as _),
            _ => hv_result_err!(ENOSYS),
        }
    }

    pub fn wrmsr(&mut self, msr: Msr, value: u64) -> HvResult {
        match msr {
            IA32_X2APIC_SIVR => {
                self.virt_sivr_bits = value as u32;
                if !self.software_enabled() {
                    self.virt_tsc_deadline = 0;
                    unsafe { IA32_TSC_DEADLINE.write(0) };
                }
                self.sync_timer_gate();
                Ok(())
            }
            IA32_X2APIC_EOI => {
                // info!("eoi");
                pop_vector(this_cpu_id());
                Ok(())
            }
            IA32_X2APIC_ESR => Ok(()),
            IA32_X2APIC_ICR => {
                // info!("ICR value: {:x}", value);
                if self.is_enabled() {
                    let _ = ipi::send_ipi(value);
                }
                Ok(())
            }
            IA32_X2APIC_LVT_TIMER => {
                self.virt_lvt_timer_bits = value as u32;
                self.virt_timer_vector = value.get_bits(0..=7) as u8;
                self.sync_timer_config();
                self.sync_timer_gate();
                Ok(())
            }
            IA32_X2APIC_INIT_COUNT => {
                self.virt_timer_initial_count = value as u32;
                self.reload_timer_initial();
                Ok(())
            }
            IA32_X2APIC_DIV_CONF => {
                self.virt_timer_divide_config = value as u32;
                unsafe {
                    self.phys_lapic
                        .set_timer_divide(Self::timer_divide(self.virt_timer_divide_config));
                }
                Ok(())
            }
            IA32_TSC_DEADLINE => {
                self.virt_tsc_deadline = value;
                if self.timer_enabled() && self.is_tsc_deadline_mode() {
                    unsafe { msr.write(value) };
                } else {
                    unsafe { msr.write(0) };
                }
                Ok(())
            }
            _ => hv_result_err!(ENOSYS),
        }
    }
}
