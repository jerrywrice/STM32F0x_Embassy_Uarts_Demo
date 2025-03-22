#![no_std]
#![no_main]

use core::fmt::Write;
use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;

use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};

use defmt::info;
use embassy_stm32::exti::{AnyChannel, Channel, ExtiInput};
use embassy_stm32::gpio::{AnyPin, Level, Output, Pin, Pull, Speed};
use embassy_time::{Duration, Timer};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};
use {defmt_rtt as _, panic_probe as _};

static BLINK_FAST_ACTIVE_STATE: AtomicBool = AtomicBool::new(false); // false = slow blink rate, true = fast blink rate

const UART1_BAUD_RATE: u32 = 115200;
const UART3_BAUD_RATE: u32 = 115200;
const LED_SLOW_BLINK_RATE_MS: u32 = 1000u32; // 1000 ms
const LED_FAST_BLINK_RATE_MS: u32 = 250u32; // 250 ms

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    USART3_8 => usart::InterruptHandler<peripherals::USART3>;
});

#[embassy_executor::task]
async fn led_task(led: AnyPin) {
    // Configure the LED pin as a push pull output and obtain handler.
    // On the Nucleo F091RC there's an on-board LED connected to pin PA5.
    let mut led = Output::new(led, Level::Low, Speed::Low);
    let mut prev_blink_fast_active_state: bool = BLINK_FAST_ACTIVE_STATE.load(Ordering::Relaxed);
    let blink_fast_active_state: bool = prev_blink_fast_active_state;
    match blink_fast_active_state {
        false => {
            info!("BLINK_SLOW");
            prev_blink_fast_active_state = blink_fast_active_state;
        }
        _ => {
            info!("BLINK_FAST");
            prev_blink_fast_active_state = blink_fast_active_state;
        }
    }
    '_outer_loop: loop {
        // Read blink rate state setting each time through loop, to catch button press changes to blink rate setting (fast or slow)
        let blink_fast_active_state = BLINK_FAST_ACTIVE_STATE.load(Ordering::Relaxed); // 0 == not paused, 1 == paused
        if blink_fast_active_state != prev_blink_fast_active_state {
            match blink_fast_active_state {
                false => {
                    info!("BLINK SLOW");
                    prev_blink_fast_active_state = blink_fast_active_state;
                }
                _ => {
                    info!("BLINK FAST");
                    prev_blink_fast_active_state = blink_fast_active_state;
                }
            }
        }
        match blink_fast_active_state {
            false => {
                // slow blink mode
                let blink_delay = LED_SLOW_BLINK_RATE_MS; //BLINK_MS.load(Ordering::Relaxed);
                Timer::after_millis(blink_delay.into()).await;
                led.toggle();
            }
            _ => {
                // fast blink mode
                // Blink fast mode active
                // Blink fast (45 x 250 ms LED on/off)
                for _i in 0..45 {
                    let dyn_blink_fast_state = BLINK_FAST_ACTIVE_STATE.load(Ordering::Relaxed); // 0 == not paused, 1 == paused
                    if dyn_blink_fast_state != blink_fast_active_state {
                        // Mode changes, so abort this sequence and start next
                        prev_blink_fast_active_state = dyn_blink_fast_state;
                        continue '_outer_loop;
                    }
                    led.toggle();
                    Timer::after_millis(50).await; // Why 50 ms rather than 250 ms?
                }
                //
                // Blink slow 8 times (8 x 250 ms on/off)
                for _i in 0..8 {
                    // Bail out if the blink rate has changed to slow mode
                    let dyn_blink_fast_state = BLINK_FAST_ACTIVE_STATE.load(Ordering::Relaxed); // 0 == not paused, 1 == paused
                    if dyn_blink_fast_state != blink_fast_active_state {
                        prev_blink_fast_active_state = dyn_blink_fast_state;
                        continue '_outer_loop;
                    }
                    Timer::after_millis(LED_FAST_BLINK_RATE_MS as u64).await; // or LED_Slow_Blink_Rate_Ms???
                    led.toggle();
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn button_monitor_task(pin_pc13: AnyPin, chan_exti13: AnyChannel) {
    let mut button = ExtiInput::new(pin_pc13, chan_exti13, Pull::None);
    loop {
        // Check if button got pressed, and released
        // Uses embassy system timer to measure elapsed time button was held
        // Toggle blink rate (slow rate to/from fast rate) if button held for at least one second

        button.wait_for_low().await;
        let start_button_press_time = embassy_time::Instant::now();

        button.wait_for_high().await;
        let completion_button_release_time = embassy_time::Instant::now();
        let is_blink_rate_fast = BLINK_FAST_ACTIVE_STATE.load(Ordering::Relaxed);
        let elapsed = completion_button_release_time - start_button_press_time;
        if elapsed >= Duration::from_millis(1000) {
            // if button held at least 1 second, then toggle blink rate mode
            //let blink_fast_mask = RESET_BUTTON_BLINK_FAST_DETECTED_MONITOR_TASK_STATE.load(Ordering::Relaxed);
            match is_blink_rate_fast {
                false => {
                    BLINK_FAST_ACTIVE_STATE.store(true, Ordering::Relaxed);
                    info!("Button pressed > 1 second, blink fast activated.");
                }
                _ => {
                    BLINK_FAST_ACTIVE_STATE.store(false, Ordering::Relaxed);
                    info!("Button pressed > 1 second, blink slow activated.");
                }
            }
        }
    }
}

const RAM_START: *mut u8 = 0x2000_0000 as *mut u8; // May need revision, based on hardware configuration
#[embassy_executor::task]
async fn cmd_menu_task(mut cmd_menu_uart: Uart<'static, Async>) {
    const BAUD_RATE: u32 = UART3_BAUD_RATE;
    cmd_menu_uart.set_baudrate(BAUD_RATE).unwrap();

    let mut b1: [u8; 1] = [0u8; 1];
    'outer: loop {
        let mut s1: String<160> = String::new();
        let wr_stat = core::write!(&mut s1, "Type 1 for 'Stack hi-water address scan and dump', 2 for 'Get Stack hi-water address+size', 3 for 'TBD cmd' - followed by RETURN (or ENTER): <= ").unwrap();
        //let wr_stat = core::write!(&mut s1, "Type 1 for 'Scan', 2 for '2', 3 for '3' - followed by RETURN (or ENTER): <= ").unwrap();
        let _stat = cmd_menu_uart.write(s1.as_bytes()).await;
        match _stat {
            Err(_e) => {
                info!("Error on UART3 [1] write()!");
            }
            _ => {}
        }

        //let _pair = embassy_futures::join::join(_fut_1, _fut_3).await;
        loop {
            let mut s2: String<64> = String::new();
            b1[0] = 0;
            let _stat = cmd_menu_uart.read(&mut b1).await;
            match _stat {
                Err(_e) => {
                    info!("Error on UART3 [A] read()!");
                }
                _ => {}
            }

            match b1[0]  {
                b'1' => {
                    core::write!(
                        &mut s2,
                        "1\r\nExecuting 'Stack hi-water address scan and dump' cmd\r\n"
                    ).unwrap();
                    // const SCAN_LEN: u32 = 6;
                    let mut stack_address = 0x7FFFu32;
                    let mut _stack_val: u8 = 0;
                    let mut start_ccs_flag = false;
                    let mut ccs_region_byte_count = 0i32;
                    let mut cs_entry_address  = 0u32;
                    let mut ccs_exit_address : u32;
                    {
                        // let mut conseq_ccs_cnt = 0;
                        for _index in 0..32768isize {
                            if stack_address <= 0 {
                                {
                                    let mut stk_val_hex_img: String<2> = String::new();
                                    core::write!(&mut stk_val_hex_img, "\r\n").unwrap(); // Final line terminator
                                    let _stat = cmd_menu_uart.write(stk_val_hex_img.as_bytes()).await;
                                    match _stat {
                                        Err(_e) => {
                                            info!("Error on UART3 [E] write()!");
                                        }
                                        _ => {}
                                    }
                                }
                                break; // Don't go out of valid memory region ...
                            }
                            unsafe { // 'unsafe' (but sound) memory access
                                _stack_val = ptr::read_volatile(
                                    RAM_START.offset(stack_address as isize) as *const u8,
                                );
                            } // Direct 'unsafe' (but sound) memory access
                            if _stack_val == 0xcc {
                                if start_ccs_flag == false {
                                    start_ccs_flag = true;
                                    cs_entry_address = stack_address;
                                } 
                                // in a ccs region, update count
                                ccs_region_byte_count += 1;
                            } else {
                                if start_ccs_flag {
                                    let this_regions_ccs_cnt = ccs_region_byte_count;
                                    ccs_region_byte_count = 0;
                                    start_ccs_flag = false;
                                    ccs_exit_address = stack_address + 1;
                                    let mut stk_val_hex_img: String<200> = String::new();

                                    core::write!(
                                        &mut stk_val_hex_img,
                                        "cc pattern stack upper address 0x2000{:x}, lower address  0x2000{:x}, length {}\r\n",cs_entry_address,ccs_exit_address,this_regions_ccs_cnt
                                    ).unwrap();
                                    let _stat = cmd_menu_uart.write(stk_val_hex_img.as_bytes()).await;
                                    match _stat {
                                        Err(_e) => {
                                            info!("Error on UART3 [E] write()!");
                                        }
                                        _ => {}
                                    }    
                                }        
                            }
                            stack_address -= 1;
                        }
                    }
                }

                b'2' => {
                    
                    core::write!(
                        &mut s2,
                        "2\r\nExecuting 'Get Stack hi-water address+size' cmd\r\n"
                    ).unwrap();
                    // const SCAN_LEN: u32 = 6;
                    const STACK_HIGH_ADDRESS : u32 = 0x7FFFu32; // 32K ram for STM32F091RC - this value may change on other STM32 MCU configurations.
                    let mut _stack_top_begin_offset_from_top_ram_address: u32 = 0;

                    {
                        let mut scan_state = 0;
                        for stack_address_offset in 0u32..STACK_HIGH_ADDRESS {
                            let mut _stack_val: u8 = 0;
                            unsafe { // 'unsafe' (but sound) memory access
                                _stack_val = ptr::read_volatile(
                                    RAM_START.offset((STACK_HIGH_ADDRESS-stack_address_offset) as isize) as *const u8,
                                );
                            } // Direct 'unsafe' (but sound) memory access
                            if _stack_val == 0xcc {
                                if scan_state == 0 {
                                    _stack_top_begin_offset_from_top_ram_address = STACK_HIGH_ADDRESS - stack_address_offset;
                                    //let start_of_ram_address = RAM_START;
                                    let _stack_bytes_remaining_from_stack_limit  = STACK_HIGH_ADDRESS-stack_address_offset;
                                    let mut stk_val_hex_img: String<150> = String::new();
                                    core::write!(
                                        &mut stk_val_hex_img,
                                        "\r\nStack top address (highest): 0x2000{:x}\r\n",
                                        _stack_top_begin_offset_from_top_ram_address,
                                    ).unwrap();
                                    let _stat = cmd_menu_uart.write(stk_val_hex_img.as_bytes()).await;
                                    match _stat {
                                        Err(_e) => {
                                            info!("Error on UART3 [F] write()!");
                                        }
                                    _ => {}
                                    }
                                    scan_state = 1;
                                }
                                else if scan_state == 1 {
                                   ()
                                }
                            } else if scan_state == 1 {
                                let _stack_high_water_offset_from_stack_max_address = STACK_HIGH_ADDRESS - stack_address_offset;
                                //let start_of_ram_address = RAM_START;
                                let stack_bytes_remaining_from_stack_limit  = STACK_HIGH_ADDRESS-stack_address_offset;
                                let mut stk_val_hex_img: String<150> = String::new();
                                core::write!(
                                    &mut stk_val_hex_img,
                                    "\r\n*** Stack high-water point address: 0x2000{:x}, available stack-space beyond high-water point: {}",
                                    _stack_high_water_offset_from_stack_max_address,
                                    stack_bytes_remaining_from_stack_limit
                                ).unwrap();
                                let _stat = cmd_menu_uart.write(stk_val_hex_img.as_bytes()).await;
                                match _stat {
                                    Err(_e) => {
                                        info!("Error on UART3 [G] write()!");
                                    }
                                _ => {}
                                }
                                break; // done with command
                            }
                            
                        }
                        {
                            let mut stk_val_hex_img: String<2> = String::new();
                            core::write!(&mut stk_val_hex_img, "\r\n").unwrap(); // Final line terminator
                            let _stat = cmd_menu_uart.write(stk_val_hex_img.as_bytes()).await;
                            match _stat {
                                Err(_e) => {
                                    info!("Error on UART3 [E] write()!");
                                }
                                _ => {}
                            }
                        }
                    }
                }
    
                b'3' => {
                    core::write!(&mut s2, "3\r\nCommand 3 executing ... \r\n").unwrap();
                    let _stat = cmd_menu_uart.write(s2.as_bytes()).await;
                    match _stat {
                        Err(_e) => {
                            info!("Error on UART3 [C] write()!");
                        }
                        _ => {}
                    }
                }
                _ => {
                    core::write!(
                        &mut s2,
                        "\r\nInvalid command byte '{:#x}' submitted! Try again!!!\r\n",
                        b1[0]
                    )
                    .unwrap();
                    let _stat = cmd_menu_uart.write(s2.as_bytes()).await;
                    match _stat {
                        Err(_e) => {
                            info!("Error on UART3 [F] write()!");
                        }
                        _ => {}
                    }
                }
            }
            continue 'outer;
        }
    }
}

#[embassy_executor::task]
async fn uart_continous_write_task(mut text_write_uart: Uart<'static, Async>) {
    const BAUD_RATE: u32 = UART1_BAUD_RATE;
    const LINES_TO_TRANSMIT_FOR_REACHING_THRESHOLD: u32 = 10000u32;
    const BYTES_TRANSMITTED_THRESHOLD_BEFORE_FINISHING: u32 = 1000000u32;
    text_write_uart.set_baudrate(BAUD_RATE).unwrap();
    let mut response_input: [u8; 1] = [0u8; 1];

    'outer_prompt_loop: loop {
        loop {
            let mut total_bytes_transmitted = 0u32;
            let start_transmit_time = embassy_time::Instant::now();
            for n in 0u32..=LINES_TO_TRANSMIT_FOR_REACHING_THRESHOLD {
                let mut s2: String<256> = String::new();
                core::write!(&mut s2,"[{}] Writing text lines of > 140 ASCII bytes through UART1 via DMA with baud-rate {}. Lines written in loop with no inter-line delays inserted.\r\n",n,BAUD_RATE).unwrap();
                total_bytes_transmitted += s2.len() as u32;
                let r = text_write_uart.write(s2.as_bytes()).await;
                match r {
                    Err(_e) => {
                        info!("Error on UART1 [A] write()!")
                    }
                    _ => {}
                }
                if total_bytes_transmitted >= BYTES_TRANSMITTED_THRESHOLD_BEFORE_FINISHING {
                    break;
                }
            }

            let completion_transmit_time = embassy_time::Instant::now();
            let elapsed = completion_transmit_time - start_transmit_time;
            let total_time_in_whole_seconds = elapsed.as_millis() / 1000;
            let fractional_time_in_millis = elapsed.as_millis() % 1000;

            {
                let mut s2: String<150> = String::new();
                core::write!(&mut s2,"\n\n\n*** Completed tranfer of {} bytes over UART1 at baud-rate {} in {}.{} seconds.\n\n\n\r",total_bytes_transmitted,BAUD_RATE, total_time_in_whole_seconds,fractional_time_in_millis).unwrap();
                let r = text_write_uart.write(s2.as_bytes()).await;
                match r {
                    Err(_e) => {
                        info!("Error on uart1 [B] write()!")
                    }
                    _ => {}
                }
            }

            {
                let mut s2: String<128> = String::new();
                core::write!(
                    &mut s2,
                    "Do you wish to transmit {} bytes+ over UART1 again? [Y/N] ",
                    BYTES_TRANSMITTED_THRESHOLD_BEFORE_FINISHING
                )
                .unwrap();
                let r = text_write_uart.write(s2.as_bytes()).await;
                match r {
                    Err(_e) => {
                        info!("Error on UART1 [C] write()!")
                    }
                    _ => {}
                }
            }

            response_input[0] = 0;
            let _stat = text_write_uart.read(&mut response_input).await;
            match _stat {
                Err(_e) => {
                    info!("Error on UART1 [A] read()!")
                }
                _ => {
                    let mut cr_str: String<1> = String::new();
                    core::write!(&mut cr_str, "\r").unwrap();
                    let r = text_write_uart.write(cr_str.as_bytes()).await;
                    match r {
                        Err(_e) => {
                            info!("Error on uart1 [D] write()!")
                        }
                        _ => {}
                    }
                    if response_input[0] == b'n' || response_input[0] == b'N' {
                        let mut s2: String<150> = String::new();
                        core::write!(&mut s2,"\n\n\n*** Terminating this async task per user keyboard response. ***\n").unwrap();
                        let r = text_write_uart.write(s2.as_bytes()).await;
                        match r {
                            Err(_e) => {
                                info!("Error on uart1 [E] write()!")
                            }
                            _ => {}
                        }
                        break 'outer_prompt_loop;
                    }
                    continue 'outer_prompt_loop;
                }
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize and create handle for devicer peripherals
    let p = embassy_stm32::init(Default::default());

    let mut config_uart1 = Config::default();
    config_uart1.baudrate = UART1_BAUD_RATE;
    let mut config_uart3 = Config::default();
    config_uart3.baudrate = UART3_BAUD_RATE; // previously used 500000 where writes worked fine, but reads didn't

    // The following two uarts are configured to use DMA with interrupts, thus using uart.write(), uart.read() rather than uart.blocking_write(), uart.blocking_read()
    let uart_1 = Uart::new(
        p.USART1,
        p.PA10,
        p.PA9,
        Irqs,
        p.DMA1_CH2,
        p.DMA1_CH1,
        config_uart1,
    )
    .unwrap();
    // You may also use PB11 for USART3 TX and PB10 for USART3 RX
    let uart_3 = Uart::new(
        p.USART3,
        p.PB11,
        p.PB10,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH3,
        config_uart3,
    )
    .unwrap();

    BLINK_FAST_ACTIVE_STATE.store(false, Ordering::Relaxed); // false == slow blink rate, true == fast blink rate

    // Spawn reset monitoring task since initial edge seen
    spawner
        .spawn(button_monitor_task(p.PC13.degrade(), p.EXTI13.degrade()))
        .unwrap();

    // Spawn LED blinking task
    spawner.spawn(led_task(p.PA5.degrade())).unwrap();

    // Spawn UART3 command menu task
    spawner.spawn(cmd_menu_task(uart_3)).unwrap();
    
    
    // Spawn UART1 continous message print
    spawner.spawn(uart_continous_write_task(uart_1)).unwrap();

    let mut previous_blink_fast_state = true;
    loop {
        // Check if blink rate is slow or fast,
        let current_fast_blink_state = BLINK_FAST_ACTIVE_STATE.load(Ordering::Relaxed);
        if current_fast_blink_state != previous_blink_fast_state {
            match current_fast_blink_state {
                // Only come here and report if previous and current values change from the last update
                false => {
                    info!("BLINK SLOW");
                }
                true => {
                    info!("BLINK FAST");
                }
            }
            previous_blink_fast_state = current_fast_blink_state;
        }
        Timer::after_millis(225).await;
    }
}
