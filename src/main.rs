use std::cell::RefCell;
use std::rc::Rc;
use std::num::ParseIntError;
use std::collections::HashMap;
use std::fs::File;
use std::io::{Read, Seek};
use std::time::Duration;

use jtag_taps::cable;
use jtag_taps::statemachine::JtagSM;
use jtag_taps::taps::Taps;

use jtag_adi::{ArmDebugInterface, MemAP};
use jtag_adi::armv8::{ARMv8, DataSize};

use clap::Parser;

enum Syscall {
    Open = 1,
    Close = 2,
    Read = 6,
    Seek = 10,
    Flen = 12,
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    cable: String,
    #[arg(short, long, default_value_t = 0)]
    baud: u32,
    #[arg(short, long, default_value_t = 0)]
    /// Which JTAG TAP to use
    tap_index: usize,
    #[arg(short, long, default_value_t = 0)]
    /// Which access port to use
    ap_num: u32,
    #[arg(long)]
    cpu_base: String,
    #[arg(long)]
    cti_base: String,
}

fn parse_int(x: &str) -> Result<u32, ParseIntError> {
    if x.starts_with("0x") {
        let len = x.len();
        u32::from_str_radix(&x[2..len], 16)
    } else {
        str::parse(&x)
    }
}

fn main() {
    let args = Args::parse();
    let cable = cable::new_from_string(&args.cable, args.baud).expect("cable");
    let jtag = JtagSM::new(cable);
    let mut taps = Taps::new(jtag);

    taps.detect();

    let adi = Rc::new(RefCell::new(ArmDebugInterface::new(taps)));
    let mem = MemAP::new(adi.clone(), args.ap_num);

    let cpu_base = parse_int(&args.cpu_base).expect("invalid cpu base");
    let cti_base = parse_int(&args.cti_base).expect("invalid cti base");

    let mut v8 = ARMv8::new(mem, cpu_base, cti_base);
    v8.cpu_setup().expect("cpu setup");

    let mut open_files: HashMap<u32, File> = HashMap::new();
    let mut next_fd = 1u32;

    println!("Semihosting server started. Press Ctrl+C to stop.");

    loop {
        // Wait for a debug event (e.g., halt)
        let edprsr = v8.read_cpu(0x314).expect("prsr");
        if edprsr & (1 << 4) == 0 {
            std::thread::sleep(Duration::from_millis(100));
            continue;
        }

        // Read current PC
        let pc = v8.read_special_reg(0x1b452).expect("dlr");

        // Read instruction at PC
        let instr = v8.read_mem(pc, DataSize::Word).expect("read instruction") as u32;

        // Check for HLT #0xF000 (0xD45E0000)
        if instr == 0xD45E0000 {
            println!("Semihosting call detected at PC: 0x{:x}", pc);

            // Read x0 (syscall number)
            let x0 = v8.get_reg(0).expect("read x0") & 0xffffffff;

            if x0 == Syscall::Open as u64 {
                 // x1 is a pointer to the parameter block
                 let x1 = v8.get_reg(1).expect("read x1");
                 let param_block_ptr = x1;

                 // Field 1: Pointer to null-terminated string
                 let path_ptr = v8.read_mem(param_block_ptr, DataSize::Double).expect("read path pointer from block") as u64;
                 
                 // Field 2: Mode (integer)
                 let _mode = v8.read_mem(param_block_ptr + 8, DataSize::Double).expect("read mode from block");

                 // Field 3: Length (integer)
                 let _len = v8.read_mem(param_block_ptr + 16, DataSize::Double).expect("read len from block");
                 
                 // Read string at path_ptr
                 let mut path = Vec::new();
                 let mut offset = 0;
                 loop {
                     let c = v8.read_mem(path_ptr + offset, DataSize::Byte).expect("read path char") as u8;
                     if c == 0 {
                         break;
                     }
                     path.push(c as char);
                     offset += 1;
                     if offset > 256 {
                         break; // Safety limit
                     }
                 }
                 let path_str: String = path.into_iter().collect();
                 println!("SYS_OPEN: path = \"{}\"", path_str);

                 match std::fs::File::open(&path_str) {
                     Ok(file) => {
                         let fd = next_fd;
                         open_files.insert(fd, file);
                         next_fd += 1;
                         println!("SYS_OPEN: Success, fd = {}", fd);
                         v8.set_reg(0, fd as u64).expect("set x0 to fd");
                     }
                     Err(e) => {
                         println!("SYS_OPEN: Error opening file: {}", e);
                         v8.set_reg(0, -1i64 as u64).expect("set x0 to ENOENT");
                     }
                 }
             } else if x0 == Syscall::Read as u64 {
                 // x1 is a pointer to the parameter block
                 let x1 = v8.get_reg(1).expect("read x1");
                 let param_block_ptr = x1;

                 // Field 1: File descriptor
                 let fd = v8.read_mem(param_block_ptr, DataSize::Double).expect("read fd from block") as u32;
                 // Field 2: Buffer pointer
                 let buf_ptr = v8.read_mem(param_block_ptr + 8, DataSize::Double).expect("read buf ptr from block") as u64;
                 // Field 3: Length
                 let len = v8.read_mem(param_block_ptr + 16, DataSize::Double).expect("read len from block") as usize;

                 println!("SYS_READ: fd = {} len = {}", fd, len);

                 if let Some(file) = open_files.get_mut(&fd) {
                     let mut buffer = vec![0u8; len];
                     match file.read(&mut buffer) {
                         Ok(n) => {
                             v8.write_mem_block(buf_ptr, &buffer[0..n]).expect("write mem block");
                             v8.set_reg(0, (len - n) as u64).expect("set x0 to bytes read");
                         }
                         Err(_) => {
                             v8.set_reg(0, -1i64 as u64).expect("set x0 to error");
                         }
                     }
                 } else {
                     v8.set_reg(0, -1i64 as u64).expect("set x0 to EBADF");
                 }
             } else if x0 == Syscall::Close as u64 {
                 let param_block_ptr = v8.get_reg(1).expect("read x1");
                 let fd = v8.read_mem(param_block_ptr, DataSize::Double).expect("read fd from block") as u32;
                 println!("SYS_CLOSE: fd = \"{}\"", fd);

                 if open_files.contains_key(&fd) {
                     open_files.remove(&fd);
                     v8.set_reg(0, 0).expect("set x0 to bytes read");
                 } else {
                     v8.set_reg(0, -1i64 as u64).expect("set x0 to EBADF");
                 }
             } else if x0 == Syscall::Seek as u64 {
                 let param_block_ptr = v8.get_reg(1).expect("read x1");
                 let fd = v8.read_mem(param_block_ptr, DataSize::Double).expect("read fd from block") as u32;
                 let pos = v8.read_mem(param_block_ptr+8, DataSize::Double).expect("read fd from block");
                 println!("SYS_SEEK: fd = {} pos = {}", fd, pos);

                 if let Some(file) = open_files.get_mut(&fd) {
                     file.seek(std::io::SeekFrom::Start(pos)).expect("seek");
                     v8.set_reg(0, 0).expect("set x0 to size");
                 } else {
                     v8.set_reg(0, -1i64 as u64).expect("set x0 to EBADF");
                 }
              } else if x0 == Syscall::Flen as u64 {
                  let param_block_ptr = v8.get_reg(1).expect("read x1");
                  let fd = v8.read_mem(param_block_ptr, DataSize::Double).expect("read fd from block") as u32;
                  println!("SYS_FLEN: fd = \"{}\"", fd);

                  if let Some(file) = open_files.get(&fd) {
                      match file.metadata() {
                          Ok(metadata) => {
                              let size = metadata.len();
                              v8.set_reg(0, size as u64).expect("set x0 to size");
                          }
                          Err(_) => {
                              v8.set_reg(0, -1i64 as u64).expect("set x0 to error");
                          }
                      }
                  } else {
                      v8.set_reg(0, -1i64 as u64).expect("set x0 to EBADF");
                  }
              } else {
                println!("Unsupported syscall: {}", x0);
                v8.set_reg(0, -1i64 as u64).expect("set x0");
            }
        }

        // Resume execution
        v8.write_special_reg(0x1b452, pc+4).expect("dlr");
        v8.cpu_resume().expect("resume");
    }
}
