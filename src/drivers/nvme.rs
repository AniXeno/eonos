//! Minimal polling NVMe PCI driver, restricted to 512-byte logical blocks.
//! Writes are intentionally not exposed yet.

use alloc::boxed::Box;
use alloc::sync::Arc;
use core::sync::atomic::{fence, Ordering};
use crate::block::{BlockDevice, BlockError};
use crate::fat32::SECTOR_SIZE;
use spin::Mutex;

const CF8:u16=0xcf8; const CFC:u16=0xcfc; const QSIZE:u16=16; const TIMEOUT_MS:u64=5000;
#[derive(Clone,Copy)] struct Loc{bus:u8,dev:u8,fun:u8}
unsafe fn outl(p:u16,v:u32){core::arch::asm!("out dx, eax",in("dx")p,in("eax")v,options(nomem,nostack,preserves_flags));}
unsafe fn outw(p:u16,v:u16){core::arch::asm!("out dx, ax",in("dx")p,in("ax")v,options(nomem,nostack,preserves_flags));}
unsafe fn inl(p:u16)->u32{let v:u32;core::arch::asm!("in eax, dx",in("dx")p,out("eax")v,options(nomem,nostack,preserves_flags));v}
fn addr(l:Loc,o:u8)->u32{(1<<31)|((l.bus as u32)<<16)|((l.dev as u32)<<11)|((l.fun as u32)<<8)|(o as u32&0xfc)}
fn read(l:Loc,o:u8)->u32{unsafe{outl(CF8,addr(l,o));inl(CFC)}}
fn controllers()->alloc::vec::Vec<(Loc,u64)>{
 let mut out=alloc::vec::Vec::new();
 for b in 0..=255u16{for d in 0..32u8{let z=Loc{bus:b as u8,dev:d,fun:0};if read(z,0) as u16==0xffff{continue}
 let count=if read(z,0x0c)&(1<<23)!=0{8}else{1};for f in 0..count{let l=Loc{fun:f,..z};if read(l,0) as u16==0xffff||((read(l,8)>>8)&0x00ff_ffff)!=0x0108_02{continue}
 let bar=read(l,0x10);if bar&1!=0||(bar>>1)&3!=2{continue}let base=(bar as u64&!0xf)|((read(l,0x14) as u64)<<32);if base==0{continue}
 let cmd=(read(l,4) as u16)|6;unsafe{outl(CF8,addr(l,4));outw(CFC,cmd)}out.push((l,base));
 }}}
 out
}
#[repr(C,align(4096))] struct Page([u8;4096]);
struct Dma{phys:u64,ptr:*mut Page} unsafe impl Send for Dma{}
fn dma()->Option<Dma>{let p=crate::pmm::alloc_contiguous(1)?;let v=crate::pmm::phys_to_virt(p) as *mut Page;unsafe{core::ptr::write_bytes(v.cast::<u8>(),0,4096)}Some(Dma{phys:p,ptr:v})}
#[repr(C)]#[derive(Clone,Copy,Default)]struct Entry{opcode:u8,flags:u8,cid:u16,nsid:u32,reserved:u64,mptr:u64,prp1:u64,prp2:u64,cdw:[u32;6]}
#[repr(C)]#[derive(Clone,Copy,Default)]struct Completion{result:u32,reserved:u32,sq_head:u16,sq_id:u16,cid:u16,status:u16}
struct Queue{sq:Dma,cq:Dma,tail:u16,head:u16,phase:u16,doorbell:*mut u32,stride:usize,qid:u16,csts:*mut u32}
unsafe impl Send for Queue{}
impl Queue{
 fn submit(&mut self,e:Entry)->Option<Completion>{
  let sq=unsafe{&mut (*self.sq.ptr).0};let at=self.tail as usize*64;sq[at..at+64].copy_from_slice(unsafe{core::slice::from_raw_parts((&e as *const Entry).cast::<u8>(),64)});
  self.tail=(self.tail+1)%QSIZE;fence(Ordering::SeqCst);unsafe{core::ptr::write_volatile(self.doorbell,self.tail as u32)}
  let start=crate::pit::uptime_ms();loop{let at=self.head as usize*16;let status=unsafe{core::ptr::read_volatile(self.cq.ptr.cast::<u8>().add(at+14).cast::<u16>())};if status&1==self.phase{
    fence(Ordering::Acquire);let c=unsafe{core::ptr::read_unaligned(self.cq.ptr.cast::<u8>().add(at).cast::<Completion>())};self.head=(self.head+1)%QSIZE;if self.head==0{self.phase^=1}unsafe{core::ptr::write_volatile(self.doorbell.add(self.stride/4),self.head as u32)};return if c.status>>1==0{Some(c)}else{crate::log_fail!("NVMe","Command","Queue {} opcode {:#x} failed, completion status {:#06x}",self.qid,e.opcode,c.status);None}
  }if crate::pit::uptime_ms().saturating_sub(start)>TIMEOUT_MS{crate::log_fail!("NVMe","Command","Queue {} opcode {:#x} timed out (CSTS={:#x})",self.qid,e.opcode,unsafe{core::ptr::read_volatile(self.csts)});return None}core::hint::spin_loop()}
 }
}
struct Shared{queue:Mutex<Queue>,data:Dma}
unsafe impl Send for Shared{} unsafe impl Sync for Shared{}
struct NvmeDisk{shared:Arc<Shared>,nsid:u32,capacity:u64}
impl BlockDevice for NvmeDisk{
 fn sector_count(&self)->u64{self.capacity}
 fn read_sector(&self,lba:u64,out:&mut[u8;SECTOR_SIZE])->Result<(),BlockError>{if lba>=self.capacity{return Err(BlockError::OutOfRange)}let mut q=self.shared.queue.lock();let data=&self.shared.data;let e=Entry{opcode:2,nsid:self.nsid,prp1:data.phys,cdw:[lba as u32,(lba>>32)as u32,0,0,0,0],..Entry::default()};q.submit(e).ok_or(BlockError::BadBuffer)?;out.copy_from_slice(unsafe{&(&(*data.ptr).0)[..512]});Ok(())}
}
fn wait_ready(reg:*mut u32,want:bool)->bool{let start=crate::pit::uptime_ms();loop{let v=unsafe{core::ptr::read_volatile(reg)}&1!=0;if v==want{return true}if crate::pit::uptime_ms().saturating_sub(start)>TIMEOUT_MS{return false}core::hint::spin_loop()}}
fn controller(base:u64)->Option<(Arc<Shared>,alloc::vec::Vec<(u32,u64)>)>{
 let mm=crate::vmm::map_mmio(base,0x1000);let cap=unsafe{core::ptr::read_volatile(mm.cast::<u64>())};let mqes=(cap as u16)+1;if mqes<QSIZE{crate::log_fail!("NVMe","Init","CAP.MQES={} is below required queue depth {}",mqes,QSIZE);return None}let dstrd=((cap>>32)&0xf)as usize;let Some(stride)=4usize.checked_shl(dstrd as u32)else{crate::log_fail!("NVMe","Init","Invalid doorbell stride {}",dstrd);return None};
 let _=crate::vmm::map_mmio(base,(0x1000usize.checked_add(4usize.checked_mul(stride)?)?).try_into().ok()?);
 let cc=unsafe{mm.add(0x14).cast::<u32>()};let csts=unsafe{mm.add(0x1c).cast::<u32>()};unsafe{core::ptr::write_volatile(cc,core::ptr::read_volatile(cc)&!1)}if !wait_ready(csts,false){crate::log_fail!("NVMe","Init","Controller did not become disabled (CC={:#x}, CSTS={:#x})",unsafe{core::ptr::read_volatile(cc)},unsafe{core::ptr::read_volatile(csts)});return None}
 let Some(asq)=dma()else{crate::log_fail!("NVMe","Init","No DMA memory for admin submission queue");return None};let Some(acq)=dma()else{crate::log_fail!("NVMe","Init","No DMA memory for admin completion queue");return None};let Some(admin_data)=dma()else{crate::log_fail!("NVMe","Init","No DMA memory for Identify data");return None};unsafe{
 core::ptr::write_volatile(mm.add(0x24).cast::<u32>(),((QSIZE-1)as u32)<<16|((QSIZE-1)as u32));
 core::ptr::write_volatile(mm.add(0x28).cast::<u64>(),asq.phys);core::ptr::write_volatile(mm.add(0x30).cast::<u64>(),acq.phys);
 // CSS=0 selects the NVM command set; IOSQES=6 and IOCQES=4.
 core::ptr::write_volatile(cc,(6<<16)|(4<<20)|1);
 }if !wait_ready(csts,true){crate::log_fail!("NVMe","Init","Controller failed to become ready (CC={:#x}, CSTS={:#x})",unsafe{core::ptr::read_volatile(cc)},unsafe{core::ptr::read_volatile(csts)});return None}
 let db=unsafe{mm.add(0x1000).cast::<u32>()};let mut admin=Queue{sq:asq,cq:acq,tail:0,head:0,phase:1,doorbell:db,stride,qid:0,csts};
 let identify=Entry{opcode:6,prp1:admin_data.phys,cdw:[1,0,0,0,0,0],..Entry::default()};if admin.submit(identify).is_none(){crate::log_fail!("NVMe","Init","Identify Controller command failed");return None}
 let nn=unsafe{core::ptr::read_volatile((&(*admin_data.ptr).0[516])as *const u8 as *const u32)};if nn==0{crate::log_fail!("NVMe","Init","Identify Controller reported zero namespaces");return None}
 // Request one submission queue and one completion queue before creating
 // the I/O queues (the NVMe controller starts with admin queues only).
 let queue_count=Entry{opcode:9,cdw:[7,0,0,0,0,0],..Entry::default()};if admin.submit(queue_count).is_none(){crate::log_fail!("NVMe","Init","Set Features (Number of Queues) failed");return None}
 let Some(io_sq)=dma()else{crate::log_fail!("NVMe","Init","No DMA memory for I/O submission queue");return None};let Some(io_cq)=dma()else{crate::log_fail!("NVMe","Init","No DMA memory for I/O completion queue");return None};let io_sq_phys=io_sq.phys;let io_cq_phys=io_cq.phys;let db1=unsafe{mm.add(0x1000+2*stride).cast::<u32>()};
 // CDW10 encodes QID in bits 15:0 and zero-based QSIZE in bits 31:16.
 let create_cq=Entry{opcode:5,prp1:io_cq_phys,cdw:[(((QSIZE-1)as u32)<<16)|1,1,0,0,0,0],..Entry::default()};if admin.submit(create_cq).is_none(){crate::log_fail!("NVMe","Init","Create I/O Completion Queue failed");return None}
 let create_sq=Entry{opcode:1,prp1:io_sq_phys,cdw:[(((QSIZE-1)as u32)<<16)|1,(1<<16)|1,0,0,0,0],..Entry::default()};if admin.submit(create_sq).is_none(){crate::log_fail!("NVMe","Init","Create I/O Submission Queue failed");return None}
 let q=Queue{sq:io_sq,cq:io_cq,tail:0,head:0,phase:1,doorbell:db1,stride,qid:1,csts};
 // Probe active namespaces and expose only the 512-byte LBA format this
 // block layer currently supports.
 let Some(nsdata)=dma()else{crate::log_fail!("NVMe","Init","No DMA memory for namespace Identify data");return None};let mut namespaces=alloc::vec::Vec::new();
 for nsid in 1..=nn.min(32){let idns=Entry{opcode:6,nsid,prp1:nsdata.phys,cdw:[0,0,0,0,0,0],..Entry::default()};if admin.submit(idns).is_none(){continue}
 let raw=unsafe{&(*nsdata.ptr).0};let capacity=u64::from_le_bytes(raw[0..8].try_into().ok()?);let flbas=raw[26]&0xf;let format=128+flbas as usize*4;
 if capacity>0&&raw[format+2]==9&&u16::from_le_bytes([raw[format],raw[format+1]])==0{namespaces.push((nsid,capacity));}}
 if namespaces.is_empty(){crate::log_fail!("NVMe","Init","No active namespace has 512-byte logical blocks and no metadata");return None}let Some(data)=dma()else{crate::log_fail!("NVMe","Init","No DMA memory for block I/O buffer");return None};let shared=Arc::new(Shared{queue:Mutex::new(q),data});Some((shared,namespaces))
}
pub fn init(){let found=controllers();if found.is_empty(){crate::log_ok!("NVMe","Init","No NVMe PCI controller found");return}let mut id=0usize;for(loc,base)in found{crate::log_ok!("NVMe","Init","Controller at {:02x}:{:02x}.{} BAR0 {:#x}",loc.bus,loc.dev,loc.fun,base);let Some((shared,namespaces))=controller(base)else{crate::log_fail!("NVMe","Init","Controller at {:02x}:{:02x}.{} did not initialize or has no supported namespace",loc.bus,loc.dev,loc.fun);continue};for(nsid,capacity)in namespaces{let disk=NvmeDisk{shared:shared.clone(),nsid,capacity};let disk:&'static dyn BlockDevice=Box::leak(Box::new(disk));let name=alloc::format!("nvme{}n{}",id,nsid);let name:&'static str=Box::leak(name.into_boxed_str());crate::block::register_device(name,disk);crate::log_ok!("NVMe","Disk","{} discovered ({} sectors)",name,disk.sector_count());}id+=1;}}
