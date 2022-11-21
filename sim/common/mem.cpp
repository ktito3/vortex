#include "mem.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <assert.h>
#include "util.h"

using namespace vortex;

RamMemDevice::RamMemDevice(const char *filename, uint32_t wordSize) 
  : wordSize_(wordSize) {
  std::ifstream input(filename);

  if (!input) {
    std::cout << "Error reading file \"" << filename << "\" into RamMemDevice.\n";
    std::abort();
  }

  do {
    contents_.push_back(input.get());
  } while (input);

  while (contents_.size() & (wordSize-1))
    contents_.push_back(0x00);
}

RamMemDevice::RamMemDevice(uint64_t size, uint32_t wordSize)
  : contents_(size) 
  , wordSize_(wordSize)
{}

void RamMemDevice::read(void *data, uint64_t addr, uint64_t size) {
  auto addr_end = addr + size;
  if ((addr & (wordSize_-1))
   || (addr_end & (wordSize_-1)) 
   || (addr_end <= contents_.size())) {
    std::cout << "lookup of 0x" << std::hex << (addr_end-1) << " failed.\n";
    throw BadAddress();
  }  
  
  const uint8_t *s = contents_.data() + addr;
  for (uint8_t *d = (uint8_t*)data, *de = d + size; d != de;) {
    *d++ = *s++;
  }
}

void RamMemDevice::write(const void *data, uint64_t addr, uint64_t size) {
  auto addr_end = addr + size;
  if ((addr & (wordSize_-1))
   || (addr_end & (wordSize_-1)) 
   || (addr_end <= contents_.size())) {
    std::cout << "lookup of 0x" << std::hex << (addr_end-1) << " failed.\n";
    throw BadAddress();
  }

  const uint8_t *s = (const uint8_t*)data;
  for (uint8_t *d = contents_.data() + addr, *de = d + size; d != de;) {
    *d++ = *s++;
  }
}

///////////////////////////////////////////////////////////////////////////////

void RomMemDevice::write(const void* /*data*/, uint64_t /*addr*/, uint64_t /*size*/) {
  std::cout << "attempt to write to ROM.\n";
  std::abort();
}

///////////////////////////////////////////////////////////////////////////////

bool MemoryUnit::ADecoder::lookup(uint64_t a, uint32_t wordSize, mem_accessor_t* ma) {
  uint64_t e = a + (wordSize - 1);
  assert(e >= a);
  for (auto iter = entries_.rbegin(), iterE = entries_.rend(); iter != iterE; ++iter) {
    if (a >= iter->start && e <= iter->end) {
      ma->md   = iter->md;
      ma->addr = a - iter->start;
      return true;
    }
  }
  return false;
}

void MemoryUnit::ADecoder::map(uint64_t a, uint64_t e, MemDevice &m) {
  assert(e >= a);
  entry_t entry{&m, a, e};
  entries_.emplace_back(entry);
}

void MemoryUnit::ADecoder::read(void *data, uint64_t addr, uint64_t size) {
  mem_accessor_t ma;
  if (!this->lookup(addr, size, &ma)) {
    std::cout << "lookup of 0x" << std::hex << addr << " failed.\n";
    throw BadAddress();
  }      
  ma.md->read(data, ma.addr, size);// this is a pAddr, translated in mmu
}

void MemoryUnit::ADecoder::write(const void *data, uint64_t addr, uint64_t size) {
  mem_accessor_t ma;
  if (!this->lookup(addr, size, &ma)) {
    std::cout << "lookup of 0x" << std::hex << addr << " failed.\n";
    throw BadAddress();
  }
  ma.md->write(data, ma.addr, size);// this is a pAddr, translated in mmu
}

///////////////////////////////////////////////////////////////////////////////

MemoryUnit::MemoryUnit(uint64_t pageSize, uint64_t addrBytes, bool disableVm)
  : pageSize_(pageSize)
  , addrBytes_(addrBytes)
  , disableVM_(disableVm) {
  if (!disableVm) {
    // tlb_[0] = TLBEntry(0, 077);
    std::cout << "\nVM enabled."<< std::endl;
  }
  else
  {
    std::cout << "\nVM disabled."<< std::endl;
  }
}

void MemoryUnit::attach(MemDevice &m, uint64_t start, uint64_t end) {
  decoder_.map(start, end, m);
  ram_ = dynamic_cast<RAM*>(&m);
  this->base_pfn = dynamic_cast<RAM*>(&m)->iommu_base_pfn;
}

std::pair<bool, uint64_t> MemoryUnit::tlbLookup(uint64_t vAddr, uint32_t flagMask) {
  auto iter = tlb_.find(vAddr / pageSize_);
  if (iter != tlb_.end()) {
    if (iter->second.flags & flagMask) {
      return std::make_pair(true, iter->second.pfn);
    } else {
      return std::make_pair(false, iter->second.pfn);// throw some error about permissions
    }
  } else {
    return std::make_pair(false, 0);// not found
  }
}

void MemoryUnit::read(void *data, uint64_t addr, uint64_t size, bool sup) {
  uint64_t pAddr;

  std::cout << "\nAddress read : 0x" << std::hex << addr << std::endl;

  if (disableVM_) {
    pAddr = addr;
  } else {
     pAddr = vAddr_to_pAddr(addr);
  }
  return decoder_.read(data, pAddr, size);
}

void MemoryUnit::write(const void *data, uint64_t addr, uint64_t size, bool sup) {
  uint64_t pAddr;

  std::cout << "\nAddress write : 0x" << std::hex << addr << std::endl;

  if (disableVM_) {
    pAddr = addr;
  } else {
    pAddr = vAddr_to_pAddr(addr);
  }
  decoder_.write(data, pAddr, size);
}

void MemoryUnit::tlbAdd(uint64_t vpn, uint64_t pfn, uint32_t flags) {
  tlb_[vpn] = TLBEntry(pfn, flags);
}

void MemoryUnit::tlbRm(uint64_t va) {
  if (tlb_.find(va / pageSize_) != tlb_.end())
    tlb_.erase(tlb_.find(va / pageSize_));
}



///////////////////////////////////////////////////////////////////////////////

RAM::RAM(uint32_t page_size) 
  : size_(0)
  , page_bits_(log2ceil(page_size))
  , last_page_(nullptr)
  , last_page_index_(0) {    
   assert(ispow2(page_size));
}

RAM::~RAM() {
  this->clear();
}

void RAM::clear() {
  for (auto& page : pages_) {
    delete[] page.second;
  }
}

uint64_t RAM::size() const {
  return uint64_t(pages_.size()) << page_bits_;
}

// Edits below!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


//is_pAddr defaults to false. It is set to true only if it is being called
//from the mmu. otherwise it is treated as an untranslated vaddr for whatever
//else is trying to access the ram directly.
uint8_t *RAM::get(uint64_t address) const {
  uint32_t page_size   = 1 << page_bits_;  
  uint32_t page_offset = address & (page_size - 1);
  uint64_t page_index  = address >> page_bits_;

  uint8_t* page;
  if (last_page_ && last_page_index_ == page_index) {
    page = last_page_;
  } else {
    // Allocate new physical page
    // Make memory, add pointer to pages_, add mapping to PTE

    auto it = pages_.find(page_index);
    if (it != pages_.end()) {
      page = it->second;
    } else {
      uint8_t *ptr = new uint8_t[page_size];
      // set uninitialized data to "baadf00d"
      for (uint32_t i = 0; i < page_size; ++i) {
        ptr[i] = (0xbaadf00d >> ((i & 0x3) * 8)) & 0xff;
      }
      pages_.emplace(page_index, ptr);
      page = ptr;
    }
    last_page_ = page;
    last_page_index_ = page_index;

  }
  std::cout << "Get address 0x" << std::hex << address << " => page index " << std::dec << (page_index) << " and page offset 0x" << std::hex << (page_offset) << std::endl;
  
  return page + page_offset;
}

//these only read and write virtual addresses as they are called from outside the mmu
//so they are handled diffirently
// read and writes from within the mmu are called differently
void RAM::read(void *data, uint64_t addr, uint64_t size) {

  uint8_t* d = (uint8_t*)data;
  for (uint64_t i = 0; i < size; i++) {
    d[i] = *this->get(addr + i);
  }
}
void RAM::write(const void *data, uint64_t addr, uint64_t size) {

  const uint8_t* d = (const uint8_t*)data;
  for (uint64_t i = 0; i < size; i++) {
    *this->get(addr + i) = d[i];
  }
}





















/***************** VM Code Changes Start ******************/

// extract [s_idx, e_idx] from addr
uint64_t vortex::bits(uint64_t addr, uint8_t s_idx, uint8_t e_idx)
{
    return (addr >> s_idx) & ((1 << (e_idx - s_idx + 1)) - 1);
}
bool vortex::bit(uint64_t addr, uint8_t idx)
{
    return (addr) & (1 << idx);
}


// Expecting continuous memory allocation
uint32_t RAM::os_get_free_frame() 
{
    uint32_t page_size   = 1 << page_bits_;  
    uint32_t pfn = last_page_index_ + 1;

    auto it = pages_.find(pfn);
    if (it != pages_.end()) {
      // throw BadFrameAllocation();
        return -1;
    } 

    uint8_t *ptr = new uint8_t[page_size];
    // set uninitialized data to "baadf00d"
    for (uint32_t i = 0; i < page_size; ++i) {
      ptr[i] = (0xbaadf00d >> ((i & 0x3) * 8)) & 0xff;
    }
    pages_.emplace(pfn, ptr);
    

    last_page_ = ptr;
    last_page_index_ = pfn;

    return pfn;
}


// Expecting satp_pfn initialized
// specifically for two levels, if supporting more modes can generalize the code
// Creates PTE mapping from vAddr to pfn
void RAM::os_map_frame(uint64_t vAddr, uint32_t pfn, uint32_t base_pfn) 
{
    uint32_t pte_bytes;
    uint32_t pte_size = 4;
    uint64_t vpn_0 = bits(vAddr, 12, 21);
    uint64_t vpn_1 = bits(vAddr, 22, 31);

    
    uint64_t ppn_1 = (base_pfn << 12) + (vpn_1 * pte_size);
    read(&pte_bytes, ppn_1, sizeof(uint32_t));
    if ( bit(pte_bytes, 0) ) //valid
    {
        //if valid, proceed to next level
    }
    else
    {
        //if not valid, write pfn and set rwx = 000 to indicate 
        //this is a pointer to the next level of the page table
        pte_bytes = os_get_free_frame() << 10 | 0b0000000001 ;
        write(&pte_bytes, ppn_1, sizeof(uint32_t));
    }

    uint64_t ppn_0 = ( (pte_bytes >> 10) << 12 ) + (vpn_0 * pte_size);
    read(&pte_bytes, ppn_0, sizeof(uint32_t));
    if ( bit(pte_bytes, 0) ) //valid
    {
        //if valid, throw some error, should have only called allocation
        // if mmu found it was invalid
    }
    else
    {
        //if not valid, write pfn and set rwx = 111 to indicate 
        //this is a leaf page table (TODO: set more specific permission bits)
        pte_bytes = pfn << 10 | 0b0000001111 ;
        write(&pte_bytes, ppn_0, sizeof(uint32_t));
    }
}

//set valid bit to 0
//  => every 4 byte (one pte) set the first bit to 0
void RAM::os_init_page_table(uint32_t pfn)
{  
    uint32_t page_size   = 1 << page_bits_;  
    uint8_t *ptr;
    auto it = pages_.find(pfn);
    if (it != pages_.end()) {
        ptr = it->second;
        for (uint32_t i = 0; i < page_size; i=i+4) 
        {
            ptr[i] = 0x00;
        }
    }
}

//Allocate top level page table
uint32_t RAM::os_allocate_top_level_page_table()
{
    uint32_t base_pfn = os_get_free_frame();
    os_init_page_table(base_pfn);
    return base_pfn;
}

void RAM::set_base_pfn(uint32_t base_pfn)
{
    this->iommu_base_pfn = base_pfn;
}


//Handle Page Fault
uint32_t RAM::os_handle_page_fault(uint64_t vAddr, uint64_t base_pfn)
{
    uint32_t pfn = os_get_free_frame();
    os_map_frame(vAddr, pfn, base_pfn);
}

//Volume II: RISC-V Privileged Architectures V1.12-draft -> Page 77
std::pair<bool, uint64_t> RAM::iommu_page_table_walk(uint64_t vAddr_bits, uint64_t base_pfn)
{
    uint64_t pte_size = 4; uint64_t levels = 2; VA_MODE mode = VA_MODE::SV32;
    vAddress_t vAddr(vAddr_bits, SV32);

    uint64_t a = base_pfn << page_bits_;//pageSize_ must be defined according to satp mode. check for it?
    uint32_t pte_bytes;//pageSize_ must be defined according to satp mode. check for it?

    
    for(int i = levels - 1; i >= 0 ; i--)
    {
        this->read(&pte_bytes, a+vAddr.vpn[i]*pte_size, sizeof(uint32_t));
        if (/*not valid*/ !bit(pte_bytes,0))//TODO: HAVE TO ADD EXECPTIONS AND OTHER CHECKS
        {
            return std::make_pair(true, this->os_handle_page_fault(vAddr_bits, base_pfn)); 
        }
        a = (pte_bytes >> 10 ) << page_bits_;
    }

    uint64_t pfn = a >> 12;
    return std::make_pair(true, pfn);
}
//"IOMMU" handling dma, simplistic, can improve, same code as mmu, except no tlb
void RAM::dma_read(void *data, uint64_t vAddr, uint64_t size) {

  uint8_t* d = (uint8_t*)data;
  for (uint64_t i = 0; i < size; i++) {
  
    //todo: change to a single translation
    vAddr += i;
    std::pair<bool, uint64_t> pfn = iommu_page_table_walk(vAddr, this->iommu_base_pfn);
    uint64_t pAddr = pfn.second << page_bits_ + (vAddr & ((1 << page_bits_) - 1));


    d[i] = *this->get(pAddr);
  }
}
void RAM::dma_write(const void *data, uint64_t vAddr, uint64_t size) {

  const uint8_t* d = (const uint8_t*)data;
  for (uint64_t i = 0; i < size; i++) {

    //todo: change to a single translation
    vAddr += i;
    std::pair<bool, uint64_t> pfn = iommu_page_table_walk(vAddr, this->iommu_base_pfn);
    uint64_t pAddr = pfn.second << page_bits_ + (vAddr & ((1 << page_bits_) - 1));


    *this->get(pAddr) = d[i];
  }
}


//these only read and write virtual addresses as they are called from outside the mmu
//so they are handled diffirently
// read and writes from within the mmu are called differently
// void RAM::read_from_mmu(void *data, uint64_t addr, uint64_t size) {
//         std::cout << "Read  from mmu" << std::endl;

//   uint8_t* d = (uint8_t*)data;
//   for (uint64_t i = 0; i < size; i++) {
//     d[i] = *this->get(addr + i, true);//pAddr = true
//   }
// }
// void RAM::write_from_mmu(const void *data, uint64_t addr, uint64_t size) {
//         std::cout << "Write not from mmu" << std::endl;

//   const uint8_t* d = (const uint8_t*)data;
//   for (uint64_t i = 0; i < size; i++) {
//     *this->get(addr + i, true) = d[i];//pAddr = true
//   }
// }







//Translate virtual to physical. uint64_t or uint32_t????? For RV32, PTE are 4 bytes.
uint64_t MemoryUnit::vAddr_to_pAddr(uint64_t vAddr)
{
    std::pair<bool, uint64_t> pfn = tlbLookup(vAddr, 077);
    if (!pfn.first)
    {
        pfn = page_table_walk(vAddr, this->base_pfn);
        if (pfn.first)
        {
            tlbAdd(vAddr>>12, pfn.second, 077);//todo:permissions
        }
        //pfn still false throw some error
    }
    return pfn.second * pageSize_ + (vAddr & (pageSize_ - 1));
}

//Volume II: RISC-V Privileged Architectures V1.12-draft -> Page 77
std::pair<bool, uint64_t> MemoryUnit::page_table_walk(uint64_t vAddr_bits, uint64_t base_pfn)
{
    uint64_t pte_size = 4; uint64_t levels = 2; VA_MODE mode = VA_MODE::SV32;
    vAddress_t vAddr(vAddr_bits, SV32);

    uint64_t a = base_pfn * pageSize_;//pageSize_ must be defined according to satp mode. check for it?
    uint32_t pte_bytes;//pageSize_ must be defined according to satp mode. check for it?

    
    for(int i = levels - 1; i >= 0 ; i--)
    {
        this->ram_->read(&pte_bytes, a+vAddr.vpn[i]*pte_size, sizeof(uint32_t));
        if (/*not valid*/ !bit(pte_bytes,0))//TODO: HAVE TO ADD EXECPTIONS AND OTHER CHECKS
        {
            return std::make_pair(true, dynamic_cast<RAM*>(ram_)->os_handle_page_fault(vAddr_bits, base_pfn)); 
        }
        a = (pte_bytes >> 10 ) * pageSize_;
    }

    uint64_t pfn = a >> 12;
    return std::make_pair(true, pfn);
}

/***************** VM Code Changes End ******************/











void RAM::loadBinImage(const char* filename, uint64_t destination) {
  std::ifstream ifs(filename);
  if (!ifs) {
    std::cout << "error: " << filename << " not found" << std::endl;
  }

  ifs.seekg(0, ifs.end);
  size_t size = ifs.tellg();
  std::vector<uint8_t> content(size);
  ifs.seekg(0, ifs.beg);
  ifs.read((char*)content.data(), size);

  this->clear();
  this->write(content.data(), destination, size);

}

void RAM::loadHexImage(const char* filename) {
  auto hti = [&](char c)->uint32_t {
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    return c - '0';
  };

  auto hToI = [&](const char *c, uint32_t size)->uint32_t {
    uint32_t value = 0;
    for (uint32_t i = 0; i < size; i++) {
      value += hti(c[i]) << ((size - i - 1) * 4);
    }
    return value;
  };

  std::ifstream ifs(filename);
  if (!ifs) {
    std::cout << "error: " << filename << " not found" << std::endl;
  }

  ifs.seekg(0, ifs.end);
  size_t size = ifs.tellg();
  std::vector<char> content(size);
  ifs.seekg(0, ifs.beg);
  ifs.read(content.data(), size);

  uint32_t offset = 0;
  char *line = content.data();

  this->clear();

  while (true) {
    if (line[0] == ':') {
      uint32_t byteCount = hToI(line + 1, 2);
      uint32_t nextAddr = hToI(line + 3, 4) + offset;
      uint32_t key = hToI(line + 7, 2);
      switch (key) {
      case 0:
        for (uint32_t i = 0; i < byteCount; i++) {
          uint32_t addr  = nextAddr + i;
          uint32_t value = hToI(line + 9 + i * 2, 2);
          *this->get(addr) = value;
        }
        break;
      case 2:
        offset = hToI(line + 9, 4) << 4;
        break;
      case 4:
        offset = hToI(line + 9, 4) << 16;
        break;
      default:
        break;
      }
    }
    while (*line != '\n' && size != 0) {
      ++line;
      --size;
    }
    if (size <= 1)
      break;
    ++line;
    --size;
  }
  
}

// ///////////////////////////////////////////////////////////////////////////////



// class PageTableEntry{

//     protected:

//         bool bit(uint8_t idx)
//         {
//             return (entry_bits >> idx) & 1;
//         }

//         uint64_t bit(uint8_t s_idx, e_idx)
//         {
//             return (entry_bits >> s_idx) & ((1 << e_idx) - 1);
//         }

//         uint64_t entry_bits;
//         bool V;
//         bool R;
//         bool W;
//         bool X;
//         bool U;// needed for this?
//         bool G;// needed for this?
//         bool A;
//         bool D;
//         uint64_t PPN;

    
//     public:

//         PageTableEntry(uint32_t bits) : entry_bits(bits)
//         {
//             V = bit(0);
//             R = bit(1);
//             W = bit(2);
//             X = bit(3);
//             U = bit(4);
//             G = bit(5);
//             A = bit(6);
//             D = bit(7);
//         }
// };

// // Is this this the only mode for a 32 bit system??
// // TODO: change inheriatance type
// class PageTableEntry_SV32 : public PageTableEntry{

//     private:


//     public:

//         PageTableEntry_SV32(uint32_t bits) : PageTableEntry(bits)
//         {
//             PPN = bit(12,34);//??????? 34 bit addresses in 32 bit RV??
//         }
// };

// // Are these supported in a 32 bit system?? Spec says the following are for RV64
// // First get 32 bit working

// class PageTableEntry_SV39 : public PageTableEntry{

//     private:

//         uint64_t PPN_0;
//         uint64_t PPN_1;
//         uint64_t PPN_2;

//     public:

//         PageTableEntry_SV39()
//         {
//             PPN_0 = bit(10,18);
//             PPN_1 = bit(19,27);
//             PPN_2 = bit(28,53);
//         }
// };


// class PageTableEntry_SV48 : public PageTableEntry{

//     private:

//         uint64_t PPN_0;
//         uint64_t PPN_1;
//         uint64_t PPN_2;
//         uint64_t PPN_3;

//     public:

//         PageTableEntry_SV48()
//         {
//             PPN_0 = bit(10,18);
//             PPN_1 = bit(19,27);
//             PPN_2 = bit(28,36);
//             PPN_3 = bit(37,53);
//         }
// };

// class PageTableEntry_SV57 : public PageTableEntry{

//     private:

//         uint64_t PPN_0;
//         uint64_t PPN_1;
//         uint64_t PPN_2;
//         uint64_t PPN_3;
//         uint64_t PPN_4;

//     public:

//         PageTableEntry_SV57()
//         {
//             PPN_0 = bit(10,18);
//             PPN_1 = bit(19,27);
//             PPN_2 = bit(28,36);
//             PPN_3 = bit(37,45);
//             PPN_4 = bit(46,53);
//         }
// };



// ///////////////////////////////////////////////////////////////////////////////
// class Address {

//     protected:

//         static const int LEVELS;

//         uint64_t bit(uint8_t s_idx, e_idx)
//         {
//             return (entry_bits >> s_idx) & ((1 << e_idx) - 1);
//         }

//         uint64_t entry_bits;
//         uint64_t pgoff;

    
//     public:

//         Address(uint64_t bits) : entry_bits(bits)
//         {
//             pgoff = bit(0,11);
//         }
// };

// Is this this the only mode for a 32 bit system??
// TODO: change inheriatance type


// class pAddress_SV32 : public Address{

//     private:

//         uint64_t PPN[LEVELS];

//     public:

//         pAddress_SV32(uint64_t bits) : Address(bits), LEVELS(2)//define somewhere else, has to be constexpr
//         {

//             PPN[0] = bit(12,21);
//             PPN[1] = bit(22,31);
//         }
// };

// //todo: for other modes
