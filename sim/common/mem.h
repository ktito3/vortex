#pragma once

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace vortex {
// extract [s_idx, e_idx] from addr
uint64_t bits(uint64_t addr, uint8_t s_idx, uint8_t e_idx);
bool bit(uint64_t addr, uint8_t idx);

struct BadAddress {};

class MemDevice {
public:
  virtual ~MemDevice() {}
  virtual uint64_t size() const = 0;
  virtual void read(void *data, uint64_t addr, uint64_t size) = 0;
  virtual void write(const void *data, uint64_t addr, uint64_t size) = 0;
};

///////////////////////////////////////////////////////////////////////////////

class RamMemDevice : public MemDevice {
public:
  RamMemDevice(uint64_t size, uint32_t wordSize);
  RamMemDevice(const char *filename, uint32_t wordSize);
  ~RamMemDevice() {}

  void read(void *data, uint64_t addr, uint64_t size) override;  
  void write(const void *data, uint64_t addr, uint64_t size) override;

  virtual uint64_t size() const {
    return contents_.size();
  };

protected:
  std::vector<uint8_t> contents_;
  uint32_t wordSize_;
};

///////////////////////////////////////////////////////////////////////////////

class RomMemDevice : public RamMemDevice {
public:
  RomMemDevice(const char *filename, uint32_t wordSize)
    : RamMemDevice(filename, wordSize) 
  {}

  RomMemDevice(uint64_t size, uint32_t wordSize)
    : RamMemDevice(size, wordSize) 
  {}
  
  ~RomMemDevice();

  void write(const void *data, uint64_t addr, uint64_t size) override;
};

///////////////////////////////////////////////////////////////////////////////

class MemoryUnit {
public:
  
  struct PageFault {
    PageFault(uint64_t a, bool nf)
      : faultAddr(a)
      , notFound(nf) 
    {}
    uint64_t faultAddr;
    bool notFound;
  };

  MemoryUnit(uint64_t pageSize, uint64_t addrBytes, bool disableVm = false);

  void attach(MemDevice &m, uint64_t start, uint64_t end);

  void read(void *data, uint64_t addr, uint64_t size, bool sup);  
  void write(const void *data, uint64_t addr, uint64_t size, bool sup);

  void tlbAdd(uint64_t virt, uint64_t phys, uint32_t flags);
  void tlbRm(uint64_t va);
  void tlbFlush() {
    tlb_.clear();
  }
private:

  class ADecoder {
  public:
    ADecoder() {}
    
    void read(void *data, uint64_t addr, uint64_t size);
    void write(const void *data, uint64_t addr, uint64_t size);
    
    void map(uint64_t start, uint64_t end, MemDevice &md);

  private:

    struct mem_accessor_t {
      MemDevice* md;
      uint64_t addr;
    };
    
    struct entry_t {
      MemDevice *md;
      uint64_t      start;
      uint64_t      end;        
    };

    bool lookup(uint64_t a, uint32_t wordSize, mem_accessor_t*);

    std::vector<entry_t> entries_;
  };

  struct TLBEntry {
    TLBEntry() {}
    TLBEntry(uint32_t pfn, uint32_t flags)
      : pfn(pfn)
      , flags(flags) 
    {}
    uint32_t pfn;
    uint32_t flags;
  };

  std::pair<bool, uint64_t> tlbLookup(uint64_t vAddr, uint32_t flagMask);
  uint64_t vAddr_to_pAddr(uint64_t vAddr);
  std::pair<bool, uint64_t> page_table_walk(uint64_t vAddr_bits, uint64_t base_pfn);

  std::unordered_map<uint64_t, TLBEntry> tlb_;
  uint64_t pageSize_;
  uint64_t addrBytes_;
  ADecoder decoder_;  
  bool disableVM_;
  uint32_t base_pfn;
  MemDevice *ram_;
};

///////////////////////////////////////////////////////////////////////////////

class RAM : public MemDevice {
public:
  uint32_t os_get_free_frame();
  void os_map_frame(uint64_t vAddr, uint32_t pfn, uint32_t base_pfn);
  void os_init_page_table(uint32_t pfn);
  uint32_t os_allocate_top_level_page_table();
  void set_base_pfn(uint32_t base_pfn);
  uint32_t os_handle_page_fault(uint64_t vAddr, uint64_t base_pfn);
std::pair<bool, uint64_t> iommu_page_table_walk(uint64_t vAddr_bits, uint64_t base_pfn);

  RAM(uint32_t page_size);
  ~RAM();

  void clear();

  uint64_t size() const override;

  void read(void *data, uint64_t addr, uint64_t size) override;  
  void write(const void *data, uint64_t addr, uint64_t size) override;
  


  uint8_t *os_get_free_frame() const;
  uint8_t *os_init_page_table() const;
  uint8_t *os_setup_first_level() const;

  void dma_read(void *data, uint64_t vAddr, uint64_t size);  
  void dma_write(const void *data, uint64_t vAddr, uint64_t size);



  void loadBinImage(const char* filename, uint64_t destination);
  void loadHexImage(const char* filename);

  uint8_t& operator[](uint64_t address) {
    return *this->get(address);
  }

  const uint8_t& operator[](uint64_t address) const {
    return *this->get(address);
  }
  uint32_t iommu_base_pfn;

private:

  uint8_t *get(uint64_t address) const;

  uint64_t size_;
  uint32_t page_bits_;  
  mutable std::unordered_map<uint64_t, uint8_t*> pages_;
  mutable uint8_t* last_page_;
  mutable uint64_t last_page_index_;

  // struct IOMMU_t {
  //   IOMMU() {}
  //   IOMMU(uint32_t ptbr) : ptbr(ptbr) {}
  //   uint32_t ptbr;
  // };

  // IOMMU_t IOMMU;

};

enum VA_MODE
{
  BARE,
  SV32
};
class vAddress_t {

    private:
        uint64_t address;
       

    public:
 uint64_t vpn[2];
        VA_MODE mode;uint64_t pgoff;
        vAddress_t(uint64_t address, VA_MODE mode) : address(address), mode(mode)

        {

            vpn[0] = bits(address,12,21);
            vpn[1] = bits(address,22,31);pgoff = bits(address,0,11);
        }
};
} // namespace vortex