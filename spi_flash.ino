#include <SPI.h>

//COMMANDS
#define W_EN 	0x06	//write enable
#define W_DE	0x04	//write disable
#define R_SR1	0x05	//read status reg 1
#define R_SR2	0x35	//read status reg 2
#define W_SR	0x01	//write status reg
#define PAGE_PGM	0x02	//page program
#define QPAGE_PGM	0x32	//quad input page program
#define BLK_E_64K	0xD8	//block erase 64KB
#define BLK_E_32K	0x52	//block erase 32KB
#define SECTOR_E	0x20	//sector erase 4KB
#define CHIP_ERASE	0xc7	//chip erase
#define CHIP_ERASE2	0x60	//=CHIP_ERASE
#define E_SUSPEND	0x75	//erase suspend
#define E_RESUME	0x7a	//erase resume
#define PDWN		0xb9	//power down
#define HIGH_PERF_M	0xa3	//high performance mode
#define CONT_R_RST	0xff	//continuous read mode reset
#define RELEASE		0xab	//release power down or HPM/Dev ID (deprecated)
#define R_MANUF_ID	0x90	//read Manufacturer and Dev ID (deprecated)
#define R_UNIQUE_ID	0x4b	//read unique ID (suggested)
#define R_JEDEC_ID	0x9f	//read JEDEC ID = Manuf+ID (suggested)
#define READ		0x03
#define FAST_READ	0x0b

#define SR1_BUSY_MASK	0x01
#define SR1_WEN_MASK	0x02

#define WINBOND_MANUF	0xef

#define DEFAULT_TIMEOUT 200

#define select() digitalWrite(SS, LOW)
#define deselect() digitalWrite(SS, HIGH)

//#define SPI_DEBUG
#ifdef SPI_DEBUG
#define DBG Serial.print
#define DBGLN Serial.println
#else
#define DBG
#define DBGLN
#endif

typedef struct {
  uint16_t id;
  uint16_t pages;
  uint16_t sectors;
  uint16_t blocks;
  uint32_t bytes;
}
pnListType;

static const pnListType pnList[] = {
  { /* for W25Q16 */
    0x4015,  8192,  512,  32, 2097152   }
  ,
  { /* for W25Q64 */
    0x4017, 32768, 2048, 128, 8388608   }
};

void spi_init()
{
  pinMode(MISO,INPUT_PULLUP);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE0);
  deselect();
}

// must be called after spi_init
bool sf_begin(uint8_t chip)
{
  uint8_t manuf;
  uint16_t id;

  if (chip > 1)
    return false;

  // read the ID of the spi flash chip
  select();
  SPI.transfer(RELEASE);
  deselect();
  delayMicroseconds(5);

  select();
  SPI.transfer(R_JEDEC_ID);
  manuf = SPI.transfer(0x00);
  id = SPI.transfer(0x00) << 8;
  id |= SPI.transfer(0x00);
  deselect();

  DBG("MANUF=0x");
  DBG(manuf,HEX);
  DBG(",ID=0x");
  DBG(id,HEX);
  DBGLN(" ");

  if (manuf != WINBOND_MANUF)
    return false;

  DBGLN("MANUF OK");

  if (id != pnList[chip].id)
    return false;

  DBGLN("DEVICE OK");

  return true;
}

void sf_end()
{
  select();
  SPI.transfer(PDWN);
  deselect();
  delayMicroseconds(5);
}

void spi_deinit()
{
  SPI.end();
}

bool sf_busy()
{
  uint8_t r1;
  select();
  SPI.transfer(R_SR1);
  r1 = SPI.transfer(0xff);
  deselect();
  if (r1 & SR1_BUSY_MASK)
    return true;
  return false;
}

bool sf_wait(uint16_t tmo)
{
  uint32_t start = millis();
  while (sf_busy() && ((millis() - start) < tmo));
  return !sf_busy();
}

void sf_we(bool cmd)
{
  select();
  SPI.transfer(cmd ? W_EN : W_DE);
  deselect();
}

#if 1
uint16_t sf_read(uint32_t addr, uint8_t *buf, uint16_t n)
{
  if (sf_busy())
    return 0;

  select();
  SPI.transfer(READ);
  SPI.transfer(addr>>16);
  SPI.transfer(addr>>8);
  SPI.transfer(addr);
  for (uint16_t i = 0; i < n; i++)
    buf[i] = SPI.transfer(0x00);
  deselect();

  return n;
}
#else
uint16_t sf_read(uint32_t addr, uint8_t *buf, uint16_t n)
{
  if (sf_busy())
    return 0;

  select();
  SPI.transfer(FAST_READ);
  SPI.transfer(addr>>16);
  SPI.transfer(addr>>8);
  SPI.transfer(addr);
  SPI.transfer(0);
  for (uint16_t i = 0; i < n; i++)
    buf[i] = SPI.transfer(0x00);
  deselect();

  return n;
}
#endif

void sf_writepg(uint32_t addr_start, uint8_t *buf)
{
  uint8_t i = 0;
  //DBGLN(addr_start);
  select();
  SPI.transfer(PAGE_PGM);
  SPI.transfer(addr_start>>16);
  SPI.transfer(addr_start>>8);
  SPI.transfer(0x00);
  do {
    SPI.transfer(buf[i]);
    //DBG(buf[i], HEX);
    i++;
  } while(i != 0);
  //DBGLN(" ");
  deselect();
}

void sf_writepg_n(uint32_t addr_start, uint8_t *buf, uint8_t sz)
{
  uint8_t i = 0;
  select();
  SPI.transfer(PAGE_PGM);
  SPI.transfer(addr_start>>16);
  SPI.transfer(addr_start>>8);
  SPI.transfer(0x00);
  for (i = 0; i < sz; i++)
      SPI.transfer(buf[i]);
  deselect();
}

void sf_erasesct(uint32_t addr_start)
{
  select();
  SPI.transfer(SECTOR_E);
  SPI.transfer(addr_start>>16);
  SPI.transfer(addr_start>>8);
  SPI.transfer(addr_start);
  deselect();
}

void sf_erasechp(void)
{
  select();
  SPI.transfer(CHIP_ERASE);
  deselect();
}

