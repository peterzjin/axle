#include <SoftwareSerial.h>

#define START_BYTE        0xA5
#define MSG_TYPE_MASK     0xF8
#define MSG_TYPE_OFFSET   3
#define MSG_LEN_MASK      0x07
#define MSG_LEN_MAX       6

// HOST : Phone APP
// STB  : Smart Board
// CTL  : Control Board
#define MSG_TYPE_INVALID  0x0
#define MSG_TYPE_ACK      0x1   // ACK to last cmd
#define MSG_TYPE_HOST_HB  0x2   // HOST -> STB : Heartbeat
#define MSG_TYPE_UL_ITL   0x3   // HOST -> STB : set the update interval
#define MSG_TYPE_Q_CTL    0x4   // STB -> CTL : to query the status
#define MSG_TYPE_CTL_STA  0x5   // CTL -> STB -> HOST : the control board status
#define MSG_TYPE_S_GEAR   0x6   // HOST -> STB-> CTL: set the gear
#define MSG_TYPE_TEMP     0x7   // STB -> HOST: send the temperature
#define MSG_TYPE_MAX      0x8

#define GPS_RATE          10

#define SF_CHIP_SIZE     2097152 // 2M
#define SF_GPS_SIZE      524288  // 512K
#define SF_SECT_SIZE     4096    // 4K
#define SF_PAGE_SIZE     256     // 256B
#define SF_GPS_OFFSET    8192    // Offline GPS data start from 8K
#define SF_CONF_OFFSET   0       // System configuration data start from 0K
#define SF_CONF_SIZE     8192    // 8K

#define TS_GPS_SAVE_DELTA  100   // 100ms after last gps data, to save
#define TS_CTL_DATA_DELTA  20    // 20ms after last ctl data, to switch serial to GPS
#define TS_HB_TIMEOUT      20000 // 20s after last hearbeat, consider HOST lost

#define DHT11_PIN 5

SoftwareSerial GPSSerial(6, 7); // RX, TX
SoftwareSerial CtlSerial(8, 9); // RX, TX

unsigned char ser_char;  // serial char from/to BT UART or GPS UART

/* |     start_byte     | msg_type , msg_len |      data     |
   |        1 byte      |   5 bits ,  3 bits | several bytes |*/
// 0: wait for cmd; 1: start byte received; 2: message type and length received.
unsigned char cmd_status;

unsigned char cmd_type, cmd_len, buff_ptr, cmd_buff[MSG_LEN_MAX + 2];

unsigned char sf_buf_idx, sf_buf_ptr, sf_gps_buf[2][SF_PAGE_SIZE], sf_saved_buf;

// start end end address of the offline gps data
unsigned long sf_gps_s, sf_gps_e;

// timestamp of the last gps data, last info update to HOST, last ctl data, last heartbeat
// timestamp now, and the delta timestamp
unsigned long ts_last_gps, ts_last_ul, ts_last_ctl, ts_last_hb, ts_now;

// update interval to HOST
unsigned short ul_interval;

/* debug the SPI FLASH read and write */
//#define DBG_SPI_FLASH
#ifdef DBG_SPI_FLASH
#define DBG_BYTE 0x55
#define DBG_SIZE SF_CHIP_SIZE
unsigned char *dbg_buf = sf_gps_buf[0];
unsigned char *dbg_buf_r = sf_gps_buf[1];
#endif
/* debug the GPS data saved to SPI FLASH */
//#define DBG_GPS_SF
#ifdef DBG_GPS_SF
unsigned char dbg_buf_sf[SF_PAGE_SIZE];
#endif
#define DBG_PC_AS_HOST
//#define DBG_GENERAL

unsigned short sys_sta;
#define SYS_STA_FLASH     0x1
#define SYS_STA_HOST      0x2
#define flash_ok (sys_sta & SYS_STA_FLASH)
#define host_connected (sys_sta & SYS_STA_HOST)

typedef struct _saved_conf
{
  unsigned char magic_byte;
  unsigned long gps_data_s;
  unsigned long gps_data_e;
} Saved_conf;

void gps_set_interval(unsigned char secs)
{
  unsigned char send_buff[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x00,
                                 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};
  unsigned char i, chk_a, chk_b;
  unsigned short msecs = secs * 1000;
  send_buff[6] = (unsigned char)msecs;
  send_buff[7] = (unsigned char)(msecs >> 8);
  chk_a = chk_b = 0;
  for (i = 0; i < 10; i++) {
    chk_a += send_buff[i + 2];
    chk_b += chk_a;
  }
  send_buff[12] = chk_a;
  send_buff[13] = chk_b;
  for (i = 0; i < 14; i++) {
    GPSSerial.write(send_buff[i]);
    Serial.write(send_buff[i]);
  }
}

/*
void gps_set_interval(unsigned char secs)
{
  unsigned char send_buff[16], i, j, chk_a, chk_b;
  send_buff[0] = 0xB5;
  send_buff[1] = 0x62;
  send_buff[2] = 0x06;
  send_buff[3] = 0x01;
  send_buff[4] = 0x08;
  send_buff[5] = 0x00;
  send_buff[6] = 0xF0;
  for (i = 0; i < 6; i++) {
    send_buff[7] = i;
    send_buff[8] = send_buff[9] = send_buff[10] = secs;
    send_buff[11] = send_buff[12] = send_buff[13] = secs;
    chk_a = chk_b = 0;
    for (j = 0; j < 12; j++) {
      chk_a += send_buff[j + 2];
      chk_b += chk_a;
    }
    send_buff[14] = chk_a;
    send_buff[15] = chk_b;
    for (j = 0; j < 16; j++) {
      GPSSerial.write(send_buff[j]);
    }
    delay(50);
  }
}
*/

void gps_init()
{
  gps_set_interval(GPS_RATE);
}

void bt_init()
{

}

void process_cmd()
{
  char i;
  /*
  Serial.write(START_BYTE);
  Serial.write((cmd_type << MSG_TYPE_OFFSET) | cmd_len);
  for (i = 0; i < cmd_len; i++)
    Serial.write(cmd_buff[i]);
  */

  switch (cmd_type) {
    case MSG_TYPE_HOST_HB:
      ts_last_hb = millis();
      sys_sta |= SYS_STA_HOST;
      break;
    default:
      break;
  }
}

void load_conf_from_sf()
{
  if (!flash_ok)
    return;

  // find the valid page of the SPI FLASH.
  unsigned long addr = 0;
  Saved_conf tmp;
  sf_wait(500);
  for (addr = SF_CONF_OFFSET; addr < SF_CONF_OFFSET + SF_CONF_SIZE; addr += SF_PAGE_SIZE) {
    sf_read(addr, (unsigned char *)&tmp, sizeof(Saved_conf));
    if (tmp.magic_byte != START_BYTE)
      break;
  }

  if (addr == SF_CONF_OFFSET)
    return;

  // move back to the last valid page.
  addr -= SF_PAGE_SIZE;
  sf_read(addr, (unsigned char *)&tmp, sizeof(Saved_conf));

  if (tmp.magic_byte == START_BYTE
      && tmp.gps_data_s < (SF_GPS_OFFSET + SF_GPS_SIZE)
      && tmp.gps_data_e < (SF_GPS_OFFSET + SF_GPS_SIZE)
      && tmp.gps_data_s >= SF_GPS_OFFSET
      && tmp.gps_data_s >= SF_GPS_OFFSET) {
    // make sure the value is valid.
    sf_gps_s = tmp.gps_data_s;
    sf_gps_e = tmp.gps_data_e;
  }
}

// save some system configuration to SPI FLASH.
void update_conf_to_sf()
{
  if (!flash_ok)
    return;

  // find the valid page of the SPI FLASH.
  unsigned long addr = 0;
  Saved_conf tmp;
  sf_wait(500);
  for (addr = SF_CONF_OFFSET; addr < SF_CONF_OFFSET + SF_CONF_SIZE; addr += SF_PAGE_SIZE) {
    sf_read(addr, (unsigned char *)&tmp, sizeof(Saved_conf));
    if (tmp.magic_byte != START_BYTE)
      break;
  }

  if (addr >= SF_CONF_OFFSET + SF_CONF_SIZE
      ||!(tmp.magic_byte == 0xFF && tmp.gps_data_s == 0xFFFFFFFF && tmp.gps_data_e == 0xFFFFFFFF)) {
    // case 1: the configration storage is full,
    // case 2: not magic byte, not erased page, something wrong happened,
    // so erase this sector and start over.
    addr = SF_CONF_OFFSET;
    sf_wait(500);
    sf_we(true);
    sf_erasesct(addr);
  }

  // page available to write
  sf_wait(500);
  sf_we(true);
  sf_writepg_n(addr, (unsigned char *)&tmp, sizeof(Saved_conf));
}

void save_to_sf()
{
  unsigned char to_save = !sf_saved_buf;
  unsigned long cur_size;

  // the buffer to save is not full yet!
  if (!flash_ok || to_save == sf_buf_idx)
    return;

  // if we need a new sector, erase it first.
  if (!(sf_gps_e & (SF_SECT_SIZE - 1))) {
    sf_wait(500);
    sf_we(true);
    sf_erasesct(sf_gps_e);
  }

  sf_wait(500);
  sf_we(true);
  sf_writepg(sf_gps_e, sf_gps_buf[to_save]);

#ifdef DBG_GPS_SF
  /*
  for(int i = 0; i < SF_PAGE_SIZE; i++)
    Serial.print(sf_gps_buf[to_save][i], HEX);
  Serial.println("");
  */
  sf_wait(500);
  sf_read(sf_gps_e, dbg_buf_sf, SF_PAGE_SIZE);
  if (memcmp(sf_gps_buf[to_save], dbg_buf_sf))
    Serial.println("ERROR!!!");
  /*
  for(int i = 0; i < SF_PAGE_SIZE; i++)
    Serial.print(dbg_buf[i], HEX);
  Serial.println("");
  */
#endif

  sf_gps_e += SF_PAGE_SIZE;

  // if stored GPS data size bigger than SF_GPS_SIZE,
  // update the sf_gps_s first.
  if (sf_gps_e < sf_gps_s)
    cur_size = sf_gps_e + SF_CHIP_SIZE - SF_GPS_OFFSET - sf_gps_s;
  else
    cur_size = sf_gps_e - sf_gps_s;

  // stored too much data.
  if (cur_size > SF_GPS_SIZE) {
    sf_gps_s += (cur_size - SF_GPS_SIZE);
    // if overlapped
    if (sf_gps_s >= SF_CHIP_SIZE)
      sf_gps_s = SF_GPS_OFFSET;
  }

  // if overlapped
  if (sf_gps_e >= SF_CHIP_SIZE)
    sf_gps_e = SF_GPS_OFFSET;

  sf_saved_buf = to_save;

  update_conf_to_sf();
}

void save_offline_gps(unsigned char data)
{
  sf_gps_buf[sf_buf_idx][sf_buf_ptr++] = data;

  // it is time to swift the buffer.
  if (sf_buf_ptr == 0) {
    sf_buf_idx = !sf_buf_idx;
    // too many date received, last buffer hasn't been saved !!!
    // normally, it should NOT happen!!!
    if (sf_buf_idx != sf_saved_buf) {
      save_to_sf();
    }
  }
}

// port 0 for the HOST
// port 1 for the CTL
void send_msg(unsigned char port, unsigned char msg_type, unsigned char msg_len, unsigned char *msg)
{
  if (port > 1 || msg_type > MSG_TYPE_MAX || msg_len > MSG_LEN_MAX || (msg == NULL && msg_len != 0))
    return;

  unsigned char i;
  if (port) {
    CtlSerial.write(START_BYTE);
    CtlSerial.write((msg_type << MSG_TYPE_OFFSET) | msg_len);
    for (i = 0; i < msg_len; i++)
      CtlSerial.write(msg[i]);
  } else {
    Serial.write(START_BYTE);
    Serial.write((msg_type << MSG_TYPE_OFFSET) | msg_len);
    for (i = 0; i < msg_len; i++)
      Serial.write(msg[i]);
  }
}

unsigned long get_ts_delta(unsigned long ts_old, unsigned long ts_new)
{
  if (ts_new > ts_old)
    return (ts_new - ts_old);
  else
    return ((~ts_old) + ts_new);
}

void setup()
{
  Serial.begin(9600);
  GPSSerial.begin(9600);
  CtlSerial.begin(9600);

  ser_char = cmd_status = cmd_type = cmd_len = buff_ptr = sys_sta = 0;
  sf_buf_ptr = sf_buf_idx = 0;
  sf_gps_s = sf_gps_e = SF_GPS_OFFSET; // start from 8K
  sf_saved_buf = !sf_buf_idx;
  ts_last_gps = ts_last_ul = ts_last_ctl  = ts_last_hb = ts_now = 0;
  ul_interval = 10000; // 10s as default

  dht11_init(DHT11_PIN);
  spi_init();
  bt_init();
  delay(200); // wait for the gps module
  gps_init();

  bool sf = sf_begin(0);
  if (sf)
    sys_sta |= SYS_STA_FLASH;

  load_conf_from_sf();

  //sf_end();

  // Listen on the GPS Serial by default,
  // and will switch to CTL Serial if needed.
  GPSSerial.listen();

#ifdef DBG_PC_AS_HOST
  sys_sta |= SYS_STA_HOST;
#endif

#ifdef DBG_SPI_FLASH

  if (flash_ok)
    Serial.println("SPI FLAHSH OK");
  else
    Serial.println("SPI FLAHSH ERROR");

  unsigned long addr = 0;
  unsigned long start, write_end, read_end;

  for (int i = 0; i < SF_PAGE_SIZE; i++)
    dbg_buf[i] = DBG_BYTE;

  start = millis();
  for (addr = 0; addr < DBG_SIZE; addr += SF_PAGE_SIZE) {
    // if we need a new sector, erase it first.
    if (!(addr & (SF_SECT_SIZE - 1))) {
      sf_wait(500);
      sf_we(true);
      sf_erasesct(addr);
      //Serial.println("ERASE DONE");
    }

    sf_wait(500);
    sf_we(true);
    sf_writepg(addr, dbg_buf);
  }

  write_end = millis();
  Serial.println("WRITE DONE");

  addr = 0;
  sf_wait(500);
  for (addr = 0; addr < DBG_SIZE; addr += SF_PAGE_SIZE) {
    sf_read(addr, dbg_buf_r, SF_PAGE_SIZE);
    if (memcmp(dbg_buf_r, dbg_buf, SF_PAGE_SIZE)) {
      Serial.println(addr, HEX);
      Serial.println(dbg_buf_r[0], HEX);
      Serial.println(dbg_buf_r[1], HEX);
      Serial.println(dbg_buf_r[2], HEX);
      Serial.println(dbg_buf_r[3], HEX);
    }
  }
  read_end = millis();
  Serial.println(start);
  Serial.println(write_end);
  Serial.println(read_end);
  Serial.println("TEST DONE");
  while(1);
#endif

}

void loop()
{
  // send the GPS data to HOST directly if HOST connected
  // or save the GPS data to flash
  while (GPSSerial.available() > 0) {
    ser_char = GPSSerial.read();
    if (host_connected)
      Serial.write(ser_char);
    else
      save_offline_gps(ser_char);

    ts_last_gps = millis();
  }

  // send the CTL data to HOST directly if HOST connected
  // or just discard.
  //CtlSerial.listen();
  while (CtlSerial.available() > 0) {
    ser_char = GPSSerial.read();
    if (host_connected)
      Serial.write(ser_char);

    ts_last_ctl = millis();
  }

  while (Serial.available() > 0) {
    ser_char = Serial.read();
    // if got a START_BYTE, reset the cmd_status to 1.
    if (ser_char == START_BYTE) {
      cmd_status = 1;
      continue;
    }
    switch (cmd_status) {
      case 1:
        cmd_type = (ser_char & MSG_TYPE_MASK) >> MSG_TYPE_OFFSET;
        cmd_len = ser_char & MSG_LEN_MASK;
        if (cmd_type < MSG_TYPE_MAX && cmd_len <= MSG_LEN_MAX) {
          if (cmd_len == 0) {
            cmd_status = 0;
            process_cmd();
          } else {
            buff_ptr = 0;
            cmd_status = 2;
          }
        } else {
          cmd_status = 0;
        }
        break;
      case 2:
        cmd_buff[buff_ptr++] = ser_char;
        if (buff_ptr == cmd_len) {
          cmd_status = 0;
          process_cmd();
        }
        break;
      default:
        cmd_status = 0;
        break;
    }
  }

  if (ts_last_gps) {
    // still have gps data received.
    ts_now = millis();
    if (get_ts_delta(ts_last_gps, ts_now) > TS_GPS_SAVE_DELTA) {
#ifdef DBG_GENERAL
      unsigned long save_start, save_end;
      Serial.println(ts_last_gps);
      Serial.println(ts_now);
      Serial.println(sf_buf_idx);
      Serial.println(sf_buf_ptr);
      save_start = millis();
#endif
      if (!host_connected)
        save_to_sf();
#ifdef DBG_GENERAL
      save_end = millis();
      Serial.println(save_start);
      Serial.println(save_end);
#endif
      ts_last_gps = 0;
    }
  }

  if (!ts_last_gps) {
    // should be safe to do other things here.
    ts_now = millis();
    if (host_connected && (get_ts_delta(ts_last_ul, ts_now) > ul_interval)) {
      // need to send update info to HOST.
      unsigned char temp[5];
      if (dht11_read(temp))
        send_msg(0, MSG_TYPE_TEMP, 5, temp);
      // send Query msg to CTL, CTL needs to respond in TS_CTL_DATA_DELTA ms
      send_msg(1, MSG_TYPE_Q_CTL, 0, NULL);
      CtlSerial.listen();
      ts_last_ul = ts_last_ctl = ts_now;
    }
  }

  if (ts_last_ctl) {
    // still have ctl data received.
    ts_now = millis();
    if (get_ts_delta(ts_last_ctl, ts_now) > TS_CTL_DATA_DELTA) {
      GPSSerial.listen();
      ts_last_ctl = 0;
    }
  }

#ifndef DBG_PC_AS_HOST
  if (host_connected) {
    ts_now = millis();
    if (get_ts_delta(ts_last_hb, ts_now) > TS_HB_TIMEOUT) {
      // No heartbeat from HOST for TS_HB_TIMEOUT seconds, lost HOST
      sys_sta &= ~SYS_STA_HOST;
    }
  }
#endif
}
