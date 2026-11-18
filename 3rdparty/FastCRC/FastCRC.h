



// Set this to 0 for smaller 32BIT-CRC-Tables:
#if !defined(CRC_BIGTABLES)
#define CRC_BIGTABLES 1
#endif

#if !defined(FastCRC_h)
#define FastCRC_h

#if defined(ARDUINO)
#include <Arduino.h>
#endif

#include <inttypes.h>
#include <cstddef>


// ================= DEFINES ===================
#if defined(KINETISK)
#define CRC_SW 0
#define CRC_FLAG_NOREFLECT         (((1<<31) | (1<<30)) | ((0<<29) | (0<<28))) //refin=false refout=false
#define CRC_FLAG_REFLECT           (((1<<31) | (0<<30)) | ((1<<29) | (0<<28))) //Reflect in- and outgoing bytes (refin=true refout=true)
#define CRC_FLAG_XOR               (1<<26)                                     //Perform XOR on result
#define CRC_FLAG_NOREFLECT_8       (0)                                         //For 8-Bit CRC
#define CRC_FLAG_REFLECT_SWAP      (((1<<31) | (0<<30)) | ((0<<29) | (1<<28))) //For 16-Bit CRC (byteswap)
#else
#define CRC_SW 1
#endif

// ================= 7-BIT CRC ===================
class FastCRC7
{
public:
  FastCRC7();
  uint8_t crc7(const uint8_t *data, const size_t datalen);		// (MultiMediaCard interface)
  uint8_t crc7_upd(const uint8_t *data, const size_t datalen);	// Call for subsequent calculations with previous seed.
#if !CRC_SW
  uint8_t generic(const uint8_t polyom, const uint8_t seed, const uint32_t flags, const uint8_t *data, const size_t datalen); //Not available in non-hw-variant (not T3.x)
#endif
private:
#if CRC_SW
  uint8_t seed;
#else
  uint8_t update(const uint8_t *data, const size_t datalen);
#endif
};

// ================= 8-BIT CRC ===================

class FastCRC8
{
public:
  FastCRC8();
  uint8_t smbus(const uint8_t *data, const size_t datalen);		// Alias CRC-8
  uint8_t maxim(const uint8_t *data, const size_t datalen);		// Equivalent to _crc_ibutton_update() in crc16.h from avr_libc

  uint8_t smbus_upd(const uint8_t *data, size_t datalen);			// Call for subsequent calculations with previous seed.
  uint8_t maxim_upd(const uint8_t *data, size_t datalen);			// Call for subsequent calculations with previous seed.
#if !CRC_SW
  uint8_t generic(const uint8_t polyom, const uint8_t seed, const uint32_t flags, const uint8_t *data, const size_t datalen); //Not available in non-hw-variant (not T3.x)
#endif
private:
#if CRC_SW
  uint8_t seed;
#else
  uint8_t update(const uint8_t *data, const size_t datalen);
#endif
};

// ================= 14-BIT CRC ===================

class FastCRC14
{
public:
#if !CRC_SW //NO Software-implemenation so far
  FastCRC14();
  uint16_t darc(const uint8_t *data, const size_t datalen);
  uint16_t gsm(const uint8_t *data, const size_t datalen);
  uint16_t eloran(const uint8_t *data, const size_t datalen);
  uint16_t ft4(const uint8_t *data, const size_t datalen);

  uint16_t darc_upd(const uint8_t *data, size_t len);
  uint16_t gsm_upd(const uint8_t *data, size_t len);
  uint16_t eloran_upd(const uint8_t *data, size_t len);
   uint16_t ft4_upd(const uint8_t *data, size_t len);
#endif
#if !CRC_SW
  uint16_t generic(const uint16_t polyom, const uint16_t seed, const uint32_t flags, const uint8_t *data, const size_t datalen); //Not available in non-hw-variant (not T3.x)
#endif
private:
#if CRC_SW
  uint16_t seed;
#else
  uint16_t update(const uint8_t *data, const size_t datalen);
#endif
};

// ================= 16-BIT CRC ===================

class FastCRC16
{
public:
  FastCRC16();
  uint16_t ccitt(const uint8_t *data, const size_t datalen);      // Alias "false CCITT"
  uint16_t mcrf4xx(const uint8_t *data,const size_t datalen);     // Equivalent to _crc_ccitt_update() in crc16.h from avr_libc
  uint16_t kermit(const uint8_t *data, const size_t datalen);     // Alias CRC-16/CCITT, CRC-16/CCITT-TRUE, CRC-CCITT
  uint16_t modbus(const uint8_t *data, const size_t datalen);     // Equivalent to _crc_16_update() in crc16.h from avr_libc
  uint16_t xmodem(const uint8_t *data, const size_t datalen);     // Alias ZMODEM, CRC-16/ACORN
  uint16_t x25(const uint8_t *data, const size_t datalen);        // Alias CRC-16/IBM-SDLC, CRC-16/ISO-HDLC, CRC-B

  uint16_t ccitt_upd(const uint8_t *data, size_t len);			// Call for subsequent calculations with previous seed
  uint16_t mcrf4xx_upd(const uint8_t *data, size_t len);			// Call for subsequent calculations with previous seed
  uint16_t kermit_upd(const uint8_t *data, size_t len);			// Call for subsequent calculations with previous seed
  uint16_t modbus_upd(const uint8_t *data, size_t len);			// Call for subsequent calculations with previous seed
  uint16_t xmodem_upd(const uint8_t *data, size_t len);			// Call for subsequent calculations with previous seed
  uint16_t x25_upd(const uint8_t *data, size_t len);				// Call for subsequent calculations with previous seed
#if !CRC_SW
  uint16_t generic(const uint16_t polyom, const uint16_t seed, const uint32_t flags, const uint8_t *data, const size_t datalen); //Not available in non-hw-variant (not T3.x)
#endif
private:
#if CRC_SW
  uint16_t seed;
#else
  uint16_t update(const uint8_t *data, const size_t datalen);
#endif
};

// ================= 32-BIT CRC ===================

class FastCRC32
{
public:
  FastCRC32();
  uint32_t crc32(const uint8_t *data, const size_t datalen);		// Alias CRC-32/ADCCP, PKZIP, Ethernet, 802.3
  uint32_t cksum(const uint8_t *data, const size_t datalen);		// Alias CRC-32/POSIX

  uint32_t crc32_upd(const uint8_t *data, size_t len);			// Call for subsequent calculations with previous seed
  uint32_t cksum_upd(const uint8_t *data, size_t len);			// Call for subsequent calculations with previous seed
#if !CRC_SW
  uint32_t generic(const uint32_t polyom, const uint32_t seed, const uint32_t flags, const uint8_t *data, const size_t datalen); //Not available in non-hw-variant (not T3.x)
#endif
private:
#if CRC_SW
  uint32_t seed;
#else
  uint32_t update(const uint8_t *data, const size_t datalen);
#endif
};

#endif