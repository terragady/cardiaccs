// Cardiaccs I2C library. Forked from i2cdevlib.

/* ============================================
I2C device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <Wire.h>


//#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
//  #define Serial SERIAL_PORT_USBVIRTUAL
//#endif



#include "I2C.h"

#include <array>
#include <bitset>

/** Default constructor.
 */
I2C::I2C() {
}

/** Read a single bit from an 8-bit device register.
 
 * @param reg Register reg to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @return Status of read operation (negative value indicates failure)
 */

int I2C::readTimeout = defaultReadTimeout;

int I2C::read8Bit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t &data)
 {
     uint8_t b = 0;
     const int count = read8(addr, reg, b);
     data = b & std::bitset<8>().set(bitNum).to_ulong();
     return count;
 }

 /** Read a single bit from a 16-bit device register.
  
  * @param reg Register reg to read from
  * @param bitNum Bit position to read (0-15)
  * @param data Container for single bit value
  * @return Status of read operation (negative value indicates failure)
  */
int I2C::read16Bit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint16_t &data)
{
    uint16_t b = 0;
    const int count = read16(addr, reg, b);
    data = b & std::bitset<16>().set(bitNum).to_ulong();
    return count;
}

uint16_t I2C::bitsValues(uint16_t val, uint8_t start, uint8_t length)
{
    const std::bitset<16> mask((1 << length) - 1); // create mask of first length bit set to 1
    auto bs = (std::bitset<16>(val) >> (start - length + 1)) & mask;
    const uint16_t res = bs.to_ulong();
    return res;
}

/** Read multiple bits from an 8-bit device register.
 
 * @param reg Register reg to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (negative value indicates failure)
 */

int I2C::read8Bits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t &data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    int res;
    uint8_t b = 0;
    if ((res = read8(addr, reg, b)) >= 0)
        data = bitsValues(b, bitStart, length);
    return res;
}

/** Read multiple bits from a 16-bit device register.
 
 * @param reg Register reg to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (negative value indicates failure)
 */
int I2C::read16Bits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint16_t &data)
{
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    int count;
    uint16_t w = 0;
    if ((count = read16(addr, reg, w)) >= 0)
        data = bitsValues(w, bitStart, length);
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param reg Register reg to read from
 * @param data Container for byte value read from device
 * @return Status of read operation (negative value indicates failure)
 */
int I2C::read8(uint8_t addr, uint8_t reg, uint8_t &data)
{
    return read8(addr, reg, 1, &data);
}

uint16_t I2C::bytesToWord(uint8_t a, uint8_t b)
{
    return (a << 8) | b;
}

std::pair<uint8_t, uint8_t> I2C::wordToBytes(uint16_t data)
{
    return { uint8_t(data >> 8), uint8_t(data) };
}

/** Read single word from a 16-bit device register.
* @param reg Register reg to read from
* @param data Container for word value read from device
* @return Status of read operation (negative value indicates failure)
*/
int I2C::read16(uint8_t addr, uint8_t reg, uint16_t &data)
{
    return read16(addr, reg, 1, &data);
}

/** Read multiple words from a 16-bit device register.

* @param reg First register reg to read from
* @param length Number of words to read
* @param data Buffer to store read data in
* @return Number of words read (negative value indicates failure)
*/
int I2C::read16(uint8_t addr, uint8_t reg, uint8_t length, uint16_t data[])
{
    const auto buf = reinterpret_cast<uint8_t *>(data);

    int res;
    if ((res = read8(addr, reg, 2 * length, buf)) <= 0)
        return res;

    for (uint8_t i = 0; i < length; ++i)
        data[i] = bytesToWord(buf[2 * i], buf[2 * i + 1]);

    return res == length * 2 ? length : int(FT::Status::readFailure);
}


bool I2C::timeover(int timeout, int64_t t1)
{
    const int32_t t2 = millis();
    return timeout && (t2 - t1 >= timeout);
}

/** Read multiple bytes from an 8-bit device register.

* @param reg First register reg to read from
* @param length Number of bytes to read
* @param data Buffer to store read data in
* @return Number of bytes read (negative value indicates failure)
*/

int I2C::read8(uint8_t addr, uint8_t reg, uint8_t length, uint8_t data[])
{
  if constexpr(ftDebug) {
        //Serial.print("I2C (0x");
        //Serial.print(addr, HEX);
        //Serial.print(") reading ");
        //Serial.print(length, DEC);
        //Serial.print(" bytes from 0x");
        //Serial.print(reg, HEX);
        //Serial.print("...");
  }

  int8_t count = 0;
  uint32_t t1 = millis();

  // this is added because min() does not work
  int foo, bar;
  if (int(length) < buffer_length)
      foo = int(length);
  else
      foo = buffer_length;

  for (uint8_t k = 0; k < length; k += foo) {
    if (int(length-k) < buffer_length) bar = int(length - k);
    else bar = buffer_length;
    
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.beginTransmission(addr);
    Wire.requestFrom(addr, (uint8_t)bar);
        
    for (; Wire.available() && (readTimeout == 0 || millis() - t1 < readTimeout); count++) {
      data[count] = Wire.read();
      if constexpr(ftDebug) {
        //Serial.print(data[count], HEX);
        //if (count + 1 < length) Serial.print(" ");
      }
    }
  }

  // check for timeout
  if (readTimeout > 0 && millis() - t1 >= readTimeout && count < length) count = -1; // timeout

  if constexpr(ftDebug) {
      //Serial.print(". Done (");
      //Serial.print(count, DEC);
      //Serial.println(" read).");
  }

  return count == length ? count : -1;

}

int I2C::read8(uint8_t addr, uint8_t length, uint8_t data[])
{

  int8_t count = 0;
  uint32_t t1 = millis();
    
    Wire.beginTransmission(addr);
    Wire.requestFrom(addr, length);
        
    for (; Wire.available() && (readTimeout == 0 || millis() - t1 < readTimeout); count++) {
      data[count] = Wire.read();
      if constexpr(ftDebug) {
        //Serial.print(data[count], HEX);
        //if (count + 1 < length) Serial.print(" ");
      }
    }

  // check for timeout
  if (readTimeout > 0 && millis() - t1 >= readTimeout && count < length) count = -1; // timeout

  return count == length ? count : -1;

}


/** write a single bit in an 8-bit device register.
 
 * @param reg Register reg to write to
 * @param bitNum Bit position to write (0-7)
 * @param data New bit value to write
 * @return Status of operation (negative value indicates failure)
 */
int I2C::write8Bit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b = 0;
    const int res = read8(addr, reg, b);
    if (res != 1) return res;
    b = std::bitset<8>(b).set(bitNum, data).to_ulong();
    return write8(addr, reg, b);
}

/** write a single bit in a 16-bit device register.
 
 * @param reg Register reg to write to
 * @param bitNum Bit position to write (0-15)
 * @param data New bit value to write
 * @return Status of operation (negative value indicates failure)
 */
int I2C::write16Bit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint16_t data)
{
    uint16_t w = 0;
    const int res = read16(addr, reg, w);
    if (res != 1)
        return res;
    w = std::bitset<16>(w).set(bitNum, data).to_ulong();
    return write16(addr, reg, w);
}

/** Write multiple bits in an 8-bit device register.
 
 * @param reg Register reg to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (negative value indicates failure)
 */
int I2C::write8Bits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b = 0;
    int res = read8(addr, reg, b);
    if (res > 0)
        res = write8(addr, reg, bitsValuesWrite(data, b, bitStart, length));
    return res;
}

uint16_t I2C::bitsValuesWrite(uint16_t val, uint16_t w, uint8_t start, uint8_t length)
{
    std::bitset<16> mask(val);
    auto bs = std::bitset<16>(w);
    for (int i = 0; i < length; ++i)
        bs[start - length + 1 + i] = mask.test(i);
    const uint16_t res = bs.to_ulong();
    return res;
}


/** Write multiple bits in a 16-bit device register.
 
 * @param reg Register reg to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
  * @return Status of operation (negative value indicates failure)
 */
int I2C::write16Bits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint16_t data)
{
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w = 0;
    const int res = read16(addr, reg, w);
    if (res <= 0)
        return res;        
    w = bitsValuesWrite(data, w, bitStart, length);
    return write16(addr, reg, w);
}

/** Write single byte to an 8-bit device register.
 
 * @param reg Register address to write to
 * @param data New byte value to write
  * @return Status of operation (negative value indicates failure)
 */
int I2C::write8(uint8_t addr, uint8_t reg, uint8_t data)
{
    return write8(addr, reg, 1, &data);
}

int I2C::write8(uint8_t addr, uint8_t reg, uint8_t length, const uint8_t data[])
{
  if constexpr(ftDebug) {
        //Serial.print("I2C (0x");
        //Serial.print(addr, HEX);
        //Serial.print(") writing ");
       // Serial.print(length, DEC);
        //Serial.print(" bytes to 0x");
        //Serial.print(reg, HEX);
        //Serial.print("...");
  }
    uint8_t status = 0;

    Wire.beginTransmission(addr);
    Wire.write((uint8_t) reg); // send address

    for (uint8_t i = 0; i < length; i++) {
        if constexpr(ftDebug) {
            //Serial.print(data[i], HEX);
            //if (i + 1 < length) Serial.print(" ");
        }
        Wire.write((uint8_t) data[i]);
    }

    status = Wire.endTransmission();

    return status == 0;
    
}

/** Write single word to a 16-bit device register.
 
 * @param reg Register address to write to
 * @param data New word value to write
  * @return Status of operation (negative value indicates failure)
 */
int I2C::write16(uint8_t addr, uint8_t reg, uint16_t data)
{
    // valgrind notifies here about uninitialised byte(s): "Syscall param ioctl(USBDEVFS_SUBMITURB).buffer points to uninitialised byte(s)"
    // Address 0x9e1d156 is 6 bytes inside a block of size 8 alloc'd
    // increasing size of our buffers to multiplies of 8 do not solve the problem
    // this appears problem of the FT library, which inside allocates some internal buffers.
    // https://stackoverflow.com/questions/19364942/points-to-uninitialised-bytes-valgrind-errors

    std::array<uint8_t, 2> buf { wordToBytes(data).first, wordToBytes(data).second };

    if constexpr(ftDebug) {
        //Serial.print("write word to dev: 0x");
        //Serial.print(addr, HEX);
        //Serial.print(", reg addr: 0x");
        //Serial.println(reg, HEX);
    }
    int res;
    if ((res = write8(addr, reg, 2, buf.data())) < 0)
        return res;

    return res == 2 ? 1 : int(FT::Status::writeFailure);
}
