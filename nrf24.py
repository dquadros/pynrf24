#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# Python port of Maniacbug NRF24L01 library
# Author: Joao Paulo Barraca <jpbarraca@gmail.com>
# Revised by Daniel Quadros <dqsoft.blogspot@gmail.com>
#

try:
    # For Raspberry Pi
    # GPIO pins will use BCM convenction
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
except ImportError:
    try:
        #For BBBB
        import Adafruit_BBIO.GPIO as GPIO
    except ImportError:
        raise ImportError('Neither RPi.GPIO nor Adafruit_BBIO.GPIO module found.')

# Try to Use spidev (which is faster) and then try Adafruit_BBIO
try:
    import spidev
    ADAFRUID_BBIO_SPI = False
except:
    from Adafruit_BBIO.SPI import SPI
    ADAFRUID_BBIO_SPI = True


# Use a monotonic clock if available to avoid unwanted side effects from clock
# changes (available on Python 3)
try:
    from time import monotonic
except ImportError:
    from time import time as monotonic

import time
import sys

if sys.version > '3':
    long = int


class NRF24:
    # Some limits
    MAX_CHANNEL = 127
    MAX_PAYLOAD_SIZE = 32

    # PA Levels
    PA_MIN = 0x00
    PA_LOW = 0x01
    PA_HIGH = 0x02
    PA_MAX = 0x03
    PA_ERROR = 0x04

    # Bit rates
    BR_1MBPS = 0
    BR_2MBPS = 1
    BR_250KBPS = 2

    # CRC
    CRC_DISABLED = 0
    CRC_8 = 1
    CRC_16 = 2

    # Registers
    CONFIG = 0x00
    EN_AA = 0x01
    EN_RXADDR = 0x02
    SETUP_AW = 0x03
    SETUP_RETR = 0x04
    RF_CH = 0x05
    RF_SETUP = 0x06
    STATUS = 0x07
    OBSERVE_TX = 0x08
    RPD = 0x09  # CD on Non-P version
    RX_ADDR_P0 = 0x0A
    RX_ADDR_P1 = 0x0B
    RX_ADDR_P2 = 0x0C
    RX_ADDR_P3 = 0x0D
    RX_ADDR_P4 = 0x0E
    RX_ADDR_P5 = 0x0F
    TX_ADDR = 0x10
    RX_PW_P0 = 0x11
    RX_PW_P1 = 0x12
    RX_PW_P2 = 0x13
    RX_PW_P3 = 0x14
    RX_PW_P4 = 0x15
    RX_PW_P5 = 0x16
    FIFO_STATUS = 0x17
    DYNPD = 0x1C
    FEATURE = 0x1D

    # Bit Mask Mnemonics - CONFIG register
    MASK_RX_DR = 0x40
    MASK_TX_DS = 0x20
    MASK_MAX_RT = 0x10
    EN_CRC = 0x08
    CRCO = 0x04
    PWR_UP = 0x02
    PRIM_RX = 0x01
    
    # Bit Mask Mnemonics - STATUS register
    RX_DR = 0x40
    TX_DS = 0x20
    MAX_RT = 0x10
    TX_FULL = 0x01
    RX_P_NO_MASK = 0x0E # isolate pipe number

    # Bit Mask Mnemonics - FIFO_STATUS register
    TX_REUSE = 0x40
    TXFIFO_FULL = 0x20
    TXFIFO_EMPTY = 0x10
    RXFIFO_FULL = 0x02
    RXFIFO_EMPTY = 0x01

    # Bit Mask Mnemonics - DYNPD register
    DPL_P5 = 0x20
    DPL_P4 = 0x10
    DPL_P3 = 0x08
    DPL_P2 = 0x04
    DPL_P1 = 0x02
    DPL_P0 = 0x01
    
    # Bit Mask Mnemonics - FEATURE register
    EN_DPL = 0x04
    EN_ACK_PAY = 0x02
    EN_DYN_ACK = 0x01

    # Shift counts
    ARD = 4
    ARC = 0
    PLOS_CNT = 4
    ARC_CNT = 0
    RX_P_NO = 1

    # Instruction Mnemonics
    R_REGISTER = 0x00
    W_REGISTER = 0x20
    REGISTER_MASK = 0x1F
    ACTIVATE = 0x50
    R_RX_PL_WID = 0x60
    R_RX_PAYLOAD = 0x61
    W_TX_PAYLOAD = 0xA0
    W_ACK_PAYLOAD = 0xA8
    FLUSH_TX = 0xE1
    FLUSH_RX = 0xE2
    REUSE_TX_PL = 0xE3
    NOP = 0xFF

    # Non-P omissions
    LNA_HCURR = 0x01
    LNA_ON = 1
    LNA_OFF = 0

    # P model Mask Mnemonics
    RF_DR_LOW = 0x20
    RF_DR_HIGH = 0x08
    RF_PWR_LOW = 0x02
    RF_PWR_HIGH = 0x04

    datarate_e_str_P = ["1MBPS", "2MBPS", "250KBPS"]
    model_e_str_P = ["nRF24L01", "nRF24l01+"]
    crclength_e_str_P = ["Disabled", "8 bits", "16 bits"]
    pa_dbm_e_str_P = ["PA_MIN", "PA_LOW", "PA_HIGH", "PA_MAX"]

    @staticmethod
    def print_single_status_line(name, value):
        """Prints name = value"""
        print("{0:<16}= {1}".format(name, value))

    @staticmethod
    def _to_8b_list(data):
        """Convert an arbitray iteratable or single int to a list of ints
            where each int is smaller than 256."""
        if isinstance(data, str):
            data = [ord(x) & 0xFF for x in data]
        elif isinstance(data, (int, long)):
            data = [data & 0xFF]
        else:
            data = [int(x) & 0xFF for x in data]
        return data

    def __init__(self, major=None, minor=None, ce_pin=None, irq_pin=None):
        """Construtor.
        
            major and minor selects SPI port,
            ce_pin is optional GPIO pin number for CE signal
            irq_pin is optional GPIO pin number for IRQ signal"""
        # defaults and miscelaneous initialization
        self.payload_size = 32  # *< Fixed size of payloads
        self.ack_payload_available = False  # *< Whether there is an ack payload waiting
        self.dynamic_payloads_enabled = False  # *< Whether dynamic payloads are enabled.
        self.ack_payload_length = 5  # *< Dynamic size of pending ack payload.
        self.pipe0_reading_address = None  # *< Last address set on pipe 0 for reading.
        self.spidev = None
        self.last_error = 0
        self.auto_ack = 0
        self.address_length = 5

        # If all parameters are available, lets start the radio!
        if major is not None and minor is not None and irq_pin is not None:
            self.begin(major, minor, ce_pin, irq_pin)

    def begin(self, major, minor, ce_pin, irq_pin):
        """Radio initialization, must be called before anything else.
        
            major and minor selects SPI port,
            ce_pin is GPIO pin number for CE signal
            irq_pin is optional GPIO pin number for IRQ signal"""
        # Initialize SPI bus
        if ADAFRUID_BBIO_SPI:
            self.spidev = SPI(major, minor)
            self.spidev.bpw = 8
            try:
                self.spidev.msh = 10000000  # Maximum supported by NRF24L01+
            except IOError:
                pass  # Hardware does not support this speed
        else:
            self.spidev = spidev.SpiDev()
            self.spidev.open(major, minor)
            self.spidev.bits_per_word = 8
            try:
                self.spidev.max_speed_hz = 10000000  # Maximum supported by NRF24L01+
            except IOError:
                pass  # Hardware does not support this speed

        self.spidev.cshigh = False
        self.spidev.mode = 0
        self.spidev.loop = False
        self.spidev.lsbfirst = False
        self.spidev.threewire = False

        # Save pin numbers
        self.ce_pin = ce_pin
        self.irq_pin = irq_pin

        # If CE pin is not used, CE signal must be always high
        if self.ce_pin is not None:
            GPIO.setup(self.ce_pin, GPIO.OUT)
        
        # IRQ pin is optional
        if self.irq_pin is not None:
            GPIO.setup(self.irq_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        time.sleep(5 / 1000000.0)

        # Reset radio registers
        self.reset()

        # Restore our default PA level
        self.setPALevel(NRF24.PA_MAX)

        # Determine if this is a p or non-p RF24 module and then
        # reset our data rate back to default value. This works
        # because a non-P variant won't allow the data rate to
        # be set to 250Kbps.
        self.p_variant = False  # False for RF24L01 and true for RF24L01P
        if self.setDataRate(NRF24.BR_250KBPS):
            self.p_variant = True

        # Then set the data rate to the slowest (and most reliable) speed supported by all
        # hardware.
        self.setDataRate(NRF24.BR_1MBPS)

        # Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
        # WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
        # sizes must never be used. See documentation for a more complete explanation.
        # This must be done after setDataRate()
        self.setRetries(int('0101', 2), 15)
        # Line bellow will set maximum (4ms) delay
        #self.setRetries(15, 15)

        # Initialize CRC and request 2-byte (16bit) CRC
        self.setCRCLength(NRF24.CRC_16)

        # Disable dynamic payloads, to match dynamic_payloads_enabled setting
        self.write_register(NRF24.DYNPD, 0)

        # Set up default configuration.  Callers can always change it later.
        # This channel should be universally safe and not bleed over into adjacent
        # spectrum.
        self.channel = 76
        self.setChannel(self.channel)

        # Powers up the radio, this can take up to 4.5ms
        # when CE is low radio will be in standby and will initiate
        # reception or transmission very shortly after CE is raised
        # If CE pin is not used, will Power up only on startListening and stopListening
        if self.ce_pin is not None:
            self.powerUp()

        # Reset current status
        # Notice reset and flush is the last thing we do
        self.write_register(NRF24.STATUS, NRF24.RX_DR | NRF24.TX_DS | NRF24.MAX_RT)

        # Flush buffers
        self.flush_rx()
        self.flush_tx()
        self.clear_irq_flags()

    def end(self):
        """ End use of the radio """
        self.ce(0)
        if self.spidev:
            self.powerDown()
            self.spidev.close()
            self.spidev = None

    def startListening(self):
        """ Set radio for reception
        
            Use openReadingPipe to set up reception pipes before listening """
        self.write_register(NRF24.CONFIG, self.read_register(NRF24.CONFIG) | NRF24.PWR_UP | NRF24.PRIM_RX)
        self.flush_tx()
        self.flush_rx()
        self.clear_irq_flags()

        # Restore the pipe0 address, if exists
        if self.pipe0_reading_address:
            self.write_register(self.RX_ADDR_P0, self.pipe0_reading_address)

        # Go!
        self.ce(1)

        # wait for the radio to come up
        if self.ce_pin is None:
            time.sleep(45 / 10000.0) # 4.5 ms
        else:
            time.sleep(130 / 1000000.0) # 130us

    def ce(self, level, pulse=0):
        """ Controls CE pin """
        # CE Pin is optional (but highly recommended)
        if self.ce_pin is not None:
            GPIO.output(self.ce_pin, level)
            if pulse > 0:
                time.sleep(pulse)
                GPIO.output(self.ce_pin, 1 - level)

    def irqWait(self, timeout=30000):
        """ Wait for IRQ pin LOW, timeout in miliseconds """
        if self.irq_pin is None:
            return True
        # TODO: A race condition may occur here. => wait for level?
        if GPIO.input(self.irq_pin) == 0:  # Pin is already down. Packet is waiting?
            return True

        try:
            return GPIO.wait_for_edge(self.irq_pin, GPIO.FALLING, timeout) == 1
        except TypeError:  # Timeout parameter not supported
            return GPIO.wait_for_edge(self.irq_pin, GPIO.FALLING) == 1
        except AttributeError:
            raise RuntimeError("GPIO lib does not support wait_for_edge()")

    def read_register(self, reg, length=1):
        """ Read one or more registers """
        buf = [NRF24.R_REGISTER | (NRF24.REGISTER_MASK & reg)]
        buf += [NRF24.NOP] * max(1, length)

        resp = self.spidev.xfer2(buf)
        if length == 1:
            return resp[1]

        return resp[1:]

    def write_register(self, reg, value):
        """ Write register value """
        buf = [NRF24.W_REGISTER | (NRF24.REGISTER_MASK & reg)]
        buf += self._to_8b_list(value)
        self.spidev.xfer2(buf)

    def write_payload(self, buf):
        """ Writes data to the payload register, automatically padding it
            to match the required length. Returns the number of bytes
            actually written. """
        buf = self._to_8b_list(buf)
        if self.dynamic_payloads_enabled:
            if len(buf) > self.MAX_PAYLOAD_SIZE:
                raise RuntimeError("Dynamic payload is larger than the " +
                                   "maximum size.")
            blank_len = 0
        else:
            if len(buf) > self.payload_size:
                raise RuntimeError("Payload is larger than the fixed payload" +
                                   "size (%d vs. %d bytes)" % (len(buf), self.payload_size))
            blank_len = self.payload_size - len(buf)

        txbuffer = [NRF24.W_TX_PAYLOAD] + buf + ([0x00] * blank_len)
        self.spidev.xfer2(txbuffer)
        return len(txbuffer) - 1

    def read_payload(self, buf, buf_len=-1):
        """ Reads data from the payload register and clears the
            DR bit of the STATUS register. """
        if buf_len < 0:
            buf_len = self.payload_size

        if not self.dynamic_payloads_enabled:
            data_len = min(self.payload_size, buf_len)
            blank_len = self.payload_size - data_len
        else:
            data_len = self.getDynamicPayloadSize()
            blank_len = 0

        txbuffer = [NRF24.R_RX_PAYLOAD] + [NRF24.NOP] * (blank_len + data_len)

        payload = self.spidev.xfer2(txbuffer)
        del buf[:]
        buf += payload[1:data_len + 1]

        self.write_register(NRF24.STATUS, NRF24.RX_DR)

        return data_len

    def flush_rx(self):
        """ Flush RX buffer, return status """
        return self.spidev.xfer2([NRF24.FLUSH_RX])[0]

    def flush_tx(self):
        """ Flush TX buffer, return status """
        return self.spidev.xfer2([NRF24.FLUSH_TX])[0]

    def get_status(self):
        """ Read status register """
        return self.spidev.xfer2([NRF24.NOP])[0]

    def print_status(self, status):
        """ Print decoded status """
        status_str = "0x{0:02x} RX_DR={1:x} TX_DS={2:x} MAX_RT={3:x} RX_P_NO={4:x} TX_FULL={5:x}".format(
            status,
            1 if status & NRF24.RX_DR else 0,
            1 if status & NRF24.TX_DS else 0,
            1 if status & NRF24.MAX_RT else 0,
            ((status >> NRF24.RX_P_NO) & int("111", 2)),
            1 if status & NRF24.TX_FULL else 0)

        self.print_single_status_line("STATUS", status_str)

    def print_observe_tx(self, value):
        """ Print decoded observe_tx register:
        
            lost packets (accumulated) and retransmited packets (last tx) """
        tx_str = "OBSERVE_TX=0x{0:02x}: POLS_CNT={2:x} ARC_CNT={2:x}\r\n".format(
            value,
            (value >> NRF24.PLOS_CNT) & int("1111", 2),
            (value >> NRF24.ARC_CNT) & int("1111", 2))
        self.print_single_status_line("OBSERVE_TX", tx_str)

    def print_byte_register(self, name, reg, qty=1):
        """ Print byte registers """
        registers = ["0x{:0>2x}".format(self.read_register(reg+r)) for r in range(0, qty)]
        self.print_single_status_line(name, " ".join(registers))

    def print_address_register(self, name, reg, qty=1):
        """ Print address register (LSB to MSB) """
        address_registers = ["0x{0:>02x}{1:>02x}{2:>02x}{3:>02x}{4:>02x}".format(
            *self.read_register(reg+r, 5))
            for r in range(qty)]

        self.print_single_status_line(name, " ".join(address_registers))

    def setChannel(self, channel):
        """ Set radio channel (0 to MAX_CHANNEL) """
        if channel < 0 or channel > self.MAX_CHANNEL:
            raise RuntimeError("Channel number out of range")
        self.channel = channel
        self.write_register(NRF24.RF_CH, channel)

    def getChannel(self):
        """ Read channel register """
        return self.read_register(NRF24.RF_CH)

    def setPayloadSize(self, size):
        """ Set payload size """
        self.payload_size = min(max(size, 1), NRF24.MAX_PAYLOAD_SIZE)

    def getPayloadSize(self):
        """ Get payload size """
        return self.payload_size

    def printDetails(self):
        """ Prints register values and other information """
        self.print_status(self.get_status())
        self.print_address_register("RX_ADDR_P0-1", NRF24.RX_ADDR_P0, 2)
        self.print_byte_register("RX_ADDR_P2-5", NRF24.RX_ADDR_P2, 4)
        self.print_address_register("TX_ADDR", NRF24.TX_ADDR)

        self.print_byte_register("RX_PW_P0-6", NRF24.RX_PW_P0, 6)
        self.print_byte_register("EN_AA", NRF24.EN_AA)
        self.print_byte_register("EN_RXADDR", NRF24.EN_RXADDR)
        self.print_byte_register("RF_CH", NRF24.RF_CH)
        self.print_byte_register("RF_SETUP", NRF24.RF_SETUP)
        self.print_byte_register("SETUP_AW", NRF24.SETUP_AW)
        self.print_byte_register("OBSERVE_TX", NRF24.OBSERVE_TX)
        self.print_byte_register("CONFIG", NRF24.CONFIG)
        self.print_byte_register("FIFO_STATUS", NRF24.FIFO_STATUS)
        self.print_byte_register("DYNPD", NRF24.DYNPD)
        self.print_byte_register("FEATURE", NRF24.FEATURE)

        self.print_single_status_line("Data Rate", NRF24.datarate_e_str_P[self.getDataRate()])
        self.print_single_status_line("Model", NRF24.model_e_str_P[self.isPVariant()])
        self.print_single_status_line("CRC Length", NRF24.crclength_e_str_P[self.getCRCLength()])
        self.print_single_status_line("PA Power", NRF24.pa_dbm_e_str_P[self.getPALevel()])

    def stopListening(self):
        """ Stop listenning and set up transmission """
        self.ce(0)
        self.flush_tx()
        self.flush_rx()
        self.clear_irq_flags()

        # Enable TX
        self.write_register(NRF24.CONFIG,
                            (self.read_register(NRF24.CONFIG) | NRF24.PWR_UP) & ~NRF24.PRIM_RX)

        # Enable pipe 0 for auto-ack
        self.write_register(NRF24.EN_RXADDR, self.read_register(NRF24.EN_RXADDR) | 1)

        # wait for the radio to come up
        if self.ce_pin is None:
            time.sleep(45 / 10000.0) # 4.5 ms
        else:
            time.sleep(130 / 1000000.0) # 130us

    def powerDown(self):
        """ Power down radio """
        self.write_register(NRF24.CONFIG, self.read_register(NRF24.CONFIG) & ~ NRF24.PWR_UP)

    def powerUp(self):
        """ Power up radio """
        self.write_register(NRF24.CONFIG, self.read_register(NRF24.CONFIG) | NRF24.PWR_UP)
        time.sleep(4.5e-3)

    def write(self, buf):
        """ Sends buf and wait for end of transmission and acknowledgement
        
            call stopListenning and openWritingPipe before sending
            buf can be a single int or a container of char or int """
        self.last_error = None
        length = self.write_payload(buf)
        self.ce(1)

        sent_at = monotonic()
        packet_time = ((1 + length + self.crc_length + self.address_length) * 8 + 9)/(self.data_rate_bits * 1000.)

        if self.auto_ack != 0:
            packet_time *= 2

        if self.retries != 0 and self.auto_ack != 0:
            timeout = sent_at + (packet_time + self.delay)*self.retries
        else:
            timeout = sent_at + packet_time * 2  # 2 is empiric

        while monotonic() < timeout:
            time.sleep(packet_time)
            status = self.get_status()
            if status & NRF24.TX_DS:
                self.ce(0)
                return True

            if status & NRF24.MAX_RT:
                self.last_error = 'MAX_RT'
                self.ce(0)
                break

        self.ce(0)
        if self.last_error is None:
            self.last_error = 'TIMEOUT'

        self.flush_tx()  # Avoid leaving the payload in tx fifo
        return False

    def startFastWrite(self, buf):
        """ Starts sending of buf but do not wait for end of transmission. CE is left high."""
        self.write_payload(buf)
        self.ce(1)

    def startWrite(self, buf):
        """ Starts sending of buf but do not wait for end of transmission. CE is pulsed."""
        self.write_payload(buf)
        self.ce(1, 10e-6) # Pulse CE to start tranmission

    def getDynamicPayloadSize(self):
        """ Reads the size of received payload when using dynamic payloads """
        return self.spidev.xfer2([NRF24.R_RX_PL_WID, NRF24.NOP])[1]

    def available(self, pipe_num=None, irq_wait=False, irq_timeout=30000):
        """ Tests if there is a reception available
            
            pipe_num should be None or a list. If not None, it will receive information 
            on pipes with available data.
            
            if irq_wait is True, will wait for IRQ line to change from HIGH to LOW
            irq_timeout is the timeout for this wait, in miliseconds """
        status = self.get_status()
        result = False

        # Sometimes the radio specifies that there is data in one pipe but
        # doesn't set the RX flag...
        if status & NRF24.RX_DR or (status & NRF24.RX_P_NO_MASK != NRF24.RX_P_NO_MASK):
            result = True
        else:
            if irq_wait:  # Will use IRQ wait
                if self.irqWait(irq_timeout):  # Do we have a packet?
                    status = self.get_status()  # Seems like we do!
                    if status & NRF24.RX_DR or (status & NRF24.RX_P_NO_MASK != NRF24.RX_P_NO_MASK):
                        result = True

        if pipe_num is not None:
            del pipe_num[:]
            if result:
                pipe_num.append((status & NRF24.RX_P_NO_MASK) >> NRF24.RX_P_NO)

        # Handle ack payload receipt
        if status & NRF24.TX_DS:
            self.write_register(NRF24.STATUS, NRF24.TX_DS)

        return result

    def read(self, buf, buf_len=-1):
        """ Read payload from received packet. Returns != 0 if there are more packets in the FIFO. """
        # Fetch the payload
        self.read_payload(buf, buf_len)

        # was this the last of the data available?
        return self.read_register(NRF24.FIFO_STATUS) & NRF24.RXFIFO_EMPTY

    def clear_irq_flags(self):
        """ Clear flags in status register. """
        self.write_register(NRF24.STATUS, NRF24.RX_DR | NRF24.TX_DS | NRF24.MAX_RT)

    def whatHappened(self):
        """ Read the status & reset the status in one easy call
        
            Returns a dictionary informing tx_ok, tx_fail and rx_ready
            """
        status = self.spidev.xfer2(NRF24.STATUS, NRF24.RX_DR | NRF24.TX_DS | NRF24.MAX_RT)[0]

        # Report to the user what happened
        tx_ok = status & NRF24.TX_DS
        tx_fail = status & NRF24.MAX_RT
        rx_ready = status & NRF24.RX_DR
        return {'tx_ok': tx_ok, "tx_fail": tx_fail, "rx_ready": rx_ready}

    def openWritingPipe(self, address):
        """ Sets tx address 
            
            address is the address in transmited packet (2 to 5 bytes), LSB to MSB
            """
        self.write_register(NRF24.RX_ADDR_P0, address)
        self.write_register(NRF24.TX_ADDR, address)
        if not self.dynamic_payloads_enabled:
            self.write_register(NRF24.RX_PW_P0, self.payload_size)

    def openReadingPipe(self, pipe, address):
        """ Sets rx address for a pipe and enables it for recieving
            
            pipe should be 0 to 5
            address is the address
                for pipe 0 or 1, 2 to 5 bytes LSB to MSB
                for pipes 2 to 5, 1 byte (LSB, MSB cames from pipe 1)
            """
        if pipe >= 6:
            raise RuntimeError("Invalid pipe number")
        if (pipe >= 2 and len(address) > 1) or len(address) > 5:
            raise RuntimeError("Invalid adress length")

        # If this is pipe 0, cache the address.  This is needed because
        # openWritingPipe() will overwrite the pipe 0 address, so
        # startListening() will have to restore it.
        if pipe == 0:
            self.pipe0_reading_address = address

        self.write_register(NRF24.RX_ADDR_P0 + pipe, address)
        if not self.dynamic_payloads_enabled:
            self.write_register(NRF24.RX_PW_P0 + pipe, self.payload_size)

        # Note it would be more efficient to set all of the bits for all open
        # pipes at once.  However, I thought it would make the calling code
        # more simple to do it this way.
        self.write_register(NRF24.EN_RXADDR,
                            self.read_register(NRF24.EN_RXADDR) | (1 << pipe))

    def closeReadingPipe(self, pipe):
        """ Disabe a receiving pipe """
        self.write_register(NRF24.EN_RXADDR,
                            self.read_register(NRF24.EN_RXADDR) & ~(1 << pipe))

    def toggle_features(self):
        """ Enable DUNPD and FEATURE registers on non P variant """
        buf = [NRF24.ACTIVATE, 0x73]
        self.spidev.xfer2(buf)

    def enableDynamicPayloads(self):
        """ Enables dynamic size payloads """
        # First try writing to the features
        self.write_register(NRF24.FEATURE, self.read_register(NRF24.FEATURE) | NRF24.EN_DPL)

        # If it didn't work, the features are not enabled
        if not self.read_register(NRF24.FEATURE):
            # So enable them and try again
            self.toggle_features()
            self.write_register(NRF24.FEATURE, self.read_register(NRF24.FEATURE) | NRF24.EN_DPL)

        # Enable dynamic payload on all pipes
        # Not sure the use case of only having dynamic payload on certain
        # pipes, so the library does not support it.
        self.write_register(NRF24.DYNPD, self.read_register(NRF24.DYNPD) | 0b00111111)

        self.dynamic_payloads_enabled = True

    def enableAckPayload(self):
        """ Enable ack payload and dynamic payload features """
        # First try writing to the features
        self.write_register(NRF24.FEATURE,
                            self.read_register(NRF24.FEATURE) | NRF24.EN_ACK_PAY | NRF24.EN_DPL)

        # If it didn't work, the features are not enabled
        if not self.read_register(NRF24.FEATURE):
            # So enable them and try again
            self.toggle_features()
            self.write_register(NRF24.FEATURE,
                                self.read_register(NRF24.FEATURE) | NRF24.EN_ACK_PAY | NRF24.EN_DPL)

        # Enable dynamic payload on pipes 0 & 1
        self.write_register(NRF24.DYNPD, self.read_register(NRF24.DYNPD) | NRF24.DPL_P1 | NRF24.DPL_P0)

    def writeAckPayload(self, pipe, buf, buf_len):
        """ Write payload for acknowledgement """
        txbuffer = [NRF24.W_ACK_PAYLOAD | (pipe & 0x7)]

        max_payload_size = 32
        data_len = min(buf_len, max_payload_size)
        txbuffer.extend(buf[0:data_len])

        self.spidev.xfer2(txbuffer)

    def isAckPayloadAvailable(self):
        """ Check if there is a payload in a acknowledgement.

            Note: this will clear the ack payload flag. """
        result = self.ack_payload_available
        self.ack_payload_available = False
        return result

    def isPVariant(self):
        """ Returns true if nRF24L01+, False if nRF24L01 """
        return self.p_variant

    def setAutoAck(self, enable):
        """ Enable or disable auto acknoledge for all pipes """
        if enable:
            self.write_register(NRF24.EN_AA, 0x3F)
            self.auto_ack = 0x3f
            if self.self.getCRCLength() == NFR24.CRC_DISABLED:
                self.setCRCLength(NRF24.CRC_8)  # Enhanced Shockburst requires at least 1 byte CRC
        else:
            self.auto_ack = 0
            self.write_register(NRF24.EN_AA, 0)

    def setAutoAckPipe(self, pipe, enable):
        """ Enable or disable auto acknoledge for an specific pipe """
        if pipe <= 6:
            en_aa = self.read_register(NRF24.EN_AA)
            if enable:
                if self.self.getCRCLength() == NFR24.CRC_DISABLED:
                    self.setCRCLength(NRF24.CRC_8)  # Enhanced Shockburst requires at least 1 byte CRC
                en_aa |= 1 << pipe
                self.auto_ack |= 1 << pipe
            else:
                en_aa &= ~1 << pipe
                self.auto_ack &= ~1 << pipe

            self.write_register(NRF24.EN_AA, en_aa)

    def setAddressWidth(self, width):
        """ Set address width (2 to 5 bytes) """
        if width >= 2 and width <= 5:
            self.write_register(NRF24.SETUP_AW, width - 2)
            self.address_width = width

    def testCarrier(self):
        """ Tests if there is a radio signal at current channel. """
        return self.read_register(NRF24.RPD) & 1

    def setPALevel(self, level):
        """ Set transmission level """
        setup = self.read_register(NRF24.RF_SETUP)
        setup &= ~(NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH)

        if level == NRF24.PA_MAX:
            setup |= NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH
        elif level == NRF24.PA_HIGH:
            setup |= NRF24.RF_PWR_HIGH
        elif level == NRF24.PA_LOW:
            setup |= NRF24.RF_PWR_LOW
        elif level == NRF24.PA_MIN:
            pass
        elif level == NRF24.PA_ERROR:
            # On error, go to maximum PA
            setup |= NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH

        self.write_register(NRF24.RF_SETUP, setup)

    def getPALevel(self):
        """ Inform current transmission level """
        power = self.read_register(NRF24.RF_SETUP) & (NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH)
        if power == (NRF24.RF_PWR_LOW | NRF24.RF_PWR_HIGH):
            return NRF24.PA_MAX
        elif power == NRF24.RF_PWR_HIGH:
            return NRF24.PA_HIGH
        elif power == NRF24.RF_PWR_LOW:
            return NRF24.PA_LOW
        else:
            return NRF24.PA_MIN

    def setDataRate(self, speed):
        """ Set data rate. returns True if success. """
        setup = self.read_register(NRF24.RF_SETUP)
        setup &= ~(NRF24.RF_DR_LOW | NRF24.RF_DR_HIGH)

        if speed == NRF24.BR_250KBPS:
            # Must set the RF_DR_LOW to 1 RF_DR_HIGH (used to be RF_DR) is already 0
            # Making it '10'.
            self.data_rate_bits = 250
            self.data_rate = NRF24.BR_250KBPS
            setup |= NRF24.RF_DR_LOW
        elif speed == NRF24.BR_2MBPS:
            # Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
            # Making it '01'
            self.data_rate_bits = 2000
            self.data_rate = NRF24.BR_2MBPS
            setup |= NRF24.RF_DR_HIGH
        else:
            # 1Mbs
            self.data_rate_bits = 1000
            self.data_rate = NRF24.BR_1MBPS

        self.write_register(NRF24.RF_SETUP, setup)

        # Verify our result
        return self.read_register(NRF24.RF_SETUP) == setup

    def getDataRate(self):
        """ Inform current data rate """
        dr = self.read_register(NRF24.RF_SETUP) & (NRF24.RF_DR_LOW | NRF24.RF_DR_HIGH)
        # Order matters in our case below
        if dr == NRF24.RF_DR_LOW:
            # '10' = 250KBPS
            return NRF24.BR_250KBPS
        elif dr == NRF24.RF_DR_HIGH:
            # '01' = 2MBPS
            return NRF24.BR_2MBPS
        else:
            # '00' = 1MBPS
            return NRF24.BR_1MBPS

    def setCRCLength(self, length):
        """ Set CRC length 
        
            length = CRC_DISABLED, CRC_8 or CRC_16 """
        config = self.read_register(NRF24.CONFIG) & ~(NRF24.EN_CRC | NRF24.CRCO)

        if length == NRF24.CRC_DISABLED:
            self.crc_length = 0
        elif length == NRF24.CRC_8:
            config |= NRF24.EN_CRC
            self.crc_length = 1
        else:
            config |= NRF24.EN_CRC
            config |= NRF24.CRCO
            self.crc_length = 2

        self.write_register(NRF24.CONFIG, config)

    def getCRCLength(self):
        """ Get CRC length 
        
            returns CRC_DISABLED, CRC_8 or CRC_16 """
        result = NRF24.CRC_DISABLED
        config = self.read_register(NRF24.CONFIG) & (NRF24.CRCO | NRF24.EN_CRC)

        if config & NRF24.EN_CRC:
            if config & NRF24.CRCO:
                result = NRF24.CRC_16
            else:
                result = NRF24.CRC_8

        return result

    def disableCRC(self):
        """ Disable CRC """
        disable = self.read_register(NRF24.CONFIG) & ~NRF24.EN_CRC
        self.write_register(NRF24.CONFIG, disable)

    def setRetries(self, delay, count):
        """ Set timeout and number of retries
        
            delay (timeout) 0-15 as per datasheet
            count 0-15 max number of retries (0=disable retries)"""
        self.write_register(NRF24.SETUP_RETR, (delay & 0xf) << NRF24.ARD | (count & 0xf) << NRF24.ARC)
        self.delay = delay * 0.000250
        self.retries = count
        self.max_timeout = (self.payload_size / float(self.data_rate_bits) + self.delay) * self.retries
        self.timeout = (self.payload_size / float(self.data_rate_bits) + self.delay)

    def getRetries(self):
        """ Return current retry configuration. """
        return self.read_register(NRF24.SETUP_RETR)

    def getMaxTimeout(self):
        """ Return current maximum timeout (no ack after all retries). """
        return self.max_timeout

    def getTimeout(self):
        """ Return current timeout for one transmission. """
        return self.timeout

    def reset(self):
        """ Make sure the NRF is in the same state as after power up
            to avoid problems resulting from left over configuration
            from other programs."""
        self.ce(0)
        reset_values = {0: 0x08, 1: 0x3F, 2: 0x03, 3: 0x03, 4: 0x03, 5: 0x02, 6: 0x0e,
                        0x0a: [0xe7, 0xe7, 0xe7, 0xe7, 0xe7],
                        0x0b: [0xc2, 0xc2, 0xc2, 0xc2, 0xc2],
                        0x0c: 0xc3, 0x0d: 0xc4, 0x0e: 0xc5, 0x0f: 0xc6,
                        0x10: [0xe7, 0xe7, 0xe7, 0xe7, 0xe7],
                        0x11: 0, 0x12: 0, 0x13: 0, 0x14: 0, 0x15: 0, 0x16: 0,
                        0x1c: 0, 0x1d: 0}
        for reg, value in reset_values.items():
            self.write_register(reg, value)

        self.flush_rx()
        self.flush_tx()
