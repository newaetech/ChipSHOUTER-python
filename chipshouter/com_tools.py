#
# This file is part of the ChipSHOUTER Python API.
# Copyright NewAE Technology Inc., 2017-2018.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# ChipSHOUTER is a registered trademark of NewAE Technology Inc.
#

"""
>>>  | Position                | Size   | Name               | Description                |
>>>  | :-------                | :----- | :---------------   | :------                    |
>>>  | 0                       | 1      | UNIT16's           | Number of UINT16 bitfields |
>>>  | 1                       | 1      | UNIT8's            | Number of UINT8 bitfields  |
>>>  | 2                       | 1      | Variable lengths   | Number of UINT8 bitfields  |
>>>  | 3                       | n0     | UINT16 bitfields   | Bits to come in UINT16's   |
>>>  | 3 + n                   | n1     | UINT8 bitfields    | Bits to come in UINT8's    |
>>>  | 3 + (n0 ... n1)         | n2     | Variable bitfields | Bits to come in variables  |
>>>  |                         | 1      | Packets to follow  |                            |
>>>  |                         | 1      | Length             |                            |
>>>  | 3 + 1 + 1 + (n0 ... n2) | n3     | UINT16's options   | UINT16 options             |
>>>  | 3 + 1 + 1 + (n0 ... n3) | n4     | UINT8's options    | UINT16 options             |
>>>  | 3 + 1 + 1 + (n0 ... n4) | n5     | Variable options   | UINT16 options             |
>>>  |                         |        |                    |                            |

"""

from __future__ import division

from binascii import hexlify
import serial
import serial.tools.list_ports
import pprint
import time
from .console.serial_interface import Serial_interface
from .console.console import Console

class Firmware_State_Exception(Exception):
    pass

class Max_Retry_Exception(Exception):
    pass

class Reset_Exception(Exception):
    pass

class Connection(object):
    START_UP_STRING = b'ChipShouter by NewAE Technology Inc.'

    def __init__(self, comport, logging = False):
        self.comport = comport
        self.log_input  = False 
        self.log_output = False
        self.log_ascii  = False
        if logging == True:
            self.log_input  = True 
            self.log_output = True 
            self.log_ascii  = True
        self.ctl_connect()

    def ctl_connect(self):
        self.s= serial.Serial(
            timeout = .01, # 1 second timeout.
            port = self.comport,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS
        )
        self.s.flushInput()
        self.s.flushOutput()

        self.logfile = open("log.txt", "w");
        self.logfile.write("Start Logging\n")
        self.logfile.write("-------------\n")

    def ctl_disconnect(self):
        self.s.close()
        self.logfile.close()

    def s_read(self, timeout = 1):
        """
        Read data from the serial interface.

        :Returns: bytes read.

        """
        if self.s.is_open:
            data = [] 
            b    = bytearray()
            try:
                self.s.timeout = 3
                data = self.s.read(1)
                
                if not len(data):
                    return b

                self.s.timeout = .04
                data += self.s.read(500)
            except Exception as e:
                print("Could not read from port" + str(e))

            start = data.find(b'\x7e')
            end   = data.find(b'\x7f')

            txt_start = b''
            txt_end   = b''
            if start < 0:
                txt_start = data
            elif end < 0:
                txt_start = data
            else:
                txt_start = data[0:start]
                txt_end   = data[end+1:]

            txt = txt_start + txt_end
            if len(txt):
                if self.log_ascii:
                    self.logfile.write(txt)

            # End logging 
            if Connection.START_UP_STRING in data:
                raise Reset_Exception('Shouter has reset') 

            if start < 0 or end < 0 or end < start:
                b = bytearray()
                return b
            if self.log_input:
                self.logfile.write('\nIN :' + str(len(data)) + '[' + hexlify(data) + ']' + '\n')
            b.extend(data[start:end])
            return b
        else:
            raise IOError('Comport is not open, use ctl_connect()')

    def s_write(self, data):
        """
        Write data to the serial interface.
        Raises an IOError if not connected.
        """
        self.s.flushOutput()

        if self.s.is_open:
            try:
                self.s.write(data)
                if self.log_output:
                    self.logfile.write('\nIN :' + str(len(data)) + '[' + hexlify(data) + ']' + '\n')
            except Exception as e:
                print("Could not write to port " + str(e))
        else:
            raise IOError('Comport is not open, use ctl_connect()')

class Option_group(object):
    """
    Base class for the options
    """
    def __init__(self):
        self.options = {}

    def get_value(self, item):
        """
        Get the value.

        :param item: The item to get

        """
        return self.options[item]['value']

    def set_value(self, item, value):
        """
        Set the value.

        :param item:  The item to set

        :param value: The value to set

        """
        self.options[item]['value'] = value

    def get_bools_array(self, bools, limit):
        """
        Set a bool option.

        :param item (int):  The item to set

        :param value (bool): The value to set.

        :param limit (int): The item to set.

        """
        bit_array = bytearray()
        bits_array_length = (limit) // 8

        for x in range(bits_array_length):
            bit_array.append(0)


        for x in range(limit): 
            # set the bits
            if bools[x]['value'] == True:
                index = x//8
                bit   = x % 8 
                bit_array[index] |= 1 << bit 

        return bit_array

    def set_bools(self, value, bools, limit):
        """
        Set a bool option.

        :param item (int):  The item to set

        :param value (bool): The value to set.

        :param limit (int): The item to set.

        """
        for x in range(limit):
            if value & 1 << x:
                bools[x]['value'] = True
            else:
                bools[x]['value'] = False
        pass

class t_16_Bit_Options(Option_group):
    """
    All the 16 bit options:

    >>> VOLTAGE                    = 0
    >>> VOLTAGE_MEASURED           = 1
    >>> PULSE_WIDTH                = 2
    >>> PULSE_REPEAT               = 3
    >>> PULSE_DEADTIME             = 4
    >>> ARM_TIMEOUT                = 5
    >>> FAULT_ACTIVE               = 6
    >>> FAULT_LATCHED              = 7
    >>> TEMPERATURE_MOSFET         = 8
    >>> TEMPERATURE_XFORMER        = 9
    >>> TEMPERATURE_DIODE          = 10
    >>> MAX                        = 11

    """
    VOLTAGE                    = 0
    VOLTAGE_MEASURED           = 1
    PULSE_WIDTH                = 2
    PULSE_REPEAT               = 3
    PULSE_DEADTIME             = 4
    ARM_TIMEOUT                = 5
    FAULT_ACTIVE               = 6
    FAULT_LATCHED              = 7
    TEMPERATURE_MOSFET         = 8
    TEMPERATURE_XFORMER        = 9
    TEMPERATURE_DIODE          = 10
    MEASURED_PULSE_WIDTH       = 11
    PAT_WAVE_LENGTH            = 12
    MAX                        = 13

    BIT_FAULT_PROBE            = 0
    BIT_FAULT_OVERTEMP         = 1
    BIT_FAULT_PANEL_OPEN       = 2
    BIT_FAULT_HIGH_VOLTAGE     = 3
    BIT_FAULT_RAM_CRC          = 4
    BIT_FAULT_EEPROM_CRC       = 5
    BIT_FAULT_GPIO_ERROR       = 6
    BIT_FAULT_LTFAULT_ERROR    = 7
    BIT_FAULT_TRIGGER_ERROR    = 8
    BIT_FAULT_HARDWARE_EXC     = 9
    BIT_FAULT_TRIGGER_GLITCH   = 10
    BIT_FAULT_OVERVOLTAGE      = 11
    BIT_FAULT_TEMP_SENSOR      = 12
    BIT_FAULT_LAST             = 12
    BIT_FAULT_MAX              = 13

    def __init__(self):
        super(t_16_Bit_Options, self).__init__()
        self.options = {
            t_16_Bit_Options.VOLTAGE              : {'value' : 0, 'name' : 'voltage'              },
            t_16_Bit_Options.VOLTAGE_MEASURED     : {'value' : 0, 'name' : 'voltage_measured'     },
            t_16_Bit_Options.PULSE_WIDTH          : {'value' : 0, 'name' : 'pulse_width'          },
            t_16_Bit_Options.PULSE_REPEAT         : {'value' : 0, 'name' : 'pulse_repeat'         },
            t_16_Bit_Options.PULSE_DEADTIME       : {'value' : 0, 'name' : 'pulse_deadtime'       },
            t_16_Bit_Options.ARM_TIMEOUT          : {'value' : 0, 'name' : 'arm_timeout'          },
            t_16_Bit_Options.FAULT_ACTIVE         : {'value' : 0, 'name' : 'fault_active'         },
            t_16_Bit_Options.FAULT_LATCHED        : {'value' : 0, 'name' : 'fault_latched'        },
            t_16_Bit_Options.TEMPERATURE_MOSFET   : {'value' : 0, 'name' : 'temperature_mosfet'   },
            t_16_Bit_Options.TEMPERATURE_XFORMER  : {'value' : 0, 'name' : 'temperature_xformer'  },
            t_16_Bit_Options.TEMPERATURE_DIODE    : {'value' : 0, 'name' : 'temperature_diode'    },
            t_16_Bit_Options.MEASURED_PULSE_WIDTH : {'value' : 0, 'name' : 'pulse_width_measured' },
            t_16_Bit_Options.PAT_WAVE_LENGTH      : {'value' : 0, 'name' : 'pat_wave_length'      }
        }
        self.faults_current = {
            t_16_Bit_Options.BIT_FAULT_PROBE             : {'value' : 0, 'name' :  'fault_probe'          },
            t_16_Bit_Options.BIT_FAULT_OVERTEMP          : {'value' : 0, 'name' :  'fault_overtemp'       },
            t_16_Bit_Options.BIT_FAULT_PANEL_OPEN        : {'value' : 0, 'name' :  'fault_panel_open'     },
            t_16_Bit_Options.BIT_FAULT_HIGH_VOLTAGE      : {'value' : 0, 'name' :  'fault_high_voltage'   },
            t_16_Bit_Options.BIT_FAULT_RAM_CRC           : {'value' : 0, 'name' :  'fault_ram_crc'        },
            t_16_Bit_Options.BIT_FAULT_EEPROM_CRC        : {'value' : 0, 'name' :  'fault_eeprom_crc'     },
            t_16_Bit_Options.BIT_FAULT_GPIO_ERROR        : {'value' : 0, 'name' :  'fault_gpio_error'     },
            t_16_Bit_Options.BIT_FAULT_LTFAULT_ERROR     : {'value' : 0, 'name' :  'fault_ltfault_error'  },
            t_16_Bit_Options.BIT_FAULT_TRIGGER_ERROR     : {'value' : 0, 'name' :  'fault_trigger_error'  },
            t_16_Bit_Options.BIT_FAULT_HARDWARE_EXC      : {'value' : 0, 'name' :  'fault_hardware_exc'   },
            t_16_Bit_Options.BIT_FAULT_TRIGGER_GLITCH    : {'value' : 0, 'name' :  'fault_trigger_glitch' },
            t_16_Bit_Options.BIT_FAULT_OVERVOLTAGE       : {'value' : 0, 'name' :  'fault_overvoltage'    },
            t_16_Bit_Options.BIT_FAULT_TEMP_SENSOR       : {'value' : 0, 'name' :  'fault_temp_sensor'    }
        }
        self.faults_latched = {
            t_16_Bit_Options.BIT_FAULT_PROBE             : {'value' : 0, 'name' :  'fault_probe'          },
            t_16_Bit_Options.BIT_FAULT_OVERTEMP          : {'value' : 0, 'name' :  'fault_overtemp'       },
            t_16_Bit_Options.BIT_FAULT_PANEL_OPEN        : {'value' : 0, 'name' :  'fault_panel_open'     },
            t_16_Bit_Options.BIT_FAULT_HIGH_VOLTAGE      : {'value' : 0, 'name' :  'fault_high_voltage'   },
            t_16_Bit_Options.BIT_FAULT_RAM_CRC           : {'value' : 0, 'name' :  'fault_ram_crc'        },
            t_16_Bit_Options.BIT_FAULT_EEPROM_CRC        : {'value' : 0, 'name' :  'fault_eeprom_crc'     },
            t_16_Bit_Options.BIT_FAULT_GPIO_ERROR        : {'value' : 0, 'name' :  'fault_gpio_error'     },
            t_16_Bit_Options.BIT_FAULT_LTFAULT_ERROR     : {'value' : 0, 'name' :  'fault_ltfault_error'  },
            t_16_Bit_Options.BIT_FAULT_TRIGGER_ERROR     : {'value' : 0, 'name' :  'fault_trigger_error'  },
            t_16_Bit_Options.BIT_FAULT_HARDWARE_EXC      : {'value' : 0, 'name' :  'fault_hardware_exc'   },
            t_16_Bit_Options.BIT_FAULT_TRIGGER_GLITCH    : {'value' : 0, 'name' :  'fault_trigger_glitch' },
            t_16_Bit_Options.BIT_FAULT_OVERVOLTAGE       : {'value' : 0, 'name' :  'fault_overvoltage'    },
            t_16_Bit_Options.BIT_FAULT_TEMP_SENSOR       : {'value' : 0, 'name' :  'fault_temp_sensor'    }
        }

    def set_value(self, item, value):
        """
        Set the value. (And calls the base class)

        This will also check for Options to set the bools.

        - FAULTS_ACTIVE
        - FAULTS_CURRENT

        >>> BIT_FAULT_PROBE            = 0
        >>> BIT_FAULT_OVERTEMP         = 1
        >>> BIT_FAULT_PANEL_OPEN       = 2
        >>> BIT_FAULT_HIGH_VOLTAGE     = 3
        >>> BIT_FAULT_RAM_CRC          = 4
        >>> BIT_FAULT_EEPROM_CRC       = 5
        >>> BIT_FAULT_GPIO_ERROR       = 6
        >>> BIT_FAULT_LTFAULT_ERROR    = 7
        >>> BIT_FAULT_TRIGGER_ERROR    = 8
        >>> BIT_FAULT_HARDWARE_EXC     = 9
        >>> BIT_FAULT_TRIGGER_GLITCH   = 10
        >>> BIT_FAULT_OVERVOLTAGE      = 11
        >>> BIT_FAULT_TEMP_SENSOR      = 12

        :param item:  The item to set

        :param value: The value to set

        """
        super(t_16_Bit_Options, self).set_value(item, value)

        if(item == t_16_Bit_Options.FAULT_ACTIVE):
            self.set_bools(value, self.faults_current, t_16_Bit_Options.BIT_FAULT_MAX )

        if(item == t_16_Bit_Options.FAULT_LATCHED):
            self.set_bools(value, self.faults_latched, t_16_Bit_Options.BIT_FAULT_MAX )

class t_8_Bit_Options(Option_group):
    """
    All the 8 bit options:

    >>> BOOLEAN_CONFIG_1           = 0
    >>> BOOTBITS                   = 1
    >>> ABSENTTEMP                 = 2

    """
    BOOLEAN_CONFIG_1           = 0
    BOOTBITS                   = 1
    ABSENTTEMP                 = 2
    MAX                        = 3

    BIT_PROBE_TERMINATION      = 0
    BIT_TMODE                  = 1
    BIT_EMODE                  = 2
    BIT_MUTE                   = 3
    BIT_PATTERN_TRIGGER        = 4
    BIT_DEBUG_REALTIME         = 5
    BIT_DEBUGPRINT             = 6
    BIT_DEBUG_HW_OVERRIDE      = 7
    BIT_MAX                    = 8

    def __init__(self):
        super(t_8_Bit_Options, self).__init__()

        self.options = {
            t_8_Bit_Options.BOOLEAN_CONFIG_1           : {'value' : 0, 'name' : 'boolean_config_1' },
            t_8_Bit_Options.BOOTBITS                   : {'value' : 0, 'name' : 'bootbits'         },
            t_8_Bit_Options.ABSENTTEMP                 : {'value' : 0, 'name' : 'absenttemp'       }
        }
        self.bools = {
            t_8_Bit_Options.BIT_PROBE_TERMINATION      : {'value' : 0, 'name' :  'probe_termination'},
            t_8_Bit_Options.BIT_TMODE                  : {'value' : 0, 'name' :  'tmode'            },
            t_8_Bit_Options.BIT_EMODE                  : {'value' : 0, 'name' :  'emode'            },
            t_8_Bit_Options.BIT_MUTE                   : {'value' : 0, 'name' :  'mute'             },
            t_8_Bit_Options.BIT_PATTERN_TRIGGER        : {'value' : 0, 'name' :  'pattern_trigger'  },
            t_8_Bit_Options.BIT_DEBUG_REALTIME         : {'value' : 0, 'name' :  'debug_realtime'   },
            t_8_Bit_Options.BIT_DEBUGPRINT             : {'value' : 0, 'name' :  'debugprint'       },
            t_8_Bit_Options.BIT_DEBUG_HW_OVERRIDE      : {'value' : 0, 'name' :  'debug_hw_override'}
        }

    def set_value(self, item, value):
        """
        Set the value. (And calls the base class)

        This will also check for Options to set the bools.

        - BOOLEAN_CONFIG_1

        >>> BIT_PROBE_TERMINATION      = 0
        >>> BIT_TMODE                  = 1
        >>> BIT_EMODE                  = 2
        >>> BIT_MUTE                   = 3
        >>> BIT_PATTERN_TRIGGER        = 4
        >>> BIT_DEBUG_REALTIME         = 5
        >>> BIT_DEBUGPRINT             = 6
        >>> BIT_DEBUG_HW_OVERRIDE      = 7

        :param item:  The item to set

        :param value: The value to set

        """
        super(t_8_Bit_Options, self).set_value(item, value)

        if(item == t_8_Bit_Options.BOOLEAN_CONFIG_1):
            self.set_bools(value, self.bools, t_8_Bit_Options.BIT_MAX)

class t_var_size_Options(Option_group):
    """
    All the var bit options:

    >>> BOARD_ID            = 0
    >>> CURRENT_STATE       = 1
    >>> PATTERN_WAVE        = 2
    >>> PATTERN_WAVE_APPEND = 3

    **Types:**

    - LONG_VAR_STRING = 0x57,
    - LONG_VAR_HEX    = 0x58

    >>>   +---- Type
    >>>  /  +---- Length
    >>>  |  |  +--+------Data
    >>> TT LL DD DD  ..

    """
    BOARD_ID            = 0
    CURRENT_STATE       = 1
    PATTERN_WAVE        = 2
    PATTERN_WAVE_APPEND = 3
    MAX                 = 4

    LONG_VAR_STRING = 0x57,
    LONG_VAR_HEX    = 0x58

    def __init__(self):
        """
        Calls base class and sets the options. 
        """
        super(t_var_size_Options, self).__init__()
        self.options = {
            t_var_size_Options.BOARD_ID        : {'value' : '', 'name' : 'board_id'           },
            t_var_size_Options.CURRENT_STATE   : {'value' : '', 'name' : 'state'              },
            t_var_size_Options.PATTERN_WAVE    : {'value' : '', 'name' : 'pat_wav'            }
        }

    def set_value(self, item, value):
        if len(value) == 0:
            return
        if(item == t_var_size_Options.PATTERN_WAVE):
            bits  = value[0]
            value = value[1:]


            # Get the value
            count = 0
            for x in value:
                for y in range(8):
                    if count >= bits:
                        return
                    if x & (0x80 >> y):
                        self.options[t_var_size_Options.PATTERN_WAVE]['value'] += '1'
                    else:
                        self.options[t_var_size_Options.PATTERN_WAVE]['value'] += '0'
                    count += 1

#            print hexlify(value)
        else:
            super(t_var_size_Options, self).set_value(item, value)


class BP_TOOL(Connection):
    UINT16S        = 0   # Number of bytes to represent the uint16's
    UINT8S         = 1   # Number of bytes to represent the uint8's
    VARS           = 2   # Number of bytes to represent the variables lenth's
    MAX            = 3
    OVERHEAD       = 3   # UINT16S + UINT8S + VARS
    SIZE_FOLLOW    = 1   # UINT16S + UINT8S + VARS
    SIZE_LEN       = 1   # UINT16S + UINT8S + VARS
    REQUEST_16     = 0
    REQUEST_8      = 1
    REQUEST_VAR    = 2
    SHOUTER_CMD    = 3

    # Commands
    DISARM       = 0
    ARM          = 1
    DEFAULT      = 2
    RESET        = 3
    CLEAR_FAULTS = 4
    PULSE        = 5
    TRIGGER_SAFE = 6
    IS_RESET     = 7
    EEPROM_WRITE = 8
    TIMEOUT      = 0xfc
    ACK          = 0x15 
    NACK         = 0xff 

    def __init__(self, comport, logging):
        super(BP_TOOL, self).__init__(comport, logging)
        self.config_16   = t_16_Bit_Options()
        self.config_8    = t_8_Bit_Options()
        self.config_var  = t_var_size_Options()
        # From https://stackoverflow.com/a/25259157
        def _crc_init(c):
            crc = 0
            c = c << 8
            for j in range(8):
                if (crc ^ c) & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                c = c << 1
            return crc
        self._crc_tab = [ _crc_init(i) for i in range(256) ]

    def get_follow(self, data):
        return data[BP_TOOL.OVERHEAD + data[BP_TOOL.UINT16S] + data[BP_TOOL.UINT8S] + data[BP_TOOL.VARS]]

    def get_length(self, data):
        return data[BP_TOOL.OVERHEAD + data[BP_TOOL.UINT16S] + data[BP_TOOL.UINT8S] + data[BP_TOOL.VARS] + BP_TOOL.SIZE_FOLLOW]

    def get_follow_and_length(self, data):
        return BP_TOOL.OVERHEAD + data[BP_TOOL.UINT16S] + data[BP_TOOL.UINT8S] + data[BP_TOOL.VARS]

    def get_data_start(self, data):
        return BP_TOOL.OVERHEAD + data[BP_TOOL.UINT16S] + data[BP_TOOL.UINT8S] + data[BP_TOOL.VARS]  + BP_TOOL.SIZE_FOLLOW + BP_TOOL.SIZE_LEN

    def get_options_requested(self, data):
        options = []
        option_number = 0
        for x in data:
            # Check the bits
            for y in range(8):
                if x & 1 << y:
                    options.append(option_number)
                option_number += 1
        return options

    def set_options_requested(self, options):
        max_value = max(options)
        bit_array = bytearray()
        bits_array_length = max_value // 8 + 1

        for x in range(bits_array_length):
            bit_array.append(0)

        for x in options:
            # set the bits
            index = x//8
            bit   = x % 8 
            bit_array[index] |= 1 << bit 

        return bit_array

    #--------------------------------------------------------------------------
    # 16 bits
    def get_16bit_options_bits(self, data):
        return data[BP_TOOL.OVERHEAD : BP_TOOL.OVERHEAD + data[BP_TOOL.UINT16S]]

    def get_16bit_options(self, data):
        start = self.get_data_start(data)
        bits  = self.get_16bit_options_bits(data)
        end   = start + (len(self.get_options_requested(bits)) * 2)
        return data[start : end]

    #--------------------------------------------------------------------------
    # 8 bits
    def get_8bit_options_bits(self, data):
        start = BP_TOOL.OVERHEAD + data[BP_TOOL.UINT16S]
        end   = start + data[BP_TOOL.UINT8S]
        return data[start:end]

    def get_8bit_options(self, data):
        # After the 16 bit options.
        start = self.get_data_start(data)
        bits = self.get_16bit_options_bits(data)
        end = start + (len(self.get_options_requested(bits)) * 2)

        start = end
        bits = self.get_8bit_options_bits(data)
        end = start + len(self.get_options_requested(bits))
        return data[start:end]

    #--------------------------------------------------------------------------
    # 8 bits
    def get_var_options_bits(self, data):
        start = BP_TOOL.OVERHEAD + data[BP_TOOL.UINT16S] + data[BP_TOOL.UINT8S]
        end   = start + data[BP_TOOL.VARS]
        return data[start:end]

    def get_var_options(self, data):
        # After the 16 bit options.
        start = self.get_data_start(data)
        bits  = self.get_16bit_options_bits(data)
        end   = start + (len(self.get_options_requested(bits)) * 2)
        start = end

        # After the 8 bit options.
        bits  = self.get_8bit_options_bits(data)
        end   = start + (len(self.get_options_requested(bits)))
        start = end

        #----------------------------------------------------------------------
        bits           = self.get_var_options_bits(data)
        vars_requested = len(self.get_options_requested(bits))
        options        = self.get_options_requested(bits)

        rval = []
        for x in range(vars_requested):
            s = data[start + 2 : start + 2 + data[start + 1]]
#            print 'PAT = ' + hexlify(s)
            opt = options[x]
            rval.append({'option' : opt, 'type': data[start], 'length' : data[start + 1], 'value' : s})
#            pprint.pprint(rval)
            start = start + 2 + data[start + 1]

        return rval

    def build_command_packet(self, command):
        """
        Builds a command packet
        """
        packet = bytearray()
        # All option fields are 0
        packet.append(0)
        packet.append(0)
        packet.append(0)
        packet.append(command)
        return packet

    def build_request(self, options, req_type):
        packet = bytearray()

        bit_array = self.set_options_requested(options)
        len_bits = len(bit_array)  

        packet.append(0)
        packet.append(0)
        packet.append(0)
        packet[req_type] = len_bits

        packet += bit_array
        packet.append(0)  # No Following packets
        packet.append(0) # Number of options to follow

        return packet

    def build_request_all(self):
        packet = bytearray()
        packet.append(int(t_16_Bit_Options.MAX // 8 + 1))
        packet.append(int(t_8_Bit_Options.MAX // 8 + 1))
        packet.append(int(t_var_size_Options.MAX // 8 + 1))

        # ---------------------------------------------------------------------
        # Request all the 16 bit options.
        packet.append(
            0x01 << t_16_Bit_Options.VOLTAGE             |
            0x01 << t_16_Bit_Options.VOLTAGE_MEASURED    |
            0x01 << t_16_Bit_Options.PULSE_WIDTH         |
            0x01 << t_16_Bit_Options.PULSE_REPEAT        |
            0x01 << t_16_Bit_Options.PULSE_DEADTIME      |
            0x01 << t_16_Bit_Options.ARM_TIMEOUT         |
            0x01 << t_16_Bit_Options.FAULT_ACTIVE        |
            0x01 << t_16_Bit_Options.FAULT_LATCHED
        )
        # ---------------------------------------------------------------------
        # Request all the rest of the 16 bit options.
        packet.append(
            0x01 << (t_16_Bit_Options.TEMPERATURE_MOSFET   - 8) |
            0x01 << (t_16_Bit_Options.TEMPERATURE_XFORMER  - 8) |
            0x01 << (t_16_Bit_Options.TEMPERATURE_DIODE    - 8) |
            0x01 << (t_16_Bit_Options.MEASURED_PULSE_WIDTH - 8)
        )

        # ---------------------------------------------------------------------
        # Request all the 8 bit options.
        packet.append(
            0x01 << t_8_Bit_Options.BOOLEAN_CONFIG_1  |
            0x01 << t_8_Bit_Options.BOOTBITS          |
            0x01 << t_8_Bit_Options.ABSENTTEMP
        )
        # ---------------------------------------------------------------------
        # Request all the variable length options.
        packet.append(
            0x01 << t_var_size_Options.BOARD_ID      |
            0x01 << t_var_size_Options.CURRENT_STATE
        )
        # ---------------------------------------------------------------------
        # Packets to follow
        packet.append(0)
        # ---------------------------------------------------------------------
        # Length of the bytes to follow
        packet.append(0)

        return packet

    def add_crc(self, packet):
        # Add the CRC.
        crc = 0x1d0f
        for c in packet:
            crc = ((crc << 8) ^ self._crc_tab[(crc >> 8) ^ c]) & 0xFFFF
        packet.append(crc >> 8)
        packet.append(crc & 0xFF)
        return packet

    def build_set_option(self, options, req_type, values):
        if len(values) != len(options):
            return
        
        packet = bytearray()

        bit_array = self.set_options_requested(options)
        len_bits = len(bit_array)  

        packet.append(0)
        packet.append(0)
        packet.append(0)
        packet[req_type] = len_bits

        packet += bit_array
        packet.append(0)  # No Following packets
        packet.append(1) # Number of options to follow

        for x in values:
            if req_type == BP_TOOL.REQUEST_16:
                set_value = (x >> 8) & 0x00ff
                packet.append(set_value)
                set_value = (x & 0x00ff)
                packet.append(set_value)
            else:
                packet.append(x)

        return packet

    def __show_protocol__(self, data):
        """

        Show the protocol that was received.

        :param data (bytearray): The received bytes.

        """
        t_16   = t_16_Bit_Options()
        t_8    = t_8_Bit_Options()
        t_var  = t_8_Bit_Options()
        print('Received ' + str(len(data)) + ' Bytest')

        #----------------------------------------------------------------------
        print('='*80)
        print('Handling Protocol response: ' + hexlify(data))
        #----------------------------------------------------------------------
        print('='*80)
        print('Overhead Bytes: ' + hexlify(data[:BP_TOOL.OVERHEAD]))
        print('Number of UINT16 bitstream data = ' + str(data[BP_TOOL.UINT16S]))
        print('Number of UINT8  bitstream data = ' + str(data[BP_TOOL.UINT8S]))
        print('Number of var    bitstream data = ' + str(data[BP_TOOL.VARS]))
        print('Follow = ' + str(self.get_follow(data)))
        print('Length = ' + str(self.get_length(data)))
        start = self.get_follow_and_length(data)
        end   = start + BP_TOOL.SIZE_FOLLOW + BP_TOOL.SIZE_LEN
        print('Following bytes and length      = ' + hexlify(data[start:end]))
        #----------------------------------------------------------------------
        print('='*80)
        bits = self.get_16bit_options_bits(data)
        values = self.get_16bit_options(data)
        options = self.get_options_requested(bits)

        # Display the options if exist
        if len(options):
            print('UINT16 bits...... : ' + hexlify(bits))
            print('UINT16 data...... : ' + hexlify(values))
            print('UINT16 Num of opts ... : ' + str(len(values) // 2))
            print('UINT16 options... : ' + str(options))
            print('-'*80)
            for x in range(len(options)):
                value = (values[x*2] << 8) | (values[x*2 + 1])
                opt = options[x]
                t_16.set_value(opt, value)
                print('Option: ' + t_16.options[opt]['name'] + ' ' + str(value))
            pprint.pprint(t_16.options)
        else:
            print('No 16 bit options')

        #----------------------------------------------------------------------
        print('-'*80)
        bits = self.get_8bit_options_bits(data)
        values = self.get_8bit_options(data)
        options = self.get_options_requested(bits)
        # Display the options if exist
        if len(options):
            print('UINT8 bits...... : ' + hexlify(bits))
            print('UINT8 data...... : ' + hexlify(values))
            print('UINT8 options... : ' + str(options))
            print('-'*80)
            for x in range(len(options)):
                value = values[x]
                opt = options[x]
                t_8.set_value(opt, value)
                print('Option: ' + t_8.options[x]['name'] + ' ' + str(value))
            pprint.pprint(t_8.options)
        else:
            print('No 8 bit options')

        #----------------------------------------------------------------------
        print('-'*80)
        bits = self.get_var_options_bits(data)
        values = self.get_var_options(data)
        print('VARS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        # Display the options if exist
        if len(values):
            pprint.pprint(values)
        else:
            print('No var bit options')

        print('VAR options... : ' + str(self.get_options_requested(bits)))
        print('VARS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        print('-'*80)

    def display_options(self):
        print('Options')
        print('='*80)
        print('16 bit options')
        print('='*80)
        pprint.pprint(self.config_16.options)
        print('Current Faults')
        print('='*80)
        pprint.pprint(self.config_16.faults_current)
        print('Latched Faults')
        print('='*80)
        pprint.pprint(self.config_16.faults_latched)
        print('8 bit options')
        print('='*80)
        pprint.pprint(self.config_8.options)
        print('bool options')
        print('='*80)
        pprint.pprint(self.config_8.bools)
        print('-'*80)

class Protocol(BP_TOOL):
    """
    Base class for the Commands
    """

    def __init__(self, comport, logging):
        super(Protocol, self).__init__(comport, logging)
        self.to_follow = 0
        self.validcommands = [
            BP_TOOL.ARM,         
            BP_TOOL.EEPROM_WRITE,
            BP_TOOL.DISARM,      
            BP_TOOL.DEFAULT,     
            BP_TOOL.RESET,       
            BP_TOOL.CLEAR_FAULTS,
            BP_TOOL.ACK,         
            BP_TOOL.NACK         
        ]
        pass

    #---------------------------------------------------------------------------
    def __packet_unstuff(self, data):
        """ remove the special characters for stuffing
        """
        unstuffed = bytearray()
        escape = False
        for count in data:
            if escape == False:
                if count == 0x7e:
                    continue
                if count == 0x7f:
                    continue
                if count == 0x7d:
                    escape = True
                else:
                    unstuffed.append(count)
            else:
                unstuffed.append(count + 0x7d)
                escape = False

        return(unstuffed)

    #---------------------------------------------------------------------------
    def __packet_stuff(self, data):
        """ inserts the special characters for stuffing
        """
        stuffed = bytearray()
        stuffed.append(0x7e)
        for count in data:
            if count >= 0x7d and count <= 0x7f:
                stuffed.append(0x7d)
                stuffed.append(count - 0x7d)
            else:
                stuffed.append(count)
        stuffed.append(0x7f)
        return(stuffed)

    #--------------------------------------------------------------------------
    # vars bits
    def parse_protocol(self, data):
        start = self.get_follow_and_length(data)
        if start == 0:
            return
        end   = start + BP_TOOL.SIZE_FOLLOW + BP_TOOL.SIZE_LEN
        self.to_follow = data[BP_TOOL.OVERHEAD + data[BP_TOOL.UINT16S] + data[BP_TOOL.UINT8S] + data[BP_TOOL.VARS]]
        
        # Is it a command ??
        if data[BP_TOOL.UINT16S] == 0  and data[BP_TOOL.UINT8S] == 0 and data[BP_TOOL.VARS] == 0:
            return

        bits = self.get_16bit_options_bits(data)
        values = self.get_16bit_options(data)
        options = self.get_options_requested(bits)

        # 16 Bit options.
        for x in range(len(options)):
            value = (values[x*2] << 8) | (values[x*2 + 1])
            opt = options[x]
            self.config_16.set_value(opt, value)

        # 8 Bit options.
        bits    = self.get_8bit_options_bits(data)
        values  = self.get_8bit_options(data)
        options = self.get_options_requested(bits)
        for x in range(len(options)):
            value = values[x]
            opt = options[x]
            self.config_8.set_value(opt, value)

        # Variable len options.
        values = self.get_var_options(data)

        # Set the vaules received
        for x in values:
            self.config_var.set_value(x['option'], x['value'])

    def __get_received_packet(self,data):
        '''
        Get the received packet.

        :param data:

        :return : data packet if it is good, False if it is bad

        '''
        # Unpack the data.
        data = self.__packet_unstuff(data)
        # Verify that the packet is good.
        crc = 0x1d0f
        for c in data:
            crc = ((crc << 8) ^ self._crc_tab[(crc >> 8) ^ c]) & 0xFFFF

        # Validate and return
        if crc == 0:
            if self.__is_nack(data[:-2]):
                return False
            return data
        return False

    def __send_with_retries(self, data, retries = 5):
        # Retry up to 5 times.
        #for retry in range(retries):
        while retries:
            retries -= 1
            self.s_write(data)
            r = self.s_read()

            if len(r) == 0:
                if not retries:
                    raise IOError('No response from shouter.') 
                continue

            # Handle response will set the dict with proper values.
            rval = self.__get_received_packet(r)
            if rval != False:
                return rval
        raise Max_Retry_Exception('Max NACK reached!') 

    def interact_with_shouter(self, data):
        '''

        Ask the shouter for a response to the command
        will retry up to 5 times

        :Returns (bytearray): data that it has responded with.
                              Empty array if nothing.

        '''
        data = self.add_crc(data)
        data = self.__packet_stuff(data)
        # Add the CRC.
        packet = bytearray()
        packet.extend(data)

        rval = self.__send_with_retries(data)
        return rval

    def refresh(self):
        val = self.build_request_all()
        r   = self.interact_with_shouter(val)
        self.parse_protocol(r)

    def __show_data(self, data, ask = True):
        if ask:
            print('Requesting: ' + ' ' + hexlify(data))
        print('BP_TOOL.OVERHEAD) ' + str(BP_TOOL.OVERHEAD))
        print('[BP_TOOL.UINT16S] ' + str(data[BP_TOOL.UINT16S]))
        print('[BP_TOOL.UINT8S]) ' + str(data[BP_TOOL.UINT8S]))
        print('[BP_TOOL.VARS     ' + str(data[BP_TOOL.VARS]))
        pass

    def __is_nack(self, data):
        if len(data) == 4:
            if data[0] == 0 and data[1] == 0 and data[2] == 0 and data[3] == 0xFF:
                return True
        return False

    def get_option_from_shouter(self, options, req_type):

        val = self.build_request(options, req_type)
        r   = self.interact_with_shouter(val)
        self.parse_protocol(r)

        # Handle the returning of a list.
        if req_type == BP_TOOL.REQUEST_16:
            updated_settings = self.config_16.options
            pass
        elif req_type == BP_TOOL.REQUEST_8:
            updated_settings = self.config_8.options
            pass
        elif req_type == BP_TOOL.REQUEST_VAR:
            updated_settings = self.config_var.options
            pass
        else:
            return

        rval = []
        for x in options:
            rval.append(updated_settings[x]['value'])
        return rval

    def send_command_to_shouter(self, command):
        val = self.build_command_packet(command)
        r   = self.interact_with_shouter(val)
        response = r[BP_TOOL.OVERHEAD]
        return response

    def set_option_on_shouter(self, options, req_type, values):
        val = self.build_set_option(options, req_type, values)
        r   = self.interact_with_shouter(val)
        self.parse_protocol(r)
        return r

class Bin_API(Protocol):
    """

    >>>  | Position                | Size   | Name               | Description                |
    >>>  | :-------                | :----- | :---------------   | :------                    |
    >>>  | 0                       | 1      | UNIT16's           | Number of UINT16 bitfields |
    >>>  | 1                       | 1      | UNIT8's            | Number of UINT8 bitfields  |
    >>>  | 2                       | 1      | Variable lengths   | Number of UINT8 bitfields  |
    >>>  | 3                       | n0     | UINT16 bitfields   | Bits to come in UINT16's   |
    >>>  | 3 + n                   | n1     | UINT8 bitfields    | Bits to come in UINT8's    |
    >>>  | 3 + (n0 ... n1)         | n2     | Variable bitfields | Bits to come in variables  |
    >>>  |                         | 1      | Packets to follow  |                            |
    >>>  |                         | 1      | Length             |                            |
    >>>  | 3 + 1 + 1 + (n0 ... n2) | n3     | UINT16's options   | UINT16 options             |
    >>>  | 3 + 1 + 1 + (n0 ... n3) | n4     | UINT8's options    | UINT16 options             |
    >>>  | 3 + 1 + 1 + (n0 ... n4) | n5     | Variable options   | UINT16 options             |
    >>>  |                         |        |                    |                            |

    """

    def __init__(self, comport, logging = False):
        """
        Init will connect to the com port of the shouter.
        """
        super(Bin_API, self).__init__(comport, logging)

    def cmd_default_options(self, timeout = 0):
        self.send_command_to_shouter(BP_TOOL.DEFAULT)

    def cmd_disarm(self, timeout = 0):
        self.send_command_to_shouter(BP_TOOL.DISARM)

    def cmd_arm(self, timeout = 0):
        rval = self.send_command_to_shouter(BP_TOOL.ARM)
        if rval != BP_TOOL.ARM:
            raise Firmware_State_Exception("State:" + self.get_state() + str(self.get_faults_latched()))
    def cmd_eeprom_write(self, timeout=0):
        self.send_command_to_shouter(BP_TOOL.EEPROM_WRITE)
    
    def cmd_pulse(self, timeout = 0):
        self.send_command_to_shouter(BP_TOOL.PULSE)

    def cmd_clear_faults(self, timeout = 0):
        self.send_command_to_shouter(BP_TOOL.CLEAR_FAULTS)

    def cmd_clear_arm(self, timeout = 0):
        self.send_command_to_shouter(BP_TOOL.CLEAR_FAULTS)
        self.cmd_arm()

    def ready_for_commands(self, retries = 3):
        """
        ready_for_commands is a function to wait for the firmware to be ready to communicate
        """
        while retries:
            try:
                self.refresh()
                return True
            except Reset_Exception as e:
                pass
            except Max_Retry_Exception as e:
                pass
            finally:
                retries -= 1
        raise e

    def cmd_reset(self, timeout = 0):
        self.send_command_to_shouter(BP_TOOL.RESET)
        time.sleep(5)
        return self.ready_for_commands()

    def get_board_id(self, timeout = 0):
        """ This will get the board id. 

        :returns (String): name of the board programmed at time of assembly. 
        """
        self.get_option_from_shouter([t_var_size_Options.BOARD_ID], BP_TOOL.REQUEST_VAR)
        return str(self.config_var.options[t_var_size_Options.BOARD_ID]['value'].decode("ASCII"))

    def get_id(self, timeout = 0):
        return self.get_board_id()

    def get_state(self, timeout = 0):
        """ This will string representing the current state of the system.. 

        :returns (String): string of the state.
        """
        self.get_option_from_shouter([t_var_size_Options.CURRENT_STATE], BP_TOOL.REQUEST_VAR)
        rval = str(self.config_var.options[t_var_size_Options.CURRENT_STATE]['value'].decode("ASCII"))
        return rval

    def get_is_reset(self, timeout = 0):
        """ This will ask if the shouter has reset or not. 

        :returns (bool): True if the shouter has reset.

        """
        response = self.send_command_to_shouter(BP_TOOL.IS_RESET)
        if response == BP_TOOL.ACK:
            return  False
        elif response == BP_TOOL.IS_RESET:
            return True
        else:
            return False

    def get_trigger_safe(self, timeout = 0):
        response = self.send_command_to_shouter(BP_TOOL.TRIGGER_SAFE)
        if response == BP_TOOL.TRIGGER_SAFE:
            return True
        else:
            return False

    def __get_faults_list(self, faults):
        """ This will get the current faults on the system.

        :param faults (list): Reference to the faults_current or faults_latched
        :returns (list): List of fault names that exist currently
        """
        r_faults = []
        for x in faults:
            if faults[x]['value']:
                r_faults.append(faults[x]['name'])
        return r_faults

    def get_faults_current(self):
        """ This will get the current faults on the system.

        :returns (list): List of fault names that exist currently
        """
        request = self.get_option_from_shouter([t_16_Bit_Options.FAULT_ACTIVE], BP_TOOL.REQUEST_16)
        return self.__get_faults_list(self.config_16.faults_current)

    def get_faults_latched(self):
        """ This will get the latched faults on the system.

        :returns (list): List of fault names that are latched.
        """
        request = self.get_option_from_shouter([t_16_Bit_Options.FAULT_LATCHED], BP_TOOL.REQUEST_16)
        return self.__get_faults_list(self.config_16.faults_latched)

    def get_temperature_mosfet(self, timeout = 0):
        self.get_option_from_shouter([t_16_Bit_Options.TEMPERATURE_MOSFET], BP_TOOL.REQUEST_16)
        return self.config_16.options[t_16_Bit_Options.TEMPERATURE_MOSFET]['value']
    def get_temperature_diode(self, timeout = 0):
        self.get_option_from_shouter([t_16_Bit_Options.TEMPERATURE_DIODE], BP_TOOL.REQUEST_16)
        return self.config_16.options[t_16_Bit_Options.TEMPERATURE_DIODE]['value']
    def get_temperature_xformer(self, timeout = 0):
        self.get_option_from_shouter([t_16_Bit_Options.TEMPERATURE_XFORMER], BP_TOOL.REQUEST_16)
        return self.config_16.options[t_16_Bit_Options.TEMPERATURE_XFORMER]['value']

    def get_voltage_measured(self, timeout = 0):
        self.get_option_from_shouter([t_16_Bit_Options.VOLTAGE_MEASURED], BP_TOOL.REQUEST_16)
        return self.config_16.options[t_16_Bit_Options.VOLTAGE_MEASURED]['value']

    def get_pulse_width_measured(self, timeout = 0):
        return self.config_16.options[t_16_Bit_Options.MEASURED_PULSE_WIDTH]['value']

    def get_arm_timeout(self, timeout = 0):
        """ Arm timeout 
        Because of safty reasons we will monitor the trigger when in the armed
        state. The trigger needs to be activated within the timeout programmed.
        Every trigger will reset the timer with every pulse.

        If the time out occurs, the shouter will disarm and disable the high
        voltage circuitry.

        The Arm timer will be reset on every trigger / internal or external.

        **NOTE** Valid range is between 1 - 60 minutes

        :param state:   (int): time in minutes for the timeout. 
        :param timeout: (int): time in seconds to wait for response. 

        :Returns (int): The arm timeout without triggering. 

        """
        self.get_option_from_shouter([t_16_Bit_Options.ARM_TIMEOUT], BP_TOOL.REQUEST_16)
        return self.config_16.options[t_16_Bit_Options.ARM_TIMEOUT]['value']

    def set_arm_timeout(self, value, timeout = 0):
        packet = self.set_option_on_shouter([t_16_Bit_Options.ARM_TIMEOUT], BP_TOOL.REQUEST_16, [value])

    def get_voltage(self, timeout = 0):
        self.get_option_from_shouter([t_16_Bit_Options.VOLTAGE], BP_TOOL.REQUEST_16)
        return self.config_16.options[t_16_Bit_Options.VOLTAGE]['value']

    def set_voltage(self, value, timeout = 0):
        packet = self.set_option_on_shouter([t_16_Bit_Options.VOLTAGE], BP_TOOL.REQUEST_16, [value])

    def get_pulse_width(self, timeout = 0):
        self.get_option_from_shouter([t_16_Bit_Options.PULSE_WIDTH], BP_TOOL.REQUEST_16)
        return self.config_16.options[t_16_Bit_Options.PULSE_WIDTH]['value']

    def set_pulse_width(self, value, timeout = 0):
        packet = self.set_option_on_shouter([t_16_Bit_Options.PULSE_WIDTH], BP_TOOL.REQUEST_16, [value])

    def get_pulse_repeat(self, timeout = 0):
        self.get_option_from_shouter([t_16_Bit_Options.PULSE_REPEAT], BP_TOOL.REQUEST_16)
        return self.config_16.options[t_16_Bit_Options.PULSE_REPEAT]['value']

    def set_pulse_repeat(self, value, timeout = 0):
        packet = self.set_option_on_shouter([t_16_Bit_Options.PULSE_REPEAT], BP_TOOL.REQUEST_16, [value])

    def get_pulse_deadtime(self, timeout = 0):
        self.get_option_from_shouter([t_16_Bit_Options.PULSE_DEADTIME], BP_TOOL.REQUEST_16)
        return self.config_16.options[t_16_Bit_Options.PULSE_DEADTIME]['value']

    def set_pulse_deadtime(self, value, timeout = 0):
        packet = self.set_option_on_shouter([t_16_Bit_Options.PULSE_DEADTIME], BP_TOOL.REQUEST_16, [value])

    def __set_8_bool(self, bit, value):
        # Get the bool
        request = self.get_option_from_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8)
        self.config_8.bools[bit]['value'] = value;
        result = self.config_8.get_bools_array(self.config_8.bools, 8)
        send = self.config_8.get_bools_array(self.config_8.bools, 8)
        value = send[0]
        packet = self.set_option_on_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8, [value])
        return

    def get_hwtrig_term(self, timeout = 0):
        request = self.get_option_from_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8)
        return self.config_8.bools[t_8_Bit_Options.BIT_PROBE_TERMINATION]['value']
    def set_hwtrig_term(self, value, timeout = 0):
        request = self.get_option_from_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8)
        self.__set_8_bool(t_8_Bit_Options.BIT_PROBE_TERMINATION, value)
    def get_hwtrig_mode(self, timeout = 0):
        request = self.get_option_from_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8)
        return self.config_8.bools[t_8_Bit_Options.BIT_TMODE]['value']
    def set_hwtrig_mode(self, value, timeout = 0):
        request = self.get_option_from_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8)
        self.__set_8_bool(t_8_Bit_Options.BIT_TMODE, value)
    def get_emode(self, timeout = 0):
        request = self.get_option_from_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8)
        return self.config_8.bools[t_8_Bit_Options.BIT_EMODE]['value']
    def set_emode(self, value, timeout = 0):
        request = self.get_option_from_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8)
        self.__set_8_bool(t_8_Bit_Options.BIT_EMODE, value)
    def get_mute(self, timeout = 0):
        request = self.get_option_from_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8)
        return self.config_8.bools[t_8_Bit_Options.BIT_MUTE]['value']
    def set_mute(self, value, timeout = 0):
        request = self.get_option_from_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8)
        self.__set_8_bool(t_8_Bit_Options.BIT_MUTE, value)
    def get_pat_length(self, timeout = 0):
        request = self.get_option_from_shouter([t_16_Bit_Options.PAT_WAVE_LENGTH], BP_TOOL.REQUEST_16)
        return self.config_16.options[t_16_Bit_Options.PAT_WAVE_LENGTH]['value']
    def get_pat_enable(self, timeout = 0):
        request = self.get_option_from_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8)
        return self.config_8.bools[t_8_Bit_Options.BIT_PATTERN_TRIGGER]['value']
    def set_pat_enable(self, value, timeout = 0):
        request = self.get_option_from_shouter([t_8_Bit_Options.BOOLEAN_CONFIG_1], BP_TOOL.REQUEST_8)
        self.__set_8_bool(t_8_Bit_Options.BIT_PATTERN_TRIGGER, value)


    def get_absentTemp(self, timeout = 0):
        self.get_option_from_shouter([t_8_Bit_Options.ABSENTTEMP], BP_TOOL.REQUEST_8)
        return self.config_8.options[t_8_Bit_Options.ABSENTTEMP]['value']

    def set_absentTemp(self, value, timeout = 0):
        packet = self.set_option_on_shouter([t_8_Bit_Options.ABSENTTEMP], BP_TOOL.REQUEST_8, [value])

    def __request_pat_wave(self, r_number):
        """
        Gets the pattern wave
        # pat_wave  101011110011 ....

        >>> Request
        >>> 0------------------------>
        >>> Pattern Wave [More to follow]
        >>> <------------------------)
        >>> Request Next block
        >>> 0------------------------>
        >>> Pattern Wave [More to follow]
        >>> <------------------------)
        >>>
        >>> ..... 
        >>>
        >>> Request Next block
        >>> 0------------------------>
        >>> Pattern Wave [No More to follow]
        >>> <------------------------)

        """
        packet = bytearray()
        packet.append(0) # 16 bit options
        packet.append(0) #  8 bit options
        packet.append(1) # Request the 1 option

        # ---------------------------------------------------------------------
        # Request the variable length options. pattern wave.
        packet.append(0x01 << t_var_size_Options.PATTERN_WAVE)

        # ---------------------------------------------------------------------
        # Packets to follow
        packet.append(r_number)

        # ---------------------------------------------------------------------
        # Length of the bytes to follow
        packet.append(0)
        rval = self.interact_with_shouter(packet)
        if rval != False:
            return rval
        return []

    def get_pat_wave(self, timeout = 0, short_wave = False):
        self.config_var.options[t_var_size_Options.PATTERN_WAVE]['value'] = ''
        # Check the pat enable, it will also ensure that the count is reset
        # on the shouter side.
        pat_en = self.get_pat_enable()
        count = 0
        more_to_follow = 1
        self.to_follow = 0

        while more_to_follow:
            r = self.__request_pat_wave(self.to_follow)
            self.parse_protocol(r)
            count += 1
            more_to_follow = self.to_follow

            if short_wave:
                break

        return  self.config_var.options[t_var_size_Options.PATTERN_WAVE]['value']

    def set_pat_wave(self, value, timeout = 0):
        wave = value
        bit_array = bytearray()
        bits_array_length = (len(wave)) // 8

        if len(wave) % 8:
            bits_array_length += 1

        max_bytes_packet = 12
        max_bits_packet  = (max_bytes_packet * 8)

        for x in range(bits_array_length):
            bit_array.append(0)

        index = 0
        for x in wave: 
            if x == '1' or x == 1:
                bit_array[index // 8] |= (0x80 >> (index % 8))
                pass
            elif x == '0' or x == 0:
                pass
            else:
                raise ValueError('Only \'1\' an \'0\' allowed')
            index += 1

        count = 0
        total_bit_count = 0
        total_bits      = len(wave)
        total_bytes     = len(bit_array)

        for x in range(0, total_bytes, max_bytes_packet):
            send = bit_array[x:(x + max_bytes_packet)]
            send_bits_length = max_bits_packet

            if (((x // max_bytes_packet) + 1) * (max_bits_packet)) > len(wave):
                send_bits_length = len(wave) % (max_bits_packet)

            total_bit_count += send_bits_length

            packet = bytearray()
            packet.append(0) # 16 bit options
            packet.append(0) #  8 bit options
            packet.append(1) # Request the 1 option

            # ---------------------------------------------------------------------
            # Request the variable length options. pattern wave.
            if count:
                packet.append(0x01 << t_var_size_Options.PATTERN_WAVE_APPEND)
            else:
                packet.append(0x01 << t_var_size_Options.PATTERN_WAVE)
            count += 1

            # ---------------------------------------------------------------------
            # Packets to follow
            packet.append(0)
            packet.append(send_bits_length)
            packet += send

            self.interact_with_shouter(packet)

    def run_console(self):
        self.ctl_disconnect()
        serial = Serial_interface(use_threads = False)
        # Open the port
        if serial.s_open(self.comport, timeout = .01):
            print("Connected successful")
        else:
            print("Not connected")
            exit()
        my_console = Console(serial, threads = False)
        my_console.console()
        self.ctl_connect()
        print('Console Complete')

################################################################################
def main():
    """ This is the main entry for the console."""
    bp = Bin_API('COM10')
    print('Testing')

    bp.set_hwtrig_term(1)


################################################################################
if __name__ == "__main__":
    main()
