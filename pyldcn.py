import serial
import struct
import time

class pyldcn:
    cmds = {
        'SET_ADDR':     0x01,	#Set address and group address (2 bytes)
        'DEF_STAT':     0x02,	#Define status items to return (1 byte)
        'READ_STAT':    0x03,	#Read value of current status items
        'SET_BAUD':     0x0A, 	#Set the baud rate (1 byte)
        'NOP':          0x0E,	#No operation - returns prev. defined status (0 bytes)
        'HARD_RESET':   0x0F,	#RESET - no status is returned
    }
    
    ldcn_read_status = {
        'SEND_ID':		0x20,	#2 bytes
    }
    
    device_types = {
        'SERVOMODTYPE' : 0,
        'SERVOHYBTYPE' : 90,
        'IOMODTYPE'    : 2,
        'STEPMODTYPE'  : 3
    }
    
    stepper_cmds = {
        'RESET_POS':    0x00,	#Reset encoder counter to 0 (0 bytes)
        'SET_ADDR':     0x01,	#Set address and group address (2 bytes)
        'DEF_STAT':     0x02,	#Define status items to return (1 byte)
        'READ_STAT':    0x03,	#Read value of current status items
        'LOAD_TRAJ':    0x04,	#Load trajectory data
        'START_MOVE':   0x05,	#Start pre-loaded trajectory (0 bytes)
        'SET_PARAM':    0x06,  #Set operating parameters (6 bytes)
        'STOP_MOTOR':   0x07,	#Stop motor (1 byte)
        'SET_OUTPUTS':  0x08,	#Set output bits (1 byte)
        'SET_HOMING':   0x09,  #Define homing mode (1 byte)
        'SET_BAUD':     0x0A, 	#Set the baud rate (1 byte)
        'RESERVED':     0x0B,  #
        'SAVE_AS_HOME': 0x0C,	#Store the input bytes and timer val (0 bytes)
        'NOT_USED':     0x0D,
        'NOP':          0x0E,	#No operation - returns prev. defined status (0 bytes)
        'HARD_RESET':   0x0F,	#RESET - no status is returned
    }
        
    stepper_read_status = {
        'SEND_POS':		0x01,	#4 bytes data
        'SEND_AD':		0x02,	#1 byte
        'SEND_ST':		0x04,	#2 bytes
        'SEND_INBYTE':	0x08,	#1 byte
        'SEND_HOME':	0x10,	#4 bytes
        'SEND_ID':		0x20,	#2 bytes
        'SEND_OUT':		0x40,    #1 byte
        'SEND_ALL' :    0x7F    # sum
    }
    
    stepper_load_traj = {
        'LOAD_POS':		0x01,	#+4 bytes
        'LOAD_SPEED':	0x02,	#+1 bytes
        'LOAD_ACC':		0x04,	#+1 bytes
        'LOAD_ST':		0x08,	#+3 bytes
        'STEP_REV':		0x10,   #reverse dir
        'START_NOW':	0x80    #1 = start now, 0 = wait for START_MOVE command
    }
    
    stepper_set_param = {
        'SPEED_8X':			0x00,	#use 8x timing
        'SPEED_4X':			0x01,	#use 4x timing
        'SPEED_2X':			0x02,	#use 2x timing
        'SPEED_1X':			0x03,	#use 1x timing
        'IGNORE_LIMITS':	0x04,	#Do not stop automatically on limit switches
        'MOFF_LIMIT':		0x08,   #Turn Off Motor on Limit
        'MOFF_STOP':		0x10    #Turn Off motor on Stop 
    }
    
    stepper_stop_motor = {
        'STP_ENABLE_AMP':	0x01,	#1 = raise amp enable output, 0 = lower amp enable
        'STOP_ABRUPT':		0x04,	#set to stop motor immediately
        'STOP_SMOOTH':		0x08    #set to decellerate motor smoothly
    }
    
    stepper_set_homing = {
        'ON_LIMIT1':		0x01,	#home on change in limit 1
        'ON_LIMIT2':		0x02,	#home on change in limit 2
        'HOME_MOTOR_OFF':	0x04,   #turn motor off when homed
        'ON_HOMESW':		0x08,	#home on change in index
        'HOME_STOP_ABRUPT':	0x10,   #stop abruptly when homed
        'HOME_STOP_SMOOTH':	0x20    #stop smoothly when homed
    }
    
    stepper_status = {
        'MOTOR_MOVING':		0x01,	#Set when motor is moving
        'CKSUM_ERROR':		0x02,	#checksum error in received command
        'STP_AMP_ENABLED':	0x04,	#set if amplifier is enabled
        'POWER_ON':			0x08,	#set when motor power is on
        'AT_SPEED':			0x10,   #set if the commanded velocity is reached.
        'VEL_MODE':			0x20,	#set when in velocity profile mode
        'TRAP_MODE':		0x40,	#set when in trap. profile mode
        'HOME_IN_PROG':		0x80    #set while searching for home, cleared when home found
    }
    
    stepper_module_input_defs = {
        'ESTOP':			0x01,	#emergency stop input
        'AUX_IN1':			0x02,	#auxilliary input #1
        'AUX_IN2':			0x04,	#auxilliary input #2
        'FWD_LIMIT':		0x08,	#forward limit switch
        'REV_LIMIT':		0x10,	#reverse limit switch
        'HOME_SWITCH':		0x20	#homing limit switch
    }
    
    def __init__(self, comport="COM3"):
        self.serial = serial.Serial(comport, 19200, timeout=50/1000) # timeout? in original lib it seems to be 46ms
        self.devices = []
        
    def __del__(self):
        #print("Destructor call!")
        self.serial.flush()
        self.serial.close()
        self.devices.clear()
        
    def _read(self, nbytes):
        ret = self.serial.read(nbytes)
        if len(ret) != nbytes:
            raise IOError("Number of response bytes is shorter than expected")
        return ret
    
    def _write(self, bytes):
        self.serial.write(bytes)
        
    def _LdcnFixSioError(self):
        self._write(b'\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00\0x00')
        self.serial.flush()
        time.sleep(50/1000)
        self.serial.reset_input_buffer()
        
    def LdcnSendCmd(self, addr, cmd, data=b'', nread=0):
        checksum = (sum(data) + cmd + addr) % 256
        ndata = len(data)
        str = struct.pack("=BBB{len}sB".format(len=ndata), 0xAA, addr, cmd, data, checksum)
        self.serial.reset_input_buffer()
        self._write(str)
        
        ret = self._read(nread) # whatever return count is !?

        if nread > 0 and sum(ret[0:-1]) % 256 != ret[-1]:
            raise IOError("Checksum incorrect, maybe expected number of bytes returned is incorrect?")
        return ret 
    
    def LdcnHardReset(self):
        self._LdcnFixSioError()
        self.LdcnSendCmd(0xFF, self.cmds['HARD_RESET'])
        time.sleep(64/1000)
    
    def LdcnInit(self):
        for i in range(0x20):
            self.LdcnSendCmd(i, 0x0B) # as in C lib, servo['CLEAR_BITS'], if module is servo->Save current pos. in home pos. register (0 bytes)
        time.sleep(10/1000)
        
        self.LdcnHardReset()
        self.LdcnHardReset()
        
        addr = 1
        
        for search in range(0x20):
            try:
                self.LdcnSendCmd(0x00, self.cmds['SET_ADDR']+0x20, struct.pack("=BB", addr, 0xFF), nread=2)
            except IOError:
                break
                
            # we have to set the upper nibble of the command to the number of parameters (here 1 parameter, so +0x10)
            ret_send_id = self.LdcnSendCmd(addr, self.cmds['READ_STAT']+0x10, struct.pack("=B", self.ldcn_read_status['SEND_ID']), nread=4)
            ret_nop = self.LdcnSendCmd(addr, self.cmds['NOP'], nread=2)
            
            stat = ret_send_id[0]
            statusitems = ret_nop[0]
            
            modtype = ret_send_id[1]
            modver = ret_send_id[2]
            
            self.devices.append({'addr' : addr, 'modtype' : modtype, 'modver' : modver, 'stat' : stat, 'statusitems' : statusitems, 'groupaddr' : 0xFF, 'modspecific' : {}})
            
            if modtype == self.device_types['STEPMODTYPE']:
                self.LdcnReadStatus(addr, self.stepper_read_status['SEND_ALL'])
            
            addr += 1

        return len(self.devices)
        
        #now readdress devices
        
    def LdcnShutdown(self):
        self.LdcnHardReset()
        self._LdcnFixSioError()
        self.devices.clear()

    def LdcnGetDeviceId(self, addr):
        for devid, dev in enumerate(self.devices):
            if dev['addr'] == addr:
                return dev, devid
        raise IndexError

    def LdcnGetDevice(self, addr):
        dev, devid = self.LdcnGetDeviceId(addr)
        return dev
    
    def LdcnGetStat(self, addr):
        dev = self.LdcnGetDevice(addr)
        return dev['stat']
    
    
    def LdcnGetStatItems(self, addr):
        dev = self.LdcnGetDevice(addr)
        return dev['statusitems']
    
    def _LdcnCalcReadStatusReturn(self, addr, statusitems):
        dev = self.LdcnGetDevice(addr)
        
        nret = 2 # for regular status+checksum
        if dev['modtype'] == self.device_types['STEPMODTYPE']:
            if statusitems & self.stepper_read_status['SEND_POS']:
                nret += 4
            if statusitems & self.stepper_read_status['SEND_AD']:
                nret += 1
            if statusitems & self.stepper_read_status['SEND_ST']:
                nret += 2
            if statusitems & self.stepper_read_status['SEND_INBYTE']:
                nret += 1
            if statusitems & self.stepper_read_status['SEND_HOME']:
                nret += 4
            if statusitems & self.stepper_read_status['SEND_ID']:
                nret += 2
            if statusitems & self.stepper_read_status['SEND_OUT']:
                nret += 1
            return nret
        raise NotImplemented("Only stepper implemented")
    
    def _LdcnParseStatusBytes(self, addr, statusitems, bytes):
        dev, devid = self.LdcnGetDeviceId(addr)
        if dev['modtype'] == self.device_types['STEPMODTYPE']:
            offset = 0
            self.devices[devid]['stat'] = bytes[offset]
            offset += 1
            
            if statusitems & self.stepper_read_status['SEND_POS']:
                self.devices[devid]['modspecific'][self.stepper_read_status['SEND_POS']] = struct.unpack("=l", bytes[offset:offset+4])[0]
                offset += 4
            if statusitems & self.stepper_read_status['SEND_AD']:
                self.devices[devid]['modspecific'][self.stepper_read_status['SEND_AD']] = struct.unpack("=B", bytes[offset:offset+1])[0]
                offset += 1
            if statusitems & self.stepper_read_status['SEND_ST']:
                self.devices[devid]['modspecific'][self.stepper_read_status['SEND_ST']] = struct.unpack("=H", bytes[offset:offset+2])[0]
                offset += 2
            if statusitems & self.stepper_read_status['SEND_INBYTE']:
                self.devices[devid]['modspecific'][self.stepper_read_status['SEND_INBYTE']] = struct.unpack("=B", bytes[offset:offset+1])[0]
                offset += 1
            if statusitems & self.stepper_read_status['SEND_HOME']:
                self.devices[devid]['modspecific'][self.stepper_read_status['SEND_HOME']] = struct.unpack("=l", bytes[offset:offset+4])[0]
                offset += 4
            if statusitems & self.stepper_read_status['SEND_ID']:
                self.devices[devid]['modspecific'][self.stepper_read_status['SEND_ID']] = struct.unpack("=H", bytes[offset:offset+2])[0]
                offset += 2
            if statusitems & self.stepper_read_status['SEND_OUT']:
                self.devices[devid]['modspecific'][self.stepper_read_status['SEND_OUT']] = struct.unpack("=B", bytes[offset:offset+1])[0]
                offset += 1
            return
            
        raise NotImplemented("Only stepper implemented")
        
    def LdcnReadStatus(self, addr, statusitems = -1):
        cmd = self.cmds['READ_STAT']
        if statusitems == -1: # default
            statusitems = self.LdcnGetDevice(addr)['statusitems']
            nret = self._LdcnCalcReadStatusReturn(addr, statusitems)
            ret = self.LdcnSendCmd(addr, cmd, nread=nret) # returns status
        else: # nondefault
            nret = self._LdcnCalcReadStatusReturn(addr, statusitems)
            cmd |= 0x10 # 1 parameter
            ret = self.LdcnSendCmd(addr, cmd, struct.pack("=B", statusitems), nread=nret) # returns status
        self._LdcnParseStatusBytes(addr, statusitems, ret)
        

    def GetSteppers(self):
        addrs = []
        for dev in self.devices:
            if dev['modtype'] == self.device_types['STEPMODTYPE']:
                addrs.append(dev['addr'])
        return addrs


    def StepStopMotor(self, addr, setting):
        self.LdcnSendCmd(addr, self.stepper_cmds['STOP_MOTOR']+0x10, struct.pack("=B", setting), nread=2) # returns status

            
    def StepSetParam(self, addr, mode, minspeed, runcur, holdcur, thermlim, em_acc):
        self.LdcnSendCmd(addr, self.stepper_cmds['SET_PARAM']+0x60, struct.pack("=BBBBBB", mode, minspeed, runcur, holdcur, thermlim, em_acc), nread=2) # returns status

                     
    def StepLoadTraj(self, addr, pos=0, speed=50, acc=50, steptime=0):
        mode =  (self.stepper_load_traj['START_NOW']  | 
                 self.stepper_load_traj['LOAD_SPEED'] | 
                 self.stepper_load_traj['LOAD_ACC']   |
                 self.stepper_load_traj['LOAD_POS']   )

        self.LdcnSendCmd(addr, self.stepper_cmds['LOAD_TRAJ']+0x70, # 7 parameters
                         struct.pack("=BlBB", mode, pos, speed, acc), nread=2) # returns status

            
    def StepResetPos(self, addr):
        self.LdcnSendCmd(addr, self.stepper_cmds['RESET_POS'], nread=2) # returns status
            
if __name__ == "__main__":
    print("Implement test of pyldcn here")
    