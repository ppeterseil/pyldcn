import time
import numpy as np

from pyldcn import pyldcn

steps_per_mm = 45.66
speed_prop = 200

STEPPER1 = -1
STEPPER2 = -1

def StepperInit(ldcn):
    ldcn.LdcnInit()
    stp_addrs = ldcn.GetSteppers()
    num_modules = len(stp_addrs)
    
    if num_modules != 2:
        raise Exception("Found {num_modules} steppers, require exactly 2..".format(num_modules=num_modules))
    
    global STEPPER1
    STEPPER1 = stp_addrs[0]
    global STEPPER2
    STEPPER2 = stp_addrs[1]
    
    #for d in self.devices:
    #    if d['dtype'] == self.device_types['STEPMODTYPE']:
    ldcn.StepStopMotor(STEPPER1, ldcn.stepper_stop_motor['STP_ENABLE_AMP'] | ldcn.stepper_stop_motor['STOP_SMOOTH'])
    ldcn.StepStopMotor(STEPPER2, ldcn.stepper_stop_motor['STP_ENABLE_AMP'] | ldcn.stepper_stop_motor['STOP_SMOOTH'])
    
    return num_modules
    

def StepperDrive(ldcn, m1pos, m2pos, m1speed=800, m2speed=800, m1acc_relative=0.15, m2acc_relative=0.15, out_of_bound=False):
    ldcn.StepSetParam(STEPPER1,ldcn.stepper_set_param['SPEED_8X'],1,255,5,0,10)
    ldcn.StepSetParam(STEPPER2,ldcn.stepper_set_param['SPEED_8X'],1,255,5,0,10)
    
    if (not out_of_bound) and (m1pos > 2000 or m1pos < 0 or m2pos < -1000 or m2pos > 0):
        raise Exception("Out of bound!")
    
    ldcn.StepLoadTraj(STEPPER1, int(np.round(m1pos*steps_per_mm)),int(np.round(m1speed*(steps_per_mm/speed_prop))),int(np.round(m1acc_relative*255)),0)
    ldcn.StepLoadTraj(STEPPER2, int(np.round(m2pos*steps_per_mm)),int(np.round(m2speed*(steps_per_mm/speed_prop))),int(np.round(m2acc_relative*255)),0)

    
    meas1 = []
    meas2 = []
    
    startTime = time.time()
    wait = True
    while wait:
        time1 = time.time()
        ldcn.LdcnReadStatus(STEPPER1, ldcn.stepper_read_status['SEND_POS'])
        time1 += (time.time()-time1)/2
        
        time2 = time.time()
        ldcn.LdcnReadStatus(STEPPER2, ldcn.stepper_read_status['SEND_POS'])
        time2 += (time.time()-time2)/2
        
        pos1 = ldcn.LdcnGetDevice(STEPPER1)['modspecific'][ldcn.stepper_read_status['SEND_POS']] / steps_per_mm
        pos2 = ldcn.LdcnGetDevice(STEPPER2)['modspecific'][ldcn.stepper_read_status['SEND_POS']] / steps_per_mm
        
        meas1.append((time1-startTime, pos1))
        meas2.append((time2-startTime, pos2))
        
        #print("dt1={dt1}, Pos1={pos1}, dt2={dt2}, Pos2={pos2}".format(dt1=time1-startTime,pos1=pos1,dt2=time2-startTime,pos2=pos2))
        
        #time.sleep(200/1000)
        if (ldcn.LdcnGetStat(STEPPER1) & ldcn.stepper_status['MOTOR_MOVING']) or (ldcn.LdcnGetStat(STEPPER2) & ldcn.stepper_status['MOTOR_MOVING']):
            continue

        wait = False
    #self.StepperResetPos()
    return np.array(meas1), np.array(meas2)
    
def StepperResetPos(ldcn):
    ldcn.StepResetPos(STEPPER1)
    ldcn.StepResetPos(STEPPER2)
    time.sleep(100/1000)
    
    
def StepperShutdown(ldcn):
    ldcn.StepStopMotor(STEPPER1, ldcn.stepper_stop_motor['STOP_SMOOTH'])
    ldcn.StepStopMotor(STEPPER2, ldcn.stepper_stop_motor['STOP_SMOOTH'])
    
def StepperReference(ldcn):
    ldcn.StepSetParam(STEPPER1, ldcn.stepper_set_param['SPEED_8X'],1,255,180,0,255)
    ldcn.StepSetParam(STEPPER2, ldcn.stepper_set_param['SPEED_8X'],1,255,180,0,255)    
    
    StepperResetPos(ldcn)
    
    StepperDrive(ldcn, -3000, 3000, 8, 8, 0.01, 0.01, out_of_bound=True)
    StepperResetPos(ldcn)

    StepperDrive(ldcn, 10, -10, 8, 8, 0.01, 0.01, out_of_bound=True)
    StepperResetPos(ldcn)
    
    StepperDrive(ldcn, -11, 11, 8, 8, 0.01, 0.01, out_of_bound=True)
    StepperResetPos(ldcn)
    
    StepperDrive(ldcn, 10, -10, 8, 8, 0.01, 0.01, out_of_bound=True)
    StepperResetPos(ldcn)
    
    time.sleep(100/1000)
        
if __name__ == "__main__":
    stp = pyldcn('COM3')
    
    try:
        StepperInit(stp)

        StepperReference(stp)
        
        a, b = StepperDrive(stp, 100, -100, m1speed=50, m2speed=50)
        print("Axis 1:")
        print(a)
        print("Axis 2:")
        print(b)
        
        time.sleep(100/1000)
        StepperDrive(stp, 0, 0, m1speed=50, m2speed=50)
        time.sleep(100/1000)
    
    except: # if something crashes, try to stop motors and re-raise exception
        StepperShutdown(stp)
        raise
    
    StepperShutdown(stp)
    
